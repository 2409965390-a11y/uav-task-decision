# 多地图扩展入口：依次读取场景配置，复用已有规划与A/B策略，保存总览、时间轴、统计和动图。
# 建议阅读顺序：run_suite的场景循环 → draw_map → verify_trajectory → draw_decisions → save_motion_gif。
# 地图总览与GIF展示无噪声理想轨迹；决策时间轴展示带噪闭环实际轨迹，两类数据含义不同。

"""不改变原任务的多地图教学与实验扩展：真实轨迹、动作时间轴和动图。"""

import argparse
from pathlib import Path

import matplotlib
# 本脚本只保存静态图和动图，因此在导入pyplot前选择后台绘图模式。
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Rectangle
import numpy as np
import pandas as pd
import yaml

from decision_baseline import SensorTape, run_strategy
from mission_io import PROJECT_ROOT, load_mission
from observation_simulator import load_noise_config
from simulator_2d import check_collision, simulate_motion


# 固定的结果目录标签，不是自动读取的当前日期；重新运行会写入同名结果文件。
DATE = "2026-10-15"
ACTION_NAMES = {"CONTINUE": "继续", "SLOW_DOWN": "减速", "OBSERVE": "观测",
                "HOLD": "悬停", "RETURN": "返航", "END": "结束"}
# 把动作名称映射为纵轴整数刻度。数字只方便画图，不表示动作的优劣或强弱。
ACTION_CODES = {name: index for index, name in enumerate(ACTION_NAMES)}


# 只在传入坐标区域画地图背景、障碍、安全边界和起终点；路线由调用方另画。
def draw_map(ax, mission, title):
    width, height = mission["map_size"]
    # 安全边界相当于在障碍四周各扩张margin；总宽高各增加2*margin。
    margin = mission["safety_distance"]
    for index, box in enumerate(mission["obstacles"]):
        xmin, xmax, ymin, ymax = [box[key] for key in ("xmin", "xmax", "ymin", "ymax")]
        ax.add_patch(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin,
                               facecolor="tab:red", alpha=0.25, label="障碍物" if index == 0 else None))
        ax.add_patch(Rectangle((xmin - margin, ymin - margin), xmax - xmin + 2 * margin,
                               ymax - ymin + 2 * margin, fill=False, edgecolor="tab:red",
                               linestyle="--", linewidth=1,
                               label="安全边界" if index == 0 else None))
        ax.text((xmin + xmax) / 2, (ymin + ymax) / 2, str(index + 1), ha="center", va="center")
    ax.scatter(*mission["start"], color="tab:green", s=45, label="起点", zorder=5)
    ax.scatter(*mission["goal"], color="tab:orange", marker="*", s=90, label="终点", zorder=5)
    ax.set(xlim=(0, width), ylim=(0, height), xlabel="X (m)", ylabel="Y (m)", title=title)
    # X/Y每米使用相同显示尺度，避免矩形与路线被视觉拉伸。
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.2)


# 验证理想轨迹每段只沿一个轴移动，并检查整段运动与安全区域，而非只看图是否绕开障碍。
def verify_trajectory(mission, trajectory):
    points = trajectory[["x", "y"]].to_numpy()
    # 相邻两行相减，得到每一步的[dx,dy]；N个位置点对应N-1段位移。
    changes = np.diff(points, axis=0)
    # 每段至少一个轴的位移必须为0；|对数组逐元素做或判断，all要求所有段都满足。
    if not ((changes[:, 0] == 0) | (changes[:, 1] == 0)).all():
        raise AssertionError("真实轨迹包含斜向运动")
    # 原数组与去掉第一行的数组配对，依次取得每段起点和终点。
    for start, end in zip(points, points[1:]):
        if check_collision(start.tolist(), end.tolist(), mission):
            raise AssertionError("规划轨迹违反障碍安全区域")


# 按已有真实轨迹表逐帧播放；它不重新计算控制指令，也不是A/B闭环结果。
def save_motion_gif(scene, mission, trajectory, destination):
    # 只播放真实计算出的CSV记录，不生成另一套看起来更漂亮的路线。
    fig, ax = plt.subplots(figsize=(8, 5))
    draw_map(ax, mission, scene["title"])
    ax.plot(trajectory["x"], trajectory["y"], "--", color="gray", alpha=0.6, label="完整规划路线")
    # plot返回线对象列表；逗号拆包取出唯一对象，后面逐帧更新它的数据。
    line, = ax.plot([], [], color="blue", linewidth=2, label="已执行轨迹")
    point, = ax.plot([], [], "o", color="blue", markersize=6)
    status = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top",
                     bbox={"facecolor": "white", "alpha": 0.9, "edgecolor": "none"})
    ax.legend(loc="lower right", fontsize=8)
    frames = list(range(len(trajectory)))

    # 动画回调：index是当前轨迹行号，每次更新已走路线、当前点和文字。
    def update(index):
        row = trajectory.iloc[index]
        # 切片右端不包含，所以用index+1把当前点也画进已执行轨迹。
        line.set_data(trajectory["x"].iloc[:index + 1], trajectory["y"].iloc[:index + 1])
        point.set_data([row["x"]], [row["y"]])
        status.set_text(f"仿真时间 {row['time']:.0f}s  进度 {row['task_progress']:.0%}")
        return line, point, status

    fig.tight_layout()
    animation = FuncAnimation(fig, update, frames=frames, interval=100, blit=False, repeat=False)
    # 每秒播放10帧是展示速度；仿真时刻仍来自CSV的time列，不等于实际播放秒数。
    animation.save(destination, writer=PillowWriter(fps=10), dpi=100)
    plt.close(fig)


# 上方比较A/B实际路线，下方按同一时间轴对齐在线分数、动作和真实平均速度。
def draw_decisions(scene, mission, logs, seed, destination):
    fig = plt.figure(figsize=(12, 10))
    grid = fig.add_gridspec(4, 2, height_ratios=[1.6, 0.8, 0.9, 0.7])
    for column, strategy in enumerate(("A", "B")):
        log = logs[strategy]
        ax = fig.add_subplot(grid[0, column])
        outcome = "完成" if bool(log.iloc[-1]["task_completed"]) else "碰撞" if bool(log["collision"].any()) else "未完成"
        draw_map(ax, mission, f"策略{strategy}：{outcome}")
        ax.plot(log["x"], log["y"], color="tab:blue", linewidth=2, label="实际轨迹")
        ax.scatter(log.iloc[-1]["x"], log.iloc[-1]["y"], marker="x", color="black", label="结束位置")
        ax.legend(fontsize=8, loc="best")
    score_ax = fig.add_subplot(grid[1, :])
    action_ax = fig.add_subplot(grid[2, :], sharex=score_ax)
    speed_ax = fig.add_subplot(grid[3, :], sharex=score_ax)
    for strategy, color, style in (("A", "tab:blue", "-"), ("B", "tab:orange", "--")):
        log = logs[strategy]
        # 分数曲线排除终止记录，避免把没有新动作的最后一行当作一次新的决策。
        active = log[log["action"] != "END"]
        score_ax.plot(active["time"], active["reliability_score"], label=f"{strategy}在线分数", color=color,
                      linestyle=style, alpha=0.8)
        # map将字符串动作换成刻度编号；post阶梯让本行动作显示在当前时刻到下一时刻之间。
        action_ax.step(log["time"], log["action"].map(ACTION_CODES), where="post", color=color,
                       linestyle=style, label=f"策略{strategy}")
        # 沿每一行计算sqrt(vx²+vy²)，得到非负速度大小；这样向左或返航也显示正的速率。
        speed_ax.step(log["time"], np.linalg.norm(log[["vx", "vy"]], axis=1), where="post",
                       color=color, linestyle=style, label=f"策略{strategy}")
    score_ax.axhline(0.5, color="gray", linestyle=":", linewidth=1, label="观测/停留区间上界")
    score_ax.set(ylabel="可靠性", ylim=(-0.05, 1.05))
    score_ax.legend(fontsize=8, loc="upper right", ncol=3)
    action_ax.set(ylabel="动作", yticks=list(ACTION_CODES.values()),
                  yticklabels=list(ACTION_NAMES.values()))
    action_ax.legend(fontsize=8, loc="upper right")
    speed_ax.set(xlabel="仿真时间 (s)", ylabel="速度 (m/s)", ylim=(-0.05, mission["speed"] * 1.1))
    for ax in (score_ax, action_ax):
        ax.tick_params(labelbottom=False)
    for ax in (score_ax, action_ax, speed_ax):
        ax.grid(True, alpha=0.2)
    fig.suptitle(f"{scene['title']}：同图同干扰，固定示例种子{seed}")
    fig.tight_layout()
    fig.savefig(destination, dpi=140)
    plt.close(fig)


# 主流程读取场景列表；make_gif=False只跳过GIF，地图图、实验与其他输出仍会生成。
def run_suite(make_gif=True):
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    suite = yaml.safe_load((PROJECT_ROOT / "configs/scenario_suite.yaml").read_text(encoding="utf-8"))
    experiment = yaml.safe_load((PROJECT_ROOT / "configs/experiment_week04.yaml").read_text(encoding="utf-8"))
    thresholds = yaml.safe_load((PROJECT_ROOT / "configs/decision_thresholds.yaml").read_text(encoding="utf-8"))
    noise = load_noise_config()
    # 每张可达地图至少使用20组配对种子；不可达地图只验证规划失败，不纳入执行成功率。
    if int(suite["runs"]) < 20:
        raise ValueError("地图对照实验至少使用20组配对种子")
    result_dir = PROJECT_ROOT / "data/results" / f"scenario_suite_{DATE}"
    figure_dir = PROJECT_ROOT / "figures" / f"scenario_suite_{DATE}"
    report_dir = PROJECT_ROOT / "reports"
    for directory in (result_dir, figure_dir, report_dir):
        directory.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(2, 2, figsize=(11, 9))
    outcomes, all_metrics = [], []
    # 四个场景依次对应四宫格；场景来自配置文件，不是在这里为每张地图另写算法。
    # zip会在较短一侧结束，因此当前总览设计对应四张图；以后增加场景还需同步扩展子图数量。
    for scene, ax in zip(suite["scenes"], axes.flat):
        # 读取当前场景自己的地图配置，包括地图尺寸、起终点和障碍列表。
        mission = load_mission(PROJECT_ROOT / scene["path"])
        draw_map(ax, mission, scene["title"])
        try:
            trajectory = simulate_motion(mission)
        # 规划失败时先检查场景是否本来就预期不可达；意外失败会重新抛出，而非假装正常完成。
        except ValueError as exc:
            if scene["expected_plannable"]:
                raise
            outcomes.append({"scene": scene["id"], "plannable": False, "obstacle_count": len(mission["obstacles"]),
                             "failure_reason": str(exc)})
            ax.text(0.5, 0.5, "安全区域堵住通道\n未生成可执行路线", transform=ax.transAxes,
                    ha="center", va="center", bbox={"facecolor": "white", "alpha": 0.9, "edgecolor": "none"})
            continue
        # 反过来，预期不可达却生成路线也要报错，防止安全距离或碰撞逻辑失效却被忽略。
        if not scene["expected_plannable"]:
            raise AssertionError("预期不可达地图竟生成了路线")
        verify_trajectory(mission, trajectory)
        points = trajectory[["x", "y"]].to_numpy()
        # 先求每段二维位移，再算每段长度，最后累加成理想规划路径长度。
        length = np.linalg.norm(np.diff(points, axis=0), axis=1).sum()
        outcomes.append({"scene": scene["id"], "plannable": True, "obstacle_count": len(mission["obstacles"]),
                         "sample_count": len(trajectory), "path_length": length,
                         "turn_count": max(0, len(trajectory.attrs["planned_route"]) - 2)})
        trajectory.to_csv(result_dir / f"{scene['id']}_ground_truth.csv", index=False, encoding="utf-8-sig")
        ax.plot(trajectory["x"], trajectory["y"], color="blue", linewidth=2, label="规划执行轨迹")
        ax.legend(fontsize=7, loc="best")
        all_logs = []
        for run_id in range(int(suite["runs"])):
            seed = int(suite["seed_start"]) + run_id
            # 同张地图的A/B共享该种子下按时刻生成的干扰；不同策略仍可能走出不同真实位置。
            tape = SensorTape.create(noise, experiment["max_steps"], seed)
            for strategy in ("A", "B"):
                log, metrics = run_strategy(mission, noise, thresholds, experiment, tape, strategy, run_id)
                # 每次完整执行指标附上地图与种子，供最后按地图、策略分组统计。
                all_metrics.append(dict(metrics, scene=scene["id"], seed=seed))
                all_logs.append(log.assign(scene=scene["id"], seed=seed))
        pd.concat(all_logs, ignore_index=True).to_csv(result_dir / f"{scene['id']}_decision_logs.csv",
                                                      index=False, encoding="utf-8-sig")
        # 只为交错四障碍图额外生成固定种子的决策示例；示例不会替换或筛选前面的随机实验。
        if scene["id"] == "staggered_four":
            example_seed = int(suite["example_seed"])
            tape = SensorTape.create(noise, experiment["max_steps"], example_seed)
            # 字典推导式分别运行A/B；[0]只取返回值中的逐步日志，用于时间轴。
            examples = {strategy: run_strategy(mission, noise, thresholds, experiment, tape, strategy)[0]
                        for strategy in ("A", "B")}
            draw_decisions(scene, mission, examples, example_seed, figure_dir / "decisions_and_reliability.png")
            if make_gif:
                save_motion_gif(scene, mission, trajectory, figure_dir / "staggered_motion.gif")
    fig.suptitle("地图难度分层：基础、交错障碍、窄通道与不可达\n数字标明障碍编号，虚线表示安全边界")
    fig.tight_layout()
    fig.savefig(figure_dir / "map_overview.png", dpi=150)
    plt.close(fig)
    pd.DataFrame(outcomes).to_csv(result_dir / "planning_outcomes.csv", index=False, encoding="utf-8-sig")
    per_run = pd.DataFrame(all_metrics)
    per_run.to_csv(result_dir / "runs.csv", index=False, encoding="utf-8-sig")
    # 按地图和策略同时分组，不把难度不同的地图直接混为一个成功率。
    summary = per_run.groupby(["scene", "strategy"]).agg(
        runs=("run_id", "count"), success_rate=("task_completed", "mean"),
        collisions=("collision_count", "sum"), path_length=("path_length", "mean"),
        execution_time=("execution_time", "mean"), hold_time=("hold_time", "mean"),
        return_count=("returned_to_start", "sum"), safety_violation_steps=("safety_violation_steps", "sum"),
    ).reset_index()
    summary.to_csv(result_dir / "summary.csv", index=False, encoding="utf-8-sig")
    lines = ["| 地图 | 策略 | 次数 | 成功率 | 碰撞次数 | 平均停留(s) |", "|---|---|---|---|---|---|"]
    for _, row in summary.iterrows():
        lines.append(f"| {row['scene']} | {row['strategy']} | {row['runs']} | {row['success_rate']:.0%} | "
                     f"{row['collisions']} | {row['hold_time']:.2f} |")
    # 下面是用于输出说明报告的多行字符串，里面的标题和代码片段不是本脚本正在执行的语句。
    report = f"""# 多障碍教学与展示扩展

日期：{DATE}。沿用原模拟器和决策算法，新增独立配置和演示脚本，原单障碍任务和历史结果保持原样。

## 为什么增加这些地图

老师未规定障碍数量；原单障碍满足基础示例，但现有L形路线能直接绕开它，不能充分展示复杂绕行。
单障碍用于解释基础；四道交错障碍迫使路线多次直角转弯；四块障碍形成2m宽通道，验证安全距离；同一通道将安全距离从0.75m增为1.05m后应不可达。
增加障碍本身不等于算法更好，应该展示不同条件下的实际行为、指标和失败原因。

## 实际规划验证

{pd.DataFrame(outcomes).to_string(index=False)}

可达图的真实CSV逐段验证为水平或竖直，并保持在安全区域之外。不可达图没有生成伪成功轨迹，也不混入可达任务的成功率分母。

## 随机配对结果

每张可达图20组相同种子的A/B比较，共3张图、120次策略执行。所有结果保留，没有为了演示挑选最优种子。
每张图A/B共享噪声参数和对应的按时间干扰序列；跨地图的路线长度不同，不能只比较任务时长。

{chr(10).join(lines)}

多障碍下B可能同样失败。现有控制器依赖估计判断拐点和终点，错误估计可导致偏离规划安全边界、碰撞、误判到达或返航；新增图不是给B制造必胜条件。
当前A/B同时存在异常观测处理和动作策略差异，比较的是整个方案，不能把差异全部归因于悬停动作。

## 图与动图怎样解读

map_overview.png：四种地图与理想执行轨迹。
decisions_and_reliability.png：交错地图固定示例种子100的实际轨迹、在线分数、动作与真实速度时间轴；图上标明完成/碰撞/未完成，接近终点不等于成功。
staggered_motion.gif：播放交错地图的理想Ground Truth记录；不是带噪A/B对照，不能据此声称B成功。
动图10帧/秒用于加速播放，每帧对应一个仿真时间步，显示时间来自CSV。
所有图使用已有算法，没有新增动态障碍或修改评分规则。

## 运行与学习

```powershell
python src/show_scenarios.py
```

不生成动图时可加`--no-gif`。输出位于data/results/scenario_suite_{DATE}和figures/scenario_suite_{DATE}。
学习时先读configs/scenarios中的矩形列表，再在show_scenarios中看load_mission→simulate_motion→绘图，最后对照choose_action与实际速度。
下一步优先研究拐点及终点的到达判定和安全裕度，继续保留B的失败，不以增加障碍数量代替算法验证。
安全区域违反统计是持续时间步数，不是物理碰撞次数；悬停和观测期间位于安全区域内也计数，跨策略总时长不同需结合时间解释。
"""
    (report_dir / f"scenario_suite_{DATE}.md").write_text(report, encoding="utf-8")
    print(pd.DataFrame(outcomes).to_string(index=False))
    print(summary.to_string(index=False))
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="多地图教学与同条件策略展示")
    # 命令中加入--no-gif即可跳过较耗时的动图保存；其他实验照常运行。
    parser.add_argument("--no-gif", action="store_true", help="仅生成静态图和指标")
    args = parser.parse_args()
    # no_gif为True时取反成False，传给make_gif参数，实现命令行开关。
    run_suite(make_gif=not args.no_gif)
