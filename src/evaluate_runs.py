# 第4周实验入口：重复执行A/B策略，保存逐步日志、每次指标、汇总图和报告。
# 建议阅读顺序：evaluate读取配置 → 配对循环 → groupby汇总 → 独立案例 → 报告。
# 本文件组织实验；真正的在线估计和运动执行位于online_state.py与decision_baseline.py。

"""运行第4周配对实验，保存真实动作日志、指标、成功/失败案例和报告。"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml

from decision_baseline import SensorTape, run_strategy
from mission_io import PROJECT_ROOT, load_mission
from observation_simulator import load_noise_config


# 实验配置记录运行次数、种子、时间步等；用项目根目录拼路径，避免依赖启动时所在文件夹。
EXPERIMENT_PATH = PROJECT_ROOT / "configs" / "experiment_week04.yaml"


# 设置中文字体候选和负号显示，仅影响图的外观，不改变实验数据。
def _setup_plot():
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False


# 把同一干扰案例下A/B的实际日志分别画到左右两个子图；不是重新规划一条理想路线。
def _case_plot(mission, logs, title, path):
    from matplotlib.patches import Rectangle
    fig, axes = plt.subplots(1, 2, figsize=(11, 5))
    # zip将两个子图与两个策略一一配对；ax表示当前正在绘制的坐标区域。
    for ax, strategy in zip(axes, ("A", "B")):
        log = logs[strategy]
        for box in mission["obstacles"]:
            ax.add_patch(Rectangle((box["xmin"], box["ymin"]), box["xmax"] - box["xmin"],
                                   box["ymax"] - box["ymin"], color="tab:red", alpha=0.3))
        ax.plot(log["x"], log["y"], label="实际轨迹")
        ax.scatter(*mission["start"], color="tab:green", label="起点")
        ax.scatter(*mission["goal"], color="tab:orange", marker="*", s=100, label="目标")
        # iloc按行位置取数据，-1表示最后一行；结束位置可能是目标、碰撞点或返航位置。
        end = log.iloc[-1]
        ax.scatter(end["x"], end["y"], color="black", marker="x", label="结束位置")
        ax.set(xlim=(0, mission["map_size"][0]), ylim=(0, mission["map_size"][1]),
               xlabel="X (m)", ylabel="Y (m)", title=f"策略{strategy}")
        if strategy == "A" and bool(log["collision"].any()):
            # 碰撞发生在起点附近时放大该区域，避免全图中看不清失败原因。
            center_x, center_y = float(end["x"]), float(end["y"])
            ax.set(xlim=(max(0, center_x - 8), min(mission["map_size"][0], center_x + 8)),
                   ylim=(max(0, center_y - 8), min(mission["map_size"][1], center_y + 8)),
                   title="策略A（碰撞区域放大）")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


# 函数默认使用正式实验配置，也允许传入另一份配置；返回汇总表、逐次表、案例表。
def evaluate(config_path=EXPERIMENT_PATH):
    config_path = Path(config_path)
    with config_path.open(encoding="utf-8") as file:
        # safe_load将YAML读成字典。后面通过experiment['runs']等键访问实验参数。
        experiment = yaml.safe_load(file)
    runs = int(experiment["runs"])
    # 这里要求至少20组配对种子；每个种子分别执行A和B，所以至少有40次策略执行。
    if runs < 20:
        raise ValueError("正式比较实验至少运行20个配对随机种子")
    # 实验配置先指定任务文件，再由任务文件提供地图、起终点、速度及障碍。
    mission = load_mission(PROJECT_ROOT / experiment["mission_path"])
    noise = load_noise_config(PROJECT_ROOT / experiment["noise_path"])
    with (PROJECT_ROOT / experiment["thresholds_path"]).open(encoding="utf-8") as file:
        thresholds = yaml.safe_load(file)
    result_dir, figures, reports = (PROJECT_ROOT / "data" / "results", PROJECT_ROOT / "figures",
                                    PROJECT_ROOT / "reports")
    for directory in (result_dir, figures, reports):
        # parents=True同时创建缺失的上级文件夹；exist_ok=True允许输出目录已存在。
        directory.mkdir(parents=True, exist_ok=True)
    # logs保存各次逐步DataFrame，metrics保存每次结果字典；两者统计粒度不同。
    logs, metrics = [], []
    for run_id in range(runs):
        # 运行编号从0开始递增，对应一组确定的随机干扰；相同配置和种子便于复现。
        seed = int(experiment["seed_start"]) + run_id
        # 每个种子只生成一份按时间索引的干扰表，再交给A/B共享；避免两个策略抽到不同难度的噪声。
        tape = SensorTape.create(noise, experiment["max_steps"], seed)
        for strategy in ("A", "B"):
            # run_strategy返回逐步日志与本次指标；动作实际改变运动，结果可能成功、碰撞、返航或超时。
            log, values = run_strategy(mission, noise, thresholds, experiment, tape, strategy, run_id)
            values.update(seed=seed, scenario="random")
            # assign返回添加了种子和场景列的新表；这些标签便于之后筛选某一次运行。
            logs.append(log.assign(seed=seed, scenario="random"))
            metrics.append(values)
    # concat纵向合并所有逐步表；ignore_index重新连续编号。per_run则一行对应一次完整执行。
    all_logs, per_run = pd.concat(logs, ignore_index=True), pd.DataFrame(metrics)
    all_logs.to_csv(result_dir / "week04_decision_log_czl.csv", index=False, encoding="utf-8-sig")
    per_run.to_csv(result_dir / "week04_runs_czl.csv", index=False, encoding="utf-8-sig")
    # 按策略分组后聚合。count数次数，sum累计，mean求平均；布尔成功标记的mean就是成功率。
    # 例如20次中15次True，成功率为15/20=0.75。这里以每次执行为单位，不按日志行数加权。
    summary = per_run.groupby("strategy").agg(
        run_count=("run_id", "count"), success_rate=("task_completed", "mean"),
        collision_count=("collision_count", "sum"), path_length=("path_length", "mean"),
        execution_time=("execution_time", "mean"), hold_time=("hold_time", "mean"),
        hold_count=("hold_count", "mean"), replan_count=("replan_count", "mean"),
        runtime_seconds=("runtime_seconds", "mean"), return_count=("returned_to_start", "sum"),
    ).reset_index()
    # index=False不另存DataFrame行号；utf-8-sig便于常见Windows表格软件识别中文。
    summary.to_csv(result_dir / "week04_metrics_czl.csv", index=False, encoding="utf-8-sig")
    _setup_plot()
    fig, axes = plt.subplots(1, 3, figsize=(11, 4))
    for ax, column, title in zip(axes, ("success_rate", "execution_time", "hold_time"),
                                ("任务成功率", "平均执行时间 (s)", "平均停留时间 (s)")):
        # 每个子图用同一汇总表画一个指标。较短执行时间也可能由碰撞提前终止造成，需结合成功率解释。
        ax.bar(summary["strategy"], summary[column], color=["tab:blue", "tab:orange"])
        ax.set(title=title, xlabel="策略")
    axes[0].set_ylim(0, 1)
    fig.suptitle(f"第4周：{runs}个配对种子的规则基线比较")
    fig.tight_layout()
    fig.savefig(figures / "week04_strategy_comparison_czl.png", dpi=160)
    plt.close(fig)

    # 案例单独报告，不能混入随机20次实验的成功率。
    # 案例单独保存，避免把专门设计的成功案例混入随机实验，改变正式成功率的分母。
    case_metrics, case_results = [], {}
    for index, (name, settings) in enumerate(experiment.get("cases", {}).items()):
        # 复制默认噪声参数，再用当前案例settings中的同名值覆盖；原noise保持原样。
        case_noise = dict(noise, **settings)
        tape = SensorTape.create(case_noise, experiment["max_steps"], 900 + index)
        case_logs, values_by_strategy = {}, {}
        for strategy in ("A", "B"):
            log, values = run_strategy(mission, case_noise, thresholds, experiment, tape, strategy, index)
            log.assign(scenario=name).to_csv(result_dir / f"week04_case_{name}_{strategy}.csv",
                                            index=False, encoding="utf-8-sig")
            values.update(scenario=name, seed=900 + index)
            case_metrics.append(values)
            case_logs[strategy], values_by_strategy[strategy] = log, values
        case_results[name] = values_by_strategy
        # 文件名按案例名称选后缀，并非自动判定成功；实际是否完成须查看task_completed等指标。
        suffix = "success" if name == "transient_bias" else "failure"
        _case_plot(mission, case_logs, f"干扰案例：{name}（与随机实验分开）",
                   figures / f"week04_{suffix}_case_czl.png")
    cases = pd.DataFrame(case_metrics)
    cases.to_csv(result_dir / "week04_case_metrics_czl.csv", index=False, encoding="utf-8-sig")

    # 内部辅助函数，把选定数据列转为Markdown表格文本，嵌入阶段报告；不参与控制决策。
    def table(frame, fields):
        header = "| " + " | ".join(fields) + " |\n|" + "---|" * len(fields)
        body = []
        for _, row in frame.iterrows():
            body.append("| " + " | ".join(f"{row[key]:.4f}" if isinstance(row[key], float)
                                          else str(row[key]) for key in fields) + " |")
        return header + "\n" + "\n".join(body)

    # 逐层get在字段缺失时提供默认值；检查短时异常案例是否确实出现B成功且A失败。
    case_ok = (case_results.get("transient_bias", {}).get("B", {}).get("task_completed", False)
               and not case_results.get("transient_bias", {}).get("A", {}).get("task_completed", True))
    # 以下是多行格式化字符串，花括号中的表达式会替换成实测值；字符串里的#是报告标题。
    report = f"""# 第4周阶段报告：在线状态条件决策

日期：2026-10-14。版本：baseline_v1 本地候选。

## 当前要求核对

第2周：任务配置、横竖直角运动、障碍安全距离、CSV和地图图已具备，并通过修改地图与目标的测试。
第3周：使用当前横竖Ground Truth重新生成观测、估计和可靠性输出，并完成噪声/缺失参数对照。
第4周：在线决策、真实动作执行、{runs}次配对实验及独立成功/失败案例已完成。

## 输入与方法

输入为 `{experiment['mission_path']}`、`{experiment['noise_path']}`、`{experiment['thresholds_path']}`，实验配置为 `configs/experiment_week04.yaml`。
位置单位m，时间s，速度m/s；这是本候选版本的统一约定。
策略A按路线控制，无论可靠性高低均接受有效观测并前进；策略B使用更新前的观测预测残差和连续缺失次数评分。
在线估计初始化使用配置起点，以已执行指令预测；评分和动作不读取真实位置、真值误差或人为异常标签。
真实位置仅用于产生模拟测量、环境碰撞检测和最后评价。

继续、减速、主动观测、悬停和返航实际改变位置或等待时间；返航从在线估计生成到起点的路径。
没有实现REPLAN动作，因此重新规划次数为0；没有仅输出重新规划标签来冒充路径生成。
这里的固定路线仍依赖估计位置判断转弯，噪声可能使其过早拐弯或误判到达；不是永不受传感器影响的理想开环点模型。

## 公平对照与结果

A/B每次使用同一地图、参数、随机种子和按仿真时刻固定的噪声、缺失及偏移序列；策略B主动观测用预生成的第二份同条件样本。
共{runs}个配对种子、{runs * 2}次策略执行。种子为{experiment['seed_start']}至{experiment['seed_start'] + runs - 1}。
运行时间列来自实际计时，重复运行会有小幅变化。

{table(summary, ['strategy', 'run_count', 'success_rate', 'collision_count', 'path_length', 'execution_time', 'hold_time', 'hold_count', 'replan_count', 'runtime_seconds'])}

均值包含成功、返航和失败运行。碰撞后终止，因此较短执行时间或路径不一定表示策略更好。原地图较开阔，随机实验不能单独证明复杂地图下的安全优势。
任务成功要求真实结束位置到目标不超过{experiment['goal_tolerance']}m；仅由估计位置判断到达仍可能出现`false_arrival`。
停留时间包含HOLD及OBSERVE，每个停留步增加dt；停留次数按连续停留段计数。
随机实验失败原因计数：{per_run.loc[~per_run['task_completed']].groupby(['strategy', 'failure_reason']).size().to_dict()}。

## 成功和失败案例

案例是否得到A失败/B成功：{case_ok}。案例与随机20次的结果单独保存，不能合并提高成功率。

{table(cases, ['scenario', 'strategy', 'task_completed', 'collision_count', 'hold_time', 'returned_to_start', 'failure_reason'])}

短时偏移案例：第5至8秒给Y测量加入70m偏移，故意模拟严重定位跳变。A可能错误地判断纵向段完成，提前横向转入起点附近障碍；B根据大预测残差拒绝异常，停止并等到观测恢复。
持续偏移案例：同样的跳变持续200步。B持续无法恢复时执行真实返航，任务未完成；安全返回仍记为任务失败。此例说明保守规则也不能保证任务成功。
极端70m偏移是人为压力测试，不能作为普通GPS噪声的典型强度。

## 输出与复现

- `data/results/week04_decision_log_czl.csv`：逐时刻状态、观测、可靠性、实际速度和动作；
- `data/results/week04_runs_czl.csv`：每次完整指标及失败原因；
- `data/results/week04_metrics_czl.csv`：策略汇总；
- `data/results/week04_case_metrics_czl.csv` 和各案例日志；
- `figures/week04_strategy_comparison_czl.png`、`week04_success_case_czl.png`、`week04_failure_case_czl.png`。

```powershell
python src/run_pipeline.py --stage all
python -m unittest discover -s tests -v
```

## 局限与下一步

当前是已知地图、理想指令执行、横竖二维点模型；不包含风扰、动态障碍、加速度或真实飞行系统。
在线评分会漏掉缓慢漂移，不是校准后的概率。OBSERVE的额外采样是简化模型，返航也可能因为已有估计偏差失败。
下一步最值得补充运动模型误差与缓慢偏移实验，再评估多种阈值和碰撞安全边界，不预设策略B一定更优。

## 第三次组会与共同版本状态

已整理统一运行入口、配置、CSV、单位、来源说明和可复现本地压缩包。
目前只有本人的项目；另外两位同学接口整合、三台电脑独立复现、1—3分钟演示视频、本人逐段解释、钉钉上传及教师审核尚待完成。
该包标注为baseline_v1本地候选，不能声称已经是经全组复现的正式共同版本。未替用户提交钉钉或推送Git。
正式下一周安排仍以会后经教师审核的任务单为准；本次按用户授权先完成可运行候选和缺失基础模块。

## 辅助工具使用

由Codex辅助核对任务、编写和验证代码、生成实验记录及此报告。所有数值来自本机实际执行。
个人理解、现场独立操作及口头说明需由本人完成，不能由自动测试代替。
"""
    (reports / "week04_stage_report_czl.md").write_text(report, encoding="utf-8")
    print(summary.to_string(index=False))
    print("案例：\n", cases[["scenario", "strategy", "task_completed", "failure_reason"]].to_string(index=False))
    # 依次返回策略汇总、每次执行结果、独立案例结果，供其他脚本继续分析。
    return summary, per_run, cases


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="第4周配对基线实验")
    # 命令行可用--config指定实验配置；省略时使用默认路径，type=Path自动转为路径对象。
    parser.add_argument("--config", type=Path, default=EXPERIMENT_PATH)
    args = parser.parse_args()
    # Agg适合只保存图片的批量运行，不要求弹出绘图窗口。
    plt.switch_backend("Agg")
    evaluate(args.config)
