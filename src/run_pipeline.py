# 第2—4周统一入口：按依赖顺序调用模块，而不是把所有算法写进同一个文件。
# 第2周生成真值 → 第3周离线观测/估计/评价；第4周另外运行在线闭环A/B实验。
# 初次运行建议选择all；单独运行week03前，需已有第2周生成的CSV。

"""统一入口：任务 → 横竖运动 → 带噪观测 → 状态估计 → 在线策略比较。"""

import argparse
import json
from pathlib import Path
import warnings


# 第2周流程：读取任务 → 规划并采样真实轨迹 → 保存CSV → 画地图轨迹。
def run_week02():
    from mission_io import PROJECT_ROOT, load_mission
    # 在函数内导入所需功能；本函数中的plot_trajectory来自二维模拟器。
    from simulator_2d import simulate_motion, plot_trajectory
    data_dir, figure_dir = PROJECT_ROOT / "data" / "simulated_observations", PROJECT_ROOT / "figures"
    data_dir.mkdir(parents=True, exist_ok=True)
    figure_dir.mkdir(parents=True, exist_ok=True)
    # 使用默认任务配置。mission描述已知任务，不能当作每一时刻的传感器观测。
    mission = load_mission()
    # 同一模拟函数读取不同任务就能生成不同路线；返回含time/x/y/vx/vy等列的表。
    trajectory = simulate_motion(mission)
    trajectory.to_csv(data_dir / "week02_ground_truth_czl.csv", index=False, encoding="utf-8-sig")
    plot_trajectory(mission, trajectory, figure_dir / "week02_ground_truth_czl.png")
    print(f"第2周完成：{len(trajectory)}个轨迹点，只有水平/竖直运动。")
    # 保存文件之外也返回内存中的轨迹表，方便交互调用时继续检查。
    return trajectory


# 第3周是离线分析：读取已完成的真值轨迹，再模拟带噪观测、估计位置并计算误差和可靠性。
def run_week03():
    import matplotlib.pyplot as plt
    import pandas as pd
    from mission_io import PROJECT_ROOT, load_mission
    from observation_simulator import load_noise_config, simulate_observations
    from state_estimator import (estimate_state, calculate_reliability, calculate_rmse,
                                  plot_trajectory, plot_reliability)
    data_dir = PROJECT_ROOT / "data" / "simulated_observations"
    result_dir, report_dir = PROJECT_ROOT / "data" / "results", PROJECT_ROOT / "reports"
    result_dir.mkdir(parents=True, exist_ok=True)
    report_dir.mkdir(parents=True, exist_ok=True)
    # 依赖第2周输出；只运行week03时不会自动补跑week02，缺文件需要先生成。
    ground_truth = pd.read_csv(data_dir / "week02_ground_truth_czl.csv")
    # 使用默认任务配置。mission描述已知任务，不能当作每一时刻的传感器观测。
    mission = load_mission()
    config = load_noise_config()
    # 仿真传感器给真实轨迹加随机噪声、缺失和异常，返回观测表；真值轨迹本身不被改写。
    observations = simulate_observations(ground_truth, config)
    # 指数平滑以0.8权重融合有效观测；旧离线模块缺失时保持上一估计，没有第4周的控制位移预测。
    estimates = estimate_state(observations, alpha=0.8)
    # 这里的旧评分使用仿真真值误差，只能用于离线分析；第4周在线评分由online_state.py单独实现。
    reliability = calculate_reliability(estimates)
    observations.to_csv(data_dir / "week03_noisy_czl.csv", index=False, encoding="utf-8-sig")
    estimates[["time", "estimated_x", "estimated_y", "position_error", "estimate_error"]].to_csv(
        data_dir / "week03_estimated_czl.csv", index=False, encoding="utf-8-sig")
    reliability.to_csv(data_dir / "week03_reliability_czl.csv", index=False, encoding="utf-8-sig")
    # 此处调用的是state_estimator导入的观测/估计对比绘图函数，与run_week02中的同名函数不同。
    plot_trajectory(estimates)
    plot_reliability(reliability)
    # 释放当前已打开的图对象，避免批量运行时不断占用内存。
    plt.close("all")
    rows = []
    for name, std, rate in (("baseline", config["position_noise_std"], config["missing_rate"]),
                             ("high_noise", 2 * config["position_noise_std"], config["missing_rate"]),
                             ("high_missing", config["position_noise_std"], 0.25)):
        # 为每种对照条件复制配置，只覆盖噪声标准差和缺失率，便于比较参数变化。
        settings = dict(config, position_noise_std=std, missing_rate=rate)
        # 每种条件重新模拟观测；保持相同真值与配置种子，但噪声尺度或有效样本集合会变化。
        sampled = simulate_observations(ground_truth, settings)
        estimated = estimate_state(sampled, alpha=0.8)
        scores = calculate_reliability(estimated)
        rows.append({"case": name, "position_noise_std": std, "missing_rate": rate,
                     "sample_count": len(sampled), "missing_count": int(sampled["is_missing"].sum()),
                     "observation_rmse": float(calculate_rmse(estimated["position_error"])),
                     "estimate_rmse": float(calculate_rmse(estimated["estimate_error"])),
                     "low_reliability_count": int(scores["is_low_reliability"].sum())})
    # 一行代表一种参数条件，汇总缺失数、观测/估计RMSE与低可靠次数。RMSE越小通常表示误差越小。
    comparisons = pd.DataFrame(rows)
    comparisons.to_csv(result_dir / "week03_parameter_comparison_czl.csv", index=False, encoding="utf-8-sig")
    # 列表第一项是基准配置，供报告取值；它与第4周策略A/B不是同一层面的分类。
    baseline = rows[0]
    table = "| 条件 | 噪声标准差 | 缺失率 | 缺失数 | 观测RMSE (m) | 估计RMSE (m) | 低可靠数 |\n|---|---|---|---|---|---|---|\n"
    for row in rows:
        table += (f"| {row['case']} | {row['position_noise_std']} | {row['missing_rate']} | {row['missing_count']} | "
                  f"{row['observation_rmse']:.6f} | {row['estimate_rmse']:.6f} | {row['low_reliability_count']} |\n")
    # 以下字符串用于生成报告。数值插入报告，不会反过来影响估计或可靠性计算。
    report = f"""# 第3周复核报告：横竖路线的观测、估计与可靠性

日期：2026-10-14；版本：baseline_v1 本地候选。历史直线轨迹实验与旧报告已单独归档。

## 输入与处理

使用当前第2周的{len(ground_truth)}行横竖Ground Truth，起点{mission['start']}、终点{mission['goal']}。
配置由`configs/observation_noise.yaml`读取：{json.dumps(config, ensure_ascii=False)}。
位置m、时间s、速度m/s，是本候选约定。第50至59个样本加入[+5σ,-5σ]偏移；缺失用NaN表示。
指数平滑alpha=0.8；缺失时保持上一估计，首条缺失时旧离线模块使用配置轨迹起点的真值初始化。

## 实测结果与参数对照

{table}

所有条件使用同一Ground Truth与随机种子。观测RMSE只评价有效观测；估计RMSE覆盖所有时刻，包括缺失。
基准估计RMSE为{baseline['estimate_rmse']:.6f}m，观测RMSE为{baseline['observation_rmse']:.6f}m。
当前基准下指数平滑没有改善总体精度，主要受到运动滞后、缺失保持和异常偏移影响；不能将曲线变平滑等同于精度提高。
缺失增加时观测RMSE可能下降，因为参与统计的有效样本集合变化，不表示传感器更准确。

## 可靠性与低可靠时间段

旧第3周评分使用真值误差、连续缺失、观测估计残差和相邻观测跳变，仅用于离线仿真验证。
当前基准低可靠时刻(s)：{reliability.loc[reliability['is_low_reliability'], 'time'].tolist()}。
异常区间和随机缺失均会降低分数；精确结果见CSV。阈值为0.5。
这套离线评分不得直接用于模拟在线决策，第4周另用`online_state.py`的预测残差及历史缺失评分，不读真值。

## 输出与复现

已重新生成`week03_noisy_czl.csv`、`week03_estimated_czl.csv`、`week03_reliability_czl.csv`，
以及真实/观测/估计对比图、可靠性图和`week03_parameter_comparison_czl.csv`。

```powershell
python src/run_pipeline.py --stage week03
```

## 局限与辅助工具

估计只做指数平滑，缺失时没有运动预测；离线可靠性依赖仿真真值。这些局限已在第4周在线模块中明确分开。
由Codex辅助核对任务、重新运行实验及整理此报告，数值来自实际运行。本人仍需现场说明代码和结果。
"""
    (report_dir / "week03_report_czl.md").write_text(report, encoding="utf-8")
    print("第3周完成：数据与当前横竖轨迹一致。\n", comparisons.to_string(index=False))
    return comparisons


# 统一解析命令行参数，再决定运行哪些阶段；函数定义本身不会立即执行各周实验。
def main():
    parser = argparse.ArgumentParser(description="无人机基础流程统一入口")
    # --stage限定可选阶段；默认all按第2、3、4周顺序运行。
    parser.add_argument("--stage", choices=("all", "week02", "week03", "week04"), default="all")
    # store_true表示命令中出现--show时才设为True，不需要再填写True这个值。
    parser.add_argument("--show", action="store_true", help="显示交互绘图窗口；默认只保存图片")
    args = parser.parse_args()
    # 默认选择后台绘图；在导入各周绘图模块前设置后端，适合批量生成文件。
    if not args.show:
        import matplotlib
        matplotlib.use("Agg")
        warnings.filterwarnings("ignore", message="FigureCanvasAgg is non-interactive.*", category=UserWarning)
    # 这三个if各自判断，不是if/elif互斥分支；all会让三个阶段全部执行。
    if args.stage in ("all", "week02"):
        run_week02()
    if args.stage in ("all", "week03"):
        run_week03()
    if args.stage in ("all", "week04"):
        # 第4周读取任务和干扰配置，独立执行在线闭环实验，不把第3周真值评分传给决策器。
        from evaluate_runs import evaluate
        evaluate()
    print("运行完成；输出见data、figures和reports。")


# 直接运行本文件时调用main；被其他文件import时只提供函数，不自动启动全部实验。
if __name__ == "__main__":
    main()
