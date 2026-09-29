import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from mission_io import PROJECT_ROOT


# =========================================================
# 文件路径
# =========================================================

# 输入：第3周带噪观测数据
NOISY_DATA_PATH = (
    PROJECT_ROOT
    / "data"
    / "simulated_observations"
    / "week03_noisy_czl.csv"
)

# 输出：状态估计结果
ESTIMATED_OUTPUT_PATH = (
    PROJECT_ROOT
    / "data"
    / "simulated_observations"
    / "week03_estimated_czl.csv"
)

# 输出：可靠性结果
RELIABILITY_OUTPUT_PATH = (
    PROJECT_ROOT
    / "data"
    / "simulated_observations"
    / "week03_reliability_czl.csv"
)

# 输出：真实轨迹、观测轨迹、估计轨迹对比图
TRAJECTORY_FIGURE_PATH = (
    PROJECT_ROOT
    / "figures"
    / "week03_observation_and_estimate_czl.png"
)

# 输出：可靠性分数图
RELIABILITY_FIGURE_PATH = (
    PROJECT_ROOT
    / "figures"
    / "week03_reliability_score_czl.png"
)


# =========================================================
# 状态估计
# =========================================================

def estimate_state(data, alpha=0.8):
    """
    使用指数平滑估计无人机位置。

    alpha 越大：
    越相信当前观测，跟随速度越快。

    alpha 越小：
    越依赖之前的估计，曲线更平滑，
    但容易产生明显滞后。
    """

    estimated_x = []
    estimated_y = []

    previous_x = None
    previous_y = None

    for _, row in data.iterrows():

        observed_x = row["observed_x"]
        observed_y = row["observed_y"]

        # -------------------------------------------------
        # 当前观测缺失
        # -------------------------------------------------
        if pd.isna(observed_x) or pd.isna(observed_y):

            # 如果之前已经有估计值
            # 当前时刻暂时保持上一时刻的位置估计
            if previous_x is not None:

                current_x = previous_x
                current_y = previous_y

            # 如果第一条数据恰好就缺失
            # 当前仿真中暂时使用真实起点初始化
            else:

                current_x = row["true_x"]
                current_y = row["true_y"]

        # -------------------------------------------------
        # 当前存在有效观测
        # -------------------------------------------------
        else:

            # 第一个有效观测直接作为初始估计
            if previous_x is None:

                current_x = observed_x
                current_y = observed_y

            else:

                # 指数平滑
                current_x = (
                    alpha * observed_x
                    + (1 - alpha) * previous_x
                )

                current_y = (
                    alpha * observed_y
                    + (1 - alpha) * previous_y
                )

        # 保存当前估计值
        estimated_x.append(current_x)
        estimated_y.append(current_y)

        # 当前估计变成下一时刻的 previous
        previous_x = current_x
        previous_y = current_y

    # 复制原始观测数据
    result = data.copy()

    # 加入估计位置
    result["estimated_x"] = estimated_x
    result["estimated_y"] = estimated_y

    # -------------------------------------------------
    # 观测误差
    #
    # Observation 与 Ground Truth 之间的距离
    # -------------------------------------------------
    result["position_error"] = np.sqrt(
        (result["observed_x"] - result["true_x"]) ** 2
        + (result["observed_y"] - result["true_y"]) ** 2
    )

    # -------------------------------------------------
    # 估计误差
    #
    # Estimate 与 Ground Truth 之间的距离
    # -------------------------------------------------
    result["estimate_error"] = np.sqrt(
        (result["estimated_x"] - result["true_x"]) ** 2
        + (result["estimated_y"] - result["true_y"]) ** 2
    )

    return result


# =========================================================
# 可靠性评分
# =========================================================

def calculate_reliability(data):
    """
    根据以下信息计算简单可靠性分数：

    1. 当前观测是否缺失
    2. 是否连续缺失
    3. 观测位置与真实位置之间的误差
    4. 观测与估计之间的残差
    5. 相邻观测是否发生突然跳变

    可靠性分数范围：
    0 ~ 1

    分数越高表示当前观测越可靠。
    """

    reliability_scores = []
    missing_counts = []
    residuals = []

    # 连续缺失次数
    consecutive_missing = 0

    # 上一个有效观测
    previous_observed_x = None
    previous_observed_y = None

    for _, row in data.iterrows():

        observed_x = row["observed_x"]
        observed_y = row["observed_y"]

        estimated_x = row["estimated_x"]
        estimated_y = row["estimated_y"]

        # 默认完全可靠
        score = 1.0

        # =================================================
        # 1. 当前观测缺失
        # =================================================

        if pd.isna(observed_x) or pd.isna(observed_y):

            consecutive_missing += 1

            # 第一次缺失：
            # reliability = 0.35
            #
            # 连续第二次：
            # reliability = 0.25
            #
            # 连续第三次：
            # reliability = 0.15
            #
            # 最低保持在 0.05
            score = max(
                0.05,
                0.35 - 0.10 * (consecutive_missing - 1)
            )

            # 没有观测时无法计算观测-估计残差
            residual = np.nan

            # 缺失之后停止连续跳变比较
            previous_observed_x = None
            previous_observed_y = None

        # =================================================
        # 2. 当前有有效观测
        # =================================================

        else:

            # 当前没有连续缺失
            consecutive_missing = 0

            # -------------------------------------------------
            # 计算观测与估计之间的残差
            # -------------------------------------------------

            residual = np.sqrt(
                (observed_x - estimated_x) ** 2
                + (observed_y - estimated_y) ** 2
            )

            # -------------------------------------------------
            # 计算观测本身的误差
            #
            # 这里 position_error 已经由 estimate_state()
            # 根据 Ground Truth 计算完成
            # -------------------------------------------------

            observation_error = row["position_error"]

            # -------------------------------------------------
            # 3. 根据观测误差扣分
            # -------------------------------------------------

            # 普通小误差只少量降低可靠性
            #
            # 例如误差为 0.5：
            # 扣 0.5 / 4 = 0.125
            #
            # 最大只扣 0.65
            score -= min(
                observation_error / 4.0,
                0.65
            )

            # -------------------------------------------------
            # 4. 残差过大时额外扣分
            # -------------------------------------------------

            if residual > 1.0:

                score -= min(
                    (residual - 1.0) / 3.0,
                    0.15
                )

            # -------------------------------------------------
            # 5. 判断相邻观测是否突然跳变
            # -------------------------------------------------

            if (
                previous_observed_x is not None
                and previous_observed_y is not None
            ):

                jump = np.sqrt(
                    (observed_x - previous_observed_x) ** 2
                    + (observed_y - previous_observed_y) ** 2
                )

                # 如果两个连续观测之间距离突然超过 2
                # 进一步降低可靠性
                if jump > 2.0:

                    score -= 0.20

            # 当前观测保存下来
            # 给下一时刻计算 jump 使用
            previous_observed_x = observed_x
            previous_observed_y = observed_y

        # =================================================
        # 6. 把可靠性限制在 0～1
        # =================================================

        score = max(
            0.0,
            min(1.0, score)
        )

        reliability_scores.append(score)
        missing_counts.append(consecutive_missing)
        residuals.append(residual)

    # =====================================================
    # 生成可靠性数据表
    # =====================================================

    reliability = pd.DataFrame({
        "time": data["time"],
        "reliability_score": reliability_scores,
        "missing_count": missing_counts,
        "residual": residuals,

        # 低于 0.5 认为是低可靠观测
        "is_low_reliability": (
            np.array(reliability_scores) < 0.5
        )
    })

    return reliability


# =========================================================
# RMSE
# =========================================================

def calculate_rmse(error_series):
    """
    根据误差序列计算 RMSE。

    RMSE 越小，整体位置误差越小。
    """

    # 去掉 NaN
    valid_errors = error_series.dropna()

    return np.sqrt(
        np.mean(valid_errors ** 2)
    )


# =========================================================
# 绘制轨迹对比图
# =========================================================

def plot_trajectory(data):
    """
    绘制：

    Ground Truth
    Observation
    Estimate
    """

    fig, ax = plt.subplots(figsize=(7, 7))

    # 真实轨迹
    ax.plot(
        data["true_x"],
        data["true_y"],
        label="Ground Truth"
    )

    # 传感器观测
    ax.scatter(
        data["observed_x"],
        data["observed_y"],
        s=20,
        label="Observation"
    )

    # 状态估计
    ax.plot(
        data["estimated_x"],
        data["estimated_y"],
        label="Estimate"
    )

    ax.set_xlabel("X")
    ax.set_ylabel("Y")

    ax.set_title(
        "Week 03 Observation and Estimate"
    )

    ax.grid(True)
    ax.legend()

    ax.set_aspect(
        "equal",
        adjustable="box"
    )

    # 确保 figures 文件夹存在
    TRAJECTORY_FIGURE_PATH.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    # 保存图片
    fig.savefig(
        TRAJECTORY_FIGURE_PATH,
        dpi=200,
        bbox_inches="tight"
    )

    plt.show()


# =========================================================
# 绘制可靠性图
# =========================================================

def plot_reliability(reliability):
    """
    绘制可靠性分数。

    reliability < 0.5
    认为是低可靠观测。
    """

    fig, ax = plt.subplots(
        figsize=(9, 4)
    )

    # 可靠性曲线
    ax.plot(
        reliability["time"],
        reliability["reliability_score"],
        label="Reliability Score"
    )

    # 低可靠阈值
    ax.axhline(
        y=0.5,
        linestyle="--",
        label="Low Reliability Threshold"
    )

    # -------------------------------------------------
    # 找出所有低可靠观测
    # -------------------------------------------------

    low_reliability = reliability[
        reliability["is_low_reliability"]
    ]

    # 把低可靠点单独标出来
    ax.scatter(
        low_reliability["time"],
        low_reliability["reliability_score"],
        s=35,
        label="Low Reliability"
    )

    ax.set_ylim(
        0,
        1.05
    )

    ax.set_xlabel("Time")
    ax.set_ylabel("Reliability Score")

    ax.set_title(
        "Week 03 Reliability Score"
    )

    ax.grid(True)
    ax.legend()

    # 确保 figures 文件夹存在
    RELIABILITY_FIGURE_PATH.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    # 保存图片
    fig.savefig(
        RELIABILITY_FIGURE_PATH,
        dpi=200,
        bbox_inches="tight"
    )

    plt.show()


# =========================================================
# 主程序
# =========================================================

if __name__ == "__main__":

    # -------------------------------------------------
    # 1. 读取第3周带噪观测
    # -------------------------------------------------

    noisy_data = pd.read_csv(
        NOISY_DATA_PATH
    )

    # -------------------------------------------------
    # 2. 状态估计
    #
    # alpha = 0.8
    #
    # 当前观测占 80%
    # 上一次估计占 20%
    # -------------------------------------------------

    estimated_data = estimate_state(
        noisy_data,
        alpha=0.8
    )

    # -------------------------------------------------
    # 3. 计算可靠性
    # -------------------------------------------------

    reliability_data = calculate_reliability(
        estimated_data
    )

    # -------------------------------------------------
    # 4. 计算观测 RMSE
    # -------------------------------------------------

    observation_rmse = calculate_rmse(
        estimated_data["position_error"]
    )

    # -------------------------------------------------
    # 5. 计算估计 RMSE
    # -------------------------------------------------

    estimate_rmse = calculate_rmse(
        estimated_data["estimate_error"]
    )

    # -------------------------------------------------
    # 6. 整理需要保存的估计数据
    # -------------------------------------------------

    estimated_output = estimated_data[
        [
            "time",
            "estimated_x",
            "estimated_y",
            "position_error",
            "estimate_error"
        ]
    ]

    # -------------------------------------------------
    # 7. 确保数据文件夹存在
    # -------------------------------------------------

    ESTIMATED_OUTPUT_PATH.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    # -------------------------------------------------
    # 8. 保存状态估计 CSV
    # -------------------------------------------------

    estimated_output.to_csv(
        ESTIMATED_OUTPUT_PATH,
        index=False,
        encoding="utf-8-sig"
    )

    # -------------------------------------------------
    # 9. 保存可靠性 CSV
    # -------------------------------------------------

    reliability_data.to_csv(
        RELIABILITY_OUTPUT_PATH,
        index=False,
        encoding="utf-8-sig"
    )

    # -------------------------------------------------
    # 10. 绘图
    # -------------------------------------------------

    plot_trajectory(
        estimated_data
    )

    plot_reliability(
        reliability_data
    )

    # -------------------------------------------------
    # 11. 输出实验结果
    # -------------------------------------------------

    print("状态估计完成！")

    print(
        "观测 RMSE：",
        observation_rmse
    )

    print(
        "估计 RMSE：",
        estimate_rmse
    )

    print(
        "低可靠观测数量：",
        reliability_data[
            "is_low_reliability"
        ].sum()
    )

    print(
        "估计数据保存位置：",
        ESTIMATED_OUTPUT_PATH
    )

    print(
        "可靠性数据保存位置：",
        RELIABILITY_OUTPUT_PATH
    )

    # -------------------------------------------------
    # 12. 打印所有低可靠数据
    # -------------------------------------------------

    low_reliability_data = reliability_data[
        reliability_data[
            "is_low_reliability"
        ]
    ]

    print(
        "\n低可靠观测："
    )

    print(
        low_reliability_data
    )