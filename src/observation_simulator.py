import numpy as np
import pandas as pd
import yaml

from mission_io import PROJECT_ROOT


# 第2周生成的真实轨迹
GROUND_TRUTH_PATH = (
    PROJECT_ROOT
    / "data"
    / "simulated_observations"
    / "week02_ground_truth_czl.csv"
)

# 第3周噪声配置
NOISE_CONFIG_PATH = (
    PROJECT_ROOT
    / "configs"
    / "observation_noise.yaml"
)

# 第3周带噪观测输出
OUTPUT_PATH = (
    PROJECT_ROOT
    / "data"
    / "simulated_observations"
    / "week03_noisy_czl.csv"
)


def load_noise_config(config_path=NOISE_CONFIG_PATH):
    """读取观测噪声配置文件。"""

    with open(config_path, "r", encoding="utf-8") as file:
        config = yaml.safe_load(file)

    required_fields = [
        "position_noise_std",
        "missing_rate",
        "outlier_start",
        "outlier_length",
        "random_seed"
    ]

    for field in required_fields:
        if field not in config:
            raise ValueError(f"噪声配置缺少字段：{field}")

    return config


def simulate_observations(ground_truth, config):
    """根据真实轨迹生成带噪声、缺失和连续异常的观测。"""

    # 复制真实轨迹，避免修改原始数据
    observations = ground_truth.copy()

    # 创建随机数生成器
    rng = np.random.default_rng(config["random_seed"])

    # 获取参数
    noise_std = config["position_noise_std"]
    missing_rate = config["missing_rate"]
    outlier_start = config["outlier_start"]
    outlier_length = config["outlier_length"]

    sample_count = len(observations)

    # 保存真实位置
    true_x = observations["x"].to_numpy()
    true_y = observations["y"].to_numpy()

    # 1. 给真实位置加入随机高斯噪声
    noise_x = rng.normal(
        loc=0.0,
        scale=noise_std,
        size=sample_count
    )

    noise_y = rng.normal(
        loc=0.0,
        scale=noise_std,
        size=sample_count
    )

    observed_x = true_x + noise_x
    observed_y = true_y + noise_y

    # 2. 制造一段连续异常观测
    outlier_end = min(
        outlier_start + outlier_length,
        sample_count
    )

    # 这里使用 5 倍噪声标准差作为额外异常偏移
    outlier_offset = 5 * noise_std

    observed_x[outlier_start:outlier_end] += outlier_offset
    observed_y[outlier_start:outlier_end] -= outlier_offset

    # 记录哪些位置属于人为制造的连续异常段
    is_outlier = np.zeros(sample_count, dtype=bool)
    is_outlier[outlier_start:outlier_end] = True

    # 3. 随机制造观测缺失
    missing_mask = (
        rng.random(sample_count) < missing_rate
    )

    observed_x[missing_mask] = np.nan
    observed_y[missing_mask] = np.nan

    # 4. 整理输出数据
    result = pd.DataFrame({
        "time": observations["time"],
        "true_x": true_x,
        "true_y": true_y,
        "observed_x": observed_x,
        "observed_y": observed_y,
        "is_missing": missing_mask,
        "is_outlier": is_outlier
    })

    return result


if __name__ == "__main__":

    # 读取真实轨迹
    ground_truth = pd.read_csv(GROUND_TRUTH_PATH)

    # 读取噪声配置
    config = load_noise_config()

    # 生成模拟传感器观测
    noisy_data = simulate_observations(
        ground_truth,
        config
    )

    # 保存 CSV
    OUTPUT_PATH.parent.mkdir(
        parents=True,
        exist_ok=True
    )

    noisy_data.to_csv(
        OUTPUT_PATH,
        index=False,
        encoding="utf-8-sig"
    )

    print("带噪观测生成完成！")
    print("数据总数：", len(noisy_data))
    print("缺失观测数量：", noisy_data["is_missing"].sum())
    print("异常段观测数量：", noisy_data["is_outlier"].sum())
    print("输出位置：", OUTPUT_PATH)

    print("\n前5行数据：")
    print(noisy_data.head())

    print("\n第48到62行数据：")
    print(noisy_data.iloc[48:63])