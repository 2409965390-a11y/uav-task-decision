"""
observation_simulator.py

功能：
    根据二维仿真生成的真实轨迹，
    模拟无人机传感器产生的观测数据。

输入：
    week02_ground_truth_czl.csv

输出：
    week03_noisy_czl.csv


模拟的问题包括：

1. 随机测量误差
    ----------------
    模拟GPS、视觉定位等传感器
    存在的小范围测量偏差。


2. 观测缺失
    ----------------
    模拟通信中断、遮挡等情况，
    导致某些时刻无法获得有效位置。


3. 连续异常
    ----------------
    模拟传感器短时间受到干扰，
    产生连续错误观测。


注意：
    Ground Truth表示仿真环境中的真实状态，
    本文件不会修改它。

    所有误差只作用于Observation，
    用于测试后续状态估计算法。
"""


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



# 噪声配置文件

NOISE_CONFIG_PATH = (
    PROJECT_ROOT
    / "configs"
    / "observation_noise.yaml"
)



# 输出带噪观测

OUTPUT_PATH = (
    PROJECT_ROOT
    / "data"
    / "simulated_observations"
    / "week03_noisy_czl.csv"
)




def load_noise_config(config_path=NOISE_CONFIG_PATH):
    """
    读取传感器误差配置。

    配置包括：

    position_noise_std:
        高斯噪声标准差。

    missing_rate:
        观测缺失概率。

    outlier_start:
        连续异常开始位置。

    outlier_length:
        连续异常持续长度。

    random_seed:
        随机种子，用于保证实验可复现。
    """


    with open(
        config_path,
        "r",
        encoding="utf-8"
    ) as file:

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

            raise ValueError(
                f"噪声配置缺少字段：{field}"
            )



    return config





def simulate_observations(
        ground_truth,
        config
):
    """
    根据真实轨迹生成模拟传感器观测。


    输入：

        ground_truth:
            仿真环境中的真实无人机位置。


        config:
            传感器误差参数。


    输出：

        DataFrame:

            true_x,true_y:
                真实位置。

            observed_x,observed_y:
                传感器观测位置。

            is_missing:
                是否缺失。

            is_outlier:
                是否属于人为异常区域。



    说明：

        真实无人机不会知道Ground Truth，
        但在仿真阶段可以使用它评价算法效果。
    """


    # 复制数据，
    # 防止修改原始真实轨迹。

    observations = ground_truth.copy()



    # 创建随机数生成器。
    #
    # 固定随机种子后，
    # 每次实验可以得到相同干扰，
    # 方便比较不同算法。

    rng = np.random.default_rng(
        config["random_seed"]
    )



    noise_std = config["position_noise_std"]

    missing_rate = config["missing_rate"]

    outlier_start = config["outlier_start"]

    outlier_length = config["outlier_length"]



    sample_count = len(observations)



    # 保存真实位置

    true_x = observations["x"].to_numpy()

    true_y = observations["y"].to_numpy()



    # ============================
    # 1. 添加随机测量噪声
    # ============================


    noise_x = rng.normal(
        0,
        noise_std,
        sample_count
    )


    noise_y = rng.normal(
        0,
        noise_std,
        sample_count
    )



    observed_x = true_x + noise_x

    observed_y = true_y + noise_y



    # ============================
    # 2. 制造连续异常观测
    # ============================


    outlier_end = min(
        outlier_start + outlier_length,
        sample_count
    )


    # 异常偏移量设置为
    # 普通噪声标准差的5倍。
    #
    # 目的：
    # 制造明显但可控的传感器异常。

    outlier_offset = 5 * noise_std



    observed_x[
        outlier_start:outlier_end
    ] += outlier_offset



    observed_y[
        outlier_start:outlier_end
    ] -= outlier_offset



    is_outlier = np.zeros(
        sample_count,
        dtype=bool
    )


    is_outlier[
        outlier_start:outlier_end
    ] = True




    # ============================
    # 3. 制造随机缺失
    # ============================


    missing_mask = (
        rng.random(sample_count)
        < missing_rate
    )



    # 使用NaN表示没有有效观测。
    #
    # 注意：
    # NaN不是坐标0，
    # 而是表示当前没有数据。

    observed_x[missing_mask] = np.nan

    observed_y[missing_mask] = np.nan




    # ============================
    # 4. 组织输出数据
    # ============================


    result = pd.DataFrame({

        "time":
            observations["time"],


        "true_x":
            true_x,


        "true_y":
            true_y,


        "observed_x":
            observed_x,


        "observed_y":
            observed_y,


        "is_missing":
            missing_mask,


        "is_outlier":
            is_outlier

    })



    return result





if __name__ == "__main__":


    # Step 1:
    # 读取真实轨迹

    ground_truth = pd.read_csv(
        GROUND_TRUTH_PATH
    )



    # Step 2:
    # 读取噪声参数

    config = load_noise_config()



    # Step 3:
    # 生成模拟观测

    noisy_data = simulate_observations(
        ground_truth,
        config
    )



    # 创建输出目录

    OUTPUT_PATH.parent.mkdir(
        parents=True,
        exist_ok=True
    )



    # 保存结果

    noisy_data.to_csv(
        OUTPUT_PATH,
        index=False,
        encoding="utf-8-sig"
    )



    print("带噪观测生成完成！")

    print(
        "数据总数：",
        len(noisy_data)
    )


    print(
        "缺失数量：",
        noisy_data["is_missing"].sum()
    )


    print(
        "异常数量：",
        noisy_data["is_outlier"].sum()
    )


    print(
        "输出位置：",
        OUTPUT_PATH
    )