# 第3周核心①：在第2周已生成的真实轨迹上模拟测量误差，不改变无人机实际运动。
# 建议阅读顺序：load_noise_config → simulate_observations中的噪声/异常/缺失 → 返回表字段 → 直接运行入口。
# true_x/true_y属于仿真真值；observed_x/observed_y才是传感器报告。is_outlier是实验标签，不能给在线决策当答案。

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


# NumPy用于批量生成随机数和数组计算；np是常用缩写，不是新定义的算法。
import numpy as np
# Pandas负责CSV和表格；DataFrame可理解为有列名的二维数据表。
import pandas as pd
# PyYAML负责把噪声配置读成字典。YAML允许#注释，任务JSON则不允许这样直接注释。
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




# 配置和处理逻辑分开，修改噪声或缺失率时只改配置，不必重写加噪声函数。
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

        # safe_load读取常规数据类型，不执行YAML对象构造。返回字典后通过config['字段名']取值。
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

    # 创建表的副本作为处理输入；最终仍保留真实坐标用于离线评价，不把噪声写回Ground Truth文件。
    observations = ground_truth.copy()



    # 创建随机数生成器。
    #
    # 固定随机种子后，
    # 每次实验可以得到相同干扰，
    # 方便比较不同算法。

    # 固定种子使同一长度和相同参数下的实验可重复；轨迹点数改变时，不能直接沿用旧实验数值。
    rng = np.random.default_rng(
        config["random_seed"]
    )



    noise_std = config["position_noise_std"]

    missing_rate = config["missing_rate"]

    outlier_start = config["outlier_start"]

    outlier_length = config["outlier_length"]



    # 表的行数就是采样时刻数；点数随路线长度与时间步长变化，不应在程序中写死。
    sample_count = len(observations)



    # 保存真实位置

    # 从表中取出真实X列并转为数组，便于一次性对所有时刻做加法；Y列同理。
    true_x = observations["x"].to_numpy()

    true_y = observations["y"].to_numpy()



    # ============================
    # 1. 添加随机测量噪声
    # ============================


    # 一次抽取sample_count个随机误差：均值0、标准差noise_std；标准差大表示测量更分散。
    noise_x = rng.normal(
        0,
        noise_std,
        sample_count
    )


    # Y方向另抽一组随机数；X/Y并不共用同一个误差值。
    noise_y = rng.normal(
        0,
        noise_std,
        sample_count
    )



    # 数组逐元素相加：第i个真实坐标加第i个噪声，得到第i个观测坐标。Y方向同样处理。
    observed_x = true_x + noise_x

    observed_y = true_y + noise_y



    # ============================
    # 2. 制造连续异常观测
    # ============================


    # 异常结束下标不能超过数据长度；切片使用左闭右开区间，例如[50:60]是第50到59号样本。
    outlier_end = min(
        outlier_start + outlier_length,
        sample_count
    )


    # 异常偏移量设置为
    # 普通噪声标准差的5倍。
    #
    # 目的：
    # 制造明显但可控的传感器异常。

    # 这是人为构造的压力条件；当前实现把异常偏移与噪声标准差绑在一起，不代表真实传感器必然如此。
    outlier_offset = 5 * noise_std



    # 切片选中连续异常区间，只修改这段观测；这里X加偏移，Y在下面减偏移。
    observed_x[
        outlier_start:outlier_end
    ] += outlier_offset



    observed_y[
        outlier_start:outlier_end
    ] -= outlier_offset



    # 创建布尔标签数组，默认False；后面把人为设置的异常区间标True，用于解释和评价。
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


    # 每个样本生成一个[0,1)随机数，小于missing_rate则标记缺失；这是一组布尔掩码。
    missing_mask = (
        rng.random(sample_count)
        < missing_rate
    )



    # 使用NaN表示没有有效观测。
    #
    # 注意：
    # NaN不是坐标0，
    # 而是表示当前没有数据。

    # 布尔索引只选中缺失位置；赋NaN表示没收到测量。True的行在X/Y两列都会被置为NaN。
    observed_x[missing_mask] = np.nan

    observed_y[missing_mask] = np.nan




    # ============================
    # 4. 组织输出数据
    # ============================


    # 按字段组织输出：时间、两列真值、两列观测、缺失与异常标签；后续估计器根据这些列名读取。
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



    # 返回新的观测表，不在此函数里做状态估计、可靠性判断或飞行动作决策。
    return result





# 直接运行时读取第2周CSV和噪声配置，再保存第3周观测CSV；被统一入口import时不执行这里。
if __name__ == "__main__":


    # Step 1:
    # 读取真实轨迹

    # 输入必须与本次任务的真实路线对应；改了第2周任务后要重新生成这份输入，再重跑第3周。
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