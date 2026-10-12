"""
mission_io.py

功能：
    负责读取无人机任务配置文件。

说明：
    无人机任务中的地图大小、起点、终点、速度、
    障碍物等信息不直接写入模拟程序，而是保存在 JSON 文件中。

    这样做的好处：
        修改任务时只需要修改配置文件，
        不需要重新修改模拟代码。

    当前模块负责：
        1. 定位项目根目录；
        2. 读取 JSON 配置；
        3. 检查必要字段；
        4. 返回任务参数。
"""


import json
from pathlib import Path


# 获取项目根目录
# __file__ 表示当前文件路径
# parent.parent 表示返回到项目根目录
PROJECT_ROOT = Path(__file__).resolve().parent.parent


# 默认任务配置路径
CONFIG_PATH = (
    PROJECT_ROOT
    / "configs"
    / "mission_bridge_czl.json"
)


def load_mission(config_path=CONFIG_PATH):
    """
    读取无人机任务配置。

    参数：
        config_path:
            JSON任务文件路径。

    返回：
        mission:
            包含地图、起点、终点、速度等信息的字典。

    例如：
        {
            "map_size": [100,100],
            "start": [6,6],
            "goal": [61,61],
            "speed": 1.0
        }

    注意：
        这里读取的是任务信息，
        不是无人机实时状态。

        后续 simulator_2d.py 会根据这些参数
        生成 Ground Truth 轨迹。
    """


    # 打开JSON配置文件
    # encoding="utf-8"保证中文路径和注释正常读取
    with open(config_path, "r", encoding="utf-8") as file:

        mission = json.load(file)


    # 检查任务配置是否完整
    # 如果缺少字段，提前报错，
    # 避免模拟过程中出现难以定位的问题。

    required_fields = [
        "map_size",
        "start",
        "goal",
        "speed",
        "safety_distance",
        "obstacles"
    ]


    for field in required_fields:

        if field not in mission:

            raise ValueError(
                f"任务配置缺少字段：{field}"
            )


    # 基础参数合理性检查

    if mission["speed"] <= 0:

        raise ValueError(
            "无人机速度必须大于0"
        )


    if len(mission["map_size"]) != 2:

        raise ValueError(
            "地图大小必须包含X和Y两个方向"
        )


    return mission



if __name__ == "__main__":


    # 测试配置读取是否正常

    mission = load_mission()


    print("任务配置读取成功！")

    print("地图大小：", mission["map_size"])

    print("起点：", mission["start"])

    print("终点：", mission["goal"])

    print("速度：", mission["speed"])

    print("安全距离：", mission["safety_distance"])

    print("障碍物：", mission["obstacles"])