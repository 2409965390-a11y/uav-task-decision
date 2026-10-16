# 第2周阅读起点：这个文件只读取任务，不控制无人机运动。
# 输入是JSON文件；输出mission是字典，交给simulator_2d中的模拟函数。
# 建议阅读顺序：PROJECT_ROOT → CONFIG_PATH → load_mission → 文件底部的直接运行入口。

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


# json负责把JSON文本转成Python字典。JSON中的数组会读成列表，数值会读成int或float。
import json
# Path是路径对象，使用 / 连接文件夹，比把某台电脑的盘符写进程序更方便迁移。
from pathlib import Path


# 获取项目根目录
# __file__ 表示当前文件路径
# parent.parent 表示返回到项目根目录
# __file__是本文件路径；resolve()得到绝对路径；第一个parent是src，第二个parent是项目根目录。
PROJECT_ROOT = Path(__file__).resolve().parent.parent


# 默认任务配置路径
CONFIG_PATH = (
    PROJECT_ROOT
    / "configs"
    / "mission_bridge_czl.json"
)


# def定义函数；参数有默认值，所以load_mission()读默认任务，传入路径则能切换到另一张地图。
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
    # with管理文件的打开和关闭；as file把打开的文件对象保存到变量file。任务JSON不要直接加入#注释。
    with open(config_path, "r", encoding="utf-8") as file:

        # load读取文件内容。例如mission['start']得到[6, 6]；这些是任务参数，不是传感器观测。
        mission = json.load(file)


    # 检查任务配置是否完整
    # 如果缺少字段，提前报错，
    # 避免模拟过程中出现难以定位的问题。

    # 这里只检查必需字段是否存在；坐标是否在地图内、障碍大小等更完整的检查在模拟器中完成。
    required_fields = [
        "map_size",
        "start",
        "goal",
        "speed",
        "safety_distance",
        "obstacles"
    ]


    # for依次取出每个字段名；field只是当前循环用的变量名，不是配置中的固定字段。
    for field in required_fields:

        # 对字典使用in检查的是键；缺键时主动报错，比后面运行到一半才发现更容易排查。
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


    # return把字典交回调用者；调用者写mission = load_mission()接住这个返回值。
    return mission



# 直接运行这个文件才进入下面的打印演示；被其他模块import时不会执行这段。
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