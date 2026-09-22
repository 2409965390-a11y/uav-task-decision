import json
from pathlib import Path



PROJECT_ROOT = Path(__file__).resolve().parent.parent


CONFIG_PATH = PROJECT_ROOT / "configs" / "mission_bridge_czl.json"


def load_mission(config_path=CONFIG_PATH):


    with open(config_path, "r", encoding="utf-8") as file:
        mission = json.load(file)


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
            raise ValueError(f"任务配置缺少字段：{field}")

    return mission


if __name__ == "__main__":
    mission = load_mission()

    print("任务配置读取成功！")
    print("地图大小：", mission["map_size"])
    print("起点：", mission["start"])
    print("终点：", mission["goal"])
    print("速度：", mission["speed"])
    print("安全距离：", mission["safety_distance"])
    print("障碍物：", mission["obstacles"])