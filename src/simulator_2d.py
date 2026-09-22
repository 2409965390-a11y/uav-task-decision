import math

import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from mission_io import load_mission, PROJECT_ROOT


def simulate_straight_line(mission, dt=1.0):
    """让无人机从起点沿直线以固定速度飞向终点。"""

    start_x, start_y = mission["start"]
    goal_x, goal_y = mission["goal"]
    speed = mission["speed"]

    # 终点减去起点，得到两个方向上的距离
    dx = goal_x - start_x
    dy = goal_y - start_y

    # 计算起点到终点的直线距离
    distance = math.hypot(dx, dy)

    if speed <= 0:
        raise ValueError("speed 必须大于 0")

    # 如果起点和终点相同，就不用移动
    if distance == 0:
        return pd.DataFrame([
            {
                "time": 0.0,
                "x": start_x,
                "y": start_y,
                "vx": 0.0,
                "vy": 0.0,
                "target_id": 0,
                "task_progress": 1.0
            }
        ])

    # 把总速度分解成 x、y 两个方向的速度
    vx = speed * dx / distance
    vy = speed * dy / distance

    # 计算总共需要飞多久
    total_time = distance / speed

    # 需要记录多少个时间步
    steps = math.ceil(total_time / dt)

    records = []

    for i in range(steps + 1):

        # 当前时间
        time = min(i * dt, total_time)

        # 任务完成比例：0 表示刚开始，1 表示到达终点
        progress = time / total_time

        # 根据速度计算当前位置
        x = start_x + vx * time
        y = start_y + vy * time

        records.append({
            "time": time,
            "x": x,
            "y": y,
            "vx": vx,
            "vy": vy,
            "target_id": 0,
            "task_progress": progress
        })

    return pd.DataFrame(records)


def plot_trajectory(mission, trajectory, save_path):
    """画出地图、障碍物、起点、终点和真实轨迹。"""

    map_width, map_height = mission["map_size"]
    start_x, start_y = mission["start"]
    goal_x, goal_y = mission["goal"]

    fig, ax = plt.subplots(figsize=(7, 7))

    # 绘制真实轨迹
    ax.plot(
        trajectory["x"],
        trajectory["y"],
        label="Ground Truth"
    )

    # 绘制起点和终点
    ax.scatter(start_x, start_y, marker="o", s=80, label="Start")
    ax.scatter(goal_x, goal_y, marker="x", s=100, label="Goal")

    # 绘制所有障碍物
    for index, obstacle in enumerate(mission["obstacles"]):

        xmin = obstacle["xmin"]
        xmax = obstacle["xmax"]
        ymin = obstacle["ymin"]
        ymax = obstacle["ymax"]

        rectangle = Rectangle(
            (xmin, ymin),
            xmax - xmin,
            ymax - ymin,
            alpha=0.3,
            label="Obstacle" if index == 0 else None
        )

        ax.add_patch(rectangle)

    # 设置地图范围
    ax.set_xlim(0, map_width)
    ax.set_ylim(0, map_height)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Week 02 Ground Truth Trajectory")

    ax.grid(True)
    ax.legend()
    ax.set_aspect("equal", adjustable="box")

    # 保存图片
    fig.savefig(save_path, dpi=200, bbox_inches="tight")

    # 显示图片
    plt.show()


if __name__ == "__main__":

    # 1. 读取任务
    mission = load_mission()

    # 2. 模拟无人机飞行
    trajectory = simulate_straight_line(mission)

    # 3. 准备输出路径
    data_dir = PROJECT_ROOT / "data" / "simulated_observations"
    figure_dir = PROJECT_ROOT / "figures"

    data_dir.mkdir(parents=True, exist_ok=True)
    figure_dir.mkdir(parents=True, exist_ok=True)

    csv_path = data_dir / "week02_ground_truth_czl.csv"
    figure_path = figure_dir / "week02_ground_truth_czl.png"

    # 4. 保存 CSV
    trajectory.to_csv(
        csv_path,
        index=False,
        encoding="utf-8-sig"
    )

    # 5. 绘制并保存轨迹图
    plot_trajectory(
        mission,
        trajectory,
        figure_path
    )

    # 6. 输出一些信息，方便检查
    print("二维模拟完成！")
    print("轨迹点数量：", len(trajectory))
    print("CSV 保存位置：", csv_path)
    print("图片保存位置：", figure_path)

    print("\n前5行轨迹数据：")
    print(trajectory.head())

    print("\n最后5行轨迹数据：")
    print(trajectory.tail())