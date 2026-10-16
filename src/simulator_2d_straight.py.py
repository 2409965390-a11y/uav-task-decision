"""
simulator_2d.py

功能：
    根据任务配置文件，模拟无人机在二维地图中的运动。

当前版本：
    - 使用二维直线运动模型；
    - 假设无人机保持固定速度；
    - 生成真实轨迹（Ground Truth）；
    - 输出轨迹CSV文件和可视化地图。

说明：
    本文件生成的是仿真环境中的真实状态。

    真实状态不会包含：
    - 传感器噪声；
    - 定位误差；
    - 数据缺失。

    第3周会基于这里生成的Ground Truth，
    模拟真实无人机可能遇到的观测问题。
"""


import math

import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from mission_io import load_mission, PROJECT_ROOT



def simulate_straight_line(mission, dt=1.0):
    """
    模拟无人机沿直线飞向目标点。

    参数：
        mission:
            任务配置字典。

            包含：
            - start: 起点坐标
            - goal: 目标坐标
            - speed: 飞行速度

        dt:
            时间采样间隔。

    返回：
        pandas.DataFrame

        包含无人机每个时间点的真实状态：

        time:
            当前时间

        x, y:
            无人机真实位置

        vx, vy:
            x和y方向速度

        target_id:
            当前目标编号

        task_progress:
            任务完成比例


    注意：
        当前版本只模拟二维直线飞行。

        不考虑：
        - 加速度变化；
        - 风扰动；
        - 避障；
        - 路径重新规划。

        后续路径规划模块会在此基础上扩展。
    """


    # 从任务配置中读取起点、终点和速度
    start_x, start_y = mission["start"]

    goal_x, goal_y = mission["goal"]

    speed = mission["speed"]


    # 检查时间步长是否合法
    if dt <= 0:
        raise ValueError("dt必须大于0")


    # 检查速度是否合法
    if speed <= 0:
        raise ValueError("speed必须大于0")


    # 计算目标方向向量
    # dx、dy表示无人机需要移动的水平和垂直距离

    dx = goal_x - start_x

    dy = goal_y - start_y


    # 使用勾股定理计算直线距离

    distance = math.hypot(dx, dy)


    # 如果起点和终点相同，
    # 说明任务已经完成，不需要移动。

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


    # 将无人机速度分解为x方向和y方向速度。
    #
    # speed表示实际飞行速度，
    # vx和vy只是速度在两个坐标轴上的分量。

    vx = speed * dx / distance

    vy = speed * dy / distance


    # 根据距离和速度计算预计飞行时间

    total_time = distance / speed


    # 根据时间间隔计算采样次数

    steps = math.ceil(total_time / dt)


    records = []


    # 按时间推进模拟过程

    for i in range(steps + 1):

        # 当前模拟时间
        #
        # 使用min限制最后一个时间点，
        # 避免超过实际到达时间。

        time = min(i * dt, total_time)


        # 当前任务完成比例
        #
        # 0表示刚开始，
        # 1表示已经到达目标。

        progress = time / total_time


        # 根据匀速运动模型计算当前位置
        #
        # 当前阶段生成Ground Truth，
        # 因此这里不会加入任何随机误差。

        x = start_x + vx * time

        y = start_y + vy * time


        records.append(
            {
                "time": time,
                "x": x,
                "y": y,
                "vx": vx,
                "vy": vy,
                "target_id": 0,
                "task_progress": progress
            }
        )


    return pd.DataFrame(records)




def plot_trajectory(mission, trajectory, save_path):
    """
    绘制二维任务地图和无人机真实轨迹。

    图中包含：
        - 地图范围；
        - 起点；
        - 终点；
        - 障碍物；
        - Ground Truth轨迹。


    当前障碍物仅用于地图展示。

    由于第2周尚未实现路径规划，
    无人机仍会按照直线路径运动。
    """


    map_width, map_height = mission["map_size"]


    start_x, start_y = mission["start"]

    goal_x, goal_y = mission["goal"]


    fig, ax = plt.subplots(figsize=(8, 8))


    # 绘制无人机真实轨迹

    ax.plot(
        trajectory["x"],
        trajectory["y"],
        linewidth=2,
        label="Ground Truth"
    )


    # 绘制起点和终点

    ax.scatter(
        start_x,
        start_y,
        marker="o",
        s=100,
        label="Start"
    )


    ax.scatter(
        goal_x,
        goal_y,
        marker="*",
        s=150,
        label="Goal"
    )


    # 绘制障碍物

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

    ax.set_xlim(
        0,
        map_width
    )

    ax.set_ylim(
        0,
        map_height
    )


    # 坐标说明

    ax.set_xlabel("X coordinate")

    ax.set_ylabel("Y coordinate")


    ax.set_title(
        "Week02 Ground Truth Trajectory"
    )


    # 地图网格

    ax.grid(True)


    # 保持x/y比例一致，
    # 避免地图被拉伸。

    ax.set_aspect(
        "equal",
        adjustable="box"
    )


    ax.legend()


    fig.savefig(
        save_path,
        dpi=200,
        bbox_inches="tight"
    )


    plt.show()




if __name__ == "__main__":


    # Step 1:
    # 读取任务配置。
    # 任务参数和程序代码分离，
    # 方便后续修改不同任务。

    mission = load_mission()



    # Step 2:
    # 根据任务生成无人机真实轨迹。

    trajectory = simulate_straight_line(
        mission
    )



    # Step 3:
    # 创建输出目录。

    data_dir = (
        PROJECT_ROOT
        / "data"
        / "simulated_observations"
    )


    figure_dir = (
        PROJECT_ROOT
        / "figures"
    )


    data_dir.mkdir(
        parents=True,
        exist_ok=True
    )


    figure_dir.mkdir(
        parents=True,
        exist_ok=True
    )



    csv_path = (
        data_dir
        / "week02_ground_truth_czl.csv"
    )


    figure_path = (
        figure_dir
        / "week02_ground_truth_czl.png"
    )



    # Step 4:
    # 保存Ground Truth数据。
    #
    # 后续传感器模拟模块会读取该文件。

    trajectory.to_csv(
        csv_path,
        index=False,
        encoding="utf-8-sig"
    )



    # Step 5:
    # 绘制地图轨迹图。

    plot_trajectory(
        mission,
        trajectory,
        figure_path
    )



    # 输出运行结果

    print("二维模拟完成！")

    print(
        "轨迹点数量：",
        len(trajectory)
    )

    print(
        "CSV保存位置：",
        csv_path
    )

    print(
        "图片保存位置：",
        figure_path
    )


    print("\n前5行轨迹数据：")

    print(
        trajectory.head()
    )


    print("\n最后5行轨迹数据：")

    print(
        trajectory.tail()
    )