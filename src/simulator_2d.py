# 第2周核心：读取任务参数后，先规划横竖路线，再按时间步生成真实轨迹，最后保存并绘图。
# 建议先读simulate_motion理解运动，再读_find_route理解路线，最后读碰撞检查和绘图。
# position是真实仿真位置，waypoint是当前航点；这里没有噪声和在线估计。

"""二维水平/竖直逐步运动与简单静态避障。

保留 simulate_motion(mission, dt=1.0)、plot_trajectory() 和原 CSV 格式。
已知矩形障碍按 safety_distance 向外扩张。路线只能水平或竖直，
转向时保留直角航点；通过四邻接坐标网格搜索绕行路线，再逐步执行。
这是静态点模型，不包含动态障碍、转弯动力学或传感器决策。
"""

import heapq
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import pandas as pd

from mission_io import load_mission, PROJECT_ROOT


# 统一列顺序，确保后续观测程序可以继续读取原CSV接口；单位约定为秒、米和米/秒。
COLUMNS = ["time", "x", "y", "vx", "vy", "target_id", "task_progress"]


# 以下划线开头的是内部辅助函数。先把输入转为有限数值，再进行运动计算，避免无穷或NaN传播。
def _number(value, name):
    """拒绝 NaN、无穷大和非数字，避免产生无效轨迹。"""
    # Python中bool属于int的子类，但True不能当作速度1使用，因此先单独拒绝。
    if isinstance(value, bool):
        raise ValueError(f"{name}必须是有限数值")
    try:
        result = float(value)
    # 捕获类型、内容或范围不合法的输入，转成带字段名的清晰错误，而不是让后面的计算莫名失败。
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name}必须是有限数值") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name}必须是有限数值")
    return result


# 坐标和地图大小都需要两个分量。返回tuple后可写x,y = ...进行拆包。
def _pair(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(f"{name}必须包含两个数值")
    return tuple(_number(item, name) for item in value)


# 把每个障碍向四周扩张安全距离：xmin减margin、xmax加margin，Y方向相同。
# 后面仍按点模型检测扩张矩形，因此能保守地保留安全距离；角部比圆形欧氏膨胀更保守。
def _expanded_obstacles(mission):
    margin = _number(mission["safety_distance"], "safety_distance")
    if margin < 0:
        raise ValueError("safety_distance不能小于0")
    if not isinstance(mission["obstacles"], list):
        raise ValueError("obstacles必须是列表")
    rectangles = []
    # enumerate同时得到编号和当前障碍字典；障碍多少个，循环就检查多少个，不需要另写算法。
    for index, obstacle in enumerate(mission["obstacles"]):
        if not isinstance(obstacle, dict):
            raise ValueError(f"障碍物{index}必须是字典")
        try:
            # 生成器按四个键的顺序读取坐标，再拆成四个变量；障碍用轴对齐矩形表示。
            xmin, xmax, ymin, ymax = (
                _number(obstacle[key], f"障碍物{index}.{key}")
                for key in ("xmin", "xmax", "ymin", "ymax")
            )
        except KeyError as exc:
            raise ValueError(f"障碍物{index}缺少字段：{exc.args[0]}") from exc
        if xmin >= xmax or ymin >= ymax:
            raise ValueError(f"障碍物{index}的最小坐标必须小于最大坐标")
        rectangles.append((xmin - margin, xmax + margin,
                           ymin - margin, ymax + margin))
    return rectangles


# 线段写成start+t*(end-start)，其中t从0到1。下面求线段进入/离开矩形X、Y区间的t范围。
# 两个方向都允许的t有交集，才说明整条运动线段碰到矩形；只检查终点会漏掉高速穿障。
def _segment_hits_rectangle(start, end, rectangle):
    """检查整条线段与闭合矩形是否相交，包含擦边、穿越和零长度线段。"""
    xmin, xmax, ymin, ymax = rectangle
    # 一开始允许整条线段；随后用每个轴的有效区间不断收窄进入和离开的参数。
    enter, leave = 0.0, 1.0
    for origin, delta, lower, upper in (
        (start[0], end[0] - start[0], xmin, xmax),
        (start[1], end[1] - start[1], ymin, ymax),
    ):
        # 当前轴坐标不变，线段与该轴边界平行；若该坐标不在矩形范围内，则不可能碰到矩形。
        if delta == 0.0:
            if origin < lower or origin > upper:
                return False
            continue
        # 求该轴上下边界对应的t，并排序。这样向正方向和负方向运动都能使用同一逻辑。
        first, last = sorted(((lower - origin) / delta, (upper - origin) / delta))
        # 取区间交集：进入时间取较晚者，离开时间取较早者。擦边时enter等于leave，仍按碰撞处理。
        enter, leave = max(enter, first), min(leave, last)
        # 交集为空表示某个方向无法同时满足，整条线段与矩形不相交。
        if enter > leave:
            return False
    return True


# 先检查地图边界，再逐个检查扩张障碍。返回True才允许把这一整段当成可行路径。
def _segment_is_safe(start, end, map_size, rectangles):
    # 地图是凸矩形；两端都在地图内，连接线段也在地图内。
    width, height = map_size
    if any(not (0 <= x <= width and 0 <= y <= height) for x, y in (start, end)):
        return False
    # any只要找到一个碰撞就为True，外层not把它变为不安全；没有任何碰撞才返回True。
    return not any(_segment_hits_rectangle(start, end, box) for box in rectangles)


# 对外接口与_segment_is_safe的布尔含义相反：这里True表示碰撞或越界，False表示安全。
def check_collision(start, end, mission):
    """下一步整段运动是否越界或触碰障碍物的安全区域。"""
    return not _segment_is_safe(
        _pair(start, "start"), _pair(end, "end"),
        _pair(mission["map_size"], "map_size"), _expanded_obstacles(mission),
    )


# 规划只允许水平和竖直移动。优先尝试简单路线；两个L形方案都被挡住时，再搜索坐标网格。
def _find_route(start, goal, map_size, rectangles):
    """生成贪吃蛇式横竖折线；任何路线段都只改变一个坐标。"""
    if start == goal:
        return [start]
    if (start[0] == goal[0] or start[1] == goal[1]) and _segment_is_safe(
        start, goal, map_size, rectangles
    ):
        return [start, goal]

    # 先尝试两个简单的L形路线，安全时无需搜索。优先纵向再横向。
    # 两个候选拐点分别代表先竖后横、先横后竖；每段都安全才直接返回对应L形路线。
    for corner in ((start[0], goal[1]), (goal[0], start[1])):
        if _segment_is_safe(start, corner, map_size, rectangles) and _segment_is_safe(
            corner, goal, map_size, rectangles
        ):
            return [start, corner, goal]

    # 避开安全区域边缘的精确接触；不把贴边运动当作安全。
    # 在安全边界外再留一个极小数值间隙，防止规划角点恰好落在被定义为碰撞的闭合边界上。
    clearance = 1e-7 * max(1.0, *map_size)
    width, height = map_size
    # 花括号创建集合，自动去重。加入起终点和障碍附近的关键X/Y坐标，形成压缩坐标网格。
    x_values = {0.0, width, start[0], goal[0]}
    y_values = {0.0, height, start[1], goal[1]}
    for xmin, xmax, ymin, ymax in rectangles:
        x_values.update(x for x in (xmin - clearance, xmax + clearance) if 0 <= x <= width)
        y_values.update(y for y in (ymin - clearance, ymax + clearance) if 0 <= y <= height)
    # 排序后相邻下标对应相邻坐标线；它不是每米一个格子的固定栅格，允许非整数起终点。
    xs, ys = sorted(x_values), sorted(y_values)
    nodes = {}
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            point = (x, y)
            if _segment_is_safe(point, point, map_size, rectangles):
                # nodes的键(i,j)是坐标线下标，值point才是真实(x,y)坐标。两种编号不要混淆。
                nodes[(i, j)] = point

    # 只连接上下左右相邻节点，禁止斜向边；每条边都检查整段碰撞。
    # 邻接表：每个安全节点对应一个可通往邻居的列表，后面只连接横竖方向的安全边。
    graph = {index: [] for index in nodes}
    for (i, j), point in nodes.items():
        # 先检查右侧与上侧，再把边反向加入邻居，就得到上下左右四邻接，始终没有对角线边。
        # 这里axis=1表示横向、2表示纵向，仅用于统计转弯；它不同于控制器数组的0/1下标。
        for neighbor, axis in (((i + 1, j), 1), ((i, j + 1), 2)):
            if neighbor in nodes and _segment_is_safe(point, nodes[neighbor], map_size, rectangles):
                length = math.dist(point, nodes[neighbor])
                graph[(i, j)].append((neighbor, length, axis))
                graph[neighbor].append(((i, j), length, axis))

    start_index = (xs.index(start[0]), ys.index(start[1]))
    goal_index = (xs.index(goal[0]), ys.index(goal[1]))
    # 搜索状态同时记录上次移动的轴；长度相同的路线优先少转弯。
    # 搜索状态不仅记录节点，还记录上次运动轴。0表示尚未出发，所以第一段不算转弯。
    initial = (start_index, 0)
    # 代价是(累计长度,转弯数)。Python元组先比较长度，长度完全相同再比较转弯数。
    costs = {initial: (0.0, 0)}
    parents = {initial: None}
    queue = [(0.0, 0, start_index, 0)]
    while queue:
        # 小根堆每次取当前代价最小的候选，这是Dijkstra搜索的核心；不是随机尝试方向。
        distance, turns, index, previous_axis = heapq.heappop(queue)
        state = (index, previous_axis)
        # 同一状态可能多次入堆；若后来找到更好路线，旧候选仍在堆里，这里跳过过期记录。
        if (distance, turns) != costs[state]:
            continue
        # 首次弹出目标状态后，按parents从终点反向找回起点，得到一条完整的横竖路线。
        if index == goal_index:
            route = []
            while state is not None:
                route.append(nodes[state[0]])
                state = parents[state]
            # 合并同一直线上的中间节点，只在真正的拐弯处缩短时间步。
            simplified = []
            # 反向回溯得到的是终点到起点，reversed恢复执行顺序；随后合并同一直线上的多余节点。
            for point in reversed(route):
                if len(simplified) >= 2:
                    first, last = simplified[-2:]
                    if (first[0] == last[0] == point[0] or
                            first[1] == last[1] == point[1]):
                        # 删掉位于同一直线中间的节点，只保留真正拐点，避免执行时在无意义节点前反复缩短一步。
                        simplified.pop()
                simplified.append(point)
            return simplified
        for neighbor, length, axis in graph[index]:
            next_state = (neighbor, axis)
            # 沿边增加长度；如果上个轴既非初始0也非当前轴，就多一次转弯。布尔值int(True)等于1。
            candidate = (distance + length, turns + int(previous_axis not in (0, axis)))
            if candidate < costs.get(next_state, (math.inf, math.inf)):
                costs[next_state] = candidate
                # 保存这个更好状态从哪里来；只有最终回溯时才把路线拼起来。
                parents[next_state] = (index, previous_axis)
                heapq.heappush(queue, (*candidate, neighbor, axis))

    raise ValueError("未找到安全绕行路线，请检查障碍物、安全距离及起终点配置")


# 输入任务字典及时间步长dt，输出每个时刻的真实状态表。dt是仿真时间，不是让程序sleep这么久。
def simulate_motion(mission, dt=1.0):
    """生成原格式 Ground Truth，时间固定间隔 dt。

    第0行是真实起点；vx/vy 表示从本行到下一行的实际位移除以 dt，
    终点速度为0。每步位移不超过 speed*dt，靠近转角或终点时缩短
    最后一步，避免跨角切入障碍。每步只有vx或vy之一非零，禁止斜走。
    进度按已走路线长度计算，单调递增。
    """
    required = ("map_size", "start", "goal", "speed", "safety_distance", "obstacles")
    for field in required:
        if field not in mission:
            raise ValueError(f"任务配置缺少字段：{field}")
    dt = _number(dt, "dt")
    speed = _number(mission["speed"], "speed")
    map_size = _pair(mission["map_size"], "map_size")
    start, goal = _pair(mission["start"], "start"), _pair(mission["goal"], "goal")
    if dt <= 0 or speed <= 0:
        raise ValueError("dt和speed必须大于0")
    if any(size <= 0 for size in map_size):
        raise ValueError("地图宽度和高度必须大于0")
    # 单步最大位移 = 速度×时间。speed=1、dt=1时最多走1米；拐点和终点前可少走一点。
    max_step = speed * dt
    if not math.isfinite(max_step) or max_step <= 0:
        raise ValueError("speed*dt必须是有限正数")
    rectangles = _expanded_obstacles(mission)
    # 起点和终点也必须安全；不能从障碍中生成看起来到达成功的轨迹。
    for name, point in (("起点", start), ("终点", goal)):
        if not _segment_is_safe(point, point, map_size, rectangles):
            raise ValueError(f"{name}越界或位于障碍物的安全区域内")

    # 先计算航点，再执行运动。route包含起点、必要的中间拐点和最终目标，而不是每秒的全部坐标。
    route = _find_route(start, goal, map_size, rectangles)
    # zip把相邻航点两两配对，累加每段长度。绕行时用整条路线长度算进度，不能只用起终点直线距离。
    total_length = sum(math.dist(a, b) for a, b in zip(route, route[1:]))
    records = [{"time": 0.0, "x": start[0], "y": start[1],
                "vx": 0.0, "vy": 0.0, "target_id": 0,
                "task_progress": 1.0 if start == goal else 0.0}]
    # position是本时刻真实坐标，travelled是从出发到现在累计移动的路径长度。
    position, travelled = start, 0.0
    # route[1:]跳过已所在的起点，依次把后续航点赋给waypoint。外层循环决定当前执行哪一段。
    for waypoint in route[1:]:
        # 内层循环逐步走到当前航点；到达后退出while，外层for再换下一个航点。
        while position != waypoint:
            # 计算到当前航点的剩余距离，而不是总到最终目标的距离。
            distance = math.dist(position, waypoint)
            if distance <= max_step:
                # 剩余不足一步就精确落到航点，避免越过拐角；这一步的平均速度可能低于设定速度。
                next_position = waypoint
            else:
                # 例如还剩55米、本步走1米，比例就是1/55。它只表示沿当前线段前进多大比例。
                ratio = max_step / distance
                # zip分别配对当前和目标的X/Y；a是当前分量，b是目标分量，tuple把两个新坐标组成元组。
                # 例如(6,6)到(6,61)，X差为0、Y差55，乘1/55后下一点为(6,7)，因此不会斜走。
                next_position = tuple(a + (b - a) * ratio for a, b in zip(position, waypoint))
            # 位移太小可能被浮点精度吞掉，主动报错避免while循环永远无法前进。
            if next_position == position:
                raise ValueError("时间步过小，无法产生可表示的位移，请增大dt或speed")
            # 不仅规划时检查，执行每一步也再次检查整段，避免程序生成跨角或穿障的记录。
            if not _segment_is_safe(position, next_position, map_size, rectangles):
                raise RuntimeError("下一步运动不安全，已停止仿真")

            # 先为上一行填入本时间段速度，再记录下一时刻位置。
            # [-1]是最后一条记录。本行速度表示从本行时刻到下一行时刻的实际位移除以dt。
            records[-1]["vx"] = (next_position[0] - position[0]) / dt
            records[-1]["vy"] = (next_position[1] - position[1]) / dt
            travelled += math.dist(position, next_position)
            records.append({
                # 第0行已经记录起点，所以记录数乘dt恰好得到下一时刻；固定时间间隔也便于第3周按行加噪声。
                "time": len(records) * dt,
                "x": next_position[0], "y": next_position[1],
                "vx": 0.0, "vy": 0.0, "target_id": 0,
                "task_progress": (1.0 if next_position == goal else
                                  min(travelled / total_length, math.nextafter(1.0, 0.0))),
            })
            position = next_position

    trajectory = pd.DataFrame(records, columns=COLUMNS)
    # 附加规划航点供第4周复用；attrs不是CSV字段，保存CSV时只保留COLUMNS的七列。
    trajectory.attrs["planned_route"] = route
    return trajectory


# 此函数只把已有数据画出来，不负责移动、避障或修改轨迹。图由trajectory中的真实坐标连接而成。
def plot_trajectory(mission, trajectory, save_path):
    """绘制地图、安全区域和横竖折线轨迹，并保持原显示行为。"""
    width, height = mission["map_size"]
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_facecolor("#f5f5dc")
    for index, (obstacle, box) in enumerate(zip(mission["obstacles"], _expanded_obstacles(mission))):
        xmin, xmax, ymin, ymax = box
        ax.add_patch(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin,
                               fill=False, edgecolor="tab:red", linestyle="--",
                               label="安全区域边界" if index == 0 else None))
        ax.add_patch(Rectangle((obstacle["xmin"], obstacle["ymin"]),
                               obstacle["xmax"] - obstacle["xmin"],
                               obstacle["ymax"] - obstacle["ymin"],
                               color="tab:red", alpha=0.3,
                               label="障碍物" if index == 0 else None))
    ax.plot(trajectory["x"], trajectory["y"], color="blue", linewidth=2,
            label="实际轨迹（水平/竖直）", zorder=3)
    ax.scatter(*mission["start"], s=100, marker="o", color="tab:green", label="起点", zorder=4)
    ax.scatter(*mission["goal"], s=100, marker="s", color="blue", label="终点", zorder=4)
    ax.set(xlim=(0, width), ylim=(0, height), xlabel="X 坐标 (m)", ylabel="Y 坐标 (m)",
           title="无人机二维地图：直角运动与简单避障")
    ax.grid(True)
    ax.legend()
    # X/Y使用相同单位比例，地图上的一米横向距离和一米纵向距离看起来一样长。
    ax.set_aspect("equal", adjustable="box")
    save_path = Path(save_path)
    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=200, bbox_inches="tight")
    plt.show()
    # 释放图片对象，避免批量生成地图时一直占用内存。
    plt.close(fig)


# 直接运行本文件时：读任务→生成轨迹→保存CSV→绘图→打印；被import时只提供函数。
if __name__ == "__main__":
    mission = load_mission()
    trajectory = simulate_motion(mission)
    data_dir = PROJECT_ROOT / "data" / "simulated_observations"
    figure_dir = PROJECT_ROOT / "figures"
    data_dir.mkdir(parents=True, exist_ok=True)
    figure_dir.mkdir(parents=True, exist_ok=True)
    csv_path = data_dir / "week02_ground_truth_czl.csv"
    figure_path = figure_dir / "week02_ground_truth_czl.png"
    trajectory.to_csv(csv_path, index=False, encoding="utf-8-sig")
    plot_trajectory(mission, trajectory, figure_path)
    print("二维模拟完成！")
    print("轨迹点数量：", len(trajectory))
    print("CSV保存位置：", csv_path)
    print("图片保存位置：", figure_path)
    print("\n前5行轨迹数据：\n", trajectory.head())
    print("\n最后5行轨迹数据：\n", trajectory.tail())
