# 第4周核心②：把观测、估计、选动作、执行运动和评价串成闭环。
# 建议先读choose_action，再读run_strategy；最后读SensorTape及三个以下划线开头的辅助函数。
# truth/next_truth属于仿真环境；动作和转弯判断读取estimator.position。真值只用于生成测量、检测碰撞与评价。
# A与B同时在异常观测处理和动作选择上不同，因此实验比较的是整个方案，不能只归因于某一个动作。

"""第4周在线闭环规则基线：固定路线A与考虑可靠性的B。

观测影响控制器判断拐点/终点的位置；继续、减速、主动观测、悬停、
返航实际改变运动。环境负责真值和碰撞评价，决策只读取在线估计。
"""

# dataclass用于创建主要装数据的类，自动生成初始化方法，省去手写四个数组的赋值。
from dataclasses import dataclass
import math
# 高精度计时器只统计程序运行耗时；它和tick*dt表示的仿真任务时间是两种时间。
from time import perf_counter

import numpy as np
import pandas as pd

from online_state import OnlineEstimator
from simulator_2d import check_collision, simulate_motion


# 以@开头的是装饰器；这里给SensorTape添加自动初始化等数据容器功能。
@dataclass
class SensorTape:
    """按仿真时刻预先生成干扰，A/B相同时刻共享相同随机数。"""
    # 四张表按仿真时刻索引：普通噪声、主动观测噪声、缺失标记、持续偏移。
    # 二维数组一行对应一个时刻，两列分别是X/Y；冒号是类型提示，不是给数组赋值。
    noise: np.ndarray
    extra_noise: np.ndarray
    missing: np.ndarray
    bias: np.ndarray

    # 类方法接收cls，也就是这个类本身；可直接调用SensorTape.create(...)创建一个干扰表对象。
    @classmethod
    def create(cls, settings, steps, seed):
        # 相同seed生成同一序列。提前生成而非随策略临时抽样，可让A/B在相同时间共享相同干扰。
        rng = np.random.default_rng(seed)
        std = float(settings["position_noise_std"])
        rate = float(settings["missing_rate"])
        if not math.isfinite(std) or std < 0 or not 0 <= rate <= 1:
            raise ValueError("传感器噪声和缺失率不合法")
        # normal生成均值0、标准差std的高斯噪声；steps+1包含初始时刻0，两列对应X与Y。
        noise = rng.normal(0, std, (steps + 1, 2))
        # 为OBSERVE预先生成第二份噪声。即使A不主动观测，也不会使B改变后续普通噪声的抽样顺序。
        extra_noise = rng.normal(0, std, (steps + 1, 2))
        # 每时刻产生[0,1)随机数，与缺失率比较得到True/False数组。True表示该时刻观测缺失。
        missing = rng.random(steps + 1) < rate
        # 默认没有系统偏移；后面只给指定异常区间填入偏移。它与每时刻独立的随机噪声不同。
        bias = np.zeros((steps + 1, 2))
        begin = max(0, int(settings.get("outlier_start", 50)))
        end = min(steps + 1, begin + max(0, int(settings.get("outlier_length", 10))))
        # 字典get在缺少该字段时取默认值；[+5std,-5std]是人为设置的异常，不是规划器知道的定位误差。
        offset = settings.get("outlier_offset", [5 * std, -5 * std])
        if len(offset) != 2 or not np.isfinite(offset).all():
            raise ValueError("异常偏移必须是两个有限数值")
        # 切片包含begin、不包含end，区间内每一行都加同一个二维偏移。
        bias[begin:end] = offset
        # 调用当前类的构造方法，将四个数组装进SensorTape对象并返回。
        return cls(noise, extra_noise, missing, bias)

    # 这是仿真传感器，所以能接触环境真值；返回值才是交给控制器的观测。
    # 它不把bias或missing的标签额外提供给估计器，避免让决策直接知道人为异常区间。
    def observe(self, truth, tick, extra=False):
        if self.missing[tick]:
            # 缺失同时作用于X/Y；主动观测在本简化模型中也共享这个时刻的缺失状态。
            return np.array([math.nan, math.nan])
        noise = self.extra_noise[tick] if extra else self.noise[tick]
        # 模拟观测 = 真实位置 + 随机噪声 + 持续偏移；A/B位置不同，观测坐标可不同，但对应时刻的干扰相同。
        return np.asarray(truth) + noise + self.bias[tick]


# 只返回动作字符串，不直接移动无人机。真正的位移在run_strategy后半段生成并执行。
def choose_action(score, low_steps, thresholds):
    """只有连续不可靠超过上限才返航；阈值完全来自配置。"""
    # 返航判断优先于其他动作；默认上限10，必须连续低可靠达到第11步才触发，不是第10步。
    if low_steps > thresholds["max_hold_steps"]:
        return "RETURN"
    # 默认0.75及以上正常前进；若前面的返航条件成立，函数已经提前return，不会走到这里。
    if score >= thresholds["continue_threshold"]:
        return "CONTINUE"
    # 默认0.50到0.75之间减速，实际减速比例由实验配置决定。
    if score >= thresholds["slow_down_threshold"]:
        return "SLOW_DOWN"
    # 默认0.30到0.50之间主动观测；分数再低则落入最后的HOLD。
    if score >= thresholds["hold_threshold"]:
        return "OBSERVE"
    return "HOLD"


# 以下划线开头表示内部辅助函数。它复用第2周的横竖路线规划，不另写一套规则。
def _route(mission, start=None, goal=None):
    # 先浅拷贝任务字典，再替换起点/终点，避免返航时把原任务中的最终目标改掉。嵌套障碍列表只读。
    request = dict(mission)
    if start is not None:
        request["start"] = list(start)
    if goal is not None:
        request["goal"] = list(goal)
    # 时间步足够大，只生成各拐角，保持与二维模拟器相同的规划规则。
    # 选一个足够大的dt，让每条横竖路线段一次就能到达拐点；这里只取规划航点，不代表实际飞行用这么大时间步。
    step_time = sum(mission["map_size"]) / mission["speed"] + 1
    # DataFrame.attrs是附带元数据，取出完整航点列表。CSV不会自动保存attrs。
    return simulate_motion(request, step_time).attrs["planned_route"]


# 输入估计位置、航点列表、当前目标下标和本步最大位移；返回二维位移以及可能更新后的目标下标。
# 控制器不知道truth，所以可能误判到达拐点；这正是定位误差能够影响实际路线的原因。
def _control(estimate, route, target_index, max_step):
    """仅沿当前路线段的轴移动。是否到达拐点由估计位置判断。"""
    while target_index < len(route):
        # 一条路线段由前后两个航点定义；target_index初始为1，因为下标0是已经所在的起点。
        origin, target = route[target_index - 1], route[target_index]
        # 条件表达式：X改变则沿X轴(axis=0)，否则沿Y轴(axis=1)。规划器保证每段只改变一个坐标。
        axis = 0 if target[0] != origin[0] else 1
        # 向右/向上为+1，向左/向下为-1。方向由规划段确定，不依赖真值。
        direction = 1.0 if target[axis] > origin[axis] else -1.0
        # 把剩余距离投影到当前前进方向：正数表示还没到，接近0或负数表示按估计已到达或越过目标。
        remaining = direction * (target[axis] - estimate[axis])
        # 1e-9是很小的数值容差，减少浮点舍入导致的无意义微小移动。
        if remaining > 1e-9:
            # 先生成[0,0]。在控制函数中还会填入当前轴的位移；在HOLD/OBSERVE分支中保持全零，才是真正停留。
            displacement = np.zeros(2)
            # 先创建[0,0]，只修改当前轴；min保证按估计不会一步跨过目标，因此运动仍是水平/竖直。
            displacement[axis] = direction * min(remaining, max_step)
            return displacement, target_index
        # 估计认为当前航点已到达，切换到下一航点。多个已越过的航点可在同一次调用里连续跳过。
        target_index += 1
    return np.zeros(2), target_index


# 环境检查从真实起点到候选终点的整段运动，返回实际接触位置和是否碰撞；这里可以使用真值。
def _impact_position(start, end, mission):
    """环境评价真实碰撞，遇到障碍时停在首次接触位置，不穿透障碍。"""
    # 物理碰撞按障碍本体和地图边界判断；侵入安全区域另记safety_violation，不能把两者混为碰撞次数。
    physical_map = dict(mission, safety_distance=0)
    if not check_collision(list(start), list(end), physical_map):
        return end, False
    lower, upper = 0.0, 1.0
    # 二分搜索首次接触的位置。_表示不使用循环序号，45次迭代足以给出很精细的线段比例。
    for _ in range(45):
        middle = (lower + upper) / 2
        # middle在0到1之间：0是start，1是end。判断从start到该点的整段是否已发生碰撞。
        point = start + middle * (end - start)
        if check_collision(list(start), point.tolist(), physical_map):
            upper = middle
        else:
            lower = middle
    return start + upper * (end - start), True


# 主函数输入地图、噪声参数、阈值、实验参数、共享干扰表和策略名；输出(log, metrics)。
# 建议按循环内的七步阅读：观测→估计→评分取值→决策→实际位移→环境执行→记录评价。
def run_strategy(mission, sensor_settings, thresholds, experiment, tape, strategy, run_id=0):
    """相同任务与传感器干扰下执行一个策略，返回逐步日志和实际指标。"""
    if strategy not in ("A", "B"):
        raise ValueError("strategy必须为A或B")
    # 从这里开始计算程序运行耗时；不用于决定无人机飞行，也不等于仿真时间。
    started = perf_counter()
    dt, max_steps = float(experiment["dt"]), int(experiment["max_steps"])
    if not math.isfinite(dt) or dt <= 0 or max_steps < 1:
        raise ValueError("实验dt和max_steps必须为正数")
    ordered = [thresholds[key] for key in ("hold_threshold", "slow_down_threshold", "continue_threshold")]
    if not 0 <= ordered[0] <= ordered[1] <= ordered[2] <= 1:
        raise ValueError("可靠性阈值必须依次递增并在0到1之间")
    if thresholds["max_hold_steps"] < 0 or not 0 < experiment["slow_speed_ratio"] <= 1:
        raise ValueError("悬停上限或减速比例不合法")
    # 环境中的真实位置初始化为配置起点。随后true状态只供传感器模拟、碰撞检测和任务评价。
    truth = np.array(mission["start"], dtype=float)
    # 控制器也知道配置起点，但后续估计只读取观测和控制指令。两者从同一起点开始不意味着可随时读取真值。
    estimator = OnlineEstimator(mission["start"], sensor_settings["position_noise_std"],
                                experiment["alpha"], thresholds["hold_threshold"])
    route, target_index = _route(mission), 1
    low_steps = 0
    # 返航状态标记。一旦为True，之后保持返航，不因某一步分数暂时恢复就重新切回去原目标。
    returning = False
    path_length, hold_time, hold_count, observation_count, safety_violations = 0.0, 0.0, 0, 0, 0
    # 用于把连续多个停留步合并为一次停留事件；例如连续HOLD五步只增加一次hold_count。
    previous_stationary = False
    completed, collision, returned = False, False, False
    # 预设失败原因是超时，后面成功、碰撞或返航时再改写，避免漏记正常退出以外的结果。
    failure_reason = "timeout"
    records = []
    # tick从0开始；tick*dt是当前仿真时刻。额外包含max_steps用于记录最终状态，不再执行下一步。
    for tick in range(max_steps + 1):
        # 第1步：环境产生本时刻观测。B若悬停，下一时刻会在停留位置继续获得测量，形成真实闭环。
        observation = tape.observe(truth, tick)
        # 第2步：更新估计并取得评分。strategy=='B'得到布尔值，用来启用异常观测拒绝。
        state = estimator.update(observation, robust=(strategy == "B"))
        score = state["reliability_score"]
        # 连续低于0.5的采样步数加1；恢复到0.5及以上就清零。它不同于连续'观测缺失'次数。
        low_steps = low_steps + 1 if score < thresholds["slow_down_threshold"] else 0
        # A不根据分数主动调整动作；B调用规则函数。A仍使用估计位置控制，所以不能保证准确执行原规划路线。
        action = "CONTINUE" if strategy == "A" else choose_action(score, low_steps, thresholds)
        if returning:
            action = "RETURN"
        # 先创建当前时刻的日志。**state把估计器返回字典的各项展开并合入row。
        # 位置记录本时刻的位置；vx/vy随后填写本行到下一行之间的实际平均速度。
        row = {"run_id": run_id, "strategy": strategy, "time": tick * dt,
               "x": float(truth[0]), "y": float(truth[1]),
               "observed_x": float(observation[0]), "observed_y": float(observation[1]),
               **state, "action": action, "collision": False, "task_completed": False,
               "vx": 0.0, "vy": 0.0, "safety_violation": False}
        # 列表保存的是row这个字典对象；后面给row补速度、动作或终止标记，列表里的同一条记录也会同步更新。
        records.append(row)
        if tick == max_steps:
            row["action"] = "END"
            break
        # 第一次触发返航时，用在线估计位置到配置起点重新生成横竖路线；不是拿真实位置偷偷纠正估计。
        if action == "RETURN" and not returning:
            try:
                route = _route(mission, estimator.position, mission["start"])
            # 若估计位置不合法或找不到返航路线，停止本次执行并记录失败，不能伪造返航成功。
            except ValueError:
                failure_reason = "return_plan_unavailable"
                row["action"] = "HOLD"
                break
            returning, target_index = True, 1
        # 主动观测会真正获取同一时刻的第二份样本，但本时间步仍停留；这是简化的主动观测模型。
        if action == "OBSERVE":
            # 主动观测真的获取第二份传感器样本，仍在本时间步停留。
            extra = tape.observe(truth, tick, extra=True)
            row["extra_observed_x"], row["extra_observed_y"] = extra.tolist()
            if np.isfinite(extra).all():
                extra_state = estimator.update(extra, robust=True)
                row["estimated_x"], row["estimated_y"] = estimator.position.tolist()
                # 额外观测的评分另存；原reliability_score保留为当时选择动作的依据，避免事后改分数使图难以解释。
                row["post_observe_score"] = extra_state["reliability_score"]
            # 同一时刻第二份样本缺失，不能再增加连续缺失的时间步数。
            # 日志reliability_score保留决定动作时的分数，额外观测分数单列。
            observation_count += 1
        # in检查动作是否属于这两个名字之一。两种动作都不移动，但OBSERVE多做了一次测量。
        stationary = action in ("HOLD", "OBSERVE")
        if stationary:
            # 先生成[0,0]。在控制函数中还会填入当前轴的位移；在HOLD/OBSERVE分支中保持全零，才是真正停留。
            displacement = np.zeros(2)
            # 位置不变，仿真时间仍前进；HOLD与OBSERVE的停留时间都计入这里。
            hold_time += dt
            # 只有从移动进入停留时才加1；True转整数为1，False为0，连续停留不会重复计次数。
            hold_count += int(not previous_stationary)
        else:
            # 当前默认减速比例0.5。正常与返航使用1.0，实际位移由speed*dt*比例及航点剩余距离共同限制。
            speed_ratio = experiment["slow_speed_ratio"] if action == "SLOW_DOWN" else 1.0
            displacement, target_index = _control(estimator.position, route, target_index,
                                                   mission["speed"] * dt * speed_ratio)
            # 控制器按估计已走完航点。接下来才由环境使用真值检查是否真的到达；两步不能混淆。
            if target_index == len(route):
                if returning:
                    returned = float(np.linalg.norm(truth - mission["start"])) <= experiment["goal_tolerance"]
                    # 即使安全返回起点，也不等于完成原任务；task_completed仍为False，便于如实统计任务成功率。
                    failure_reason = "safe_return" if returned else "return_position_error"
                else:
                    completed = float(np.linalg.norm(truth - mission["goal"])) <= experiment["goal_tolerance"]
                    # false_arrival表示控制器以为到了，但真实结束位置距目标超过容差。空字符串表示无失败原因。
                    failure_reason = "" if completed else "false_arrival"
                    row["task_completed"] = completed
                row["action"] = "END"
                break
        previous_stationary = stationary
        # 环境执行指令并检测整段物理碰撞；碰撞时将实际终点截在接触位置，避免直接穿过障碍。
        next_truth, collision = _impact_position(truth, truth + displacement, mission)
        actual_displacement = next_truth - truth
        # 未碰撞时，运动模型完全执行指令。控制器预测使用指令，不读取next_truth。
        # 预测使用发送并执行的指令，不用next_truth重新定位。若碰撞，本次立即终止，不再使用后续预测。
        estimator.predict(displacement)
        # 实际平均速度 = 实际位移/dt；tolist把二维数组转列表，然后分别赋给vx、vy。
        row["vx"], row["vy"] = (actual_displacement / dt).tolist()
        row["safety_violation"] = check_collision(truth.tolist(), next_truth.tolist(), mission)
        # 统计安全区域违反的步数，包括在区域内停留的时间步；不等于独立事件次数，更不等于物理碰撞次数。
        safety_violations += int(row["safety_violation"])
        # 将每步实际移动距离累计成路径长度；零位移的悬停不增加长度，但增加任务时间。
        path_length += float(np.linalg.norm(actual_displacement))
        truth = next_truth
        # 单独添加碰撞接触后的终止记录，让最后一行坐标是实际结束位置；时间记为该固定时间步结束时刻。
        if collision:
            # 单独记录碰撞接触时刻，确保日志最后一行是实际终止位置。
            records.append({**row, "time": (tick + 1) * dt,
                            "x": float(truth[0]), "y": float(truth[1]),
                            "observed_x": math.nan, "observed_y": math.nan,
                            "action": "END", "collision": True, "vx": 0.0, "vy": 0.0})
            failure_reason = "collision"
            break
    # 逐步字典列表转成数据表。log便于保存CSV、画时间轴以及逐行解释某次动作的原因。
    log = pd.DataFrame(records)
    # metrics只汇总本次运行的结果；evaluate_runs会把多次metrics合在一起比较策略。
    # replan_count为0，因为未实现REPLAN动作；返航路线生成不能冒充该动作的重规划次数。
    metrics = {"run_id": run_id, "strategy": strategy, "task_completed": completed,
               "collision_count": int(collision), "path_length": path_length,
               "execution_time": float(log.iloc[-1]["time"]), "hold_count": hold_count,
               "hold_time": hold_time, "observe_count": observation_count,
               "replan_count": 0, "returned_to_start": returned,
               "safety_violation_steps": safety_violations, "failure_reason": failure_reason,
               "runtime_seconds": perf_counter() - started}
    # 返回两个对象；调用者可以写log, metrics = run_strategy(...)分别接收。
    return log, metrics
