# 独立教学文件：先沿Y轴理解预测、观测、估计、评分、动作与实际位移；不需要安装第三方库。
# 推荐阅读顺序：get_score → choose_action → simulate → 最下方运行示例。
# 本例没有障碍与碰撞，且把主动观测简化为悬停；完整项目的接口和阈值以src内模块为准。

"""第4周教学版：只沿Y轴飞行，展示在线评分、动作和运动的关系。

只依赖Python标准库，不修改完整项目。为方便学习，将主动观测简化为悬停。
无地图或碰撞模型，不作为完整项目的验收程序。
"""

import math
import random

# 用两个变量同时接住两个数；起点Y=6米，目标Y=16米，X固定且省略。
START, GOAL = 6.0, 16.0
# 每步模拟1秒，正常速度1米/秒，因此正常单步位移是1米；DT不是实际等待时间。
DT, SPEED = 1.0, 1.0
# 给模拟设置上限，避免始终悬停等情况导致无限循环。
MAX_STEPS = 80


# 输入当前观测、更新前的预测和连续缺失次数；返回(可靠性分数,更新后的缺失次数)。
def get_score(observed, predicted, missing_count):
    # None表示没有观测，与合法位置数值0不同；缺失时不能直接做减法计算残差。
    if observed is None:
        missing_count += 1
        # 第一次缺失得0.35，之后每步再减0.1，最低0.05；逗号表示返回两个值。
        return max(0.05, 0.35 - 0.1 * (missing_count - 1)), missing_count
    # 一维残差取绝对值，比较观测和预测，不比较真实位置true_y。
    residual = abs(observed - predicted)
    # exp(z)表示e的z次方；残差越大，指数越负，得分越接近0。0.5是评分尺度，不是概率。
    score = math.exp(-0.5 * (residual / 0.5) ** 2)
    # 获得有效观测就把缺失计数清零；即使分数很低也属于有观测，不能算缺失。
    return score, 0


# 动作函数只返回字符串；实际减速、停留和返航要由simulate把字符串转换为位移。
def choose_action(score, bad_steps):
    # 连续低可靠达到11步才返航；此判断放在最前面，优先于其他动作。
    if bad_steps > 10:
        return "RETURN"
    if score >= 0.75:
        return "CONTINUE"
    if score >= 0.50:
        return "SLOW_DOWN"
    return "HOLD"


# strategy选A或B；seed固定随机序列；bias_length控制从第3步开始的异常持续多久。
def simulate(strategy, seed=42, bias_length=4):
    if strategy not in ("A", "B"):
        raise ValueError("strategy必须为A或B")
    # 预先生成按时间排列的干扰，同一种子下A、B使用相同干扰。
    # 独立随机数生成器，相同种子给A/B相同的误差序列，便于公平对比。
    rng = random.Random(seed)
    # 列表推导式提前生成每时刻的高斯噪声，均值0、标准差0.1米，包含初始时刻。
    errors = [rng.gauss(0, 0.1) for _ in range(MAX_STEPS + 1)]
    for tick in range(3, min(3 + bias_length, MAX_STEPS + 1)):
        # 异常区间统一加8米，模拟突然定位偏移；默认第3、4、5、6步，共4步。
        errors[tick] += 8.0
    # 额外设置第12步观测缺失，让教学输出同时展示缺失与严重偏移两种情况。
    errors[12] = None

    # true_y属于仿真环境；estimate_y是控制器估计。初始都来自已知起点，后续各自更新。
    true_y = estimate_y = START
    # 保存上一步已执行的位移指令；初始尚未移动，所以为0。
    last_dy = 0.0
    missing_count = bad_steps = 0
    returning = False
    records = []

    for tick in range(MAX_STEPS + 1):
        # 根据上次估计和刚才执行的位移，预测本时刻位置。
        # 根据上次估计和上步指令预测现在的位置；没有读取真实位置来修正预测。
        predicted_y = estimate_y + last_dy
        # 环境知道真值；控制器只接收观测。
        # 仿真传感器用真值生成测量；控制器拿到的只有observed，不知道具体误差是多少。
        observed = None if errors[tick] is None else true_y + errors[tick]
        score, missing_count = get_score(observed, predicted_y, missing_count)

        # A接受有效观测；B拒绝低可靠观测。0.8是平滑权重。
        # 默认沿用预测；若后面接受观测，再在这个基础上做修正。
        estimate_y = predicted_y
        # and要求有观测；括号内or表示A接受所有有效观测，B只接受达到0.5的观测。
        if observed is not None and (strategy == "A" or score >= 0.5):
            # 例如预测9、观测9.1，修正后为9+0.8*0.1=9.08；权重0.8表示向观测靠近80%。
            estimate_y += 0.8 * (observed - predicted_y)

        # 低分就累积连续低可靠步数，恢复到0.5及以上就清零；不同于连续缺失次数。
        bad_steps = bad_steps + 1 if score < 0.5 else 0
        action = "CONTINUE" if strategy == "A" else choose_action(score, bad_steps)
        # 一旦返航就保持True；即使后面分数恢复，也继续返回起点。
        returning = returning or action == "RETURN"
        if returning:
            action = "RETURN"

        # 返航把目标换成起点；否则仍向终点飞行。
        target = START if returning else GOAL
        # Y增大是正向，返回起点则沿负向；实际位移要乘这个方向符号。
        direction = -1 if returning else 1
        # 沿当前前进方向计算剩余距离；按估计到达或越过目标时，该值不大于0。
        remaining = direction * (target - estimate_y)
        # 先假定本步不移动；HOLD分支保留0，才会让真实位置停留。
        dy = 0.0
        # 达到时间上限或控制器按估计认为到达时，输出END并终止；最终是否真正成功还要单独评价。
        if tick == MAX_STEPS or (action != "HOLD" and remaining <= 1e-9):
            action = "END"
        elif action != "HOLD":
            # 减速时把速度减半；继续和返航使用正常速度。
            speed = SPEED * 0.5 if action == "SLOW_DOWN" else SPEED
            # 位移=方向*速度*时间，但不超过按估计得到的剩余距离，避免一步跨过目标。
            dy = direction * min(speed * DT, remaining)

        # 位置记录当前时刻；vy记录从当前到下一步的速度。先记录再移动，便于逐行对齐动作。
        records.append({"time": tick * DT, "y": true_y, "observed_y": observed,
                        "score": score, "action": action, "vy": dy / DT})
        if action == "END":
            break

        # 真正改变仿真位置的语句。动作字符串本身不会移动无人机，只有dy决定这一步走多少。
        true_y += dy                  # 环境执行运动
        last_dy = dy                  # 下个时间步用这条指令预测，不读取真值

    # 真值仅在环境模拟和最终评价中使用，不进入可靠性公式。
    # 用环境真值做最终评价：没有返航且距终点不超过0.5米才成功。估计认为到达不一定成功。
    success = not returning and abs(true_y - GOAL) <= 0.5
    # 返回逐步日志和成功标记；调用者可分别接收，或用[1]只取成功标记。
    return records, success


if __name__ == "__main__":
    for strategy in ("A", "B"):
        records, success = simulate(strategy)
        print(f"\n策略{strategy}，任务成功：{success}")
        print("时间    真实Y   分数  动作          Y速度")
        for row in records:
            print(f"{row['time']:4.0f}  {row['y']:7.2f}  {row['score']:.2f}  "
                  f"{row['action']:12s}  {row['vy']:5.2f}")

    print("\n同一教学场景的20组配对种子：")
    for strategy in ("A", "B"):
        # 20个种子逐次模拟；True可作为1相加，False作为0，得到总成功次数。
        successes = sum(simulate(strategy, seed)[1] for seed in range(20))
        print(f"策略{strategy}：成功{successes}/20")

    # 最后延长异常持续时间，观察B能否触发返航；这是独立教学案例，不加入前面20次统计。
    records, success = simulate("B", bias_length=20)
    print(f"持续异常：B任务成功={success}，结束Y={records[-1]['y']:.2f}")
