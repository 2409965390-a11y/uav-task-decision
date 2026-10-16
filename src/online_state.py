# 第4周核心①：维护一个在线估计器，先比较观测与预测，再决定是否融合观测。
# 本模块不接收Ground Truth、position_error或is_outlier，避免把真实系统拿不到的信息用于决策。
# 一次循环的调用顺序：update(当前观测) → 外部选择并执行动作 → predict(已执行的控制指令)。

"""用于在线决策的状态估计：只接收观测、历史估计和已执行的指令。

真实位置及异常标签只能用于仿真环境与评价，不进入此模块。
"""

import math

import numpy as np


# class把需要跨时间保留的数据和方法放在一起；self表示当前这个估计器对象。
# 例如estimator = OnlineEstimator(start)建立一个对象，后续反复调用它的update和predict。
class OnlineEstimator:
    # __init__在创建对象时执行一次。start来自任务已知起点；alpha是观测融合权重。
    # reject_threshold是接受观测的下限，与'动作何时悬停'属于不同判断，不能混成一个阈值。
    def __init__(self, start, noise_std=0.5, alpha=0.8, reject_threshold=0.3):
        if not 0 < alpha <= 1 or noise_std < 0 or not 0 <= reject_threshold <= 1:
            raise ValueError("在线估计参数不合法")
        # asarray将[x,y]转成可逐元素计算的数组；copy防止修改估计时连原起点数据一起改掉。
        # update前这里是当前时刻的预测；融合观测后是当前估计；predict后又变成下一时刻预测。
        self.position = np.asarray(start, dtype=float).copy()
        # 评分尺度随已知噪声标准差增大，且至少为0.5米。它是当前规则的参数，不是可靠性真值。
        self.noise_scale = max(2.0 * noise_std, 0.5)
        self.alpha = alpha
        self.reject_threshold = reject_threshold
        # 记录连续缺失的采样步数。有有效观测时清零；即使观测被判为异常，它仍不是'缺失'。
        self.missing_count = 0

    # observation是本时刻传感器给出的二维坐标。robust=True时会拒绝低分观测，B使用该模式；A使用False。
    def update(self, observation, robust=True):
        """先用更新前的预测计算残差，防止平滑后的估计跟着异常一起偏移。"""
        # 转换后可以直接计算observation - self.position，分别得到X、Y方向的差值。
        observation = np.asarray(observation, dtype=float)
        # isfinite对每个分量检查是否为有限数；all要求两者都有效。NaN或无穷大都会使本次观测无效。
        missing = not np.isfinite(observation).all()
        if missing:
            self.missing_count += 1
            # 缺失时无法计算观测与预测的距离，用NaN保留'无法计算'这一事实，而不是假装距离为0。
            residual = math.nan
            # 连续缺失第1/2/3步对应0.35/0.25/0.15，之后最低0.05；max限制分数下界。
            score = max(0.05, 0.35 - 0.10 * (self.missing_count - 1))
            accepted = False
        else:
            # 记录连续缺失的采样步数。有有效观测时清零；即使观测被判为异常，它仍不是'缺失'。
            self.missing_count = 0
            # 先计算更新前残差。二维norm是sqrt(dx²+dy²)，单位米；此处比较预测，不比较真值。
            residual = float(np.linalg.norm(observation - self.position))
            # 残差为0时分数为1，越偏离预测分数越接近0。math.exp(z)表示e的z次方。
            # 这个数是规则分数，不是已经校准的'定位正确概率'。缓慢漂移可能仍得到高分。
            score = math.exp(-0.5 * (residual / self.noise_scale) ** 2)
            # or表示至少一个条件成立：A关闭异常拒绝，任何有效观测都接受；B要求分数达到接受下限。
            accepted = not robust or score >= self.reject_threshold
            if accepted:
                # 等价于新估计 = alpha*观测 + (1-alpha)*预测。alpha=0.8表示沿观测方向修正差值的80%。
                # 先评分再更新，避免先被异常带偏，再误以为观测与估计很接近。
                self.position += self.alpha * (observation - self.position)
        # 返回字典便于日志记录：两个估计坐标、评分、缺失次数、残差和是否接受观测。
        # [0]取X，[1]取Y；float把NumPy标量转换成普通Python数值。
        return {"estimated_x": float(self.position[0]), "estimated_y": float(self.position[1]),
                "reliability_score": score, "missing_count": self.missing_count,
                "residual": residual, "observation_accepted": accepted}

    # displacement是刚才执行的二维位移，例如[0,1]表示向上1米，[0,0]表示停留。
    def predict(self, displacement):
        # 已执行的控制指令用于下一时刻预测，不读取实际位置或真值误差。
        # 下一时刻预测 = 当前估计 + 控制指令位移。当前模型假设未碰撞时指令被准确执行；没有风扰或执行误差模型。
        self.position += np.asarray(displacement, dtype=float)
