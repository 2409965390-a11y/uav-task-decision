# 第4周阶段报告：在线状态条件决策

日期：2026-10-14。版本：baseline_v1 本地候选。

## 当前要求核对

第2周：任务配置、横竖直角运动、障碍安全距离、CSV和地图图已具备，并通过修改地图与目标的测试。
第3周：使用当前横竖Ground Truth重新生成观测、估计和可靠性输出，并完成噪声/缺失参数对照。
第4周：在线决策、真实动作执行、20次配对实验及独立成功/失败案例已完成。

## 输入与方法

输入为 `configs/mission_bridge_czl.json`、`configs/observation_noise.yaml`、`configs/decision_thresholds.yaml`，实验配置为 `configs/experiment_week04.yaml`。
位置单位m，时间s，速度m/s；这是本候选版本的统一约定。
策略A按路线控制，无论可靠性高低均接受有效观测并前进；策略B使用更新前的观测预测残差和连续缺失次数评分。
在线估计初始化使用配置起点，以已执行指令预测；评分和动作不读取真实位置、真值误差或人为异常标签。
真实位置仅用于产生模拟测量、环境碰撞检测和最后评价。

继续、减速、主动观测、悬停和返航实际改变位置或等待时间；返航从在线估计生成到起点的路径。
没有实现REPLAN动作，因此重新规划次数为0；没有仅输出重新规划标签来冒充路径生成。
这里的固定路线仍依赖估计位置判断转弯，噪声可能使其过早拐弯或误判到达；不是永不受传感器影响的理想开环点模型。

## 公平对照与结果

A/B每次使用同一地图、参数、随机种子和按仿真时刻固定的噪声、缺失及偏移序列；策略B主动观测用预生成的第二份同条件样本。
共20个配对种子、40次策略执行。种子为100至119。
运行时间列来自实际计时，重复运行会有小幅变化。

| strategy | run_count | success_rate | collision_count | path_length | execution_time | hold_time | hold_count | replan_count | runtime_seconds |
|---|---|---|---|---|---|---|---|---|---|
| A | 20 | 0.0000 | 0 | 112.9055 | 114.7000 | 0.0000 | 0.0000 | 0.0000 | 0.0048 |
| B | 20 | 0.2000 | 0 | 84.6130 | 145.9000 | 42.4500 | 23.6500 | 0.0000 | 0.0048 |

均值包含成功、返航和失败运行。碰撞后终止，因此较短执行时间或路径不一定表示策略更好。原地图较开阔，随机实验不能单独证明复杂地图下的安全优势。
任务成功要求真实结束位置到目标不超过0.5m；仅由估计位置判断到达仍可能出现`false_arrival`。
停留时间包含HOLD及OBSERVE，每个停留步增加dt；停留次数按连续停留段计数。
随机实验失败原因计数：{('A', 'false_arrival'): 20, ('B', 'false_arrival'): 6, ('B', 'return_position_error'): 7, ('B', 'safe_return'): 3}。

## 成功和失败案例

案例是否得到A失败/B成功：True。案例与随机20次的结果单独保存，不能合并提高成功率。

| scenario | strategy | task_completed | collision_count | hold_time | returned_to_start | failure_reason |
|---|---|---|---|---|---|---|
| transient_bias | A | False | 1 | 0.0000 | False | collision |
| transient_bias | B | True | 0 | 4.0000 | False |  |
| persistent_bias | A | False | 1 | 0.0000 | False | collision |
| persistent_bias | B | False | 0 | 10.0000 | True | safe_return |

短时偏移案例：第5至8秒给Y测量加入70m偏移，故意模拟严重定位跳变。A可能错误地判断纵向段完成，提前横向转入起点附近障碍；B根据大预测残差拒绝异常，停止并等到观测恢复。
持续偏移案例：同样的跳变持续200步。B持续无法恢复时执行真实返航，任务未完成；安全返回仍记为任务失败。此例说明保守规则也不能保证任务成功。
极端70m偏移是人为压力测试，不能作为普通GPS噪声的典型强度。

## 输出与复现

- `data/results/week04_decision_log_czl.csv`：逐时刻状态、观测、可靠性、实际速度和动作；
- `data/results/week04_runs_czl.csv`：每次完整指标及失败原因；
- `data/results/week04_metrics_czl.csv`：策略汇总；
- `data/results/week04_case_metrics_czl.csv` 和各案例日志；
- `figures/week04_strategy_comparison_czl.png`、`week04_success_case_czl.png`、`week04_failure_case_czl.png`。

```powershell
python src/run_pipeline.py --stage all
python -m unittest discover -s tests -v
```

## 局限与下一步

当前是已知地图、理想指令执行、横竖二维点模型；不包含风扰、动态障碍、加速度或真实飞行系统。
在线评分会漏掉缓慢漂移，不是校准后的概率。OBSERVE的额外采样是简化模型，返航也可能因为已有估计偏差失败。
下一步最值得补充运动模型误差与缓慢偏移实验，再评估多种阈值和碰撞安全边界，不预设策略B一定更优。

## 第三次组会与共同版本状态

已整理统一运行入口、配置、CSV、单位、来源说明和可复现本地压缩包。
目前只有本人的项目；另外两位同学接口整合、三台电脑独立复现、1—3分钟演示视频、本人逐段解释、钉钉上传及教师审核尚待完成。
该包标注为baseline_v1本地候选，不能声称已经是经全组复现的正式共同版本。未替用户提交钉钉或推送Git。
正式下一周安排仍以会后经教师审核的任务单为准；本次按用户授权先完成可运行候选和缺失基础模块。

## 辅助工具使用

由Codex辅助核对任务、编写和验证代码、生成实验记录及此报告。所有数值来自本机实际执行。
个人理解、现场独立操作及口头说明需由本人完成，不能由自动测试代替。
