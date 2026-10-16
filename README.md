# 无人机任务仿真：第2—4周基础训练与多地图扩展

版本：week04-2026-10-16-v1，整理日期：2026-10-16。
基础算法沿用baseline_v1个人候选，新增多地图展示与详细中文注释。
本包整理本人的第2—4周流程。未整合另外两位同学的项目，也未声称完成全组三台电脑复现或教师审核。

## 环境与安装

导师建议Python 3.11；本机实际验证为Python 3.12.10，Python 3.11尚未实测。项目不依赖某台电脑的绝对路径。

```powershell
python -m pip install -r requirements.txt
```

依赖为NumPy、Pandas、Matplotlib和PyYAML。测试采用标准库unittest；也能被pytest发现。
`requirements.txt`给出依赖版本范围，安装器根据Python版本选择兼容版本。
本机精确依赖另存`requirements.lock.txt`，该锁定组合要求Python 3.12或以上；使用同一Python 3.12.10环境复现时可以运行`python -m pip install -r requirements.lock.txt`。
Python版本需与全组及老师对齐，不能把本机测试说成已完成Python 3.11验证。
Matplotlib图中中文字体优先Microsoft YaHei或SimHei；没有中文字体时需安装字体，计算与CSV不受影响。

## 一次运行全部流程

在本项目根目录运行：

```powershell
python src/run_pipeline.py --stage all
python -m unittest discover -s tests -v
```

默认只保存图片，不弹出窗口。`--show`可显示第2、3周交互图。
也可以单独运行`--stage week02`、`--stage week03`、`--stage week04`。
单独运行第3周需要先有当前任务的第2周Ground Truth。

旧运行方式保留：

```powershell
python src/simulator_2d.py
python src/observation_simulator.py
python src/state_estimator.py
python src/evaluate_runs.py
```

重跑会更新本包的对应CSV、图和报告。原实验报告及旧数据另外归档，不能用旧数值解释新轨迹。

## 输入与目录

- `configs/mission_bridge_czl.json`：地图大小、起点、终点、速度、安全距离与矩形障碍；
- `configs/observation_noise.yaml`：噪声、缺失率、异常区间和随机种子；
- `configs/decision_thresholds.yaml`：继续、减速、观测/悬停、返航阈值；
- `configs/experiment_week04.yaml`：至少20个配对种子、时间步、最长执行步数及独立压力案例；
- `src/`：任务读取、模拟器、观测、离线估计、在线估计、决策、评价与统一入口；
- `tests/`：运动、碰撞、在线信息限制及实际动作的测试；
- `data/simulated_observations/`：第2、3周数据；`data/results/`：实验指标与决策日志；
- `figures/`：各阶段结果；`reports/`：基于本次实际运行的报告。

`czl`为现有文件名后缀，对应导师任务卡中的“姓名”占位，不修改既有接口。

## 坐标、单位与数据字段

统一约定：二维笛卡尔坐标，原点在地图左下，X向右、Y向上；长度m，时间s，速度m/s。
无人机是二维点模型。`safety_distance`按矩形向四周扩张，角部比欧氏距离更保守。
每步只改变X或Y之一，转弯为90度；转角前可缩短一步，保留实际拐点，禁止斜线切角。

第2周CSV字段：`time,x,y,vx,vy,target_id,task_progress`。
第0行是真实起点；`vx/vy`是本行到下一行的位移除以dt；终点速度为0。
`target_id=0`表示唯一最终目标。进度为已走规划路线长度比例，到最终目标时为1。

第3周字段继续使用原格式：

- 观测：`time,true_x,true_y,observed_x,observed_y,is_missing,is_outlier`；
- 估计：`time,estimated_x,estimated_y,position_error,estimate_error`；
- 可靠性：`time,reliability_score,missing_count,residual,is_low_reliability`。

第3周可靠性是包含真值误差的离线验证分数，不能用作第4周的在线决策输入。
第4周在线模块单独从当前观测、历史状态、已执行指令计算更新前残差与连续缺失，不读取真值或异常标签。

第4周日志包含任务卡要求的`run_id,time,x,y,reliability_score,action,collision,task_completed`，
同时保留策略、观测、估计、实际速度、残差和安全区域违反信息。
在线评分与旧离线评分不混用。`OBSERVE`额外样本评分单列，决策分数保持为动作选择时的分数。

## 参数修改与公平实验

修改任务JSON后先重跑全部流程，确保第3周数据对应新任务。
修改噪声或缺失率可重跑第3周；修改可靠性阈值可重跑第4周。
第4周A/B使用同一地图、噪声配置、对应种子和按时间生成的干扰序列；20组随机实验与极端异常案例分别统计。
命令真正改变模拟运动或停留时间；返航真正生成到起点的路线，没有仅输出REPLAN标签。

## 输出与已知问题

主要输出采用原`week02_*_czl`、`week03_*_czl`及新增`week04_*_czl`命名。
完整文件对应关系和实际指标见`reports/week04_stage_report_czl.md`。

模型不包含风扰、动态障碍、加速度和转弯半径。在线残差评分可能漏掉缓慢漂移。
第3周指数平滑可能增加误差；第4周考虑可靠性的B也可能返航、超时或误判到达，不能保证更优。
两种策略的失败和不利结果均保留。真实位置只供环境与评价使用。

## 来源、版本与待办

原模块来源：用户本人的uav-task-decision项目，原仓库HEAD为66eb9df20e36732e491c0995a23aa9d30b4949fe。
该HEAD是改动前基线，不是本候选改动的提交号；当前候选用日期、版本和文件SHA256清单识别。
Codex协助修改二维运动、补齐在线基线与评价入口、运行测试和整理报告；旧接口与文件名保留。
本包包含`baseline_manifest.json`记录文件散列、来源与实际验证环境。

最新第三次组会拟议共同版本还需：另外两位同学项目、接口对齐、三台电脑各自复现、本人说明、演示视频及教师审核。
这些尚未完成。正式任务与截止时间以会后经教师审核的安排为准。
钉钉材料由本人提交；尚未完成全组复现或教师验收。旧报告中的“未推送Git”为当时状态，以本次版本记录为准。


## 本周提交与新增内容（2026-10-16）

- Git仓库地址：https://github.com/2409965390-a11y/uav-task-decision
- 本周提交号：以本版本Git提交记录为准；钉钉提交包的README记录完整提交号。
- 运行命令：`python src/run_pipeline.py --stage all`。
- 多地图演示：`python src/show_scenarios.py`；加`--no-gif`跳过动图生成。
- 独立教学例子：`python examples/week04_learning_demo.py`，只依赖Python标准库。
- 本次复核：原项目24项测试全部通过；不改写历史实验数值，结果仍来自已有实际运行记录。
- 详细中文注释覆盖任务读取、二维规划、观测、离线/在线估计、决策、评价、入口及多地图演示。
- 当前一页周报：`reports/week04_report_czl.md`；完整阶段报告：`reports/week04_stage_report_czl.md`。
- ChatGPT使用记录：`reports/chatgpt_usage_2026-10-16.md`。

主要输出：`data/results/week04_decision_log_czl.csv`、`week04_runs_czl.csv`、`week04_metrics_czl.csv`及案例日志；
`figures/week04_strategy_comparison_czl.png`、`week04_success_case_czl.png`、`week04_failure_case_czl.png`；
多地图输出位于`data/results/scenario_suite_2026-10-15/`与`figures/scenario_suite_2026-10-15/`。

`baseline_manifest.json`是2026-10-14初始候选的历史清单，不是本次新增注释与多地图版本的完整校验清单。
旧直线模拟器`src/simulator_2d_straight.py.py`与`figures/废案/`保留为历史参考，当前入口使用`src/simulator_2d.py`。

本次仍为个人可运行版本。缺少另外两位同学的项目整合、三台电脑复现记录及本人1—3分钟运行视频。
动图只播放理想轨迹，不能替代本人运行与讲解视频。不得将个人压缩包标注为已验收的全组共同基线。
