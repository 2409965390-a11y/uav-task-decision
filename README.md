# UAV Task Decision

## 1. 项目说明

本项目用于完成无人机自主任务方向第2周基础训练。

当前阶段主要实现“任务配置 → 二维模拟 → 数据保存 → 结果绘制”的基础流程。

程序能够从 JSON 文件读取任务配置，根据起点、终点和速度生成二维直线轨迹，并保存真实轨迹数据和轨迹图。

---

## 2. 当前功能

当前程序可以完成以下功能：

1. 从 JSON 文件读取无人机任务配置；
2. 获取地图大小、起点、终点、速度、安全距离和障碍物信息；
3. 检查任务配置中的必要字段是否存在；
4. 根据起点和终点生成二维直线路径；
5. 按固定速度模拟无人机运动；
6. 记录每个时间步的位置、速度和任务进度；
7. 将真实轨迹保存为 CSV 文件；
8. 绘制地图、障碍物、起点、终点和真实轨迹；
9. 修改 JSON 中的起点或终点后，不修改 Python 程序即可生成新的轨迹。

---

## 3. 项目目录

uav-task-decision/

├── configs/

│   └── mission_bridge_czl.json

├── data/

│   └── simulated_observations/

│       └── week02_ground_truth_czl.csv

├── figures/

│   └── week02_ground_truth_czl.png

├── reports/

│   └── week02_report_czl.md

├── src/

│   ├── mission_io.py

│   └── simulator_2d.py

├── tests/

├── requirements.txt

└── README.md

---

## 4. 环境说明

本项目当前使用 Python 虚拟环境运行。

主要第三方依赖包括：

- pandas
- matplotlib

安装依赖前，请先进入项目根目录。

安装命令：

python -m pip install -r requirements.txt

---

## 5. 任务配置文件

任务配置文件位置：

configs/mission_bridge_czl.json

当前任务配置主要包含以下字段：

- map_size：地图大小
- start：无人机起点
- goal：无人机终点
- speed：飞行速度
- safety_distance：安全距离
- obstacles：障碍物信息

示例：

{
  "map_size": [67, 67],
  "start": [6, 6],
  "goal": [50, 20],
  "speed": 1.0,
  "safety_distance": 1.0,
  "obstacles": [
    {
      "xmin": 8,
      "xmax": 12,
      "ymin": 7,
      "ymax": 13
    }
  ]
}

程序不会把这些任务参数写死在 Python 代码中。

如果需要修改任务，可以直接修改 JSON 文件中的参数，然后重新运行仿真程序。

---

## 6. 程序说明

### 6.1 mission_io.py

文件位置：

src/mission_io.py

主要功能：

- 获取项目根目录；
- 找到任务 JSON 文件；
- 使用 json.load() 读取任务配置；
- 检查任务配置中的必要字段；
- 将任务配置返回给后续程序使用。

需要检查的字段包括：

- map_size
- start
- goal
- speed
- safety_distance
- obstacles

如果缺少必要字段，程序会给出错误提示。

---

### 6.2 simulator_2d.py

文件位置：

src/simulator_2d.py

主要功能：

- 调用 mission_io.py 读取任务；
- 获取无人机起点、终点和速度；
- 计算起点到终点之间的直线距离；
- 将总速度分解为 X 和 Y 方向速度；
- 按固定时间计算无人机当前位置；
- 计算任务完成进度；
- 生成真实轨迹数据；
- 保存 CSV 文件；
- 绘制并保存轨迹图。

真实轨迹数据至少包含以下字段：

- time
- x
- y
- vx
- vy
- target_id
- task_progress

---

## 7. 运行方法

在项目根目录打开 PyCharm Terminal 或 PowerShell。

运行二维仿真程序：

python src/simulator_2d.py

运行完成后，程序会输出：

- 轨迹点数量；
- CSV 保存位置；
- 图片保存位置；
- 前5行轨迹数据；
- 最后5行轨迹数据。

如果程序正常运行，最后会显示退出代码为 0。

---

## 8. 输出文件

### 8.1 真实轨迹 CSV

文件位置：

data/simulated_observations/week02_ground_truth_czl.csv

CSV 中包含：

time,x,y,vx,vy,target_id,task_progress

其中：

- time：当前仿真时间；
- x：无人机当前 X 坐标；
- y：无人机当前 Y 坐标；
- vx：X 方向速度；
- vy：Y 方向速度；
- target_id：当前目标编号；
- task_progress：任务完成比例。

task_progress 的范围为 0 到 1。

0 表示任务刚开始。

1 表示已经到达目标点。

---

### 8.2 真实轨迹图

文件位置：

figures/week02_ground_truth_czl.png

图中包含：

- X、Y 坐标轴；
- Start：无人机起点；
- Goal：无人机终点；
- Ground Truth：模拟器生成的真实轨迹；
- Obstacle：任务配置中的障碍物；
- 图例；
- 网格。

---

## 9. 参数修改测试

为了验证任务配置和程序代码是否已经分离，进行了终点修改测试。

原终点：

[61, 61]

修改为：

[50, 20]

修改过程中没有更改 mission_io.py 和 simulator_2d.py。

重新运行 simulator_2d.py 后，程序自动生成了新的轨迹。

最终轨迹到达：

x = 50

y = 20

task_progress = 1.0

说明程序能够根据 JSON 配置自动生成新的任务轨迹。

---

## 10. CSV 读取验证

生成 CSV 后，使用 Pandas 进行了重新读取测试。

验证命令：

python -c "import pandas as pd; df=pd.read_csv('data/simulated_observations/week02_ground_truth_czl.csv'); print(df.head()); print(df.tail())"

CSV 可以正常读取。

读取结果中第一行对应起点：

x = 6

y = 6

task_progress = 0

最后一行对应终点：

x = 50

y = 20

task_progress = 1.0

说明生成的数据文件可以正常保存和重新使用。

---

## 11. 当前已知问题

当前程序仍然是基础版本。

目前只实现了从起点到终点的二维直线运动。

虽然任务配置中已经包含障碍物，并且程序能够在地图上绘制障碍物，但还没有实现真正地避障功能。

因此，目前生成的直线路径可能直接经过障碍物。

当前尚未实现：

- 障碍物碰撞检测；
- 自动绕障；
- A* 等路径规划算法；
- 传感器噪声模拟；
- 数据缺失；
- 异常观测；
- 状态估计；
- 可靠性分析；
- 根据可靠性进行减速、悬停或重新规划。

这些功能将在后续训练阶段继续实现。

---

## 12. 本周主要输出

任务配置文件：

configs/mission_bridge_czl.json

任务读取程序：

src/mission_io.py

二维仿真程序：

src/simulator_2d.py

真实轨迹数据：

data/simulated_observations/week02_ground_truth_czl.csv

真实轨迹图：

figures/week02_ground_truth_czl.png

第2周报告：

reports/week02_report_czl.md

---

## 13. Git 信息

Git 仓库地址：

https://github.com/2409965390-a11y/uav-task-decision

本周提交号：

1ff6a84

运行命令：

python src/simulator_2d.py

主要输出文件：

data/simulated_observations/week02_ground_truth_czl.csv

figures/week02_ground_truth_czl.png

已知问题：

当前只支持二维直线路径，尚未实现避障和路径规划。

---

## 第3周：模拟传感器误差与状态可靠性

### 本周新增功能

第3周在第2周真实轨迹基础上加入模拟传感器观测，并进行基础状态估计和可靠性分析。

当前新增功能包括：

- 给真实位置加入高斯随机噪声；
- 随机制造观测缺失；
- 制造连续异常观测；
- 使用指数平滑进行状态估计；
- 计算观测误差和估计误差；
- 计算观测 RMSE 和估计 RMSE；
- 生成可靠性分数；
- 标记低可靠时间点；
- 对不同噪声强度和缺失率进行对照实验。

### 新增配置文件

configs/observation_noise.yaml

基准参数：

position_noise_std = 0.5

missing_rate = 0.1

outlier_start = 50

outlier_length = 10

random_seed = 42

### 新增程序

src/observation_simulator.py

用于根据第2周 Ground Truth 生成带随机噪声、缺失和连续异常的模拟传感器观测。

src/state_estimator.py

用于进行指数平滑状态估计、RMSE 计算和可靠性评分。

当前指数平滑参数：

alpha = 0.8

### 第3周输出文件

带噪观测：

data/simulated_observations/week03_noisy_czl.csv

状态估计：

data/simulated_observations/week03_estimated_czl.csv

可靠性结果：

data/simulated_observations/week03_reliability_czl.csv

状态估计对比图：

figures/week03_observation_and_estimate_czl.png

可靠性评分图：

figures/week03_reliability_score_czl.png

第3周报告：

reports/week03_report_czl.md

### 基准实验结果

position_noise_std = 0.5

missing_rate = 0.10

alpha = 0.8

观测 RMSE ≈ 1.4624

估计 RMSE ≈ 1.4159

低可靠观测数量 = 17

在当前实验条件下，指数平滑后的估计 RMSE 略低于原始观测 RMSE。

### 参数对照实验

实验A：基准组

noise_std = 0.5

missing_rate = 0.10

观测 RMSE = 1.4624

估计 RMSE = 1.4159

低可靠数量 = 17

实验B：高噪声组

noise_std = 1.0

missing_rate = 0.10

观测 RMSE = 2.9249

估计 RMSE = 2.7245

低可靠数量 = 28

实验C：高缺失率组

noise_std = 0.5

missing_rate = 0.25

观测 RMSE = 1.3707

估计 RMSE = 1.7186

低可靠数量 = 31

实验表明，增大位置噪声会明显增加观测和估计误差；提高缺失率会增加低可靠状态数量，并降低状态估计效果。

### 第3周运行方法

首先生成模拟传感器观测：

python src/observation_simulator.py

然后进行状态估计和可靠性分析：

python src/state_estimator.py

### 当前已知问题

当前状态估计只使用简单指数平滑。

观测缺失时仅保持上一时刻估计，没有使用无人机运动模型进行预测。

当前可靠性评分在仿真实验中使用了 Ground Truth 计算观测误差，因此主要用于验证基础流程，无法直接作为真实无人机系统中的最终可靠性算法。

后续需要研究不依赖 Ground Truth 的状态可靠性判断方法。
## 13. Git 信息 

Git 仓库地址[https://github.com/2409965390-a11y/uav-task-decision](https://github.com/2409965390-a11y/uav-task-decision)

本周提交号： 9899adc

运行命令：
python src/observation_simulator.py
python src/state_estimator.py

主要输出文件：
data/simulated_observations/week03_noisy_czl.csv
data/simulated_observations/week03_estimated_czl.csv
data/simulated_observations/week03_reliability_czl.csv
figures/week03_observation_and_estimate_czl.png
figures/week03_reliability_score_czl.png

已知问题： 当前状态估计仅使用简单指数平滑，观测缺失时仅保持上一时刻估计；当前可靠性评分仍依赖仿真 Ground Truth，尚不能直接用于真实无人机系统。