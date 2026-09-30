# 第三次车载试验数据处理与导航分析

本目录是第三次车载试验的唯一代码入口，覆盖原始数据解析、标准输入导出、
参考轨迹构造、纯惯导、前向 EKF、一次/二次 RTS 以及跨批次结果汇总。

## 数据集与 case07 对应关系

统一配置入口为 `experiment03_dataset_paths.m`：

| 编号 | 数据目录 | 配置时长 |
| --- | --- | ---: |
| 1 | `run-0817` | 11000 s |
| 2 | `run-0818` | 13000 s |
| 3 | `run-0818-noon` | 18000 s |

算法探索目录中的实测 `case-07` 就是这里的 `run-0817`。其核心输入
`imu_120.txt`、`pva_830.txt`、`std_830.txt`、`range.txt`、
`depth_raw.txt` 和 `truth.nav` 与 0817 数据一致。

各批次的初始位置、速度、姿态及 IMU 上机时间与 UTC 的对应关系由各自的
`initial_state.yaml` 提供。

## 目录与脚本职责

- `experiment03_dataset_paths.m`：集中管理三个批次的路径和时长。
- `setup_all_real_data_preprocessing.m`：加载依赖、检查输入并建立输出目录。
- `data_process/raw_data_read.m`：解析 830 二进制、120 IMU、测距、深度和
  AUXA 原始文件，写入 `intermediate/raw_data_*.mat`。
- `data_process/process_data_1.m`：按 YAML 对齐时间，检查数据质量并导出标准
  输入文件和诊断图表；原始导航格式IMU保存为 `imu_raw.txt`，短时掉帧
  插值修复后的规则100 Hz IMU保存为 `imu_120.txt`。
- `build_dataset_range_03.m`：构造 120/830 参考距离，仅用于数据评价。
- `build_dataset_truth_03.m`：用 120 IMU 递推和 830 三维位置更新构造
  `truth.nav`。
- `run_navigation_comparison.m`：单批导航核心，完成前向 EKF、一次 RTS、
  二次 RTS 和评价；直接运行时默认使用 `run-0817`。
- `run_all_experiment03_navigation.m`：批量运行指定数据集和 rad/m 状态单位。
- `run_all_experiment03_datasets.m`：真值构造、导航和汇总的统一入口。
- `plot_all_error_summary_03.m`、`plot_all_pureins_03.m`：跨批次汇总。
- `functions/`：真值构造、纯惯导和结果评价的内部函数。
- `data_process/+yaml/ReadYaml.m`：本专题使用的轻量 YAML 读取器。

## 完整处理顺序

```matlab
% 1. 原始数据解析
raw_data_read(1:3)

% 2. 导出标准输入，并构造 120/830 评估距离
build_dataset_range_03(1:3, false)

% 3. 120 IMU 递推 + 830 三维位置更新，生成 truth.nav
build_dataset_truth_03(1:3)

% 4. rad/m 两种单位批跑 PureINS、EKF、一次 RTS 和二次 RTS
run_all_experiment03_navigation(1:3, ["rad", "m"])

% 5. 三批结果汇总
plot_all_error_summary_03(1:3, ["rad", "m"])
plot_all_pureins_03(1:3, ["rad", "m"])
```

步骤 3～5 也可以统一运行：

```matlab
run_all_experiment03_datasets(1:3, ["rad", "m"])
```

只处理一个批次时可以传入编号或名称，例如：

```matlab
raw_data_read('run-0817')
process_data_1('run-0817', false)
run_all_experiment03_navigation('run-0817', "rad")
```

## 文件约定

- `input/range.txt`：原始实测距离，供导航使用。
- `input/range_120.txt`：用 120 位置重算的距离，仅用于评价。
- `input/range_830.txt`：用 830 位置重算的距离，仅用于评价。
- `input/depth_raw.txt`：深度/高度观测。
- `input/imu_raw.txt`：UTC和坐标系已转换、但未补帧的120 IMU。
- `input/imu_120.txt`：短于0.5 s的掉帧已按相邻角速度/比力插值修复的
  规则100 Hz IMU，供真值、Pure INS、EKF和RTS共同使用。
- `input/truth.nav`：120 IMU 递推与 830 三维位置更新得到的参考轨迹。
- `output/rad`、`output/m`：两种位置误差状态单位的导航结果。
- `output/artifacts`：单批数据诊断图表。
- `summary`：三批汇总图表。

每批IMU修复区间记录在 `output/artifacts/imu-time-repair.csv`。超过0.5 s的
间断不会自动伪造数据，处理程序会停止并要求人工检查。

真值构造只使用 830 位置及标准差，不使用 830 速度和姿态；姿态与速度来自
120 IMU 递推。纯惯导统一评价前 3600 s，并额外输出完整有效时段统计。

## 与 algorithm-exploration 的 RTS 结果为何不同

`run-0817` 与 `algorithm-exploration/experiment/case-07` 的核心输入和初始状态
相同，0817 的 26 个测距时刻也都精确落在 IMU 历元上。因此当前结果差异
不是数据、初值或测距时间对齐造成的，而是两套 RTS 实现和输出口径不同：

1. 本目录的 RTS 默认每 1 s 保存一个关键节点，在关键节点上反向递推，再把
   误差线性插值到 100 Hz 输出；算法探索脚本在每个 IMU 历元保存并递推。
2. 本目录按关键节点保存一套聚合协方差和状态转移；算法探索脚本逐历元分别
   保存量测校正协方差、一步预测协方差和状态转移矩阵。两者计算的平滑增益
   并不等价。
3. 本目录输出与 EKF 等长，并用 EKF 补齐没有完整平滑条件的尾段；算法探索
   的 RTS 文件只写到最后一个完整测距区间。
4. 本目录把测距历元的前向结果改写为量测反馈后的状态；算法探索脚本在该行
   保留反馈前状态，从下一 IMU 行开始体现反馈。内部前向轨迹在其余历元一致。

因此如果要做严格回归比较，应先统一 RTS 节点密度、校正/预测协方差缓存、
测距历元写出规则和尾段处理口径，不能只比较两个现有 `.nav` 文件的同名曲线。
