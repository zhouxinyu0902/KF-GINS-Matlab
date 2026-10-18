# 第三次车载试验数据处理与导航分析

本目录按“数据预处理—四步计算—结果评价”组织。三个批次及有效时长统一由
`experiment03_dataset_paths.m` 管理：

| 编号 | 数据目录 | 配置时长 |
| --- | --- | ---: |
| 1 | `run-0817` | 11000 s |
| 2 | `run-0818` | 13000 s |
| 3 | `run-0818-noon` | 18000 s |

## 目录与准备文件

- `data_process/`：原始文件读取、时间构造、IMU补帧、标准输入导出及数据检查。
- `plot/`：纯惯导和正式导航的结果评价、统计及绘图。
- `common/`：Step 2/4使用的单批计算核心和公共格式转换函数，不需要手动运行。
- `back/`：旧入口和不再参与当前流程的脚本，仅供历史核对。
- `experiment03_dataset_paths.m`：数据集路径和各批有效时长。
- `setup_experiment03.m`：添加依赖路径并检查输入环境。
- `create_experiment03_config.m`：从路径和 `initial_state.yaml` 构造导航配置。

原始数据需要重新解析时，先运行：

```matlab
setup_experiment03(1, 'raw');
read_experiment03_raw_data(1:3)
process_experiment03_data(1:3, false)
```

`data_process` 输出的主要标准输入包括 `imu_raw.txt`、`imu_120.txt`、
`pva_120.txt`、`pva_830.txt`、`std_830.txt`、`range.txt` 和
`depth_raw.txt`。

## 四步计算流程

### Step 1：构造真值

```matlab
step1_build_dataset_truth_03(1:3)
```

使用120 IMU递推，并仅用830三维位置及其标准差更新，生成 `truth.nav`。
830速度和姿态不参与更新。IMU或初始状态变化后，需要重新执行此步骤。

### Step 2：运行纯惯导

```matlab
step2_run_pureins_03(1:3, ["rad", "m"])
```

只使用IMU和初始状态，分别输出两种位置误差状态单位下的纯惯导结果。

### Step 3：构造评估距离

```matlab
step3_build_dataset_range_03(1:3, false)
```

保留 `range.txt` 的测距时刻和信标位置，分别利用 `pva_120.txt` 和
`pva_830.txt` 重算第3列，生成 `range_120.txt` 与 `range_830.txt`。
这两个文件仅用于评估，正式导航仍使用原始 `range.txt`。

### Step 4：运行正式导航

```matlab
step4_run_navigation_03(1:3, ["rad", "m"])
```

运行前向EKF、一次RTS和二次RTS，只生成导航结果，不读取真值，也不会
重复运行纯惯导。

## 结果评价

评价脚本统一放在 `plot/`：

```matlab
setup_experiment03(1, 'navigation');
plot_all_pureins_03(1:3, ["rad", "m"])
plot_all_error_summary_03(1:3, ["rad", "m"])
```

汇总表和图片输出到 `data/experiment-data/experiment-03/summary`。

## back归档

下列旧名称已移入 `back/`，不再加入正常MATLAB路径，仅供核对历史调用：

- `build_dataset_truth_03.m`、`build_experiment03_truth.m` → Step 1；
- `build_dataset_range_03.m` → Step 3；
- `run_all_experiment03_navigation.m` → Step 2 + Step 4；
- `Config.m` → `create_experiment03_config.m`；
- `setup_all_real_data_preprocessing.m` → `setup_experiment03.m`；
- `raw_data_read.m`、`process_data_1.m` → 对应规范预处理入口；
- `run_all_experiment03_datasets.m` → 旧的一键批跑入口。

当前流程不依赖 `back/` 中的任何文件。

## 主要文件约定

- `input/imu_raw.txt`：UTC和坐标系已转换、尚未补帧的120 IMU。
- `input/imu_120.txt`：短时掉帧修复后的规则100 Hz IMU。
- `input/truth.nav`：Step 1生成的参考轨迹。
- `input/range.txt`：原始实测距离，供Step 4导航使用。
- `input/range_120.txt`、`range_830.txt`：Step 3生成的评估距离。
- `output/rad`、`output/m`：纯惯导、EKF和RTS结果。
- `output/artifacts`：单批诊断图表和统计文件。
- `summary`：跨批次评价结果。
