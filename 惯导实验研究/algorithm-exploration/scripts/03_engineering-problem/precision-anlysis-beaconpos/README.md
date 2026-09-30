# 潜标位置精度分析

本目录研究长时间尺度下潜标真实位置变化及位置不确定度对距离辅助惯导、
一次 RTS 和二次 RTS 的影响。仿真与实测数据采用同一套四阶段入口。

## 文件与执行顺序

1. `step0_generate_time_varying_beacon_data.m`：生成移动潜标真值、固定名义
   坐标和三路理想距离数据；
2. `step1_run_time_varying_beacon_navigation.m`：运行无补偿导航，或选择真实
   时变潜标坐标作为理想参考；
3. `step2_run_time_varying_beacon_uncertainty_navigation.m`：把潜标位置不确定度
   等效加入距离量测方差后运行导航；
4. `step3_run_time_varying_beacon_consider_navigation.m`：使用三潜标统一堆叠的
   Schmidt/Consider 方法保留潜标误差与 INS 的交叉协方差；
5. `analyze_time_varying_beacon_navigation_results.m`：忽略已有分析文件，直接
   从 truth 和 NAV 重新比较未补偿、真实潜标位置与位置不确定度补偿结果。

辅助文件：

- `resolve_beacon_position_dataset.m`：统一解析仿真/实测数据集、真值文件和
  导航配置；
- `function/`：Schmidt/Consider 初始化、时间传播、距离更新、深度更新和
  T0/T5 回归检查函数；
- `prework/beacon_anlysis.m`：系泊绳长、深度和倾角误差向水平圆环约束传播；
- `prework/beacon_drift.m`：Release–Beacon 三维几何与漂移示意。

## 数据集目录

```text
data/inertial-experiment/algorithm-exploration/
├─ simulation/<dataset_id>/
│  ├─ input/
│  └─ output/
└─ experiment/<dataset_id>/
   ├─ input/
   └─ output/
```

每个 `input` 至少包含：

- `IMU_120.txt`；
- `range1.txt`、`range2.txt`、`range3.txt`；
- `truth.txt` 或 `truth.nav`。

实测数据默认继承 `ProcessConfig_exper` 的初始状态与传感器参数。新实测
数据集若初始化不同，应在其 `input/navigation-config.mat` 中保存结构体
`navigation_config` 或 `cfg_overrides`，至少覆盖 `starttime`、`initpos`、
`initvel` 和 `initatt`。

## step0：生成时变潜标数据

首先选择数据集和时段：

```matlab
data_source = "experiment";     % 或 "simulation"
dataset_id = 'case-07';
start_time_s = [];              % []：首个公共range时刻
generation_duration_s = 24*3600;
```

如果数据集短于请求时长，脚本会明确发出警告，在源数据终点截断，并将
`requested_duration_s`、`requested_end_time_s` 和
`duration_was_truncated` 写入 `generation-context.mat`。

潜标活动范围支持圆形或圆环：

```matlab
motion_region_mode = 'circle';
activity_radius_m = 61;

% 或
motion_region_mode = 'annulus';
annulus_inner_radius_m = 40;
annulus_outer_radius_m = 200;
```

主要参数：

- `initial_measurement_error_max_m`：固定名义坐标相对真实初始位置的最大
  水平误差；
- `velocity_std_mps`、`maximum_speed_mps`、
  `velocity_correlation_time_s`：潜标随机运动参数；
- `beacon_initial_position_std_m`、`beacon_24h_position_std_m`、
  `beacon_uncertainty_horizon_s`：step2 使用的位置不确定度模型；
- `overwrite_existing`：是否覆盖同名工况。

输出位于：

```text
<dataset>/input/<study_id>/
├─ range1.txt
├─ range2.txt
├─ range3.txt
├─ beacon1-position-truth.txt
├─ beacon2-position-truth.txt
├─ beacon3-position-truth.txt
└─ generation-context.mat
```

生成距离采用与 `myRangeUpdate` 相同的“潜标处局部曲率”水平距离模型，
确保 `beacon_position_source="truth"` 时不存在由两套坐标投影公式造成的
额外量测偏差。

## step1：无补偿与理想参考导航

step1 默认运行无补偿导航：

```matlab
beacon_position_source = "fixed-initial";
```

此时距离由真实移动潜标产生，但滤波器始终使用 step0 生成的固定名义
坐标。若要获得理想参考结果，可改为：

```matlab
beacon_position_source = "truth";
```

两种结果分别写入 `uncompensated-*` 和 `truth-beacon-reference-*` 目录。

### 初始化时刻规则

导航必须从 `cfg.starttime` 对应的初始位置、速度和姿态开始。首个测距可以
晚于初始化时刻，不能用首个测距时刻推迟导航起点。

`start_time_s=[]` 时自动使用配置初始化时刻。若显式设置的 `start_time_s`
与 `cfg.starttime` 不一致，脚本会停止并提示同步更新
`navigation-config.mat` 中的初始状态，避免在错误时刻套用旧初值。

导航终点由 IMU、真值和 `duration_s` 决定，不要求最后时刻必须有测距。

## step2：位置不确定度补偿

默认从 step0 的 `generation-context.mat` 读取不确定度模型：

```matlab
use_generation_uncertainty_model = true;
```

也可改为 `false`，通过以下参数手动设置：

```matlab
manual_beacon_initial_std_m = 5;
manual_beacon_24h_std_m = 60;
manual_uncertainty_horizon_s = 24*3600;
```

潜标不确定度龄期从 step0 的初始潜标测量时刻开始累计，不会因截取导航
区间而重新归零。每次测距的潜标编号、龄期、潜标标准差和有效距离标准差
保存在 `beacon-uncertainty-history.mat`。

## step3：Schmidt/Consider 不确定度融合

step3 不改变 INS 机械编排和 15 维误差状态，也不在线估计潜标坐标。三个
潜标的水平位置误差统一作为 6 维 consider 参数：

```text
Pxx : 15×15  INS 主动状态协方差
Pxb : 15×6   INS 与三个潜标水平位置误差的交叉协方差
Pbb : 6×6    三个潜标水平位置误差协方差
```

`Pxb` 在每个 IMU 协方差传播步同步左乘状态转移矩阵；`Pbb` 按 step0 的
不确定度增长模型随绝对龄期增加并设置上限。距离更新只修正 INS 主动状态，
不收缩 `Pbb`。潜标位置协方差已经显式进入创新协方差，因此测距噪声 `R`
中不会再次加入同一项，避免双重计数。

仿真示例：

```matlab
step3_run_time_varying_beacon_consider_navigation( ...
    'DataSource', 'simulation', ...
    'DatasetId', 'case-00', ...
    'ActivityRadiusM', 61, ...
    'DurationS', []);
```

实测示例：

```matlab
step3_run_time_varying_beacon_consider_navigation( ...
    'DataSource', 'experiment', ...
    'DatasetId', 'case-06', ...
    'ActivityRadiusM', 61, ...
    'DurationS', []);
```

结果写入 `<study_id>/consider-rad/`，包括三种 NAV、
`schmidt-consider-history.csv/.mat` 和 `navigation-context.mat`。日志记录
创新、创新方差、潜标 LOS 方差、交叉协方差项、NIS、`Pxb` 范数以及量测
前后 `Pbb` 变化量。

每次运行默认执行两项回归检查：

- T0：`Pbb=0`、`Pxb=0` 时严格退化为原普通 ES-EKF；
- T5：`Pxb=0` 时第一次更新与原 `R_eff` 方法一致，随后 `Pxb` 应变为非零，
  而 `Pbb` 在量测更新前后保持不变。

当前 RTS 继续使用已包含 Consider 影响的 `Pxx`，尚未把潜标参数块扩展进
平滑器。因此前向 EKF 是第一阶段最严格的验证对象，RTS 结果作为工程对照。

## 统一误差分析

直接运行：

```matlab
analyze_time_varying_beacon_navigation_results
```

默认扫描仿真 `case-00/05/06` 和实测 `case-06/07/08/09` 下已经存在的
`beacon-position-*` 导航结果，使用 `calc_radial_error_gjb` 按共同时间段
重新计算前向 EKF、单次 RTS 和双次 RTS 的误差，并自动纳入已有的
`consider-rad`。旧的 CSV、Excel 和图片不会作为分析输入。

也可以只运行指定数据集或算法：

```matlab
analyze_time_varying_beacon_navigation_results( ...
    'DatasetFilter', "simulation/case-05", ...
    'AlgorithmFilter', "rts-double");
```

每个工况的结果写入：

```text
<dataset>/output/figures-tables/<study_id>/
├─ navigation-error-forward-ekf.png/.fig
├─ navigation-error-rts-single.png/.fig
├─ navigation-error-rts-double.png/.fig
├─ navigation-error-<algorithm>-statistics.csv/.xlsx
├─ navigation-error-summary.csv/.xlsx
└─ navigation-error-analysis-report.txt
```

汇总表同时给出 RMSE、均值、中位数、P95、最大误差、CEP50、GJB 位置误差率、
相对未补偿的 RMSE 改善率以及相对真实潜标位置的 RMSE 差距。某种处理方式
缺少 NAV 文件时，脚本分析其余已完成结果，并在报告中记录缺失项。

## experiment/case-07 当前数据范围

该实测数据的三路 range 公共时段为 `122233.57~126853.57 s`，实际只有
`4620 s`（约 1.28 h），不能产生真实的 24 h 实测实验。

当 `range_interval_s=420` 时，完整数据内只有 11 次测距更新；导航仍从
配置时刻 `122235.00 s` 开始，首个测距在 `122652.57 s` 到达。

## 后续建议

1. 将 step1、step2 重复的 EKF/RTS 主循环提取为公共函数；
2. 加入真实长时实测数据及独立 `navigation-config.mat`；
3. 将 step3 的各向同性水平 `Pbb` 扩展为具有方向性的 2×2 协方差；
4. 为仿真增加 NEES 与 3σ 覆盖率，为实测增加 NIS 门限和异常距离统计；
5. 将潜标参数块纳入严格的 Schmidt/Consider RTS 平滑；
6. 建立短小回归数据，自动检查初始化时刻、测距轮换、时间裁剪和丢包。

## Step3 NIS robust range update

`step3_run_time_varying_beacon_consider_navigation` now supports an optional
scalar-NIS gate for the acoustic range row:

- `EnableNisRobust=false` keeps the original Schmidt/Consider result in
  `consider-rad`.
- `EnableNisRobust=true` writes an independent result to `consider-nis-rad`.
- `NisSoftThreshold` defaults to `3.841` (chi-square, 1 DoF, 95%). Values
  above it inflate only the independent range-noise variance.
- `NisRejectThreshold` defaults to `10.828` (chi-square, 1 DoF, 99.9%).
  With `EnableNisRejection=true`, values above it reject the range row while
  retaining the depth update. Rejection is disabled by default for sparse
  acoustic updates.
- `NisMinimumWeight` defaults to `0.10`, so soft downweighting cannot remove
  more than 90% of the independent range information.

The history CSV records raw/effective NIS, information weight, and action
code (`0=accepted`, `1=downweighted`, `2=range rejected`). The unified
analysis script compares `compensated-rad`, `consider-rad`, and
`consider-nis-rad` without overwriting earlier navigation files.
