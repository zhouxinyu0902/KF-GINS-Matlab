# 专利实验：大尺度稀疏测距/惯导仿真

## 1. 专题目的

本专题用于验证大尺度平面航行条件下，稀疏单信标或多信标距离观测对惯性导航误差的约束效果。仿真构造三个固定信标、长时间航迹、带误差 IMU 和三路距离数据，再使用 15 状态 RANGE/INS EKF 进行解算与对比。

当前整理版只保留一套有效算法链路：

```text
generate_simulation_data.m
    生成轨迹、IMU、真值和三路距离
                     ↓
patent_kf_range_ins.m
    批量运行纯惯导、固定单信标和轮换三信标 RANGE/INS
                     ↓
evaluate_results.m
    新结果及历史结果对比
```

原 `patent_simu.m` 中基于 `myins/myekf1` 的旧滤波算法不再作为本专题代码保留。

## 2. 目录结构

```text
专利实验/
├─ README.md
├─ config/
│  └─ patent_Configsimu.m
├─ scripts/
│  ├─ generate_simulation_data.m
│  ├─ patent_kf_range_ins.m
│  └─ evaluate_results.m
├─ archive/pre_refactor_2026-08-11/ # 整理前代码备份，不加入运行路径
└─ output/                         # 历史结果，只用于对比，不覆盖

data/patent/
├─ input/                          # 当前仿真输入
├─ output/                         # 整理后程序的新输出
└─ legacy_input/                   # 整理前的旧输入备份
```

## 3. 运行顺序

在 MATLAB 中将仓库及所需 PSINS 路径加入搜索路径，然后从仓库任意位置运行：

```matlab
run('D:\Github\KF-GINS-Matlab\专利实验\scripts\generate_simulation_data.m')
run('D:\Github\KF-GINS-Matlab\专利实验\scripts\patent_kf_range_ins.m')
run('D:\Github\KF-GINS-Matlab\专利实验\scripts\evaluate_results.m')
```

一般情况下，已存在 `data/patent/input` 时可跳过第一步。

## 4. 文件职责

### `config/patent_Configsimu.m`

统一管理：

- 仓库、数据和输出路径；
- 处理起止时间；
- 初始位置、速度、姿态及其误差；
- IMU 随机游走、零偏及相关时间；
- 仿真信标构型、采样周期和 IMU 误差；
- 测距/深度注入噪声和滤波采用的量测标准差；
- 是否运行纯惯导、三个固定单信标和轮换三信标；
- 固定轮换顺序和一个或多个测距更新周期。

### `scripts/generate_simulation_data.m`

只负责生成数据，不再执行滤波算法。输出：

- `line-imu.nav`；
- `line-truth.nav`；
- `line-range1.nav`；
- `line-range2.nav`；
- `line-range3.nav`。

### `scripts/patent_kf_range_ins.m`

保留原有“数据读取—降采样—惯导传播—测距更新—反馈—结果输出”主循环。一次批处理默认运行：

- 一次纯惯导；
- 固定信标 B01、B02、B03 各一次；
- 按 B01→B02→B03 固定顺序轮换的三信标方案一次。

测距间隔由 `cfg.experiment.range_intervals_min` 控制，可设置为单值或数组，例如 `[1, 2, 4]`。

结果文件采用统一规则：

```text
nav-pure-ins.nav
nav-range-single-b01-dt240s.nav
nav-range-single-b02-dt240s.nav
nav-range-single-b03-dt240s.nav
nav-range-rotate-b01-b02-b03-dt240s.nav
```

### `scripts/evaluate_results.m`

集中进行轨迹和径向误差比较。新结果来自 `data/patent/output`；整理前的历史结果保留在 `专利实验/output`。

## 5. 数据格式

| 文件 | 列格式 |
| --- | --- |
| `line-imu.nav` | 时间、三轴角增量、三轴速度增量 |
| `line-truth.nav` | 占位列、时间、纬度、经度、高度、三轴速度、三轴姿态 |
| `line-range*.nav` | 时间、斜距、水平距离、信标纬度、信标经度、信标高度 |
| 输出 `.nav` | 占位列、时间、纬度、经度、高度、三轴速度、三轴姿态 |

## 6. 主要依赖

仓库内部依赖根目录 `InsMech.m`、`function/` 和 `function_zxy/`。仿真生成还依赖已配置的 PSINS 函数，例如 `glvs`、`trjsegment`、`trjsimu`、`imuerrset`、`imuadderr`、`dxyz2pos` 和 `pos2dxyz`。

## 7. 当前实验约定

- 仿真频率：100 Hz；
- 滤波处理区间：0～7200 s；
- 三个信标形成约 5 km 尺度的三角构型；
- 当前默认批处理：纯惯导、三个固定单信标、按1-2-3轮换的三信标；
- 默认测距间隔为4 min，可在配置中一次设置多个间隔；
- 仿真数据生成固定随机种子，保证 IMU 数据可复现；
- `data/patent/output` 是新程序输出；`专利实验/output` 仅作历史对比。

## 8. 保留的算法特性

为避免在目录整理阶段改变专利实验结论，当前仍保留原程序的平面航行假设和高度约束方式。姿态误差反馈等算法细节暂不在本轮目录重构中调整，后续应单独进行算法审查与验证。

## 9. 配置说明

本专题的运行参数统一放在 `config/patent_Configsimu.m`。数据生成、组合导航和结果评价均读取同一份配置，通常不需要在三个脚本中重复修改参数。

### 9.1 路径与图片导出

| 配置项 | 默认值或目录 | 说明 |
| --- | --- | --- |
| `cfg.inputfolder` | `data/patent/input` | IMU、真值和三路测距输入 |
| `cfg.outputfolder` | `data/patent/output` | 组合导航结果输出 |
| `cfg.legacyoutputfolder` | `专利实验/output` | 整理前历史结果，只用于对比 |
| `cfg.simfigurefolder` | `data/patent/output/figures/simulation` | 数据生成脚本的图片 |
| `cfg.evalfigurefolder` | `data/patent/output/figures/evaluation` | 结果评价脚本的图片 |
| `cfg.figure.save_fig` | `true` | 保存 MATLAB 可编辑的 `.fig` 文件 |
| `cfg.figure.save_png` | `true` | 保存便于查看和插入文档的 `.png` 文件 |
| `cfg.figure.png_resolution` | `300` dpi | PNG 导出分辨率 |

`generate_simulation_data.m` 默认保存以下图片：

```text
simulation-trajectory-and-beacons.fig/.png
simulation-range-b01.fig/.png
simulation-range-b02.fig/.png
simulation-range-b03.fig/.png
```

`evaluate_results.m` 按测距间隔保存以下图片；例如默认 4 min 对应 `dt240s`：

```text
evaluation-trajectory-dt240s.fig/.png
evaluation-radial-error-dt240s.fig/.png
```

若 `专利实验/output` 中存在整理前的历史结果，还会生成 `evaluation-legacy-trajectory` 和 `evaluation-legacy-radial-error` 两组图片。

### 9.2 实验组合与时间范围

| 配置项 | 当前默认值 | 说明 |
| --- | --- | --- |
| `cfg.starttime` | `0` s | 解算起始时刻 |
| `cfg.endtime` | `7200` s | 解算结束时刻 |
| `cfg.experiment.run_pure_ins` | `true` | 运行纯惯导 |
| `cfg.experiment.run_single_beacons` | `true` | 运行固定单信标方案 |
| `cfg.experiment.run_rotating_beacons` | `true` | 运行轮换信标方案 |
| `cfg.experiment.single_beacon_ids` | `[1, 2, 3]` | 分别运行 B01、B02、B03 |
| `cfg.experiment.rotation_sequence` | `[1, 2, 3]` | 轮换顺序固定为 B01→B02→B03 |
| `cfg.experiment.range_intervals_min` | `4` min | 测距更新间隔；可改为 `[1, 2, 4]` 等数组以批量运行 |
| `cfg.experiment.output_prefix` | `'nav'` | 输出文件名前缀 |
| `cfg.randomseed` | `1` | IMU 仿真随机种子，保证数据可复现 |

纯惯导与测距间隔无关，因此一次批处理中只计算一次；固定信标和轮换信标会对 `range_intervals_min` 中的每个间隔分别计算。

### 9.3 量测噪声与滤波权重

| 配置项 | 当前默认值 | 说明 |
| --- | --- | --- |
| `cfg.measurement.range_noise_std` | `2` m | 注入仿真测距量的高斯噪声标准差 |
| `cfg.measurement.depth_noise_std` | `0.2` m | 注入仿真深度量的高斯噪声标准差 |
| `cfg.filter.range_std` | `3` m | EKF 采用的测距量测标准差 |
| `cfg.filter.depth_std` | `0.4` m | EKF 采用的深度量测标准差 |

`measurement` 表示生成或注入到观测中的真实噪声，`filter` 表示滤波器对量测不确定度的设定，两者有意分开配置。

### 9.4 仿真场景与 IMU

| 配置项 | 当前默认值 | 说明 |
| --- | --- | --- |
| `cfg.sim.sample_interval` | `0.01` s | 仿真采样周期，即 100 Hz |
| `cfg.sim.beacon_origin_deg` | `[17.574, 117.7900, 0]` | 局部场景参考点 `[纬度°, 经度°, 高度 m]` |
| `cfg.sim.layout_m` | 5 个局部坐标点组成的矩阵 | 前 3 行是信标，后 2 行用于轨迹起点等场景构造；单位 m |
| `cfg.sim.rotation_deg` | `0`° | 整体局部布局绕竖直轴旋转角 |
| `cfg.sim.start_point_index` | `5` | 轨迹初始位置使用布局矩阵的第 5 个点 |
| `cfg.sim.initial_heading_deg` | `-95`° | 初始航向角 |
| `cfg.sim.imu_error.eb` | `0.027` | `imuerrset` 的陀螺常值零偏参数 |
| `cfg.sim.imu_error.db` | `15` | `imuerrset` 的加速度计常值零偏参数 |
| `cfg.sim.imu_error.web` | `0.003` | `imuerrset` 的陀螺白噪声参数 |
| `cfg.sim.imu_error.wdb` | `0.03×10^5/3600` | `imuerrset` 的加速度计白噪声参数 |

信标与轨迹的局部坐标矩阵当前为：

```matlab
[0, 0, 0;
 10, 10*sqrt(3), 0;
 20, 0, 0;
 0, 5*sqrt(3), 0;
 7, 10, 0] * 1000 / 4
```

### 9.5 初始化与滤波模型

真值首历元由 `line-truth.nav` 读取，再通过 `avperrset([0.005; 0.005; 0.03] * 60, 0.1, 1)` 注入初始姿态、速度和位置误差。按 PSINS 的参数约定，对应姿态误差 `[0.3, 0.3, 1.8]` arcmin、速度误差 `0.1` m/s、位置误差 `1` m。

滤波器当前的原始配置值如下，函数末尾会统一转换到内部 SI 单位：

| 参数组 | 当前默认值 | 配置单位 |
| --- | --- | --- |
| 初始陀螺零偏 / 加计零偏 | `[0,0,0]` / `[0,0,0]` | °/h / mGal |
| 初始陀螺比例因子 / 加计比例因子 | `[0,0,0]` / `[0,0,0]` | ppm |
| 陀螺零偏初始标准差 | `[0.005,0.005,0.005]` | °/h |
| 加计零偏初始标准差 | `[7,7,7]` | mGal |
| 陀螺 / 加计比例因子初始标准差 | `[5,5,5]` / `[10,10,10]` | ppm |
| `cfg.gyrarw` | `0.0003` | °/√h |
| `cfg.accvrw` | `1e-6` | 配置文件原始速度随机游走单位 |
| `cfg.gyrbiasstd` / `cfg.accbiasstd` | `0.005` / `7` | °/h / mGal |
| `cfg.gyrscalestd` / `cfg.accscalestd` | `5` / `10` | ppm |
| `cfg.corrtime` | `4` h | 一阶高斯—马尔可夫相关时间 |
| 杆臂与安装角 | 全零 | 当前仿真不引入杆臂和安装角误差 |

修改配置后，如果改动了轨迹、IMU 或量测生成参数，应重新运行 `generate_simulation_data.m`；如果只改滤波权重、实验组合或测距间隔，可直接重新运行 `patent_kf_range_ins.m`，最后运行 `evaluate_results.m` 更新评价图片。
