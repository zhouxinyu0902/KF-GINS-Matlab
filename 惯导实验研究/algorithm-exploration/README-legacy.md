# 惯导实验研究：实测数据与旋转收缩方案

## 1. 合并结论

`惯导实验数据` 与 `旋转收缩方案` 可以合并为同一专题。两者研究对象都是大尺度、稀疏测距辅助惯导：前者提供实测 IMU/GNSS/测距/高度数据及 FGO、平滑处理程序，后者提供多场景仿真、前后向处理和旋转收缩轨迹校正方法。原来的 `cmp_simu_exper.m` 已经直接比较仿真和实测结果，说明两者属于同一研究链。

合并后以实测数据为主线，以仿真作为算法验证与机理对照。为避免同名 `input`、`output` 和 `func` 相互覆盖，代码与数据都按 `experiment` 和 `simulation` 分层。

本轮采用安全迁移方式：新目录中保存整理后的代码及数据副本，原始目录 `惯导实验数据`、`旋转收缩方案` 暂不删除，可用于核对和回退。

## 2. 统一目录

```text
惯导实验研究/
├─ README.md
├─ setup_inertial_experiment.m       # 加载配置和专题函数路径
├─ config/
│  ├─ experiment/                    # 实测数据配置
│  └─ simulation/                    # 仿真场景配置
├─ scripts/
│  ├─ experiment/                    # 实测数据处理与旋转收缩主链
│  ├─ simulation/                    # 仿真算法与多场景比较
│  │  └─ data-generation/            # case-00 至 case-04 参数化生成与输入图片
│  └─ evaluation/                    # 结果比较
├─ functions/
│  ├─ experiment/
│  └─ simulation/
└─ archive/exploratory/              # 自动保存稿、早期生成脚本和未命名成果

data/inertial-experiment/
├─ experiment/
│  ├─ raw/                           # 原始实测输入
│  ├─ preprocessed/                  # 加噪或预处理数据
│  ├─ reference/                     # truth.nav 等参考结果
│  ├─ output/                        # 新程序输出
│  └─ figures/                       # 新程序图片
└─ simulation/
   ├─ input/
   │  ├─ case-00/                    # 原 input_simu
   │  ├─ case-01/                    # 原 input_simu1
   │  ├─ case-02/                    # 原 input_simu2
   │  ├─ case-03/                    # 原 input_simu3
   │  └─ case-04/                    # 原 input_simu4
   ├─ output/                        # 新仿真输出
   └─ figures/
```

## 3. 代码分类

### 实测主链

| 文件 | 作用 | 当前状态 |
| --- | --- | --- |
| `dataget.m` | 从实测 PVA/GNSS 数据构造高度、测距和参考轨迹 | 已改为统一相对路径 |
| `FGO_gnss_ins.m` | GNSS/INS 与纯惯导对比 | 已改为统一相对路径 |
| `FGO_range_ins_m.m` | 测距/高度辅助 INS 与平滑，位置误差按米处理 | 已改为统一相对路径 |
| `FGO_range_ins_rad.m` | 测距/高度辅助 INS 与平滑，位置误差按弧度处理 | 已改为统一相对路径 |
| `FGO_range_ins_rad_bk.m` | 带后向与旋转收缩处理的实测程序 | 已归入实测主链 |
| `ins_range_threetoge.m` | EKF、AEKF、BRC、GTS 与旋转收缩综合实验 | 原缺少 `ProcessConfigforSemiPhy3`，现改接统一实测配置；算法框架未改 |

### 仿真验证

| 文件 | 作用 | 当前状态 |
| --- | --- | --- |
| `simulate_range_ins_forward_backward.m` | 三信标固定轮换的测距/惯导事件驱动仿真 | 默认使用 `case-01`；输出前向 AEKF、当前周期纯反向轨迹，以及延迟一周期修正的上一段轨迹 |
| `simulate_range_ins_rts_comparison.m` | 一次、二次 RTS 平滑对比仿真 | 在上述三种结果基础上，增加分段一次 RTS、桥接二次 RTS、统计表和对比图；详细说明见 `RTS平滑仿真说明.md` |
| `simulate_range_ins_rts_rotation_contraction.m` | 二次 RTS 残余桥接旋转收缩仿真 | 用二次 RTS 起点的残余桥接节点延迟修正前一段二次 RTS 轨迹；详细说明见 `二次RTS残余桥接旋转收缩说明.md` |
| `replay_ins_with_double_rts_position.m` | 二次 RTS 水平位置约束的历史 IMU 重放 | 每 1 s 将二次 RTS 水平位置作为 30 m 标准差的伪量测，高度继续采用深度计；信息时延为 840 s |
| `replay_ins_with_double_rts_position_velocity.m` | 二次 RTS 水平位置、速度联合约束的历史 IMU 重放 | 每 1 s 联合约束水平位置和速度；默认标准差为 30 m、0.3 m/s，并在 420 s 分段边界附近平滑放宽量测噪声 |
| `data-generation/generate_simulation_data.m` | 统一生成 case-00 至 case-04 的 IMU、真值和三信标距离数据 | 默认不覆盖已有数据；每次调用都会更新所选场景图片 |
| `data-generation/plot_simulation_cases.m` | 为已有仿真输入生成轨迹—信标图和三信标距离曲线 | PNG/FIG 统一保存到 `data/inertial-experiment/simulation/figures` |
| `cmp_4simu.m` | 比较 4 个仿真方向/场景 | 已改为统一数据目录 |
| `process_range_event_segment.m` | 单个相邻测距区间的纯反向惯导推算 | 反向过程不传播 KF、不进行测距更新和误差反馈，只以当前前向 AEKF 锚点为反推初值 |
| `transform_previous_backward_segment.m` | 用当前周期的反推终点延迟修正上一段红色反推轨迹 | 固定上一段红线的 `t0-840 s` 起点，将其 `t0-420 s` 终点旋转、缩放到当前反推终点 |

仿真脚本输出到 `data/inertial-experiment/simulation/output/forward-backward`：

- `range-ins-forward.nav`：测距辅助的前向滤波轨迹；
- `range-ins-backward-constrained.nav`：从当前前向 AEKF 锚点出发得到的纯反向惯导轨迹；文件名为兼容已有评估流程暂不修改；
- `range-ins-delayed-previous-geometry.nav`：在时刻 `t0` 利用当前反推终点，固定上一段红色反推轨迹的 `t0-840 s` 点，并修正其 `[t0-840 s,t0-420 s]` 区间的结果；
- `range-segment-diagnostics.csv`：各区间的起止时间、红色反推节点跳距，以及延迟几何变换的缩放比例和旋转角。

二次 RTS 约束历史 IMU 重放输出：

- `range-ins-double-rts-position-guided-replay.nav`：仅使用二次 RTS 水平位置约束的重放结果；
- `range-ins-double-rts-position-velocity-guided-replay.nav`：使用二次 RTS 水平位置、速度联合约束的重放结果；
- 同名 `-updates.csv`：逐次伪量测的接受状态、创新量和实际采用的标准差；
- 同名 `-statistics.csv` 与 `-comparison.png/.fig`：前向 EKF、二次 RTS 和约束重放结果的误差统计与对比图。

先前试验的全局连续平滑脚本及其历史结果已经移除。当前保留的后处理主线为二次 RTS、二次 RTS 加旋转收缩，以及基于二次 RTS 位置或位置与速度约束的历史 IMU 重放。

这里的 BRC-AEKF 表示“以前向 AEKF 测距更新点为区间锚点的反向轨迹重建”，并不表示在反向时间轴上再次运行 AEKF。反向过程中直接使用正向误差状态传播和测距反馈，在理论上不等同于严格的反向 Kalman 平滑，因此当前版本已经移除这部分反向滤波。

仿真主链和二次 RTS 约束重放统一采用 `new_惯导试验/all_process/all_m.m` 的高度处理方式：非测距历元不再直接覆盖名义高度，而是每个 IMU 历元执行一次解耦高度 Kalman 更新，仅反馈垂向位置误差和垂向速度误差；测距历元仍执行距离与高度联合量测。高度观测噪声及滤波标准差均为 0.4 m。公用实现位于 `functions/simulation/update_decoupled_height.m`。一次、二次 RTS 的累计状态转移矩阵同时包含惯性传播和高度量测闭环转移，保证节点协方差与状态转移一致。纯反向惯导区间不运行 Kalman 滤波，因此仍使用高度序列约束其垂向轨迹。

延迟上一段几何修正的时序为：当前测距时刻 `t0` 先得到红色纯反向区间 `[t0-420 s,t0]`，但不处理该区间；随后取上一周期已经保存的红色反推轨迹 `[t0-840 s,t0-420 s]`，固定其 `t0-840 s` 起点，并把上一段红线在 `t0-420 s` 的终点旋转、缩放到当前反向推算得到的 `t0-420 s` 终点。这里当前反推终点是修正参考点，待修正对象是上一段红色轨迹。原先直接处理当前红色区间的 GTS 双端点几何对齐已不再作为本脚本的对比输出。

第一个测距点用于建立首个真实锚点，第二个测距点到达时才处理两者之间的完整区间。首个测距点之前和最后一个测距点之后没有双端点约束，因此三类输出均保留前向结果。

### 结果评价

| 文件 | 作用 |
| --- | --- |
| `result_plot.m` | 实测 FGO、平滑、后向与旋转收缩结果比较 |
| `cmp_simu_exper.m` | 同一指标下比较实测与仿真结果 |
| `visualize_event_driven_results.m` | 比较前向 AEKF、纯反向 BRC 和延迟上一段几何修正结果，并按测距区间统计误差与改进率 |
| `compare_all_rts_methods.m` | 统一比较前向 EKF、一次/二次 RTS、2RTS+旋转收缩、2RTS+位置约束和2RTS+位置速度约束 | 截止到第11个测距点后1 s，即0–4621 s；输出轨迹、水平径向误差和统一统计表 |

`visualize_event_driven_results.m` 沿用 `calc_radial_error_gjb.m` 的逐点曲率半径误差计算口径，生成：

- `range-results-overview.png/.fig`：轨迹、水平径向误差、北向误差和东向误差总览；
- `range-segment-comparison.png/.fig`：各测距区间的 RMSE 和相对前向算法的改进率；
- `range-delayed-geometry-diagnostics.png/.fig`：上一段红色反推轨迹的延迟旋转角、尺度因子和变换前后节点间距；
- `range-result-statistics.csv`：全时段及双端点有效区间的 Max、RMSE、Mean、Median 和 P95；
- `range-segment-statistics.csv`：每个相邻测距区间的误差统计和改进率。

图形保存在 `data/inertial-experiment/simulation/output/forward-backward/figures`，统计表保存在其上一级结果目录。

## 4. 数据说明

实测数据来自原 `惯导实验数据/input`，包括：

- `IMU_120.txt`：实测 IMU；
- `pva_430.txt`、`pva_830.txt`、`pva_RS.txt`：不同设备或处理链的 PVA；
- `gps_430.txt`、`gps_830.txt`：GNSS 数据；
- `std_430.txt`、`std_830.txt`：定位标准差；
- `range1.txt` 至 `range3.txt`：三信标测距；
- `height*.txt`、`rangedata*.txt`：高度及测距派生数据。

原 `惯导实验数据/output/truth.nav` 被单独归入 `experiment/reference`，因为它是后续误差评价的参考轨迹，不应与新算法输出混放。

仿真数据按 case 编号保留，不合并同名文件。`case-00` 至 `case-04` 的 `IMU_120.txt`、`truth.txt` 和 `range*.txt` 并不是重复文件，分别代表不同轨迹/初始航向或噪声条件。

## 5. 运行方式

先初始化专题路径：

```matlab
paths = setup_inertial_experiment();
```

然后按研究目的运行相应脚本，例如：

```matlab
run(fullfile(paths.topic, 'scripts', 'experiment', 'FGO_gnss_ins.m'))
run(fullfile(paths.topic, 'scripts', 'experiment', 'FGO_range_ins_rad.m'))
run(fullfile(paths.topic, 'scripts', 'experiment', 'FGO_range_ins_rad_bk.m'))
run(fullfile(paths.topic, 'scripts', 'evaluation', 'result_plot.m'))
run(fullfile(paths.topic, 'scripts', 'simulation', ...
    'simulate_range_ins_forward_backward.m'))
run(fullfile(paths.topic, 'scripts', 'simulation', ...
    'replay_ins_with_double_rts_position_velocity.m'))
generation_dir = fullfile(paths.topic, 'scripts', 'simulation', ...
    'data-generation');
addpath(generation_dir);
plot_simulation_cases();
run(fullfile(paths.topic, 'scripts', 'evaluation', ...
    'compare_all_rts_methods.m'))
run(fullfile(paths.topic, 'scripts', 'evaluation', ...
    'visualize_event_driven_results.m'))
```

运行代码仍依赖仓库根目录中的 KF-GINS/PSINS 公共函数，例如 `Param`、`InsMech`、初始化函数、绘图函数和坐标转换函数。

## 6. 旧目录映射与历史输出

| 原位置 | 新位置 |
| --- | --- |
| `惯导实验数据/*.m` | `惯导实验研究/scripts/experiment` 或 `scripts/evaluation` |
| `惯导实验数据/func` | `惯导实验研究/functions/experiment` |
| `惯导实验数据/input` | `data/inertial-experiment/experiment/raw` |
| `惯导实验数据/input_pre` | `data/inertial-experiment/experiment/preprocessed` |
| `惯导实验数据/output/truth.nav` | `data/inertial-experiment/experiment/reference/truth.nav` |
| `旋转收缩方案/*.m` | `惯导实验研究/scripts/simulation`、`scripts/experiment` 或 `scripts/evaluation` |
| `旋转收缩方案/func` | `惯导实验研究/functions/simulation` |
| `旋转收缩方案/input_simu*` | `data/inertial-experiment/simulation/input/case-*` |

原有大体量结果暂不重复复制：

- `惯导实验数据/output`：约 0.81 GB；
- `旋转收缩方案/output`：约 1.31 GB。

它们继续作为历史结果保留在原目录。新代码统一写入 `data/inertial-experiment/**/output`。等新目录完成一次全流程复算并对比无误后，再决定是否迁移或删除旧输出与旧专题目录。

## 7. 后续整理建议

1. 先以当前实测数据跑通 `GNSS/INS → RANGE/INS → 后向/旋转收缩 → 评价`。
2. 将脚本内的 `rngstd`、`depthstd`、测距间隔 `id=420`、平滑方式等逐步移入统一配置。
3. 对 `FGO_range_ins_m.m`、`FGO_range_ins_rad.m` 和备份版本做逐行差异审查，确定唯一主版本。
4. 用历史输出对比事件驱动版本的反向区间及双端点旋转收缩结果。
5. 全流程结果与历史输出一致后，才删除原 `惯导实验数据` 和 `旋转收缩方案`。
