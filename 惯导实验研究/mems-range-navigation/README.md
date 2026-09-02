# MEMS/测距组合导航专题

## 专题定位

本专题由原 `MEMS_RANGE` 整理而来，保存不同等级 MEMS 与距离/方位角辅助惯导的早期实验代码。内容包括：

- 830/430 MEMS 实测数据上的距离/INS、反向推算、RTS和轨迹旋转收缩探索；
- ADIS16545（原始数据文件名为 `ADIS16465.txt`）与 i300 数据上的距离/INS试验；
- 仿真轨迹、IMU、固定/移动信标距离数据生成，以及基于距离残差的IMU增量优化；
- 已知信标位置、距离、相对方位角和载体航向条件下的二维定位探索。

这些代码多数属于探索性原型，保留了不同阶段的算法分支和调试段。当前整理以“路径可迁移、代码与数据分离、用途可追溯”为目标，没有合并不同版本的算法主体。

## 目录结构

```text
惯导实验研究/mems-range-navigation/
├─ setup_mems_range_navigation.m
├─ config/
│  ├─ experiment/       实测数据配置
│  └─ simulation/       仿真配置
├─ functions/
│  ├─ analysis/         MEMS参数分析
│  ├─ azimuth/          方位角几何、噪声和约束
│  ├─ navigation/       反向惯导、分段反推和RTS原型
│  └─ optimization/     IMU增量优化
└─ scripts/
   ├─ experiment/       830/430、ADIS16545和i300实测脚本
   ├─ simulation/       仿真数据生成与优化试验
   ├─ azimuth/          距离+方位角定位探索
   └─ evaluation/       依赖已有工作区结果的评价脚本
```

数据统一位于：

```text
data/inertial-experiment/mems-range-navigation/
├─ data_830_430_1 ... data_830_430_6
├─ data_16545
├─ data_i300
└─ data_simu
```

数据集名称暂时保留，以便与旧实验记录和输出文件对应。830/430数据仍采用各数据集内部的 `input/` 和 `output/`；仿真数据使用 `data_simu/input/` 与 `data_simu/output/`。ADIS16545和i300原始文件未进一步拆分，避免一次整理中改变大量旧脚本语义。

## 使用方式

各入口脚本会自动调用路径配置。也可以先手动运行：

```matlab
paths = setup_mems_range_navigation();
```

默认830/430实验数据集是 `data_830_430_2`，可在相应脚本的 `path = paths.dataset_830_430_2` 处切换为第1至第6组。

## 脚本功能分析

### 实测实验脚本

| 脚本 | 主要功能 | 当前判断 |
|---|---|---|
| `scripts/experiment/ins_range_MEMS_1.m` | 830/430数据上的纯惯导、EKF/AEKF距离辅助导航主原型；包含固定/移动/轮换信标选择、可选IMU增量优化、1 s反向推算、RTS和结果评价段。 | 主探索脚本，但运行与绘图仍混合；默认关闭反向与RTS分支。 |
| `scripts/experiment/dataget_truth_ADIS16545.m` | 使用ADIS IMU和GNSS位置进行GNSS/INS融合，生成 `truth_gen.nav` 参考结果。 | 已修复原脚本未定义的 `starttime`、`endtime` 和 `height`；仍需用结果与已有 `truth.nav`核对。 |
| `scripts/experiment/ins_range_ADIS16545.m` | 在ADIS16545数据上生成三信标距离并运行纯惯导/EKF/AEKF距离组合导航，保存协方差与若干RTS/反向处理缓存。 | 大型探索脚本，包含数据生成、导航和绘图，尚未拆成独立入口。 |
| `scripts/experiment/ins_range_i300.m` | i300数据上的三信标距离/INS实验，主体与ADIS版本相近。 | 原配置误指向 `dataset4/truth.nav`，现已改为本数据集的 `truth.nav`。 |

### 仿真脚本

| 脚本 | 主要功能 | 当前判断 |
|---|---|---|
| `scripts/simulation/data_get.m` | 使用PSINS轨迹段生成仿真轨迹、带误差IMU、GNSS、高度以及固定信标距离数据。 | 数据现在统一写入 `data_simu/input`；脚本包含多套注释场景，建议运行前确认当前启用轨迹段。 |
| `scripts/simulation/ins_range_imuInforOpti.m` | 对仿真数据运行纯惯导、EKF/AEKF距离组合导航；在测距点利用1 s历史IMU增量和距离残差调用 `fsolve` 优化角增量、速度增量；还包含单信标、移动信标和多信标生成函数。 | 算法验证原型，超过千行且混有数据生成和评价；需要Optimization Toolbox。 |

### 方位角专题脚本

| 脚本 | 主要功能 | 当前判断 |
|---|---|---|
| `scripts/azimuth/ins_range_azi_MEMS.m` | 在830/430数据上构造带不规则方位角噪声的固定/移动信标量测，并运行距离EKF、可选IMU优化、反向处理和评价。 | 虽然生成了方位角，第一个版本当前活动量测更新仍主要调用 `myRangeUpdate`，不能直接视为距离+方位角融合结果。 |
| `scripts/azimuth/ins_range_azi_MEMS_new.m` | 在上一版本基础上改用NED误差传播和 `myRange_azi_pos_Update`，加入按维度限制位置反馈的 `hard_constrain`，并把部分反向处理改为函数调用。 | 三个版本中最接近“距离+方位角+位置约束”试验的实现；反向分支调用接口与现有 `backward_1s(runArgs)` 尚未统一，默认关闭。 |
| `scripts/azimuth/ins_range_azi_MEMS_newnew.m` | 尝试把主循环按配置、预处理、滤波和后处理重新排版。 | 可读性较好但功能未整理完整：`meas_mode`、硬约束和RTS开关尚未真正接入，活动更新仍是距离更新；定位为重构草稿。已补回多信标分支缺失的 `seq/nnn`。 |
| `scripts/azimuth/position_sample.m` | 生成二维机动轨迹，模拟固定信标距离、相对方位角与航向噪声，并用几何反算恢复载体位置。 | 独立几何仿真，可用于检查方位角定义和误差敏感性。 |
| `scripts/azimuth/test.m` | 对 `pos2azimuth` 与 `calc_position_from_beacon` 做两个场景的正反算闭环一致性验证。 | 小型单元测试；内部保留了两个函数副本，便于独立运行。 |
| `scripts/evaluation/azi_position_cmp.m` | 使用已有 `rangedata`、`navforward` 和 `truth` 工作区变量，把距离+方位角+航向转换为位置，计算东西向及水平误差并绘图。 | 不是独立入口；必须先由导航脚本准备工作区变量。 |

## 配置文件

| 文件 | 功能 |
|---|---|
| `config/experiment/ProcessConfig_MEMS_exper.m` | 选择830或430 IMU，读取 `data_830_430_x/input`，设置初始状态、噪声和输出目录。当前默认 `type=830`。 |
| `config/experiment/ProcessConfig_16545.m` | ADIS16545/16465数据的IMU、GNSS、真值、信标距离和滤波参数配置。 |
| `config/experiment/ProcessConfig_i300.m` | i300数据及600 s试验区间配置。 |
| `config/simulation/ProcessConfig_imusimu.m` | 仿真IMU、真值、GNSS、距离和高度文件配置；已按现有 `input/output` 目录修正。 |

## 函数功能分析

| 函数/脚本片段 | 主要功能 | 注意事项 |
|---|---|---|
| `functions/azimuth/calc_position_from_beacon.m` | 根据已知信标EN坐标、距离、相对基线法线方位角和载体航向反算载体EN位置。 | 方位角与航向单位均为度。 |
| `functions/azimuth/pos2azimuth.m` | 根据载体、信标EN位置和载体航向生成相对基线法线方位角。 | 与上一个函数构成正反算对。 |
| `functions/azimuth/add_azimuth_noise_irregular.m` | 按方位角相关的非线性标准差包络加入高斯噪声，并限制最大噪声。 | 输入输出单位为度。 |
| `functions/azimuth/hard_constrain.m` | 根据速度和最大修正阈值，分别压缩滤波位置误差状态的纬度、经度修正量。 | 实质是反馈限幅，不是严格几何硬约束。 |
| `functions/analysis/analyze_mems_engineering_units.m` | 从静止IMU角增量/速度增量估计陀螺零偏、ARW、加计零偏和VRW并换算工程单位。 | 依赖正确的采样率、纬度和姿态矩阵。 |
| `functions/navigation/InsMechBackwardMEMS.m` | 从较晚状态和反向IMU增量执行反向速度、位置与姿态编排。 | 反向符号和坐标系敏感，正式使用前应做前推—反推闭环测试。 |
| `functions/navigation/backward_1s.m` | 在相邻测距区间反向推算，在上一测距点再次更新，并对分段轨迹做旋转伸缩连接。 | 当前接口为单一 `runArgs` 结构体；旧主脚本中的无参或多参数调用尚未统一，相关开关默认关闭。 |
| `functions/navigation/RunRtsBackward.m` | 根据滤波/预测协方差和状态转移矩阵执行简化RTS反向递推。 | 当前终端误差固定为零且没有输入滤波误差序列，因而会得到全零误差；它是公式骨架，不是完整可用RTS。 |
| `functions/optimization/optimize_imu_incremental_1s.m` | 用距离残差建立6维非线性方程，通过 `fsolve` 优化1 s累计角增量和速度增量。 | 依赖Optimization Toolbox；模型中的坐标和量纲仍需要单独验证。 |
| `functions/optimization/optimize_imu.m` | 从调用者工作区提取导航状态、测距和最近100个IMU历元，调用上面的优化函数并把改正量分配回单历元。 | 这是脚本片段而非函数，依赖大量调用者变量，耦合较强。 |

## 已知问题与推荐入口

当前建议按以下优先级使用：

1. 先运行 `scripts/azimuth/test.m` 检查方位角几何闭环；
2. 用 `scripts/azimuth/position_sample.m` 检查加入航向、距离和方位角噪声后的定位误差；
3. 830/430距离/INS基线从 `scripts/experiment/ins_range_MEMS_1.m` 开始；
4. 真正研究距离+方位角联合更新时，以 `ins_range_azi_MEMS_new.m` 为主，另外两个版本作为历史对照；
5. 仿真IMU信息优化使用 `scripts/simulation/ins_range_imuInforOpti.m`，运行前先确认 `data_simu/input` 数据是否需要由 `data_get.m`重新生成。

目前不建议直接启用 `backwardIsOpen_1s` 或把 `RunRtsBackward` 作为正式RTS结果；两者都需要先统一接口并补充闭环验证。大型主脚本中的绘图、数据生成与核心导航后续还可以继续拆分，但本次迁移未改变算法主框架。

## MATLAB历史搜索路径

如果MATLAB启动时仍提示原 `D:\Github\KF-GINS-Matlab\MEMS_RANGE` 不存在，这是MATLAB“设置路径”中保存的旧条目，并非本专题代码仍引用旧目录。可在 MATLAB 的“主页 → 设置路径”中删除所有 `MEMS_RANGE` 条目并保存；新脚本会通过 `setup_mems_range_navigation.m` 添加当前路径。
