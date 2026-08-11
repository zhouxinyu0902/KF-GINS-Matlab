# DR_INS 运行说明与数据格式

生成日期：2026-08-10

## 1. MATLAB 路径前提

当前脚本依赖 PSINS 和 KF-GINS 项目中的若干函数。运行前至少需要保证以下函数可见：

- PSINS：`glvs`、`earth`、`a2mat`、`trjsimu`、`trjsegment`、`imuerrset`、`imuadderr` 等。
- 项目函数：`avpENU2NED`、`imuRFU2FRD`、`calc_radial_error_gjb`。
- 当前目录函数：`function/*.m`。

建议运行前在 MATLAB 中执行：

```matlab
addpath(genpath('D:\Github\PSINS\psins2401'));
addpath(genpath('D:\Github\KF-GINS-Matlab'));
addpath(genpath('D:\Github\KF-GINS-Matlab\graduation\DR_INS'));
```

如果不希望污染全局路径，至少要把 `DR_INS/function` 加入路径。

## 2. 生成数据

入口：

```matlab
cd('D:\Github\KF-GINS-Matlab\graduation\DR_INS\input')
dataget
```

当前 `dataget.m` 内部默认设置：

```matlab
traj_case = 'line_N';
beacon_case = 'four_quadrants';
cfg.range_mode = 'horizontal';
cfg.range_schedule = 'simultaneous';
```

输出目录：

```text
D:\Github\KF-GINS-Matlab\graduation\DR_INS\input\data_line_N_four_quadrants
```

如需生成其他数据集，需要在 `dataget.m` 中修改：

- `traj_case`
- `beacon_case`
- `cfg.range_mode`
- `cfg.range_schedule`
- 传感器噪声和 DVL 刻度误差参数

## 3. 单信标 DR/RANGE 运行

入口：

```matlab
cd('D:\Github\KF-GINS-Matlab\graduation\DR_INS')
main_DR_RANGE
```

当前脚本默认输入：

```matlab
in_dir = 'D:\Github\KF-GINS-Matlab\graduation\DR_INS\input\data_line_N_single_side';
```

主要输出：

```text
input/data_line_N_single_side/output_DR_RANGE/Origin-DR.nav
input/data_line_N_single_side/output_DR_RANGE/DR-RANGE.nav
input/data_line_N_single_side/output_DR_RANGE/DR-RANGE-state.txt
input/data_line_N_single_side/output_DR_RANGE/DR-RANGE-innovation.txt
```

注意：当前 `main_DR_RANGE.m` 存在外层 `for ii=1:4` 但输出文件名固定的问题，会重复覆盖同一批输出。正式使用前建议先修正。

## 4. 四信标对比运行

入口：

```matlab
cd('D:\Github\KF-GINS-Matlab\graduation\DR_INS')
main_DR_RANGE_4beacon
```

当前脚本默认输入：

```text
input/data_line_N_four_quadrants
```

该目录应包含：

```text
beacon_1.txt
beacon_2.txt
beacon_3.txt
beacon_4.txt
compass.txt
depth.txt
dvl.txt
reference.txt
```

输出：

```text
input/data_line_N_four_quadrants/output_DR_RANGE/Origin-DR-1.nav
input/data_line_N_four_quadrants/output_DR_RANGE/DR-RANGE-1.nav
...
input/data_line_N_four_quadrants/output_DR_RANGE/Origin-DR-4.nav
input/data_line_N_four_quadrants/output_DR_RANGE/DR-RANGE-4.nav
```

每个编号代表只使用对应信标的距离辅助结果，适合比较不同信标几何布局下的约束效果。

## 5. 结果绘图

当前 `result_plot.m` 的目标是比较四信标结果：

```matlab
path = "D:\Github\KF-GINS-Matlab\graduation\DR_INS\input\data_line_N_four_quadrants\output_DR_RANGE";
pathcell = {
    path + "/Origin-DR-1.nav", ...
    path + "/DR-RANGE-1.nav", ...
    path + "/DR-RANGE-2.nav", ...
    path + "/DR-RANGE-3.nav", ...
    path + "/DR-RANGE-4.nav"
};
```

但该脚本缺少：

```matlab
cfg = config_DR_RANGE(...);
outputfolder = ...;
```

因此当前不能作为独立入口直接运行。建议补全后再使用。

## 6. 输入数据格式

### 6.1 `reference.txt`

```text
t N E D pitch roll yaw vN vE vD
```

说明：

- `N/E/D`：局部 NED 坐标，单位 m。
- `D`：向下为正。
- `pitch/roll/yaw`：姿态角，通常由生成脚本输出，需结合生成脚本确认单位。当前 `compass.txt` 明确为 rad。
- `vN/vE/vD`：NED 速度，单位 m/s。

### 6.2 `dvl.txt`

```text
t vb_x vb_y vb_z
```

说明：

- `t`：时间，单位 s。
- `vb_x/vb_y/vb_z`：体坐标系速度，单位 m/s。
- 在 `DRmechanization` 中会除以 `dr.kod`，再用 `a2mat(att)` 转到导航系。

### 6.3 `compass.txt`

```text
t pitch roll yaw
```

说明：姿态角单位 rad。

### 6.4 `depth.txt`

```text
t depth
```

说明：

- `depth` 单位 m。
- 深度按 NED 中 D 向下为正。
- DR 内部 `pos(3)=h` 向上为正，所以代码中使用 `dr.pos(3) = -depth`。

### 6.5 `beacon.txt` / `beacon_*.txt`

```text
t slant_range horizontal_range beacon_lat beacon_lon beacon_h
```

说明：

- `slant_range`：三维斜距，单位 m。
- `horizontal_range`：水平距离，单位 m。
- 当前 `DRRangeUpdate` 使用第三列 `horizontal_range` 作为量测。
- `beacon_lat/beacon_lon`：rad。
- `beacon_h`：m，PSINS 高度形式，向上为正。

## 7. 导航输出格式

`DRWriteNavLine` 输出 `.nav`：

```text
id time lat lon h VE VN VU pitch roll yaw
```

默认 `cfg.output_in_degree = true`，因此：

- `lat/lon`：deg。
- `pitch/roll/yaw`：deg。
- `h`：m，向上为正。
- `VE/VN/VU`：m/s。

## 8. EKF 状态定义

默认 4 维状态：

```text
x = [dK; dYaw; dLat; dLon]
```

定义：

```text
x = true - DR_estimate
```

含义：

- `dK`：DVL 刻度因子误差。
- `dYaw`：航向误差，rad。
- `dLat/dLon`：位置误差，rad。

默认反馈策略：

```matlab
cfg.feedback.position = true;
cfg.feedback.dvl_scale = false;
cfg.feedback.yaw = false;
```

即只将水平位置误差反馈到 DR 主状态，不反馈 DVL 刻度因子和航向误差。这样便于观察参数可观测性，但不代表最终组合导航最优策略。

## 9. 推荐运行顺序

1. 检查 MATLAB 路径。
2. 根据实验目的修改 `input/dataget.m` 的轨迹和信标布局。
3. 运行 `dataget.m` 生成数据。
4. 单信标实验运行 `main_DR_RANGE.m`；四信标对比运行 `main_DR_RANGE_4beacon.m`。
5. 检查 `output_DR_RANGE` 中 `.nav`、`state`、`innovation` 文件。
6. 修正并运行 `result_plot.m` 做结果汇总。

## 10. 后续 DR/INS 仿真扩展建议

若后续要做 DR/INS，而不只是 DR/RANGE，建议新增独立结构：

```text
main_INS_DR_RANGE.m
function/INSInitialize.m
function/INSMechanization.m
function/INS_DR_RangeUpdate.m
function/ErrorFeedback_INS_DR.m
```

原则：

- INS 机械编排使用 `imu.txt`。
- DR/DVL/罗盘/深度作为辅助观测或独立对比链路。
- 距离量测作为位置相关观测。
- 明确统一状态定义，避免 `true - estimate` 与 `estimate - true` 混用。
