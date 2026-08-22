# 实测数据处理

## 目录职责

- `dataget.m`：从参考轨迹生成高度和三个固定信标的原始距离文件，不再创建图窗；
- `run_experiment_four_method_comparison.m`：实测 m 链四方法运行入口，只生成导航结果与统计表；
- `run_experiment_rts_core.m`：事件驱动 EKF、RTS、旋转收缩和历史 IMU 重放核心，不含评价绘图；
- `run_experiment_four_method_comparison_rad.m`：rad 链四方法运行入口，只生成导航结果与统计表；
- `run_experiment_rts_core_rad.m`：rad 链四方法核心，不含评价绘图；
- `compare_range_delay_compensation_rad.m`：运行三种测距延迟工况并生成导航结果、统计和绘图上下文，不创建图窗；
- `STATE-UNITS.md`：两套误差状态、反馈符号和RTS桥接误差说明；
- `FGO_gnss_ins.m`：GNSS/INS探索与基准脚本，暂时保留；
- `ins_range_threetoge.m`：早期反向推算探索脚本，归档保留，不参与当前四方法主链。

所有实测绘图集中到 `scripts/evaluation`。四方法统一绘图入口为 `evaluate_experiment_results.m`，输入数据检查入口为 `evaluate_experiment_input_data.m`；测距延迟补偿实验的绘图入口为 `plot_range_delay_compensation_rad.m`。

## 四种对比方法

1. 前向EKF；
2. 二次RTS；
3. 2RTS+旋转收缩；
4. 2RTS+位置速度约束（事件驱动固定滞后重放）。

位置速度约束不是等全时段结束后统一重算：每当新测距点使上一段二次RTS成熟，就立即重放该段历史IMU。由于测距间隔为420 s，同一成熟区间内的实际延迟约为7～14 min（区间末端约7 min、区间起点约14 min）。

主链直接读取：

```text
data/inertial-experiment/algorithm-exploration/input/experiment-preprocessed/rangedata_noised.txt
data/inertial-experiment/algorithm-exploration/input/experiment-preprocessed/height_noised.txt
```

这些文件作为当前正式实验输入使用，导航脚本不会重新生成或覆盖它们。`dataget.m` 只负责重新生成原始距离/高度数据。结果保存到：

```text
data/inertial-experiment/algorithm-exploration/navigation-results/experiment/four-method-comparison
```

## 位置单位约定

名义导航位置 `navstate.pos` 始终是：

```text
[纬度(rad), 经度(rad), 高度(m)]
```

误差状态存在两套互斥定义：

- rad链：`x(1:3)=[dLat(rad), dLon(rad), dH(m)]`；
- m链：`x(1:3)=[dN(m), dE(m), dD(m)]`，第三维为下向误差。

实测四方法主链统一采用m链。RTS桥接误差的比较、阈值和诊断必须统一换算成米：m链直接使用 `hypot(x(1),x(2))`；rad链使用 `bridge_error_horizontal_m` 按当前纬度和高度换算。

两条链不得交叉调用传播、量测更新、反馈和平滑应用函数。
完整的函数对应关系和符号推导见 `STATE-UNITS.md`。

## rad链对照实验

运行 `run_experiment_four_method_comparison_rad.m`，结果写入：

```text
data/inertial-experiment/algorithm-exploration/navigation-results/experiment/four-method-comparison-rad
```

该入口使用与 m 链相同的输入、4621 s 时段、11 个测距点和四种处理方法，只替换误差状态相关的配置、传播、更新、反馈、高度更新及 RTS 误差应用。若 m 链统计已经存在，运行入口只更新 `rad-vs-meter-rmse-comparison.csv`；分组 RMSE 图片由 `scripts/evaluation/evaluate_experiment_results.m` 生成。

当前实测结果显示rad链四方法RMSE依次为188.72、37.76、36.85和38.66 m，均高于对应m链。需要注意，两套现有传播函数并不是严格的相似变换版本，因此该对比反映的是“当前两套完整实现”的差异，不能仅解释为rad与m的浮点数单位差异。

## 前向EKF与二次RTS测距延迟敏感性

运行 `scripts/evaluation/compare_double_rts_range_delay_rad.m`。脚本固定采用rad位置误差状态、420 s测距间隔和4621 s处理时长，对比：

1. 在更新时刻 `t` 使用该时刻的距离；
2. 更新时刻仍为 `t`，但使用同一信标在 `t-4 s` 的距离。

第二组模拟的是“量测值已经陈旧，但系统仍把它当作当前时刻量测”的未补偿延迟。脚本从三条1 Hz原始距离序列中插值得到 `t-4 s` 真值，并给两组叠加完全相同的原有噪声残差。除距离真值的来源时刻外，IMU、高度、信标、更新时间和滤波参数均不变。

评价分为两部分：前向EKF读取两次运行的 `forward_path`，使用两组共同有效的完整4621 s时段；二次RTS读取 `double_rts_path`，只使用两组都真正完成第二遍RTS的共同区间。两类算法分别生成轨迹—水平径向误差图和统计表，不混用评价掩码。导航结果保存到：

```text
data/inertial-experiment/algorithm-exploration/navigation-results/experiment/ekf-double-rts-range-delay-rad
```

输入差值、轨迹、水平径向误差和统计保存到对应的 `figures-tables` 目录。`run_experiment_rts_core_rad` 仍保持原调用方式兼容，同时新增可选 `runtime_options`，供本对比传入自定义测距矩阵并关闭无关的旋转收缩及历史重放计算。

当前这组11个测距点中，4 s造成的距离真值变化平均绝对值为10.76 m，最大绝对值为18.15 m。前向EKF水平RMSE由195.53 m增至210.24 m，增加14.71 m（7.52%）；二次RTS水平RMSE由37.76 m增至56.53 m，增加18.77 m（49.71%）。这说明未补偿的陈旧距离对二次RTS桥接和平滑结果的相对影响更明显。

## 固定4 s延迟的回退重推进补偿

先运行 `scripts/experiment/compare_range_delay_compensation_rad.m`。脚本比较无延迟、延迟4 s未处理、延迟4 s固定滞后补偿三种工况，并分别统计前向EKF、二次RTS和2RTS+旋转收缩。

补偿工况把陈旧距离的事件时间从“到达时刻”恢复为“真实采集时刻”，然后按时间顺序重新计算导航与RTS结果。对确定性的离线计算而言，这与量测在到达时回退4 s、更新并用同一段IMU重推进等价；但当前实现不是在线环形缓存回放器，也不能提前使用尚未到达的量测。因而补偿结果的可用时刻仍固定滞后4 s，二次RTS和旋转收缩还要叠加各自原有的固定滞后。

实验脚本只写导航结果、CSV统计和 `range-delay-compensation-evaluation-context.mat`，不包含任何绘图调用。完成实验后运行 `scripts/evaluation/plot_range_delay_compensation_rad.m`，读取上述上下文和已有导航结果生成三组轨迹—水平径向误差图片。

原README中的补偿统计来自未被核心函数读取的 `compensate_range_delay` 参数，不能作为有效结论，已删除。修正后的数值应以重新运行脚本生成的CSV为准。结果保存到：

```text
data/inertial-experiment/algorithm-exploration/navigation-results/experiment/range-delay-compensation-rad
data/inertial-experiment/algorithm-exploration/figures-tables/experiment/range-delay-compensation-rad
```

