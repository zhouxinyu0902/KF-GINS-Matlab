# DVL、罗盘与单距离辅助仿真

本目录按照“先构造数据，再研究误差，最后进行滤波”的顺序组织。

## Step1：构造仿真数据

运行：

```matlab
Step1_Generate_Typical_Data
```

生成直线、单次转弯、矩形和 S 形四类真实轨迹，同时生成：

- 真实 DVL 速度；
- 带正负 0.4% 刻度因子误差的 DVL 速度；
- 真实罗盘航向；
- 带正负 0.23°系统误差的罗盘航向；
- 真实位置、速度、深度和时间。

这一阶段暂不加入随机噪声。这样可以先把系统误差造成的现象看清楚。

## Step2：初步研究误差传播和距离残差

运行：

```matlab
Step2_Error_Propagation_And_Range_Residual
```

Step2 读取 Step1 生成的 1800 s 直线航行数据，比较五种工况：

1. 仅 DVL +0.4%；
2. 仅罗盘 +0.23°；
3. 仅罗盘 −0.23°；
4. DVL +0.4%、罗盘 +0.23°；
5. DVL +0.4%、罗盘 −0.23°。

主要观察两类结果：

- 位置误差：判断 DVL 误差是否主要沿航向累积、罗盘误差是否主要沿横向累积；
- 距离残差：判断位置误差经过信标视线投影后，是增强、减弱还是相互抵消。

这里不使用 EKF。它只验证从传感器误差到距离残差的基础链条：

```text
DVL/罗盘误差 → 速度误差 → 位置误差 → 距离残差
```

## Step3：扫描信标相对方位角

运行：

```matlab
Step3_Beacon_Bearing_Sweep
```

Step3 固定 1800 s 直线航行终点，把信标布置在航行器周围半径
3000 m 的圆周上，并将信标相对航向方位角从 0°扫描到 360°。
分别计算 DVL 项、罗盘项、一阶总距离残差和精确总距离残差，用于寻找：

- DVL 刻度因子误差的最大敏感方向；
- 罗盘误差的最大敏感方向；
- 两类误差相互抵消、距离残差接近零的方向。

## Step4：比较多种移动信标几何

运行：

```matlab
Step4_Three_Beacon_Geometry_Comparison
```

Step4 令多个信标随真实载体平行移动，并始终保持 3000 m 距离。
重点比较 0°、90°和根据当前误差参数自动计算的一阶抵消角，并保留
45°、80°补充工况。脚本使用简单二维位置 EKF，对比恒定视线方向下
的距离残差和位置修正效果，并绘制载体与信标轨迹。

## Step5：相同方位角、不同移动信标起点

运行：

```matlab
Step5_Same_Bearing_Different_Beacon_Start
```

Step5 从 Step3 自动选取正误差组合的残差抵消方向和最大绝对残差方向。
在每个方位角下分别设置 1000 m、3000 m 和 5000 m 三条移动信标轨迹，
比较相同方位、不同信标轨迹起点下的距离残差和辅助效果。

## Step6：典型轨迹下的单距离辅助航位推算

运行：

```matlab
Step6_Single_Range_Aided_DR
```

Step6 使用 `mydr` 和 `myekf`，对比四类典型轨迹下的纯 DR 与单固定信标距离辅助 DR。
建议先解释清楚 Step2 至 Step5 的误差和几何规律，再调整完整滤波参数。

## 当前参数

| 参数 | 数值 | 说明 |
|---|---:|---|
| 采样周期 | 0.5 s | 与现有实测处理一致 |
| 巡航速度 | 1 m/s | 便于直接解释误差量级 |
| 直线匀速时间 | 1800 s | 用于观察误差累积 |
| DVL 刻度因子误差 | ±0.4% | 当前研究基准 |
| 罗盘航向系统误差 | ±0.23° | 与 DVL 位置误差量级基本齐平 |
| 随机噪声 | 关闭 | 留到后续蒙特卡洛实验 |

在 1 m/s、1800 s 条件下，0.4% DVL 刻度误差约产生 7.2 m 沿程误差；
0.23°航向误差的一阶量级约为 7.2 m 横向误差。

## 输出

所有结果位于 `generated_data`：

- `trajectory_*.mat`：真实轨迹、DVL 和罗盘仿真数据；
- `trajectory_catalog.mat`：场景文件列表和公共参数；
- `step2_error_propagation_results.mat`：Step2 数值结果；
- `step2_error_propagation_summary.csv`：1800 s 误差汇总；
- `step2_error_propagation.png`：位置误差和距离残差对比图；
- `step3_beacon_bearing_sweep_results.mat`：信标方位扫描数据；
- `step3_beacon_bearing_sweep_summary.csv`：最大敏感方向汇总；
- `step3_beacon_bearing_sweep.png`：距离残差随信标方位变化曲线；
- `step4_three_beacon_geometry_results.mat`：多种移动信标几何完整结果；
- `step4_three_beacon_geometry_summary.csv`：多种几何定量对比；
- `step4_three_beacon_geometry.png`：方位变化、距离残差和位置误差；
- `step5_beacon_start_comparison_results.mat`：不同信标起点完整结果；
- `step5_beacon_start_comparison_summary.csv`：不同信标起点定量对比；
- `step5_beacon_start_trajectories.png`：载体与信标轨迹对比；
- `step5_beacon_start_effect.png`：距离残差与辅助效果对比；
- `single_range_aided_dr_results.mat`：Step6 滤波完整结果；
- `single_range_summary.csv`：纯 DR 与距离辅助 DR 统计结果。

## 实测脚本

`Exper*` 文件仍用于实测数据提取和处理，本次整理未修改其研究逻辑。
