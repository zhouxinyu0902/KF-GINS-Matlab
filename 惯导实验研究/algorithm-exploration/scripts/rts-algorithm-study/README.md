# RTS算法研究

本目录集中研究前向 EKF、一次 RTS、二次 RTS，以及二次 RTS 的旋转收缩和位置速度约束重放。

## 推荐入口

- `run_rts_navigation_study.m`：最容易阅读的统一主脚本。通过 `data_source` 切换仿真/实测，通过 `position_error_unit` 切换 rad/m。
- `run_four_method_dataset_comparison.m`：仿真多数据集离线四方法对比。
- `run_fixed_lag_four_method_dataset_comparison.m`：仿真事件驱动固定滞后四方法对比。
- `run_navigation_experiment.m`：完整实测 rad 链函数，供四方法和延迟专题复用。

## 数据切换原则

仿真和实测只在以下环节不同：

- 配置函数；
- IMU、距离、高度和真值路径；
- 仿真是否现场加入距离/高度噪声；
- 输出根目录。

进入主循环后，量测更新、反馈、惯性传播、一次 RTS 和二次 RTS 使用同一段代码。

## 状态单位

```matlab
position_error_unit = "rad"; % [dLat,dLon,dH]
position_error_unit = "m";   % [dN,dE,dD]
```

两种单位必须成套选择对应配置、传播、更新、反馈和高度函数。详细定义见 `STATE-UNITS.md`。
