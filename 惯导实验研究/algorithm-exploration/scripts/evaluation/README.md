# 结果评价与绘图

本目录只保存结果读取、统计和绘图脚本。算法级评价与工程误差专题评价分开组织。

## rts

- `compare_radVSm.m`：仿真 rad/m 状态链对比，支持最简三方法和扩展四方法结果。
- `evaluate_experiment_results.m`：实测四方法统一评价入口。
- `evaluate_experiment_four_methods.m`：四方法统计与轨迹/径向误差公共函数。
- `compare_experiment_state_units.m`：实测 m/rad RMSE 对比函数。
- `compare_double_rts_correction_results.m`：当前四方法目录中的二次RTS和旋转收缩对比。

## input-data

- `evaluate_experiment_input_data.m`：实测输入与信标场景检查。
- `plot_navigation_scene.m`：轨迹与静止/移动信标通用绘图。
- `plot_trajectory_and_beacons.m`：真值和三信标局部平面绘图。

## gnss-ins

- `evaluate_fgo_gnss_ins.m`：GNSS/INS基准结果评价。

## engineering-problem

- `beacon-position-error/`：潜标位置误差对 EKF、一次 RTS、二次 RTS 的影响。
- `range-measurement-error/`：测距噪声标准差对三种算法的影响。
- `range-delay/`：测距延迟敏感性与回退重推进补偿评价。
- `common/`：以上工程专题共用的仿真/实测上下文读取、严格区间统计和绘图函数。

固定读取旧 `forward-backward` 结果的脚本以及 `cmp_simu_exper.m`、`result_plot.m` 已移入 `archive/legacy/evaluation`，不会由 `setup_inertial_experiment` 加入路径。
