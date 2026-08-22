# 结果评价与绘图

本目录集中存放 `algorithm-exploration` 专题的结果读取、统计和绘图代码。实测核心算法与 `dataget.m` 不再直接创建图窗。

## 推荐入口

- `evaluate_experiment_results.m`：读取已有 m/rad 实测四方法结果，生成统计表、轨迹图、水平径向误差图和 m/rad RMSE 对比图；
- `evaluate_experiment_input_data.m`：检查实测输入数据，绘制 PVA、定位标准差、轨迹—信标和三信标距离曲线；
- `evaluate_fgo_gnss_ins.m`：评价 `FGO_gnss_ins.m` 已生成的 GNSS/INS 结果；
- `compare_double_rts_range_delay_rad.m`：4 s 陈旧测距对前向 EKF 和二次 RTS 的敏感性试验；
- `plot_range_delay_compensation_rad.m`：读取已完成的三工况延迟补偿实验结果，分别绘制前向 EKF、二次 RTS 和2RTS+旋转收缩的轨迹与水平径向误差；运行前须先执行 `scripts/experiment/compare_range_delay_compensation_rad.m`。

## 公用函数

- `evaluate_experiment_four_methods.m`：m/rad 共用的四方法统计与双图绘制函数；`create_figure=false` 时只写统计表；
- `compare_experiment_state_units.m`：统一生成 m/rad RMSE 对比表，并可选绘图；
- `plot_navigation_scene.m`：直角坐标或经纬高轨迹—信标场景图；
- `plot_trajectory_and_beacons.m`：将真值和三信标转换到局部平面后绘图。

统一绘图使用 `myfigurestartup`，中文图表字体采用 `TimesSimSun`，正式 PNG 默认以 600 dpi 导出。

## 使用顺序

1. 运行 `scripts/experiment` 下的数据生成或导航核心入口；
2. 导航结果确认生成后，再运行本目录对应评价脚本；
3. 图、FIG 和 CSV 仍保存到专题总数据目录的 `figures-tables` 对应子目录。
