# 声学测距掉帧专题

本专题研究低频声学测距包偶发丢失对前向 EKF、一次 RTS 和二次 RTS 的
影响。默认使用 algorithm-exploration 的实测 `case-06`，也可以在运行
脚本顶部切换为任一仿真 case。

运行入口：`run_acoustic_range_dropout_study.m`。

默认工况包括完整测距，以及分别丢失第 1、3、5、7、9、11 个测距包。
各工况使用相同输入噪声和滤波参数，只改变一个测距包是否存在。

生成内容按当前统一目录保存：

- 专题输入：对应 case 的 `input/.../acoustic-range-dropout`；
- 导航结果：对应 case 的 `navigation-results/.../acoustic-range-dropout-rad`；
- 图表统计：对应 case 的 `figures-tables/.../acoustic-range-dropout-rad`。

导航计算结束后，运行评价目录中的
`evaluate_acoustic_range_dropout_study.m` 生成统计、径向误差和轨迹图。
