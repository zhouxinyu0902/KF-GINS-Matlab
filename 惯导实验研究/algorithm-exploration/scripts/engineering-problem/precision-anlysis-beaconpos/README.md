# 潜标位置精度分析

本专题负责构造潜标位置误差工况，通过 `data_source` 选择仿真或实测数据，并运行统一 EKF/RTS 接口。

- `process_beaconpos.m`：按仿真或实测数据和任意 ENU 误差表批量生成输入，采用 WGS-84 曲率半径换算，不覆盖原始数据。
- `run_beacon_position_error_study.m`：统一运行前向 EKF、一次 RTS 和二次 RTS，并保存评价上下文。

统计和绘图位于 `scripts/evaluation/engineering-problem/beacon-position-error`，评价入口使用相同的 `data_source` 开关。
