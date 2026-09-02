# 工程化应用评价

本目录集中保存输入误差和时序误差等工程问题的统计与绘图代码。每个入口均通过 `data_source` 读取仿真或实测专题上下文，不再维护两套评价实现。

- `beacon-position-error`：潜标位置误差。
- `range-measurement-error`：测距误差。
- `range-delay`：测距延迟及补偿。
- `acoustic-range-dropout`：声学测距包缺失敏感性。
- `common`：统一读取结果、严格共同区间统计、轨迹/径向误差绘图和 RMSE 参数响应。
