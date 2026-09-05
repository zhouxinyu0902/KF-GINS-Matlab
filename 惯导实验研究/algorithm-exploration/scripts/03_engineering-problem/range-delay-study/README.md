# 测距延迟研究

本目录只负责距离数据延迟工况的构造和导航计算。

## 入口

- `run_range_delay_sensitivity_study.m`：对比当前距离和固定延迟距离对前向 EKF、二次 RTS 的影响。
- `run_range_delay_compensation_study.m`：对比无延迟、延迟未处理和回退重推进三种工况。
- `../../evaluation/engineering-problem/range-delay/evaluate_range_delay_sensitivity_study.m`：仿真/实测延迟敏感性统一统计和绘图。
- `../../evaluation/engineering-problem/range-delay/evaluate_range_delay_compensation_study.m`：仿真/实测延迟补偿统一统计和绘图。

两个计算入口通过公共适配器分别复用仿真和实测核心。延迟补偿结果仍具有相应固定滞后，不等于零延迟实时导航。评价脚本统一归入 `evaluation/engineering-problem`。
