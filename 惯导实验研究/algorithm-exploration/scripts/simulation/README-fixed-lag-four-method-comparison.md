# 事件驱动固定滞后四方法对比

运行 `run_fixed_lag_four_method_dataset_comparison.m`，对比：

- 前向 EKF；
- 二次 RTS；
- 2RTS + 旋转收缩；
- 2RTS + 位置速度约束（事件驱动固定滞后重放）。

本入口与 `run_four_method_dataset_comparison.m` 相互独立。原入口在完整二次 RTS 文件生成后进行全时段历史 IMU 重放；本入口在测距事件处一旦得到上一段二次 RTS，就立即重放对应区间的历史 IMU，并将重放终点的完整状态和协方差继承给下一段。

当测距间隔为 420 s 时，在当前测距时刻 `t_k` 发布的是区间 `[t_k-840 s, t_k-420 s]`。因此该区间起点使用了 840 s 后的信息，区间最新端点相对当前时刻滞后 420 s；这是分段固定滞后输出，不是等待全时段结束的离线处理。

输出目录：

```text
data/inertial-experiment/algorithm-exploration/navigation-results/simulation/<case>/fixed-lag-four-method-comparison
```

每个数据集生成四条导航结果、旋转收缩诊断、固定滞后伪量测诊断、统一统计表，以及轨迹和水平径向误差对比图。跨数据集汇总保存为：

```text
data/inertial-experiment/algorithm-exploration/navigation-results/simulation/fixed-lag-four-method-comparison-summary.csv
```

默认自动扫描全部 `case-*`。也可以在入口脚本顶部设置：

```matlab
selected_cases = ["case-00", "case-02"];
overwrite_existing = true;
```

