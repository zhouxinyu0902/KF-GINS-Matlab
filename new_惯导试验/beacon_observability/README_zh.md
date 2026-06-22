# 轮换信标有限时间可观测度分析

该代码使用与 `myRangeUpdate.m` 相同的水平距离雅可比，并利用
`myInsPropagate_15state.m` 输出的离散状态转移矩阵 `kf.phi` 构造有限时间
可观测矩阵。

## 比较方案

- `Alternating`：主程序实际采用的轮换信标序列。
- `Fixed-B1`、`Fixed-B2`、`Fixed-B3`：在相同车辆轨迹、相同测量时刻和相同
  测距噪声下，假设始终由同一个信标测距。

因此，比较结果只反映信标几何变化，不受测量次数和车辆轨迹差异影响。

## 主要指标

- `horizontal_lambda_min`：水平位置二维信息矩阵的最小特征值，越大表示最弱
  水平位置方向受到的约束越强。
- `horizontal_condition`：水平信息矩阵条件数，越小表示两个水平方向的信息越
  均衡。
- `horizontal_logdet`：水平位置信息体积，越大越好。
- `full_rank`：归一化15维误差状态在当前时间窗口内的有效可观测秩。

水平位置指标是论证轮换信标几何优势的主要结果；15维秩可作为辅助结果。代码
仅分析声学水平测距，不把高频深度观测混入，以免夸大轮换信标贡献。

## 主程序接入

参考 `exper1_observability.m` 中带有 `OBSERVABILITY` 注释的代码。程序结束后会
在当前实验输出目录生成：

- `beacon_observability_summary.csv`
- `beacon_observability_windows.csv`：每个滑动窗口、每种信标方案的指标明细
- `beacon_observability_results.mat`
- `beacon_observability_comparison.png`
- `beacon_observability_boxplots.png`

默认最多采用9次测距，即三个完整的1-2-3轮换周期。若可用测距少于9次，程序
自动选择不超过数据量的最大3的倍数。论文实验可进一步比较6、9、12或更长窗口，
并报告不同窗口长度下结论是否一致。
