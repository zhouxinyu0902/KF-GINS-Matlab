# 一次、二次 RTS 平滑仿真说明

## 1. 目的

本次新增独立脚本 `scripts/simulation/simulate_range_ins_rts_comparison.m`，在不修改已经验证的 `simulate_range_ins_forward_backward.m` 的前提下，把 `scripts/experiment/FGO_range_ins_rad.m` 中的分段 RTS 思路引入仿真。

新脚本统一生成并比较以下五种结果：

1. 前向 AEKF；
2. BRC：由当前测距锚点向上一测距时刻执行纯反向惯导；
3. DPG-BRC：利用当前反推终点，延迟修正上一段红色反推轨迹；
4. 一次 RTS：利用当前测距更新误差平滑当前 420 s 区间；
5. 二次 RTS：利用下一段一次 RTS 传回的桥接误差，再次平滑上一段轨迹。

仿真条件与现有脚本保持一致：默认使用 `case-01`、测距间隔 420 s、三信标按 `1→2→3` 固定轮换，并固定随机数种子为 1。

## 2. 一次 RTS

在相邻测距时刻 `t(k-1)` 和 `t(k)` 之间，前向 15 状态 AEKF 连续传播，并缓存：

- 名义导航状态；
- 校正协方差 `P(i|i)`；
- 下一关键节点的预测协方差 `P(i+1|i)`；
- 相邻关键节点之间的状态转移矩阵 `Φ(i+1,i)`。

到达 `t(k)` 后，先进行测距更新，但暂不执行误差反馈。测距更新得到的 15 维误差状态作为区间末端条件：

```text
x_s(t(k)) = x_range(t(k))
```

随后按 RTS 形式向区间起点递推：

```text
G(i) = P(i|i) Φ(i+1,i)' pinv(P(i+1|i))
x_s(i) = G(i) x_s(i+1)
```

最后按照当前滤波器的误差反馈符号，将位置和速度误差从名义状态中扣除。为了与 `myErrorFeedback_range.m` 保持一致，当前版本不反馈姿态误差。

## 3. 二次 RTS

一次 RTS 到达区间起点后会产生桥接误差：

```text
x_bridge(k-1) = x_s(t(k-1))
```

当当前区间 `[t(k-1),t(k)]` 完成一次 RTS 后，把该桥接误差作为上一段 `[t(k-2),t(k-1)]` 的新末端条件，再对上一段已经完成一次 RTS 的轨迹执行一次 RTS 递推。

因此：

- 一次 RTS 使用当前测距事件提供的信息；
- 二次 RTS 比一次 RTS 多使用一个未来测距周期的信息；
- 二次 RTS 的输出延迟约为两个测距周期；
- 最后一个完整区间由于没有下一段桥接误差，二次结果退化为该区间的一次 RTS 结果。

这与 experiment 脚本中“当前段一次平滑—桥接误差传回上一段—上一段二次平滑”的结构一致。

## 4. RTS 关键节点间隔

配置项：

```matlab
options.rts_node_interval_s = 1;
```

前向 AEKF 的状态和协方差仍按照原始 100 Hz IMU 数据连续传播。为了控制 RTS 中大量 `15×15` 矩阵求解的计算量，默认每 1 s 保存一个 RTS 关键节点，再把关键节点上的平滑误差线性插值回 100 Hz 输出时刻。

如果需要逐 IMU 历元执行严格 RTS，可设置为：

```matlab
options.rts_node_interval_s = 0.01;
```

该设置会显著增加运行时间和内存占用。

## 5. 输出文件

结果保存在：

```text
data/inertial-experiment/algorithm-exploration/navigation-results/simulation/forward-backward
```

主要输出：

| 文件 | 内容 |
| --- | --- |
| `range-ins-forward.nav` | 前向 AEKF |
| `range-ins-backward-constrained.nav` | 纯反向 BRC |
| `range-ins-delayed-previous-geometry.nav` | 延迟修正上一段红线的 DPG-BRC |
| `range-ins-rts-single.nav` | 一次 RTS |
| `range-ins-rts-double.nav` | 二次 RTS |
| `range-rts-diagnostics.csv` | 各段末端误差、一次桥接误差和二次末端误差 |
| `range-rts-result-statistics.csv` | 五种算法的全时段及有效区间误差统计 |

图形保存在 `figures` 子目录：

| 文件 | 内容 |
| --- | --- |
| `range-rts-comparison.png/.fig` | 五种结果的平面轨迹、水平径向误差、北向误差和东向误差 |

## 6. 运行方法

在 MATLAB 中执行：

```matlab
run('D:\Github\KF-GINS-Matlab\惯导实验研究\setup_inertial_experiment.m');
run(fullfile('D:\Github\KF-GINS-Matlab', '惯导实验研究', ...
    'scripts', 'simulation', 'simulate_range_ins_rts_comparison.m'));
```

## 7. 解释结果时的注意事项

1. 当前实现平滑的是前向 AEKF 的 15 维误差状态，不是红色反向惯导轨迹。
2. 一次、二次 RTS 都属于离线或延迟处理，不能与无延迟实时滤波完全等价比较。
3. experiment 原实现采用 `pinv` 和极小对角正则项处理预测协方差的病态问题，新脚本保留这一数值保护。
4. 二次 RTS 并不是把同一个测距残差重复使用两次，而是把下一段的一次平滑起点误差作为上一段的新末端条件。
5. 如果二次 RTS 明显发散，应首先检查预测协方差的条件数、桥接误差量级以及自适应测距更新产生的末端误差，而不是直接增加平滑次数。

