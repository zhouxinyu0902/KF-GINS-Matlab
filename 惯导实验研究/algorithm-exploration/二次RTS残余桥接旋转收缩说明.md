# 二次 RTS 残余桥接旋转收缩说明

## 1. 新增方法

当前方法位于 `functions/simulation/simulate_navigation_case.m`，由 `scripts/rts-algorithm-study` 下的四方法批处理入口调用。统一最简脚本 `run_rts_navigation_study.m` 不包含旋转收缩。该方法在一次、二次 RTS 基础上增加一种残余桥接误差驱动的旋转收缩方法，记为：

```text
RC-2RTS（Residual-Bridge-Constrained Double RTS）
```

## 2. 为什么使用延迟旋转收缩

设三个连续测距区间为：

```text
S(k-1) = [t(k-2), t(k-1)]
S(k)   = [t(k-1), t(k)]
S(k+1) = [t(k),   t(k+1)]
```

区间 `S(k+1)` 的一次 RTS 会在 `t(k)` 产生桥接误差，该误差作为区间 `S(k)` 的二次 RTS 末端条件。区间 `S(k)` 完成二次 RTS 后，又会在其起点 `t(k-1)` 产生新的残余桥接误差。

这个残余桥接误差已经参与 `S(k)` 的二次 RTS，不能再次直接从 `S(k)` 中扣除，否则会重复使用同一误差信息。

因此，本方法把区间 `S(k)` 的二次 RTS 起点视为 `t(k-1)` 的新参考节点，并用它延迟修正更早的区间 `S(k-1)`：

1. 取区间 `S(k-1)` 的二次 RTS 轨迹；
2. 固定其较早端点 `t(k-2)`；
3. 把其较晚端点 `t(k-1)` 对齐到区间 `S(k)` 二次 RTS 的起点；
4. 对 `S(k-1)` 的水平轨迹整体进行旋转和等比例缩放；
5. 高度、速度和姿态保留二次 RTS 的原结果。

这个结构与前面“当前反推节点延迟修正上一段红色轨迹”的思想相同，但待修正对象改成了上一段二次 RTS 轨迹，参考节点来自二次 RTS 的残余桥接误差。

## 3. 输出延迟

- 一次 RTS 需要当前区间末端测距信息；
- 二次 RTS 需要再等待一个测距周期；
- RC-2RTS 需要获得下一段二次 RTS 的起点参考节点，因此相对待修正区间约延迟三个测距周期。

没有获得后续残余桥接节点的末尾区间保留二次 RTS 结果，不进行人为外推。

## 4. 输出文件

结果目录：

```text
data/inertial-experiment/algorithm-exploration/navigation-results/simulation/forward-backward
```

新增输出：

| 文件 | 内容 |
| --- | --- |
| `range-ins-rts-double-bridge-rotation.nav` | 二次 RTS 残余桥接旋转收缩结果 |
| `range-rts-bridge-rotation-diagnostics.csv` | 残余桥接误差、旋转角、尺度因子和端点残差 |
| `range-rts-bridge-rotation-statistics.csv` | 六种算法的误差统计 |

新增图片：

| 文件 | 内容 |
| --- | --- |
| `range-rts-bridge-rotation-comparison.png/.fig` | RC-2RTS 与前五种算法的轨迹和误差对比 |
| `range-rts-bridge-rotation-diagnostics.png/.fig` | 各区间残余桥接误差、旋转角、尺度因子和端点对齐残差 |

## 5. 运行方法

```matlab
run('D:\Github\KF-GINS-Matlab\惯导实验研究\setup_inertial_experiment.m');
run(fullfile('D:\Github\KF-GINS-Matlab', '惯导实验研究', ...
    'scripts', 'simulation', ...
    'simulate_range_ins_rts_rotation_contraction.m'));
```

## 6. 注意事项

1. 残余桥接误差用于建立新的端点参考，不在同一区间重复执行误差反馈。
2. 旋转收缩仅作用于水平位置，避免水深变化参与三维旋转和尺度计算。
3. RC-2RTS 是延迟轨迹重建方法，不属于实时滤波输出。
4. 若尺度因子明显偏离 1，应重点检查残余桥接误差是否过大、RTS 协方差是否病态，以及测距更新是否产生异常末端误差。

## 7. case-01 初次结果

默认配置完整运行后：

| 方法 | 相邻测距有效区间 RMSE |
| --- | ---: |
| 二次 RTS | 25.65 m |
| RC-2RTS | 28.65 m |

RC-2RTS 将 9 个区间修正前约 `3.93～54.92 m` 的端点错位降低到约 `0.00026～0.00233 m`，但总体 RMSE 比二次 RTS 增加约 3.00 m。这说明刚性旋转与等比例缩放虽然改善了区间拼接连续性，却会在部分区间放大内部轨迹误差。后续若继续优化，应考虑按残余桥接误差大小设置修正权重、限制尺度因子和旋转角，或使用沿时间渐变的柔性修正代替整段刚性相似变换。

