# RTS 算法研究入口

本目录保留十个职责明确的入口。各脚本不再相互复制或隐式执行其他算法。

## 脚本职责

| 脚本 | 唯一职责 | 主要输出 |
|---|---|---|
| `run_rts_navigation_study.m` | 原始前向 ES-EKF、一次 RTS、二次 RTS | `simple-forward-ekf-*.nav`、`simple-rts-single-*.nav`、`simple-rts-double-*.nav` |
| `run_fixed_lag_rts_study.m` | 可配置 `lag_segments` 的固定滞后 RTS | `simple-rts-lagN-*.nav` |
| `run_standard_fixed_lag_rts_study.m` | 连续窗口内单次反向递推、只发布最老成熟区间的标准固定滞后RTS | 严格成熟轨迹、事件诊断、精度与连续性对比 |
| `run_four_method_dataset_comparison.m` | 读取原始 2RTS 结果，执行旋转收缩和位置速度约束重放 | `four-method-rts-double-*.nav` |
| `run_standard_global_rts_study.m` | 测距触发的标准全局 RTS | `range-ins-rts-standard-global.nav` |
| `run_periodic_global_rts_study.m` | 保留7 min测距更新、按可配置长周期触发全局RTS | 严格全局快照、触发诊断、精度与连续性对比 |
| `run_windowed_periodic_global_rts_study.m` | 每42 min对最近56 min执行一次窗口RTS，并以2RTS补齐尚未触发的尾段 | 混合在线轨迹、窗口接缝诊断、精度与计算量对比 |
| `run_windowed_periodic_global_rts_dataset_comparison.m` | 只读取case-06～09已有的56 min窗口RTS结果，汇总并绘制跨数据集对比 | 四数据集精度、改善比例和窗口接缝汇总 |
| `run_windowed_periodic_global_rts_error_curves.m` | 只读取已有导航结果，绘制case-06～09的水平误差时间曲线 | 四工况2RTS、窗口RTS与前向EKF误差图 |
| `run_all_lags_global_rts_comparison.m` | 只读取已有结果，批量比较 case-06～09 的 lag1～4 与标准全局 RTS | 精度、相对计算量、曲线及折中分析表/图 |

## 推荐运行顺序

原始 2RTS 与旋转收缩对比：

```matlab
run_rts_navigation_study
run_four_method_dataset_comparison
```

固定滞后研究：

```matlab
run_fixed_lag_rts_study
```

标准单次固定滞后RTS（推荐用于替代分段重复平滑）：

```matlab
run_standard_fixed_lag_rts_study
```

该脚本在每次测距到达后，将最近`lag_segments`个区间组成一个连续窗口，只对窗口执行一次标准RTS反向递推，并只发布窗口中最老的成熟区间。它始终使用原始前向EKF名义轨迹、协方差和状态转移矩阵，不把平滑结果反馈到前向链，也不将已经平滑的轨迹再次送入RTS。默认`lag_segments=2`，对应约14 min窗口和7 min输出延迟。

标准全局 RTS：

```matlab
run_standard_global_rts_study
```

周期全局RTS（默认每42 min执行一次）：

```matlab
run_periodic_global_rts_study
```

该脚本仍然每7 min执行一次测距EKF更新，只将全局RTS的触发频率降低为每6次测距一次。输出采用严格快照口径，只包含最近一次周期触发时已经全局平滑的历史，不用前向EKF填充未触发的末尾。可通过`KF_GINS_GLOBAL_RTS_INTERVAL_S`修改全局反推周期。

周期窗口RTS（默认56 min窗口、每42 min触发）：

```matlab
run_windowed_periodic_global_rts_study
```

窗口始终使用原始前向EKF保存的协方差和状态转移，不把2RTS结果再次送入平滑器。每次触发时只覆盖最近窗口，更早结果冻结；尚未到达下一触发点的尾段暂用2RTS，从而形成完整在线输出。可通过`KF_GINS_GLOBAL_RTS_WINDOW_S`和`KF_GINS_GLOBAL_RTS_INTERVAL_S`分别修改窗口长度与触发周期。

四数据集综合分析（先确保固定滞后和全局结果已生成）：

```matlab
run_all_lags_global_rts_comparison
```

综合分析采用严格共同区间：最大窗口为 lag4 时，排除末尾三个尚未获得完整未来测距的区间。相对计算量按相同 RTS 节点密度估算，以 lag1 为 1，反映算法窗口长度本身的增长，不混入固定滞后 100 Hz 与全局 RTS 1 Hz 节点配置的差异。结果集中写入：

```text
data/inertial-experiment/algorithm-exploration/summary/all-lags-global-rts-comparison
```

标准全局脚本默认读取已有固定滞后结果生成比较表，但不会触发固定滞后计算。缺少固定滞后文件时只跳过比较，不影响全局 RTS 结果生成。可设置：

```matlab
compare_existing_fixed_lag = false;
```

## 去重原则

- 原始 2RTS 的导航主循环只保留在 `run_rts_navigation_study.m`。
- 四方法脚本不再复制或重跑前向 EKF/一次 RTS/二次 RTS；缺少基线时会明确提示先运行原始 2RTS。
- 四方法使用独立的严格 2RTS 文件，不修改原始 2RTS 输出；严格口径会排除末尾仅完成一次 RTS 的区间。
- 标准全局 RTS 不再写出其内部的一次 RTS、二次 RTS 和旋转收缩文件。
- 固定滞后脚本仍独立运行，因为不同 `lag_segments` 需要保存完整的状态、协方差和转移矩阵历史，不能只靠已有导航文件重建。

## 环境变量

各入口支持以下覆盖方式（适用时）：

```text
KF_GINS_DATA_SOURCE
KF_GINS_CASE_NAME
KF_GINS_RTS_DURATION_S
KF_GINS_LAG_SEGMENTS
KF_GINS_GLOBAL_RTS_NODE_INTERVAL_S
KF_GINS_GLOBAL_RTS_INTERVAL_S
KF_GINS_GLOBAL_RTS_WINDOW_S
KF_GINS_COMPARE_FIXED_LAG
KF_GINS_PLOT_SINGLE_VS_GLOBAL
KF_GINS_PLOT_DOUBLE_VS_GLOBAL
KF_GINS_OVERWRITE_RESULTS
```

状态单位定义参见 `STATE-UNITS.md`。

