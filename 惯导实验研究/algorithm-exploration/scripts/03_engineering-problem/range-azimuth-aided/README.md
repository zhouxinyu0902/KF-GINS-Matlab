# 距离–方位角辅助导航验证

本小专题在 `algorithm-exploration` 已有的仿真数据和二次 RTS 框架上，对比：

1. 仅距离（同时使用深度计高度）；
2. 距离 + 相对方位角（同时使用深度计高度）。

两种量测模型分别输出前向 EKF 和二次 RTS 轨迹，距离、深度噪声及 RTS 参数保持一致。

## 文件说明

- `run_range_azimuth_double_rts_comparison.m`：主入口，控制数据生成、导航计算和评价。
- `generate_range_azimuth_measurements.m`：根据 `truth.txt`、`range1.txt`～`range3.txt` 构造方位角观测。
- `add_irregular_azimuth_noise.m`：参考 MEMS 方位角实验，生成与观测方向相关的非均匀噪声。
- `update_range_azimuth_filter_rad.m`：距离、深度和方位角的联合 Kalman 量测更新。
- `feedback_range_azimuth_state.m`：反馈位置、速度、姿态和 IMU 零偏误差。
- `evaluate_range_azimuth_aiding.m`：独立读取结果，输出水平径向误差和轨迹图。

## 方位角定义与数据格式

方位角定义与 `mems-range-navigation/functions/azimuth/pos2azimuth.m` 一致：正北为 0°、正东为 90°，先减载体航向，再减去右舷基线法线偏置 90°，最后归一化到 `[-180°, 180°)`。

生成的 `range1.txt`～`range3.txt` 保留源文件前 6 列，并增加：

| 列 | 含义 | 单位 |
|---:|---|---|
| 7 | 带噪相对方位角 | deg |
| 8 | 真实相对方位角（用于检查，不参与滤波） | deg |
| 9 | 当前方位角噪声标准差 | deg |

量测文件使用度便于检查；进入 Kalman 更新后，方位角残差、噪声和雅可比统一转换为弧度。位置误差状态仍为 `[rad, rad, m]`。

## 为什么必须反馈航向误差

相对方位角由“信标全局方位角 − 载体航向 − 基线偏置”构成，因此它同时约束水平位置和航向。联合更新矩阵包含 `H(3,9) = -1`，必须把第 7～9 维姿态误差反馈到名义姿态。若仍使用原来的 `myErrorFeedback_range`，滤波器虽然估计了航向误差，反馈阶段却会将其清零，方位角误差随后会被错误地解释为位置误差。

## 运行方法

打开并运行：

```matlab
run_range_azimuth_double_rts_comparison
```

主脚本开头可修改：

- `case_name`：选择 `case-00` 等仿真数据集；
- `generate_measurements`：是否重新生成方位角数据；
- `run_navigation`：是否重新计算两组导航结果；
- `run_evaluation`：是否生成统计与图片；
- `azimuth_generation_options`：方位角噪声、随机种子和基线方向。

只重新评价已有结果时，也可执行：

```matlab
evaluate_range_azimuth_aiding('case-00');
```

## 输入与输出

- 原始仿真输入：`data/inertial-experiment/algorithm-exploration/input/simulation/<case>`
- 新方位角输入：`data/inertial-experiment/algorithm-exploration/input/simulation/<case>/range-azimuth-aided`
- 导航结果：`data/inertial-experiment/algorithm-exploration/navigation-results/simulation/<case>/range-azimuth-aided`
- 图片和统计：`data/inertial-experiment/algorithm-exploration/figures-tables/simulation/<case>/range-azimuth-aided`

## case-00 首次验证结果

评价范围为 0.01～4200 s，末尾没有完成二次平滑的 420 s 区间不参与统计。

| 方法 | 水平 RMSE |
|---|---:|
| 前向 EKF，仅距离 | 174.96 m |
| 前向 EKF，距离 + 方位角 | 93.79 m |
| 二次 RTS，仅距离 | 32.09 m |
| 二次 RTS，距离 + 方位角 | 28.00 m |

在该数据集和当前噪声配置下，加入方位角后，前向 EKF 水平 RMSE 降低约 46.4%，二次 RTS 水平 RMSE 降低约 12.8%。这说明联合辅助有效，但结论仍应通过多个 `case-*` 和不同方位角噪声水平继续验证。
