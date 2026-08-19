# 实测实验脚本说明

本目录保存论文阶段两组实测数据（`input5`/`input6`）的固定实验流程。所有脚本通过 `setup_paper_study.m` 获取输入、函数和输出路径；脚本目录本身不保存大体量输入或导航结果。

## 脚本入口

| 脚本 | 功能 | 主要输出 |
| --- | --- | --- |
| `run_experimental_navigation.m` | 依次处理 Dataset 1/2；统一生成纯惯导、前向测距 EKF、一次 RTS 和二次 RTS | `PureIns.nav`、`ESKF.nav`、`Single-stage RTS.nav`、`Proposed two-stage RTS.nav` |
| `run_experimental_beacon_observability.m` | 比较三信标轮换与固定 B1/B2/B3，并分析轮换模式的有限时域可观测度 | `alt-B1-B2-B3/ES-EKF-*.nav`、`observ.mat` 及可观测度图表 |
| `compare_experimental_height_updates.m` | 比较不更新高度、直接赋值、卡尔曼量测更新 | `heightwayCMP/*.nav`、高度/垂向速度 RMSE 图表 |
| `run_experimental_lbl_constraint.m` | 使用低频带噪三维位置模拟类似长基线定位约束，并生成前向、一次 RTS、二次 RTS 结果 | `ESKF-LBL.nav`、`Single-stage RTS-LBL.nav`、`Proposed two-stage RTS-LBL.nav` |

## 纯惯导合并说明

旧脚本 `exper1_pureins.m` 已合并到 `run_experimental_navigation.m`。纯惯导与测距组合导航使用相同的数据预处理、INS 机械编排和高度更新链，只通过 `solution_modes` 控制是否将测距滤波误差反馈到导航状态：

```matlab
solution_modes = ["pure-ins", "range-aided"];
```

这样可避免两份主循环继续分叉。为兼容旧论文输出，纯惯导结果仍命名为 `PureIns.nav`。

## 统一设置

- 默认处理 `input_ids = 5:6`，对应论文 Dataset 1/2。
- 主实验使用 `rad, rad, m` 的位置误差状态链；保留 `unit_types` 参数，必要时可扩展到 `m, m, m` 链。
- 距离原始文件为 1 Hz，采用 `range_stride = 360`，即约 6 min 一个测距/定位约束时刻。旧注释中的“420 s/7 min”与实际代码不一致，现已修正注释但未改变抽样参数。
- 测距标准差为 6 m，高度/深度标准差为 0.4 m。
- 对比实验在每种方案开始前执行 `rng(1)`，保证各方案使用相同的随机噪声序列。
- 高度仍按当前论文版本采用深度计量测更新；长基线脚本中的三维位置量测由实测真值加噪构造，只用于验证定位约束效果。

## 输入与输出

实测输入默认位于：

```text
F:/2_Data/惯导试验/实验数据/All_data/input5
F:/2_Data/惯导试验/实验数据/All_data/input6
```

导航结果位于：

```text
data/inertial-experiment/paper-study/navigation-results/experiment/
├── dataset1/
└── dataset2/
```

图、表、MAT 诊断文件由 `paper_artifact_dir` 映射到：

```text
data/inertial-experiment/paper-study/figures-tables/experiment/
```

## 建议运行顺序

```matlab
run_experimental_navigation
run_experimental_beacon_observability
compare_experimental_height_updates
run_experimental_lbl_constraint
```

主实验和长基线脚本会执行 RTS 平滑，计算时间和内存占用明显高于高度更新、纯前向可观测度实验。

## 异常后的文件锁

四个脚本均使用 `onCleanup`，新版在运行异常时会自动关闭已经打开的 `.nav` 文件。若异常来自更新前的脚本，MATLAB 会话可能仍持有旧文件句柄，可先执行：

```matlab
fclose('all');
```

然后重新运行相应入口。主实验最后一个测距区间没有未来协方差信息，不能再次执行 RTS；该区间会原样保留已经得到的一次 RTS 轨迹，并补入二次 RTS 输出。
