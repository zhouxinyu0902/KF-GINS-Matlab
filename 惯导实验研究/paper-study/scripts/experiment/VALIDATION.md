# 实测脚本重构验证记录

验证日期：2026-08-14

## 旧输出备份

重跑前已备份 28 个导航结果文件，共 1,525,291,549 B：

```text
data/inertial-experiment/paper-study/validation-baseline-20260814/
└── navigation-results/experiment/dataset1|dataset2/
```

该目录仅用于本次重构数值对比，未被新脚本覆盖。

## 运行结果

| 入口 | 结果 | 说明 |
| --- | --- | --- |
| `run_experimental_navigation.m` | 通过 | 两组数据的纯惯导、测距 EKF、一次 RTS、二次 RTS 均完整运行；修复最后区间空协方差导致的越界 |
| `run_experimental_beacon_observability.m` | 通过 | Dataset 1/2 各运行轮换、固定 B1/B2/B3；分别得到 14/12 个量测事件 |
| `compare_experimental_height_updates.m` | 通过 | 两组数据三种方式均运行；自动生成 RMSE CSV 和 PNG |
| `run_experimental_lbl_constraint.m` | 通过 | 两组数据的前向、一次 RTS、二次 RTS 均运行；移除对 PSINS 全局 `glv` 的 `d2r` 依赖 |

MATLAB Code Analyzer 对四个入口和 `functions/navigation` 下四个函数检查结果均为 0 条问题。

## 与旧输出的比较

### Dataset 1

- `PureIns.nav`：文件大小一致，SHA-256 完全一致。
- 高度对比的三个文件：SHA-256 全部完全一致。
- `ES-EKF-Alternating.nav`：SHA-256 完全一致。
- `ESKF-LBL.nav`：SHA-256 完全一致。
- LBL 一次/二次 RTS：行数、时间轴、水平位置、速度和姿态一致；仅高度存在微小数值差：
  - 一次 RTS：高度最大差 0.0277 m，RMS 差 0.00811 m；
  - 二次 RTS：高度最大差 0.0280 m，RMS 差 0.00846 m。
- 固定 B1/B2/B3 结果按设计发生变化：新版为每个方案重置 `rng(1)`，使四种信标几何使用同一噪声序列，比较更公平。

### Dataset 2

当前输入/配置生成的结果终止于 `126552.58 s`（431,902 行），旧输出终止于 `127272.58 s`（503,902 行），相差 720 s。因此 Dataset 2 不适合直接做整文件哈希一致性判断。

进一步逐行检查表明，“No-height update” 在首次测距前 35,901 行与旧输出完全一致，首次测距后才分叉；这与当前测距序列长度/噪声抽取次数变化吻合，说明差异来自历史输入或处理时段变化，而非 INS 状态传播重构。

## 已修复问题

1. 从 `scripts/experiment` 直接运行时找不到 `setup_paper_study`；四个脚本现可从任意当前目录启动。
2. `setup_paper_study` 未加入 `function_zxy/Update` 和 `ErrorFeedback`，且论文高度/RTS 函数仍依赖探索专题；现已显式配置并迁入 `functions/navigation`。
3. 主实验和 LBL 的最后区间用 `RTS` 配合空的 `P/PHI` 缓存，导致 `reshape` 越界；现改为用零误差 `Linear` 路径原样写出一次 RTS 结果。
4. 可观测度和 LBL 跨 IMU 帧量测分支使用未定义的 `firstimu/secondimu`；现先插值并传播到精确量测时刻。
5. LBL 脚本调用 PSINS 版 `d2r`，缺少全局 `glv` 时失败；现显式使用 `param.D2R`。
6. 脚本异常退出后可能遗留 `.nav` 文件锁；现通过 `onCleanup` 自动关闭文件。

## 当前会话注意事项

重构期间，旧版主实验曾在交互式 MATLAB 中异常退出，Dataset 1 的 `ESKF.nav`、`Single-stage RTS.nav` 和 `Proposed two-stage RTS.nav` 仍被该 MATLAB 会话锁定。请在该会话执行：

```matlab
fclose('all');
```

再重新运行 `run_experimental_navigation.m`，即可用新版自动关文件保护生成最终主实验输出。
