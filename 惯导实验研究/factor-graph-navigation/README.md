# 因子图组合导航

本专题整理原 `new_惯导试验/fgo_navigation`，用于研究 GNSS/INS 与稀疏测距/INS 的批量因子图、固定滞后因子图及对应 KF/RTS 基线。

## 入口

- `scripts/gnss-ins/run_gnss_ins_fgo_demo.m`：GNSS 量测构造与单轮示例；
- `scripts/gnss-ins/run_gnss_ins_fgo_standard.m`：迭代重线性化 GNSS/INS 因子图；
- `scripts/gnss-ins/run_kf_gnss_ins_baseline.m`：GNSS/INS KF 基线；
- `scripts/range-ins/run_range_ins_fgo_batch.m`：批量 RANGE/INS 因子图；
- `scripts/range-ins/run_range_ins_fgo_fixed_lag.m`：完整 15 状态固定滞后因子图；
- `scripts/range-ins/run_kf_range_ins_baseline.m`：1 s 高度更新的 KF/RTS 基线；
- `scripts/analysis/compare_factor_graph_results.m`：统一误差评估。

## 数据

输入与 `paper-study` 的仿真 Dataset 1 逐文件哈希相同，直接复用：

```text
data/inertial-experiment/paper-study/input/simulation/dataset1
```

本专题输出位于：

```text
data/inertial-experiment/factor-graph-navigation/
├─ derived-input/       simulated GNSS 等派生量测
├─ navigation-results/  KF、RTS 与 FGO 导航结果
└─ figures-tables/      对比图片与统计表
```

先运行 `setup_factor_graph_navigation`，再运行相应脚本。当前实现为项目内自编因子图求解，不依赖 GTSAM。

## 已删除的重复版本

- `Copy_of_kf_range_ins.m`：纯惯导临时副本；
- `kf_range_ins.m`：被 1 s 高度更新版本替代；
- `fgo_range_ins_realtime_int15.m`：被完整 15 状态 revised 固定滞后版本替代。
