# Paper Study（论文实验专题）

本专题保存论文阶段已经收敛的实测、仿真、可观测度分析与论文图表导出代码。它与同级的 `algorithm-exploration` 分开：前者服务论文复现与固定流程，后者用于算法有效性的初步探索。

## 代码目录

```text
paper-study/
├── setup_paper_study.m          统一配置代码、输入和输出路径
├── config/                      实测与仿真配置
├── functions/                   本专题使用的算法、可观测度及绘图函数
└── scripts/
   ├── experiment/               F 盘实测 Dataset 1/2 处理
   ├── simulation/               仿真数据生成与算法处理
   ├── analysis/                 论文图片和表格统一导出
   └── legacy/                   历史绘图脚本，仅用于追溯
```

专题代码目录不再保存输入和输出。数据统一位于：

```text
data/inertial-experiment/paper-study/
├── input/simulation/dataset1|dataset2/
├── navigation-results/
│  ├── experiment/dataset1|dataset2/
│  └── simulation/dataset1|dataset2/
└── figures-tables/
   ├── experiment/dataset1|dataset2/
   ├── simulation/dataset1|dataset2/
   └── paper-artifacts/figures|tables/
```

- `navigation-results` 只保存 `.nav` 导航轨迹结果。
- `figures-tables` 保存 PNG、FIG、PDF、CSV、XLSX、MAT 和诊断文本等评价产物。
- 实测 Dataset 1/2 的原始输入仍分别使用 `F:/2_Data/惯导试验/实验数据/All_data/input5` 和 `input6`，不重复复制到仓库。

## 主要入口

### 实测实验

- `scripts/experiment/run_experimental_navigation.m`：两组数据的纯惯导、前向 EKF、一次/二次 RTS；
- `scripts/experiment/run_experimental_beacon_observability.m`：固定/轮换信标及可观测度分析；
- `scripts/experiment/compare_experimental_height_updates.m`：高度更新方式对比；
- `scripts/experiment/run_experimental_lbl_constraint.m`：类似长基线定位结果的位置约束实验；
- `scripts/experiment/README.md`：实测脚本、参数和输出的详细说明。

### 仿真实验

- `scripts/simulation/generate_simulation_dataset.m`：生成轨迹、IMU、真值与测距输入；
- `scripts/simulation/run_navigation_simulation.m`：论文仿真主算法；
- `scripts/simulation/compare_height_update_methods.m`：高度更新方式对比；
- `scripts/simulation/run_beacon_observability_analysis.m`：固定/轮换信标及可观测度分析。

### 结果分析

- `scripts/analysis/result_exper.m`：实测论文图表；
- `scripts/analysis/result_simu.m`：仿真论文图表。

## 使用

```matlab
cd('D:/Github/KF-GINS-Matlab/惯导实验研究/paper-study')
paths = setup_paper_study();
```

若实测数据盘符或目录改变，只需修改 `setup_paper_study.m` 中的 `external_experiment_root`。算法脚本使用 `paths.output_*` 写导航结果；图表和诊断文件通过 `paper_artifact_dir` 自动映射到相同数据集的 `figures-tables` 目录。
