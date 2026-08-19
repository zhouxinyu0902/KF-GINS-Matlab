# Algorithm Exploration（算法探索专题）

本专题用于验证惯性/距离组合导航算法是否有效，属于初步探索阶段。代码、配置和专题函数封装在本目录；输入和输出统一放在项目总数据目录。论文阶段固定流程位于同级 `paper-study`。

## 代码目录

```text
algorithm-exploration/
├─ setup_inertial_experiment.m  统一路径入口
├─ config/
│  ├─ experiment/               实测配置（m,m,m 与 rad,rad,m）
│  └─ simulation/               仿真配置
├─ functions/
│  ├─ experiment/               实测更新、传播、反馈与评价函数
│  └─ simulation/               仿真、RTS、旋转收缩等函数
├─ scripts/
│  ├─ experiment/               实测探索脚本
│  ├─ simulation/               仿真与四方法对比脚本
│  └─ evaluation/               结果评价和可视化
└─ archive/                     历史探索代码
```

## 数据目录

```text
data/inertial-experiment/algorithm-exploration/
├─ input/
│  ├─ experiment-raw/
│  ├─ experiment-preprocessed/
│  ├─ experiment-reference/
│  └─ simulation/case-*/
├─ navigation-results/
│  ├─ experiment/
│  └─ simulation/
└─ figures-tables/
   ├─ experiment/
   └─ simulation/
```

- `navigation-results` 只保存 `.nav`。
- `figures-tables` 保存图片、表格、MAT 文件和诊断记录。
- 两类目录保持相同的实验/仿真及方法子目录，因此可按相对路径一一对应。

## 使用

```matlab
cd('D:/Github/KF-GINS-Matlab/惯导实验研究/algorithm-exploration')
paths = setup_inertial_experiment();
```

常用入口包括 `scripts/experiment/run_experiment_four_method_comparison.m`、`run_experiment_four_method_comparison_rad.m`、`scripts/simulation/run_four_method_dataset_comparison.m` 和 `run_fixed_lag_four_method_dataset_comparison.m`。脚本产生导航结果后，`exploration_artifact_dir` 会把评估产物自动写到相同相对目录的 `figures-tables`。

`README-legacy.md` 仅保留重构前的完整说明用于追溯，其中旧路径不再作为当前运行依据。
