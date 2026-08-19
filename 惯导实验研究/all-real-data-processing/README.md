# 全实测数据处理

本专题用于统一处理 F 盘 `All_data/input1` 至 `input8` 的八组实测惯导数据。F 盘只保留原始输入；代码产生的 truth、导航结果、图片、表格及诊断文件统一保存到项目总 `data` 目录。

## 目录

```text
all-real-data-processing/
├─ config/
│  └─ create_real_dataset_config.m       rad/m 共用配置
├─ functions/navigation/                 本专题使用的高度更新与 RTS 函数
├─ scripts/
│  ├─ run_all_real_datasets.m            1–8 组数据统一导航入口
│  ├─ preprocessing/
│  │  └─ build_dataset_truth.m           构造 truth 与 GNSS-RTK 派生数据
│  ├─ experiments/
│  │  └─ run_missing_range_experiment.m  测距点缺失敏感性实验
│  └─ analysis/
│     ├─ plot_all_dataset_error_summary.m 八组结果汇总图
│     └─ compare_rts_linear_dataset6.m    第 6 组历史桥接结果对比
└─ setup_all_real_data_processing.m
```

对应数据目录：

```text
data/inertial-experiment/all-real-data-processing/
├─ derived-input/dataset1...dataset8/    truth、GNSS-RTK 等派生输入
├─ navigation-results/dataset1...dataset8/  .nav 轨迹
└─ figures-tables/dataset1...dataset8/    图片、表格、CSV、MAT、FIG
```

## 运行方法

1. 打开专题目录，运行 `setup_all_real_data_processing`。
2. 在目标脚本顶部修改数据编号和运行开关。
3. 运行脚本。

`run_all_real_datasets.m` 默认处理 1–8 组数据，使用 `rad` 状态链，同时生成纯惯导、前向 ESKF、一次 RTS 与二次 RTS。若要检查米制水平位置误差状态，将 `unit_types` 改为 `["rad", "m"]`。

主要配置：

- `range_stride = 420`：距离量测间隔，原始距离文件为 1 Hz 时对应 7 min；
- `range_std_m = 6`：距离噪声标准差；
- `depth_std_m = 0.4`：深度/高度噪声标准差；
- `end_time_override = []`：空值表示处理到该组数据末尾，也可指定统一截止时刻；
- `solution_modes`：`pure-ins` 与 `range-aided` 可分别启停。

## 重构说明

- 原 `alll_overall.m`、`all_pureins.m`、`all_m.m`、`all_rad.m` 已合并为一个可配置入口；
- 原 `config_1.m`、`config_1_m.m`、`ProcessConfig_truth.m` 已合并为一个配置函数；
- 完全重复的两个八组汇总绘图脚本仅保留一份；
- truth 构造不再向 F 盘写结果，生成内容进入 `derived-input/datasetN`；
- 历史输出按 `.nav` 与图表分析产物拆分存放，便于批量评估和清理。

## 迁移状态

2026-08-14 已将历史内容复制到新结构并完成逐文件长度校验：108 个数据文件，共 2,902,712,863 字节。随后按整理要求删除旧 `new_惯导试验/all_process` 及重复脚本，不再保留旧代码备份。
