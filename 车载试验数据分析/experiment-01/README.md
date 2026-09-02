# 第一次车载试验数据处理

本专题集中保存第一次车载试验八组数据的导出、检查、参考轨迹构造、
惯性/测距组合导航和结果汇总代码。输入、导航结果与图表统一放在
`data/experiment-data/experiment-01`，不再依赖原
`惯导实验研究/all-real-data-processing` 专题。

## 代码结构

```text
experiment-01/
├─ data_process/                       原始 MAT/TXT 导出和输入检查
├─ functions/                          本专题导航辅助函数
├─ setup_experiment_01.m               统一路径入口
├─ create_real_dataset_config.m        rad/m 配置构造
├─ range_dataget.m                      八组测距生成及信标—轨迹对比图
├─ run_all_real_datasets.m             八组数据导航入口
├─ build_dataset_truth.m               构造 truth 与 GNSS-RTK 文件
├─ compare_rts_linear_case06.m         第6组历史结果对比
└─ plot_all_dataset_error_summary.m    八组误差汇总
```

## 数据结构

```text
data/experiment-data/experiment-01/
├─ case-01 ... case-08/
│  ├─ raw/                 原始导出文件
│  ├─ mat/                 MAT 中间文件
│  ├─ input/               导航通用输入和 truth
│  ├─ navigation-results/  NAV 结果
│  └─ figures-tables/      图片、表格和诊断文件
└─ summary/                八组数据汇总图表
```

## 使用方法

1. 运行 `setup_experiment_01` 检查八组输入和配置路径。
2. 如需重建真值，运行 `build_dataset_truth.m`。
3. 运行 `range_dataget.m`，根据每组 `truth.nav` 生成 `range1.txt`～
   `range3.txt`。三个距离文件写入对应的 `case-*/input`，8 组轨迹与
   信标的 2×4 对比图写入 `summary`。脚本顶部的 `align_modes` 可按组
   切换 `right` 或 `bottom`，当前全部默认为 `right`。
4. 在 `run_all_real_datasets.m` 开头设置 `input_ids`、单位、运行模式、
   测距间隔和截止时长，然后运行导航解算。
5. 运行两个分析脚本生成单组或八组汇总图。

`end_time_override=[]` 表示使用该组数据的完整有效时段；设置数值时，
该数值表示与输入文件一致的绝对截止时刻。

原有测距点缺失脚本已经独立为 algorithm-exploration 下的
`acoustic-range-dropout-study`，不再维护一份重复的导航核心。
