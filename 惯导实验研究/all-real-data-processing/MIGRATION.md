# all_process 迁移记录

迁移日期：2026-08-14。

## 代码映射

| 原内容 | 新内容 | 处理方式 |
|---|---|---|
| `alll_overall.m`、`all_pureins.m` | `scripts/run_all_real_datasets.m` | 合并运行模式 |
| `all_m.m`、`all_rad.m` | `scripts/run_all_real_datasets.m` | 合并单位开关 |
| `config_1.m`、`config_1_m.m`、`ProcessConfig_truth.m` | `config/create_real_dataset_config.m` | 合并配置 |
| `truthget.m` | `scripts/preprocessing/build_dataset_truth.m` | 输出改到总 data |
| `loss_range.m` | `scripts/experiments/run_missing_range_experiment.m` | 移除旧专题硬编码路径 |
| 两份相同的 `plot_8rounds_error_summary*.m` | `scripts/analysis/plot_all_dataset_error_summary.m` | 去重 |
| `output/output6/result.m` | `scripts/analysis/compare_rts_linear_dataset6.m` | 规范命名并更新路径 |

## 数据映射

- 历史 `.nav`：`data/inertial-experiment/all-real-data-processing/navigation-results/datasetN`；
- PNG、FIG、XLSX、CSV、MAT：`.../figures-tables/datasetN`；
- 八组汇总图：`.../figures-tables/summary`；
- `GNSS_RTK.txt` 与 `GNSS_RTK_1s.txt`：`.../derived-input/dataset6`。

本次共复制并校验 108 个数据文件，合计 2,902,712,863 字节。F 盘原始输入未移动、未修改。

## 清理状态

完成文件数量与长度校验后，旧 `new_惯导试验/all_process` 已随整个旧专题目录删除。重复入口和旧脚本不再保留备份。
