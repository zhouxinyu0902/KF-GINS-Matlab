# 03 四信标 PSINS 与本地 DR/RANGE 对比修改记录

日期：2026-08-10

## 修改目标

针对 `input/data_line_N_four_quadrants` 数据集：

1. 在 `PSINS_Dr_RangeDr.m` 中分别使用 4 个信标单独辅助 DR，并输出 PSINS 版本结果。
2. 使用 `main_DR_RANGE_4beacon.m` 分别使用 4 个信标单独辅助 DR，并输出本地拆分函数版本结果。
3. 使用 `result_plot.m` 对比 PSINS 与本地版本的四信标结果。

## 输出目录约定

- PSINS 版本输出：`input/data_line_N_four_quadrants/output_psins`
- 本地版本输出：`input/data_line_N_four_quadrants/output_here`
- 对比图与统计：`input/data_line_N_four_quadrants/output_compare`

## 修改文件

### `PSINS_Dr_RangeDr.m`

- 输入数据切换为 `data_line_N_four_quadrants`。
- 循环读取 `beacon_1.txt` 到 `beacon_4.txt`。
- 每个信标单独运行一次 PSINS `mydr + myekf` 距离辅助流程。
- 输出统一 `.nav` 格式，便于与本地版本共同对比。
- 输出文件：
  - `PSINS-Origin-DR-1.nav` 到 `PSINS-Origin-DR-4.nav`
  - `PSINS-DR-RANGE-1.nav` 到 `PSINS-DR-RANGE-4.nav`
  - `PSINS-DR-RANGE-state-*.txt`
  - `PSINS-DR-RANGE-innovation-*.txt`

### `main_DR_RANGE_4beacon.m`

- 输出目录改为 `output_here`。
- 每个信标循环都从原始 `dvl/compass/depth/beacon` 数据重新裁剪，避免前一轮裁剪影响后一轮。
- 输出 state 和 innovation 记录，文件名带信标编号。
- 输出文件：
  - `Origin-DR-1.nav` 到 `Origin-DR-4.nav`
  - `DR-RANGE-1.nav` 到 `DR-RANGE-4.nav`
  - `DR-RANGE-state-*.txt`
  - `DR-RANGE-innovation-*.txt`

### `result_plot.m`

- 改为自包含脚本，不依赖外部工作区变量 `cfg` 或 `outputfolder`。
- 同时读取 `output_here` 和 `output_psins`。
- 自行计算水平径向误差，避免外部 `calc_radial_error_gjb` 的 6 条曲线颜色表限制。
- 输出：
  - `four_beacon_here_vs_psins_radial_error.png`
  - `four_beacon_here_vs_psins_stats.xlsx`
  - `four_beacon_here_vs_psins_stats.csv`

## 运行顺序

```matlab
cd('D:\Github\KF-GINS-Matlab\graduation\DR_INS')
PSINS_Dr_RangeDr
main_DR_RANGE_4beacon
result_plot
```

## 注意事项

- 当前两套流程都是“4 个信标分别单独辅助”的对比，不是 4 个信标同时联合更新。
- 两套流程都使用 `range_interval = 2` 对信标观测下采样，保持对比条件一致。
- PSINS 脚本输出的 `PSINS-Origin-DR-*.nav` 理论上只与传感器数据有关，不随信标变化；保留编号是为了文件组织一致。

## 本次执行验证

已按以下顺序执行完成：

```matlab
PSINS_Dr_RangeDr
main_DR_RANGE_4beacon
result_plot
```

生成结果已确认存在：

- `output_psins`：4 组 `PSINS-DR-RANGE-*.nav`、4 组 `PSINS-Origin-DR-*.nav`、state 和 innovation 文件。
- `output_here`：4 组 `DR-RANGE-*.nav`、4 组 `Origin-DR-*.nav`、state 和 innovation 文件。
- `output_compare`：对比图、xlsx 统计表和 csv 统计表。

统计表路径：

```text
input/data_line_N_four_quadrants/output_compare/four_beacon_here_vs_psins_stats.csv
```

本次运行得到的 RMS 水平径向误差摘要：

| 系列 | RMS / m |
|---|---:|
| Here-Origin | 19.1422 |
| Here-B1 | 6.2082 |
| Here-B2 | 3.6367 |
| Here-B3 | 14.0855 |
| Here-B4 | 21.5154 |
| PSINS-Origin | 19.0765 |
| PSINS-B1 | 33.3651 |
| PSINS-B2 | 48.6161 |
| PSINS-B3 | 92.5528 |
| PSINS-B4 | 56.7962 |

初步结论：本地 `DRRangeUpdate/DRfeedback` 版本在 B1/B2 信标下明显改善纯 DR；PSINS `mydr/myekf` 版本按原脚本的 `avp_range(:,7:8) = avp_range(:,7:8) - x(3:4)` 逻辑复现后，四个信标辅助结果均劣于纯 DR。后续若要让 PSINS 版本作为公平基准，需要单独检查 `myekf` 状态定义、创新符号和位置反馈符号。

## 路径与配置一致性修订

按最新要求，输出目录从数据集子目录提升到 `DR_INS` 根目录一级：

- PSINS 版本输出：`D:\Github\KF-GINS-Matlab\graduation\DR_INS\output_psins`
- 本地版本输出：`D:\Github\KF-GINS-Matlab\graduation\DR_INS\output_here`
- 对比输出：`D:\Github\KF-GINS-Matlab\graduation\DR_INS\output_compare`

同时对 PSINS 与 here 配置做一致化处理：

- 两者均使用 `input/data_line_N_four_quadrants` 作为输入数据集。
- 两者均调用 `config_DR_RANGE(input_dir)` 获取公共配置。
- 两者均使用 `cfg.range_std` 作为距离量测噪声。
- 两者均使用 `cfg.range_time_tolerance` 作为测距时间对齐容差。
- 两者均使用 `range_interval = 2` 对 `beacon_*.txt` 下采样。
- PSINS 初始水平位置改为由 `cfg.pos0` 转换得到，不再使用独立的 `[1;1;0.2]` 初始误差硬编码。
- PSINS EKF 初始方差与过程噪声改为来自 `cfg.kf.*`，与 here 的 `DREKFInitialize` 设置保持一致。

已重新执行：

```matlab
PSINS_Dr_RangeDr
main_DR_RANGE_4beacon
result_plot
```

根目录级输出已确认生成。

本次根目录级输出的 RMS 水平径向误差摘要：

| 系列 | RMS / m |
|---|---:|
| Here-Origin | 19.1422 |
| Here-B1 | 6.2082 |
| Here-B2 | 3.6367 |
| Here-B3 | 14.0855 |
| Here-B4 | 21.5154 |
| PSINS-Origin | 19.1582 |
| PSINS-B1 | 6.7109 |
| PSINS-B2 | 3.5398 |
| PSINS-B3 | 13.9709 |
| PSINS-B4 | 21.2793 |

修订后，PSINS 与 here 的总体趋势已基本一致。B1/B2 对纯 DR 有明显改善，B3 接近中等改善，B4 不优于纯 DR。

## 绘图线型区分修订

按最新要求，`result_plot.m` 的曲线样式调整为：

- Here 系列使用实线 `-`。
- PSINS 系列使用虚线 `--`。
- 同一类别使用同一颜色：Origin、B1、B2、B3、B4 各自对应固定颜色，便于直接观察 Here 与 PSINS 的同信标差异。
- 图例调整为 2 列显示，减少 10 条曲线同时显示时的遮挡。

已重新运行 `result_plot.m`，更新输出图：

```text
D:\Github\KF-GINS-Matlab\graduation\DR_INS\output_compare\four_beacon_here_vs_psins_radial_error.png
```

PSINS 与 here 结果不完全一致的原因：当前只统一了输入数据、初始位置、测距噪声、测距时间容差和下采样间隔；两套程序的机理编排、滤波状态定义、误差反馈方式、时间推进/量测更新顺序、位置误差符号约定和数值插值细节仍来自不同实现，因此结果应当趋势接近，但不应预期逐点完全相同。
