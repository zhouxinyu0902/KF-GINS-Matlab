# DR_INS 文件清单与职责记录

日期：2026-08-10

## 根目录脚本

| 文件 | 类型 | 当前用途 | 备注 |
|---|---|---|---|
| `main_DR_RANGE.m` | 主脚本 | 单信标 DR/RANGE | 当前有重复循环覆盖输出风险。 |
| `main_DR_RANGE_4beacon.m` | 主脚本 | 四信标逐信标对比 | 当前已有四组 `.nav` 输出。 |
| `PSINS_Dr_RangeDr.m` | 早期脚本 | 基于 PSINS `mydr/myekf` 的距离辅助 DR 验证 | 可作为算法原型参考。 |
| `result_plot.m` | 绘图脚本 | 四信标结果汇总 | 缺少自包含配置变量。 |
| `README_DR_RANGE.txt` | 说明 | 早期说明 | 编码损坏，建议废弃。 |
| `*.asv` | 自动备份 | 无正式用途 | 不建议纳入运行链。 |

## function 目录

| 文件 | 职责 |
|---|---|
| `config_DR_RANGE.m` | 路径、初始状态、噪声、EKF 和反馈配置。 |
| `DRInitialize.m` | 初始化 DR 主状态。 |
| `DRmechanization.m` | DVL + 罗盘 + 深度计航位推算。 |
| `DREKFInitialize.m` | 初始化误差状态 EKF。 |
| `DRinspropagation.m` | EKF 误差状态传播。 |
| `DRRangeUpdate.m` | 水平距离量测更新。 |
| `DRfeedback.m` | EKF 误差反馈到 DR 主状态。 |
| `DRGetClosestIndex.m` | 近邻时间索引查找。 |
| `DRWriteNavLine.m` | 写 `.nav` 结果。 |
| `DRNavDegToNED.m` | `.nav` 转局部 NED。 |

## input 目录

| 路径 | 内容 | 输出状态 |
|---|---|---|
| `input/dataget.m` | 数据生成脚本 | 可生成 `data_<轨迹>_<信标布局>`。 |
| `input/data_line_N_single_side` | 北向直线 + 单侧单信标 | 已有 `output_DR_RANGE`。 |
| `input/data_line_N_four_quadrants` | 北向直线 + 四象限信标 | 已有 `output_DR_RANGE`。 |
| `input/data_lawnmower_single_side` | 割草机 + 单侧单信标 | 已有 `output_DR_RANGE`。 |
| `input/data_circle_single_side` | 圆形 + 单侧单信标 | 未见输出目录。 |
| `input/data_square_single_side` | 方形 + 单侧单信标 | 未见输出目录。 |

## 外部依赖

| 函数/工具 | 来源 | 用途 |
|---|---|---|
| `glvs`, `earth`, `a2mat` | PSINS | 地球参数、姿态矩阵、导航计算。 |
| `trjsimu`, `trjsegment` | PSINS | 数据生成轨迹仿真。 |
| `imuerrset`, `imuadderr` | PSINS | IMU 噪声仿真。 |
| `avpENU2NED`, `imuRFU2FRD` | KF-GINS 项目函数 | 数据格式转换。 |
| `calc_radial_error_gjb` | KF-GINS 项目函数 | 径向误差评估。 |
| `mydr`, `myekf`, `RCompu` | 早期 PSINS 测试函数 | `PSINS_Dr_RangeDr.m` 原型脚本依赖。 |
