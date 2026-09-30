# 数据生成

本目录提供仿真和实测数据生成入口。

## 实测 case-07 及后续 case-0x 的旧接口兼容数据

`generate_experiment_legacy_inputs.m` 把新版实测目录中的
`range.txt`、`depth_raw.txt`、`truth.nav` 等文件整理为旧算法仍会读取的
`range1.txt` 至 `range3.txt`、`rangedata_noised.txt`、`height.txt` 和
`height_noised.txt`。默认只补齐缺失文件，不覆盖已有结果。

```matlab
generate_experiment_legacy_inputs();                 % 默认 case-07
generate_experiment_legacy_inputs('case-08');        % 后续数据集
generate_experiment_legacy_inputs('case-07', true);  % 明确覆盖
generate_experiment_legacy_inputs('case-07', false, true); % 只检查
```

旧入口 `generate_experiment_dataget.m` 现在等价于为 `case-07` 调用上述
兼容生成器。仿真 `case-05`、`case-06` 的专用生成器保持独立。

## 仿真数据

`generate_simulation_dataget.m` 生成 `case-00` 至 `case-04` 的：

- `IMU_120.txt`；
- `truth.txt`；
- `range1.txt`、`range2.txt`、`range3.txt`。

数据使用当前统一目录：

```text
data/inertial-experiment/algorithm-exploration/
  simulation/case-XX/input/
```

默认调用不会覆盖已有的大体量数据：

```matlab
generate_simulation_dataget();
```

重新生成指定场景：

```matlab
generate_simulation_dataget("case-00", true);
```

一次重新生成多个场景：

```matlab
generate_simulation_dataget(["case-00", "case-01"], true);
```

如果一个场景只剩部分核心文件，函数会停止并要求整组重建，避免混用不同
批次的 IMU、真值和距离数据。每次写入后还会检查文件维度、时间递增性和
三路距离的数据格式。

## 实测数据

`generate_experiment_dataget.m` 用于生成实测高度和三路距离辅助数据。

## case-05：24小时仿真

`generate_simulation_24h_case05.m` 单独生成24小时、100 Hz的IMU和真值，
以及1 Hz的三路理想水平距离。输出位置为：

```text
data/inertial-experiment/algorithm-exploration/
  simulation/case-05/input/
```

直接生成（case-05 已完整存在时不会覆盖）：

```matlab
generate_simulation_24h_case05();
```

确认后覆盖旧的 case-05：

```matlab
generate_simulation_24h_case05(true);
```

只检查路径与参数，不写入数据：

```matlab
generate_simulation_24h_case05(false, true);
```

24小时、100 Hz共有864万行IMU和真值，预计生成约2.0 GB文本。
脚本按1小时分块写入临时文件，全部完成并通过检查后才替换正式文件。

## case-06：case-00 航迹的24小时往返仿真

`generate_simulation_24h_case06.m` 沿用 `case-00` 的转弯轮廓、初始位置和
信标布局。载体走完正向航段后停车、原地掉头，再按相反次序和相反角速度
重走该航段；如此循环直至24小时。数据格式、采样率、分块写入和覆盖保护
与 `case-05` 相同，输出位置为：

```text
data/inertial-experiment/algorithm-exploration/
  simulation/case-06/input/
```

直接生成：

```matlab
generate_simulation_24h_case06();
```

覆盖已有的完整数据：

```matlab
generate_simulation_24h_case06(true);
```

只检查路径和参数：

```matlab
generate_simulation_24h_case06(false, true);
```
