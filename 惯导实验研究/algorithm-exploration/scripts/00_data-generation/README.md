# 数据生成

本目录提供仿真和实测数据生成入口。

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
