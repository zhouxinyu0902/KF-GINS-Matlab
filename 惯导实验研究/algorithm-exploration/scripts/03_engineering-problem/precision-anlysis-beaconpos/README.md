# 潜标位置精度分析

本专题研究潜标坐标误差对距离辅助惯导、一次 RTS 和二次 RTS 的影响。
仿真/实测数据和 `rad`/`m` 位置误差状态均由同一套三阶段入口支持。

## 推荐工作流

按下面的顺序运行三个脚本，三个阶段互不隐式调用：

1. `generate_beacon_position_error_data.m`：生成输入；
2. `run_beacon_position_error_study.m`：执行导航并产生结果；
3. `evaluate_beacon_position_error_results.m`：统计并绘图。

数据源在三个脚本顶部通过 `data_source` 选择：

```matlab
data_source = "simulation";  % 或 "experiment"
simulation_case = 'case-00';
```

执行和评价脚本通过下面的变量选择位置误差状态单位：

```matlab
position_error_units = ["rad"];       % 只运行 rad
position_error_units = ["m"];         % 只运行 m
position_error_units = ["rad", "m"]; % 两种都运行
```

## 三个阶段的职责

### 1. 数据生成

`generate_beacon_position_error_data.m` 从三个原始距离文件抽取固定间隔
量测，按照潜标 `1→2→3` 轮换，统一加入距离噪声，然后仅修改距离文件
第 4:6 列的潜标坐标。深度输入也在此阶段一次性生成。

所有潜标误差工况共用相同的距离和深度噪声，确保只比较潜标位置误差。
生成结果包括：

- `input-manifest.csv`：误差工况与输入文件清单；
- `generation-context.mat`：采样间隔、随机种子和噪声配置；
- 每个工况的 `rangedata.txt`；
- 公用的 `heightdata.txt`。

### 2. 导航执行

`run_beacon_position_error_study.m` 只读取上述输入，不再调用数据生成函数。
其单工况导航主循环与 `run_rts_navigation_study.m` 保持同一结构：

- 独立纯惯导链；
- 前向 15 状态 ES-EKF；
- 非对齐测距时刻的 IMU 增量拆分；
- 一次 RTS 和跨相邻区间的二次 RTS；
- `rad`/`m` 对应的距离更新、反馈、深度更新和传播函数。

每个工况输出 `pure-ins.nav`、`range-ins-forward.nav`、
`range-ins-rts-single.nav` 和 `range-ins-rts-double.nav`。运行结束后写出
`study-context.mat`，作为评价阶段唯一入口。

### 3. 结果评价

`evaluate_beacon_position_error_results.m` 只读取导航结果，输出水平位置
RMSE、均值、中位数、P95、最大值、误差曲线、轨迹图和误差—精度响应图。

## 其他文件

- `process_beaconpos.m`：数据生成阶段使用的专题函数，不执行导航；
- `beacon.m`：潜标布放几何与误差传播的独立理论分析，未纳入上述批处理链。
