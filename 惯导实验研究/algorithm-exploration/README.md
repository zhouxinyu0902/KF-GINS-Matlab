# Algorithm Exploration（算法探索专题）

本专题按“研究问题”组织代码，不再按 experiment/simulation 分割算法脚本。仿真与实测仅作为两种数据适配方式，共用 RTS 研究主循环和状态单位选择。

## 目录结构

```text
algorithm-exploration/
├─ config/
│  ├─ experiment/                 实测数据与初始状态配置
│  └─ simulation/                 仿真数据与初始状态配置
├─ functions/
│  ├─ experiment/                 RTS、坐标和高度更新公共函数
│  └─ simulation/                 扩展四方法引擎与历史IMU重放
├─ scripts/
│  ├─ rts-algorithm-study/        EKF、一次/二次RTS及其扩展方法
│  ├─ engineering-problem/        工程问题统一仿真/实测入口
│  │  ├─ precision-anlysis-beaconpos/ 潜标位置误差
│  │  ├─ range-error-study/       测距误差
│  │  ├─ range-delay-study/       测距延迟与补偿
│  │  └─ acoustic-range-dropout-study/ 声学测距掉帧
│  ├─ range-azimuth-aided/        距离+方位角辅助专题
│  ├─ data-generation/            仿真数据生成
│  ├─ data-preparation/           实测输入预处理
│  ├─ gnss-ins-baseline/          GNSS/INS基准
│  └─ evaluation/
│     ├─ rts/                     RTS与状态单位对比
│     ├─ input-data/              输入数据、轨迹和信标绘图
│     ├─ gnss-ins/                GNSS/INS结果评价
│     └─ engineering-problem/     工程问题统一仿真/实测评价
└─ archive/legacy/                不加入运行路径的历史代码
```

## RTS统一入口

打开 `scripts/rts-algorithm-study/run_rts_navigation_study.m`，在文件开头选择数据来源和位置误差状态单位：

```matlab
data_source = "simulation";   % 或 "experiment"
simulation_case = 'case-00';
position_error_unit = "rad";  % 或 "m"
```

该脚本的前向 EKF、一次 RTS 和二次 RTS 主循环只有一份：

- `simulation`：读取 `input/simulation/case-*`，按配置生成带噪距离和高度；
- `experiment`：读取 `input/experiment/case-06` 中已有距离和高度；
- `rad/m`：成套切换配置、传播、量测更新、反馈、高度更新和 RTS 误差应用。

扩展四方法、多数据集批处理和完整实测四方法仍位于同一个 `rts-algorithm-study` 目录，避免把最简主循环塞入大量专题分支。

## 测距延迟专题

`scripts/engineering-problem/range-delay-study` 集中保存：

1. 4 s陈旧距离的敏感性对比；
2. 无延迟、延迟未处理、回退重推进三工况补偿；
3. 生成供统一工程评价读取的结果和上下文。

延迟脚本复用 `rts-algorithm-study/run_navigation_experiment.m`，不会维护另一份实测核心算法。

## 结果评价

通用评价位于 `scripts/evaluation`：

- `rts/compare_radVSm.m`：仿真 rad/m 状态链的严格共同时间段对比；
- `rts/evaluate_experiment_results.m`：实测四方法评价；
- `input-data/`：输入、轨迹与信标场景；
- `gnss-ins/`：GNSS/INS基准评价。
- `engineering-problem/`：潜标位置误差、测距误差和测距延迟的统一评价。

工程误差专题采用同一数据契约：运行脚本写导航结果和
`study-context.mat`，评价脚本自动读取各工况，在共同有效区间内输出
水平径向误差统计、轨迹图和 RMSE 响应曲线。

## 数据与结果目录

输入、导航结果和图表均采用完全相同的“数据来源/数据集”层级：

```text
input/
├─ simulation/case-00 ... case-04/
└─ experiment/case-06/

navigation-results/
├─ simulation/case-00 ... case-04/
└─ experiment/case-06/

figures-tables/
├─ simulation/case-00 ... case-04/
└─ experiment/case-06/
```

IMU、真值、距离和高度等通用文件直接放在 case 根目录。某个专题生成的
专用输入直接放在该 case 下的同名子目录，例如
`input/simulation/case-00/range-azimuth-aided`。专题结果也使用相同名称，
不再额外增加 `engineering-problem` 等中间层。
