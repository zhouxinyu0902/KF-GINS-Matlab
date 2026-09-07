`03_sum/2013` 这一组代码的逻辑可以理解成三层：**原始实测数据预处理 → 构造距离辅助实验数据 → 用 DR/EKF 做距离辅助导航实验**。它不是一个完全工程化的单入口项目，更像你早期研究过程中逐步堆出来的一套实验脚本。

**总体主线**

```text
Simulation_trj.m
    -> data_1/avp.mat
    -> Simu1_Dr_And_Ins.m
        -> data_1/data_dr.mat
        -> Simu2_Dr_RangeDr.m / Simu3_Dr_RangeDr_SLBL.m

Exper0_PreData.m
    -> data_1/deep-sea.mat
    -> Exper0_PreData_PreWork.m
        -> data_1/DataNeed.mat
        -> Exper1_Dr_Range.m / Exper1_Dr_Range_2beacons.m

Exper0_PreData_optimized.m
    -> data_1/deep-sea_optimized.mat
    -> 后来 paper 目录继续使用
```

**1. 原始实测数据预处理**

`Exper0_PreData.m` 是最早的大脚本，功能很全，也最乱。它从原始 txt/log 数据开始，完成：

| 阶段 | 做什么 |
|---|---|
| 舱内数据导入 | 读取罗盘、Octans、DVL、深度、LBL 等 AUV 数据 |
| USBL 数据导入 | 读取 PTSAG、PTSAX、PIXOG 日志 |
| 时间段截取 | 统一 LBL 和 USBL 的时间段 |
| 时间同步 | 处理时区、重复时间点、采样对齐 |
| 姿态/深度/DVL 处理 | 得到 `compass`、`octans`、`vxy`、`depther` |
| LBL 处理 | 平滑 LBL 定位结果，构造 `avp_m` |
| USBL 处理 | 处理 PTSAG 绝对位置、PTSAX 相对位移、PIXOG 传播时间 |
| 声学+DR 融合 | 调用 `AcousticDeadR` 得到 `avp_LBL_DR` 等融合轨迹 |
| 测距计算 | 得到 `RNG`、`RNG_raw`、传播时间补偿距离等 |
| 保存数据 | 保存到 `data_1/deep-sea.mat` |

`Exper0_PreData_seeonly.m` 是一个“边看边验证”的版本。它把很多分析、绘图、清洗函数直接写在脚本末尾，比如：

| 局部函数 | 作用 |
|---|---|
| `solve_lbl_with_depth` | 用四个 LBL 信标距离 + 深度约束反解 AUV 位置 |
| `clean_v` | 清洗速度 |
| `AcousticDeadR_mod` | 局部版声学定位 + DR 融合 |
| `plot_nav_analysis` | 统一做轨迹/误差统计 |
| `smooth_outlier_cleaner_pchip` | 平滑趋势 + 异常点识别 + PCHIP 插值修复 |

`Exper0_PreData_optimized.m` 是后来的模块化版本。它把原来 `Exper0_PreData.m` 的大段代码拆到了 `03_sum/func_1` 中，逻辑最清晰：

```matlab
load_nav_raw_data
sync_nav_time
process_auv_sensors
process_acoustic_data
integNAV
eval_acoustic_ranges
save deep-sea_optimized.mat
```

所以如果你以后整理论文代码，`Exper0_PreData_optimized.m` 更适合作为预处理主入口。

**2. DataNeed 构造逻辑**

`Exper0_PreData_PreWork.m` 是连接“预处理结果”和“距离辅助实验”的桥。它读取：

```matlab
load('03_sum\data_1\deep-sea.mat')
```

然后构造后续实验统一使用的 `DataNeed.mat`。

它主要做几件事：

| 变量 | 含义 |
|---|---|
| `avp_LBL_DR` | LBL + DR 融合轨迹，高频参考 |
| `avp_ref` | 从 `avp_LBL_DR` 每 16 点抽取一次，对应 8 s 低频声学时刻 |
| `avp_dr` | 纯航位推算轨迹 |
| `BCN` | 信标位置集合 |
| `RNG` | 不同信标/不同方法下的水平距离观测 |
| `RNG1` | 由 LBL 定位结果反算的距离 |
| `RNG2` | 由参考轨迹计算并加噪的理想距离 |
| `vxy` / `compass` / `depther` | 后续 DR/EKF 的传感器输入 |

这里一个重要采样逻辑是：

```matlab
ID = 1:16:16*LenUSBL;
```

因为主导航递推是 `0.5 s`，声学测距是 `8 s`，所以 `16` 个 0.5 s 点对应一个声学观测点。

它还人为构造了多种信标情形：

| 信标编号 | 类型 |
|---|---|
| `BCN{1}~BCN{4}` | 原始 LBL 固定信标 |
| `BCN{5}` | 人工设置的理想固定信标 |
| `BCN{6}` | 使用母船位置构造的移动信标 |
| `BCN{7}` | 使用 USBL 换能器轨迹作为移动信标 |
| `BCN{8}~BCN{10}` | PTSAG、PTSAX、PIXOG 三类 USBL 方法得到的移动信标距离 |
| `BCN{11}~BCN{14}` | 把原始固定信标人为平移后的新信标，用来比较几何布局影响 |

最终保存：

```matlab
save 03_sum\data_1\DataNeed.mat ...
```

后面的 `Exper1_Dr_Range*.m` 就都依赖它。

**3. 实测距离辅助实验**

`Exper1_Dr_Range.m` 是单信标距离辅助 DR。

核心循环是：

```text
每 0.5 s：
    mydr update，做纯 DR 推算
    myekf fk + algo('T')，做 EKF 时间更新

每 8 s：
    取当前信标距离观测
    计算 DR 到信标的预测距离
    残差 = 预测距离 - 测量距离
    myekf hk + algo('M')，量测更新
    用估计出的纬经度误差修正 avp_range
```

它对 `id = 1:4` 的四个固定信标逐个做实验，最后用：

```matlab
calc_radial_error_avp(avp_ref, label_1, avp_dr, avp_RNG1{bbb})
```

比较纯 DR 和各信标辅助后的径向误差。

`Exper1_Dr_Range_2beacons.m` 是双信标辅助。它遍历六种组合：

```matlab
[1,2; 1,3; 1,4; 2,3; 2,4; 3,4]
```

量测从一维距离残差变成二维：

```matlab
kf.yk = [r_dr1 - r1;
         r_dr2 - r2]
```

调用的是：

```matlab
myekf('hk', kf, dr, '2range')
```

最后比较不同双信标组合的径向误差，并用 `xk_plot` 看 EKF 输出状态。

**4. 仿真链路**

`Simulation_trj.m` / `Simulation_trj_5m.m` 负责生成仿真轨迹。它们用 PSINS 的 `trjsegment` 和 `trjsimu` 构造轨迹，保存：

```matlab
data_1/avp.mat
data_1/avp_5m.mat
```

`Simu1_Dr_And_Ins.m` 做两件事：

| 部分 | 作用 |
|---|---|
| DR 仿真 | 从真实轨迹生成带误差的 compass、DVL、depth，再用 `mydr` 推纯 DR |
| INS 仿真 | 给 IMU 加误差，用 `myins` 做惯导解算 |

它会保存：

```matlab
data_1/data_dr.mat
```

这个文件是后续 `Simu2` 和 `Simu3` 的基础。

`Simu2_Dr_RangeDr.m` 是仿真条件下的主距离辅助脚本。它调用 `beacon_gen` 生成固定/移动信标与距离，先做单个信标实验，然后批量比较：

```matlab
for ii = [1:4, 9]
```

也就是四个固定信标 + 一个移动信标。它还额外计算了统计指标：

| 指标 |
|---|
| 平均误差 |
| 最大误差 |
| RMS |
| 标准差 |
| 中位数 |
| 95% 分位 |
| 相对 DR 改进率 |

`Simu3_Dr_RangeDr_SLBL.m` 和 `Simu2` 类似，但更偏向研究“测距来临后立即反馈”的策略。它里面明确分了两种思路：

| 策略 | 逻辑 |
|---|---|
| 后处理修正 | EKF 估计出位置误差，最后统一修正 `avp_range(:,7:8)` |
| 在线反馈 | 每次量测更新后，立即 `dr.pos(1:2) = dr.pos(1:2) - kf.xk(3:4)`，再把位置误差状态清零 |

这就是它看起来有重复代码块的原因：不是完全重复，而是在试不同反馈方式。

**核心算法模式**

`2013` 目录的距离辅助核心基本都是这个模板：

```text
1. 初始化 DR：
   dr = mydr('init', 初始位置, 初始误差, ts)

2. 初始化 EKF：
   kf = myekf('init', ts, x0, dx0, vk, rk)

3. 高频递推：
   dr = mydr('update', dr, depth, yaw, vxy)
   kf = myekf('fk', kf, dr)
   kf = myekf('algo', kf, 'T')

4. 低频声学量测：
   计算预测距离 kf.r_dr
   构造残差 kf.yk
   kf = myekf('hk', kf, dr, 'range' 或 '2range')
   kf = myekf('algo', kf, 'M')

5. 轨迹修正：
   either 后处理修正 avp_range
   or 在线反馈修正 dr.pos
```

常见状态量是 4 维：

```text
[ DVL刻度因子误差, 航向误差, 纬度误差, 经度误差 ]
```

**我对这组代码的判断**

`2013` 是你的“原始实验生长区”：逻辑完整，但有明显研究过程痕迹。最重要的数据生产链是：

```text
Exper0_PreData.m -> deep-sea.mat
Exper0_PreData_PreWork.m -> DataNeed.mat
Exper1_Dr_Range*.m -> 实测距离辅助分析
```

仿真链是：

```text
Simulation_trj.m -> avp.mat
Simu1_Dr_And_Ins.m -> data_dr.mat
Simu2/Simu3 -> 仿真距离辅助分析
```

而 `Exper0_PreData_optimized.m` 是你后来为了论文整理出来的更干净版本，它已经把很多逻辑迁移到 `func_1`，更适合作为后续维护入口。