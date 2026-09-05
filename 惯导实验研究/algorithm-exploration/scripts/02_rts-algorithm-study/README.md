# RTS 四方法统一研究

本目录以 `run_four_method_dataset_comparison.m` 作为四方法导航结果的统一生成入口。脚本只负责生成导航结果，不再承担误差统计、绘图或跨数据集汇总。

## 四种方法

1. 前向 ES-EKF；
2. 二次 RTS；
3. 二次 RTS + 分段旋转收缩；
4. 二次 RTS + 水平位置速度约束的固定滞后历史 IMU 重放。

一次 RTS 仍会作为二次 RTS 的中间结果生成并保存，但不列入最终四方法。

## 运行

直接运行：

```matlab
run_four_method_dataset_comparison
```

主要选项位于脚本顶部：

```matlab
data_source = "experiment";          % "experiment" 或 "simulation"
simulation_case = 'case-00';
position_error_unit = "rad";         % "rad" 或 "m"

force_rerun_navigation_core = false; % 优先复用已有前向和二次RTS
overwrite_method_results = false;    % 是否强制重算方法3和方法4
enable_rotation_contraction = true;  % 是否生成旋转收缩结果
enable_position_velocity_replay = true; % 是否生成位置速度约束结果
```

## 已有结果复用

脚本使用与 `run_rts_navigation_study.m` 相同的输出目录和文件名：

- `simple-forward-ekf-<rad|m>.nav`；
- `simple-rts-double-<rad|m>.nav`。

当两者都存在且非空时，`force_rerun_navigation_core = false` 会跳过前向导航、一次 RTS 和二次 RTS，只生成缺少或已经过期的方法 3、方法 4。

方法 3、方法 4 的文件时间如果早于二次 RTS，脚本会自动重新生成，防止复用旧二次 RTS 对应的结果。

## 导航核心

需要补算方法 1、方法 2 时，脚本内部直接运行与当前 `run_rts_navigation_study.m` 同步的完整导航架构：

1. 测距与上一 IMU 历元重合：测距更新 → RTS → 误差反馈 → 前向传播；
2. 测距位于两个 IMU 历元之间：拆分 IMU 增量 → 传播到测距时刻 → 测距更新和 RTS → 反馈 → 传播剩余增量；
3. 普通历元：惯导机械编排 → 解耦高度更新 → EKF 传播。

实测和仿真的距离抽取、距离噪声、深度噪声以及随机数顺序也与当前 `run_rts_navigation_study.m` 保持一致。

## 输出

四种方法结果统一位于：

```text
<case>/output/navigation-results/simple-ekf-rts-<rad|m>
```

主要文件：

```text
simple-forward-ekf-<rad|m>.nav
simple-rts-double-<rad|m>.nav
four-method-rts-double-rotation-<rad|m>.nav
four-method-rts-double-position-velocity-<rad|m>.nav
```

`run_navigation_experiment.m` 目前不能删除，因为 `scripts/03_engineering-problem/common/run_engineering_navigation_case.m` 的实测分支仍在调用它。统一四方法脚本本身不调用该函数。

状态单位定义参见 `STATE-UNITS.md`。
