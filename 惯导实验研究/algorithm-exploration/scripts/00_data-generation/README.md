# 仿真数据生成

本目录统一管理 `case-00` 至 `case-04` 的仿真 IMU、真值和三信标距离数据生成。

## 文件

- `run_generate_simulation_data.m`：可直接运行的数据生成入口脚本；
- `generate_simulation_data.m`：参数化数据生成器，替代原来的 `simu_dataget.m` 至 `simu_dataget5.m`；
- `plot_simulation_cases.m`：为已有或新生成的 case 输出轨迹—信标图和三信标距离曲线图。

## 输出位置

- 数据：`data/inertial-experiment/algorithm-exploration/input/simulation/case-XX`；
- 图片：`data/inertial-experiment/algorithm-exploration/figures-tables/simulation`。

每个场景生成四个图片文件：

- `case-XX-trajectory-beacons.png/.fig`；
- `case-XX-beacon-ranges.png/.fig`。

## 使用方式

在 MATLAB 中直接运行 `run_generate_simulation_data.m`，即可处理 `case-00` 至 `case-04`。默认保留已有数据，只生成缺失数据，并更新全部图片。

如需在其他脚本中按场景调用函数，可使用：

```matlab
paths = setup_inertial_experiment();
generation_dir = fullfile(paths.topic, 'scripts', 'simulation', ...
    'data-generation');
addpath(generation_dir);

% 默认：保留已有数据，只生成缺失数据，并更新全部图片。
generate_simulation_data();

% 只处理指定场景；true 表示允许覆盖已有大体量数据。
generate_simulation_data("case-01", true);

% 不生成数据，只为已有场景补图。
plot_simulation_cases("case-01");
plot_simulation_cases();
```

正式场景只有 `case-00` 至 `case-04`。历史 `simu_dataget5.m` 是信标旋转角探索稿，没有形成独立 `case-05`，其信标布局试验可通过修改 `generate_simulation_data.m` 中的场景配置复现。

