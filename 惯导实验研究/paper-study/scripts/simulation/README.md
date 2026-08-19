# Paper Study：仿真脚本

本目录只保留四个用途明确的入口脚本，命名统一采用“动作 + 对象”。脚本开头集中放置可修改参数，算法主体仍沿用原论文程序。

| 脚本 | 用途 | 默认数据集 | 主要输出 |
|---|---|---|---|
| `generate_simulation_dataset.m` | 生成轨迹、IMU、真值与三信标距离数据 | Dataset 2 | `input/simulation/dataset*` |
| `run_navigation_simulation.m` | 正式仿真：前向 ES-EKF，可选一次/二次 RTS | Dataset 2 | 数据集导航结果目录 |
| `compare_height_update_methods.m` | 对比无高度约束、直接赋值、量测更新 | Dataset 1 | `height-update-comparison` |
| `run_beacon_observability_analysis.m` | 对比轮换信标和三个固定信标，并分析可观测度 | Dataset 1 | `alt-B1-B2-B3` |

## 使用顺序

```matlab
cd('D:/Github/KF-GINS-Matlab/惯导实验研究/paper-study')
setup_paper_study();

% 需要重新造数据时运行
run('scripts/simulation/generate_simulation_dataset.m')

% 正式算法
run('scripts/simulation/run_navigation_simulation.m')
```

高度与可观测度专题按需单独运行。每个脚本都会自行调用 `setup_paper_study`，因此也可以直接从 MATLAB 编辑器运行。

## 参数说明

- 正式程序中的 `dataset_ids` 控制处理 Dataset 1/2；默认只处理 Dataset 2。
- `enable_smoothing` 控制是否输出一次和二次 RTS，默认关闭。
- 正式程序和可观测度程序保留原论文的 360 行距离抽样间隔，约为 6 min。
- 高度对比使用 420 行间隔，约为 7 min；`add_height_outlier` 控制是否注入 5 m 高度异常。
- 造数据脚本中的 `dataset_id` 控制写入的数据集，运行时会同步保存生成轨迹图。

所有 `.nav` 写入 `data/inertial-experiment/paper-study/navigation-results`；PNG、FIG、CSV、XLSX 和 MAT 写入相同相对层级的 `figures-tables`。
