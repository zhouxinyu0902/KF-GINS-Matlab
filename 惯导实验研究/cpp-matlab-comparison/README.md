# MATLAB 与 C++ 结果对比

本专题仅用于核对同一组实测数据在 MATLAB 与 C++ 导航程序下的 EKF、一次 RTS 和二次 RTS 结果，不承载新的导航算法。

## 脚本

- `scripts/prepare_cpp_comparison_inputs.m`：按照固定随机种子生成两端共用的加噪距离和高度量测；
- `scripts/compare_cpp_matlab_results.m`：统一绘制六条结果的水平径向误差并导出统计表。

## 数据

```text
data/inertial-experiment/cpp-matlab-comparison/
├─ derived-input/              rangedata_noised、height_noised、C++ YAML
├─ navigation-results/
│  ├─ matlab/                  MATLAB EKF/一次 RTS/二次 RTS
│  └─ cpp/                     C++ EKF/一次 RTS/二次 RTS
└─ figures-tables/             对比图和统计表
```

原始 truth 与传感器文件仍读取 F 盘 `All_data/input6`。外部 C++ 工程默认位于 `D:/Github/KF-GINS-main/dataset_exper6`，但评估脚本只读取已经整理到总 data 下的最终结果，避免依赖 C++ 工程内部不断变化的临时文件名。

原 `exper_6.m` 与全实测数据处理主程序高度重复，且旧插值分支含未定义变量，因此不再保留。MATLAB 导航算法本体统一在 `all-real-data-processing` 专题维护。
