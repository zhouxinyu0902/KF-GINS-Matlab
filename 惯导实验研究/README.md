# 惯导实验研究

该目录现在按研究用途划分为六个平级专题：

```text
惯导实验研究/
├─ algorithm-exploration/       初步算法探索与有效性验证
├─ paper-study/                 论文阶段固定实验与图表复现
├─ all-real-data-processing/    八组实测数据的统一预处理、导航与评估
├─ cpp-matlab-comparison/       MATLAB 与 C++ 导航结果一致性对比
├─ factor-graph-navigation/     GNSS/INS 与 RANGE/INS 因子图研究
└─ mems-range-navigation/       MEMS、距离及方位角辅助惯导探索
```

每个专题内部保存自己的 `scripts`、`config`、`functions` 和入口配置，避免不同阶段的同名函数及参数相互污染。全部输入输出位于项目总目录：

```text
data/inertial-experiment/
├─ algorithm-exploration/
├─ paper-study/
├─ all-real-data-processing/
├─ cpp-matlab-comparison/
├─ factor-graph-navigation/
└─ mems-range-navigation/
```

每个专题的数据均按以下原则组织：

- `input`：输入数据；
- `navigation-results`：仅保存导航轨迹 `.nav`；
- `figures-tables`：图片、表格、诊断 CSV/MAT 等分析产物。

进入具体专题后，先运行该专题的 `setup_*.m`，再运行对应脚本。更详细的入口、数据映射和输出说明见各专题 README。
