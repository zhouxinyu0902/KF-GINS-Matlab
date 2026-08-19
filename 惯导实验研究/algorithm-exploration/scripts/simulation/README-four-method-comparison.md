# 多数据集四方法对比

运行 `run_four_method_dataset_comparison.m`，可批量处理不同仿真数据集，并统一对比：

- 前向 EKF；
- 二次 RTS；
- 2RTS + 旋转收缩；
- 2RTS + 位置速度约束。

## 数据集选择

脚本顶部的 `selected_cases` 用于选择数据集：

```matlab
% 自动扫描并处理全部 case-* 数据集
selected_cases = strings(0, 1);

% 只处理指定数据集
selected_cases = ["case-00", "case-02"];
```

`overwrite_existing = false` 时会复用齐全的已有结果，只补算缺失阶段；设为 `true` 时重新运行全部算法。

## 输入与输出

输入：

```text
data/inertial-experiment/algorithm-exploration/input/simulation/<case>
```

每个数据集独立输出到：

```text
data/inertial-experiment/algorithm-exploration/navigation-results/simulation/<case>/four-method-comparison
```

其中包含四种导航结果、旋转收缩诊断、位置速度伪量测诊断、误差统计，以及轨迹和水平径向误差对比图。所有数据集的统计汇总保存为：

```text
data/inertial-experiment/algorithm-exploration/navigation-results/simulation/four-method-comparison-summary.csv
```

