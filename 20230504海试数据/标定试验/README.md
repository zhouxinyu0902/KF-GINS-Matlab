# 2023-04-30 海试数据处理

现在有两个相互独立的可执行脚本。

## 含惯导的数据批次

```matlab
run('D:\Github\KF-GINS-Matlab\20230504海试数据\scripts\ins\process_ins_dvl_usbl.m')
```

该脚本解析 log1、log2 和最后三个不完整 USBL 文件，只导出解析结果，不进行
USBL 全局分析或画图。结果位于 `解析后的数据\ins_batch`。

## 全部 USBL 数据的整体分析

```matlab
run('D:\Github\KF-GINS-Matlab\20230504海试数据\scripts\analyze_usbl_overview.m')
```

该脚本独立解析全部五个 USBL 文件，生成批次汇总、ID 汇总、内容汇总以及
USBL 四联总览图。结果位于 `解析后的数据\usbl_analysis`。

所有表格均为 UTF-8、制表符分隔的 `.txt`，不在结果目录生成 CSV。
