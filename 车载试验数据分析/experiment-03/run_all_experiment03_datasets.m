function run_all_experiment03_datasets(dataset_ids, unit_types)
%RUN_ALL_EXPERIMENT03_DATASETS 第三次车载试验完整统一入口。
%
% 处理顺序：
%   1. 构造truth.nav；
%   2. 批跑Pure INS、EKF、一次RTS和二次RTS；
%   3. 生成三批导航汇总和纯惯导汇总。
%
% 原始数据整理与距离评估如需重建，请先运行：
%   raw_data_read(1:3)
%   build_dataset_range_03(1:3, false)

    if nargin < 1 || isempty(dataset_ids)
        dataset_ids = 1:3;
    end
    if nargin < 2 || isempty(unit_types)
        unit_types = ["rad"];
    end

    build_dataset_truth_03(dataset_ids);
    run_all_experiment03_navigation(dataset_ids, unit_types);
    plot_all_error_summary_03(dataset_ids, unit_types);
    plot_all_pureins_03(dataset_ids, unit_types);
end
