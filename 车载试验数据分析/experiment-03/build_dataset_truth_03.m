function results = build_dataset_truth_03(dataset_ids)
%BUILD_DATASET_TRUTH_03 批量构造第三次车载试验的truth.nav。
%
% build_dataset_truth_03()      构造全部三批。
% build_dataset_truth_03([1 3]) 只构造第一、第三批。

    if nargin < 1 || isempty(dataset_ids)
        dataset_ids = 1:3;
    end

    result_template = struct( ...
        'dataset_name', '', ...
        'output_file', '', ...
        'start_time', NaN, ...
        'end_time', NaN, ...
        'duration_s', NaN, ...
        'sample_count', 0, ...
        'position_update_count', 0);
    results = repmat(result_template, numel(dataset_ids), 1);
    for index = 1:numel(dataset_ids)
        fprintf('\n============================================================\n');
        fprintf('构造experiment-03真值：数据集 %s\n', ...
            char(string(dataset_ids(index))));
        fprintf('============================================================\n');
        results(index) = build_experiment03_truth(dataset_ids(index));
    end
end
