clear;
close all;
clc;

%% 多数据集事件驱动固定滞后四方法对比入口
% 与 run_four_method_dataset_comparison.m 的区别：
% 本脚本不是等全部二次 RTS 完成后再全局重放，而是在每个测距事件处，
% 只要上一段二次 RTS 结果生成，就立即重放该段历史 IMU，并继承上一段
% 重放结束时的导航状态和协方差。

%% 用户配置
% 空数组表示自动扫描全部 case-*；也可指定 ["case-00", "case-02"]。
selected_cases = strings(0, 1);
overwrite_existing = false;

%% 初始化目录
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
project_root = fileparts(fileparts(topic_dir));
addpath(script_dir);
addpath(topic_dir);
setup_inertial_experiment();

input_root = fullfile(project_root, 'data', 'inertial-experiment', ...
    'algorithm-exploration', 'input', 'simulation');
output_root = fullfile(project_root, 'data', 'inertial-experiment', ...
    'algorithm-exploration', 'navigation-results', 'simulation');

if isempty(selected_cases)
    entries = dir(fullfile(input_root, 'case-*'));
    entries = entries([entries.isdir]);
    selected_cases = sort(string({entries.name}));
else
    selected_cases = unique(string(selected_cases(:)), 'stable');
end
if isempty(selected_cases)
    error('输入目录中没有找到 case-* 数据集：%s', input_root);
end

fprintf('准备处理 %d 个固定滞后数据集：%s\n', ...
    numel(selected_cases), strjoin(selected_cases, ', '));

%% 逐数据集执行事件驱动四方法计算
all_statistics = table();
failed_cases = strings(0, 1);
for case_index = 1:numel(selected_cases)
    case_name = selected_cases(case_index);
    input_dir = fullfile(input_root, case_name);
    result_dir = fullfile(output_root, case_name, ...
        'fixed-lag-four-method-comparison');
    statistics_path = fullfile(exploration_artifact_dir(result_dir), ...
        'fixed-lag-four-method-statistics.csv');
    fixed_lag_path = fullfile(result_dir, ...
        'range-ins-double-rts-position-velocity-fixed-lag-replay.nav');
    rotation_path = fullfile(result_dir, ...
        'range-ins-rts-double-bridge-rotation.nav');

    fprintf('\n[%d/%d] 开始处理 %s\n', ...
        case_index, numel(selected_cases), case_name);
    try
        validate_fixed_lag_dataset(input_dir, case_name);
        results_ready = isfile(statistics_path) ...
            && isfile(fixed_lag_path) && isfile(rotation_path);
        if overwrite_existing || ~results_ready
            outputs = simulate_range_ins_rts_comparison( ...
                case_name, result_dir, false, true);
            case_statistics = outputs.fixed_lag_statistics;
        else
            fprintf('固定滞后四方法结果齐全，直接读取统计表。\n');
            case_statistics = readtable(statistics_path, ...
                'TextType', 'string');
        end

        case_statistics.Method = string(case_statistics.Method);
        dataset = repmat(case_name, height(case_statistics), 1);
        case_statistics = addvars(case_statistics, dataset, ...
            'Before', 1, 'NewVariableNames', 'Dataset');
        all_statistics = [all_statistics; case_statistics]; %#ok<AGROW>
        fprintf('%s 固定滞后处理完成：%s\n', case_name, result_dir);
        close all;
    catch exception
        failed_cases(end + 1, 1) = case_name; %#ok<SAGROW>
        warning('%s 处理失败：%s', case_name, ...
            getReport(exception, 'basic', 'hyperlinks', 'off'));
    end
end

%% 保存跨数据集汇总
if ~isempty(all_statistics)
    summary_path = fullfile(exploration_artifact_dir(output_root), ...
        'fixed-lag-four-method-comparison-summary.csv');
    writetable(all_statistics, summary_path);
    fprintf('\n跨数据集固定滞后统计：%s\n', summary_path);
end

if isempty(failed_cases)
    fprintf('全部 %d 个数据集处理完成。\n', numel(selected_cases));
else
    fprintf('成功 %d 个，失败 %d 个：%s\n', ...
        numel(selected_cases) - numel(failed_cases), ...
        numel(failed_cases), strjoin(failed_cases, ', '));
end

%% 局部函数
function validate_fixed_lag_dataset(input_dir, case_name)
%VALIDATE_FIXED_LAG_DATASET 检查固定滞后算法所需输入。
    required_names = { ...
        'IMU_120.txt', 'truth.txt', ...
        'range1.txt', 'range2.txt', 'range3.txt'};
    if ~isfolder(input_dir)
        error('数据集目录不存在：%s', input_dir);
    end
    for file_index = 1:numel(required_names)
        file_path = fullfile(input_dir, required_names{file_index});
        if ~isfile(file_path)
            error('数据集 %s 缺少输入文件：%s', case_name, file_path);
        end
    end
end
