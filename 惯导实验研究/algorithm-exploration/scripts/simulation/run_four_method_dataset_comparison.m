clear;
close all;
clc;

%% 多数据集四方法对比入口
% 对每个数据集依次计算并对比：
%   1）前向 EKF；
%   2）二次 RTS；
%   3）2RTS + 旋转收缩；
%   4）2RTS + 水平位置速度约束的历史 IMU 重放。
%
% 输出目录按输入数据集自动隔离：
%   data/inertial-experiment/algorithm-exploration/navigation-results/simulation/<case>/four-method-comparison

%% 用户配置
% 指定数据集，例如 ["case-00", "case-02"]。
% 设置为 strings(0, 1) 时，自动处理 input 目录下全部 case-* 数据集。
selected_cases = strings(0, 1);
% selected_cases = ["case-00", "case-01"];
% true：重新运行算法并覆盖本专题已有结果；
% false：结果齐全时直接读取已有统计，缺失时才重新计算。
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
    case_entries = dir(fullfile(input_root, 'case-*'));
    case_entries = case_entries([case_entries.isdir]);
    selected_cases = sort(string({case_entries.name}));
else
    selected_cases = unique(string(selected_cases(:)), 'stable');
end

if isempty(selected_cases)
    error('输入目录中没有找到可处理的数据集：%s', input_root);
end

fprintf('准备处理 %d 个数据集：%s\n', numel(selected_cases), ...
    strjoin(selected_cases, ', '));

%% 逐数据集运行算法
all_statistics = table();
failed_cases = strings(0, 1);

for case_index = 1:numel(selected_cases)
    case_name = selected_cases(case_index);
    input_dir = fullfile(input_root, case_name);
    result_dir = fullfile(output_root, case_name, ...
        'four-method-comparison');

    fprintf('\n[%d/%d] 开始处理 %s\n', case_index, ...
        numel(selected_cases), case_name);

    try
        validate_dataset(input_dir, case_name);
        if ~exist(result_dir, 'dir')
            mkdir(result_dir);
        end

        forward_path = fullfile(result_dir, 'range-ins-forward.nav');
        double_rts_path = fullfile(result_dir, 'range-ins-rts-double.nav');
        rotation_contraction_path = fullfile(result_dir, ...
            'range-ins-rts-double-bridge-rotation.nav');
        replay_path = fullfile(result_dir, ...
            'range-ins-double-rts-position-velocity-guided-replay.nav');
        statistics_path = fullfile(exploration_artifact_dir(result_dir), ...
            'range-ins-double-rts-position-velocity-guided-replay-statistics.csv');

        first_stage_ready = isfile(forward_path) && isfile(double_rts_path) ...
            && isfile(rotation_contraction_path);
        replay_stage_ready = isfile(replay_path) && isfile(statistics_path);

        if overwrite_existing || ~first_stage_ready
            % false 表示不计算本次四方法对比不需要的 BRC 辅助结果。
            simulate_range_ins_rts_comparison(case_name, result_dir, false);
            first_stage_was_run = true;
        else
            fprintf('已有前向 EKF 和二次 RTS 结果，跳过第一阶段。\n');
            first_stage_was_run = false;
        end

        if overwrite_existing || first_stage_was_run || ~replay_stage_ready
            outputs = replay_ins_with_double_rts_position_velocity( ...
                case_name, result_dir);
            case_statistics = outputs.statistics;
        else
            fprintf('已有位置速度约束结果，直接读取统计表。\n');
            case_statistics = readtable(statistics_path, ...
                'TextType', 'string');
        end

        case_statistics.Method = string(case_statistics.Method);
        dataset = repmat(case_name, height(case_statistics), 1);
        case_statistics = addvars(case_statistics, dataset, ...
            'Before', 1, 'NewVariableNames', 'Dataset');
        all_statistics = [all_statistics; case_statistics]; %#ok<AGROW>

        fprintf('%s 处理完成，输出目录：%s\n', case_name, result_dir);
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
        'four-method-comparison-summary.csv');
    writetable(all_statistics, summary_path);
    fprintf('\n跨数据集统计汇总：%s\n', summary_path);
else
    summary_path = '';
end

if isempty(failed_cases)
    fprintf('全部 %d 个数据集处理完成。\n', numel(selected_cases));
else
    fprintf('成功 %d 个，失败 %d 个。失败数据集：%s\n', ...
        numel(selected_cases) - numel(failed_cases), ...
        numel(failed_cases), strjoin(failed_cases, ', '));
end

%% 局部函数
function validate_dataset(input_dir, case_name)
%VALIDATE_DATASET 检查四方法计算所需的仿真输入文件。
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

