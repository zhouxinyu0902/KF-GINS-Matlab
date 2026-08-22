clear;
close all;
clc;

%% 实测四方法结果统一评价入口
% 本脚本不运行核心算法，只读取已有 m/rad 导航结果并生成统计和图片。
% 如只想评价其中一条状态链，可修改 state_modes。

state_modes = ["m", "rad"];

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
paths = setup_inertial_experiment();
truth_path = fullfile(paths.experiment_reference, 'truth.nav');

evaluations = struct();
for state_mode = state_modes
    if state_mode == "m"
        case_name = 'experiment-m-state';
        result_dir = fullfile(paths.experiment_navigation, ...
            'four-method-comparison');
    elseif state_mode == "rad"
        case_name = 'experiment-rad-state';
        result_dir = fullfile(paths.experiment_navigation, ...
            'four-method-comparison-rad');
    else
        error('不支持的状态模式：%s', state_mode);
    end

    if ~isfolder(result_dir)
        warning('结果目录不存在，跳过 %s：%s', state_mode, result_dir);
        continue;
    end
    evaluations.(char(state_mode)) = evaluate_experiment_four_methods( ...
        result_dir, truth_path=truth_path, case_name=case_name, ...
        create_figure=true);
    fprintf('\n%s 链四方法统计：\n', state_mode);
    disp(evaluations.(char(state_mode)).statistics);
end

if isfield(evaluations, 'm') && isfield(evaluations, 'rad')
    output_dir = fullfile(paths.experiment_artifacts, ...
        'four-method-comparison-rad');
    unit_comparison = compare_experiment_state_units( ...
        evaluations.m.statistics_path, evaluations.rad.statistics_path, ...
        output_dir, create_figure=true);
    fprintf('\nm 链与 rad 链 RMSE 对比：\n');
    disp(unit_comparison);
end
