clear;
clc;

%% 实测数据四方法对比入口（rad位置误差状态）
% x(1:3)=[dLat(rad),dLon(rad),dH(m)]。
% 使用与米制版本相同的数据、时段、测距噪声和RTS节点间隔。

overwrite_existing = false;

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
project_root = fileparts(fileparts(topic_dir));
addpath(script_dir);
addpath(topic_dir);
setup_inertial_experiment();

input_dir = fullfile(project_root, 'data', 'inertial-experiment', ...
    'algorithm-exploration', 'input', 'experiment-preprocessed');
result_dir = fullfile(project_root, 'data', 'inertial-experiment', ...
    'algorithm-exploration', 'navigation-results', 'experiment', 'four-method-comparison-rad');
artifact_dir = exploration_artifact_dir(result_dir);

required_inputs = { ...
    fullfile(input_dir, 'rangedata_noised.txt'), ...
    fullfile(input_dir, 'height_noised.txt'), ...
    fullfile(project_root, 'data', 'inertial-experiment', ...
    'algorithm-exploration', 'input', 'experiment-raw', 'IMU_120.txt'), ...
    fullfile(project_root, 'data', 'inertial-experiment', ...
    'algorithm-exploration', 'input', 'experiment-reference', 'truth.nav')};
for file_index = 1:numel(required_inputs)
    if ~isfile(required_inputs{file_index})
        error('缺少rad链实测输入文件：%s', required_inputs{file_index});
    end
end

required_results = { ...
    fullfile(result_dir, 'range-ins-forward.nav'), ...
    fullfile(result_dir, 'range-ins-rts-double.nav'), ...
    fullfile(result_dir, 'range-ins-rts-double-bridge-rotation.nav'), ...
    fullfile(result_dir, ...
    'range-ins-double-rts-position-velocity-fixed-lag-replay.nav')};
results_ready = all(cellfun(@isfile, required_results));

if overwrite_existing || ~results_ready
    run_experiment_rts_core_rad(result_dir);
else
    fprintf('rad链四方法导航结果齐全，跳过核心算法。\n');
end

% 运行入口只生成导航结果和统计表，不创建图窗。
evaluation = evaluate_experiment_four_methods(result_dir, ...
    truth_path=required_inputs{4}, case_name='experiment-rad-state', ...
    create_figure=false);
rad_statistics = evaluation.statistics;

fprintf('\nrad链实测四方法统计：\n');
disp(rad_statistics);

% 若米制结果存在，则给出相同方法的RMSE差值，正值表示rad链误差更大。
meter_statistics_path = fullfile(project_root, 'data', ...
    'inertial-experiment', 'algorithm-exploration', 'figures-tables', 'experiment', ...
    'four-method-comparison', 'fixed-lag-four-method-statistics.csv');
if isfile(meter_statistics_path)
    unit_comparison = compare_experiment_state_units( ...
        meter_statistics_path, evaluation.statistics_path, ...
        artifact_dir, create_figure=false);
    fprintf('\nrad链与m链RMSE对比：\n');
    disp(unit_comparison);
end

fprintf('rad链结果目录：%s\n', result_dir);
fprintf('如需绘图，请运行 scripts/evaluation/evaluate_experiment_results.m。\n');
