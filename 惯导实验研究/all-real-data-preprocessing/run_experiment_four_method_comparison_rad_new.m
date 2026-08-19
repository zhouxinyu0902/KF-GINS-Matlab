clear;
close all;
clc;

%% 实测数据四方法对比入口（rad位置误差状态）
% x(1:3)=[dLat(rad),dLon(rad),dH(m)]。
% 使用与米制版本相同的数据、时段、测距噪声和RTS节点间隔。

overwrite_existing = false;

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir, '-begin');
paths = setup_all_real_data_preprocessing();
project_root = paths.project;
input_dir = paths.input;
result_dir = paths.output;
artifact_dir = paths.artifacts;

required_inputs = { ...
    fullfile(input_dir, 'range.txt'), ...
    fullfile(input_dir, 'depth_raw.txt'), ...
    fullfile(input_dir, 'imu_120.txt'), ...
    fullfile(input_dir, 'pva_830.txt')};
for file_index = 1:numel(required_inputs)
    if ~isfile(required_inputs{file_index})
        error('缺少rad链实测输入文件：%s', required_inputs{file_index});
    end
end

statistics_path = fullfile(artifact_dir, ...
    'fixed-lag-four-method-statistics.csv');
required_results = { ...
    fullfile(result_dir, 'range-ins-forward.nav'), ...
    fullfile(result_dir, 'range-ins-rts-double.nav'), ...
    fullfile(result_dir, 'range-ins-rts-double-bridge-rotation.nav'), ...
    ... fullfile(result_dir,'range-ins-double-rts-position-velocity-fixed-lag-replay.nav'), ...%
    statistics_path};
results_ready = all(cellfun(@isfile, required_results));
result_is_current = results_ready;
if results_ready
    dependency_files = [required_inputs, { ...
        fullfile(script_dir, 'run_experiment_four_method_comparison_rad_new.m'), ...
        fullfile(script_dir, 'Config.m'), ...
        fullfile(script_dir, 'setup_all_real_data_preprocessing.m'), ...
        fullfile(script_dir, 'run_experiment_rts_core_rad_new.m')}];
    dependency_time = zeros(numel(dependency_files), 1);
    for file_index = 1:numel(dependency_files)
        file_information = dir(dependency_files{file_index});
        dependency_time(file_index) = file_information.datenum;
    end
    result_time = zeros(numel(required_results), 1);
    for file_index = 1:numel(required_results)
        file_information = dir(required_results{file_index});
        result_time(file_index) = file_information.datenum;
    end
    result_is_current = min(result_time) >= max(dependency_time);
end

if overwrite_existing || ~result_is_current
    if results_ready && ~result_is_current
        fprintf('检测到输入或程序比现有结果更新，重新计算四方法结果。\n');
    end
    outputs = run_experiment_rts_core_rad_new(result_dir);
    rad_statistics = outputs.fixed_lag_statistics;
else
    fprintf('rad链四方法结果齐全，直接读取已有统计。\n');
    rad_statistics = readtable(statistics_path, 'TextType', 'string');
end

fprintf('\nrad链实测四方法统计：\n');
disp(rad_statistics);

% 若米制结果存在，则给出相同方法的RMSE差值，正值表示rad链误差更大。
meter_statistics_path = fullfile(project_root, 'data', ...
    'inertial-experiment', 'algorithm-exploration', 'figures-tables', 'experiment', ...
    'four-method-comparison', 'fixed-lag-four-method-statistics.csv');
if isfile(meter_statistics_path)
    meter_statistics = readtable(meter_statistics_path, 'TextType', 'string');
    unit_comparison = table(rad_statistics.Method, ...
        meter_statistics.RMSE_m, rad_statistics.RMSE_m, ...
        rad_statistics.RMSE_m - meter_statistics.RMSE_m, ...
        'VariableNames', {'Method', 'MeterState_RMSE_m', ...
        'RadState_RMSE_m', 'RadMinusMeter_m'});
    writetable(unit_comparison, fullfile(artifact_dir, ...
        'rad-vs-meter-rmse-comparison.csv'));
    figure_dir = artifact_dir;
    if ~exist(figure_dir, 'dir')
        mkdir(figure_dir);
    end
    comparison_figure = figure('Color', 'w', ...
        'Name', 'rad与m位置误差状态RMSE对比', ...
        'Position', [120, 120, 1100, 620]);
    method_axis = categorical(unit_comparison.Method, ...
        unit_comparison.Method, 'Ordinal', true);
    bar(method_axis, [unit_comparison.MeterState_RMSE_m, ...
        unit_comparison.RadState_RMSE_m], 'grouped');
    grid on;
    ylabel('水平径向误差RMSE（m）');
    title('实测数据：m链与rad链四方法RMSE对比');
    legend('m链', 'rad链', 'Location', 'northwest');
    xtickangle(15);
    exportgraphics(comparison_figure, fullfile(figure_dir, ...
        'rad-vs-meter-rmse-comparison.png'), 'Resolution', 300);
    savefig(comparison_figure, fullfile(figure_dir, ...
        'rad-vs-meter-rmse-comparison.fig'));
    fprintf('\nrad链与m链RMSE对比：\n');
    disp(unit_comparison);
end

fprintf('rad链结果目录：%s\n', result_dir);
