clear;
close all;
clc;

%% 实测数据四方法对比入口
% 对比：前向EKF、二次RTS、2RTS+旋转收缩、
% 2RTS+位置速度约束（事件驱动固定滞后重放）。
%
% 直接使用 experiment/preprocessed 中已经导出的距离和高度数据，
% 不重新加噪、不覆盖预处理数据。

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
    'algorithm-exploration', 'navigation-results', 'experiment', 'four-method-comparison');
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
        error('缺少实测四方法输入文件：%s', required_inputs{file_index});
    end
end

statistics_path = fullfile(artifact_dir, ...
    'fixed-lag-four-method-statistics.csv');
required_results = { ...
    fullfile(result_dir, 'range-ins-forward.nav'), ...
    fullfile(result_dir, 'range-ins-rts-double.nav'), ...
    fullfile(result_dir, 'range-ins-rts-double-bridge-rotation.nav'), ...
    fullfile(result_dir, ...
    'range-ins-double-rts-position-velocity-fixed-lag-replay.nav'), ...
    statistics_path};
results_ready = all(cellfun(@isfile, required_results));

if overwrite_existing || ~results_ready
    outputs = run_experiment_rts_core(result_dir);
    statistics = outputs.fixed_lag_statistics;
else
    fprintf('实测四方法结果齐全，直接读取已有统计。\n');
    statistics = readtable(statistics_path, 'TextType', 'string');
end

fprintf('\n实测四方法统计：\n');
disp(statistics);
fprintf('结果目录：%s\n', result_dir);
