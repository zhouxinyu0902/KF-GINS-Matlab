clear;
close all;
clc;

%% 声学测距掉帧评价：仿真/实测统一入口
data_source = "experiment";         % 必须与运行入口一致
simulation_case = 'case-00';
position_error_unit = "rad";

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(fileparts(script_dir))));
addpath(topic_dir);
paths = setup_inertial_experiment();
directories = engineering_problem_directories(paths, data_source, ...
    simulation_case, 'acoustic-range-dropout', position_error_unit);
context_path = fullfile(directories.result_root, 'study-context.mat');
evaluate_engineering_sensitivity_context(context_path);
