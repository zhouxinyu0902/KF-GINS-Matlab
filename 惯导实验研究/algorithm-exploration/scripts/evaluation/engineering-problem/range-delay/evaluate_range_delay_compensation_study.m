clear;
close all;
clc;

%% 测距延迟补偿评价：仿真/实测统一入口
data_source = "experiment";         % "simulation" 或 "experiment"
simulation_case = 'case-00';
position_error_unit = "rad";

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(fileparts(script_dir))));
addpath(topic_dir);
paths = setup_inertial_experiment();
directories = engineering_problem_directories(paths, data_source, ...
    simulation_case, 'range-delay-compensation', position_error_unit);
evaluate_range_delay_context(fullfile( ...
    directories.result_root, 'study-context.mat'));
