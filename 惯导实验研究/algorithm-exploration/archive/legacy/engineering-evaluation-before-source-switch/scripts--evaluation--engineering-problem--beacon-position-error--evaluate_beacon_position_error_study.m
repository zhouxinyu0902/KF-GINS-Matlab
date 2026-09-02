clear;
close all;
clc;

%% 潜标位置误差专题评价
simulation_case = 'case-00';
position_error_unit = "rad";       % 必须与运行脚本一致

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(fileparts(script_dir))));
common_dir = fullfile(fileparts(script_dir), 'common');
addpath(topic_dir);
addpath(common_dir);
paths = setup_inertial_experiment();

context_path = fullfile(paths.simulation_navigation, simulation_case, ...
    'engineering-applications', 'beacon-position-error', ...
    lower(position_error_unit), 'study-context.mat');
evaluate_engineering_sensitivity_context(context_path);
