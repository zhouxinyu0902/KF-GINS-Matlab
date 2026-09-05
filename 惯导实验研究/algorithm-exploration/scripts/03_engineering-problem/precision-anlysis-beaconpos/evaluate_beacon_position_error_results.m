clear;
close all;
clc;
%% 潜标位置误差专题：结果评价入口
% 本脚本只读取已有导航结果，输出统计表和图片，不重新生成数据或导航。

%% 1. 用户配置
data_source = "experiment";         % "simulation" 或 "experiment"
simulation_case = 'case-00';        % 仅仿真使用
position_error_units = ["rad"];    % 可设为 ["rad", "m"]

%% 2. 定位结果并逐单位评价
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));
addpath(topic_dir);
paths = setup_inertial_experiment();

data_source = lower(string(data_source));
position_error_units = lower(string(position_error_units));
if ~ismember(data_source, ["simulation", "experiment"])
    error('data_source 只能设置为 "simulation" 或 "experiment"。');
end
if any(~ismember(position_error_units, ["rad", "m"]))
    error('position_error_units 只能包含 "rad" 或 "m"。');
end

for position_error_unit = position_error_units
    directories = engineering_problem_directories(paths, data_source, ...
        simulation_case, 'beacon-position-error', position_error_unit);
    context_path = fullfile(directories.result_root, 'study-context.mat');
    if ~isfile(context_path)
        error(['找不到 %s 结果上下文：%s\n请先运行 ', ...
            'run_beacon_position_error_study.m。'], ...
            position_error_unit, context_path);
    end
    fprintf('\n开始评价 %s 位置误差状态结果。\n', position_error_unit);
    evaluate_engineering_sensitivity_context(context_path);
end
