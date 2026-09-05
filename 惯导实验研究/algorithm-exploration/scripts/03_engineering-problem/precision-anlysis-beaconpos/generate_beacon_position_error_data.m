clear;
close all;
clc;
%% 潜标位置误差专题：数据生成入口
% 本脚本只生成距离/深度输入和清单，不执行导航，也不评价结果。

%% 1. 用户配置
data_source = "experiment";         % "simulation" 或 "experiment"
simulation_case = 'case-00';        % 仅仿真使用
error_enu_m = [0,0,0; 10,10,0.4; 20,20,0.4; ...
    50,50,0.4; 100,100,0.4; 200,200,1.0];
scenario_ids = [];                  % 留空时根据 ENU 误差自动命名
range_interval_s = 420;
random_seed = 1;
simulation_range_noise_std_m = 6;
experiment_range_noise_std_m = 6;
simulation_depth_noise_std_m = 0.4;
experiment_depth_noise_std_m = 0.4;

%% 2. 选择当前数据源的噪声参数
data_source = lower(string(data_source));
if data_source == "simulation"
    range_noise_std_m = simulation_range_noise_std_m;
    depth_noise_std_m = simulation_depth_noise_std_m;
elseif data_source == "experiment"
    range_noise_std_m = experiment_range_noise_std_m;
    depth_noise_std_m = experiment_depth_noise_std_m;
else
    error('data_source 只能设置为 "simulation" 或 "experiment"。');
end

%% 3. 生成专题输入
[manifest, generation_context] = process_beaconpos( ...
    data_source, simulation_case, error_enu_m, scenario_ids, ...
    range_interval_s, random_seed, range_noise_std_m, depth_noise_std_m);

fprintf('\n数据生成完成。后续请运行 run_beacon_position_error_study.m。\n');
fprintf('输入清单：%s\n', generation_context.manifest_path);
disp(manifest(:, {'ScenarioId', 'EastError_m', 'NorthError_m', ...
    'UpError_m', 'RangeFilePath'}));
