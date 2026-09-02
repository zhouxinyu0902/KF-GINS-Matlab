clear;
clc;

%% 测距误差对EKF/RTS导航的影响
% 固定滤波器参数、IMU、深度噪声和随机序列，只改变输入距离噪声标准差。
% 本脚本只运行导航；统计和绘图位于 evaluation/engineering-applications。

%% 1. 用户配置
simulation_case = 'case-00';
position_error_unit = "rad";       % "rad" 或 "m"
range_noise_levels_m = [0; 5; 10; 20; 50];
filter_range_std_m = 10;           % 固定，避免将量测误差与滤波调参混为一谈
random_seed = 1;                   % 各工况使用同一标准正态噪声序列
end_time_s = 4621;

%% 2. 路径和参数检查
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
paths = setup_inertial_experiment();

position_error_unit = lower(string(position_error_unit));
range_noise_levels_m = range_noise_levels_m(:);
if ~ismember(position_error_unit, ["rad", "m"])
    error('position_error_unit 只能设置为 "rad" 或 "m"。');
end
if isempty(range_noise_levels_m) || any(~isfinite(range_noise_levels_m)) || ...
        any(range_noise_levels_m < 0) || ...
        numel(unique(range_noise_levels_m)) ~= numel(range_noise_levels_m)
    error('range_noise_levels_m 必须是非负、有限且不重复的列向量。');
end

scenario_count = numel(range_noise_levels_m);
scenario_ids = compose('sigma-%gm', range_noise_levels_m);
scenario_ids = replace(scenario_ids, '.', 'p');
scenario_names = compose('测距误差标准差 %.1f m', range_noise_levels_m);
result_root = fullfile(paths.simulation_navigation, simulation_case, ...
    'engineering-applications', 'range-measurement-error', ...
    position_error_unit);
if ~isfolder(result_root), mkdir(result_root); end

%% 3. 逐误差等级运行同一套导航算法
output_dirs = strings(scenario_count, 1);
valid_masks = cell(scenario_count, 3);
for scenario_index = 1:scenario_count
    output_dirs(scenario_index) = fullfile(result_root, ...
        scenario_ids(scenario_index));
    runtime_options = struct( ...
        'position_error_unit', position_error_unit, ...
        'range_noise_std_m', range_noise_levels_m(scenario_index), ...
        'filter_range_std_m', filter_range_std_m, ...
        'random_seed', random_seed, ...
        'end_time_s', end_time_s);
    fprintf('\n========== 测距误差工况 %d/%d：sigma=%.1f m ==========\n', ...
        scenario_index, scenario_count, range_noise_levels_m(scenario_index));
    outputs = simulate_navigation_case(simulation_case, ...
        output_dirs(scenario_index), false, false, runtime_options);
    valid_masks{scenario_index, 1} = outputs.forward_mask;
    valid_masks{scenario_index, 2} = outputs.single_rts_mask;
    valid_masks{scenario_index, 3} = outputs.double_rts_mask;
end

%% 4. 保存轻量评价上下文
cfg = ProcessConfigforSimu(fullfile(paths.simulation_input, simulation_case));
study_context = struct();
study_context.version = 1;
study_context.study_id = "range-measurement-error";
study_context.study_title = "测距误差敏感性";
study_context.simulation_case = simulation_case;
study_context.position_error_unit = char(position_error_unit);
study_context.scenario_ids = scenario_ids;
study_context.scenario_names = scenario_names;
study_context.parameter_values = range_noise_levels_m;
study_context.parameter_name = "RangeNoiseStd_m";
study_context.parameter_label = "测距误差标准差（m）";
study_context.output_dirs = output_dirs;
study_context.truth_path = cfg.truthpath;
study_context.method_ids = ["ekf"; "single-rts"; "double-rts"];
study_context.method_names = ["前向EKF"; "一次RTS"; "二次RTS"];
study_context.method_files = ["range-ins-forward.nav"; ...
    "range-ins-rts-single.nav"; "range-ins-rts-double.nav"];
study_context.valid_masks = valid_masks;
study_context.artifact_root = fullfile(paths.simulation_artifacts, ...
    simulation_case, 'engineering-applications', ...
    'range-measurement-error', position_error_unit);
save(fullfile(result_root, 'study-context.mat'), 'study_context');

fprintf('\n测距误差工况计算完成：%s\n', result_root);
fprintf(['结果评价入口：scripts/evaluation/engineering-applications/', ...
    'range-measurement-error/evaluate_range_measurement_error_study.m\n']);
