clear;
clc;

%% 潜标位置误差对EKF/RTS导航的影响
% 本脚本只构造工况并运行导航，不进行统计和绘图。
% 评价请运行 scripts/evaluation/engineering-applications/
% beacon-position-error/evaluate_beacon_position_error_study.m。

%% 1. 用户配置
simulation_case = 'case-00';
position_error_unit = "rad";       % "rad" 或 "m"
error_enu_m = [ ...                 % 每行：[东向, 北向, 天向]，单位m
    0,   0,   0;
    10,  10,  0.4;
    20,  20,  0.4;
    50,  50,  0.4;
    100, 100, 0.4;
    200, 200, 1.0];
range_noise_std_m = 10;            % 所有工况使用相同距离噪声
filter_range_std_m = 10;           % 所有工况使用相同滤波器参数
random_seed = 1;                   % 保证各工况的随机噪声完全一致
end_time_s = 4621;

%% 2. 路径与潜标误差输入
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
paths = setup_inertial_experiment();

position_error_unit = lower(string(position_error_unit));
if ~ismember(position_error_unit, ["rad", "m"])
    error('position_error_unit 只能设置为 "rad" 或 "m"。');
end
manifest = process_beaconpos(simulation_case, error_enu_m);
scenario_count = height(manifest);

result_root = fullfile(paths.simulation_navigation, simulation_case, ...
    'engineering-applications', 'beacon-position-error', ...
    position_error_unit);
if ~isfolder(result_root), mkdir(result_root); end

%% 3. 逐工况运行同一套导航算法
output_dirs = strings(scenario_count, 1);
valid_masks = cell(scenario_count, 3);
for scenario_index = 1:scenario_count
    scenario_id = manifest.ScenarioId(scenario_index);
    output_dirs(scenario_index) = fullfile(result_root, scenario_id);
    runtime_options = struct( ...
        'position_error_unit', position_error_unit, ...
        'range_source_paths', {{char(manifest.Range1Path(scenario_index)), ...
            char(manifest.Range2Path(scenario_index)), ...
            char(manifest.Range3Path(scenario_index))}}, ...
        'range_noise_std_m', range_noise_std_m, ...
        'filter_range_std_m', filter_range_std_m, ...
        'random_seed', random_seed, ...
        'end_time_s', end_time_s);
    fprintf('\n========== 潜标位置误差工况 %d/%d：%s ==========\n', ...
        scenario_index, scenario_count, scenario_id);
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
study_context.study_id = "beacon-position-error";
study_context.study_title = "潜标位置误差敏感性";
study_context.simulation_case = simulation_case;
study_context.position_error_unit = char(position_error_unit);
study_context.scenario_ids = manifest.ScenarioId;
study_context.scenario_names = compose( ...
    '水平误差 %.1f m', manifest.HorizontalError_m);
study_context.parameter_values = manifest.HorizontalError_m;
study_context.parameter_name = "HorizontalBeaconPositionError_m";
study_context.parameter_label = "潜标水平位置误差（m）";
study_context.output_dirs = output_dirs;
study_context.truth_path = cfg.truthpath;
study_context.method_ids = ["ekf"; "single-rts"; "double-rts"];
study_context.method_names = ["前向EKF"; "一次RTS"; "二次RTS"];
study_context.method_files = ["range-ins-forward.nav"; ...
    "range-ins-rts-single.nav"; "range-ins-rts-double.nav"];
study_context.valid_masks = valid_masks;
study_context.artifact_root = fullfile(paths.simulation_artifacts, ...
    simulation_case, 'engineering-applications', ...
    'beacon-position-error', position_error_unit);
save(fullfile(result_root, 'study-context.mat'), 'study_context');

fprintf('\n潜标位置误差工况计算完成：%s\n', result_root);
fprintf(['结果评价入口：scripts/evaluation/engineering-applications/', ...
    'beacon-position-error/evaluate_beacon_position_error_study.m\n']);
