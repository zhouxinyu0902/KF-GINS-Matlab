clear;
close all;
clc;

%% 距离辅助与“距离+方位角”辅助的前向 EKF / 二次 RTS 对比
% 本脚本只负责数据生成和导航计算；具体统计与绘图由
% evaluate_range_azimuth_aiding.m 独立完成。

%% 用户配置
case_name = 'case-00';
generate_measurements = true;
run_navigation = true;
run_evaluation = true;

azimuth_generation_options = struct( ...
    'sigma_min_deg', 0.10, ...
    'sigma_max_deg', 0.50, ...
    'shape_power', 1.5, ...
    'jitter_ratio', 0.20, ...
    'random_seed', 20260831, ...
    'baseline_normal_offset_deg', 90);
default_filter_azimuth_std_deg = 0.30;

%% 初始化专题路径
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
project_root = fileparts(fileparts(topic_dir));
addpath(script_dir);
addpath(topic_dir);
addpath(fullfile(topic_dir, 'scripts', 'rts-algorithm-study'));
% 原工程按功能分了多层函数目录，这里递归加入路径以支持独立运行。
addpath(genpath(fullfile(project_root, 'function')));
addpath(genpath(fullfile(project_root, 'function_zxy')));
addpath(genpath(fullfile(project_root, 'GINS-KF')));
paths = setup_inertial_experiment();

augmented_input_dir = fullfile(paths.simulation_input, case_name, ...
    'range-azimuth-aided');
augmented_range_paths = arrayfun( ...
    @(index) fullfile(augmented_input_dir, sprintf('range%d.txt', index)), ...
    1:3, 'UniformOutput', false);

%% 1. 根据真值和已有测距文件构造相对方位角
if generate_measurements
    generated = generate_range_azimuth_measurements( ...
        case_name, azimuth_generation_options);
    augmented_range_paths = generated.range_paths;
elseif ~all(cellfun(@isfile, augmented_range_paths))
    error('距离+方位角输入不存在，请先令 generate_measurements = true。');
end

%% 2. 使用同一前向/二次 RTS 框架运行两种量测模型
if run_navigation
    comparison_root = fullfile(paths.simulation_navigation, case_name, ...
        'range-azimuth-aided');
    range_only_dir = fullfile(comparison_root, 'range-only');
    range_azimuth_dir = fullfile(comparison_root, 'range-azimuth');

    % 公平对比：距离组使用标准距离更新，联合组只增加方位角这一维量测；
    % 两组均由核心脚本以同一种随机种子加入相同的距离和深度噪声。
    range_only_runtime = struct( ...
        'measurement_update_function', @myRangeUpdate);
    range_only_outputs = simulate_navigation_case( ...
        case_name, range_only_dir, false, false, range_only_runtime);

    joint_update = @(navstate, range_data, depth_data, kf) ...
        update_range_azimuth_filter_rad(navstate, range_data, ...
        depth_data, kf, default_filter_azimuth_std_deg);
    range_azimuth_runtime = struct( ...
        'range_source_paths', {augmented_range_paths}, ...
        'measurement_update_function', joint_update, ...
        'error_feedback_function', @feedback_range_azimuth_state);
    range_azimuth_outputs = simulate_navigation_case( ...
        case_name, range_azimuth_dir, false, false, range_azimuth_runtime);

    context_path = fullfile(comparison_root, ...
        'range-azimuth-comparison-context.mat');
    if ~isfolder(fileparts(context_path))
        mkdir(fileparts(context_path));
    end
    save(context_path, 'case_name', 'azimuth_generation_options', ...
        'default_filter_azimuth_std_deg', 'range_only_outputs', ...
        'range_azimuth_outputs', 'augmented_range_paths');
    fprintf('距离+方位角导航计算完成：%s\n', fileparts(context_path));
end

%% 3. 独立统计和绘图
if run_evaluation
    evaluate_range_azimuth_aiding(case_name);
end
