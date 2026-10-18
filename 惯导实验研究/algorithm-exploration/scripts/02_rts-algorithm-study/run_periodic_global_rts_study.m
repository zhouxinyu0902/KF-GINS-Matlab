clear;
close all;
clc;
%% 周期触发全局RTS：测距仍每7 min更新，全局反推每42 min执行一次

%% 1. 用户配置
data_source = "experiment";
case_name = 'case-09';
position_error_unit = "m";
range_interval_s = 420;
global_rts_interval_s = 2520;          % 42 min
duration_s = 5041;                     % 默认覆盖两个42 min触发点
rts_node_interval_s = 1;
range_noise_std_m = 6;
depth_noise_std_m = 0.4;
random_seed = 1;

data_source_override = strtrim(getenv('KF_GINS_DATA_SOURCE'));
if ~isempty(data_source_override)
    data_source = lower(string(data_source_override));
end
case_name_override = strtrim(getenv('KF_GINS_CASE_NAME'));
if ~isempty(case_name_override)
    case_name = char(case_name_override);
end
duration_override = str2double(getenv('KF_GINS_RTS_DURATION_S'));
if isfinite(duration_override) && duration_override > 0
    duration_s = duration_override;
end
interval_override = str2double(getenv('KF_GINS_GLOBAL_RTS_INTERVAL_S'));
if isfinite(interval_override) && interval_override > 0
    global_rts_interval_s = interval_override;
end
node_override = str2double(getenv('KF_GINS_GLOBAL_RTS_NODE_INTERVAL_S'));
if isfinite(node_override) && node_override > 0
    rts_node_interval_s = node_override;
end

event_stride = round(global_rts_interval_s / range_interval_s);
if event_stride < 1 || ...
        abs(event_stride * range_interval_s - global_rts_interval_s) > 1e-9
    error('全局RTS间隔必须是测距间隔的正整数倍。');
end

%% 2. 运行共享前向EKF和周期全局RTS
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
paths = setup_inertial_experiment();
case_id = str2double(case_name(end));
if data_source == "simulation"
    input_dir = paths.simulation_input(case_id);
else
    input_dir = paths.experiment_input(case_id);
end
cfg = load_algorithm_exploration_config( ...
    data_source, position_error_unit, input_dir);
result_dir = fullfile(cfg.outputfolder, sprintf( ...
    'periodic-global-rts-%gmin-%s', ...
    global_rts_interval_s / 60, position_error_unit));

runtime_options = struct( ...
    'data_source', char(data_source), ...
    'input_dir', input_dir, ...
    'position_error_unit', char(position_error_unit), ...
    'random_seed', random_seed, ...
    'range_interval_s', range_interval_s, ...
    'range_noise_std_m', range_noise_std_m, ...
    'depth_noise_std_m', depth_noise_std_m, ...
    'filter_range_std_m', range_noise_std_m, ...
    'filter_depth_std_m', depth_noise_std_m, ...
    'use_adaptive_forward_update', false, ...
    'end_time_s', cfg.starttime + duration_s, ...
    'rts_node_interval_s', rts_node_interval_s, ...
    'enable_standard_global_rts', true, ...
    'standard_global_rts_event_stride', event_stride, ...
    'standard_global_rts_strict_output', true, ...
    'enable_standard_fixed_lag_rts', false, ...
    'write_segmented_rts_outputs', false);

fprintf(['开始周期全局RTS：%s/%s，测距间隔=%.1f min，' ...
    '每%d次测距（%.1f min）执行一次全局反推。\n'], ...
    data_source, case_name, range_interval_s / 60, event_stride, ...
    global_rts_interval_s / 60);
run_timer = tic;
outputs = simulate_navigation_case( ...
    case_name, result_dir, false, false, runtime_options);
elapsed_s = toc(run_timer);

%% 3. 严格快照评价
global_nav = readmatrix(outputs.standard_global_rts_path, ...
    'FileType', 'text');
if isempty(global_nav)
    error(['运行时长内尚未达到一次全局RTS触发条件；' ...
        '请增加duration_s或减小global_rts_interval_s。']);
end
forward_nav = readmatrix(outputs.forward_path, 'FileType', 'text');
forward_nav = align_nav_to_time(forward_nav, global_nav(:, 2));
truth = readmatrix(cfg.truthpath, 'FileType', 'text');

method_names = ["周期全局RTS"; "前向EKF"];
method_nav = {global_nav; forward_nav};
statistics = zeros(2, 8);
horizontal_errors = cell(2, 1);
for method_index = 1:2
    nav = method_nav{method_index};
    horizontal_errors{method_index} = ...
        calculate_horizontal_error_m(nav, truth);
    position_step = calculate_horizontal_step_m(nav);
    velocity_step = vecnorm(diff(nav(:, 6:8), 1, 1), 2, 2);
    statistics(method_index, :) = [ ...
        sqrt(mean(horizontal_errors{method_index}.^2)), ...
        mean(horizontal_errors{method_index}), ...
        prctile(horizontal_errors{method_index}, 95), ...
        max(horizontal_errors{method_index}), ...
        median(position_step), prctile(position_step, 99), ...
        max(position_step), max(velocity_step)];
end
comparison = table(method_names, statistics(:, 1), statistics(:, 2), ...
    statistics(:, 3), statistics(:, 4), statistics(:, 5), ...
    statistics(:, 6), statistics(:, 7), statistics(:, 8), ...
    'VariableNames', {'method', 'horizontal_rmse_m', ...
    'horizontal_mean_m', 'horizontal_p95_m', 'horizontal_max_m', ...
    'position_step_median_m', 'position_step_p99_m', ...
    'position_step_max_m', 'velocity_step_max_mps'});
comparison_path = fullfile(result_dir, ...
    'periodic-global-rts-comparison.csv');
writetable(comparison, comparison_path);
disp(comparison);

%% 4. 误差和轨迹连续性曲线
plot_indices = select_plot_indices(size(global_nav, 1), 8000);
elapsed_min = (global_nav(plot_indices, 2) - global_nav(1, 2)) / 60;
comparison_figure = figure('Color', 'w', 'Visible', 'off', ...
    'Position', [100, 100, 1200, 760]);
tiledlayout(comparison_figure, 2, 1, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
nexttile;
plot(elapsed_min, horizontal_errors{1}(plot_indices), 'LineWidth', 1.1);
hold on;
plot(elapsed_min, horizontal_errors{2}(plot_indices), 'LineWidth', 1.0);
grid on;
ylabel('水平误差 / m');
title(sprintf('%s / %s：每%.1f min执行一次全局RTS', ...
    char(data_source), case_name, global_rts_interval_s / 60));
legend(cellstr(method_names), 'Location', 'best');

nexttile;
step_indices = plot_indices(plot_indices > 1);
for method_index = 1:2
    position_step = calculate_horizontal_step_m(method_nav{method_index});
    plot((global_nav(step_indices, 2) - global_nav(1, 2)) / 60, ...
        position_step(step_indices - 1), 'LineWidth', 1.0);
    hold on;
end
grid on;
xlabel('最近一次全局快照相对时间 / min');
ylabel('相邻位置步长 / m');
legend(cellstr(method_names), 'Location', 'best');
figure_path = fullfile(result_dir, ...
    'periodic-global-rts-horizontal-error-and-smoothness.png');
exportgraphics(comparison_figure, figure_path, 'Resolution', 200);
close(comparison_figure);

diagnostics = readtable(outputs.standard_global_rts_diagnostic_path);
fprintf('周期全局RTS完成，耗时%.2f s，共触发%d次。\n', ...
    elapsed_s, height(diagnostics));
fprintf('严格全局快照：%s\n', outputs.standard_global_rts_path);
fprintf('触发诊断：%s\n', outputs.standard_global_rts_diagnostic_path);
fprintf('比较统计：%s\n', comparison_path);
fprintf('比较曲线：%s\n', figure_path);

function aligned = align_nav_to_time(nav, target_time)
    [unique_time, unique_indices] = unique(nav(:, 2), 'stable');
    unique_nav = nav(unique_indices, :);
    aligned = nan(numel(target_time), size(nav, 2));
    aligned(:, 2) = target_time;
    aligned(:, 1) = interp1(unique_time, unique_nav(:, 1), ...
        target_time, 'nearest', nan);
    aligned(:, 3:end) = interp1(unique_time, unique_nav(:, 3:end), ...
        target_time, 'linear', nan);
    if any(~isfinite(aligned), 'all')
        error('前向EKF没有覆盖周期全局RTS快照区间。');
    end
end

function horizontal_error = calculate_horizontal_error_m(nav, truth)
    param = Param();
    truth_position = interp1(truth(:, 2), truth(:, 3:5), ...
        nav(:, 2), 'linear', nan);
    if any(~isfinite(truth_position), 'all')
        error('评价区间内存在无法插值的真值。');
    end
    latitude = mean(truth_position(:, 1)) * param.D2R;
    height = mean(truth_position(:, 3));
    [rm, rn] = getRmRn(latitude, param);
    north = (nav(:, 3) - truth_position(:, 1)) * ...
        param.D2R * (rm + height);
    east = (nav(:, 4) - truth_position(:, 2)) * ...
        param.D2R * (rn + height) * cos(latitude);
    horizontal_error = hypot(north, east);
end

function position_step = calculate_horizontal_step_m(nav)
    param = Param();
    latitude = mean(nav(:, 3)) * param.D2R;
    height = mean(nav(:, 5));
    [rm, rn] = getRmRn(latitude, param);
    north = diff(nav(:, 3)) * param.D2R * (rm + height);
    east = diff(nav(:, 4)) * param.D2R * ...
        (rn + height) * cos(latitude);
    position_step = hypot(north, east);
end

function indices = select_plot_indices(sample_count, maximum_count)
    if sample_count <= maximum_count
        indices = (1:sample_count)';
    else
        indices = unique(round(linspace(1, sample_count, ...
            maximum_count)))';
    end
end
