clear;
close all;
clc;
%% 标准单次固定滞后RTS
% 每次测距到达后，对最近lag_segments个测距区间组成的连续窗口执行一次
% 标准RTS反向递推；只发布窗口中最老的成熟区间，不覆盖历史结果，也不
% 将平滑轨迹再次作为前向EKF或下一次RTS的名义轨迹。

%% 1. 用户配置
data_source = "experiment";
case_name = 'case-09';
position_error_unit = "m";
range_interval_s = 420;
duration_s = 5000;
lag_segments = 2;
rts_node_interval_s = 1;
range_noise_std_m = 6;
depth_noise_std_m = 0.4;
random_seed = 1;
compare_existing_results = true;

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
lag_override = str2double(getenv('KF_GINS_LAG_SEGMENTS'));
if isfinite(lag_override) && lag_override >= 1 && ...
        lag_override == round(lag_override)
    lag_segments = round(lag_override);
end
node_override = str2double(getenv('KF_GINS_GLOBAL_RTS_NODE_INTERVAL_S'));
if isfinite(node_override) && node_override > 0
    rts_node_interval_s = node_override;
end
compare_override = str2double(getenv('KF_GINS_COMPARE_FIXED_LAG'));
if isfinite(compare_override)
    compare_existing_results = compare_override ~= 0;
end

%% 2. 路径和共享前向导航配置
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
    'standard-fixed-lag-rts-%s', position_error_unit));

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
    'enable_standard_global_rts', false, ...
    'enable_standard_fixed_lag_rts', true, ...
    'standard_fixed_lag_segments', lag_segments, ...
    'write_segmented_rts_outputs', false);

fprintf(['开始标准固定滞后RTS：%s/%s，lag_segments=%d，' ...
    '窗口长度约%.1f min，RTS节点间隔=%.3f s。\n'], ...
    data_source, case_name, lag_segments, ...
    lag_segments * range_interval_s / 60, rts_node_interval_s);
run_timer = tic;
outputs = simulate_navigation_case( ...
    case_name, result_dir, false, false, runtime_options);
elapsed_s = toc(run_timer);

%% 3. 严格共同区间评价
new_nav = readmatrix(outputs.standard_fixed_lag_rts_path, ...
    'FileType', 'text');
if isempty(new_nav)
    error('没有成熟的固定滞后区间；请增加运行时长或减小lag_segments。');
end
truth = readmatrix(cfg.truthpath, 'FileType', 'text');
method_names = "标准固定滞后RTS";
raw_method_nav = {new_nav};

forward_nav = readmatrix(outputs.forward_path, 'FileType', 'text');
method_names(end + 1, 1) = "前向EKF";
raw_method_nav{end + 1, 1} = forward_nav;

if compare_existing_results
    segmented_path = fullfile(cfg.outputfolder, ...
        sprintf('fixed-lag-rts-%s', position_error_unit), ...
        sprintf('simple-rts-lag%d-%s.nav', ...
        lag_segments, position_error_unit));
    if isfile(segmented_path)
        segmented_nav = readmatrix(segmented_path, 'FileType', 'text');
        method_names(end + 1, 1) = sprintf("原分段lag%d RTS", lag_segments);
        raw_method_nav{end + 1, 1} = segmented_nav;
    else
        warning('缺少原分段固定滞后结果，跳过：%s', segmented_path);
    end

    global_path = fullfile(cfg.outputfolder, ...
        sprintf('standard-global-rts-%s', position_error_unit), ...
        'range-ins-rts-standard-global.nav');
    if isfile(global_path)
        global_nav = readmatrix(global_path, 'FileType', 'text');
        method_names(end + 1, 1) = "标准全局RTS";
        raw_method_nav{end + 1, 1} = global_nav;
    end
end

method_count = numel(method_names);
common_start_time = max(cellfun(@(nav) nav(1, 2), raw_method_nav));
common_end_time = min(cellfun(@(nav) nav(end, 2), raw_method_nav));
new_nav = new_nav(new_nav(:, 2) >= common_start_time & ...
    new_nav(:, 2) <= common_end_time, :);
if isempty(new_nav)
    error('各对比方法没有共同的严格成熟区间。');
end
method_nav = cell(method_count, 1);
for method_index = 1:method_count
    method_nav{method_index} = align_nav_to_time( ...
        raw_method_nav{method_index}, new_nav(:, 2));
end
statistics = zeros(method_count, 8);
horizontal_errors = cell(method_count, 1);
for method_index = 1:method_count
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
    'standard-fixed-lag-comparison.csv');
writetable(comparison, comparison_path);
disp(comparison);

%% 4. 水平误差和相邻位置步长曲线
plot_indices = select_plot_indices(size(new_nav, 1), 6000);
elapsed_min = (new_nav(plot_indices, 2) - new_nav(1, 2)) / 60;
comparison_figure = figure('Color', 'w', 'Visible', 'off', ...
    'Position', [100, 100, 1200, 760]);
tiledlayout(comparison_figure, 2, 1, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
nexttile;
hold on;
for method_index = 1:method_count
    plot(elapsed_min, horizontal_errors{method_index}(plot_indices), ...
        'LineWidth', 1.0);
end
grid on;
ylabel('水平误差 / m');
title(sprintf('%s / %s：标准固定滞后RTS', ...
    char(data_source), case_name));
legend(cellstr(method_names), 'Location', 'best');

nexttile;
hold on;
step_plot_indices = plot_indices(plot_indices > 1);
for method_index = 1:method_count
    position_step = calculate_horizontal_step_m(method_nav{method_index});
    plot((new_nav(step_plot_indices, 2) - new_nav(1, 2)) / 60, ...
        position_step(step_plot_indices - 1), 'LineWidth', 1.0);
end
grid on;
xlabel('成熟输出相对时间 / min');
ylabel('相邻位置步长 / m');
legend(cellstr(method_names), 'Location', 'best');
figure_path = fullfile(result_dir, ...
    'standard-fixed-lag-horizontal-error-and-smoothness.png');
exportgraphics(comparison_figure, figure_path, 'Resolution', 200);
close(comparison_figure);

fprintf('标准固定滞后RTS完成，耗时%.2f s。\n', elapsed_s);
fprintf('严格成熟结果：%s\n', outputs.standard_fixed_lag_rts_path);
fprintf('事件诊断：%s\n', ...
    outputs.standard_fixed_lag_rts_diagnostic_path);
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
        error('对比结果没有覆盖标准固定滞后RTS的严格成熟区间。');
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
