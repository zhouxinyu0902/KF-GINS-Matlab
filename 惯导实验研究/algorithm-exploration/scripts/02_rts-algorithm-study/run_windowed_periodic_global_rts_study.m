clear;
close all;
clc;
%% 2RTS + 周期窗口RTS：最近56 min，每42 min统一反推一次

%% 1. 用户配置
data_source = "experiment";
case_name = 'case-09';
position_error_unit = "m";
range_interval_s = 420;
global_rts_interval_s = 2520;          % 每42 min触发
global_rts_window_s = 3360;            % 最近56 min（8个测距区间）
duration_s = 6301;                     % 运行105 min，保留末尾21 min的2RTS暂定段
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
trigger_override = str2double(getenv('KF_GINS_GLOBAL_RTS_INTERVAL_S'));
if isfinite(trigger_override) && trigger_override > 0
    global_rts_interval_s = trigger_override;
end
window_override = str2double(getenv('KF_GINS_GLOBAL_RTS_WINDOW_S'));
if isfinite(window_override) && window_override > 0
    global_rts_window_s = window_override;
end
node_override = str2double(getenv('KF_GINS_GLOBAL_RTS_NODE_INTERVAL_S'));
if isfinite(node_override) && node_override > 0
    rts_node_interval_s = node_override;
end

event_stride = require_integer_multiple( ...
    global_rts_interval_s, range_interval_s, '窗口RTS触发周期');
window_segments = require_integer_multiple( ...
    global_rts_window_s, range_interval_s, '窗口RTS长度');

%% 2. 运行共享前向EKF、2RTS和周期窗口RTS
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
    'windowed-periodic-global-rts-%gmin-every-%gmin-%s', ...
    global_rts_window_s / 60, global_rts_interval_s / 60, ...
    position_error_unit));

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
    'standard_global_rts_window_segments', window_segments, ...
    'standard_global_rts_strict_output', true, ...
    'enable_standard_fixed_lag_rts', false, ...
    'return_double_rts_nav', true, ...
    'write_segmented_rts_outputs', false);

fprintf(['开始周期窗口RTS：%s/%s，窗口=%.1f min（%d段），' ...
    '每%.1f min（%d次测距）触发。\n'], ...
    data_source, case_name, global_rts_window_s / 60, window_segments, ...
    global_rts_interval_s / 60, event_stride);
run_timer = tic;
outputs = simulate_navigation_case( ...
    case_name, result_dir, false, false, runtime_options);
elapsed_s = toc(run_timer);

%% 3. 构造“窗口内RTS、未覆盖尾段2RTS”的在线混合输出
window_nav = readmatrix(outputs.standard_global_rts_path, 'FileType', 'text');
if isempty(window_nav)
    error(['运行时长内尚未达到一次窗口RTS触发条件；' ...
        '请增加duration_s或减小global_rts_interval_s。']);
end
forward_nav = readmatrix(outputs.forward_path, 'FileType', 'text');
double_rts_nav = outputs.double_rts_nav;
window_mask = outputs.standard_global_rts_mask;
if size(window_nav, 1) ~= nnz(window_mask)
    error('窗口RTS严格输出与内部有效掩码长度不一致。');
end
hybrid_nav = double_rts_nav;
hybrid_nav(window_mask, :) = window_nav;
hybrid_path = fullfile(result_dir, ...
    'range-ins-2rts-windowed-global-hybrid.nav');
write_nav_file_local(hybrid_path, hybrid_nav);

truth = readmatrix(cfg.truthpath, 'FileType', 'text');
strict_double = align_nav_to_time(double_rts_nav, window_nav(:, 2));
strict_forward = align_nav_to_time(forward_nav, window_nav(:, 2));
strict_table = evaluate_methods( ...
    ["周期窗口RTS"; "二次RTS"; "前向EKF"], ...
    {window_nav; strict_double; strict_forward}, truth, "严格已覆盖区间");
full_table = evaluate_methods( ...
    ["2RTS+周期窗口RTS"; "二次RTS"; "前向EKF"], ...
    {hybrid_nav; double_rts_nav; forward_nav}, truth, "完整运行区间");
comparison = [strict_table; full_table];
comparison_path = fullfile(result_dir, ...
    'windowed-periodic-global-rts-comparison.csv');
writetable(comparison, comparison_path);
disp(comparison);

%% 4. 误差曲线与窗口拼接诊断
diagnostics = readtable(outputs.standard_global_rts_diagnostic_path);
figure_path = fullfile(result_dir, ...
    'windowed-periodic-global-rts-error-and-boundary.png');
comparison_figure = figure('Color', 'w', 'Visible', 'off', ...
    'Position', [100, 100, 1250, 850]);
tiledlayout(comparison_figure, 3, 1, ...
    'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
strict_navs = {window_nav, strict_double, strict_forward};
strict_names = {'周期窗口RTS', '二次RTS', '前向EKF'};
for method_index = 1:numel(strict_navs)
    error_m = calculate_horizontal_error_m(strict_navs{method_index}, truth);
    plot((window_nav(:, 2) - window_nav(1, 2)) / 60, error_m, ...
        'LineWidth', 1.0, 'DisplayName', strict_names{method_index});
    hold on;
end
grid on;
ylabel('水平误差 / m');
title(sprintf('%s / %s：%.0f min窗口，每%.0f min触发', ...
    char(data_source), case_name, global_rts_window_s / 60, ...
    global_rts_interval_s / 60));
legend('Location', 'best');

nexttile;
full_navs = {hybrid_nav, double_rts_nav, forward_nav};
full_names = {'混合输出', '二次RTS', '前向EKF'};
for method_index = 1:numel(full_navs)
    error_m = calculate_horizontal_error_m(full_navs{method_index}, truth);
    plot((full_navs{method_index}(:, 2) - hybrid_nav(1, 2)) / 60, ...
        error_m, 'LineWidth', 1.0, 'DisplayName', full_names{method_index});
    hold on;
end
grid on;
xlabel('运行时间 / min');
ylabel('水平误差 / m');
title('完整在线输出：已触发历史用窗口RTS，末尾暂用2RTS');
legend('Location', 'best');

nexttile;
valid_boundary = isfinite(diagnostics.boundary_position_step_m);
if any(valid_boundary)
    diagnostic_elapsed_min = ...
        (diagnostics.time_s - window_nav(1, 2)) / 60;
    yyaxis left;
    plot(diagnostic_elapsed_min(valid_boundary), ...
        diagnostics.boundary_position_step_m(valid_boundary), 'o-', ...
        'LineWidth', 1.1);
    ylabel('窗口边界位置步长 / m');
    yyaxis right;
    plot(diagnostic_elapsed_min(valid_boundary), ...
        diagnostics.boundary_velocity_step_mps(valid_boundary), 's-', ...
        'LineWidth', 1.1);
    ylabel('窗口边界速度步长 / (m/s)');
else
    text(0.5, 0.5, '当前时长只有首次触发，尚无窗口拼接边界', ...
        'HorizontalAlignment', 'center');
end
grid on;
xlabel('触发时刻 / min');
title('相邻周期窗口的冻结/覆盖接缝');
exportgraphics(comparison_figure, figure_path, 'Resolution', 200);
close(comparison_figure);

fprintf('周期窗口RTS完成，总耗时%.2f s，共触发%d次。\n', ...
    elapsed_s, height(diagnostics));
fprintf('混合导航结果：%s\n', hybrid_path);
fprintf('触发诊断：%s\n', outputs.standard_global_rts_diagnostic_path);
fprintf('比较统计：%s\n', comparison_path);
fprintf('比较曲线：%s\n', figure_path);
if any(valid_boundary)
    fprintf('最大窗口边界位置步长：%.4f m；最大速度步长：%.4f m/s。\n', ...
        max(diagnostics.boundary_position_step_m(valid_boundary)), ...
        max(diagnostics.boundary_velocity_step_mps(valid_boundary)));
end

function count = require_integer_multiple(value, base, label)
    count = round(value / base);
    if count < 1 || abs(count * base - value) > 1e-9
        error('%s必须是测距间隔的正整数倍。', label);
    end
end

function statistics = evaluate_methods(names, navs, truth, scope)
    method_count = numel(navs);
    values = zeros(method_count, 8);
    for method_index = 1:method_count
        nav = navs{method_index};
        horizontal_error = calculate_horizontal_error_m(nav, truth);
        position_step = calculate_horizontal_step_m(nav);
        velocity_step = vecnorm(diff(nav(:, 6:8), 1, 1), 2, 2);
        values(method_index, :) = [ ...
            sqrt(mean(horizontal_error .^ 2)), mean(horizontal_error), ...
            prctile(horizontal_error, 95), max(horizontal_error), ...
            median(position_step), prctile(position_step, 99), ...
            max(position_step), max(velocity_step)];
    end
    statistics = table(repmat(string(scope), method_count, 1), ...
        string(names(:)), values(:, 1), values(:, 2), values(:, 3), ...
        values(:, 4), values(:, 5), values(:, 6), values(:, 7), ...
        values(:, 8), 'VariableNames', {'scope', 'method', ...
        'horizontal_rmse_m', 'horizontal_mean_m', 'horizontal_p95_m', ...
        'horizontal_max_m', 'position_step_median_m', ...
        'position_step_p99_m', 'position_step_max_m', ...
        'velocity_step_max_mps'});
end

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
        error('待比较导航结果没有覆盖窗口RTS输出区间。');
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

function write_nav_file_local(file_path, nav_data)
    file_id = fopen(file_path, 'w');
    if file_id < 0
        error('无法打开导航结果文件：%s', file_path);
    end
    cleaner = onCleanup(@() fclose(file_id));
    format = ['%2d %12.6f %12.8f %12.8f %8.4f %8.4f ', ...
        '%8.4f %8.4f %8.4f %8.4f %8.4f\n'];
    fprintf(file_id, format, nav_data');
    clear cleaner;
end
