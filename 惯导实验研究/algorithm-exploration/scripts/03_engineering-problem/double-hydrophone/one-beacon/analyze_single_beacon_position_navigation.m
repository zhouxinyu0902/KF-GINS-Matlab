clear; clc; close all;

%% case-07 单信标直接定位与三种前向导航结果可视化
case_id = 7;
phase_std_deg = 5;
range_interval_s = 60;
show_figures = false;

script_dir = fileparts(mfilename('fullpath'));
double_hydrophone_dir = fileparts(script_dir);
topic_dir = fileparts(fileparts(fileparts(double_hydrophone_dir)));
addpath(topic_dir);
paths = setup_inertial_experiment();
param = Param();

input_dir = paths.simulation_input(case_id);
result_dir = fullfile(paths.simulation_navigation(case_id), ...
    'navigation-results', sprintf( ...
    'one-beacon-range-azimuth-phasestd%ddeg-range%ds', ...
    phase_std_deg, range_interval_s));
truth_path = fullfile(input_dir, 'truth.txt');
range_path = fullfile(input_dir, 'range1.txt');
position_measurement_path = fullfile(result_dir, ...
    'one-beacon-position-depth-measurements.txt');
nav_paths = { ...
    fullfile(result_dir, 'one-beacon-range-only-forward.nav'), ...
    fullfile(result_dir, 'one-beacon-range-phase-azimuth-forward.nav'), ...
    fullfile(result_dir, 'one-beacon-position-depth-forward.nav')};
method_names = {'Range only', 'Range + azimuth', 'Position + depth'};
method_colors = [0.55, 0.55, 0.55; 0.08, 0.42, 0.72; 0.84, 0.24, 0.18];

required_paths = [{truth_path, range_path, position_measurement_path}, ...
    nav_paths];
missing = required_paths(~cellfun(@isfile, required_paths));
if ~isempty(missing)
    error('缺少单信标分析输入：%s', strjoin(missing, ', '));
end
if ~isfolder(result_dir)
    error('结果目录不存在：%s', result_dir);
end

%% 1. 读取真值、信标和每分钟直接定位结果
truth = readmatrix(truth_path, 'FileType', 'text');
truth = truth(all(isfinite(truth(:, [2, 3, 4, 5, 11])), 2), :);
[truth_time_s, truth_unique_index] = unique(truth(:, 2), 'stable');
truth = truth(truth_unique_index, :);
if numel(truth_time_s) < 2
    error('truth.txt 中有效真值点不足。');
end

range_data = readmatrix(range_path, 'FileType', 'text');
range_data = range_data(all(isfinite(range_data(:, 1:6)), 2), :);
beacon_rad_m = range_data(1, 4:6);

position_data = readmatrix(position_measurement_path, 'FileType', 'text');
if size(position_data, 2) < 23
    error('直接定位诊断文件没有有效的23列数据。');
end
position_data = position_data( ...
    all(isfinite(position_data(:, 1:2)), 2), :);
if isempty(position_data)
    error('直接定位诊断文件没有有效量测行。');
end

reference_lat_rad = deg2rad(truth(1, 3));
reference_lon_rad = deg2rad(truth(1, 4));
reference_height_m = truth(1, 5);
[reference_rm, reference_rn] = getRmRn(reference_lat_rad, param);
north_scale_m = reference_rm + reference_height_m;
east_scale_m = (reference_rn + reference_height_m) * ...
    cos(reference_lat_rad);

truth_stride = max(1, round(size(truth, 1)/5000));
truth_plot = truth(1:truth_stride:end, :);
[truth_east_m, truth_north_m] = geodetic_to_local( ...
    truth_plot(:, 3), truth_plot(:, 4), reference_lat_rad, ...
    reference_lon_rad, north_scale_m, east_scale_m);
[direct_east_m, direct_north_m] = geodetic_to_local( ...
    position_data(:, 8), position_data(:, 9), reference_lat_rad, ...
    reference_lon_rad, north_scale_m, east_scale_m);
[event_truth_east_m, event_truth_north_m] = geodetic_to_local( ...
    position_data(:, 10), position_data(:, 11), reference_lat_rad, ...
    reference_lon_rad, north_scale_m, east_scale_m);
[beacon_east_m, beacon_north_m] = geodetic_to_local( ...
    rad2deg(beacon_rad_m(1)), rad2deg(beacon_rad_m(2)), ...
    reference_lat_rad, reference_lon_rad, north_scale_m, east_scale_m);

position_error_m = position_data(:, 14);
position_used = position_data(:, 3) == 1;
position_cov_nn = position_data(:, 16).^2;
position_cov_ee = position_data(:, 17).^2;
position_cov_ne = position_data(:, 18);
position_major_std_m = sqrt(max(0, 0.5 * (position_cov_nn + ...
    position_cov_ee + sqrt((position_cov_nn-position_cov_ee).^2 + ...
    4*position_cov_ne.^2))));
valid_direct = isfinite(position_error_m);
direct_rmse_m = sqrt(mean(position_error_m(valid_direct).^2));
direct_mae_m = mean(abs(position_error_m(valid_direct)));

%% 2. 评估三种前向导航结果
metrics = nan(numel(nav_paths), 6);
navigation = cell(numel(nav_paths), 1);
for method_index = 1:numel(nav_paths)
    nav = readmatrix(nav_paths{method_index}, 'FileType', 'text');
    [nav, horizontal_error_m, yaw_error_deg] = evaluate_navigation( ...
        nav, truth_time_s, truth, param);
    [nav_east_m, nav_north_m] = geodetic_to_local( ...
        nav(:, 3), nav(:, 4), reference_lat_rad, reference_lon_rad, ...
        north_scale_m, east_scale_m);
    navigation{method_index} = struct( ...
        'data', nav, 'east_m', nav_east_m, 'north_m', nav_north_m, ...
        'horizontal_error_m', horizontal_error_m, ...
        'yaw_error_deg', yaw_error_deg);
    sorted_error = sort(horizontal_error_m);
    p95_index = max(1, ceil(0.95*numel(sorted_error)));
    metrics(method_index, :) = [ ...
        sqrt(mean(horizontal_error_m.^2)), ...
        mean(abs(horizontal_error_m)), sorted_error(p95_index), ...
        max(horizontal_error_m), horizontal_error_m(end), ...
        sqrt(mean(yaw_error_deg.^2))];
end

summary_table = table(string(method_names(:)), metrics(:, 1), ...
    metrics(:, 2), metrics(:, 3), metrics(:, 4), metrics(:, 5), ...
    metrics(:, 6), 'VariableNames', {'Method', 'HorizontalRMSE_m', ...
    'HorizontalMAE_m', 'HorizontalP95_m', 'HorizontalMax_m', ...
    'HorizontalFinal_m', 'YawRMSE_deg'});
summary_path = fullfile(result_dir, ...
    'one-beacon-forward-comparison-summary.txt');
writetable(summary_table, summary_path, 'Delimiter', '\t', ...
    'FileType', 'text');
disp(summary_table);

%% 3. 每分钟直接定位点及保守位置R
visibility = 'off';
if show_figures
    visibility = 'on';
end
direct_figure = figure('Color', 'w', 'Visible', visibility, ...
    'Position', [80, 80, 1220, 760]);
direct_layout = tiledlayout(direct_figure, 2, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');

trajectory_axis = nexttile(direct_layout, [2, 1]);
hold(trajectory_axis, 'on'); grid(trajectory_axis, 'on');
box(trajectory_axis, 'on'); axis(trajectory_axis, 'equal');
plot(trajectory_axis, truth_east_m/1000, truth_north_m/1000, ...
    'k-', 'LineWidth', 1.4, 'DisplayName', 'Truth trajectory');
for event_index = 1:size(position_data, 1)
    plot(trajectory_axis, ...
        [event_truth_east_m(event_index), direct_east_m(event_index)]/1000, ...
        [event_truth_north_m(event_index), direct_north_m(event_index)]/1000, ...
        '-', 'Color', [0.82, 0.82, 0.82], 'LineWidth', 0.5, ...
        'HandleVisibility', 'off');
end
scatter(trajectory_axis, direct_east_m/1000, direct_north_m/1000, ...
    28, position_data(:, 1)/60, 'filled', ...
    'DisplayName', '1-min direct positions');
plot(trajectory_axis, beacon_east_m/1000, beacon_north_m/1000, ...
    'p', 'MarkerSize', 14, 'MarkerFaceColor', [0.84, 0.24, 0.18], ...
    'MarkerEdgeColor', [0.55, 0.08, 0.06], ...
    'DisplayName', 'Beacon 1');
xlabel(trajectory_axis, 'East / km');
ylabel(trajectory_axis, 'North / km');
title(trajectory_axis, 'Every-minute direct positions');
legend(trajectory_axis, 'Location', 'best');
trajectory_colorbar = colorbar(trajectory_axis);
trajectory_colorbar.Label.String = 'Time / min';

error_axis = nexttile(direct_layout, 2);
hold(error_axis, 'on'); grid(error_axis, 'on'); box(error_axis, 'on');
plot(error_axis, position_data(:, 1)/60, position_error_m, ...
    '.-', 'Color', [0.84, 0.24, 0.18], 'LineWidth', 1.0, ...
    'MarkerSize', 10, 'DisplayName', 'Direct-position error');
plot(error_axis, position_data(:, 1)/60, position_major_std_m, ...
    '--', 'Color', [0.08, 0.42, 0.72], 'LineWidth', 1.3, ...
    'DisplayName', 'Major-axis R standard deviation');
xlabel(error_axis, 'Time / min'); ylabel(error_axis, 'Horizontal / m');
title(error_axis, sprintf('Direct RMSE %.1f m, MAE %.1f m', ...
    direct_rmse_m, direct_mae_m));
legend(error_axis, 'Location', 'best');

r_axis = nexttile(direct_layout, 4);
hold(r_axis, 'on'); grid(r_axis, 'on'); box(r_axis, 'on');
plot(r_axis, position_data(:, 1)/60, position_data(:, 16), ...
    '-', 'Color', [0.10, 0.55, 0.45], 'LineWidth', 1.1, ...
    'DisplayName', 'R std North');
plot(r_axis, position_data(:, 1)/60, position_data(:, 17), ...
    '-', 'Color', [0.55, 0.32, 0.70], 'LineWidth', 1.1, ...
    'DisplayName', 'R std East');
not_used = ~position_used;
plot(r_axis, position_data(not_used, 1)/60, ...
    position_major_std_m(not_used), 'rx', 'LineWidth', 1.5, ...
    'MarkerSize', 8, 'DisplayName', 'Fallback');
xlabel(r_axis, 'Time / min'); ylabel(r_axis, 'Position R std / m');
title(r_axis, sprintf('Position updates used: %d/%d', ...
    nnz(position_used), numel(position_used)));
legend(r_axis, 'Location', 'best');

sgtitle(direct_layout, sprintf( ...
    'case-07 single-beacon direct positioning, phase STD = %d deg', ...
    phase_std_deg));
direct_png = fullfile(result_dir, ...
    'one-beacon-direct-positioning-analysis.png');
direct_fig = fullfile(result_dir, ...
    'one-beacon-direct-positioning-analysis.fig');
exportgraphics(direct_figure, direct_png, 'Resolution', 300);
savefig(direct_figure, direct_fig);
if ~show_figures
    close(direct_figure);
end

%% 4. 三种前向导航方法对比
comparison_figure = figure('Color', 'w', 'Visible', visibility, ...
    'Position', [100, 80, 1280, 820]);
comparison_layout = tiledlayout(comparison_figure, 2, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');

nav_axis = nexttile(comparison_layout, 1);
hold(nav_axis, 'on'); grid(nav_axis, 'on'); box(nav_axis, 'on');
axis(nav_axis, 'equal');
plot(nav_axis, truth_east_m/1000, truth_north_m/1000, ...
    'k-', 'LineWidth', 1.6, 'DisplayName', 'Truth');
for method_index = 1:numel(navigation)
    nav_stride = max(1, round(numel(navigation{method_index}.east_m)/5000));
    idx = 1:nav_stride:numel(navigation{method_index}.east_m);
    plot(nav_axis, navigation{method_index}.east_m(idx)/1000, ...
        navigation{method_index}.north_m(idx)/1000, '-', ...
        'Color', method_colors(method_index, :), 'LineWidth', 1.1, ...
        'DisplayName', method_names{method_index});
end
xlabel(nav_axis, 'East / km'); ylabel(nav_axis, 'North / km');
title(nav_axis, 'Forward-EKF trajectories');
legend(nav_axis, 'Location', 'best');

all_error_axis = nexttile(comparison_layout, 2);
hold(all_error_axis, 'on'); grid(all_error_axis, 'on');
box(all_error_axis, 'on');
for method_index = 1:numel(navigation)
    semilogy(all_error_axis, navigation{method_index}.data(:, 2)/60, ...
        max(navigation{method_index}.horizontal_error_m, 1e-3), ...
        '-', 'Color', method_colors(method_index, :), ...
        'LineWidth', 1.1, 'DisplayName', method_names{method_index});
end
set(all_error_axis, 'YScale', 'log');
xlabel(all_error_axis, 'Time / min');
ylabel(all_error_axis, 'Horizontal error / m');
title(all_error_axis, 'Horizontal error, logarithmic scale');
legend(all_error_axis, 'Location', 'best');

zoom_axis = nexttile(comparison_layout, 3);
hold(zoom_axis, 'on'); grid(zoom_axis, 'on'); box(zoom_axis, 'on');
for method_index = 2:3
    plot(zoom_axis, navigation{method_index}.data(:, 2)/60, ...
        navigation{method_index}.horizontal_error_m, '-', ...
        'Color', method_colors(method_index, :), ...
        'LineWidth', 1.1, 'DisplayName', method_names{method_index});
end
xlabel(zoom_axis, 'Time / min'); ylabel(zoom_axis, 'Horizontal error / m');
title(zoom_axis, 'Angle-aided methods');
legend(zoom_axis, 'Location', 'best');

bar_axis = nexttile(comparison_layout, 4);
bar_handle = bar(bar_axis, metrics(:, [1, 3]), 'grouped');
bar_handle(1).FaceColor = [0.18, 0.48, 0.70];
bar_handle(2).FaceColor = [0.85, 0.52, 0.20];
grid(bar_axis, 'on'); box(bar_axis, 'on');
set(bar_axis, 'YScale', 'log');
ylim(bar_axis, [max(1, 0.7*min(metrics(:, [1, 3]), [], 'all')), ...
    1.35*max(metrics(:, [1, 3]), [], 'all')]);
xticks(bar_axis, 1:numel(method_names));
xticklabels(bar_axis, method_names);
xtickangle(bar_axis, 18);
ylabel(bar_axis, 'Horizontal error / m');
title(bar_axis, 'RMSE and P95');
legend(bar_axis, {'RMSE', 'P95'}, 'Location', 'best');

sgtitle(comparison_layout, ...
    'case-07 single-beacon forward-EKF comparison');
comparison_png = fullfile(result_dir, ...
    'one-beacon-forward-navigation-comparison.png');
comparison_fig = fullfile(result_dir, ...
    'one-beacon-forward-navigation-comparison.fig');
exportgraphics(comparison_figure, comparison_png, 'Resolution', 300);
savefig(comparison_figure, comparison_fig);
if ~show_figures
    close(comparison_figure);
end

fprintf('每分钟直接定位图：%s\n', direct_png);
fprintf('三种前向导航对比图：%s\n', comparison_png);
fprintf('前向导航误差汇总：%s\n', summary_path);

function [east_m, north_m] = geodetic_to_local(lat_deg, lon_deg, ...
        reference_lat_rad, reference_lon_rad, north_scale_m, east_scale_m)
north_m = (deg2rad(lat_deg)-reference_lat_rad)*north_scale_m;
east_m = (deg2rad(lon_deg)-reference_lon_rad)*east_scale_m;
end

function [nav, horizontal_error_m, yaw_error_deg] = ...
        evaluate_navigation(nav, truth_time_s, truth, param)
valid_nav = size(nav, 2) >= 11 & all(isfinite(nav(:, [2, 3, 4, 11])), 2);
nav = nav(valid_nav, :);
truth_lat_deg = interp1(truth_time_s, truth(:, 3), nav(:, 2), 'linear');
truth_lon_deg = interp1(truth_time_s, truth(:, 4), nav(:, 2), 'linear');
truth_height_m = interp1(truth_time_s, truth(:, 5), nav(:, 2), 'linear');
truth_yaw_rad = interp1(truth_time_s, unwrap(deg2rad(truth(:, 11))), ...
    nav(:, 2), 'linear');
valid = all(isfinite([truth_lat_deg, truth_lon_deg, truth_height_m, ...
    truth_yaw_rad]), 2);
nav = nav(valid, :);
truth_lat_deg = truth_lat_deg(valid);
truth_lon_deg = truth_lon_deg(valid);
truth_height_m = truth_height_m(valid);
truth_yaw_rad = truth_yaw_rad(valid);
north_error_m = nan(size(nav, 1), 1);
east_error_m = nan(size(nav, 1), 1);
for index = 1:size(nav, 1)
    truth_lat_rad = deg2rad(truth_lat_deg(index));
    [rm, rn] = getRmRn(truth_lat_rad, param);
    north_error_m(index) = deg2rad(nav(index, 3)-truth_lat_deg(index))* ...
        (rm+truth_height_m(index));
    east_error_m(index) = deg2rad(nav(index, 4)-truth_lon_deg(index))* ...
        (rn+truth_height_m(index))*cos(truth_lat_rad);
end
horizontal_error_m = hypot(north_error_m, east_error_m);
yaw_error_deg = rad2deg(atan2(sin(deg2rad(nav(:, 11))-truth_yaw_rad), ...
    cos(deg2rad(nav(:, 11))-truth_yaw_rad)));
end
