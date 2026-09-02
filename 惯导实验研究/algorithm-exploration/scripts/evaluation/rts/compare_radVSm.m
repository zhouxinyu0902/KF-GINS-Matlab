clear;
close all;
clc;
% 对比仿真阶段 rad 和 m 两套状态链，可选择三方法或四方法
%% 仿真 rad/m 位置误差状态结果对比
% 本脚本只读取已有导航结果，不重新运行滤波。两条状态链使用同一真值、
% 同一时间交集和同一误差计算方法，输出统计表、轨迹图及径向误差图。

%% 用户配置
case_name = 'case-00';
result_scope = "simple";       % "simple" 或 "four-method"
end_time_s = 4621;
maximum_plot_points = 20000;

%% 路径与结果文件
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));
addpath(topic_dir);
paths = setup_inertial_experiment();

truth_path = fullfile(paths.simulation_input, case_name, 'truth.txt');
result_scope = lower(string(result_scope));
switch result_scope
    case "simple"
        method_names = ["前向 EKF", "一次 RTS", "二次 RTS"];
        rad_dir = fullfile(paths.simulation_navigation, case_name, ...
            'simple-ekf-rts-rad');
        meter_dir = fullfile(paths.simulation_navigation, case_name, ...
            'simple-ekf-rts-m');
        rad_names = { ...
            'simple-forward-ekf-rad.nav', ...
            'simple-rts-single-rad.nav', ...
            'simple-rts-double-rad.nav'};
        meter_names = { ...
            'simple-forward-ekf-m.nav', ...
            'simple-rts-single-m.nav', ...
            'simple-rts-double-m.nav'};
    case "four-method"
        method_names = ["前向 EKF", "二次 RTS", ...
            "2RTS+旋转收缩", "2RTS+位置速度约束"];
        rad_dir = fullfile(paths.simulation_navigation, case_name, ...
            'four-method-comparison-rad');
        meter_dir = fullfile(paths.simulation_navigation, case_name, ...
            'four-method-comparison-m');
        common_names = { ...
            'range-ins-forward.nav', ...
            'range-ins-rts-double.nav', ...
            'range-ins-rts-double-bridge-rotation.nav', ...
            'range-ins-double-rts-position-velocity-guided-replay.nav'};
        rad_names = common_names;
        meter_names = common_names;
    otherwise
        error('result_scope 只能设置为 "simple" 或 "four-method"。');
end

rad_paths = cellfun(@(name) fullfile(rad_dir, name), rad_names, ...
    'UniformOutput', false);
meter_paths = cellfun(@(name) fullfile(meter_dir, name), meter_names, ...
    'UniformOutput', false);
required_paths = [{truth_path}, rad_paths, meter_paths];
missing_mask = ~cellfun(@isfile, required_paths);
if any(missing_mask)
    error(['rad/m 对比结果不完整。请先分别运行两种单位的导航程序。\n', ...
        '缺失文件：\n%s'], strjoin(required_paths(missing_mask), newline));
end

artifact_dir = fullfile(paths.simulation_artifacts, case_name, ...
    'rad-vs-m', char(result_scope));
if ~isfolder(artifact_dir)
    mkdir(artifact_dir);
end

%% 读取结果并建立严格共同时间轴
truth = readmatrix(truth_path, 'FileType', 'text');
method_count = numel(method_names);
rad_nav = cell(method_count, 1);
meter_nav = cell(method_count, 1);
common_time = [];
for method_index = 1:method_count
    rad_nav{method_index} = readmatrix(rad_paths{method_index}, ...
        'FileType', 'text');
    meter_nav{method_index} = readmatrix(meter_paths{method_index}, ...
        'FileType', 'text');
    validate_nav(rad_nav{method_index}, rad_paths{method_index});
    validate_nav(meter_nav{method_index}, meter_paths{method_index});

    if isempty(common_time)
        common_time = rad_nav{method_index}(:, 2);
    else
        common_time = intersect(common_time, rad_nav{method_index}(:, 2), ...
            'stable');
    end
    common_time = intersect(common_time, meter_nav{method_index}(:, 2), ...
        'stable');
end
common_time = common_time(common_time <= end_time_s + 1e-8);
if isempty(common_time)
    error('rad/m 结果没有共同有效时间区间。');
end

truth_position = interp1(truth(:, 2), truth(:, 3:5), common_time, ...
    'linear', 'extrap');
rad_position = cell(method_count, 1);
meter_position = cell(method_count, 1);
radial_error_rad = zeros(numel(common_time), method_count);
radial_error_meter = zeros(numel(common_time), method_count);
state_difference = zeros(numel(common_time), method_count);

for method_index = 1:method_count
    rad_position{method_index} = interp1(rad_nav{method_index}(:, 2), ...
        rad_nav{method_index}(:, 3:5), common_time, 'linear');
    meter_position{method_index} = interp1(meter_nav{method_index}(:, 2), ...
        meter_nav{method_index}(:, 3:5), common_time, 'linear');
    radial_error_rad(:, method_index) = horizontal_radial_error( ...
        rad_position{method_index}, truth_position);
    radial_error_meter(:, method_index) = horizontal_radial_error( ...
        meter_position{method_index}, truth_position);
    state_difference(:, method_index) = horizontal_radial_error( ...
        rad_position{method_index}, meter_position{method_index});
end

%% 统一统计
rad_rmse_m = column_rmse(radial_error_rad);
meter_rmse_m = column_rmse(radial_error_meter);
rad_mean_m = mean(radial_error_rad, 1)';
meter_mean_m = mean(radial_error_meter, 1)';
rad_p95_m = column_percentile(radial_error_rad, 95);
meter_p95_m = column_percentile(radial_error_meter, 95);
rad_maximum_m = max(radial_error_rad, [], 1)';
meter_maximum_m = max(radial_error_meter, [], 1)';
rad_minus_meter_rmse_m = rad_rmse_m - meter_rmse_m;
output_difference_rmse_m = column_rmse(state_difference);
output_difference_maximum_m = max(state_difference, [], 1)';
method = method_names(:);

statistics = table(method, rad_rmse_m, meter_rmse_m, ...
    rad_minus_meter_rmse_m, rad_mean_m, meter_mean_m, ...
    rad_p95_m, meter_p95_m, rad_maximum_m, meter_maximum_m, ...
    output_difference_rmse_m, output_difference_maximum_m, ...
    'VariableNames', {'Method', 'RadState_RMSE_m', 'MeterState_RMSE_m', ...
    'RadMinusMeter_RMSE_m', 'RadState_Mean_m', 'MeterState_Mean_m', ...
    'RadState_P95_m', 'MeterState_P95_m', 'RadState_Maximum_m', ...
    'MeterState_Maximum_m', 'RadVsMeter_OutputDifference_RMSE_m', ...
    'RadVsMeter_OutputDifference_Maximum_m'});
statistics_path = fullfile(artifact_dir, ...
    'simulation-rad-vs-m-statistics.csv');
writetable(statistics, statistics_path);

%% 水平径向误差对比
display_index = unique(round(linspace(1, numel(common_time), ...
    min(maximum_plot_points, numel(common_time)))));
error_figure = myfigurestartup(7.2, max(5.2, 2.0 * method_count), 'paper');
error_layout = tiledlayout(error_figure, method_count, 1, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
title(error_layout, sprintf('%s：rad 与 m 位置误差状态的水平误差对比', ...
    case_name));
for method_index = 1:method_count
    nexttile(error_layout);
    plot(common_time(display_index), ...
        radial_error_rad(display_index, method_index), '-', ...
        'LineWidth', 1.1, 'DisplayName', 'rad状态');
    hold on;
    plot(common_time(display_index), ...
        radial_error_meter(display_index, method_index), '--', ...
        'LineWidth', 1.1, 'DisplayName', 'm状态');
    grid on;
    box on;
    ylabel('误差（m）');
    title(sprintf('%s：RMSE %.2f / %.2f m', method_names(method_index), ...
        rad_rmse_m(method_index), meter_rmse_m(method_index)));
    if method_index == 1
        legend('Location', 'best');
    end
    if method_index == method_count
        xlabel('时间（s）');
    end
end
apply_figure_font(error_figure);
error_figure_path = fullfile(artifact_dir, ...
    'simulation-rad-vs-m-radial-error.png');
exportgraphics(error_figure, error_figure_path, 'Resolution', 600);
savefig(error_figure, replace(error_figure_path, '.png', '.fig'));

%% 平面轨迹对比
origin = truth_position(1, :);
[truth_east, truth_north] = position_to_local_en( ...
    truth_position(display_index, :), origin);
trajectory_figure = myfigurestartup(7.2, max(5.2, 2.5 * method_count), 'paper');
trajectory_layout = tiledlayout(trajectory_figure, method_count, 1, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
title(trajectory_layout, sprintf('%s：rad 与 m 导航轨迹对比', case_name));
for method_index = 1:method_count
    [rad_east, rad_north] = position_to_local_en( ...
        rad_position{method_index}(display_index, :), origin);
    [meter_east, meter_north] = position_to_local_en( ...
        meter_position{method_index}(display_index, :), origin);
    nexttile(trajectory_layout);
    plot(truth_east / 1000, truth_north / 1000, 'k-', ...
        'LineWidth', 1.4, 'DisplayName', '真值');
    hold on;
    plot(rad_east / 1000, rad_north / 1000, '-', ...
        'LineWidth', 1.1, 'DisplayName', 'rad状态');
    plot(meter_east / 1000, meter_north / 1000, '--', ...
        'LineWidth', 1.1, 'DisplayName', 'm状态');
    axis equal;
    grid on;
    box on;
    xlabel('东向（km）');
    ylabel('北向（km）');
    title(method_names(method_index));
    if method_index == 1
        legend('Location', 'best');
    end
end
apply_figure_font(trajectory_figure);
trajectory_figure_path = fullfile(artifact_dir, ...
    'simulation-rad-vs-m-trajectory.png');
exportgraphics(trajectory_figure, trajectory_figure_path, 'Resolution', 600);
savefig(trajectory_figure, replace(trajectory_figure_path, '.png', '.fig'));

%% RMSE 汇总柱状图
rmse_figure = myfigurestartup(7.2, 4.6, 'paper');
bar(categorical(method_names, method_names), ...
    [rad_rmse_m, meter_rmse_m], 'grouped');
grid on;
box on;
ylabel('水平位置 RMSE（m）');
title(sprintf('%s：rad/m状态链RMSE汇总', case_name));
legend({'rad状态', 'm状态'}, 'Location', 'best');
apply_figure_font(rmse_figure);
rmse_figure_path = fullfile(artifact_dir, ...
    'simulation-rad-vs-m-rmse.png');
exportgraphics(rmse_figure, rmse_figure_path, 'Resolution', 600);
savefig(rmse_figure, replace(rmse_figure_path, '.png', '.fig'));

fprintf('\n仿真 rad/m 状态链对比完成：%s，范围=%s。\n', ...
    case_name, result_scope);
disp(statistics(:, 1:4));
fprintf('统计表：%s\n', statistics_path);
fprintf('轨迹图：%s\n', trajectory_figure_path);
fprintf('径向误差图：%s\n', error_figure_path);

%% 局部函数
function validate_nav(nav, file_path)
    if size(nav, 2) < 5 || size(nav, 1) < 2
        error('导航结果格式不正确：%s', file_path);
    end
    if any(~isfinite(nav(:, 2))) || any(diff(nav(:, 2)) <= 0)
        error('导航结果时间轴无效：%s', file_path);
    end
end

function values = column_rmse(data)
    values = sqrt(mean(data .^ 2, 1))';
end

function values = column_percentile(data, percentile)
    values = zeros(size(data, 2), 1);
    for column_index = 1:size(data, 2)
        sorted_data = sort(data(:, column_index));
        location = 1 + (numel(sorted_data) - 1) * percentile / 100;
        lower_index = floor(location);
        upper_index = ceil(location);
        if lower_index == upper_index
            values(column_index) = sorted_data(lower_index);
        else
            weight = location - lower_index;
            values(column_index) = (1 - weight) * sorted_data(lower_index) ...
                + weight * sorted_data(upper_index);
        end
    end
end

function radial_error = horizontal_radial_error(estimate, reference)
    latitude = deg2rad(reference(:, 1));
    height = reference(:, 3);
    [rm, rn] = wgs84_radii(latitude);
    north_error = deg2rad(estimate(:, 1) - reference(:, 1)) ...
        .* (rm + height);
    east_error = deg2rad(estimate(:, 2) - reference(:, 2)) ...
        .* (rn + height) .* cos(latitude);
    radial_error = hypot(north_error, east_error);
end

function [east, north] = position_to_local_en(position, origin)
    latitude_0 = deg2rad(origin(1));
    [rm, rn] = wgs84_radii(latitude_0);
    north = deg2rad(position(:, 1) - origin(1)) * (rm + origin(3));
    east = deg2rad(position(:, 2) - origin(2)) ...
        * (rn + origin(3)) * cos(latitude_0);
end

function [rm, rn] = wgs84_radii(latitude)
    semi_major_axis = 6378137;
    flattening = 1 / 298.257223563;
    eccentricity_squared = flattening * (2 - flattening);
    denominator = sqrt(1 - eccentricity_squared .* sin(latitude) .^ 2);
    rn = semi_major_axis ./ denominator;
    rm = semi_major_axis * (1 - eccentricity_squared) ./ denominator .^ 3;
end

function apply_figure_font(figure_handle)
    set(findall(figure_handle, '-property', 'FontName'), ...
        'FontName', 'TimesSimSun');
end
