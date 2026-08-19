clear;
close all;

%% 前向 EKF、二次 RTS及2RTS+旋转收缩结果对比
% 本脚本只读取已经生成的导航结果，不重新运行导航算法。
% 对比对象：
%   1）前向 EKF；
%   2）二次 RTS；
%   3）2RTS+旋转收缩：利用残余桥接误差修正二次 RTS 轨迹。
% 输出内容仅包括平面轨迹、水平径向误差和汇总统计表。

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
project_root = fileparts(fileparts(topic_dir));

case_name = 'case-01';
end_time_s = 4621;
result_dir = fullfile(project_root, 'data', 'inertial-experiment', ...
    'algorithm-exploration', 'navigation-results', 'simulation', 'forward-backward');
figure_dir = exploration_artifact_dir(result_dir);
truth_path = fullfile(project_root, 'data', 'inertial-experiment', ...
    'algorithm-exploration', 'input', 'simulation', case_name, 'truth.txt');

result_files = {
    fullfile(result_dir, 'range-ins-forward.nav'), ...
    fullfile(result_dir, 'range-ins-rts-double.nav'), ...
    fullfile(result_dir, 'range-ins-rts-double-bridge-rotation.nav')};
result_labels = {
    '前向 EKF', ...
    '二次 RTS', ...
    '2RTS+旋转收缩'};

figure_path = fullfile(figure_dir, ...
    'ekf-double-rts-rotation-comparison.png');
figure_source_path = fullfile(figure_dir, ...
    'ekf-double-rts-rotation-comparison.fig');
statistics_path = fullfile(exploration_artifact_dir(result_dir), ...
    'ekf-double-rts-rotation-statistics.csv');

required_files = [{truth_path}, result_files];
for file_index = 1:numel(required_files)
    if ~isfile(required_files{file_index})
        error('缺少结果分析输入文件：%s', required_files{file_index});
    end
end
if ~exist(figure_dir, 'dir')
    mkdir(figure_dir);
end

%% 读取真值和三种导航结果
truth_raw = readmatrix(truth_path, 'FileType', 'text');
truth_time = truth_raw(:, 2);
truth_position = truth_raw(:, 3:5);  % [纬度(°), 经度(°), 高度(m)]
clear truth_raw;

result_count = numel(result_files);
results = repmat(struct('time', [], 'position', [], ...
    'radial_error', []), result_count, 1);
reference_time = [];

for result_index = 1:result_count
    nav_raw = readmatrix(result_files{result_index}, 'FileType', 'text');
    nav_raw = nav_raw(nav_raw(:, 2) <= end_time_s + 1e-9, :);
    results(result_index).time = nav_raw(:, 2);
    results(result_index).position = nav_raw(:, 3:5);
    clear nav_raw;

    if isempty(reference_time)
        reference_time = results(result_index).time;
    elseif numel(results(result_index).time) ~= numel(reference_time) || ...
            max(abs(results(result_index).time - reference_time)) > 1e-7
        error('三份导航结果的时间轴不一致：%s', ...
            result_files{result_index});
    end
end

% 将真值插值到导航结果时刻，保证三种方法使用完全相同的比较基准。
truth_at_result = interp1(truth_time, truth_position, reference_time, ...
    'linear', 'extrap');
clear truth_time truth_position;

for result_index = 1:result_count
    results(result_index).radial_error = calculate_horizontal_radial_error( ...
        results(result_index).position, truth_at_result);
end

%% 计算全时段误差统计
rmse_m = zeros(result_count, 1);
mean_m = zeros(result_count, 1);
median_m = zeros(result_count, 1);
p95_m = zeros(result_count, 1);
maximum_m = zeros(result_count, 1);

for result_index = 1:result_count
    error_data = results(result_index).radial_error;
    error_data = error_data(isfinite(error_data));
    rmse_m(result_index) = sqrt(mean(error_data .^ 2));
    mean_m(result_index) = mean(error_data);
    median_m(result_index) = median(error_data);
    p95_m(result_index) = calculate_percentile(error_data, 95);
    maximum_m(result_index) = max(error_data);
end

% 正值表示相对于前向 EKF 有所改善，负值表示误差增大。
rmse_improvement_percent = 100 * (rmse_m(1) - rmse_m) / rmse_m(1);
method = result_labels(:);
statistics = table(method, rmse_m, mean_m, median_m, p95_m, maximum_m, ...
    rmse_improvement_percent, 'VariableNames', { ...
    'Method', 'RMSE_m', 'Mean_m', 'Median_m', 'P95_m', 'Maximum_m', ...
    'RMSEImprovement_percent'});
writetable(statistics, statistics_path);

fprintf('\n前向 EKF、二次 RTS及2RTS+旋转收缩误差统计：\n');
disp(statistics);

%% 绘制平面轨迹与水平径向误差
% 仅对显示数据降采样，全部误差指标仍由原始全频数据计算。
maximum_plot_points = 20000;
plot_count = min(maximum_plot_points, numel(reference_time));
display_index = unique(round(linspace(1, numel(reference_time), plot_count)));
time_minute = (reference_time - reference_time(1)) / 60;

colors = [
    0.12, 0.35, 0.70; ...
    0.05, 0.55, 0.55; ...
    0.80, 0.15, 0.55];
line_styles = {'-', '--', '-.'};

comparison_figure = figure('Color', 'w', ...
    'Name', 'EKF、二次 RTS及旋转收缩对比', ...
    'Position', [80, 120, 1500, 600]);
layout = tiledlayout(comparison_figure, 1, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, sprintf('%s：EKF、二次 RTS及2RTS+旋转收缩对比', case_name), ...
    'FontWeight', 'bold');

% 平面轨迹统一转换到以真值起点为原点的东北局部坐标系。
nexttile(layout, 1);
[truth_east, truth_north] = position_to_local_plane( ...
    truth_at_result(display_index, :), truth_at_result(1, :));
plot(truth_east / 1000, truth_north / 1000, 'k-', ...
    'LineWidth', 1.8, 'DisplayName', '真值');
hold on;
for result_index = 1:result_count
    [east, north] = position_to_local_plane( ...
        results(result_index).position(display_index, :), ...
        truth_at_result(1, :));
    plot(east / 1000, north / 1000, ...
        'Color', colors(result_index, :), ...
        'LineStyle', line_styles{result_index}, ...
        'LineWidth', 1.35, ...
        'DisplayName', result_labels{result_index});
end
% 本算例东向跨度远大于北向跨度。采用横纵轴独立缩放，便于观察
% 两种后处理造成的局部轨迹差异；坐标数值仍保持真实的 km 单位。
axis tight;
grid on;
box on;
xlabel('东向位置 (km)');
ylabel('北向位置 (km)');
title('平面轨迹（横纵轴独立缩放）');
legend('Location', 'best', 'Interpreter', 'none');
xlim([0, end_time_s / 60]);

nexttile(layout, 2);
hold on;
for result_index = 1:result_count
    plot(time_minute(display_index), ...
        results(result_index).radial_error(display_index), ...
        'Color', colors(result_index, :), ...
        'LineStyle', line_styles{result_index}, ...
        'LineWidth', 1.25, ...
        'DisplayName', result_labels{result_index});
end
grid on;
box on;
xlabel('相对时间 (min)');
ylabel('水平径向误差 (m)');
title('水平径向误差');
legend('Location', 'best', 'Interpreter', 'none');

set(findall(comparison_figure, '-property', 'FontName'), ...
    'FontName', 'Microsoft YaHei');
exportgraphics(comparison_figure, figure_path, 'Resolution', 300);
savefig(comparison_figure, figure_source_path);

fprintf('对比图已保存：%s\n', figure_path);
fprintf('统计表已保存：%s\n', statistics_path);

%% 局部函数
function radial_error = calculate_horizontal_radial_error( ...
        estimated_position, truth_position)
% 按 WGS-84 曲率半径把经纬度误差逐点转换为东北向米制误差。
    latitude = deg2rad(truth_position(:, 1));
    height = truth_position(:, 3);
    [meridian_radius, prime_vertical_radius] = wgs84_radii(latitude);

    latitude_error = deg2rad( ...
        estimated_position(:, 1) - truth_position(:, 1));
    longitude_error = deg2rad( ...
        estimated_position(:, 2) - truth_position(:, 2));

    north_error = latitude_error .* (meridian_radius + height);
    east_error = longitude_error .* (prime_vertical_radius + height) ...
        .* cos(latitude);
    radial_error = hypot(north_error, east_error);
end

function [east, north] = position_to_local_plane(position, origin)
% 将经纬高转换到以 origin 为原点的局部东北平面坐标。
    latitude_0 = deg2rad(origin(1));
    [meridian_radius, prime_vertical_radius] = wgs84_radii(latitude_0);
    north = deg2rad(position(:, 1) - origin(1)) ...
        .* (meridian_radius + origin(3));
    east = deg2rad(position(:, 2) - origin(2)) ...
        .* (prime_vertical_radius + origin(3)) .* cos(latitude_0);
end

function [meridian_radius, prime_vertical_radius] = wgs84_radii(latitude)
% 计算 WGS-84 子午圈和卯酉圈曲率半径。
    semi_major_axis = 6378137;
    flattening = 1 / 298.257223563;
    eccentricity_squared = flattening * (2 - flattening);
    denominator = sqrt(1 - eccentricity_squared .* sin(latitude) .^ 2);
    prime_vertical_radius = semi_major_axis ./ denominator;
    meridian_radius = semi_major_axis * (1 - eccentricity_squared) ...
        ./ denominator .^ 3;
end

function value = calculate_percentile(data, percentile)
% 线性插值百分位数，避免依赖额外工具箱。
    sorted_data = sort(data(:));
    if isempty(sorted_data)
        value = NaN;
        return;
    end
    position = 1 + (numel(sorted_data) - 1) * percentile / 100;
    lower_index = floor(position);
    upper_index = ceil(position);
    if lower_index == upper_index
        value = sorted_data(lower_index);
    else
        weight = position - lower_index;
        value = (1 - weight) * sorted_data(lower_index) ...
            + weight * sorted_data(upper_index);
    end
end
