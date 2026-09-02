clear;
close all;

%% 前向 EKF 与全部 RTS 派生方法统一对比
% 本脚本只读取已经生成的导航结果，不重新运行各导航算法。
% 统一对比：前向 EKF、一次 RTS、二次 RTS、2RTS+旋转收缩、
% 2RTS+位置约束 INS 重放、2RTS+位置速度约束 INS 重放。
% 第11个测距点位于4620 s，评价截止到4621 s，以包含更新后1 s结果。

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));
project_root = fileparts(fileparts(topic_dir));

case_name = 'case-01';
end_time_s = 4621;
range_interval_s = 420;
result_dir = fullfile(project_root, 'data', 'inertial-experiment', ...
    'algorithm-exploration', 'navigation-results', 'simulation', 'forward-backward');
figure_dir = exploration_artifact_dir(result_dir);
truth_path = fullfile(project_root, 'data', 'inertial-experiment', ...
    'algorithm-exploration', 'input', 'simulation', case_name, 'truth.txt');

result_files = {
    fullfile(result_dir, 'range-ins-forward.nav'), ...
    fullfile(result_dir, 'range-ins-rts-single.nav'), ...
    fullfile(result_dir, 'range-ins-rts-double.nav'), ...
    fullfile(result_dir, 'range-ins-rts-double-bridge-rotation.nav'), ...
    fullfile(result_dir, 'range-ins-double-rts-position-guided-replay.nav'), ...
    fullfile(result_dir, ...
        'range-ins-double-rts-position-velocity-guided-replay.nav')};
result_labels = {
    '前向 EKF', ...
    '一次 RTS', ...
    '二次 RTS', ...
    '2RTS+旋转收缩', ...
    '2RTS+位置约束', ...
    '2RTS+位置速度约束'};

figure_path = fullfile(figure_dir, 'all-rts-methods-comparison.png');
figure_source_path = fullfile(figure_dir, 'all-rts-methods-comparison.fig');
statistics_path = fullfile(exploration_artifact_dir(result_dir), ...
    'all-rts-methods-statistics.csv');

required_files = [{truth_path}, result_files];
for file_index = 1:numel(required_files)
    if ~isfile(required_files{file_index})
        error(['缺少结果文件：%s\n请依次运行 RTS、旋转收缩、位置约束', ...
            '和位置速度约束脚本。'], required_files{file_index});
    end
end
if ~exist(figure_dir, 'dir')
    mkdir(figure_dir);
end

%% 读取真值与六种导航结果
truth_raw = readmatrix(truth_path, 'FileType', 'text');
truth_time = truth_raw(:, 2);
truth_position = truth_raw(:, 3:5);
clear truth_raw;

result_count = numel(result_files);
results = repmat(struct('time', [], 'position', [], ...
    'radial_error', []), result_count, 1);
reference_time = [];

for result_index = 1:result_count
    nav_raw = readmatrix(result_files{result_index}, 'FileType', 'text');
    nav_raw = nav_raw(nav_raw(:, 2) <= end_time_s + 1e-9, :);
    if isempty(nav_raw)
        error('结果文件在评价时段内没有数据：%s', result_files{result_index});
    end
    results(result_index).time = nav_raw(:, 2);
    results(result_index).position = nav_raw(:, 3:5);
    clear nav_raw;

    if isempty(reference_time)
        reference_time = results(result_index).time;
    elseif numel(results(result_index).time) ~= numel(reference_time) || ...
            max(abs(results(result_index).time - reference_time)) > 1e-7
        error(['六种结果的时间轴不一致：%s\n请先按4621 s截止时间', ...
            '重新运行全部仿真脚本。'], result_files{result_index});
    end
end

if reference_time(end) < end_time_s - 0.02
    error(['结果只到 %.2f s，尚未包含第11个测距点之后的1 s。', ...
        '请重新运行全部算法脚本。'], reference_time(end));
end

truth_at_result = interp1(truth_time, truth_position, reference_time, ...
    'linear', 'extrap');
clear truth_time truth_position;

for result_index = 1:result_count
    results(result_index).radial_error = calculate_horizontal_radial_error( ...
        results(result_index).position, truth_at_result);
end

%% 统一误差统计
rmse_m = zeros(result_count, 1);
mean_m = zeros(result_count, 1);
median_m = zeros(result_count, 1);
p95_m = zeros(result_count, 1);
maximum_m = zeros(result_count, 1);
for result_index = 1:result_count
    values = results(result_index).radial_error;
    values = values(isfinite(values));
    rmse_m(result_index) = sqrt(mean(values .^ 2));
    mean_m(result_index) = mean(values);
    median_m(result_index) = median(values);
    p95_m(result_index) = calculate_percentile(values, 95);
    maximum_m(result_index) = max(values);
end

% 正值表示误差减小；二次 RTS 自身相对二次 RTS 的改善率为0。
improvement_vs_forward_percent = 100 * (rmse_m(1) - rmse_m) / rmse_m(1);
improvement_vs_double_rts_percent = ...
    100 * (rmse_m(3) - rmse_m) / rmse_m(3);
method = string(result_labels(:));
statistics = table(method, rmse_m, mean_m, median_m, p95_m, maximum_m, ...
    improvement_vs_forward_percent, improvement_vs_double_rts_percent, ...
    'VariableNames', {'Method', 'RMSE_m', 'Mean_m', 'Median_m', ...
    'P95_m', 'Maximum_m', 'RMSEImprovementVsForward_percent', ...
    'RMSEImprovementVsDoubleRTS_percent'});
writetable(statistics, statistics_path);

%% 平面轨迹与水平径向误差
maximum_plot_points = 25000;
plot_count = min(maximum_plot_points, numel(reference_time));
display_index = unique(round(linspace(1, numel(reference_time), plot_count)));
origin = truth_at_result(1, :);
[truth_east, truth_north] = position_to_local_plane( ...
    truth_at_result(display_index, :), origin);

colors = [
    0.10, 0.35, 0.75; ... % 前向 EKF
    0.90, 0.45, 0.10; ... % 一次 RTS
    0.05, 0.55, 0.55; ... % 二次 RTS
    0.78, 0.16, 0.52; ... % 旋转收缩
    0.45, 0.30, 0.75; ... % 位置约束
    0.20, 0.65, 0.35];    % 位置速度约束
line_styles = {'-', '--', '-.', ':', '--', '-'};
comparison_figure = myfigurestartup(10,5,'prese');
% comparison_figure = figure('Color', 'w', ...
%     'Name', '全部 RTS 方法统一对比', 'Position', [60, 100, 1600, 650]);
layout = tiledlayout(comparison_figure, 1, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, sprintf('%s：截止第11个测距点后1 s（0–%d s）', ...
    case_name, end_time_s), 'FontWeight', 'bold','fontsize',15);

nexttile(layout, 1);
plot(truth_east / 1000, truth_north / 1000, 'k-', ...
    'LineWidth', 1.8, 'DisplayName', '真值');
hold on;
for result_index = 1:result_count
    [east, north] = position_to_local_plane( ...
        results(result_index).position(display_index, :), origin);
    plot(east / 1000, north / 1000, ...
        'Color', colors(result_index, :), ...
        'LineStyle', line_styles{result_index}, ...
        'LineWidth', 1.25, 'DisplayName', result_labels{result_index});
end
axis tight;
grid on;
box on;
xlabel('东向位置（km）');
ylabel('北向位置（km）');
title('平面轨迹（横纵轴独立缩放）');
legend('Location', 'best', 'Interpreter', 'none');

nexttile(layout, 2);
hold on;
for event_time = range_interval_s:range_interval_s:4620
    xline(event_time, ':', 'Color', [0.78, 0.78, 0.78], ...
        'HandleVisibility', 'off');
end
for result_index = 1:result_count
    plot(reference_time(display_index), ...
        results(result_index).radial_error(display_index), ...
        'Color', colors(result_index, :), ...
        'LineStyle', line_styles{result_index}, ...
        'LineWidth', 1.2, 'DisplayName', result_labels{result_index});
end
grid on;
box on;
xlim([0, end_time_s]);
xlabel('时间（s）');
ylabel('水平径向误差（m）');
title('水平径向误差（灰色虚线为测距时刻）');
legend('Location', 'best', 'Interpreter', 'none');

set(findall(comparison_figure, '-property', 'FontName'), ...
    'FontName', 'TimesSimSun');
exportgraphics(comparison_figure, figure_path, 'Resolution', 600);
savefig(comparison_figure, figure_source_path);

fprintf('\n全部 RTS 相关方法统一误差统计（截止 %.0f s）：\n', end_time_s);
disp(statistics);
fprintf('对比图：%s\n', figure_path);
fprintf('统计表：%s\n', statistics_path);

%% 局部函数
function radial_error = calculate_horizontal_radial_error( ...
        estimated_position, truth_position)
% 按 WGS-84 曲率半径把经纬度误差逐点转换为水平米制误差。
    latitude = deg2rad(truth_position(:, 1));
    height = truth_position(:, 3);
    [meridian_radius, prime_vertical_radius] = wgs84_radii(latitude);
    north_error = deg2rad(estimated_position(:, 1) ...
        - truth_position(:, 1)) .* (meridian_radius + height);
    east_error = deg2rad(estimated_position(:, 2) ...
        - truth_position(:, 2)) .* (prime_vertical_radius + height) ...
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
