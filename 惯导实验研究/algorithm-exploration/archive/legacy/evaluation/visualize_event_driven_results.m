clear;
close all;

%% 测距事件驱动算法结果的统一评估与可视化
% 本脚本沿用 calc_radial_error_gjb.m 的误差定义：
% 根据每个真值点的纬度和高度计算 WGS-84 曲率半径，将经纬度误差逐点转换为
% 北向、东向米制误差，再计算水平径向误差。
%
% 对比结果：
%   1）前向 AEKF；
%   2）AEKF 锚定的纯反向 INS；
%   3）利用当前反推终点，延迟修正上一段红色反推轨迹的几何变换方法。

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));
project_root = fileparts(fileparts(topic_dir));

case_name = 'case-01';
truth_path = fullfile(project_root, 'data', 'inertial-experiment', ...
    'algorithm-exploration', 'input', 'simulation', case_name, 'truth.txt');
result_dir = fullfile(project_root, 'data', 'inertial-experiment', ...
    'algorithm-exploration', 'navigation-results', 'simulation', 'forward-backward');
figure_dir = exploration_artifact_dir(result_dir);
if ~exist(figure_dir, 'dir')
    mkdir(figure_dir);
end

result_files = {
    fullfile(result_dir, 'range-ins-forward.nav'), ...
    fullfile(result_dir, 'range-ins-backward-constrained.nav'), ...
    fullfile(result_dir, 'range-ins-delayed-previous-geometry.nav')};
result_labels = { ...
    '前向 AEKF', ...
    'BRC（AEKF 锚定纯反向 INS）', ...
    'DPG-BRC（延迟修正上一段红色轨迹）'};
diagnostic_path = fullfile(exploration_artifact_dir(result_dir), ...
    'range-segment-diagnostics.csv');

required_files = [{truth_path}, result_files, {diagnostic_path}];
for file_index = 1:numel(required_files)
    if ~isfile(required_files{file_index})
        error('缺少评估输入文件：%s', required_files{file_index});
    end
end

%% 读取真值、导航结果和测距区间诊断数据
truth_raw = readmatrix(truth_path);
truth_time = truth_raw(:, 2);
truth_position = truth_raw(:, 3:5);       % [纬度(°), 经度(°), 高度(m)]

result_count = numel(result_files);
results = repmat(struct( ...
    'label', '', 'time', [], 'position', [], ...
    'north_error', [], 'east_error', [], 'height_error', [], ...
    'radial_error', []), result_count, 1);

% 四个结果文件由同一主脚本生成，时间轴应保持一致。仍以第一个结果为基准，
% 对其他文件进行检查，防止比较时发生静默错位。
reference_time = [];
for result_index = 1:result_count
    % .nav 是空格分隔的纯文本文件，显式指定文本类型以避免 MATLAB
    % 因自定义扩展名而无法判断读取器。
    nav_raw = readmatrix(result_files{result_index}, 'FileType', 'text');
    results(result_index).label = result_labels{result_index};
    results(result_index).time = nav_raw(:, 2);
    results(result_index).position = nav_raw(:, 3:5);

    if isempty(reference_time)
        reference_time = results(result_index).time;
    elseif numel(results(result_index).time) ~= numel(reference_time) || ...
            max(abs(results(result_index).time - reference_time)) > 1e-7
        error('导航结果的时间轴不一致：%s', result_files{result_index});
    end
end

truth_at_result = interp1(truth_time, truth_position, reference_time, ...
    'linear', 'extrap');
for result_index = 1:result_count
    [north_error, east_error, height_error, radial_error] = ...
        calculate_gjb_position_error(results(result_index).position, truth_at_result);
    results(result_index).north_error = north_error;
    results(result_index).east_error = east_error;
    results(result_index).height_error = height_error;
    results(result_index).radial_error = radial_error;
end

diagnostic_options = detectImportOptions(diagnostic_path, ...
    'VariableNamingRule', 'preserve');
segment_diagnostics = readtable(diagnostic_path, diagnostic_options);
segment_start = segment_diagnostics.('start-time');
segment_end = segment_diagnostics.('end-time');
segment_number = segment_diagnostics.('segment');

%% 计算全时段、有效双端点区间及各测距区间的统计指标
full_mask = true(size(reference_time));
constrained_mask = reference_time >= segment_start(1) & ...
    reference_time <= segment_end(end);

overall_statistics = table();
for result_index = 1:result_count
    overall_statistics = [overall_statistics; create_statistics_row( ...
        results(result_index).label, '全时段', ...
        results(result_index).radial_error(full_mask))]; %#ok<AGROW>
    overall_statistics = [overall_statistics; create_statistics_row( ...
        results(result_index).label, '双端点有效区间', ...
        results(result_index).radial_error(constrained_mask))]; %#ok<AGROW>
end

segment_count = numel(segment_number);
segment_rmse = zeros(segment_count, result_count);
segment_max = zeros(segment_count, result_count);
for segment_index = 1:segment_count
    mask = reference_time >= segment_start(segment_index) & ...
        reference_time <= segment_end(segment_index);
    for result_index = 1:result_count
        error_data = results(result_index).radial_error(mask);
        segment_rmse(segment_index, result_index) = sqrt(mean(error_data .^ 2));
        segment_max(segment_index, result_index) = max(error_data);
    end
end

backward_improvement = 100 * (segment_rmse(:, 1) - segment_rmse(:, 2)) ...
    ./ segment_rmse(:, 1);
delayed_improvement = 100 * (segment_rmse(:, 1) - segment_rmse(:, 3)) ...
    ./ segment_rmse(:, 1);
segment_statistics = table(segment_number, segment_start, segment_end, ...
    segment_rmse(:, 1), segment_rmse(:, 2), segment_rmse(:, 3), ...
    segment_max(:, 1), segment_max(:, 2), segment_max(:, 3), ...
    backward_improvement, delayed_improvement, ...
    'VariableNames', {'Segment', 'StartTime_s', 'EndTime_s', ...
    'ForwardRMSE_m', 'BackwardRMSE_m', 'DelayedGeometryRMSE_m', ...
    'ForwardMax_m', 'BackwardMax_m', 'DelayedGeometryMax_m', ...
    'BackwardImprovement_percent', 'DelayedGeometryImprovement_percent'});

writetable(overall_statistics, fullfile(figure_dir, ...
    'range-result-statistics.csv'));
writetable(segment_statistics, fullfile(figure_dir, ...
    'range-segment-statistics.csv'));

%% 图 1：轨迹与误差总览
colors = [0.15, 0.35, 0.70; 0.85, 0.33, 0.10; 0.55, 0.25, 0.70];
line_styles = {'-', '--', ':'};
display_index = unique(round(linspace(1, numel(reference_time), ...
    min(12000, numel(reference_time)))));

overview_figure = myfigurestartup(10, 6, 'prese');
layout = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, sprintf('%s：测距事件驱动算法结果总览', case_name));

nexttile;
[truth_east, truth_north] = position_to_local_plane( ...
    truth_at_result(display_index, :), truth_at_result(1, :));
plot(truth_east / 1000, truth_north / 1000, 'k-', ...
    'LineWidth', 1.8, 'DisplayName', '真值');
hold on;
for result_index = 1:result_count
    [east, north] = position_to_local_plane( ...
        results(result_index).position(display_index, :), truth_at_result(1, :));
    plot(east / 1000, north / 1000, ...
        'Color', colors(result_index, :), ...
        'LineStyle', line_styles{result_index}, 'LineWidth', 1.2, ...
        'DisplayName', results(result_index).label);
end
axis equal;
grid on;
box on;
xlabel('东向位置（km）');
ylabel('北向位置（km）');
title('平面轨迹');
legend('Location', 'best');

nexttile;
hold on;
for result_index = 1:result_count
    plot(reference_time(display_index), ...
        results(result_index).radial_error(display_index), ...
        'Color', colors(result_index, :), ...
        'LineStyle', line_styles{result_index}, 'LineWidth', 1.2, ...
        'DisplayName', results(result_index).label);
end
draw_range_boundaries(segment_start, segment_end);
grid on;
box on;
xlabel('时间（s）');
ylabel('水平径向误差（m）');
title('水平径向误差');
legend('Location', 'best');

nexttile;
hold on;
for result_index = 1:result_count
    plot(reference_time(display_index), ...
        results(result_index).north_error(display_index), ...
        'Color', colors(result_index, :), ...
        'LineStyle', line_styles{result_index}, 'LineWidth', 1.2, ...
        'DisplayName', results(result_index).label);
end
yline(0, 'k:', 'HandleVisibility', 'off');
draw_range_boundaries(segment_start, segment_end);
grid on;
box on;
xlabel('时间（s）');
ylabel('北向误差（m）');
title('北向位置误差');

nexttile;
hold on;
for result_index = 1:result_count
    plot(reference_time(display_index), ...
        results(result_index).east_error(display_index), ...
        'Color', colors(result_index, :), ...
        'LineStyle', line_styles{result_index}, 'LineWidth', 1.2, ...
        'DisplayName', results(result_index).label);
end
yline(0, 'k:', 'HandleVisibility', 'off');
draw_range_boundaries(segment_start, segment_end);
grid on;
box on;
xlabel('时间（s）');
ylabel('东向误差（m）');
title('东向位置误差');

set(findall(overview_figure, '-property', 'FontName'), ...
    'FontName', 'TimesSimSun');
exportgraphics(overview_figure, fullfile(figure_dir, ...
    'range-results-overview.png'), 'Resolution', 600);
savefig(overview_figure, fullfile(figure_dir, 'range-results-overview.fig'));

%% 图 2：各测距区间误差与改进率
segment_figure = myfigurestartup(10, 5, 'prese');
layout = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, sprintf('%s：相邻测距区间性能对比', case_name));

nexttile;
bar_handle = bar(segment_number, segment_rmse, 'grouped');
for result_index = 1:result_count
    bar_handle(result_index).FaceColor = colors(result_index, :);
end
grid on;
box on;
xlabel('测距区间编号');
ylabel('水平径向误差 RMSE（m）');
title('各区间 RMSE');
legend(result_labels, 'Location', 'best');

nexttile;
plot(segment_number, backward_improvement, '-o', ...
    'Color', colors(2, :), 'LineWidth', 1.5, ...
    'DisplayName', 'BRC 纯反向重建相对前向');
hold on;
plot(segment_number, delayed_improvement, '-d', ...
    'Color', colors(3, :), 'LineWidth', 1.5, ...
    'DisplayName', 'DPG-BRC 延迟上一段几何修正相对前向');
yline(0, 'k:', 'HandleVisibility', 'off');
grid on;
box on;
xlabel('测距区间编号');
ylabel('RMSE 改进率（%）');
title('相对前向 AEKF 的区间改进率（正值表示误差减小）');
legend('Location', 'best');

set(findall(segment_figure, '-property', 'FontName'), ...
    'FontName', 'TimesSimSun');
exportgraphics(segment_figure, fullfile(figure_dir, ...
    'range-segment-comparison.png'), 'Resolution', 600);
savefig(segment_figure, fullfile(figure_dir, 'range-segment-comparison.fig'));

%% 图 3：延迟修正上一段红色反推轨迹的几何参数
delayed_valid = ~isnan(segment_diagnostics.('delayed-rotation-angle-deg'));
delayed_segment = segment_number(delayed_valid);
delayed_angle = segment_diagnostics.('delayed-rotation-angle-deg');
delayed_scale = segment_diagnostics.('delayed-scale');
delayed_gap_before = segment_diagnostics.('delayed-gap-before-m');
delayed_gap_after = segment_diagnostics.('delayed-gap-after-m');
delayed_figure = myfigurestartup(10, 7, 'prese');
layout = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, sprintf('%s：延迟修正上一段红色反推轨迹', case_name));

nexttile;
plot(delayed_segment, delayed_angle(delayed_valid), ...
    '-o', 'Color', colors(3, :), 'LineWidth', 1.5);
yline(0, 'k:', 'HandleVisibility', 'off');
grid on;
box on;
xlabel('被修正的测距区间编号');
ylabel('旋转角（°）');
title('固定 t0-840 s 起点、移动 t0-420 s 终点所需的旋转角');

nexttile;
plot(delayed_segment, delayed_scale(delayed_valid), ...
    '-s', 'Color', colors(3, :), 'LineWidth', 1.5);
yline(1, 'k:', 'HandleVisibility', 'off');
grid on;
box on;
xlabel('被修正的测距区间编号');
ylabel('尺度因子');
title('上一段红色反推轨迹的尺度调整');

nexttile;
gap_comparison = [ ...
    delayed_gap_before(delayed_valid), delayed_gap_after(delayed_valid)];
bar(delayed_segment, gap_comparison, 'grouped');
grid on;
box on;
xlabel('被修正的测距区间编号');
ylabel('t-7 min 节点间距（m）');
title('上一段红色终点与当前反推终点的变换前后间距');
legend('变换前', '变换后', 'Location', 'best');

set(findall(delayed_figure, '-property', 'FontName'), ...
    'FontName', 'TimesSimSun');
exportgraphics(delayed_figure, fullfile(figure_dir, ...
    'range-delayed-geometry-diagnostics.png'), 'Resolution', 600);
savefig(delayed_figure, fullfile(figure_dir, ...
    'range-delayed-geometry-diagnostics.fig'));

disp('全时段及有效区间统计：');
disp(overall_statistics);
fprintf('图形输出目录：%s\n', figure_dir);
fprintf('统计结果目录：%s\n', result_dir);

%% 局部辅助函数
function [north_error, east_error, height_error, radial_error] = ...
    calculate_gjb_position_error(estimated_position, truth_position)
%CALCULATE_GJB_POSITION_ERROR 按逐点曲率半径计算米制位置误差。
% 与 calc_radial_error_gjb.m 保持一致：纬度差对应北向误差，经度差对应
% 东向误差；高度误差采用“真值减估计值”的天向符号约定。

    latitude = deg2rad(truth_position(:, 1));
    height = truth_position(:, 3);
    [meridian_radius, prime_vertical_radius] = wgs84_radii(latitude);

    latitude_error = deg2rad(estimated_position(:, 1) - truth_position(:, 1));
    longitude_error = deg2rad(estimated_position(:, 2) - truth_position(:, 2));
    north_error = latitude_error .* (meridian_radius + height);
    east_error = longitude_error .* (prime_vertical_radius + height) .* cos(latitude);
    height_error = truth_position(:, 3) - estimated_position(:, 3);
    radial_error = hypot(north_error, east_error);
end

function [meridian_radius, prime_vertical_radius] = wgs84_radii(latitude)
%WGS84_RADII 计算 WGS-84 子午圈和卯酉圈曲率半径。
    semi_major_axis = 6378137.0;
    flattening = 1 / 298.257223563;
    eccentricity_squared = flattening * (2 - flattening);
    denominator = sqrt(1 - eccentricity_squared .* sin(latitude) .^ 2);
    prime_vertical_radius = semi_major_axis ./ denominator;
    meridian_radius = semi_major_axis * (1 - eccentricity_squared) ...
        ./ denominator .^ 3;
end

function row = create_statistics_row(system_name, evaluation_scope, error_data)
%CREATE_STATISTICS_ROW 生成一行水平径向误差统计结果。
    row = table(string(system_name), string(evaluation_scope), ...
        sqrt(mean(error_data .^ 2)), max(error_data), mean(error_data), ...
        median(error_data), prctile(error_data, 95), ...
        'VariableNames', {'System', 'Scope', 'RMSE_m', 'Max_m', ...
        'Mean_m', 'Median_m', 'P95_m'});
end

function [east, north] = position_to_local_plane(position, origin)
%POSITION_TO_LOCAL_PLANE 将经纬度轨迹转换到以 origin 为原点的局部平面。
    origin_latitude = deg2rad(origin(1));
    [meridian_radius, prime_vertical_radius] = wgs84_radii(origin_latitude);
    north = deg2rad(position(:, 1) - origin(1)) * ...
        (meridian_radius + origin(3));
    east = deg2rad(position(:, 2) - origin(2)) * ...
        (prime_vertical_radius + origin(3)) * cos(origin_latitude);
end

function draw_range_boundaries(segment_start, segment_end)
%DRAW_RANGE_BOUNDARIES 用浅色竖线标出真实测距事件时刻。
    range_epochs = unique([segment_start; segment_end]);
    for epoch_index = 1:numel(range_epochs)
        xline(range_epochs(epoch_index), ':', 'Color', [0.72, 0.72, 0.72], ...
            'HandleVisibility', 'off');
    end
end
