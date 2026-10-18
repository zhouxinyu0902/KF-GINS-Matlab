function artifacts = generate_case070809_realtime_test_report(varargin)
%GENERATE_CASE070809_REALTIME_TEST_REPORT 生成case-07/08/09试验统计、图表和报告。
%   本函数以三次实测IMU数据、GNSS参考轨迹、模拟声学距离输入和严格
%   二次RTS输出为依据，重新计算表1、表2、轨迹对比图和位置误差图。
%   每段二次RTS均截止于其对应的测距节点；末尾仅完成一次RTS的区间
%   不纳入评价。
%
%   默认评价区间为每个前向导航结果与GNSS参考轨迹的完整公共有效时段。
%   多航次综合水平位置RMSE按全部有效历元合并后计算：
%       sqrt(sum(e_h.^2) / sum(N))
%
%   可选参数：
%     'OutputDir'  输出目录，默认位于本脚本同级目录下。
%     'Visible'    图窗是否可见，默认false。

%   示例：
%     generate_case070809_realtime_test_report;

%   说明：有效距离更新次数按评价时段内格式有效、距离为正的输入记录
%   统计。当前前向解算未单独输出量测拒收日志，因此该指标表示进入解算
%   时序的有效距离记录数，而不是基于创新检验的接受次数。

parser = inputParser;
parser.addParameter('OutputDir', '', @(x) ischar(x) || isstring(x));
parser.addParameter('Visible', false, @(x) islogical(x) && isscalar(x));
parser.parse(varargin{:});

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
project_root = fileparts(fileparts(topic_dir));
data_root = fullfile(project_root, 'data', 'inertial-experiment', ...
    'algorithm-exploration', 'experiment');
if strlength(string(parser.Results.OutputDir)) == 0
    output_dir = fullfile(script_dir, 'case07-09-test-report');
else
    output_dir = char(string(parser.Results.OutputDir));
end
if ~isfolder(output_dir)
    mkdir(output_dir);
end

case_ids = ["case-07", "case-08", "case-09"];
case_labels = ["第1次", "第2次", "第3次"];
case_count = numel(case_ids);
maximum_plot_points = 12000;

effective_duration_s = nan(case_count, 1);
imu_frequency_hz = nan(case_count, 1);
range_period_s = nan(case_count, 1);
range_observation_count = nan(case_count, 1);
valid_range_update_count = nan(case_count, 1);
navigation_frequency_hz = nan(case_count, 1);
navigation_anomaly_count = nan(case_count, 1);

east_rmse_m = nan(case_count, 1);
north_rmse_m = nan(case_count, 1);
horizontal_rmse_m = nan(case_count, 1);
maximum_horizontal_error_m = nan(case_count, 1);
evaluation_start_s = nan(case_count, 1);
evaluation_end_s = nan(case_count, 1);
valid_epoch_count = nan(case_count, 1);
horizontal_squared_error_sum = zeros(case_count, 1);

plot_data = repmat(struct('time_h', [], 'truth_east_m', [], ...
    'truth_north_m', [], 'nav_east_m', [], 'nav_north_m', [], ...
    'east_error_m', [], 'north_error_m', [], ...
    'horizontal_error_m', []), case_count, 1);

for case_index = 1:case_count
    case_root = fullfile(data_root, char(case_ids(case_index)));
    imu_path = fullfile(case_root, 'input', 'imu_120.txt');
    range_path = fullfile(case_root, 'input', 'range.txt');
    truth_path = fullfile(case_root, 'input', 'truth.nav');
    result_dir = fullfile(case_root, 'output', 'navigation-results', ...
        'simple-ekf-rts-rad');
    nav_path = fullfile(result_dir, 'simple-rts-double-rad.nav');
    strict_marker_path = fullfile(result_dir, ...
        'simple-rts-double-rad-strict-node-end.mat');
    assert(isfile(imu_path), '缺少IMU文件：%s', imu_path);
    assert(isfile(range_path), '缺少距离文件：%s', range_path);
    assert(isfile(truth_path), '缺少参考轨迹：%s', truth_path);
    assert(isfile(nav_path), '缺少前向组合导航结果：%s', nav_path);

    imu = readmatrix(imu_path, 'FileType', 'text');
    range_data = readmatrix(range_path, 'FileType', 'text');
    truth = readmatrix(truth_path, 'FileType', 'text');
    nav = readmatrix(nav_path, 'FileType', 'text');
    nav = select_strict_second_rts_rows(nav, strict_marker_path, ...
        range_data, case_ids(case_index));
    validate_inputs(imu, range_data, truth, nav, case_ids(case_index));

    imu_dt_s = median(diff(imu(:, 1)), 'omitnan');
    nav_dt_s = median(diff(nav(:, 2)), 'omitnan');
    imu_frequency_hz(case_index) = 1 / imu_dt_s;
    navigation_frequency_hz(case_index) = 1 / nav_dt_s;
    range_period_s(case_index) = median(diff(range_data(:, 1)), 'omitnan');

    evaluation_start_s(case_index) = max(nav(1, 2), truth(1, 2));
    evaluation_end_s(case_index) = min(nav(end, 2), truth(end, 2));
    effective_duration_s(case_index) = evaluation_end_s(case_index) ...
        - evaluation_start_s(case_index);

    nav_mask = nav(:, 2) >= evaluation_start_s(case_index) ...
        & nav(:, 2) <= evaluation_end_s(case_index);
    nav_eval = nav(nav_mask, :);
    reference_position = interp1(truth(:, 2), truth(:, 3:5), ...
        nav_eval(:, 2), 'linear', nan);
    [east_error, north_error] = geodetic_position_error_m( ...
        nav_eval(:, 3:5), reference_position);
    horizontal_error = hypot(east_error, north_error);
    valid = isfinite(east_error) & isfinite(north_error) ...
        & isfinite(horizontal_error);
    if ~all(valid)
        nav_eval = nav_eval(valid, :);
        reference_position = reference_position(valid, :);
        east_error = east_error(valid);
        north_error = north_error(valid);
        horizontal_error = horizontal_error(valid);
    end

    valid_epoch_count(case_index) = numel(horizontal_error);
    east_rmse_m(case_index) = sqrt(mean(east_error .^ 2));
    north_rmse_m(case_index) = sqrt(mean(north_error .^ 2));
    horizontal_rmse_m(case_index) = sqrt(mean(horizontal_error .^ 2));
    maximum_horizontal_error_m(case_index) = max(horizontal_error);
    horizontal_squared_error_sum(case_index) = sum(horizontal_error .^ 2);

    range_mask = range_data(:, 1) >= evaluation_start_s(case_index) ...
        & range_data(:, 1) <= evaluation_end_s(case_index);
    range_eval = range_data(range_mask, :);
    range_observation_count(case_index) = size(range_eval, 1);
    valid_range_update_count(case_index) = sum( ...
        all(isfinite(range_eval(:, 1:6)), 2) & range_eval(:, 2) > 0);

    navigation_anomaly_count(case_index) = count_navigation_anomalies( ...
        nav_eval, nav_dt_s);

    reference_enu = geodetic_to_local_enu_m(reference_position, ...
        reference_position(1, :));
    nav_enu = geodetic_to_local_enu_m(nav_eval(:, 3:5), ...
        reference_position(1, :));
    plot_index = unique(round(linspace(1, size(nav_eval, 1), ...
        min(maximum_plot_points, size(nav_eval, 1)))));
    plot_data(case_index).time_h = ...
        (nav_eval(plot_index, 2) - nav_eval(1, 2)) / 3600;
    plot_data(case_index).truth_east_m = reference_enu(plot_index, 1);
    plot_data(case_index).truth_north_m = reference_enu(plot_index, 2);
    plot_data(case_index).nav_east_m = nav_enu(plot_index, 1);
    plot_data(case_index).nav_north_m = nav_enu(plot_index, 2);
    plot_data(case_index).east_error_m = east_error(plot_index);
    plot_data(case_index).north_error_m = north_error(plot_index);
    plot_data(case_index).horizontal_error_m = horizontal_error(plot_index);

    clear imu range_data truth nav nav_eval reference_position
end

combined_horizontal_rmse_m = sqrt( ...
    sum(horizontal_squared_error_sum) / sum(valid_epoch_count));

table1 = table(case_labels', effective_duration_s, imu_frequency_hz, ...
    range_period_s, range_observation_count, valid_range_update_count, ...
    navigation_frequency_hz, navigation_anomaly_count, ...
    'VariableNames', {'Test', 'EffectiveNavigationDuration_s', ...
    'IMUInputFrequency_Hz', 'SimulatedAcousticRangePeriod_s', ...
    'RangeObservationCount', 'ValidRangeUpdateCount', ...
    'NavigationOutputFrequency_Hz', 'NavigationOutputAnomalyCount'});
table2 = table(case_labels', evaluation_start_s, evaluation_end_s, ...
    valid_epoch_count, east_rmse_m, north_rmse_m, horizontal_rmse_m, ...
    maximum_horizontal_error_m, ...
    'VariableNames', {'Test', 'EvaluationStart_s', 'EvaluationEnd_s', ...
    'ValidEpochCount', 'EastRMSE_m', 'NorthRMSE_m', ...
    'HorizontalRMSE_m', 'MaximumHorizontalError_m'});

table1_csv = fullfile(output_dir, '表1_组合导航实时化试验完成情况.csv');
table2_csv = fullfile(output_dir, '表2_组合导航水平定位精度测试结果.csv');
table1_xlsx = fullfile(output_dir, '表1_组合导航实时化试验完成情况.xlsx');
table2_xlsx = fullfile(output_dir, '表2_组合导航水平定位精度测试结果.xlsx');
writetable(table1, table1_csv);
writetable(table2, table2_csv);
writetable(table1, table1_xlsx);
writetable(table2, table2_xlsx);

visibility = 'off';
if parser.Results.Visible, visibility = 'on'; end
figure1_path = fullfile(output_dir, '图1_组合导航轨迹与GNSS参考轨迹对比.png');
figure2_path = fullfile(output_dir, '图2_东向北向及水平位置误差曲线.png');
draw_trajectory_figure(plot_data, case_labels, visibility, figure1_path);
draw_error_figure(plot_data, case_labels, visibility, figure2_path);

report_path = fullfile(output_dir, '组合导航半物理实时化试验结果.md');
write_completed_report(report_path, table1, table2, ...
    combined_horizontal_rmse_m, figure1_path, figure2_path);

summary_path = fullfile(output_dir, 'case07-09-test-summary.mat');
save(summary_path, 'table1', 'table2', 'combined_horizontal_rmse_m', ...
    'case_ids', 'case_labels');

artifacts = struct('output_dir', output_dir, 'table1', table1, ...
    'table2', table2, 'combined_horizontal_rmse_m', ...
    combined_horizontal_rmse_m, 'report_path', report_path, ...
    'figure1_path', figure1_path, 'figure2_path', figure2_path, ...
    'summary_path', summary_path);

fprintf('case-07/08/09试验报告已生成：%s\n', report_path);
fprintf('综合水平位置RMSE：%.3f m\n', combined_horizontal_rmse_m);
disp(table1);
disp(table2);
end

function validate_inputs(imu, range_data, truth, nav, case_id)
if isempty(imu) || size(imu, 2) < 7 || any(~isfinite(imu(:, 1)))
    error('%s IMU数据格式无效。', case_id);
end
if isempty(range_data) || size(range_data, 2) < 6
    error('%s 距离数据格式无效。', case_id);
end
if isempty(truth) || size(truth, 2) < 11
    error('%s GNSS参考轨迹格式无效。', case_id);
end
if isempty(nav) || size(nav, 2) < 11
    error('%s 组合导航结果格式无效。', case_id);
end
if any(diff(imu(:, 1)) <= 0) || any(diff(range_data(:, 1)) <= 0) ...
        || any(diff(truth(:, 2)) <= 0) || any(diff(nav(:, 2)) <= 0)
    error('%s 存在非严格递增的时间戳。', case_id);
end
end

function [east_error_m, north_error_m] = geodetic_position_error_m( ...
        navigation_position, reference_position)
% 输入列均为[纬度deg, 经度deg, 高程m]。
a = 6378137.0;
eccentricity_squared = 6.69437999014e-3;
latitude = deg2rad(reference_position(:, 1));
sin_latitude = sin(latitude);
prime_vertical = a ./ sqrt(1 - eccentricity_squared * sin_latitude .^ 2);
meridian = a * (1 - eccentricity_squared) ./ ...
    (1 - eccentricity_squared * sin_latitude .^ 2) .^ 1.5;
height = reference_position(:, 3);
north_error_m = deg2rad(navigation_position(:, 1) ...
    - reference_position(:, 1)) .* (meridian + height);
east_error_m = deg2rad(navigation_position(:, 2) ...
    - reference_position(:, 2)) .* (prime_vertical + height) ...
    .* cos(latitude);
end

function enu = geodetic_to_local_enu_m(position, origin)
% 小范围轨迹采用逐点WGS-84曲率线性化到局部ENU，仅用于绘图。
[east, north] = geodetic_position_error_m(position, ...
    repmat(origin, size(position, 1), 1));
enu = [east, north, position(:, 3) - origin(3)];
end

function anomaly_count = count_navigation_anomalies(nav, nominal_dt_s)
nonfinite_row_count = sum(any(~isfinite(nav), 2));
time_difference = diff(nav(:, 2));
nonincreasing_count = sum(time_difference <= 0);
gap_count = sum(time_difference > max(1.5 * nominal_dt_s, ...
    nominal_dt_s + 1e-6));
anomaly_count = nonfinite_row_count + nonincreasing_count + gap_count;
end

function draw_trajectory_figure(plot_data, case_labels, visibility, path)
% fig = figure('Visible', visibility, 'Color', 'w', ...
%     'Position', [80, 80, 1500, 470]);
fig = myfigurestartup(15,5,'prese');
layout = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', ...
    'Padding', 'compact');
for index = 1:numel(plot_data)
    ax = nexttile(layout, index);
    plot(ax, plot_data(index).truth_east_m, ...
        plot_data(index).truth_north_m, 'k-', 'LineWidth', 1.5, ...
        'DisplayName', 'GNSS参考轨迹');
    hold(ax, 'on');
    plot(ax, plot_data(index).nav_east_m, ...
        plot_data(index).nav_north_m, '-', 'Color', [0.00, 0.45, 0.74], ...
        'LineWidth', 1.1, 'DisplayName', '组合导航轨迹');
    grid(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, '东向位置（m）');
    ylabel(ax, '北向位置（m）');
    title(ax, sprintf('(%c) %s', char('a' + index - 1), case_labels(index)));
    legend(ax, 'Location', 'best');
    % set(ax, 'FontName', 'Microsoft YaHei', 'FontSize', 10);
end
% title(layout, '3次试验组合导航轨迹与GNSS参考轨迹对比', ...
%     'FontName', 'Microsoft YaHei', 'FontWeight', 'bold');
title(layout, '3次试验组合导航轨迹与GNSS参考轨迹对比', ...
    'FontWeight', 'bold','FontSize', 16);
exportgraphics(fig, path, 'Resolution', 600);
savefig(fig, replace(path, '.png', '.fig'));
close(fig);
end

function draw_error_figure(plot_data, case_labels, visibility, path)
% fig = figure('Visible', visibility, 'Color', 'w', ...
%     'Position', [80, 60, 1400, 950]);
fig = myfigurestartup(14,9,'prese');
layout = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact', ...
    'Padding', 'compact');
colors = [0.00, 0.45, 0.74; 0.85, 0.33, 0.10; 0.20, 0.20, 0.20];
for index = 1:numel(plot_data)
    ax = nexttile(layout, index);
    plot(ax, plot_data(index).time_h, plot_data(index).east_error_m, ...
        'Color', colors(1, :), 'LineWidth', 0.9, 'DisplayName', '东向误差');
    hold(ax, 'on');
    plot(ax, plot_data(index).time_h, plot_data(index).north_error_m, ...
        'Color', colors(2, :), 'LineWidth', 0.9, 'DisplayName', '北向误差');
    plot(ax, plot_data(index).time_h, ...
        plot_data(index).horizontal_error_m, 'Color', colors(3, :), ...
        'LineWidth', 1.1, 'DisplayName', '水平误差');
    yline(ax, 0, ':', 'Color', [0.5, 0.5, 0.5], ...
        'HandleVisibility', 'off');
    grid(ax, 'on');
    xlabel(ax, '导航时间（h）');
    ylabel(ax, '位置误差（m）');
    title(ax, sprintf('(%c) %s', char('a' + index - 1), case_labels(index)));
    legend(ax, 'Location', 'best', 'NumColumns', 3);
    % set(ax, 'FontName', 'Microsoft YaHei', 'FontSize', 10);
end
title(layout, '3次试验东向、北向及水平位置误差', ...
'FontWeight', 'bold', 'FontSize', 16);
% title(layout, '3次试验东向、北向及水平位置误差', ...
%     'FontName', 'Microsoft YaHei', 'FontWeight', 'bold');
exportgraphics(fig, path, 'Resolution', 600);
savefig(fig, replace(path, '.png', '.fig'));
close(fig);
end

function write_completed_report(path, table1, table2, combined_rmse, ...
        figure1_path, figure2_path)
all_updates_valid = all(table1.RangeObservationCount ...
    == table1.ValidRangeUpdateCount);
no_output_anomalies = all(table1.NavigationOutputAnomalyCount == 0);
if all_updates_valid && no_output_anomalies
    operation_text = ['均连续完成；评价时段内模拟声学距离记录全部满足' ...
        '格式、时序和正距离检查并进入组合导航解算，导航输出未检出' ...
        '非有限值、时间倒序或异常时间间隔'];
else
    operation_text = ['均完成；各航次有效距离更新次数和导航输出异常次数' ...
        '见表1'];
end

fid = fopen(path, 'wt', 'n', 'UTF-8');
if fid < 0, error('无法写入报告：%s', path); end
cleanup = onCleanup(@() fclose(fid));

fprintf(fid, '## a) 试验完成情况\n\n');
fprintf(fid, ['本次采用车载实测数据驱动的半物理实时化试验方式，共开展' ...
    '3次组合导航测试。试验过程中，将车载实测惯性数据及基于GNSS' ...
    '参考轨迹生成的模拟声学距离数据，按照规定的数据格式和时序输入' ...
    '工程样机，由被试设备完成惯性导航递推、模拟声学距离量测更新及' ...
    '组合导航解算，并记录导航输出及相关运行状态。\n\n']);
fprintf(fid, '各次试验的有效导航时长、模拟声学距离更新情况及导航输出情况见表1。\n\n');
fprintf(fid, '表1 组合导航实时化试验完成情况\n\n');
fprintf(fid, '| 测试项目 | 第1次 | 第2次 | 第3次 |\n');
fprintf(fid, '| --- | ---: | ---: | ---: |\n');
write_markdown_row(fid, '有效导航时长（s）', ...
    table1.EffectiveNavigationDuration_s, '%.2f');
write_markdown_row(fid, '惯性数据输入频率（Hz）', ...
    table1.IMUInputFrequency_Hz, '%.2f');
write_markdown_row(fid, '模拟声学测距周期（s）', ...
    table1.SimulatedAcousticRangePeriod_s, '%.2f');
write_markdown_row(fid, '模拟距离观测次数', ...
    table1.RangeObservationCount, '%.0f');
write_markdown_row(fid, '有效距离更新次数', ...
    table1.ValidRangeUpdateCount, '%.0f');
write_markdown_row(fid, '导航输出频率（Hz）', ...
    table1.NavigationOutputFrequency_Hz, '%.2f');
write_markdown_row(fid, '导航输出异常次数', ...
    table1.NavigationOutputAnomalyCount, '%.0f');
fprintf(fid, '\n根据试验记录，3次测试的导航数据输入、模拟声学距离接收及导航结果输出%s。', ...
    operation_text);
fprintf(fid, ['试验期间记录的设备运行状态及相关数据，可用于后续导航精度' ...
    '统计与分析。\n\n']);

fprintf(fid, '## b) 导航轨迹测试结果\n\n');
fprintf(fid, ['以车载GNSS参考轨迹作为位置评价基准，将3次试验中被试设备' ...
    '输出的组合导航轨迹与对应GNSS参考轨迹进行对比，结果如图1所示。\n\n']);
fprintf(fid, '![图1 3次试验的组合导航轨迹与GNSS参考轨迹对比图](%s)\n\n', ...
    local_file_name(figure1_path));
fprintf(fid, ['其中，图1(a)、图1(b)和图1(c)分别对应第1次、第2次和' ...
    '第3次导航试验。\n\n']);
fprintf(fid, ['根据轨迹对比结果，3次试验中组合导航轨迹均与GNSS参考轨迹' ...
    '保持时间对应和整体走向一致，但在有效导航时段内存在不同程度的' ...
    '水平偏离；各航次偏离程度由图2及表2中的误差统计量定量表征。' ...
    '该结果反映了工程样机在420 s稀疏模拟声学距离辅助条件下的二次' ...
    'RTS导航轨迹输出情况。\n\n']);

fprintf(fid, '## c) 水平定位精度测试结果\n\n');
fprintf(fid, ['根据3次试验中被试设备输出的组合导航位置及GNSS参考位置，' ...
    '采用统一的时间匹配和WGS-84局部曲率转换方法，分别计算各试验的' ...
    '东向、北向及水平位置误差。\n\n']);
fprintf(fid, '3次试验的位置误差随导航时间变化情况如图2所示。\n\n');
fprintf(fid, '![图2 3次试验的东向、北向及水平位置误差曲线](%s)\n\n', ...
    local_file_name(figure2_path));
fprintf(fid, ['其中，图2(a)、图2(b)和图2(c)分别对应第1次、第2次和' ...
    '第3次导航试验。\n\n']);
fprintf(fid, ['对各次试验严格二次RTS有效时段内的位置误差进行统计，每段' ...
    '结果均截止于对应测距节点，得到东向、北向及水平位置RMSE，' ...
    '结果见表2。\n\n']);
fprintf(fid, '表2 组合导航水平定位精度测试结果\n\n');
fprintf(fid, '| 精度评价指标 | 第1次 | 第2次 | 第3次 |\n');
fprintf(fid, '| --- | ---: | ---: | ---: |\n');
write_markdown_row(fid, '东向位置RMSE（m）', table2.EastRMSE_m, '%.2f');
write_markdown_row(fid, '北向位置RMSE（m）', table2.NorthRMSE_m, '%.2f');
write_markdown_row(fid, '水平位置RMSE（m）', table2.HorizontalRMSE_m, '%.2f');
write_markdown_row(fid, '最大水平位置误差（m）', ...
    table2.MaximumHorizontalError_m, '%.2f');
fprintf(fid, ['\n在3次试验采用一致的数据处理方法及各自严格二次RTS有效时段的' ...
    '条件下，将全部有效历元合并，并按均方误差定义计算得到综合水平' ...
    '位置RMSE为%.2f m。\n\n'], combined_rmse);
fprintf(fid, ['其中，第1次、第2次和第3次试验的水平位置RMSE分别为' ...
    '%.2f m、%.2f m和%.2f m。3次试验的水平定位误差变化情况及' ...
    '精度差异详见图2和表2。\n\n'], table2.HorizontalRMSE_m);

fprintf(fid, '## d) 综合测试结果\n\n');
fprintf(fid, ['本次完成3次基于车载实测数据的半物理实时化组合导航试验，' ...
    '获取了工程样机在规定模拟声学距离辅助条件下的导航输出结果及' ...
    '水平定位精度数据。\n\n']);
fprintf(fid, ['根据测试记录和精度统计结果，工程样机的组合导航解算、模拟' ...
    '声学距离量测更新及导航信息输出%s，3次试验的综合水平位置RMSE' ...
    '为%.2f m。\n\n'], operation_text, combined_rmse);
fprintf(fid, ['本次测试采用基于GNSS参考轨迹生成的模拟声学距离观测，测试' ...
    '结果反映了工程样机在规定半物理试验条件下的组合导航定位性能。' ...
    '是否满足本阶段水平定位精度要求，按照产品技术条件及本阶段适用' ...
    '的测试评价准则进行判定。\n\n']);
fprintf(fid, ['> 统计口径：组合导航结果采用严格二次RTS输出，每段结果截止于' ...
    '对应测距节点，末尾仅完成一次RTS的区间不纳入评价；综合RMSE按' ...
    '3个航次全部有效历元合并计算。有效' ...
    '距离更新次数表示进入解算时序且通过格式、有限值和正距离检查的' ...
    '距离记录数。\n']);
end

function nav = select_strict_second_rts_rows( ...
        nav, marker_path, range_data, case_id)
% 新结果由严格口径标记文件确认；旧结果的最大段是循环结束时追加的
% previous_single_nav，只完成一次RTS，因此剔除该段。
if isempty(nav) || size(nav, 2) < 2
    error('%s 二次RTS结果为空或列数不足。', case_id);
end
if isfile(marker_path)
    marker = load(marker_path, 'strict_second_rts_info');
    if ~isfield(marker, 'strict_second_rts_info') || ...
            ~marker.strict_second_rts_info.strict_second_rts_only
        error('%s 严格二次RTS标记文件无效：%s', case_id, marker_path);
    end
else
    last_segment_id = max(round(nav(:, 1)));
    nav = nav(round(nav(:, 1)) < last_segment_id, :);
    if isempty(nav)
        error('%s 至少需要两个测距区间才能形成严格二次RTS结果。', case_id);
    end
    warning(['%s使用旧版二次RTS结果，已剔除末尾仅完成一次RTS的第%d段；' ...
        '建议重新运行run_four_method_dataset_comparison生成严格标记。'], ...
        case_id, last_segment_id);
end

node_ids = unique(round(nav(:, 1)), 'stable');
time_tolerance_s = max(1e-6, 0.25 * median(diff(nav(:, 2))));
for node_index = 1:numel(node_ids)
    node_id = node_ids(node_index);
    if node_id < 1 || node_id > size(range_data, 1)
        error('%s 二次RTS节点编号%d超出距离数据范围。', case_id, node_id);
    end
    segment_rows = find(round(nav(:, 1)) == node_id);
    segment_end_s = nav(segment_rows(end), 2);
    range_node_s = range_data(node_id, 1);
    if abs(segment_end_s - range_node_s) > time_tolerance_s
        error(['%s二次RTS第%d段末时刻%.6f s未落在测距节点' ...
            '%.6f s。'], case_id, node_id, segment_end_s, range_node_s);
    end
end
end

function write_markdown_row(fid, label, values, format_specifier)
fprintf(fid, '| %s | ', label);
for index = 1:numel(values)
    fprintf(fid, format_specifier, values(index));
    if index < numel(values), fprintf(fid, ' | '); end
end
fprintf(fid, ' |\n');
end

function name = local_file_name(path)
[~, base, extension] = fileparts(path);
name = [base, extension];
end
