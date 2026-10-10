% clear;
close all;
clc;
%% 双水听器距离+方位角直接定位（不进入 EKF/RTS）
% 已知信标位置、水平距离和双水听器相对方位角时，还需要航行器航向角
% 才能把艇体系方位角转换到东北地理坐标系。本脚本默认采用“不掉帧、
% 仅测距前向导航”的航向角进行实际可用的直接定位，同时计算真实航向
% 下的定位结果，后者只作为量测几何精度上限，不参与实际定位。
%
% 输入来自 run_double_hydrophone_forward_ekf(.m/_adaptive.m) 已生成的：
%   1) range-phase-azimuth-diagnostic*.txt（先验角和选择角）；
%   2) range-phase-azimuth-measurement-accuracy*.txt（测距和真实方位）；
%   3) simple-ekf-rts-rad/simple-forward-ekf-rad.nav（仅测距航向）。
%
% 输出的每一行对应一个通信历元；当前 420 s 间隔、4621 s 时长时应有
% 11 个直接定位点。

%% 用户配置
if ~exist('data_source', 'var')
    data_source = "experiment";   % "simulation" 或 "experiment"
end
if ~exist('case_name', 'var')
    case_name = 'case-06';
end
if ~exist('phasestd', 'var')
    phasestd = 5;                 % 5 / 10 / 15 deg
end
if ~exist('selection_mode', 'var')
    selection_mode = "fixed";    % "fixed" 或 "adaptive"
end
if ~exist('heading_source', 'var')
    heading_source = "range-only"; % "range-only" 或 "truth"
end
if ~exist('baseline_install_deg', 'var')
    baseline_install_deg = 0;
end
if ~exist('show_figure', 'var')
    show_figure = true;
end

%% 路径和输入配置
case_name = char(case_name);
data_source = lower(data_source);
selection_mode = lower(selection_mode);
heading_source = lower(heading_source);
if ~ismember(selection_mode, ["fixed", "adaptive"])
    error('selection_mode 只能设置为 "fixed" 或 "adaptive"。');
end
if ~ismember(heading_source, ["range-only", "truth"])
    error('heading_source 只能设置为 "range-only" 或 "truth"。');
end

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));
addpath(topic_dir);
addpath(script_dir);
addpath(fullfile(script_dir, 'function'));
paths = setup_inertial_experiment();
param = Param();

case_id = sscanf(case_name, 'case-%d', 1);
if isempty(case_id) || ~isscalar(case_id) || case_id < 0 || ...
        case_id ~= fix(case_id)
    error('case_name 必须采用 case-XX 格式，例如 case-00 或 case-06。');
end

if data_source == "simulation"
    input_dir = paths.simulation_input(case_id);
    navigation_dir = paths.simulation_navigation(case_id);
    cfg = load_algorithm_exploration_config('simulation', 'rad', input_dir);
elseif data_source == "experiment"
    input_dir = paths.experiment_input(case_id);
    navigation_dir = paths.experiment_navigation(case_id);
    cfg = load_algorithm_exploration_config('experiment', 'rad', input_dir);
    cfg.rangefile1path = fullfile(input_dir, 'range1.txt');
    cfg.rangefile2path = fullfile(input_dir, 'range2.txt');
    cfg.rangefile3path = fullfile(input_dir, 'range3.txt');
    cfg.truthpath = fullfile(input_dir, 'truth.nav');
else
    error('data_source 只能设置为 "simulation" 或 "experiment"。');
end

result_dir = fullfile(navigation_dir, 'navigation-results', ...
    sprintf('double-hydrophone-phasestd%ddeg', phasestd));
if selection_mode == "adaptive"
    suffix = '-adaptive';
else
    suffix = '';
end
diagnostic_path = fullfile(result_dir, ...
    ['range-phase-azimuth-diagnostic', suffix, '.txt']);
accuracy_path = fullfile(result_dir, ...
    ['range-phase-azimuth-measurement-accuracy', suffix, '.txt']);
range_only_nav_path = fullfile(navigation_dir, 'navigation-results', ...
    'simple-ekf-rts-rad', 'simple-forward-ekf-rad.nav');

required_paths = {diagnostic_path, accuracy_path, cfg.truthpath, ...
    cfg.rangefile1path, cfg.rangefile2path, cfg.rangefile3path};
missing_mask = ~cellfun(@isfile, required_paths);
if any(missing_mask)
    error('缺少直接定位输入文件：\n%s', ...
        strjoin(required_paths(missing_mask), '\n'));
end
if heading_source == "range-only" && ~isfile(range_only_nav_path)
    error(['heading_source="range-only"，但找不到仅测距前向结果：\n%s\n' ...
        '请先生成该结果，或将 heading_source 改为 "truth"。'], ...
        range_only_nav_path);
end

%% 读取 11 个量测事件
diagnostic_all = readmatrix(diagnostic_path, 'FileType', 'text');
diagnostic_valid = size(diagnostic_all, 2) >= 4 && ...
    ~isempty(diagnostic_all);
if ~diagnostic_valid
    error('方位角诊断文件格式无效：%s', diagnostic_path);
end
diagnostic_data = diagnostic_all(all(isfinite( ...
    diagnostic_all(:, 1:4)), 2), :);

accuracy_all = readmatrix(accuracy_path, 'FileType', 'text');
if size(accuracy_all, 2) < 11
    error('量测精度文件应至少包含 11 列：%s', accuracy_path);
end
accuracy_data = accuracy_all(all(isfinite( ...
    accuracy_all(:, [1, 2, 3, 4, 8, 9])), 2), :);
if isempty(accuracy_data)
    error('量测精度文件中没有有效事件：%s', accuracy_path);
end

event_time_s = accuracy_data(:, 1);
beacon_id = round(accuracy_data(:, 2));
truth_relative_azimuth_deg = accuracy_data(:, 3);
selected_relative_azimuth_deg = accuracy_data(:, 4);
measured_horizontal_range_m = accuracy_data(:, 8);
truth_horizontal_range_m = accuracy_data(:, 9);
event_count = numel(event_time_s);

diagnostic_index = zeros(event_count, 1);
for event_index = 1:event_count
    [time_error_s, nearest_index] = min(abs( ...
        diagnostic_data(:, 1) - event_time_s(event_index)));
    if time_error_s > 1e-3 || ...
            round(diagnostic_data(nearest_index, 2)) ~= beacon_id(event_index)
        error('第 %d 个事件无法与方位角诊断数据匹配。', event_index);
    end
    diagnostic_index(event_index) = nearest_index;
end
prior_relative_azimuth_deg = diagnostic_data(diagnostic_index, 3);
diagnostic_selected_deg = diagnostic_data(diagnostic_index, 4);
selected_consistency_deg = atan2d(sind(diagnostic_selected_deg - ...
    selected_relative_azimuth_deg), cosd(diagnostic_selected_deg - ...
    selected_relative_azimuth_deg));
if any(abs(selected_consistency_deg) > 1e-5)
    error('诊断文件与量测精度文件中的选择角不一致。');
end

%% 获取每个事件对应的信标位置
range_paths = {cfg.rangefile1path, cfg.rangefile2path, ...
    cfg.rangefile3path};
range_sources = cellfun(@(path) readmatrix(path, 'FileType', 'text'), ...
    range_paths, 'UniformOutput', false);
beacon_position_rad_m = nan(event_count, 3);
for event_index = 1:event_count
    source = range_sources{beacon_id(event_index)};
    if size(source, 2) < 6
        error('range%d.txt 至少应包含 6 列。', beacon_id(event_index));
    end
    [time_error_s, source_index] = min(abs( ...
        source(:, 1) - event_time_s(event_index)));
    source_dt_s = median(diff(source(:, 1)));
    if time_error_s > max(0.51 * source_dt_s, 1e-3)
        error('第 %d 个事件在对应测距文件中没有同步信标位置。', ...
            event_index);
    end
    beacon_position_rad_m(event_index, :) = source(source_index, 4:6);
end

%% 读取真实位置、真实航向以及用于实际定位的航向
truth_all = readmatrix(cfg.truthpath, 'FileType', 'text');
truth_valid = size(truth_all, 2) >= 11 & all(isfinite( ...
    truth_all(:, [2, 3, 4, 5, 11])), 2);
truth_reference = truth_all(truth_valid, :);
[truth_time_s, truth_unique_index] = unique(truth_reference(:, 2), 'stable');
truth_reference = truth_reference(truth_unique_index, :);
truth_lat_deg = interp1(truth_time_s, truth_reference(:, 3), ...
    event_time_s, 'linear');
truth_lon_deg = interp1(truth_time_s, truth_reference(:, 4), ...
    event_time_s, 'linear');
truth_height_m = interp1(truth_time_s, truth_reference(:, 5), ...
    event_time_s, 'linear');
truth_heading_deg = rad2deg(interp1(truth_time_s, unwrap(deg2rad( ...
    truth_reference(:, 11))), event_time_s, 'linear'));

if heading_source == "range-only"
    range_only_nav = readmatrix(range_only_nav_path, 'FileType', 'text');
    nav_valid = size(range_only_nav, 2) >= 11 & all(isfinite( ...
        range_only_nav(:, [2, 3, 4, 11])), 2);
    range_only_nav = range_only_nav(nav_valid, :);
    [nav_time_s, nav_unique_index] = unique( ...
        range_only_nav(:, 2), 'stable');
    range_only_nav = range_only_nav(nav_unique_index, :);
    positioning_heading_deg = rad2deg(interp1(nav_time_s, ...
        unwrap(deg2rad(range_only_nav(:, 11))), event_time_s, 'linear'));
else
    positioning_heading_deg = truth_heading_deg;
end

if any(~isfinite(positioning_heading_deg))
    error('所选航向数据没有覆盖全部量测时刻。');
end

%% 距离+方位角直接反算位置
absolute_bearing_deg = mod(selected_relative_azimuth_deg + ...
    positioning_heading_deg + baseline_install_deg + 90, 360);
truth_heading_bearing_deg = mod(selected_relative_azimuth_deg + ...
    truth_heading_deg + baseline_install_deg + 90, 360);

direct_position_rad_m = nan(event_count, 3);
truth_heading_position_rad_m = nan(event_count, 3);
for event_index = 1:event_count
    beacon = beacon_position_rad_m(event_index, :)';
    [rm, rn] = getRmRn(beacon(1), param);
    north_scale = rm + beacon(3);
    east_scale = (rn + beacon(3)) * cos(beacon(1));

    delta_north_m = measured_horizontal_range_m(event_index) * ...
        cosd(absolute_bearing_deg(event_index));
    delta_east_m = measured_horizontal_range_m(event_index) * ...
        sind(absolute_bearing_deg(event_index));
    direct_position_rad_m(event_index, :) = [ ...
        beacon(1) - delta_north_m / north_scale, ...
        beacon(2) - delta_east_m / east_scale, truth_height_m(event_index)];

    truth_delta_north_m = measured_horizontal_range_m(event_index) * ...
        cosd(truth_heading_bearing_deg(event_index));
    truth_delta_east_m = measured_horizontal_range_m(event_index) * ...
        sind(truth_heading_bearing_deg(event_index));
    truth_heading_position_rad_m(event_index, :) = [ ...
        beacon(1) - truth_delta_north_m / north_scale, ...
        beacon(2) - truth_delta_east_m / east_scale, truth_height_m(event_index)];
end

direct_lat_deg = rad2deg(direct_position_rad_m(:, 1));
direct_lon_deg = rad2deg(direct_position_rad_m(:, 2));
truth_heading_lat_deg = rad2deg(truth_heading_position_rad_m(:, 1));
truth_heading_lon_deg = rad2deg(truth_heading_position_rad_m(:, 2));

direct_error_ne_m = nan(event_count, 2);
truth_heading_error_ne_m = nan(event_count, 2);
for event_index = 1:event_count
    truth_lat_rad = deg2rad(truth_lat_deg(event_index));
    [rm, rn] = getRmRn(truth_lat_rad, param);
    north_scale = rm + truth_height_m(event_index);
    east_scale = (rn + truth_height_m(event_index)) * cos(truth_lat_rad);
    direct_error_ne_m(event_index, :) = [ ...
        (direct_position_rad_m(event_index, 1) - truth_lat_rad) * ...
            north_scale, ...
        (direct_position_rad_m(event_index, 2) - ...
            deg2rad(truth_lon_deg(event_index))) * east_scale];
    truth_heading_error_ne_m(event_index, :) = [ ...
        (truth_heading_position_rad_m(event_index, 1) - truth_lat_rad) * ...
            north_scale, ...
        (truth_heading_position_rad_m(event_index, 2) - ...
            deg2rad(truth_lon_deg(event_index))) * east_scale];
end
direct_horizontal_error_m = hypot(direct_error_ne_m(:, 1), ...
    direct_error_ne_m(:, 2));
truth_heading_horizontal_error_m = hypot( ...
    truth_heading_error_ne_m(:, 1), truth_heading_error_ne_m(:, 2));
heading_error_deg = atan2d(sind(positioning_heading_deg - ...
    truth_heading_deg), cosd(positioning_heading_deg - truth_heading_deg));
azimuth_error_deg = atan2d(sind(selected_relative_azimuth_deg - ...
    truth_relative_azimuth_deg), cosd(selected_relative_azimuth_deg - ...
    truth_relative_azimuth_deg));

%% 保存逐帧定位结果
output_path = fullfile(result_dir, ...
    ['range-phase-azimuth-direct-positioning', suffix, '.txt']);
output_fp = fopen(output_path, 'wt');
if output_fp < 0
    error('无法创建直接定位结果：%s', output_path);
end
cleaner = onCleanup(@() fclose(output_fp));
fprintf(output_fp, ['# heading_source=%s; truth-heading position is for ' ...
    'accuracy-bound analysis only.\n'], char(heading_source));
fprintf(output_fp, ['frame time_s beacon_id measured_range_m truth_range_m ' ...
    'prior_relative_deg selected_relative_deg truth_relative_deg ' ...
    'azimuth_error_deg positioning_heading_deg truth_heading_deg ' ...
    'heading_error_deg absolute_bearing_deg direct_lat_deg direct_lon_deg ' ...
    'truth_lat_deg truth_lon_deg north_error_m east_error_m horizontal_error_m ' ...
    'truth_heading_lat_deg truth_heading_lon_deg ' ...
    'truth_heading_horizontal_error_m\n']);
output_matrix = [(1:event_count)', event_time_s, beacon_id, ...
    measured_horizontal_range_m, truth_horizontal_range_m, ...
    prior_relative_azimuth_deg, selected_relative_azimuth_deg, ...
    truth_relative_azimuth_deg, azimuth_error_deg, ...
    positioning_heading_deg, truth_heading_deg, heading_error_deg, ...
    absolute_bearing_deg, direct_lat_deg, direct_lon_deg, ...
    truth_lat_deg, truth_lon_deg, direct_error_ne_m, ...
    direct_horizontal_error_m, truth_heading_lat_deg, ...
    truth_heading_lon_deg, truth_heading_horizontal_error_m];
fprintf(output_fp, ['%2d %12.6f %2d %14.6f %14.6f %12.6f %12.6f ' ...
    '%12.6f %12.6f %12.6f %12.6f %12.6f %12.6f %14.9f %14.9f ' ...
    '%14.9f %14.9f %12.4f %12.4f %12.4f %14.9f %14.9f %12.4f\n'], ...
    output_matrix');
clear cleaner;

summary_table = table((1:event_count)', event_time_s, beacon_id, ...
    measured_horizontal_range_m, selected_relative_azimuth_deg, ...
    positioning_heading_deg, heading_error_deg, direct_lat_deg, ...
    direct_lon_deg, direct_horizontal_error_m, ...
    truth_heading_horizontal_error_m, 'VariableNames', { ...
    'Frame', 'Time_s', 'Beacon', 'Range_m', 'SelectedAzimuth_deg', ...
    'Heading_deg', 'HeadingError_deg', 'Latitude_deg', 'Longitude_deg', ...
    'HorizontalError_m', 'TruthHeadingError_m'});
disp(summary_table);
fprintf('\n直接定位点数：%d\n', event_count);
fprintf('采用 %s 航向的水平 RMSE：%.3f m；平均误差：%.3f m。\n', ...
    char(heading_source), sqrt(mean(direct_horizontal_error_m .^ 2)), ...
    mean(direct_horizontal_error_m));
fprintf('真实航向下的量测定位水平 RMSE：%.3f m；平均误差：%.3f m。\n', ...
    sqrt(mean(truth_heading_horizontal_error_m .^ 2)), ...
    mean(truth_heading_horizontal_error_m));

%% 可视化 11 个直接定位点
reference_lat_rad = deg2rad(truth_lat_deg(1));
reference_lon_rad = deg2rad(truth_lon_deg(1));
[reference_rm, reference_rn] = getRmRn(reference_lat_rad, param);
reference_height_m = truth_height_m(1);
north_scale = reference_rm + reference_height_m;
east_scale = (reference_rn + reference_height_m) * cos(reference_lat_rad);
truth_north_m = (deg2rad(truth_lat_deg) - reference_lat_rad) * north_scale;
truth_east_m = (deg2rad(truth_lon_deg) - reference_lon_rad) * east_scale;
direct_north_m = (direct_position_rad_m(:, 1) - reference_lat_rad) * ...
    north_scale;
direct_east_m = (direct_position_rad_m(:, 2) - reference_lon_rad) * ...
    east_scale;
truth_heading_north_m = (truth_heading_position_rad_m(:, 1) - ...
    reference_lat_rad) * north_scale;
truth_heading_east_m = (truth_heading_position_rad_m(:, 2) - ...
    reference_lon_rad) * east_scale;
beacon_north_m = (beacon_position_rad_m(:, 1) - reference_lat_rad) * ...
    north_scale;
beacon_east_m = (beacon_position_rad_m(:, 2) - reference_lon_rad) * ...
    east_scale;

figure_visibility = 'off'; %#ok<NASGU>
if show_figure
    figure_visibility = 'on';
end
position_figure = figure('Color', 'w', 'Visible', figure_visibility, ...
    'Position', [120, 100, 1050, 760]);
hold on;
grid on;
box on;
axis equal;
plot(truth_east_m, truth_north_m, 'ko-', 'LineWidth', 1.2, ...
    'MarkerSize', 5, 'DisplayName', '量测时刻真实位置');
plot(direct_east_m, direct_north_m, 'r.-', 'LineWidth', 1.1, ...
    'MarkerSize', 15, 'DisplayName', '先验航向直接定位');
plot(truth_heading_east_m, truth_heading_north_m, 'b.--', ...
    'LineWidth', 1.0, 'MarkerSize', 12, ...
    'DisplayName', '真实航向定位（上限）');
scatter(beacon_east_m, beacon_north_m, 65, beacon_id, 'filled', ...
    'DisplayName', '信标位置');
for event_index = 1:event_count
    text(direct_east_m(event_index), direct_north_m(event_index), ...
        sprintf('  %d', event_index), 'Color', [0.65, 0, 0], ...
        'FontSize', 9);
end
xlabel('East / m');
ylabel('North / m');
title(sprintf('%s / %s / phase std %d deg：距离+方位角直接定位', ...
    char(data_source), case_name, phasestd));
legend('Location', 'best');

figure_path = fullfile(result_dir, ...
    ['range-phase-azimuth-direct-positioning', suffix, '.png']);
matlab_figure_path = fullfile(result_dir, ...
    ['range-phase-azimuth-direct-positioning', suffix, '.fig']);
exportgraphics(position_figure, figure_path, 'Resolution', 300);
savefig(position_figure, matlab_figure_path);

fprintf('逐帧定位结果：%s\n', output_path);
fprintf('定位结果图：%s\n', figure_path);
