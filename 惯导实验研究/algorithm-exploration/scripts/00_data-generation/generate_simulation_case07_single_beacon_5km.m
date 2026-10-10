function output_dir = generate_simulation_case07_single_beacon_5km( ...
        overwrite_existing, dry_run)
%GENERATE_SIMULATION_CASE07_SINGLE_BEACON_5KM 生成约5 km单信标仿真。
%   generate_simulation_case07_single_beacon_5km() 在 case-07 不存在时
%   生成数据；完整数据已经存在时不覆盖。
%   generate_simulation_case07_single_beacon_5km(true) 验证后覆盖旧数据。
%   generate_simulation_case07_single_beacon_5km(false, true) 只检查环境
%   和参数，不写入数据。
%
% 输出文件：IMU_120.txt、truth.txt、range1.txt。
% IMU/真值为100 Hz，单信标理想水平距离为1 Hz。载体短暂静止并加速
% 后沿直线匀速航行；信标位于航迹侧方约5 km，并放在航程中点的
% 侧方，使相对方位角随航行过程连续变化。case-07不生成range2.txt
% 和range3.txt。

%% 1. 用户可调整参数
case_id = 7;
duration_s = 5000;
sample_rate_hz = 100;
range_rate_hz = 1;
static_duration_s = 60;
acceleration_duration_s = 10;
cruise_speed_mps = 2;
target_range_m = 5000;
initial_yaw_deg = 0;
random_seed = 7;
range_validation_tolerance_m = 20;
% case-07采用较低精度IMU，用于放大单信标测距/测向辅助的可观测效果。
imu_gyro_bias_dph = 0.05;
imu_acc_bias_ug = 10;
imu_gyro_arw_deg_sqrt_h = 0.003;
imu_acc_vrw_mps_sqrt_h = 100e-6;

if nargin < 1 || isempty(overwrite_existing)
    overwrite_existing = false;
end
if nargin < 2 || isempty(dry_run)
    dry_run = false;
end
overwrite_existing = validate_logical_scalar( ...
    overwrite_existing, 'overwrite_existing');
dry_run = validate_logical_scalar(dry_run, 'dry_run');

if duration_s <= static_duration_s + acceleration_duration_s
    error('总时长必须大于静止和加速时长之和。');
end
if sample_rate_hz <= 0 || range_rate_hz <= 0 || ...
        mod(sample_rate_hz, range_rate_hz) ~= 0
    error('IMU和测距采样率必须为正数，且两者必须成整数倍。');
end

acceleration_mps2 = cruise_speed_mps/acceleration_duration_s;
uniform_duration_s = duration_s-static_duration_s- ...
    acceleration_duration_s;
sample_interval_s = 1/sample_rate_hz;
range_stride = sample_rate_hz/range_rate_hz;

%% 2. 初始化工程路径和PSINS
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
paths = setup_inertial_experiment();
psins_dir = fullfile(fileparts(paths.project), 'PSINS', 'psins2401');
if exist('glvs', 'file') ~= 2
    if ~isfolder(psins_dir)
        error('找不到PSINS：%s', psins_dir);
    end
    addpath(genpath(psins_dir));
end
required_functions = {'glvs', 'trjsegment', 'trjsimu', 'imuerrset', ...
    'pvaNED2ENU', 'dxyz2pos', 'pos2dxyz', 'imuRFU2FRD', ...
    'avpENU2NED'};
for function_index = 1:numel(required_functions)
    if exist(required_functions{function_index}, 'file') ~= 2
        error('PSINS路径初始化后仍缺少函数：%s', ...
            required_functions{function_index});
    end
end
glvs;

output_dir = paths.simulation_input(case_id);
reference_truth_path = fullfile(paths.experiment_input( ...
    paths.default_experiment_id), 'truth.nav');
if ~isfile(reference_truth_path)
    error('缺少case-07初始位置所需的参考真值：%s', ...
        reference_truth_path);
end

% 航迹沿局部北向直线前进。信标放在完整航程中点的东侧5 km处，
% 因而最近距离约5 km，首末距离约7 km；阵列法线与信标方向的夹角
% 大致由-45 deg连续变化到+45 deg，适合验证双水听器方位角。
acceleration_distance_m = 0.5*acceleration_mps2* ...
    acceleration_duration_s^2;
total_along_track_m = acceleration_distance_m + ...
    cruise_speed_mps*uniform_duration_s;
beacon_enu_m = [target_range_m, 0.5*total_along_track_m, 0];
expected_max_range_m = hypot(target_range_m, 0.5*total_along_track_m);

expected_imu_rows = round(duration_s*sample_rate_hz);
expected_range_rows = floor(expected_imu_rows/range_stride);
estimated_size_mb = 130;
fprintf('case-07输出目录：%s\n', output_dir);
fprintf(['单信标直线场景：航迹侧向距离 %.1f km，时长 %.0f s，', ...
    '巡航速度 %.2f m/s，预计首末距离 %.2f km。\n'], ...
    target_range_m/1000, duration_s, cruise_speed_mps, ...
    expected_max_range_m/1000);
fprintf(['预计生成%d个IMU/真值历元、%d个测距历元，', ...
    '约%d MB文本。\n'], expected_imu_rows, expected_range_rows, ...
    estimated_size_mb);
fprintf('仅生成range1.txt，不生成range2.txt和range3.txt。\n');
if dry_run
    fprintf('dry_run=true：环境和参数检查通过，未写入数据。\n');
    return;
end

%% 3. 覆盖保护
final_paths = build_case_paths(output_dir, '');
final_files = struct2cell(final_paths);
existing_mask = cellfun(@isfile, final_files);
if all(existing_mask) && ~overwrite_existing
    fprintf('case-07已完整存在，未覆盖。若需重建，请传入true。\n');
    return;
end
if any(existing_mask) && ~overwrite_existing
    error(['case-07只存在部分核心文件。为避免混用不同批次数据，', ...
        '请确认后运行generate_simulation_case07_single_beacon_5km(true)。']);
end
if ~isfolder(output_dir)
    mkdir(output_dir);
end

temporary_paths = build_case_paths(output_dir, '.generating');
temporary_files = struct2cell(temporary_paths);
for index = 1:numel(temporary_files)
    delete_if_exists(temporary_files{index});
end
temporary_cleanup = onCleanup(@() delete_file_list(temporary_files));

%% 4. 构造信标侧方5 km的直线轨迹
reference_pva = readmatrix(reference_truth_path, 'FileType', 'text');
reference_avp = pvaNED2ENU(reference_pva);
initial_position = reference_avp(1, 7:9)';
initial_avp = [[0; 0; deg2rad(initial_yaw_deg)]; ...
    [0; 0; 0]; initial_position];

segment = trjsegment([], 'init', 0);
segment = trjsegment(segment, 'uniform', static_duration_s);
segment = trjsegment(segment, 'accelerate', ...
    acceleration_duration_s, [], acceleration_mps2);
segment = trjsegment(segment, 'uniform', uniform_duration_s);
trajectory = trjsimu(initial_avp, segment.wat, sample_interval_s, 1);
if size(trajectory.imu, 1) ~= expected_imu_rows
    error('轨迹历元数异常：应为%d，实际为%d。', ...
        expected_imu_rows, size(trajectory.imu, 1));
end

%% 5. 生成IMU、真值和单信标测距
rng(random_seed, 'twister');
imu_acc_vrw_ug_sqrt_hz = imu_acc_vrw_mps_sqrt_h*1e5/3600;
imu_error = imuerrset(imu_gyro_bias_dph, imu_acc_bias_ug, ...
    imu_gyro_arw_deg_sqrt_h, imu_acc_vrw_ug_sqrt_hz);
noisy_imu = imuadderr(trajectory.imu, imu_error);
imu = imuRFU2FRD(noisy_imu);
truth = avpENU2NED(trajectory.avp);

origin = trajectory.avp(1, 7:9)';
beacon_position = dxyz2pos(beacon_enu_m, origin);
trajectory_position = truth(:, 3:5);
trajectory_position(:, 1:2) = deg2rad(trajectory_position(:, 1:2));
trajectory_xyz = pos2dxyz(trajectory_position, origin);
max_cross_track_m = max(abs(trajectory_xyz(:, 1)));
if max_cross_track_m > 1
    error('case-07轨迹没有保持直线：最大横向偏移为%.3f m。', ...
        max_cross_track_m);
end
range_indices = range_stride:range_stride:size(trajectory_xyz, 1);
horizontal_range_m = hypot( ...
    trajectory_xyz(range_indices, 1)-beacon_enu_m(1), ...
    trajectory_xyz(range_indices, 2)-beacon_enu_m(2));
beacon_rows = repmat(beacon_position, numel(range_indices), 1);
range_output = [truth(range_indices, 2), horizontal_range_m, ...
    horizontal_range_m, beacon_rows];

fprintf('实际水平距离：最小 %.3f m，平均 %.3f m，最大 %.3f m。\n', ...
    min(horizontal_range_m), mean(horizontal_range_m), ...
    max(horizontal_range_m));
if abs(min(horizontal_range_m)-target_range_m) > ...
        range_validation_tolerance_m
    error(['直线轨迹的最近距离不是目标值：目标%.1f m，实际%.3f m，', ...
        '允许偏差%.3f m。'], target_range_m, min(horizontal_range_m), ...
        range_validation_tolerance_m);
end
if abs(max(horizontal_range_m)-expected_max_range_m) > ...
        range_validation_tolerance_m
    error(['直线轨迹的最远距离异常：预计%.3f m，实际%.3f m，', ...
        '允许偏差%.3f m。'], expected_max_range_m, ...
        max(horizontal_range_m), range_validation_tolerance_m);
end

writematrix(imu, temporary_paths.imu, ...
    'FileType', 'text', 'Delimiter', ' ');
writematrix(truth, temporary_paths.truth, ...
    'FileType', 'text', 'Delimiter', ' ');
writematrix(range_output, temporary_paths.range1, ...
    'FileType', 'text', 'Delimiter', ' ');

%% 6. 验证后提交正式文件
validate_case07_files(temporary_paths, expected_imu_rows, ...
    expected_range_rows, sample_interval_s, 1/range_rate_hz, ...
    duration_s, target_range_m, expected_max_range_m, ...
    range_validation_tolerance_m);
field_names = fieldnames(final_paths);
for index = 1:numel(field_names)
    field_name = field_names{index};
    [move_ok, move_message] = movefile(temporary_paths.(field_name), ...
        final_paths.(field_name), 'f');
    if ~move_ok
        error('提交%s失败：%s', field_name, move_message);
    end
end
clear temporary_cleanup;

generation_info = struct();
generation_info.case_name = 'case-07';
generation_info.description = ...
    'single beacon, straight track with approximately 5 km cross-track range';
generation_info.duration_s = duration_s;
generation_info.sample_rate_hz = sample_rate_hz;
generation_info.range_rate_hz = range_rate_hz;
generation_info.static_duration_s = static_duration_s;
generation_info.acceleration_duration_s = acceleration_duration_s;
generation_info.cruise_speed_mps = cruise_speed_mps;
generation_info.target_range_m = target_range_m;
generation_info.uniform_duration_s = uniform_duration_s;
generation_info.initial_yaw_deg = initial_yaw_deg;
generation_info.beacon_enu_m = beacon_enu_m;
generation_info.total_along_track_m = total_along_track_m;
generation_info.expected_max_range_m = expected_max_range_m;
generation_info.max_cross_track_m = max_cross_track_m;
generation_info.random_seed = random_seed;
generation_info.imu_gyro_bias_dph = imu_gyro_bias_dph;
generation_info.imu_acc_bias_ug = imu_acc_bias_ug;
generation_info.imu_gyro_arw_deg_sqrt_h = imu_gyro_arw_deg_sqrt_h;
generation_info.imu_acc_vrw_mps_sqrt_h = imu_acc_vrw_mps_sqrt_h;
generation_info.imu_acc_vrw_ug_sqrt_hz = imu_acc_vrw_ug_sqrt_hz;
generation_info.actual_range_min_m = min(horizontal_range_m);
generation_info.actual_range_mean_m = mean(horizontal_range_m);
generation_info.actual_range_max_m = max(horizontal_range_m);
generation_info.expected_imu_rows = expected_imu_rows;
generation_info.expected_range_rows = expected_range_rows;
[geometry_png, geometry_fig] = draw_case07_geometry(trajectory_xyz, ...
    range_indices, beacon_enu_m, horizontal_range_m, output_dir);
generation_info.geometry_png = geometry_png;
generation_info.geometry_fig = geometry_fig;
save(fullfile(output_dir, 'generation-info.mat'), 'generation_info');
fprintf('case-07单信标5 km仿真数据生成并验证完成：%s\n', output_dir);
end

function [png_path, fig_path] = draw_case07_geometry(trajectory_xyz, ...
        range_indices, beacon_enu_m, horizontal_range_m, output_dir)
%DRAW_CASE07_GEOMETRY 绘制直线航迹与单信标的水平几何关系。
range_xyz = trajectory_xyz(range_indices, :);
[minimum_range_m, closest_index] = min(horizontal_range_m);
closest_point_m = range_xyz(closest_index, 1:2);
track_stride = max(1, floor(size(trajectory_xyz, 1)/5000));
plot_index = unique([1:track_stride:size(trajectory_xyz, 1), ...
    size(trajectory_xyz, 1)]);
track_km = trajectory_xyz(plot_index, 1:2)/1000;
beacon_km = beacon_enu_m(1:2)/1000;
closest_km = closest_point_m/1000;

fig = figure('Color', 'w', 'Visible', 'off', ...
    'Position', [100, 100, 820, 650], ...
    'Name', 'case-07 trajectory and beacon geometry');
ax = axes(fig);
hold(ax, 'on');
h_track = plot(ax, track_km(:, 1), track_km(:, 2), ...
    '-', 'Color', [0.10, 0.38, 0.72], 'LineWidth', 2.0);
h_start = plot(ax, track_km(1, 1), track_km(1, 2), 'o', ...
    'MarkerSize', 8, 'MarkerFaceColor', [0.16, 0.62, 0.42], ...
    'MarkerEdgeColor', 'w', 'LineWidth', 1.0);
h_end = plot(ax, track_km(end, 1), track_km(end, 2), 's', ...
    'MarkerSize', 8, 'MarkerFaceColor', [0.22, 0.22, 0.22], ...
    'MarkerEdgeColor', 'w', 'LineWidth', 1.0);
h_beacon = plot(ax, beacon_km(1), beacon_km(2), 'p', ...
    'MarkerSize', 14, 'MarkerFaceColor', [0.86, 0.24, 0.18], ...
    'MarkerEdgeColor', [0.55, 0.08, 0.06], 'LineWidth', 1.0);
h_closest = plot(ax, closest_km(1), closest_km(2), 'd', ...
    'MarkerSize', 8, 'MarkerFaceColor', [0.95, 0.65, 0.16], ...
    'MarkerEdgeColor', [0.55, 0.32, 0.03], 'LineWidth', 1.0);
plot(ax, [closest_km(1), beacon_km(1)], ...
    [closest_km(2), beacon_km(2)], '--', ...
    'Color', [0.45, 0.45, 0.45], 'LineWidth', 1.2);

direction_index = max(1, round(0.18*size(track_km, 1)));
direction_next = min(size(track_km, 1), direction_index + ...
    max(2, round(0.08*size(track_km, 1))));
quiver(ax, track_km(direction_index, 1), track_km(direction_index, 2), ...
    track_km(direction_next, 1)-track_km(direction_index, 1), ...
    track_km(direction_next, 2)-track_km(direction_index, 2), ...
    0, 'Color', [0.10, 0.38, 0.72], 'LineWidth', 1.6, ...
    'MaxHeadSize', 0.8, 'HandleVisibility', 'off');

text(ax, track_km(1, 1)+0.15, track_km(1, 2), 'Start', ...
    'Color', [0.10, 0.40, 0.25], 'FontWeight', 'bold');
text(ax, track_km(end, 1)+0.15, track_km(end, 2), 'End', ...
    'Color', [0.20, 0.20, 0.20], 'FontWeight', 'bold');
text(ax, beacon_km(1)+0.15, beacon_km(2), 'Beacon 1', ...
    'Color', [0.65, 0.10, 0.08], 'FontWeight', 'bold');
text(ax, mean([closest_km(1), beacon_km(1)]), ...
    mean([closest_km(2), beacon_km(2)])+0.18, ...
    sprintf('Closest range = %.2f km', minimum_range_m/1000), ...
    'HorizontalAlignment', 'center', 'Color', [0.30, 0.30, 0.30]);

range_text = sprintf(['Minimum range: %.2f km\n', ...
    'Mean range: %.2f km\nMaximum range: %.2f km'], ...
    min(horizontal_range_m)/1000, mean(horizontal_range_m)/1000, ...
    max(horizontal_range_m)/1000);
text(ax, 0.025, 0.975, range_text, 'Units', 'normalized', ...
    'VerticalAlignment', 'top', 'BackgroundColor', 'w', ...
    'EdgeColor', [0.78, 0.78, 0.78], 'Margin', 7);

axis(ax, 'equal');
grid(ax, 'on');
box(ax, 'on');
xlabel(ax, 'East / km');
ylabel(ax, 'North / km');
title(ax, 'case-07 straight trajectory and single-beacon geometry');
legend(ax, [h_track, h_start, h_end, h_beacon, h_closest], ...
    {'Trajectory', 'Start', 'End', 'Beacon 1', 'Closest point'}, ...
    'Location', 'southeast');
set(ax, 'FontName', 'Times New Roman', 'FontSize', 11, ...
    'LineWidth', 0.8, 'Layer', 'top');

padding_km = 0.65;
all_x = [track_km(:, 1); beacon_km(1)];
all_y = [track_km(:, 2); beacon_km(2)];
xlim(ax, [min(all_x)-padding_km, max(all_x)+padding_km]);
ylim(ax, [min(all_y)-padding_km, max(all_y)+padding_km]);

png_path = fullfile(output_dir, 'trajectory-beacon-geometry.png');
fig_path = fullfile(output_dir, 'trajectory-beacon-geometry.fig');
exportgraphics(fig, png_path, 'Resolution', 300);
savefig(fig, fig_path);
close(fig);
fprintf('航迹—信标几何图已生成：%s\n', png_path);
end

function value = validate_logical_scalar(value, name)
%VALIDATE_LOGICAL_SCALAR 检查逻辑开关。
if ~isscalar(value) || ~(islogical(value) || isnumeric(value)) || ...
        ~isfinite(double(value))
    error('%s必须是逻辑标量。', name);
end
value = logical(value);
end

function paths = build_case_paths(output_dir, suffix)
%BUILD_CASE_PATHS 生成正式或临时核心文件路径。
paths = struct();
paths.imu = fullfile(output_dir, ['IMU_120.txt', suffix]);
paths.truth = fullfile(output_dir, ['truth.txt', suffix]);
paths.range1 = fullfile(output_dir, ['range1.txt', suffix]);
end

function validate_case07_files(paths, expected_imu_rows, ...
        expected_range_rows, imu_dt, range_dt, duration_s, ...
        target_range_m, expected_max_range_m, range_tolerance_m)
%VALIDATE_CASE07_FILES 检查单信标case-07的数据契约。
imu = readmatrix(paths.imu, 'FileType', 'text');
truth = readmatrix(paths.truth, 'FileType', 'text');
range_data = readmatrix(paths.range1, 'FileType', 'text');
if size(imu, 1) ~= expected_imu_rows || size(imu, 2) ~= 7 || ...
        size(truth, 1) ~= expected_imu_rows || size(truth, 2) ~= 11 || ...
        size(range_data, 1) ~= expected_range_rows || ...
        size(range_data, 2) ~= 6
    error('case-07输出文件的行数或列数不符合数据契约。');
end
if any(~isfinite(imu), 'all') || any(~isfinite(truth), 'all') || ...
        any(~isfinite(range_data), 'all') || ...
        any(diff(imu(:, 1)) <= 0) || any(diff(truth(:, 2)) <= 0) || ...
        any(diff(range_data(:, 1)) <= 0)
    error('case-07输出中存在无效值或时间未严格递增。');
end
if abs(imu(1, 1)-imu_dt) > 1e-8 || ...
        abs(truth(1, 2)-imu_dt) > 1e-8 || ...
        abs(range_data(1, 1)-range_dt) > 1e-8 || ...
        abs(imu(end, 1)-duration_s) > 1e-8 || ...
        abs(truth(end, 2)-duration_s) > 1e-8 || ...
        abs(range_data(end, 1)-duration_s) > 1e-8
    error('case-07输出的首末时间不正确。');
end
if any(abs(range_data(:, 2)-range_data(:, 3)) > 1e-9) || ...
        abs(min(range_data(:, 2))-target_range_m) > range_tolerance_m || ...
        abs(max(range_data(:, 2))-expected_max_range_m) > range_tolerance_m
    error('case-07测距列不一致，或直线航迹的距离范围异常。');
end
if any(max(abs(range_data(:, 4:6)-range_data(1, 4:6)), [], 1) > ...
        1e-12)
    error('case-07的单信标位置不是常量。');
end
fprintf(['完整性检查通过：%d个IMU/真值历元，%d个单信标测距历元，', ...
    '水平距离范围为[%.3f, %.3f] m。\n'], expected_imu_rows, ...
    expected_range_rows, min(range_data(:, 2)), max(range_data(:, 2)));
end

function delete_file_list(file_list)
%DELETE_FILE_LIST 清理失败运行遗留的临时文件。
for index = 1:numel(file_list)
    delete_if_exists(file_list{index});
end
end

function delete_if_exists(file_path)
%DELETE_IF_EXISTS 删除明确命名的case-07临时文件。
if isfile(file_path)
    delete(file_path);
end
end
