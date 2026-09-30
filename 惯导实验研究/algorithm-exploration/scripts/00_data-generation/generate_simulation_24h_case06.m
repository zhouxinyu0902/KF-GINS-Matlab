function output_dir = generate_simulation_24h_case06( ...
        overwrite_existing, dry_run)
%GENERATE_SIMULATION_24H_CASE06 分块生成 case-06 的24小时往返仿真数据。
%   generate_simulation_24h_case06() 在 case-06 不存在时生成数据。
%   generate_simulation_24h_case06(true) 生成并验证后替换旧数据。
%   generate_simulation_24h_case06(false, true) 只检查路径和参数，不写文件。
%
% 输出文件：IMU_120.txt、truth.txt、range1.txt、range2.txt、range3.txt。
% IMU/真值为100 Hz，三路理想水平距离为1 Hz。为避免约864万历元
% 同时驻留内存，脚本按1小时分块仿真并流式写入临时文件。航迹沿用
% case-00 的转弯轮廓，并在两端停车掉头后反向重走，实现连续往返。

%% 1. 用户可调整参数
duration_hours = 24;
chunk_duration_s = 3600;
sample_rate_hz = 100;
range_rate_hz = 1;
acceleration_duration_s = 10;
acceleration_mps2 = 0.43;
initial_yaw_deg = 37;
turnaround_rate_deg_s = 3;
turnaround_duration_s = 180/turnaround_rate_deg_s;
random_seed = 6;
beacons_enu_m = [0, -5*sqrt(3), 0; ...
    -10, 5*sqrt(3), 0; -20, -5*sqrt(3), 0]*1000;

if nargin < 1 || isempty(overwrite_existing)
    overwrite_existing = false;
end
if nargin < 2 || isempty(dry_run)
    dry_run = false;
end
overwrite_existing = validate_logical_scalar( ...
    overwrite_existing, 'overwrite_existing');
dry_run = validate_logical_scalar(dry_run, 'dry_run');

%% 2. 初始化路径和初始状态
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
paths = setup_inertial_experiment();
psins_dir = fullfile(fileparts(paths.project), 'PSINS', 'psins2401');
if exist('glvs', 'file') ~= 2
    if ~isfolder(psins_dir)
        error('找不到 PSINS：%s', psins_dir);
    end
    addpath(genpath(psins_dir));
end
required_functions = {'glvs', 'trjsegment', 'trjsimu', 'imuerrset', ...
    'pvaNED2ENU'};
for function_index = 1:numel(required_functions)
    if exist(required_functions{function_index}, 'file') ~= 2
        error('PSINS 路径初始化后仍缺少函数：%s', ...
            required_functions{function_index});
    end
end
glvs;

case_id = 6;
output_dir = paths.simulation_input(case_id);
if ~isfolder(output_dir)
    mkdir(output_dir);
end
case00_truth_path = fullfile(paths.simulation_input(0), 'truth.txt');
if ~isfile(case00_truth_path)
    error('缺少 case-06 往返航迹所需的 case-00 真值：%s', ...
        case00_truth_path);
end
case00_truth = readmatrix(case00_truth_path, 'FileType', 'text');
case00_avp = pvaNED2ENU(case00_truth);
control_indices = [1, 3921, 7001, 12658, 15624, 58769, 85522, ...
    98542, 129033, 135292, 145273, 173556, 180845, 185802, ...
    192935, 210573, 238800, 265681, 293682, 307233, 327564, ...
    354446, 404289, 419691, 499999];
sample_delta = diff(control_indices);
segment_duration_s = sample_delta/sample_rate_hz;
static_duration_s = segment_duration_s(1);
route_duration_s = sum(segment_duration_s(2:end));
route_start_row = round((static_duration_s+acceleration_duration_s)* ...
    sample_rate_hz);
route_boundary_rows = [route_start_row, ...
    route_start_row+cumsum(sample_delta(2:end))];
if size(case00_avp, 1) < route_boundary_rows(end)
    error('case-00 真值至少需要 %d 行，实际只有 %d 行。', ...
        route_boundary_rows(end), size(case00_avp, 1));
end
case00_yaw_rad = unwrap(case00_avp(:, 3));
route_yaw_rad = reshape(case00_yaw_rad(route_boundary_rows), 1, []);
yaw_rate_deg_s = rad2deg(diff(route_yaw_rad))./ ...
    segment_duration_s(2:end);
if ~isrow(yaw_rate_deg_s) || ...
        numel(yaw_rate_deg_s) ~= numel(segment_duration_s(2:end))
    error('case-00 转弯角速度与分段时长的维度不一致。');
end

initial_position = case00_avp(1, 7:9)';
initial_avp = [[0; 0; deg2rad(initial_yaw_deg)]; ...
    [0; 0; 0]; initial_position];

duration_s = duration_hours*3600;
sample_interval_s = 1/sample_rate_hz;
samples_per_chunk = round(chunk_duration_s*sample_rate_hz);
range_stride = round(sample_rate_hz/range_rate_hz);
expected_imu_rows = round(duration_s*sample_rate_hz);
expected_range_rows = round(duration_s*range_rate_hz);
if mod(duration_s, chunk_duration_s) ~= 0 || ...
        mod(sample_rate_hz, range_rate_hz) ~= 0
    error('duration/chunk 或 IMU/range 采样率必须是整数倍关系。');
end
chunk_count = duration_s/chunk_duration_s;
estimated_size_gb = 2.0;
round_trip_duration_s = 2*(acceleration_duration_s+route_duration_s+ ...
    acceleration_duration_s+turnaround_duration_s);
expected_round_trips = (duration_s-static_duration_s)/ ...
    round_trip_duration_s;

fprintf('case-06 输出目录：%s\n', output_dir);
fprintf('时长：%g h；IMU/真值：%g Hz；距离：%g Hz。\n', ...
    duration_hours, sample_rate_hz, range_rate_hz);
fprintf('预计写入：%d 个IMU历元，%d 个距离历元，约 %.1f GB文本。\n', ...
    expected_imu_rows, expected_range_rows, estimated_size_gb);
fprintf(['航迹：复用 case-00 转弯轮廓，端点停车并以 %.3f deg/s ', ...
    '原地掉头；24小时约 %.2f 个往返。\n'], ...
    turnaround_rate_deg_s, expected_round_trips);
if dry_run
    fprintf('dry_run=true：环境检查通过，未写入任何数据。\n');
    return;
end

%% 3. 覆盖保护和临时文件
final_paths = build_case_paths(output_dir, '');
final_exists = cellfun(@isfile, struct2cell(final_paths));
if all(final_exists) && ~overwrite_existing
    fprintf('case-06 已完整存在，未覆盖。若需重建，请传入 true。\n');
    return;
end
if any(final_exists) && ~overwrite_existing
    error(['case-06 只存在部分核心文件。为避免混用不同批次数据，', ...
        '请确认后运行 generate_simulation_24h_case06(true)。']);
end

temporary_paths = build_case_paths(output_dir, '.generating');
temporary_files = struct2cell(temporary_paths);
for index = 1:numel(temporary_files)
    delete_if_exists(temporary_files{index});
end

imu_fp = fopen(temporary_paths.imu, 'wt');
truth_fp = fopen(temporary_paths.truth, 'wt');
range_fp = [fopen(temporary_paths.range1, 'wt'), ...
    fopen(temporary_paths.range2, 'wt'), ...
    fopen(temporary_paths.range3, 'wt')];
all_file_ids = [imu_fp, truth_fp, range_fp];
if any(all_file_ids < 0)
    close_file_ids(all_file_ids);
    error('无法创建 case-06 临时输出文件：%s', output_dir);
end
generation_cleanup = onCleanup(@() cleanup_partial_generation( ...
    all_file_ids, temporary_files));

%% 4. 固定信标和IMU误差
origin = initial_position;
beacon_position = dxyz2pos(beacons_enu_m, origin);
imu_error = imuerrset(0.01, 7, 0.0005, 10e-6*1e5/3600);
rng(random_seed, 'twister');

imu_format = ['%.9f %.12g %.12g %.12g %.12g %.12g %.12g\n'];
truth_format = ['%2d %12.6f %12.8f %12.8f %8.4f %8.4f ', ...
    '%8.4f %8.4f %8.4f %8.4f %8.4f\n'];
range_format = '%.6f %.8f %.8f %.12g %.12g %.4f\n';

%% 5. 构造 case-00 往返运动计划，逐小时仿真并流式写入
current_avp = initial_avp;
[motion_phases, planned_duration_s] = build_shuttle_motion_phases( ...
    duration_s, static_duration_s, acceleration_duration_s, ...
    acceleration_mps2, segment_duration_s(2:end), ...
    yaw_rate_deg_s, turnaround_duration_s, ...
    turnaround_rate_deg_s);
if abs(planned_duration_s-duration_s) > 1e-9
    error('内部运动计划时长不是24小时：%.12g s。', planned_duration_s);
end

phase_index = 1;
phase_elapsed_s = 0;
current_speed_mps = 0;
for chunk_index = 1:chunk_count
    segment = trjsegment([], 'init', current_speed_mps);
    remaining_chunk_s = chunk_duration_s;
    while remaining_chunk_s > 1e-10
        if phase_index > numel(motion_phases)
            error('运动计划提前结束于第 %d 个分块。', chunk_index);
        end
        available_s = motion_phases(phase_index).duration_s- ...
            phase_elapsed_s;
        step_duration_s = min(remaining_chunk_s, available_s);
        [segment, current_speed_mps] = append_motion_phase( ...
            segment, motion_phases(phase_index), ...
            step_duration_s, current_speed_mps);
        remaining_chunk_s = remaining_chunk_s-step_duration_s;
        phase_elapsed_s = phase_elapsed_s+step_duration_s;
        if abs(phase_elapsed_s-motion_phases(phase_index).duration_s) ...
                <= 1e-10
            phase_index = phase_index+1;
            phase_elapsed_s = 0;
        end
    end

    trajectory = trjsimu(current_avp, segment.wat, ...
        sample_interval_s, 1);
    if size(trajectory.imu, 1) ~= samples_per_chunk
        error('第 %d 分块样本数异常：应为 %d，实际为 %d。', ...
            chunk_index, samples_per_chunk, size(trajectory.imu, 1));
    end

    time_offset_s = (chunk_index-1)*chunk_duration_s;
    trajectory.imu(:, 7) = trajectory.imu(:, 7)+time_offset_s;
    trajectory.avp(:, 10) = trajectory.avp(:, 10)+time_offset_s;
    noisy_imu = imuadderr(trajectory.imu, imu_error);
    imu = imuRFU2FRD(noisy_imu);
    truth = avpENU2NED(trajectory.avp);

    fprintf(imu_fp, imu_format, imu');
    fprintf(truth_fp, truth_format, truth');

    local_range_indices = range_stride:range_stride:samples_per_chunk;
    trajectory_position = truth(local_range_indices, 3:5);
    trajectory_position(:, 1:2) = ...
        deg2rad(trajectory_position(:, 1:2));
    trajectory_xyz = pos2dxyz(trajectory_position, origin);
    for beacon_index = 1:3
        horizontal_range = hypot( ...
            trajectory_xyz(:, 1)-beacons_enu_m(beacon_index, 1), ...
            trajectory_xyz(:, 2)-beacons_enu_m(beacon_index, 2));
        beacon_rows = repmat(beacon_position(beacon_index, :), ...
            numel(local_range_indices), 1);
        range_output = [truth(local_range_indices, 2), ...
            horizontal_range, horizontal_range, beacon_rows];
        fprintf(range_fp(beacon_index), range_format, range_output');
    end

    assert_no_file_error(all_file_ids, chunk_index);

    current_avp = trajectory.avp(end, 1:9)';
    fprintf('case-06 生成进度：%d/%d h（%.1f%%）\n', ...
        chunk_index, chunk_count, 100*chunk_index/chunk_count);
end

close_file_ids(all_file_ids);

%% 6. 完整性检查通过后再替换正式文件
validate_case06_files(temporary_paths, expected_imu_rows, ...
    expected_range_rows, sample_interval_s, 1/range_rate_hz, duration_s);
field_names = fieldnames(final_paths);
for index = 1:numel(field_names)
    field_name = field_names{index};
    [move_ok, move_message] = movefile(temporary_paths.(field_name), ...
        final_paths.(field_name), 'f');
    if ~move_ok
        error('提交 %s 失败：%s', field_name, move_message);
    end
end
clear generation_cleanup;

generation_info = struct();
generation_info.case_name = 'case-06';
generation_info.duration_hours = duration_hours;
generation_info.sample_rate_hz = sample_rate_hz;
generation_info.range_rate_hz = range_rate_hz;
generation_info.chunk_duration_s = chunk_duration_s;
generation_info.initial_yaw_deg = initial_yaw_deg;
generation_info.acceleration_mps2 = acceleration_mps2;
generation_info.source_trajectory = 'case-00 turn profile';
generation_info.source_truth_path = case00_truth_path;
generation_info.route_duration_s = route_duration_s;
generation_info.turnaround_rate_deg_s = turnaround_rate_deg_s;
generation_info.turnaround_duration_s = turnaround_duration_s;
generation_info.expected_round_trips = expected_round_trips;
generation_info.beacons_enu_m = beacons_enu_m;
generation_info.random_seed = random_seed;
generation_info.expected_imu_rows = expected_imu_rows;
generation_info.expected_range_rows = expected_range_rows;
save(fullfile(output_dir, 'generation-info.mat'), 'generation_info');
fprintf('case-06 24小时往返仿真数据生成并验证完成：%s\n', output_dir);
end

function [phases, total_duration_s] = build_shuttle_motion_phases( ...
        target_duration_s, static_duration_s, acceleration_duration_s, ...
        acceleration_mps2, route_duration_s, route_yaw_rate_deg_s, ...
        turnaround_duration_s, turnaround_rate_deg_s)
%BUILD_SHUTTLE_MOTION_PHASES 构造“case-00正向-掉头-反向-掉头”计划。
phase_template = struct('type', '', 'duration_s', 0, 'value', 0);
phases = repmat(phase_template, 0, 1);

initial_phase = make_motion_phase('uniform', static_duration_s, 0);
cycle_phases = repmat(phase_template, 0, 1);
cycle_phases(end+1, 1) = make_motion_phase( ...
    'accelerate', acceleration_duration_s, acceleration_mps2);
for index = 1:numel(route_duration_s)
    cycle_phases(end+1, 1) = make_motion_phase( ...
        'turn', route_duration_s(index), route_yaw_rate_deg_s(index));
end
cycle_phases(end+1, 1) = make_motion_phase( ...
    'deaccelerate', acceleration_duration_s, acceleration_mps2);
cycle_phases(end+1, 1) = make_motion_phase( ...
    'turn', turnaround_duration_s, turnaround_rate_deg_s);
cycle_phases(end+1, 1) = make_motion_phase( ...
    'accelerate', acceleration_duration_s, acceleration_mps2);
for index = numel(route_duration_s):-1:1
    cycle_phases(end+1, 1) = make_motion_phase( ...
        'turn', route_duration_s(index), -route_yaw_rate_deg_s(index));
end
cycle_phases(end+1, 1) = make_motion_phase( ...
    'deaccelerate', acceleration_duration_s, acceleration_mps2);
cycle_phases(end+1, 1) = make_motion_phase( ...
    'turn', turnaround_duration_s, turnaround_rate_deg_s);

total_duration_s = 0;
[phases, total_duration_s] = append_trimmed_phase( ...
    phases, initial_phase, total_duration_s, target_duration_s);
while total_duration_s < target_duration_s-1e-10
    for index = 1:numel(cycle_phases)
        [phases, total_duration_s] = append_trimmed_phase( ...
            phases, cycle_phases(index), total_duration_s, ...
            target_duration_s);
        if total_duration_s >= target_duration_s-1e-10
            break;
        end
    end
end
end

function phase = make_motion_phase(type, duration_s, value)
%MAKE_MOTION_PHASE 创建一个可被分块切分的运动基元。
phase = struct('type', type, 'duration_s', duration_s, 'value', value);
end

function [phases, total_duration_s] = append_trimmed_phase( ...
        phases, phase, total_duration_s, target_duration_s)
%APPEND_TRIMMED_PHASE 追加运动基元，并在24小时边界处截断。
remaining_s = target_duration_s-total_duration_s;
if remaining_s <= 1e-10
    return;
end
phase.duration_s = min(phase.duration_s, remaining_s);
if phase.duration_s <= 1e-10
    return;
end
phases(end+1, 1) = phase;
total_duration_s = total_duration_s+phase.duration_s;
end

function [segment, current_speed_mps] = append_motion_phase( ...
        segment, phase, duration_s, current_speed_mps)
%APPEND_MOTION_PHASE 向当前1小时分块追加一个完整或截断的运动基元。
switch phase.type
    case 'uniform'
        segment = trjsegment(segment, 'uniform', duration_s);
    case 'accelerate'
        segment = trjsegment(segment, 'accelerate', ...
            duration_s, [], phase.value);
        current_speed_mps = current_speed_mps+duration_s*phase.value;
    case 'deaccelerate'
        segment = trjsegment(segment, 'deaccelerate', ...
            duration_s, [], phase.value);
        current_speed_mps = current_speed_mps-duration_s*phase.value;
        if abs(current_speed_mps) < 1e-10
            current_speed_mps = 0;
        elseif current_speed_mps < 0
            error('减速阶段产生了负速度：%.12g m/s。', current_speed_mps);
        end
    case 'turn'
        segment = trjsegment(segment, 'turnleft', ...
            duration_s, phase.value);
    otherwise
        error('未知运动基元：%s。', phase.type);
end
end

function value = validate_logical_scalar(value, name)
%VALIDATE_LOGICAL_SCALAR 检查逻辑开关。
if ~isscalar(value) || ~(islogical(value) || isnumeric(value)) || ...
        ~isfinite(double(value))
    error('%s 必须是逻辑标量。', name);
end
value = logical(value);
end

function paths = build_case_paths(output_dir, suffix)
%BUILD_CASE_PATHS 生成正式或临时核心文件路径。
paths = struct();
paths.imu = fullfile(output_dir, ['IMU_120.txt', suffix]);
paths.truth = fullfile(output_dir, ['truth.txt', suffix]);
paths.range1 = fullfile(output_dir, ['range1.txt', suffix]);
paths.range2 = fullfile(output_dir, ['range2.txt', suffix]);
paths.range3 = fullfile(output_dir, ['range3.txt', suffix]);
end

function validate_case06_files(paths, expected_imu_rows, ...
        expected_range_rows, imu_dt, range_dt, duration_s)
%VALIDATE_CASE06_FILES 检查边界记录；行数由分块样本契约保证。
[imu_first, imu_last, imu_columns] = inspect_file_edges(paths.imu, 1);
[truth_first, truth_last, truth_columns] = ...
    inspect_file_edges(paths.truth, 2);
if imu_columns ~= 7 || truth_columns ~= 11 || ...
        abs(imu_first-imu_dt) > 1e-8 || ...
        abs(truth_first-imu_dt) > 1e-8 || ...
        abs(imu_last-duration_s) > 1e-8 || ...
        abs(truth_last-duration_s) > 1e-8
    error('case-06 IMU/truth 的行数、列数或时间范围不正确。');
end

for beacon_index = 1:3
    path = paths.(sprintf('range%d', beacon_index));
    [first_time, last_time, columns] = inspect_file_edges(path, 1);
    if columns ~= 6 || ...
            abs(first_time-range_dt) > 1e-8 || ...
            abs(last_time-duration_s) > 1e-8
        error('case-06 range%d 的行数、列数或时间范围不正确。', ...
            beacon_index);
    end
end
fprintf(['完整性检查通过：预期 %d 个IMU/真值历元，', ...
    '每路 %d 个距离历元。\n'], expected_imu_rows, expected_range_rows);
end

function [first_time, last_time, column_count] = ...
        inspect_file_edges(file_path, time_column)
%INSPECT_FILE_EDGES 读取首末记录，避免重新扫描约2.4 GB文本。
file_id = fopen(file_path, 'rt');
if file_id < 0
    error('无法读取生成文件：%s', file_path);
end
cleanup = onCleanup(@() fclose(file_id));
first_line = fgetl(file_id);
if ~ischar(first_line)
    error('生成文件为空：%s', file_path);
end
first_values = sscanf(first_line, '%f')';
column_count = numel(first_values);

fseek(file_id, 0, 'eof');
file_size = ftell(file_id);
tail_size = min(file_size, 8192);
fseek(file_id, -tail_size, 'eof');
tail_text = fread(file_id, tail_size, '*char')';
tail_lines = regexp(tail_text, '\r?\n', 'split');
tail_lines = tail_lines(~cellfun(@(line) isempty(strtrim(line)), tail_lines));
if isempty(tail_lines)
    error('无法读取生成文件的末行：%s', file_path);
end
last_values = sscanf(tail_lines{end}, '%f')';
if column_count ~= numel(last_values) || ...
        time_column > column_count || ...
        any(~isfinite(first_values)) || any(~isfinite(last_values))
    error('生成文件的首末记录格式不一致：%s', file_path);
end
first_time = first_values(time_column);
last_time = last_values(time_column);
end

function assert_no_file_error(file_ids, chunk_index)
%ASSERT_NO_FILE_ERROR 每个分块写入后检查磁盘写入状态。
for index = 1:numel(file_ids)
    [message, error_number] = ferror(file_ids(index));
    if error_number ~= 0
        error('第 %d 分块写入失败：%s', chunk_index, message);
    end
end
end

function close_file_ids(file_ids)
%CLOSE_FILE_IDS 关闭仍处于打开状态的文件。
for index = 1:numel(file_ids)
    if file_ids(index) >= 0
        try
            fclose(file_ids(index));
        catch
        end
    end
end
end

function delete_file_list(file_list)
%DELETE_FILE_LIST 清理失败运行遗留的临时文件。
for index = 1:numel(file_list)
    delete_if_exists(file_list{index});
end
end

function cleanup_partial_generation(file_ids, file_list)
%CLEANUP_PARTIAL_GENERATION 出错时严格先关闭文件，再删除临时结果。
close_file_ids(file_ids);
delete_file_list(file_list);
end

function delete_if_exists(file_path)
%DELETE_IF_EXISTS 只删除 case-06 中明确命名的临时文件。
if isfile(file_path)
    delete(file_path);
end
end
