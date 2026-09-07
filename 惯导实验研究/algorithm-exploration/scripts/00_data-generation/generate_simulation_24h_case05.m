function output_dir = generate_simulation_24h_case05( ...
        overwrite_existing, dry_run)
%GENERATE_SIMULATION_24H_CASE05 分块生成 case-05 的24小时仿真数据。
%   generate_simulation_24h_case05() 在 case-05 不存在时生成数据。
%   generate_simulation_24h_case05(true) 生成并验证后替换旧数据。
%   generate_simulation_24h_case05(false, true) 只检查路径和参数，不写文件。
%
% 输出文件：IMU_120.txt、truth.txt、range1.txt、range2.txt、range3.txt。
% IMU/真值为100 Hz，三路理想水平距离为1 Hz。为避免约864万历元
% 同时驻留内存，脚本按1小时分块仿真并流式写入临时文件。

%% 1. 用户可调整参数
duration_hours = 24;
chunk_duration_s = 3600;
sample_rate_hz = 100;
range_rate_hz = 1;
static_duration_s = 60;
acceleration_duration_s = 10;
acceleration_mps2 = 0.20577;
initial_yaw_deg = 323.03;
turn_rate_deg_s = 0.02;             % 约5小时完成一圈
random_seed = 5;
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
required_functions = {'glvs', 'trjsegment', 'trjsimu', 'imuerrset'};
for function_index = 1:numel(required_functions)
    if exist(required_functions{function_index}, 'file') ~= 2
        error('PSINS 路径初始化后仍缺少函数：%s', ...
            required_functions{function_index});
    end
end
glvs;

case_id = 5;
output_dir = paths.simulation_input(case_id);
if ~isfolder(output_dir)
    mkdir(output_dir);
end
reference_truth_path = fullfile(paths.experiment_input(6), 'truth.nav');
if ~isfile(reference_truth_path)
    error('缺少 case-05 初始位置所需的参考真值：%s', ...
        reference_truth_path);
end
reference_truth = readmatrix(reference_truth_path, 'FileType', 'text');
initial_position = [deg2rad(reference_truth(1, 3:4))'; ...
    reference_truth(1, 5)];
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

fprintf('case-05 输出目录：%s\n', output_dir);
fprintf('时长：%g h；IMU/真值：%g Hz；距离：%g Hz。\n', ...
    duration_hours, sample_rate_hz, range_rate_hz);
fprintf('预计写入：%d 个IMU历元，%d 个距离历元，约 %.1f GB文本。\n', ...
    expected_imu_rows, expected_range_rows, estimated_size_gb);
fprintf('默认航迹：静止 %.0f s，加速 %.0f s，随后以 %.3f deg/s持续左转。\n', ...
    static_duration_s, acceleration_duration_s, turn_rate_deg_s);
if dry_run
    fprintf('dry_run=true：环境检查通过，未写入任何数据。\n');
    return;
end

%% 3. 覆盖保护和临时文件
final_paths = build_case_paths(output_dir, '');
final_exists = cellfun(@isfile, struct2cell(final_paths));
if all(final_exists) && ~overwrite_existing
    fprintf('case-05 已完整存在，未覆盖。若需重建，请传入 true。\n');
    return;
end
if any(final_exists) && ~overwrite_existing
    error(['case-05 只存在部分核心文件。为避免混用不同批次数据，', ...
        '请确认后运行 generate_simulation_24h_case05(true)。']);
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
    error('无法创建 case-05 临时输出文件：%s', output_dir);
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

%% 5. 逐小时仿真并流式写入
current_avp = initial_avp;
cruise_speed_mps = acceleration_duration_s*acceleration_mps2;
for chunk_index = 1:chunk_count
    if chunk_index == 1
        moving_duration_s = chunk_duration_s-static_duration_s- ...
            acceleration_duration_s;
        if moving_duration_s <= 0
            error('首分块时长不足以容纳静止和加速阶段。');
        end
        segment = trjsegment([], 'init', 0);
        segment = trjsegment(segment, 'uniform', static_duration_s);
        segment = trjsegment(segment, 'accelerate', ...
            acceleration_duration_s, [], acceleration_mps2);
        segment = trjsegment(segment, 'turnleft', ...
            moving_duration_s, turn_rate_deg_s);
    else
        segment = trjsegment([], 'init', cruise_speed_mps);
        segment = trjsegment(segment, 'turnleft', ...
            chunk_duration_s, turn_rate_deg_s);
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
    fprintf('case-05 生成进度：%d/%d h（%.1f%%）\n', ...
        chunk_index, chunk_count, 100*chunk_index/chunk_count);
end

close_file_ids(all_file_ids);

%% 6. 完整性检查通过后再替换正式文件
validate_case05_files(temporary_paths, expected_imu_rows, ...
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
generation_info.case_name = 'case-05';
generation_info.duration_hours = duration_hours;
generation_info.sample_rate_hz = sample_rate_hz;
generation_info.range_rate_hz = range_rate_hz;
generation_info.chunk_duration_s = chunk_duration_s;
generation_info.initial_yaw_deg = initial_yaw_deg;
generation_info.acceleration_mps2 = acceleration_mps2;
generation_info.turn_rate_deg_s = turn_rate_deg_s;
generation_info.beacons_enu_m = beacons_enu_m;
generation_info.random_seed = random_seed;
generation_info.expected_imu_rows = expected_imu_rows;
generation_info.expected_range_rows = expected_range_rows;
save(fullfile(output_dir, 'generation-info.mat'), 'generation_info');
fprintf('case-05 24小时仿真数据生成并验证完成：%s\n', output_dir);
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

function validate_case05_files(paths, expected_imu_rows, ...
        expected_range_rows, imu_dt, range_dt, duration_s)
%VALIDATE_CASE05_FILES 检查边界记录；行数由分块样本契约保证。
[imu_first, imu_last, imu_columns] = inspect_file_edges(paths.imu, 1);
[truth_first, truth_last, truth_columns] = ...
    inspect_file_edges(paths.truth, 2);
if imu_columns ~= 7 || truth_columns ~= 11 || ...
        abs(imu_first-imu_dt) > 1e-8 || ...
        abs(truth_first-imu_dt) > 1e-8 || ...
        abs(imu_last-duration_s) > 1e-8 || ...
        abs(truth_last-duration_s) > 1e-8
    error('case-05 IMU/truth 的行数、列数或时间范围不正确。');
end

for beacon_index = 1:3
    path = paths.(sprintf('range%d', beacon_index));
    [first_time, last_time, columns] = inspect_file_edges(path, 1);
    if columns ~= 6 || ...
            abs(first_time-range_dt) > 1e-8 || ...
            abs(last_time-duration_s) > 1e-8
        error('case-05 range%d 的行数、列数或时间范围不正确。', ...
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
%DELETE_IF_EXISTS 只删除 case-05 中明确命名的临时文件。
if isfile(file_path)
    delete(file_path);
end
end
