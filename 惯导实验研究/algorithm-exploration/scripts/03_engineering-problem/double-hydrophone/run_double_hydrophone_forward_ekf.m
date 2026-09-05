clear;
close all;
clc;
%% 双水听器相位差辅助的前向 EKF 与可选 RTS
% 流程：距离预更新（仅用于候选角裁决） -> 相位差候选角 -> 距离+方位角联合更新。
% 预更新在 kf/navstate 的副本上执行，正式滤波只进行一次联合量测更新，
% 因此同一条距离量测不会被重复计入 EKF。
% 导航事件架构与原 range/INS 主链一致，并在每个测距历元：
%     1) 用距离更新副本得到候选角裁决先验；
%     2) 从同步相位差生成候选角并选择完整相对方位角；
%     3) 组装 1x9 距离+方位角观测；
%     4) 调用 update_range_azimuth_filter_rad 和配套姿态反馈；
%     5) 根据开关执行一次分段 RTS 和二次跨区间 RTS。
data_source = "experiment";          % "simulation" 或 "experiment"
case_name = 'case-00';

options = struct( ...
    'range_interval_s', 420, ...
    'end_time_s', 4621, ...
    'beacon_order', [1, 2, 3], ...
    'random_seed', 1, ...
    'simulation_range_noise_std_m', 10, ...
    'experiment_range_noise_std_m', 6, ...
    'depth_noise_std_m', 0.4, ...
    'filter_range_std_m', 10, ...
    'filter_depth_std_m', 0.4, ...
    'baseline_m', 3.0, ...
    'baseline_install_deg', 0, ...
    ... % 必须与 generate_analyze_phase_data.m 的相位仿真参数一致。
    'carrier_hz', 3e3, ...
    'sound_speed_mps', 1500, ...
    'filter_azimuth_std_deg', 0.30, ...
    'phase_time_tolerance_s', [], ...
    'enable_smoothing', true, ...
    'enable_second_rts', true);
%% 路径配置
case_name = char(case_name);
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
addpath(script_dir);
addpath(fullfile(script_dir, 'function'));
paths = setup_inertial_experiment();
param = Param();
glvs;
rng(options.random_seed, 'twister');

data_source = lower(string(data_source));
if data_source == "simulation"
    input_dir = fullfile(paths.simulation_input, case_name);
    cfg = load_algorithm_exploration_config('simulation', 'rad', input_dir);
    result_dir = fullfile(paths.simulation_navigation, case_name, ...
        'double-hydrophone');
    range_noise_std_m = options.simulation_range_noise_std_m;
    filter_range_std_m = options.filter_range_std_m;
elseif data_source == "experiment"
    case_name = paths.experiment_case;
    input_dir = paths.experiment_input;
    cfg = load_algorithm_exploration_config('experiment', 'rad', []);
    result_dir = fullfile(paths.experiment_navigation, ...
        'double-hydrophone');
    range_noise_std_m = options.experiment_range_noise_std_m;
    filter_range_std_m = options.experiment_range_noise_std_m;
else
    error('data_source 只能设置为 "simulation" 或 "experiment"。');
end
if ~isfolder(result_dir)
    mkdir(result_dir);
end
cfg.userange = true;
%% 输入文件检查
imu_all = readmatrix(cfg.imufilepath, 'FileType', 'text');
truth_all = readmatrix(cfg.truthpath, 'FileType', 'text');
range_paths = {cfg.rangefile1path, cfg.rangefile2path, cfg.rangefile3path};
phase_paths = arrayfun(@(index) fullfile(input_dir, ...
    sprintf('phase%d.txt', index)), 1:3, 'UniformOutput', false);
if any(~cellfun(@isfile, phase_paths))
    missing = phase_paths(~cellfun(@isfile, phase_paths));
    error('找不到相位文件：%s', strjoin(missing, ', '));
end

range_sources = cellfun(@(path) readmatrix(path, 'FileType', 'text'), ...
    range_paths, 'UniformOutput', false);
phase_sources = cellfun(@(path) readmatrix(path, 'FileType', 'text'), ...
    phase_paths, 'UniformOutput', false);
if data_source == "experiment"
    height_path = fullfile(input_dir, 'height_noised.txt');
    if ~isfile(height_path)
        error('找不到实验高度文件：%s', height_path);
    end
    height_source = readmatrix(height_path, 'FileType', 'text');
else
    height_source = [];
end
[rangedata, range_beacon_id, phase_measurement_deg] = ...
    build_measurement_events(range_sources, phase_sources, options);
rangedata(:, 3) = rangedata(:, 3) + range_noise_std_m * ...
    randn(size(rangedata, 1), 1);

start_time = max([cfg.starttime, imu_all(1, 1), truth_all(1, 2)]);
end_time = min([start_time + options.end_time_s, cfg.endtime, ...
    imu_all(end, 1), truth_all(end, 2)]);
cfg.starttime = start_time;
cfg.endtime = end_time;

imu_mask = imu_all(:, 1) >= start_time & imu_all(:, 1) <= end_time;
imudata = imu_all(imu_mask, :);
range_mask = rangedata(:, 1) >= start_time & rangedata(:, 1) <= end_time;
rangedata = rangedata(range_mask, :);
range_beacon_id = range_beacon_id(range_mask);
phase_measurement_deg = phase_measurement_deg(range_mask);
if isempty(rangedata)
    error('当前时间范围内没有测距/相位事件。');
end

if data_source == "simulation"
    height_values = interp1(truth_all(:, 2), truth_all(:, 5), ...
        imudata(:, 1), 'linear', 'extrap');
    height_values = height_values + options.depth_noise_std_m * ...
        randn(size(height_values));
else
    height_values = interp1(height_source(:, 1), height_source(:, 2), ...
        imudata(:, 1), 'linear', 'extrap');
end
height = [imudata(:, 1), height_values];
[imudata, height, alignment_info] = ...
    align_imu_to_range_epochs(imudata, height, rangedata(:, 1));
%% 滤波开始
[kf, navstate] = myInitialize_15state(cfg);
kf.rangstd = filter_range_std_m;
kf.depthstd = options.filter_depth_std_m;

sample_count = size(imudata, 1);
forward_nav = nan(sample_count, 11);
navstate.time = imudata(1, 1);
forward_nav(1, :) = state_to_nav_row(navstate, param);
% [time, beacon, prior azimuth, selected azimuth, absolute difference]
diagnostic_data = nan(size(rangedata, 1), 5);
used_event_count = 0;

single_rts_nav = zeros(11, 0);
double_rts_nav = zeros(11, 0);
if options.enable_smoothing
    imu_interval_s = median(diff(imudata(:, 1)));
    maximum_segment_samples = ...
        ceil(options.range_interval_s / imu_interval_s) + 20;
    state_buffer = zeros(maximum_segment_samples, 10);
    corrected_covariance_buffer = zeros(maximum_segment_samples, 225);
    predicted_covariance_buffer = zeros(maximum_segment_samples, 225);
    transition_buffer = zeros(maximum_segment_samples, 225);
    buffer_index = 1;

    previous_single_state = [];
    previous_single_nav = [];
    previous_corrected_covariance = [];
    previous_predicted_covariance = [];
    previous_transition = [];
    previous_range_index = 0;
end

range_index = find(rangedata(:, 1) >= imudata(1, 1), 1, 'first');
if isempty(range_index)
    range_index = size(rangedata, 1) + 1;
end

this_imu = imudata(1, :)';
last_progress = -1;
for imu_index = 2:sample_count
    last_imu = this_imu;
    this_imu = imudata(imu_index, :)';
    imu_dt = this_imu(1) - last_imu(1);
    time_tolerance = max(1e-8, abs(imu_dt) * 0.25);

    while range_index <= size(rangedata, 1) && ...
            rangedata(range_index, 1) < last_imu(1) - time_tolerance
        warning('测距时刻 %.6f s 未与 IMU 对齐，已跳过。', ...
            rangedata(range_index, 1));
        range_index = range_index + 1;
    end
    is_range_epoch = range_index <= size(rangedata, 1) && ...
        abs(last_imu(1) - rangedata(range_index, 1)) <= time_tolerance;

    if is_range_epoch
        current_range = rangedata(range_index, :);
        current_depth = height(imu_index - 1, :);
        [joint_range, selection] = build_range_azimuth_measurement_from_phase( ...
            navstate, kf, current_range, current_depth, ...
            phase_measurement_deg(range_index), options);

        % 正式滤波只执行本次联合更新，预更新副本不会写回。
        kf = update_range_azimuth_filter_rad( ...
            navstate, joint_range, current_depth, kf, ...
            options.filter_azimuth_std_deg);

        %% 对刚结束的测距区间执行一次、二次 RTS
        terminal_error = kf.x;
        if options.enable_smoothing && buffer_index > 1
            valid_length = buffer_index - 1;
            current_state = state_buffer(1:valid_length, :);
            current_corrected_covariance = ...
                corrected_covariance_buffer(1:valid_length, :);
            current_predicted_covariance = ...
                predicted_covariance_buffer(1:valid_length, :);
            current_transition = transition_buffer(1:valid_length, :);

            [single_nav, bridge_error, single_state] = ...
                perform_unified_smoothing( ...
                current_state, terminal_error, param, range_index, ...
                'RTS', 'rad', current_corrected_covariance, ...
                current_predicted_covariance, current_transition);
            single_rts_nav = [single_rts_nav, single_nav]; %#ok<AGROW>

            if options.enable_second_rts
                if isempty(previous_single_state)
                    % 第一段先缓存，等待下一段提供跨区间桥接误差。
                    previous_single_state = single_state;
                    previous_single_nav = single_nav;
                    previous_corrected_covariance = ...
                        current_corrected_covariance;
                    previous_predicted_covariance = ...
                        current_predicted_covariance;
                    previous_transition = current_transition;
                    previous_range_index = range_index;
                else
                    double_nav = perform_unified_smoothing( ...
                        previous_single_state, bridge_error, param, ...
                        previous_range_index, 'RTS', 'rad', ...
                        previous_corrected_covariance, ...
                        previous_predicted_covariance, previous_transition);
                    double_rts_nav = [double_rts_nav, double_nav]; %#ok<AGROW>

                    previous_single_state = single_state;
                    previous_single_nav = single_nav;
                    previous_corrected_covariance = ...
                        current_corrected_covariance;
                    previous_predicted_covariance = ...
                        current_predicted_covariance;
                    previous_transition = current_transition;
                    previous_range_index = range_index;
                end
            end

            buffer_index = 1;
            state_buffer(:) = 0;
            corrected_covariance_buffer(:) = 0;
            predicted_covariance_buffer(:) = 0;
            transition_buffer(:) = 0;
        end

        [kf, navstate] = feedback_range_azimuth_state(kf, navstate);

        used_event_count = used_event_count + 1;
        diagnostic_data(used_event_count, :) = [ ...
            current_range(1), range_beacon_id(range_index), ...
            selection.predicted_relative_azimuth_deg, ...
            selection.selected_relative_azimuth_deg, ...
            selection.selected_difference_deg];
        forward_nav(imu_index - 1, :) = state_to_nav_row(navstate, param);

        fprintf(['Event %2d | t=%9.3f s | B%d | phase=%8.3f deg | ' ...
            'prior=%8.3f deg | selected=%8.3f deg | k=%3d | diff=%6.3f deg\n'], ...
            used_event_count, current_range(1), ...
            range_beacon_id(range_index), ...
            phase_measurement_deg(range_index), ...
            selection.predicted_relative_azimuth_deg, ...
            selection.selected_relative_azimuth_deg, ...
            selection.selected_cycle_k, selection.selected_difference_deg);
        range_index = range_index + 1;
    end

    last_state = navstate;
    navstate = InsMech(last_state, last_imu, this_imu);
    if ~is_range_epoch
        [kf, navstate] = update_decoupled_height( ...
            kf, navstate, height(imu_index, :));
    end

    if options.enable_smoothing
        if buffer_index > maximum_segment_samples
            error('RTS 缓存不足，请增大 maximum_segment_samples。');
        end
        corrected_covariance_buffer(buffer_index, :) = kf.P(:)';
    end
    kf = myInsPropagate_15state(navstate, this_imu, imu_dt, kf);

    if options.enable_smoothing
        state_buffer(buffer_index, :) = [navstate.time, ...
            navstate.pos', navstate.vel', navstate.att'];
        predicted_covariance_buffer(buffer_index, :) = kf.P(:)';
        transition_buffer(buffer_index, :) = kf.phi(:)';
        buffer_index = buffer_index + 1;
    end
    forward_nav(imu_index, :) = state_to_nav_row(navstate, param);

    progress = floor(10 * imu_index / sample_count) * 10;
    if progress > last_progress && mod(progress, 20) == 0
        fprintf('处理进度：%d %%\n', progress);
        last_progress = progress;
    end
end
diagnostic_data = diagnostic_data(1:used_event_count, :);

% 最后一段没有未来桥接误差，二次 RTS 退化为该段的一次 RTS。
if options.enable_smoothing && options.enable_second_rts && ...
        ~isempty(previous_single_nav)
    double_rts_nav = [double_rts_nav, previous_single_nav];
end
%% 输出

forward_path = fullfile(result_dir, ...
    'range-phase-azimuth-forward.nav');
diagnostic_path = fullfile(result_dir, ...
    'range-phase-azimuth-diagnostic.txt');
write_nav_file(forward_path, forward_nav);

diagnostic_fp = fopen(diagnostic_path, 'wt');
if diagnostic_fp < 0
    error('无法创建方位角诊断文件：%s', diagnostic_path);
end
fprintf(diagnostic_fp, ['time_s beacon_id prior_azimuth_deg ' ...
    'selected_azimuth_deg absolute_error_deg\n']);
fprintf(diagnostic_fp, '%12.6f %2d %12.6f %12.6f %12.6f\n', ...
    diagnostic_data');
fclose(diagnostic_fp);

single_rts_path = '';
double_rts_path = '';
if options.enable_smoothing
    single_rts_path = fullfile(result_dir, ...
        'range-phase-azimuth-rts-single.nav');
    write_nav_file(single_rts_path, single_rts_nav');
end
if options.enable_smoothing && options.enable_second_rts
    double_rts_path = fullfile(result_dir, ...
        'range-phase-azimuth-rts-double.nav');
    write_nav_file(double_rts_path, double_rts_nav');
end

outputs = struct( ...
    'data_source', char(data_source), ...
    'case_name', case_name, ...
    'input_dir', input_dir, ...
    'result_dir', result_dir, ...
    'forward_path', forward_path, ...
    'diagnostic_path', diagnostic_path, ...
    'single_rts_path', single_rts_path, ...
    'double_rts_path', double_rts_path, ...
    'event_count', used_event_count, ...
    'alignment_info', alignment_info, ...
    'options', options);
fprintf('前向 EKF 结果：%s\n', forward_path);
fprintf('相位裁决诊断：%s\n', diagnostic_path);
if options.enable_smoothing
    fprintf('一次 RTS 结果：%s\n', single_rts_path);
end
if options.enable_smoothing && options.enable_second_rts
    fprintf('二次 RTS 结果：%s\n', double_rts_path);
end

%% ------ 辅助函数 -----
function [rangedata, beacon_id, phase_measurement_deg] = ...
    build_measurement_events(range_sources, phase_sources, options)
source_dt = median(diff(range_sources{1}(:, 1)));
range_step = round(options.range_interval_s / source_dt);
if abs(range_step * source_dt - options.range_interval_s) > 1e-6
    error('测距间隔 %.3f s 不是源间隔 %.3f s 的整数倍。', ...
        options.range_interval_s, source_dt);
end
for index = 1:numel(range_sources)
    range_sources{index} = range_sources{index}(range_step:range_step:end, :);
end
event_count = min(cellfun(@(data) size(data, 1), range_sources));
rangedata = zeros(event_count, size(range_sources{1}, 2));
beacon_id = zeros(event_count, 1);
phase_measurement_deg = zeros(event_count, 1);

if isempty(options.phase_time_tolerance_s)
    phase_dt = median(diff(phase_sources{1}(:, 1)));
    phase_tolerance = max(0.51 * phase_dt, 1e-6);
else
    phase_tolerance = options.phase_time_tolerance_s;
end
for event_index = 1:event_count
    order_index = mod(event_index - 1, numel(options.beacon_order)) + 1;
    source_index = options.beacon_order(order_index);
    rangedata(event_index, :) = range_sources{source_index}(event_index, :);
    beacon_id(event_index) = source_index;

    phase_data = phase_sources{source_index};
    [time_error, phase_index] = min(abs( ...
        phase_data(:, 1) - rangedata(event_index, 1)));
    if time_error > phase_tolerance
        error('Beacon %d 在 %.3f s 附近没有同步相位量测。', ...
            source_index, rangedata(event_index, 1));
    end
    phase_measurement_deg(event_index) = phase_data(phase_index, 2);
end
end

function row = state_to_nav_row(navstate, param)
row = zeros(1, 11);
row(2) = navstate.time;
row(3:5) = [navstate.pos(1:2)' * param.R2D, navstate.pos(3)];
row(6:8) = navstate.vel';
row(9:11) = navstate.att' * param.R2D;
end

function write_nav_file(file_path, nav_data)
file_id = fopen(file_path, 'w');
if file_id < 0
    error('无法打开导航结果文件：%s', file_path);
end
cleaner = onCleanup(@() fclose(file_id)); 
format = ['%2d %12.6f %12.8f %12.8f %8.4f %8.4f ', ...
    '%8.4f %8.4f %8.4f %8.4f %8.4f\n'];
fprintf(file_id, format, nav_data');
end
