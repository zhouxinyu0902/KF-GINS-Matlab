clear;
close all;
clc;
%% 双水听器测距/测向与仅测距导航对比：前向 EKF 与可选 RTS
% 流程：距离预更新（仅用于候选角裁决） -> 相位差候选角 -> 距离+方位角联合更新。
% 预更新在 kf/navstate 的副本上执行，正式滤波只进行一次联合量测更新，
% 因此同一条距离量测不会被重复计入 EKF。
% 导航事件架构与原 range/INS 主链一致，并在每个测距历元：
%     1) 用距离更新副本得到候选角裁决先验；
%     2) 从同步相位差生成候选角并选择完整相对方位角；
%     3) 组装 1x9 距离+方位角观测；
%     4) 调用 update_range_azimuth_filter_rad 和配套姿态反馈；
%     5) 根据开关执行一次分段 RTS 和二次跨区间 RTS。
% data_source = "simulation";          % "simulation" 或 "experiment"
% case_name = 'case-00';
data_source = "experiment";          % "simulation" 或 "experiment"
case_name = 'case-06';
% phasestd = 5;
% azistd = 0.4;
phasestd = 10;
azistd = 0.8;
% phasestd = 15;
% azistd = 1.2;
name = sprintf('double-hydrophone-phasestd%ddeg',phasestd);
name1 = sprintf('phase-%ddeg',phasestd);
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
    'carrier_hz', 2e3, ...
    'sound_speed_mps', 1500, ...
    'filter_azimuth_std_deg', azistd, ... % 需要更改的参数
    'phase_time_tolerance_s', [], ...
    'enable_measurement_accuracy', true, ...
    'enable_angle_visualization', true, ...
    'show_angle_figure', true, ...
    'enable_smoothing', true, ...
    'enable_second_rts', true);
%% 路径配置
case_name = char(case_name);
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));
addpath(topic_dir);
addpath(script_dir);
addpath(fullfile(script_dir, 'function'));
paths = setup_inertial_experiment();
param = Param();
glvs;
rng(options.random_seed, 'twister');

data_source = lower(data_source);
id = sscanf(case_name, 'case-%d', 1);
if isempty(id) || ~isscalar(id) || id < 0 || id ~= fix(id)
    error('case_name 必须采用 case-XX 格式，例如 case-00 或 case-06。');
end
if data_source == "simulation"
    input_dir = paths.simulation_input(id);
    cfg = load_algorithm_exploration_config('simulation', 'rad', input_dir);
    result_dir = fullfile(paths.simulation_navigation(id), ...
        'navigation-results', name);
    range_noise_std_m = options.simulation_range_noise_std_m;
    filter_range_std_m = options.filter_range_std_m;
elseif data_source == "experiment"
    input_dir = paths.experiment_input(id);
    cfg = load_algorithm_exploration_config('experiment', 'rad', input_dir);
    cfg.inputfolder = input_dir;
    cfg.preprocessedfolder = input_dir;
    cfg.referencefolder = input_dir;
    cfg.imufilepath = fullfile(input_dir, 'IMU_120.txt');
    cfg.rangefile1path = fullfile(input_dir, 'range1.txt');
    cfg.rangefile2path = fullfile(input_dir, 'range2.txt');
    cfg.rangefile3path = fullfile(input_dir, 'range3.txt');
    cfg.truthpath = fullfile(input_dir, 'truth.nav');
    result_dir = fullfile(paths.experiment_navigation(id), ...
        'navigation-results', name);
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
phasedir = fullfile(input_dir,name1);
phase_paths = arrayfun(@(index) fullfile(phasedir, ...
    sprintf('phase%d.txt', index)), 1:3, 'UniformOutput', false);
if any(~cellfun(@isfile, phase_paths))
    missing = phase_paths(~cellfun(@isfile, phase_paths));
    error('找不到相位文件：%s', strjoin(missing, ', '));
end

range_sources = cellfun(@(path) readmatrix(path, 'FileType', 'text'), ...
    range_paths, 'UniformOutput', false);
phase_sources = cellfun(@(path) readmatrix(path, 'FileType', 'text'), ...
    phase_paths, 'UniformOutput', false);
% if data_source == "experiment"
%     height_path = fullfile(input_dir, 'height_noised.txt');
%     if ~isfile(height_path)
%         error('找不到实验高度文件：%s', height_path);
%     end
%     height_source = readmatrix(height_path, 'FileType', 'text');
% else
%     height_source = [];
% end
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

%% 量测真值：用于评估候选方位角选择和水平测距精度
truth_relative_azimuth_deg = nan(size(rangedata, 1), 1);
truth_horizontal_range_m = nan(size(rangedata, 1), 1);
if options.enable_measurement_accuracy || options.enable_angle_visualization
    if size(truth_all, 2) < 11
        error('量测精度评估要求 truth 文件至少包含 11 列。');
    end
    truth_valid = all(isfinite(truth_all(:, [2, 3, 4, 11])), 2);
    truth_reference = truth_all(truth_valid, :);
    [truth_time, truth_unique_index] = unique( ...
        truth_reference(:, 2), 'stable');
    truth_reference = truth_reference(truth_unique_index, :);
    if numel(truth_time) < 2
        error('truth 文件中的有效时间点不足，无法评估量测精度。');
    end

    truth_lat_rad = deg2rad(interp1(truth_time, ...
        truth_reference(:, 3), rangedata(:, 1), 'linear'));
    truth_lon_rad = deg2rad(interp1(truth_time, ...
        truth_reference(:, 4), rangedata(:, 1), 'linear'));
    truth_yaw_rad = interp1(truth_time, ...
        unwrap(deg2rad(truth_reference(:, 11))), ...
        rangedata(:, 1), 'linear');

    beacon_lat_rad = rangedata(:, 4);
    beacon_lon_rad = rangedata(:, 5);
    delta_lon_rad = beacon_lon_rad - truth_lon_rad;
    bearing_east = cos(beacon_lat_rad) .* sin(delta_lon_rad);
    bearing_north = cos(truth_lat_rad) .* sin(beacon_lat_rad) - ...
        sin(truth_lat_rad) .* cos(beacon_lat_rad) .* cos(delta_lon_rad);
    truth_bearing_deg = mod(atan2d(bearing_east, bearing_north), 360);
    truth_relative_azimuth_deg = mod(truth_bearing_deg - ...
        rad2deg(truth_yaw_rad) - options.baseline_install_deg - 90 + ...
        180, 360) - 180;

    for event_index = 1:size(rangedata, 1)
        beacon = rangedata(event_index, 4:6)';
        [rm, rn] = getRmRn(beacon(1), param);
        delta_north_m = (truth_lat_rad(event_index) - beacon(1)) * ...
            (rm + beacon(3));
        delta_east_m = (truth_lon_rad(event_index) - beacon(2)) * ...
            (rn + beacon(3)) * cos(beacon(1));
        truth_horizontal_range_m(event_index) = hypot( ...
            delta_north_m, delta_east_m);
    end
end

% if data_source == "simulation"
    height_values = interp1(truth_all(:, 2), truth_all(:, 5), ...
        imudata(:, 1), 'linear', 'extrap');
    height_values = height_values + options.depth_noise_std_m * ...
        randn(size(height_values));
% else
    % height_values = interp1(height_source(:, 1), height_source(:, 2), ...
    %     imudata(:, 1), 'linear', 'extrap');
% end
height = [imudata(:, 1), height_values];
[imudata, height, alignment_info] = ...
    align_imu_to_range_epochs(imudata, height, rangedata(:, 1));
%% 分别运行测距+测向与仅测距滤波链
outputs = struct();
% use_azimuth_modes = [true, false];
use_azimuth_modes = true;
for chain_index = 1:numel(use_azimuth_modes)
use_azimuth = use_azimuth_modes(chain_index);
if use_azimuth
    result_prefix = 'range-phase-azimuth';
    fprintf('\n开始测距+测向导航。\n');
else
    result_prefix = 'range-only';
    fprintf('\n开始仅测距导航。\n');
end

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
% [time, beacon, truth azimuth, selected azimuth, signed/absolute azimuth
%  error, correct candidate flag, measured/truth horizontal range,
%  signed/absolute range error]
measurement_accuracy_data = nan(size(rangedata, 1), 11);
candidate_angle_data = cell(size(rangedata, 1), 1);
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
        if use_azimuth
            [joint_range, selection] = ...
                build_range_azimuth_measurement_from_phase( ...
                navstate, kf, current_range, current_depth, ...
                phase_measurement_deg(range_index), options);

            % 正式滤波只执行本次联合更新，预更新副本不会写回。
            kf = update_range_azimuth_filter_rad( ...
                navstate, joint_range, current_depth, kf, ...
                options.filter_azimuth_std_deg);
        else
            kf = myRangeUpdate(navstate, current_range, current_depth, kf);
        end

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

        if use_azimuth
            [kf, navstate] = feedback_range_azimuth_state(kf, navstate);
        else
            [kf, navstate] = myErrorFeedback_range(kf, navstate);
        end

        used_event_count = used_event_count + 1;
        if use_azimuth
            diagnostic_data(used_event_count, :) = [ ...
                current_range(1), range_beacon_id(range_index), ...
                selection.predicted_relative_azimuth_deg, ...
                selection.selected_relative_azimuth_deg, ...
                selection.selected_difference_deg];
            candidate_angle_data{used_event_count} = ...
                selection.candidate_full_azimuth_deg(:);
            if options.enable_measurement_accuracy || ...
                    options.enable_angle_visualization
                true_azimuth_deg = ...
                    truth_relative_azimuth_deg(range_index);
                azimuth_error_deg = mod( ...
                    selection.selected_relative_azimuth_deg - ...
                    true_azimuth_deg + 180, 360) - 180;
                candidate_truth_difference_deg = abs(mod( ...
                    selection.candidate_full_azimuth_deg - ...
                    true_azimuth_deg + 180, 360) - 180);
                [~, truth_candidate_index] = min( ...
                    candidate_truth_difference_deg);
                correct_candidate = abs(mod( ...
                    selection.selected_relative_azimuth_deg - ...
                    selection.candidate_full_azimuth_deg( ...
                    truth_candidate_index) + 180, 360) - 180) < 1e-8;
                range_error_m = current_range(3) - ...
                    truth_horizontal_range_m(range_index);
                measurement_accuracy_data(used_event_count, :) = [ ...
                    current_range(1), range_beacon_id(range_index), ...
                    true_azimuth_deg, ...
                    selection.selected_relative_azimuth_deg, ...
                    azimuth_error_deg, abs(azimuth_error_deg), ...
                    double(correct_candidate), current_range(3), ...
                    truth_horizontal_range_m(range_index), ...
                    range_error_m, abs(range_error_m)];
            end
        end
        forward_nav(imu_index - 1, :) = state_to_nav_row(navstate, param);

        if use_azimuth && (options.enable_measurement_accuracy || ...
                options.enable_angle_visualization)
            fprintf(['Event %2d | t=%9.3f s | B%d | phase=%8.3f deg | ' ...
                'prior=%8.3f deg | selected=%8.3f deg | ' ...
                'k=%3d | diff=%6.3f deg | correct=%d\n'], ...
                used_event_count, current_range(1), ...
                range_beacon_id(range_index), ...
                phase_measurement_deg(range_index), ...
                selection.predicted_relative_azimuth_deg, ...
                selection.selected_relative_azimuth_deg, ...
                selection.selected_cycle_k, ...
                selection.selected_difference_deg, ...
                measurement_accuracy_data(used_event_count, 7));
        elseif use_azimuth
            fprintf(['Event %2d | t=%9.3f s | B%d | phase=%8.3f deg | ' ...
                'prior=%8.3f deg | selected=%8.3f deg | ' ...
                'k=%3d | diff=%6.3f deg\n'], ...
                used_event_count, current_range(1), ...
                range_beacon_id(range_index), ...
                phase_measurement_deg(range_index), ...
                selection.predicted_relative_azimuth_deg, ...
                selection.selected_relative_azimuth_deg, ...
                selection.selected_cycle_k, ...
                selection.selected_difference_deg);
        else
            fprintf('Event %2d | t=%9.3f s | B%d | range-only\n', ...
                used_event_count, current_range(1), ...
                range_beacon_id(range_index));
        end
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
measurement_accuracy_data = ...
    measurement_accuracy_data(1:used_event_count, :);
candidate_angle_data = candidate_angle_data(1:used_event_count);

% 最后一段没有未来桥接误差，二次 RTS 退化为该段的一次 RTS。
if options.enable_smoothing && options.enable_second_rts && ...
        ~isempty(previous_single_nav)
    double_rts_nav = [double_rts_nav, previous_single_nav]; %#ok<AGROW>
end
%% 输出
forward_path = fullfile(result_dir, ...
    sprintf('%s-forward.nav', result_prefix));
write_nav_file(forward_path, forward_nav);

diagnostic_path = '';
measurement_accuracy_path = '';
angle_visualization_path = '';
angle_figure_path = '';
if use_azimuth
    diagnostic_path = fullfile(result_dir, ...
        sprintf('%s-diagnostic.txt', result_prefix));
    diagnostic_fp = fopen(diagnostic_path, 'wt');
    if diagnostic_fp < 0
        error('无法创建方位角诊断文件：%s', diagnostic_path);
    end
    fprintf(diagnostic_fp, ['time_s beacon_id prior_azimuth_deg ' ...
        'selected_azimuth_deg prior_selected_difference_deg\n']);
    fprintf(diagnostic_fp, '%12.6f %2d %12.6f %12.6f %12.6f\n', ...
        diagnostic_data');
    fclose(diagnostic_fp);

    if options.enable_measurement_accuracy
        measurement_accuracy_path = fullfile(result_dir, ...
            sprintf('%s-measurement-accuracy.txt', result_prefix));
        accuracy_fp = fopen(measurement_accuracy_path, 'wt');
        if accuracy_fp < 0
            error('无法创建量测精度评估文件：%s', ...
                measurement_accuracy_path);
        end
        fprintf(accuracy_fp, ['# 方位角误差采用 selected - truth，并包裹到 ' ...
            '[-180, 180) deg；测距误差采用 measured - truth。\n']);
        fprintf(accuracy_fp, ['# candidate_correct=1 表示先验选中的候选角，' ...
            '也是全部候选中最接近真实方位角的候选。\n']);
        fprintf(accuracy_fp, ['# summary\n# scope count candidate_correct_pct ' ...
            'az_bias_deg az_std_deg az_mae_deg az_rmse_deg ' ...
            'az_p95_abs_deg az_max_abs_deg range_bias_m range_std_m ' ...
            'range_mae_m range_rmse_m range_p95_abs_m range_max_abs_m\n']);

        summary_scopes = [0; unique(measurement_accuracy_data(:, 2))];
        for summary_index = 1:numel(summary_scopes)
            scope_beacon = summary_scopes(summary_index);
            if scope_beacon == 0
                scope_mask = true(size(measurement_accuracy_data, 1), 1);
                scope_name = 'ALL';
            else
                scope_mask = measurement_accuracy_data(:, 2) == ...
                    scope_beacon;
                scope_name = sprintf('B%d', scope_beacon);
            end
            scope_data = measurement_accuracy_data(scope_mask, :);
            valid_scope = all(isfinite(scope_data(:, [5, 7, 10])), 2);
            scope_data = scope_data(valid_scope, :);
            azimuth_error = scope_data(:, 5);
            range_error = scope_data(:, 10);
            azimuth_abs_sorted = sort(abs(azimuth_error));
            range_abs_sorted = sort(abs(range_error));
            p95_index = max(1, ceil(0.95 * size(scope_data, 1)));
            fprintf(accuracy_fp, ...
                ['%-4s %4d %10.3f %12.6f %12.6f %12.6f ' ...
                '%12.6f %12.6f %12.6f %12.6f %12.6f %12.6f ' ...
                '%12.6f %12.6f %12.6f\n'], ...
                scope_name, size(scope_data, 1), ...
                100 * mean(scope_data(:, 7)), mean(azimuth_error), ...
                std(azimuth_error), mean(abs(azimuth_error)), ...
                sqrt(mean(azimuth_error .^ 2)), ...
                azimuth_abs_sorted(p95_index), ...
                max(azimuth_abs_sorted), mean(range_error), ...
                std(range_error), mean(abs(range_error)), ...
                sqrt(mean(range_error .^ 2)), ...
                range_abs_sorted(p95_index), max(range_abs_sorted));
        end

        fprintf(accuracy_fp, ['# events\ntime_s beacon_id ' ...
            'truth_azimuth_deg selected_azimuth_deg azimuth_error_deg ' ...
            'azimuth_abs_error_deg candidate_correct ' ...
            'measured_horizontal_range_m truth_horizontal_range_m ' ...
            'range_error_m range_abs_error_m\n']);
        fprintf(accuracy_fp, ...
            ['%12.6f %2d %12.6f %12.6f %12.6f %12.6f %1d ' ...
            '%14.6f %14.6f %12.6f %12.6f\n'], ...
            measurement_accuracy_data');
        fclose(accuracy_fp);
    end

    if options.enable_measurement_accuracy || ...
            options.enable_angle_visualization
        correct_mask = measurement_accuracy_data(:, 7) == 1;
        correct_rate_pct = 100 * mean(correct_mask);
        fprintf('候选角选择：%d/%d 正确（%.2f%%）。\n', ...
            nnz(correct_mask), numel(correct_mask), correct_rate_pct);
        wrong_index = find(~correct_mask);
        for wrong_list_index = 1:numel(wrong_index)
            row_index = wrong_index(wrong_list_index);
            fprintf(['  选错：Event %d | t=%.3f s | B%d | ' ...
                'truth=%.3f deg | selected=%.3f deg | error=%.3f deg\n'], ...
                row_index, measurement_accuracy_data(row_index, 1), ...
                measurement_accuracy_data(row_index, 2), ...
                measurement_accuracy_data(row_index, 3), ...
                measurement_accuracy_data(row_index, 4), ...
                measurement_accuracy_data(row_index, 5));
        end
    end

    if options.enable_angle_visualization
        angle_visualization_path = fullfile(result_dir, ...
            sprintf('%s-angle-selection.png', result_prefix));
        angle_figure_path = fullfile(result_dir, ...
            sprintf('%s-angle-selection.fig', result_prefix));
        figure_visibility = 'off';
        if options.show_angle_figure
            figure_visibility = 'on';
        end
        % angle_figure = figure('Color', 'w', 'Visible', ...
        %     figure_visibility, 'Position', [100, 80, 1200, 850]);
        angle_figure = myfigurestartup(7,7,'zxy');
        angle_layout = tiledlayout(angle_figure, 3, 1, ...
            'TileSpacing', 'compact', 'Padding', 'compact');
        event_axis = (1:size(measurement_accuracy_data, 1))';
        wrong_mask = measurement_accuracy_data(:, 7) == 0;

        angle_axis = nexttile(angle_layout, 1);
        hold(angle_axis, 'on');
        grid(angle_axis, 'on');
        box(angle_axis, 'on');
        for event_index = 1:numel(candidate_angle_data)
            candidates = candidate_angle_data{event_index};
            plot(angle_axis, event_index * ones(size(candidates)), ...
                candidates, '.', 'Color', [0.72, 0.72, 0.72], ...
                'MarkerSize', 7, 'HandleVisibility', 'off');
        end
        candidate_handle = plot(angle_axis, nan, nan, '.', ...
            'Color', [0.72, 0.72, 0.72], 'MarkerSize', 9, ...
            'DisplayName', '全部候选角');
        truth_handle = plot(angle_axis, event_axis, ...
            measurement_accuracy_data(:, 3), 'k-', 'LineWidth', 1.3, ...
            'DisplayName', '真实相对方位角');
        prior_handle = plot(angle_axis, event_axis, diagnostic_data(:, 3), ...
            'b--', 'LineWidth', 1.1, 'DisplayName', '先验方位角');
        selected_handle = plot(angle_axis, event_axis, ...
            measurement_accuracy_data(:, 4), 'r.-', 'LineWidth', 1.0, ...
            'MarkerSize', 12, 'DisplayName', '选择方位角');
        wrong_handle = plot(angle_axis, event_axis(wrong_mask), ...
            measurement_accuracy_data(wrong_mask, 4), 'mx', ...
            'LineWidth', 1.8, 'MarkerSize', 10, ...
            'DisplayName', '选错候选角');
        ylim(angle_axis, [-180, 180]);
        ylabel(angle_axis, '方位角 / deg');
        title(angle_axis, sprintf( ...
            '%s / %s：候选角裁决，正确率 %.2f%%', ...
            char(data_source), case_name, correct_rate_pct));
        legend(angle_axis, [candidate_handle, truth_handle, prior_handle, ...
            selected_handle, wrong_handle], 'Location', 'northeast');

        error_axis = nexttile(angle_layout, 2);
        hold(error_axis, 'on');
        grid(error_axis, 'on');
        box(error_axis, 'on');
        plot(error_axis, event_axis, measurement_accuracy_data(:, 5), ...
            '.-', 'Color', [0.10, 0.45, 0.75], 'LineWidth', 1.0, ...
            'MarkerSize', 11);
        yline(error_axis, 0, 'k--');
        plot(error_axis, event_axis(wrong_mask), ...
            measurement_accuracy_data(wrong_mask, 5), 'rx', ...
            'LineWidth', 1.8, 'MarkerSize', 10);
        ylabel(error_axis, '选择误差 / deg');
        title(error_axis, '选择方位角 - 真实方位角');

        correct_axis = nexttile(angle_layout, 3);
        stem(correct_axis, event_axis, double(correct_mask), 'filled', ...
            'LineWidth', 1.0, 'MarkerSize', 4);
        grid(correct_axis, 'on');
        box(correct_axis, 'on');
        ylim(correct_axis, [-0.1, 1.1]);
        yticks(correct_axis, [0, 1]);
        yticklabels(correct_axis, {'错误', '正确'});
        xlabel(correct_axis, '联合量测事件序号');
        ylabel(correct_axis, '候选裁决');
        title(correct_axis, sprintf('正确 %d，错误 %d', ...
            nnz(correct_mask), nnz(wrong_mask)));

        exportgraphics(angle_figure, angle_visualization_path, ...
            'Resolution', 300);
        savefig(angle_figure, angle_figure_path);
        if ~options.show_angle_figure
            close(angle_figure);
        end
    end
end

single_rts_path = '';
double_rts_path = '';
if options.enable_smoothing
    single_rts_path = fullfile(result_dir, ...
        sprintf('%s-rts-single.nav', result_prefix));
    write_nav_file(single_rts_path, single_rts_nav');
end
if options.enable_smoothing && options.enable_second_rts
    double_rts_path = fullfile(result_dir, ...
        sprintf('%s-rts-double.nav', result_prefix));
    write_nav_file(double_rts_path, double_rts_nav');
end

chain_output = struct( ...
    'data_source', char(data_source), ...
    'case_name', case_name, ...
    'input_dir', input_dir, ...
    'result_dir', result_dir, ...
    'forward_path', forward_path, ...
    'diagnostic_path', diagnostic_path, ...
    'measurement_accuracy_path', measurement_accuracy_path, ...
    'angle_visualization_path', angle_visualization_path, ...
    'angle_figure_path', angle_figure_path, ...
    'single_rts_path', single_rts_path, ...
    'double_rts_path', double_rts_path, ...
    'event_count', used_event_count, ...
    'alignment_info', alignment_info, ...
    'options', options);
if use_azimuth
    outputs.has_azimuth = chain_output;
else
    outputs.range_only = chain_output;
end
fprintf('前向 EKF 结果：%s\n', forward_path);
if use_azimuth
    fprintf('相位裁决诊断：%s\n', diagnostic_path);
    if options.enable_measurement_accuracy
        fprintf('量测精度评估：%s\n', measurement_accuracy_path);
    end
    if options.enable_angle_visualization
        fprintf('测角可视化：%s\n', angle_visualization_path);
        fprintf('MATLAB 图文件：%s\n', angle_figure_path);
    end
end
if options.enable_smoothing
    fprintf('一次 RTS 结果：%s\n', single_rts_path);
end
if options.enable_smoothing && options.enable_second_rts
    fprintf('二次 RTS 结果：%s\n', double_rts_path);
end
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
