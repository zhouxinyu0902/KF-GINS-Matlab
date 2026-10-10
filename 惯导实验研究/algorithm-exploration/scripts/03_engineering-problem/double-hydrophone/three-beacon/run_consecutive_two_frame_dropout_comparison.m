% clear;
close all;
clc;
%% 连续丢两帧：仅测距与测距+测向的前向 EKF、一次 RTS、二次 RTS 对比
% 默认测试 simulation / case-00 / Phase STD = 5°，连续删除第 3、4 帧。
% 可在运行前覆盖 data_source、case_name、phasestd、azistd、
% drop_measurement_indices 和 attitude_feedback_mode。
data_source = "experiment";
case_name = 'case-06';
if ~exist('data_source', 'var') || isempty(data_source)
    data_source = "simulation";
end
if ~exist('case_name', 'var') || isempty(case_name)
    case_name = 'case-00';
end
if ~exist('phasestd', 'var') || isempty(phasestd)
    phasestd = 5;
end
if ~exist('azistd', 'var') || isempty(azistd)
    azistd = 0.4 * phasestd / 5;
end
if ~exist('drop_measurement_indices', 'var') || ...
        isempty(drop_measurement_indices)
    drop_measurement_indices = [3, 4];
end
if ~exist('attitude_feedback_mode', 'var') || ...
        isempty(attitude_feedback_mode)
    attitude_feedback_mode = "heading-only";
end
if ~exist('show_result_figure', 'var') || isempty(show_result_figure)
    show_result_figure = true;
end

range_interval_s = 420;
end_time_s = 4621;
beacon_order = [1, 2, 3];
random_seed = 1;
simulation_range_noise_std_m = 10;
experiment_range_noise_std_m = 6;
depth_noise_std_m = 0.4;
filter_depth_std_m = 0.4;
analysis_sample_interval_s = 1;

drop_measurement_indices = sort(drop_measurement_indices(:)');
if numel(drop_measurement_indices) ~= 2 || ...
        diff(drop_measurement_indices) ~= 1
    error('drop_measurement_indices 必须是两个连续帧编号，例如 [3, 4]。');
end

attitude_feedback_mode = lower(string(attitude_feedback_mode));
switch attitude_feedback_mode
    case "full"
        match_range_only_feedback = false;
        feedback_heading_only = false;
        feedback_attitude = true;
    case "heading-only"
        match_range_only_feedback = true;
        feedback_heading_only = true;
        feedback_attitude = false;
    case "none"
        match_range_only_feedback = true;
        feedback_heading_only = false;
        feedback_attitude = false;
    otherwise
        error(['attitude_feedback_mode 只能设置为 "full"、' ...
            '"heading-only" 或 "none"。']);
end

name = sprintf('double-hydrophone-phasestd%ddeg', phasestd);
phase_name = sprintf('phase-%ddeg', phasestd);
drop_tag = sprintf('drop%d-%d', drop_measurement_indices);
phase_options = struct( ...
    'baseline_m', 3.0, ...
    'baseline_install_deg', 0, ...
    'carrier_hz', 2e3, ...
    'sound_speed_mps', 1500, ...
    'filter_azimuth_std_deg', azistd, ...
    'match_range_only_feedback', match_range_only_feedback, ...
    'feedback_heading_only', feedback_heading_only, ...
    'estimate_attitude_error', true, ...
    'feedback_attitude', feedback_attitude, ...
    'estimate_imu_bias_error', true, ...
    'feedback_imu_bias', true);

%% 路径与输入
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));
function_dir = fullfile(fileparts(script_dir), 'function');
addpath(topic_dir);
addpath(script_dir);
addpath(function_dir);
paths = setup_inertial_experiment();
param = Param();
glvs;

data_source = lower(string(data_source));
case_name = char(case_name);
case_id = sscanf(case_name, 'case-%d', 1);
if isempty(case_id) || ~isscalar(case_id) || ...
        case_id < 0 || case_id ~= fix(case_id)
    error('case_name 必须采用 case-XX 格式，例如 case-00 或 case-06。');
end
if data_source == "simulation"
    input_dir = paths.simulation_input(case_id);
    cfg = load_algorithm_exploration_config( ...
        'simulation', 'rad', input_dir);
    navigation_dir = paths.simulation_navigation(case_id);
    range_noise_std_m = simulation_range_noise_std_m;
elseif data_source == "experiment"
    input_dir = paths.experiment_input(case_id);
    cfg = load_algorithm_exploration_config( ...
        'experiment', 'rad', input_dir);
    cfg.inputfolder = input_dir;
    cfg.preprocessedfolder = input_dir;
    cfg.referencefolder = input_dir;
    cfg.imufilepath = fullfile(input_dir, 'IMU_120.txt');
    cfg.rangefile1path = fullfile(input_dir, 'range1.txt');
    cfg.rangefile2path = fullfile(input_dir, 'range2.txt');
    cfg.rangefile3path = fullfile(input_dir, 'range3.txt');
    cfg.truthpath = fullfile(input_dir, 'truth.nav');
    navigation_dir = paths.experiment_navigation(case_id);
    range_noise_std_m = experiment_range_noise_std_m;
else
    error('data_source 只能设置为 "simulation" 或 "experiment"。');
end
filter_range_std_m = range_noise_std_m;
cfg.userange = true;
output_dir = fullfile(navigation_dir, 'navigation-results', name, ...
    'drop-consecutive-two');
if ~isfolder(output_dir)
    mkdir(output_dir);
end

imu_all = readmatrix(cfg.imufilepath, 'FileType', 'text');
truth_all = readmatrix(cfg.truthpath, 'FileType', 'text');
range_paths = {cfg.rangefile1path, cfg.rangefile2path, ...
    cfg.rangefile3path};
range_sources = cellfun(@(path) readmatrix(path, 'FileType', 'text'), ...
    range_paths, 'UniformOutput', false);
phase_dir = fullfile(input_dir, phase_name);
phase_paths = arrayfun(@(index) fullfile(phase_dir, ...
    sprintf('phase%d.txt', index)), 1:3, 'UniformOutput', false);
if any(~cellfun(@isfile, phase_paths))
    missing = phase_paths(~cellfun(@isfile, phase_paths));
    error('找不到相位文件：%s', strjoin(missing, ', '));
end
phase_sources = cellfun(@(path) readmatrix(path, 'FileType', 'text'), ...
    phase_paths, 'UniformOutput', false);

%% 构造完整联合量测事件，再同时删除连续两帧
source_interval_s = median(diff(range_sources{1}(:, 1)));
range_step = round(range_interval_s / source_interval_s);
if abs(range_step * source_interval_s - range_interval_s) > 1e-6
    error('测距间隔 %.3f s 不是源间隔 %.3f s 的整数倍。', ...
        range_interval_s, source_interval_s);
end
for source_index = 1:numel(range_sources)
    range_sources{source_index} = ...
        range_sources{source_index}(range_step:range_step:end, :);
end

event_count = min(cellfun(@(data) size(data, 1), range_sources));
if any(drop_measurement_indices < 1) || ...
        any(drop_measurement_indices > event_count)
    error('丢帧编号必须位于 1 到 %d。', event_count);
end
base_rangedata = zeros(event_count, size(range_sources{1}, 2));
base_beacon_id = zeros(event_count, 1);
base_phase_measurement_deg = zeros(event_count, 1);
phase_source_interval_s = median(diff(phase_sources{1}(:, 1)));
phase_time_tolerance_s = max(0.51 * phase_source_interval_s, 1e-6);
for event_index = 1:event_count
    order_index = mod(event_index - 1, numel(beacon_order)) + 1;
    source_index = beacon_order(order_index);
    base_rangedata(event_index, :) = ...
        range_sources{source_index}(event_index, :);
    base_beacon_id(event_index) = source_index;
    phase_data = phase_sources{source_index};
    [phase_time_error_s, phase_index] = min(abs( ...
        phase_data(:, 1) - base_rangedata(event_index, 1)));
    if phase_time_error_s > phase_time_tolerance_s
        error('Beacon %d 在 %.3f s 附近没有同步相位量测。', ...
            source_index, base_rangedata(event_index, 1));
    end
    base_phase_measurement_deg(event_index) = phase_data(phase_index, 2);
end

rng(random_seed, 'twister');
base_rangedata(:, 3) = base_rangedata(:, 3) + ...
    range_noise_std_m * randn(size(base_rangedata, 1), 1);
depth_rng_state = rng;

keep_event = true(event_count, 1);
keep_event(drop_measurement_indices) = false;
dropped_time_s = base_rangedata(drop_measurement_indices, 1);
dropped_beacon_id = base_beacon_id(drop_measurement_indices);
rangedata = base_rangedata(keep_event, :);
range_beacon_id = base_beacon_id(keep_event);
original_event_id = find(keep_event);
phase_measurement_deg = base_phase_measurement_deg(keep_event);

start_time = max([cfg.starttime, imu_all(1, 1), truth_all(1, 2)]);
end_time = min([start_time + end_time_s, cfg.endtime, ...
    imu_all(end, 1), truth_all(end, 2)]);
cfg.starttime = start_time;
cfg.endtime = end_time;
imu_mask = imu_all(:, 1) >= start_time & imu_all(:, 1) <= end_time;
imudata = imu_all(imu_mask, :);
range_mask = rangedata(:, 1) >= start_time & rangedata(:, 1) <= end_time;
rangedata = rangedata(range_mask, :);
range_beacon_id = range_beacon_id(range_mask);
original_event_id = original_event_id(range_mask);
phase_measurement_deg = phase_measurement_deg(range_mask);
if isempty(rangedata)
    error('删除第 %d、%d 帧后没有剩余联合量测。', ...
        drop_measurement_indices);
end

rng(depth_rng_state);
height_values = interp1(truth_all(:, 2), truth_all(:, 5), ...
    imudata(:, 1), 'linear', 'extrap');
height_values = height_values + depth_noise_std_m * ...
    randn(size(height_values));
height = [imudata(:, 1), height_values];
[imudata, height, alignment_info] = align_imu_to_range_epochs( ...
    imudata, height, rangedata(:, 1));

%% 计算保留量测的真实相对方位，用于检查长时间断档后的候选角裁决
truth_reference = truth_all(all(isfinite( ...
    truth_all(:, [2, 3, 4, 11])), 2), :);
[truth_time, truth_unique_index] = unique( ...
    truth_reference(:, 2), 'stable');
truth_reference = truth_reference(truth_unique_index, :);
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
    rad2deg(truth_yaw_rad) - phase_options.baseline_install_deg - 90 + ...
    180, 360) - 180;

fprintf(['\n%s / %s / Phase STD=%g°：连续丢第 %d、%d 帧，' ...
    '对应时刻 %.1f、%.1f s，信标 %d、%d。\n'], ...
    data_source, case_name, phasestd, drop_measurement_indices, ...
    dropped_time_s, dropped_beacon_id);
fprintf('姿态反馈模式：%s；保留 %d 帧联合量测。\n', ...
    attitude_feedback_mode, size(rangedata, 1));

%% 分别运行仅测距、测距+测向两条滤波链
sample_count = size(imudata, 1);
imu_interval_s = median(diff(imudata(:, 1)));
maximum_event_gap_s = max([ ...
    rangedata(1, 1) - imudata(1, 1); ...
    diff(rangedata(:, 1)); ...
    imudata(end, 1) - rangedata(end, 1)]);
maximum_segment_samples = ...
    ceil(maximum_event_gap_s / imu_interval_s) + 20;
measurement_mode_names = {'range-only', 'range-azimuth'};
measurement_mode_labels = {'仅测距', '测距+测向'};
stage_names = {'Forward_EKF', 'Single_RTS', 'Double_RTS'};
stage_labels = {'前向 EKF', '一次 RTS', '二次 RTS'};
navigation_results = cell(6, 1);
navigation_result_paths = cell(6, 1);
selection_diagnostic = nan(size(rangedata, 1), 9);
used_event_count = 0;
scenario_timer = tic;

for measurement_mode_index = 1:2
    use_azimuth = measurement_mode_index == 2;
    measurement_mode_name = measurement_mode_names{measurement_mode_index};
    fprintf('\n开始%s连续掉帧计算。\n', ...
        measurement_mode_labels{measurement_mode_index});

    [kf, navstate] = myInitialize_15state(cfg);
    kf.rangstd = filter_range_std_m;
    kf.depthstd = filter_depth_std_m;
    forward_nav = nan(sample_count, 11);
    navstate.time = imudata(1, 1);
    forward_nav(1, :) = state_to_nav_row(navstate, param);

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
    single_rts_nav = zeros(11, 0);
    double_rts_nav = zeros(11, 0);

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
            abs(last_imu(1) - rangedata(range_index, 1)) <= ...
            time_tolerance;

        if is_range_epoch
            current_range = rangedata(range_index, :);
            current_depth = height(imu_index - 1, :);
            if use_azimuth
                [joint_range, selection] = ...
                    build_range_azimuth_measurement_from_phase( ...
                    navstate, kf, current_range, current_depth, ...
                    phase_measurement_deg(range_index), phase_options);
                kf = update_range_azimuth_filter_rad( ...
                    navstate, joint_range, current_depth, kf, ...
                    phase_options.filter_azimuth_std_deg, ...
                    phase_options.estimate_attitude_error, ...
                    phase_options.estimate_imu_bias_error);
            else
                kf = myRangeUpdate(navstate, current_range, ...
                    current_depth, kf);
            end

            terminal_error = kf.x;
            if use_azimuth && ~phase_options.feedback_attitude && ...
                    ~phase_options.match_range_only_feedback
                terminal_error = kf.last_measurement_state_correction;
                terminal_error(7:end) = 0;
            end

            if buffer_index > 1
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
                if use_azimuth && ~phase_options.feedback_attitude && ...
                        ~phase_options.match_range_only_feedback
                    bridge_error(7:end) = 0;
                end
                single_rts_nav = [single_rts_nav, single_nav]; %#ok<AGROW>
                if isempty(previous_single_state)
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
                    double_rts_nav = ...
                        [double_rts_nav, double_nav]; %#ok<AGROW>
                    previous_single_state = single_state;
                    previous_single_nav = single_nav;
                    previous_corrected_covariance = ...
                        current_corrected_covariance;
                    previous_predicted_covariance = ...
                        current_predicted_covariance;
                    previous_transition = current_transition;
                    previous_range_index = range_index;
                end
                buffer_index = 1;
                state_buffer(:) = 0;
                corrected_covariance_buffer(:) = 0;
                predicted_covariance_buffer(:) = 0;
                transition_buffer(:) = 0;
            end

            if use_azimuth
                if phase_options.match_range_only_feedback
                    if phase_options.feedback_heading_only
                        [kf, navstate] = feedback_range_azimuth_state( ...
                            kf, navstate, false, true, true);
                    else
                        [kf, navstate] = ...
                            myErrorFeedback_range(kf, navstate);
                    end
                else
                    [kf, navstate] = feedback_range_azimuth_state( ...
                        kf, navstate, phase_options.feedback_attitude, ...
                        phase_options.feedback_imu_bias);
                end
            else
                [kf, navstate] = myErrorFeedback_range(kf, navstate);
            end

            if use_azimuth
                used_event_count = used_event_count + 1;
                true_azimuth_deg = truth_relative_azimuth_deg(range_index);
                selected_error_deg = wrap_to_180( ...
                    selection.selected_relative_azimuth_deg - ...
                    true_azimuth_deg);
                candidate_truth_difference_deg = abs(wrap_to_180( ...
                    selection.candidate_full_azimuth_deg - ...
                    true_azimuth_deg));
                [~, truth_candidate_index] = min( ...
                    candidate_truth_difference_deg);
                candidate_correct = abs(wrap_to_180( ...
                    selection.selected_relative_azimuth_deg - ...
                    selection.candidate_full_azimuth_deg( ...
                    truth_candidate_index))) < 1e-8;
                selection_diagnostic(used_event_count, :) = [ ...
                    original_event_id(range_index), current_range(1), ...
                    range_beacon_id(range_index), true_azimuth_deg, ...
                    selection.predicted_relative_azimuth_deg, ...
                    selection.selected_relative_azimuth_deg, ...
                    selected_error_deg, ...
                    selection.selected_difference_deg, ...
                    double(candidate_correct)];
                fprintf(['保留原第 %2d 帧 | t=%7.1f s | B%d | ' ...
                    'prior=%8.3f° | selected=%8.3f° | ' ...
                    'error=%7.3f° | correct=%d\n'], ...
                    original_event_id(range_index), current_range(1), ...
                    range_beacon_id(range_index), ...
                    selection.predicted_relative_azimuth_deg, ...
                    selection.selected_relative_azimuth_deg, ...
                    selected_error_deg, candidate_correct);
            end
            forward_nav(imu_index - 1, :) = ...
                state_to_nav_row(navstate, param);
            range_index = range_index + 1;
        end

        last_state = navstate;
        navstate = InsMech(last_state, last_imu, this_imu);
        if ~is_range_epoch
            [kf, navstate] = update_decoupled_height( ...
                kf, navstate, height(imu_index, :));
        end
        if buffer_index > maximum_segment_samples
            error('RTS 缓存不足：%s / %s。', ...
                measurement_mode_name, drop_tag);
        end
        corrected_covariance_buffer(buffer_index, :) = kf.P(:)';
        kf = myInsPropagate_15state(navstate, this_imu, imu_dt, kf);
        state_buffer(buffer_index, :) = [navstate.time, ...
            navstate.pos', navstate.vel', navstate.att'];
        predicted_covariance_buffer(buffer_index, :) = kf.P(:)';
        transition_buffer(buffer_index, :) = kf.phi(:)';
        buffer_index = buffer_index + 1;
        forward_nav(imu_index, :) = state_to_nav_row(navstate, param);

        progress = floor(10 * imu_index / sample_count) * 10;
        if progress > last_progress && mod(progress, 20) == 0
            fprintf('%s / %s 处理进度：%d %%\n', ...
                measurement_mode_name, drop_tag, progress);
            last_progress = progress;
        end
    end

    if ~isempty(previous_single_nav)
        double_rts_nav = ...
            [double_rts_nav, previous_single_nav]; %#ok<AGROW>
    end
    mode_navigation_results = { ...
        forward_nav, single_rts_nav', double_rts_nav'};
    for stage_index = 1:3
        result_index = (measurement_mode_index - 1) * 3 + stage_index;
        navigation_results{result_index} = ...
            mode_navigation_results{stage_index};
        navigation_result_paths{result_index} = fullfile(output_dir, ...
            sprintf('%s-%s-%s.nav', measurement_mode_name, ...
            lower(strrep(stage_names{stage_index}, '_', '-')), drop_tag));
        write_nav_file(navigation_result_paths{result_index}, ...
            mode_navigation_results{stage_index});
    end
end

selection_diagnostic = selection_diagnostic(1:used_event_count, :);
selection_path = fullfile(output_dir, sprintf( ...
    'range-azimuth-selection-%s.txt', drop_tag));
selection_fp = fopen(selection_path, 'wt');
if selection_fp < 0
    error('无法创建候选角诊断文件：%s', selection_path);
end
fprintf(selection_fp, ['original_event_id time_s beacon_id ' ...
    'truth_azimuth_deg prior_azimuth_deg selected_azimuth_deg ' ...
    'selected_error_deg prior_selected_difference_deg ' ...
    'candidate_correct\n']);
fprintf(selection_fp, ...
    '%d %.6f %d %.9f %.9f %.9f %.9f %.9f %d\n', ...
    selection_diagnostic');
fclose(selection_fp);

%% 六组结果在共同时间范围内统一对比
result_count = numel(navigation_results);
result_names = { ...
    'RangeOnly_Forward_EKF', 'RangeOnly_Single_RTS', ...
    'RangeOnly_Double_RTS', 'RangeAzimuth_Forward_EKF', ...
    'RangeAzimuth_Single_RTS', 'RangeAzimuth_Double_RTS'};
result_labels = { ...
    '仅测距 前向', '仅测距 一次RTS', '仅测距 二次RTS', ...
    '测距+测向 前向', '测距+测向 一次RTS', ...
    '测距+测向 二次RTS'};
result_mode_names = [repmat({'range-only'}, 1, 3), ...
    repmat({'range-azimuth'}, 1, 3)];
result_stage_names = [stage_names, stage_names];
valid_truth = all(isfinite(truth_all(:, [2, 3, 4, 5])), 2);
truth_analysis = truth_all(valid_truth, :);
[~, truth_unique_index] = unique(truth_analysis(:, 2), 'stable');
truth_analysis = truth_analysis(truth_unique_index, :);
time_bounds = zeros(result_count, 2);
for result_index = 1:result_count
    nav = navigation_results{result_index};
    nav = nav(all(isfinite(nav(:, [2, 3, 4, 5])), 2), :);
    navigation_results{result_index} = nav;
    time_bounds(result_index, :) = [nav(1, 2), nav(end, 2)];
end
common_start_time = max([truth_analysis(1, 2); time_bounds(:, 1)]);
common_end_time = min([truth_analysis(end, 2); time_bounds(:, 2)]);
reference_position = interp1(truth_analysis(:, 2), ...
    truth_analysis(:, 3:5), common_start_time, 'linear');

metrics = zeros(result_count, 13);
traces = cell(result_count, 1);
for result_index = 1:result_count
    [metrics(result_index, :), traces{result_index}] = ...
        analyze_navigation_result(navigation_results{result_index}, ...
        truth_analysis, common_start_time, common_end_time, ...
        analysis_sample_interval_s, reference_position);
end
horizontal_improvement_pct = zeros(result_count, 1);
yaw_improvement_pct = zeros(result_count, 1);
for measurement_mode_index = 1:2
    mode_rows = (measurement_mode_index - 1) * 3 + (1:3);
    horizontal_improvement_pct(mode_rows) = 100 * ...
        (metrics(mode_rows(1), 5) - metrics(mode_rows, 5)) / ...
        metrics(mode_rows(1), 5);
    yaw_improvement_pct(mode_rows) = 100 * ...
        (metrics(mode_rows(1), 13) - metrics(mode_rows, 13)) / ...
        metrics(mode_rows(1), 13);
end

interval_edges_s = unique([common_start_time; ...
    rangedata(rangedata(:, 1) > common_start_time & ...
    rangedata(:, 1) < common_end_time, 1); common_end_time]);
interval_horizontal_rmse_m = ...
    nan(numel(interval_edges_s) - 1, result_count);
for interval_index = 1:numel(interval_edges_s) - 1
    for result_index = 1:result_count
        if interval_index < numel(interval_edges_s) - 1
            interval_mask = traces{result_index}.SampleTimeS >= ...
                interval_edges_s(interval_index) & ...
                traces{result_index}.SampleTimeS < ...
                interval_edges_s(interval_index + 1);
        else
            interval_mask = traces{result_index}.SampleTimeS >= ...
                interval_edges_s(interval_index) & ...
                traces{result_index}.SampleTimeS <= ...
                interval_edges_s(interval_index + 1);
        end
        interval_horizontal_rmse_m(interval_index, result_index) = ...
            sqrt(mean(traces{result_index}.HorizontalErrorM( ...
            interval_mask) .^ 2));
    end
end

summary_path = fullfile(output_dir, sprintf( ...
    'range-mode-stage-comparison-%s.txt', drop_tag));
summary_fp = fopen(summary_path, 'wt');
if summary_fp < 0
    error('无法创建误差对比表：%s', summary_path);
end
fprintf(summary_fp, '# data_source=%s case=%s phase_std_deg=%g\n', ...
    data_source, case_name, phasestd);
fprintf(summary_fp, '# attitude_feedback_mode=%s dropped_frames=%d,%d\n', ...
    attitude_feedback_mode, drop_measurement_indices);
fprintf(summary_fp, '# dropped_times_s=%.6f,%.6f dropped_beacons=%d,%d\n', ...
    dropped_time_s, dropped_beacon_id);
fprintf(summary_fp, '# azimuth_candidate_correct=%d/%d\n', ...
    nnz(selection_diagnostic(:, 9)), size(selection_diagnostic, 1));
fprintf(summary_fp, ['measurement_mode stage start_time_s end_time_s ' ...
    'sample_count north_rmse_m horizontal_rmse_m horizontal_mean_m ' ...
    'horizontal_p95_m horizontal_max_m height_rmse_m speed_rmse_mps ' ...
    'roll_rmse_deg pitch_rmse_deg yaw_rmse_deg ' ...
    'horizontal_improvement_vs_same_mode_forward_pct ' ...
    'yaw_improvement_vs_same_mode_forward_pct\n']);
for result_index = 1:result_count
    fprintf(summary_fp, ...
        ['%s %s %.3f %.3f %d %.9f %.9f %.9f %.9f %.9f %.9f ' ...
        '%.9f %.9f %.9f %.9f %.6f %.6f\n'], ...
        result_mode_names{result_index}, result_stage_names{result_index}, ...
        metrics(result_index, :), ...
        horizontal_improvement_pct(result_index), ...
        yaw_improvement_pct(result_index));
end
fclose(summary_fp);

interval_summary_path = fullfile(output_dir, sprintf( ...
    'range-mode-interval-comparison-%s.txt', drop_tag));
interval_fp = fopen(interval_summary_path, 'wt');
if interval_fp < 0
    error('无法创建分段误差对比表：%s', interval_summary_path);
end
fprintf(interval_fp, ['interval_start_s interval_end_s duration_s ' ...
    'range_only_forward_m range_only_single_rts_m ' ...
    'range_only_double_rts_m range_azimuth_forward_m ' ...
    'range_azimuth_single_rts_m range_azimuth_double_rts_m\n']);
for interval_index = 1:numel(interval_edges_s) - 1
    fprintf(interval_fp, ...
        '%.3f %.3f %.3f %.9f %.9f %.9f %.9f %.9f %.9f\n', ...
        interval_edges_s(interval_index), ...
        interval_edges_s(interval_index + 1), ...
        interval_edges_s(interval_index + 1) - ...
        interval_edges_s(interval_index), ...
        interval_horizontal_rmse_m(interval_index, :));
end
fclose(interval_fp);

%% 绘制两种量测模式、三个处理阶段的统一对比图
figure_visibility = 'off';
if show_result_figure
    figure_visibility = 'on';
end
comparison_figure = figure('Color', 'w', 'Visible', figure_visibility, ...
    'Position', [80, 70, 1500, 920]);
layout = tiledlayout(comparison_figure, 2, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
mode_colors = [0.10, 0.40, 0.75; 0.88, 0.33, 0.16];
stage_line_styles = {'-', '--', ':'};

horizontal_axis = nexttile(layout, 1);
hold(horizontal_axis, 'on');
for result_index = 1:result_count
    mode_index = ceil(result_index / 3);
    stage_index = mod(result_index - 1, 3) + 1;
    plot(horizontal_axis, traces{result_index}.TimeHours, ...
        traces{result_index}.HorizontalErrorM, ...
        'Color', mode_colors(mode_index, :), ...
        'LineStyle', stage_line_styles{stage_index}, ...
        'LineWidth', 1.15, 'DisplayName', result_labels{result_index});
end
for drop_index = 1:2
    xline(horizontal_axis, ...
        (dropped_time_s(drop_index) - common_start_time) / 3600, ...
        'k-.', sprintf('丢第%d帧', ...
        drop_measurement_indices(drop_index)), ...
        'HandleVisibility', 'off');
end
grid(horizontal_axis, 'on');
box(horizontal_axis, 'on');
xlabel(horizontal_axis, '时间 / h');
ylabel(horizontal_axis, '水平位置误差 / m');
title(horizontal_axis, '水平位置误差');
legend(horizontal_axis, 'Location', 'best');

yaw_axis = nexttile(layout, 2);
hold(yaw_axis, 'on');
for result_index = 1:result_count
    mode_index = ceil(result_index / 3);
    stage_index = mod(result_index - 1, 3) + 1;
    plot(yaw_axis, traces{result_index}.TimeHours, ...
        traces{result_index}.YawErrorDeg, ...
        'Color', mode_colors(mode_index, :), ...
        'LineStyle', stage_line_styles{stage_index}, ...
        'LineWidth', 1.15, 'DisplayName', result_labels{result_index});
end
for drop_index = 1:2
    xline(yaw_axis, ...
        (dropped_time_s(drop_index) - common_start_time) / 3600, ...
        'k-.', 'HandleVisibility', 'off');
end
grid(yaw_axis, 'on');
box(yaw_axis, 'on');
xlabel(yaw_axis, '时间 / h');
ylabel(yaw_axis, '航向误差 / deg');
title(yaw_axis, '航向误差');
legend(yaw_axis, 'Location', 'best');

trajectory_axis = nexttile(layout, 3);
hold(trajectory_axis, 'on');
plot(trajectory_axis, traces{1}.TruthEastM, traces{1}.TruthNorthM, ...
    'k-', 'LineWidth', 1.7, 'DisplayName', '真值');
for result_index = 1:result_count
    mode_index = ceil(result_index / 3);
    stage_index = mod(result_index - 1, 3) + 1;
    plot(trajectory_axis, traces{result_index}.NavEastM, ...
        traces{result_index}.NavNorthM, ...
        'Color', mode_colors(mode_index, :), ...
        'LineStyle', stage_line_styles{stage_index}, ...
        'LineWidth', 1.0, 'DisplayName', result_labels{result_index});
end
axis(trajectory_axis, 'equal');
grid(trajectory_axis, 'on');
box(trajectory_axis, 'on');
xlabel(trajectory_axis, '东向 / m');
ylabel(trajectory_axis, '北向 / m');
title(trajectory_axis, '水平轨迹');
legend(trajectory_axis, 'Location', 'best');

rmse_axis = nexttile(layout, 4);
horizontal_rmse_by_stage = [metrics(1:3, 5), metrics(4:6, 5)];
yaw_rmse_by_stage = [metrics(1:3, 13), metrics(4:6, 13)];
yyaxis(rmse_axis, 'left');
bar(rmse_axis, 1:3, horizontal_rmse_by_stage, 'grouped');
ylabel(rmse_axis, '水平位置 RMSE / m');
yyaxis(rmse_axis, 'right');
hold(rmse_axis, 'on');
plot(rmse_axis, 1:3, yaw_rmse_by_stage(:, 1), 'o-', ...
    'Color', mode_colors(1, :), 'LineWidth', 1.4, ...
    'MarkerFaceColor', mode_colors(1, :));
plot(rmse_axis, 1:3, yaw_rmse_by_stage(:, 2), 's-', ...
    'Color', mode_colors(2, :), 'LineWidth', 1.4, ...
    'MarkerFaceColor', mode_colors(2, :));
ylabel(rmse_axis, '航向 RMSE / deg');
xticks(rmse_axis, 1:3);
xticklabels(rmse_axis, stage_labels);
grid(rmse_axis, 'on');
box(rmse_axis, 'on');
title(rmse_axis, '两种量测模式的三阶段 RMSE');
legend(rmse_axis, {'仅测距 水平', '测距+测向 水平', ...
    '仅测距 航向', '测距+测向 航向'}, 'Location', 'best');

sgtitle(layout, sprintf( ...
    '%s / %s / Phase STD=%g° / 连续丢第%d、%d帧', ...
    data_source, case_name, phasestd, drop_measurement_indices));
comparison_png_path = fullfile(output_dir, sprintf( ...
    'range-mode-stage-comparison-%s.png', drop_tag));
comparison_fig_path = fullfile(output_dir, sprintf( ...
    'range-mode-stage-comparison-%s.fig', drop_tag));
exportgraphics(comparison_figure, comparison_png_path, 'Resolution', 300);
savefig(comparison_figure, comparison_fig_path);
if ~show_result_figure
    close(comparison_figure);
end

outputs = struct( ...
    'data_source', char(data_source), ...
    'case_name', case_name, ...
    'phase_std_deg', phasestd, ...
    'drop_measurement_indices', drop_measurement_indices, ...
    'attitude_feedback_mode', char(attitude_feedback_mode), ...
    'output_dir', output_dir, ...
    'navigation_result_paths', {navigation_result_paths}, ...
    'selection_path', selection_path, ...
    'summary_path', summary_path, ...
    'interval_summary_path', interval_summary_path, ...
    'comparison_png_path', comparison_png_path, ...
    'comparison_fig_path', comparison_fig_path, ...
    'alignment_info', alignment_info);

fprintf('\n两种量测模式连续丢两帧计算完成，耗时 %.2f s。\n', ...
    toc(scenario_timer));
fprintf('共同评估区间：%.1f–%.1f s；方位候选角正确：%d/%d。\n', ...
    common_start_time, common_end_time, ...
    nnz(selection_diagnostic(:, 9)), size(selection_diagnostic, 1));
fprintf('方案                    水平RMSE/m  航向RMSE/deg\n');
for result_index = 1:result_count
    fprintf('%-23s %11.3f %13.6f\n', ...
        result_labels{result_index}, metrics(result_index, 5), ...
        metrics(result_index, 13));
end
fprintf('结果目录：%s\n', output_dir);
fprintf('误差表格：%s\n', summary_path);
fprintf('分段误差表格：%s\n', interval_summary_path);
fprintf('对比图片：%s\n', comparison_png_path);

%% -------- 局部函数 --------
function row = state_to_nav_row(navstate, param)
row = zeros(1, 11);
row(2) = navstate.time;
row(3:5) = [navstate.pos(1:2)' * param.R2D, navstate.pos(3)];
row(6:8) = navstate.vel';
row(9:11) = navstate.att' * param.R2D;
end

function write_nav_file(file_path, nav_data)
file_id = fopen(file_path, 'wt');
if file_id < 0
    error('无法创建导航结果：%s', file_path);
end
nav_format = ['%2d %12.6f %12.8f %12.8f %8.4f %8.4f ', ...
    '%8.4f %8.4f %8.4f %8.4f %8.4f\n'];
fprintf(file_id, nav_format, nav_data');
fclose(file_id);
end

function [metrics, trace] = analyze_navigation_result(nav, truth, ...
        common_start_time, common_end_time, sample_interval_s, ...
        reference_position)
nav = nav(all(isfinite(nav(:, 2:11)), 2), :);
[nav_time, nav_unique_index] = unique(nav(:, 2), 'stable');
nav = nav(nav_unique_index, :);
[truth_time, truth_unique_index] = unique(truth(:, 2), 'stable');
truth = truth(truth_unique_index, :);
sample_start = ceil(common_start_time / sample_interval_s) * ...
    sample_interval_s;
sample_end = floor(common_end_time / sample_interval_s) * ...
    sample_interval_s;
sample_time = (sample_start:sample_interval_s:sample_end)';
nav_value = interp1(nav_time, nav(:, 3:11), sample_time, 'linear');
truth_value = interp1(truth_time, truth(:, 3:11), ...
    sample_time, 'linear');
valid = all(isfinite(nav_value), 2) & all(isfinite(truth_value), 2);
sample_time = sample_time(valid);
nav_value = nav_value(valid, :);
truth_value = truth_value(valid, :);

earth_a = 6378137;
earth_e2 = 6.69437999014e-3;
latitude_rad = deg2rad(truth_value(:, 1));
height_m = truth_value(:, 3);
denominator = sqrt(1 - earth_e2 .* sin(latitude_rad) .^ 2);
rn = earth_a ./ denominator;
rm = earth_a * (1 - earth_e2) ./ denominator .^ 3;
north_error_m = deg2rad(nav_value(:, 1) - truth_value(:, 1)) .* ...
    (rm + height_m);
east_error_m = deg2rad(nav_value(:, 2) - truth_value(:, 2)) .* ...
    (rn + height_m) .* cos(latitude_rad);
height_error_m = nav_value(:, 3) - truth_value(:, 3);
horizontal_error_m = hypot(north_error_m, east_error_m);
velocity_error_mps = nav_value(:, 4:6) - truth_value(:, 4:6);
speed_error_mps = sqrt(sum(velocity_error_mps .^ 2, 2));
roll_error_deg = wrap_to_180(nav_value(:, 7) - truth_value(:, 7));
pitch_error_deg = wrap_to_180(nav_value(:, 8) - truth_value(:, 8));
yaw_error_deg = wrap_to_180(nav_value(:, 9) - truth_value(:, 9));
sorted_horizontal_error_m = sort(horizontal_error_m);
p95_index = max(1, ceil(0.95 * numel(sorted_horizontal_error_m)));

metrics = [sample_time(1), sample_time(end), numel(sample_time), ...
    sqrt(mean(north_error_m .^ 2)), ...
    sqrt(mean(horizontal_error_m .^ 2)), ...
    mean(horizontal_error_m), sorted_horizontal_error_m(p95_index), ...
    max(horizontal_error_m), sqrt(mean(height_error_m .^ 2)), ...
    sqrt(mean(speed_error_mps .^ 2)), ...
    sqrt(mean(roll_error_deg .^ 2)), ...
    sqrt(mean(pitch_error_deg .^ 2)), ...
    sqrt(mean(yaw_error_deg .^ 2))];

reference_lat_rad = deg2rad(reference_position(1));
reference_height_m = reference_position(3);
reference_denominator = sqrt(1 - earth_e2 * ...
    sin(reference_lat_rad) ^ 2);
reference_rn = earth_a / reference_denominator;
reference_rm = earth_a * (1 - earth_e2) / ...
    reference_denominator ^ 3;
trace.TimeHours = (sample_time - common_start_time) / 3600;
trace.SampleTimeS = sample_time;
trace.HorizontalErrorM = horizontal_error_m;
trace.YawErrorDeg = yaw_error_deg;
trace.NavNorthM = deg2rad(nav_value(:, 1) - reference_position(1)) * ...
    (reference_rm + reference_height_m);
trace.NavEastM = deg2rad(nav_value(:, 2) - reference_position(2)) * ...
    (reference_rn + reference_height_m) * cos(reference_lat_rad);
trace.TruthNorthM = deg2rad(truth_value(:, 1) - reference_position(1)) * ...
    (reference_rm + reference_height_m);
trace.TruthEastM = deg2rad(truth_value(:, 2) - reference_position(2)) * ...
    (reference_rn + reference_height_m) * cos(reference_lat_rad);
end

function angle_deg = wrap_to_180(angle_deg)
angle_deg = mod(angle_deg + 180, 360) - 180;
end
