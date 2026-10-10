clear;
close all;
clc;
%% 仿真/实测：仅测距导航的指定丢帧二次 RTS 批处理
% 依次丢弃合并后的第 1、3、5、7、9、11 帧测距数据。
% 不使用相位差或方位角，磁盘仅保存二次 RTS 导航结果。

data_source = "experiment";             % "simulation" 或 "experiment"
case_name = 'case-06';
drop_measurement_indices = [1, 3, 5, 7, 9, 11];
range_interval_s = 420;
end_time_s = 4621;
beacon_order = [1, 2, 3];
random_seed = 1;
simulation_range_noise_std_m = 10;
experiment_range_noise_std_m = 6;
depth_noise_std_m = 0.4;
filter_depth_std_m = 0.4;
% 仅用于把纯测距结果放入与联合导航相同的对比分组目录。
phasestd = 15;
name = sprintf('double-hydrophone-phasestd%ddeg', phasestd);

%% 路径与输入
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));
addpath(topic_dir);
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
    cfg_base = load_algorithm_exploration_config( ...
        'simulation', 'rad', input_dir);
    navigation_dir = paths.simulation_navigation(case_id);
    range_noise_std_m = simulation_range_noise_std_m;
elseif data_source == "experiment"
    input_dir = paths.experiment_input(case_id);
    cfg_base = load_algorithm_exploration_config( ...
        'experiment', 'rad', input_dir);
    cfg_base.inputfolder = input_dir;
    cfg_base.imufilepath = fullfile(input_dir, 'IMU_120.txt');
    cfg_base.rangefile1path = fullfile(input_dir, 'range1.txt');
    cfg_base.rangefile2path = fullfile(input_dir, 'range2.txt');
    cfg_base.rangefile3path = fullfile(input_dir, 'range3.txt');
    cfg_base.truthpath = fullfile(input_dir, 'truth.nav');
    navigation_dir = paths.experiment_navigation(case_id);
    range_noise_std_m = experiment_range_noise_std_m;
else
    error('data_source 只能设置为 "simulation" 或 "experiment"。');
end
filter_range_std_m = range_noise_std_m;
cfg_base.userange = true;
output_dir = fullfile(navigation_dir, ...
    'navigation-results', name, 'drop-noazi-2RTS');
if ~isfolder(output_dir)
    mkdir(output_dir);
end

imu_all = readmatrix(cfg_base.imufilepath, 'FileType', 'text');
truth_all = readmatrix(cfg_base.truthpath, 'FileType', 'text');
range_paths = {cfg_base.rangefile1path, cfg_base.rangefile2path, ...
    cfg_base.rangefile3path};
range_sources = cellfun(@(path) readmatrix(path, 'FileType', 'text'), ...
    range_paths, 'UniformOutput', false);

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
base_rangedata = zeros(event_count, size(range_sources{1}, 2));
base_beacon_id = zeros(event_count, 1);
for event_index = 1:event_count
    order_index = mod(event_index - 1, numel(beacon_order)) + 1;
    source_index = beacon_order(order_index);
    base_rangedata(event_index, :) = ...
        range_sources{source_index}(event_index, :);
    base_beacon_id(event_index) = source_index;
end

% 所有掉帧场景共享同一套完整量测噪声；删帧只删除对应事件，
% 保证与联合导航掉帧脚本及无掉帧基准可公平比较。
rng(random_seed, 'twister');
base_rangedata(:, 3) = base_rangedata(:, 3) + ...
    range_noise_std_m * randn(size(base_rangedata, 1), 1);
depth_rng_state = rng;

if any(drop_measurement_indices < 1) || ...
        any(drop_measurement_indices > event_count) || ...
        any(drop_measurement_indices ~= fix(drop_measurement_indices))
    error('丢帧编号必须是 1 到 %d 之间的整数。', event_count);
end

%% 逐个丢帧场景运行
nav_format = ['%2d %12.6f %12.8f %12.8f %8.4f %8.4f ', ...
    '%8.4f %8.4f %8.4f %8.4f %8.4f\n'];

for drop_case_index = 1:numel(drop_measurement_indices)
    drop_index = drop_measurement_indices(drop_case_index);

    rangedata = base_rangedata;
    dropped_time_s = rangedata(drop_index, 1);
    dropped_beacon_id = base_beacon_id(drop_index);
    rangedata(drop_index, :) = [];

    cfg = cfg_base;
    start_time = max([cfg.starttime, imu_all(1, 1), truth_all(1, 2)]);
    end_time = min([start_time + end_time_s, cfg.endtime, ...
        imu_all(end, 1), truth_all(end, 2)]);
    cfg.starttime = start_time;
    cfg.endtime = end_time;

    imu_mask = imu_all(:, 1) >= start_time & imu_all(:, 1) <= end_time;
    imudata = imu_all(imu_mask, :);
    range_mask = rangedata(:, 1) >= start_time & ...
        rangedata(:, 1) <= end_time;
    rangedata = rangedata(range_mask, :);
    if isempty(rangedata)
        error('丢第 %d 帧后，当前时间范围内没有测距事件。', drop_index);
    end

    rng(depth_rng_state);
    height_values = interp1(truth_all(:, 2), truth_all(:, 5), ...
        imudata(:, 1), 'linear', 'extrap');
    height_values = height_values + depth_noise_std_m * ...
        randn(size(height_values));
    height = [imudata(:, 1), height_values];
    [imudata, height] = align_imu_to_range_epochs( ...
        imudata, height, rangedata(:, 1));

    fprintf(['\n场景 %d/%d：丢第 %d 帧，t=%.3f s，Beacon %d，' ...
        '保留 %d 帧测距。\n'], drop_case_index, ...
        numel(drop_measurement_indices), drop_index, dropped_time_s, ...
        dropped_beacon_id, size(rangedata, 1));

    %% 初始化 ES-EKF 和 RTS 缓存
    [kf, navstate] = myInitialize_15state(cfg);
    kf.rangstd = filter_range_std_m;
    kf.depthstd = filter_depth_std_m;

    imu_interval_s = median(diff(imudata(:, 1)));
    maximum_event_gap_s = max([ ...
        rangedata(1, 1) - imudata(1, 1); ...
        diff(rangedata(:, 1)); ...
        imudata(end, 1) - rangedata(end, 1)]);
    maximum_segment_samples = ...
        ceil(maximum_event_gap_s / imu_interval_s) + 20;
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
    double_rts_nav = zeros(11, 0);

    range_index = find(rangedata(:, 1) >= imudata(1, 1), 1, 'first');
    if isempty(range_index)
        range_index = size(rangedata, 1) + 1;
    end
    this_imu = imudata(1, :)';
    last_progress = -1;
    scenario_timer = tic;

    %% 前向距离 EKF、一次 RTS 和跨区间二次 RTS
    for imu_index = 2:size(imudata, 1)
        last_imu = this_imu;
        this_imu = imudata(imu_index, :)';
        imu_dt = this_imu(1) - last_imu(1);
        time_tolerance = max(1e-8, abs(imu_dt) * 0.25);

        while range_index <= size(rangedata, 1) && ...
                rangedata(range_index, 1) < ...
                last_imu(1) - time_tolerance
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
            kf = myRangeUpdate(navstate, current_range, current_depth, kf);
            terminal_error = kf.x;

            if buffer_index > 1
                valid_length = buffer_index - 1;
                current_state = state_buffer(1:valid_length, :);
                current_corrected_covariance = ...
                    corrected_covariance_buffer(1:valid_length, :);
                current_predicted_covariance = ...
                    predicted_covariance_buffer(1:valid_length, :);
                current_transition = ...
                    transition_buffer(1:valid_length, :);

                [single_nav, bridge_error, single_state] = ...
                    perform_unified_smoothing( ...
                    current_state, terminal_error, param, range_index, ...
                    'RTS', 'rad', current_corrected_covariance, ...
                    current_predicted_covariance, current_transition);

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
                        previous_predicted_covariance, ...
                        previous_transition);
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

                buffer_index = 1;
                state_buffer(:) = 0;
                corrected_covariance_buffer(:) = 0;
                predicted_covariance_buffer(:) = 0;
                transition_buffer(:) = 0;
            end

            [kf, navstate] = myErrorFeedback_range(kf, navstate);
            range_index = range_index + 1;
        end

        last_state = navstate;
        navstate = InsMech(last_state, last_imu, this_imu);
        if ~is_range_epoch
            [kf, navstate] = update_decoupled_height( ...
                kf, navstate, height(imu_index, :));
        end

        if buffer_index > maximum_segment_samples
            error('RTS 缓存不足：场景 drop%d。', drop_index);
        end
        corrected_covariance_buffer(buffer_index, :) = kf.P(:)';
        kf = myInsPropagate_15state(navstate, this_imu, imu_dt, kf);
        state_buffer(buffer_index, :) = [navstate.time, ...
            navstate.pos', navstate.vel', navstate.att'];
        predicted_covariance_buffer(buffer_index, :) = kf.P(:)';
        transition_buffer(buffer_index, :) = kf.phi(:)';
        buffer_index = buffer_index + 1;

        progress = floor(10 * imu_index / size(imudata, 1)) * 10;
        if progress > last_progress && mod(progress, 20) == 0
            fprintf('drop%d 处理进度：%d %%\n', drop_index, progress);
            last_progress = progress;
        end
    end

    % 最后一段没有未来桥接误差，按现有二次 RTS 架构保留其一次 RTS 结果。
    if ~isempty(previous_single_nav)
        double_rts_nav = [double_rts_nav, previous_single_nav]; %#ok<AGROW>
    end

    output_path = fullfile(output_dir, sprintf( ...
        'range-noazi-rts-double-drop%d.nav', drop_index));
    output_fp = fopen(output_path, 'wt');
    if output_fp < 0
        error('无法创建二次 RTS 结果：%s', output_path);
    end
    fprintf(output_fp, nav_format, double_rts_nav);
    fclose(output_fp);
    fprintf('drop%d 完成，耗时 %.2f s：%s\n', ...
        drop_index, toc(scenario_timer), output_path);
end

fprintf('\n全部无方位角丢帧二次 RTS 结果已生成：%s\n', output_dir);
