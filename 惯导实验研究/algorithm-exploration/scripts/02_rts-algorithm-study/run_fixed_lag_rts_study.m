clear;
close all;
clc;
%% RTS算法统一研究主程序：纯惯导、前向 ES-EKF 与固定滞后 RTS
% 输出：truth、pure-ins、forward EKF，以及各 lag_segments 的 RTS 结果
%% 1. 用户配置
data_source = "experiment";            % "simulation" 或 "experiment"
case_name = 'case-09';
position_error_unit = "m";           % "rad" 或 "m"
range_interval_s = 420;                % 测距间隔：7 min
duration_s = 8640000;                     % 从数据起点开始处理的时长
beacon_order = [1, 2, 3];              % 三个信标固定轮换顺序
simulation_range_noise_std_m = 6;
simulation_depth_noise_std_m = 0.4;
experiment_range_std_m = 6;
experiment_depth_std_m = 0.4;
enable_feedback = true;
enable_smoothing = true;
lag_segments = 1:4;                    % 同时测试的测距区间窗口长度
enable_iterated_range_update = false; % 当前单信标稀疏测距下默认普通ES-EKF更优
maximum_range_iterations = 2;
range_iteration_position_tolerance_m = 1e-3;
random_seed = 1;

data_source_override = strtrim(getenv('KF_GINS_DATA_SOURCE'));
if ~isempty(data_source_override)
    data_source = string(data_source_override);
end
case_name_override = strtrim(getenv('KF_GINS_CASE_NAME'));
if ~isempty(case_name_override)
    case_name = char(case_name_override);
end

% 自动化对照实验覆盖：0=普通ES-EKF，1=迭代ES-EKF。
iterated_override = str2double(getenv('KF_GINS_ENABLE_ITERATED_RANGE'));
if isfinite(iterated_override)
    enable_iterated_range_update = iterated_override ~= 0;
end
range_iteration_override = str2double(getenv('KF_GINS_MAX_RANGE_ITERATIONS'));
if isfinite(range_iteration_override) && range_iteration_override >= 1
    maximum_range_iterations = round(range_iteration_override);
end

duration_override = str2double(getenv('KF_GINS_RTS_DURATION_S'));
if isfinite(duration_override) && duration_override > 0
    duration_s = duration_override;
end
lag_override = strtrim(getenv('KF_GINS_LAG_SEGMENTS'));
if ~isempty(lag_override)
    lag_segments = sscanf(strrep(lag_override, ',', ' '), '%f')';
end
if isempty(lag_segments) || any(~isfinite(lag_segments)) || ...
        any(lag_segments < 1) || any(lag_segments ~= round(lag_segments))
    error('lag_segments必须是非空的正整数向量。');
end
lag_segments = unique(round(lag_segments), 'sorted');
%% 2. 初始化路径、算法配置和输出目录
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
paths = setup_inertial_experiment();
param = Param();
glvs;
rng(random_seed, 'twister');
data_source = lower(data_source);
position_error_unit = lower(position_error_unit);
if ~ismember(data_source, ["simulation", "experiment"])
    error('data_source只能设置为"simulation"或"experiment"。');
end
if ~ismember(position_error_unit, ["rad", "m"])
    error('position_error_unit只能设置为"rad"或"m"。');
end
if data_source == "simulation"
    id = str2double(case_name(end));
    input_dir = paths.simulation_input(id);
    cfg = load_algorithm_exploration_config("simulation", position_error_unit, input_dir);
    output_dir = fullfile(cfg.outputfolder, sprintf('simple-ekf-rts-%s', position_error_unit));
    filter_range_std_m = simulation_range_noise_std_m;
    filter_depth_std_m = simulation_depth_noise_std_m;
else
    id = str2double(case_name(end));
    input_dir = paths.experiment_input(id);
    cfg = load_algorithm_exploration_config( ...
        "experiment", position_error_unit, input_dir);
    output_dir = fullfile(cfg.outputfolder, sprintf('simple-ekf-rts-%s', position_error_unit));
    filter_range_std_m = experiment_range_std_m;
    filter_depth_std_m = experiment_depth_std_m;
end
if enable_smoothing && ~enable_iterated_range_update
    output_dir = fullfile(cfg.outputfolder, ...
        sprintf('fixed-lag-rts-%s', position_error_unit));
end
if enable_iterated_range_update
    output_dir = fullfile(cfg.outputfolder, ...
        sprintf('iterated-%diter-ekf-rts-%s', maximum_range_iterations, position_error_unit));
end
uses_simulated_measurements = data_source == "simulation" || ...
    (data_source == "experiment" && strcmp(case_name, 'case-06'));
if ~isfolder(input_dir)
    error('%s输入目录不存在：%s', data_source, input_dir);
end

if ~isfolder(output_dir)
    mkdir(output_dir);
end
truth_output_dir = cfg.outputfolder;
cfg.userange = true;
cfg.outputfolder = output_dir;
if position_error_unit == "rad"
    if enable_iterated_range_update
        error(['当前迭代测距更新先实现全米制15状态版本；' ...
            '请将position_error_unit设置为"m"。']);
    end
    range_update_function = @myRangeUpdate;
    feedback_function = @myErrorFeedback_range;
    height_update_function = @update_decoupled_height;
    propagation_function = @myInsPropagate_15state;
else
    if enable_iterated_range_update
        range_update_function = @(nav, range, depth, filter) ...
            myRangeUpdate_iterated_m(nav, range, depth, filter, ...
            maximum_range_iterations, ...
            range_iteration_position_tolerance_m);
    else
        range_update_function = @myRangeUpdate_m;
    end
    feedback_function = @myErrorFeedback_range_m;
    height_update_function = @update_decoupled_height_m;
    propagation_function = @myInsPropagate_15state_m;
end
%% 3. 导入并整理 IMU、真值、距离和高度数据
imudata_all = readmatrix(cfg.imufilepath, 'FileType', 'text');
truth = readmatrix(cfg.truthpath, 'FileType', 'text');
if uses_simulated_measurements
    range_sources = {readmatrix(cfg.rangefile1path, 'FileType', 'text'), readmatrix(cfg.rangefile2path, 'FileType', 'text'), readmatrix(cfg.rangefile3path, 'FileType', 'text')};
    source_interval_s = median(diff(range_sources{1}(:, 1)));
    range_stride = round(range_interval_s / source_interval_s);
    if abs(range_stride * source_interval_s - range_interval_s) > 1e-6
        error('测距间隔 %.3f s 不是原始采样间隔 %.3f s 的整数倍。', range_interval_s, source_interval_s);
    end
    for source_index = 1:numel(range_sources)
        range_sources{source_index} = range_sources{source_index}(range_stride:range_stride:end, :);
    end
    event_count = min(cellfun(@(data) size(data, 1), range_sources));
    rangedata = zeros(event_count, size(range_sources{1}, 2));
    for event_index = 1:event_count
        order_index = mod(event_index - 1, numel(beacon_order)) + 1;
        source_index = beacon_order(order_index);
        rangedata(event_index, :) = range_sources{source_index}(event_index, :);
    end
    rangedata(:, 3) = rangedata(:, 3) + simulation_range_noise_std_m * randn(size(rangedata, 1), 1);
    height_source = [];
else
    rangedata = readmatrix(cfg.rangefilepath, 'FileType', 'text');
    if isempty(rangedata) || size(rangedata, 2) < 6 || ...
            any(diff(rangedata(:, 1)) <= 0)
        error('实测 range.txt 必须是时间严格递增的非空 N×6 矩阵。');
    end
    height_source = readmatrix(cfg.heightfilepath, 'FileType', 'text');
end
start_time = max([cfg.starttime, imudata_all(1, 1), truth(1, 2)]);
end_time = min([start_time + duration_s, cfg.endtime, imudata_all(end, 1), truth(end, 2)]);
cfg.starttime = start_time;
cfg.endtime = end_time;
imu_mask = imudata_all(:, 1) >= start_time & imudata_all(:, 1) <= end_time;
imudata = imudata_all(imu_mask, :);
range_mask = rangedata(:, 1) >= start_time & rangedata(:, 1) <= end_time;
rangedata = rangedata(range_mask, :);
if uses_simulated_measurements
    height_value = interp1(truth(:, 2), truth(:, 5), imudata(:, 1), 'linear', 'extrap');
    height = [imudata(:, 1), height_value + simulation_depth_noise_std_m * randn(size(height_value))];
else
    height_value = interp1(height_source(:, 1), height_source(:, 2), ...
        imudata(:, 1), 'linear', 'extrap');
    height = [imudata(:, 1), height_value];
end
if isempty(rangedata)
    error('当前时间范围内没有测距事件。');
end
%% 4. 复制真值文件并设置输出文件
[~, truth_file_name, truth_file_ext] = fileparts(cfg.truthpath);
truth_copy_path = fullfile(truth_output_dir, [truth_file_name, truth_file_ext]);
[copy_success, copy_message] = copyfile(cfg.truthpath, truth_copy_path, 'f');
if ~copy_success
    error('truth文件复制失败：%s', copy_message);
end
fprintf('参考真值已复制：%s\n', truth_copy_path);
pure_ins_path = fullfile(truth_output_dir, 'pure-ins.nav');
forward_path = fullfile(output_dir, sprintf('simple-forward-ekf-%s.nav', position_error_unit));
rts_paths = cell(size(lag_segments));
for lag_index = 1:numel(lag_segments)
    rts_paths{lag_index} = fullfile(output_dir, sprintf( ...
        'simple-rts-lag%d-%s.nav', lag_segments(lag_index), position_error_unit));
end
range_iteration_diagnostic_path = fullfile(output_dir, ...
    sprintf('range-update-diagnostics-%s.csv', position_error_unit));
nav_format = ['%2d %12.6f %12.8f %12.8f %8.4f %8.4f ', '%8.4f %8.4f %8.4f %8.4f %8.4f\n'];
pure_ins_fp = fopen(pure_ins_path, 'wt');
if pure_ins_fp < 0
    error('无法创建纯惯导结果：%s', pure_ins_path);
end
forward_fp = fopen(forward_path, 'wt');
if forward_fp < 0
    fclose(pure_ins_fp);
    error('无法创建前向导航结果：%s', forward_path);
end
rts_fps = -ones(size(lag_segments));
if enable_smoothing
    for lag_index = 1:numel(lag_segments)
        rts_fps(lag_index) = fopen(rts_paths{lag_index}, 'wt');
        if rts_fps(lag_index) < 0
            fclose(pure_ins_fp);
            fclose(forward_fp);
            close_open_files(rts_fps);
            error('无法创建lag_segments=%d的RTS结果：%s', ...
                lag_segments(lag_index), rts_paths{lag_index});
        end
    end
end
%% 5. 初始化纯惯导、ES-EKF和RTS缓存
[kf, navstate] = myInitialize_15state(cfg);
kf.rangstd = filter_range_std_m;
kf.depthstd = filter_depth_std_m;
pure_ins_navstate = navstate;
last_imu = imudata(1, :)';
this_imu = imudata(1, :)';
range_index = find(rangedata(:, 1) >= this_imu(1), 1, 'first');
imu_interval_s = median(diff(imudata(:, 1)));
maximum_segment_samples = ceil(range_interval_s / imu_interval_s) + 20;
state_buffer = zeros(maximum_segment_samples, 10);
corrected_covariance_buffer = zeros(maximum_segment_samples, 225);
predicted_covariance_buffer = zeros(maximum_segment_samples, 225);
transition_buffer = zeros(maximum_segment_samples, 225);
buffer_index = 1;
lag_queue = empty_lag_queue();
range_update_diagnostics = nan(size(rangedata, 1), 7);
last_progress = -1;
fprintf(['开始处理 %s/%s（%s位置误差状态）：纯惯导=1，前向EKF=%d，' ...
    '迭代测距=%d，固定滞后RTS=%d，lag_segments=%s。\n'], data_source, ...
    case_name, position_error_unit, enable_feedback, ...
    enable_iterated_range_update, enable_smoothing, mat2str(lag_segments));
tic;
%% 6. 主循环：结构与 run_all_real_datasets_rtsfix.m 保持一致
for imu_index = 2:size(imudata, 1)
    last_imu = this_imu;
    this_imu = imudata(imu_index, :)';
    imu_dt = this_imu(1) - last_imu(1);
    time_tolerance = max(1e-8, abs(imu_dt) * 0.25);

    % 纯惯导链独立运行，不接受距离更新；高度直接由当前观测赋值。
    pure_ins_navstate = InsMech( ...
        pure_ins_navstate, last_imu, this_imu);
    pure_ins_navstate.pos(3) = height(imu_index, 2);

    % 组合导航状态对应 last_imu 时刻。区间内非对齐测距由第二分支处理。
    while range_index <= size(rangedata, 1) && ...
            rangedata(range_index, 1) < last_imu(1) - time_tolerance
        range_index = range_index + 1;
    end
    has_range = range_index <= size(rangedata, 1);
    range_at_last_imu = has_range && ...
        abs(last_imu(1) - rangedata(range_index, 1)) <= time_tolerance;
    range_inside_interval = has_range && ...
        rangedata(range_index, 1) > last_imu(1) + time_tolerance && ...
        rangedata(range_index, 1) < this_imu(1) - time_tolerance;

    if range_at_last_imu && cfg.userange == 1
        %% 6.1 测距与上一 IMU 历元重合：先更新，再向前传播
        kf = range_update_function(navstate, rangedata(range_index, :), ...
            height(imu_index - 1, :), kf);
        if enable_iterated_range_update
            range_update_diagnostics(range_index, :) = ...
                collect_range_update_diagnostic( ...
                rangedata(range_index, 1), kf);
        end
        terminal_error = kf.x;

        if enable_smoothing && buffer_index > 1
            valid_length = buffer_index - 1;
            current_state = state_buffer(1:valid_length, :);
            current_corrected_covariance = ...
                corrected_covariance_buffer(1:valid_length, :);
            current_predicted_covariance = ...
                predicted_covariance_buffer(1:valid_length, :);
            current_transition = transition_buffer(1:valid_length, :);
            lag_queue = update_lag_smoothers(lag_queue, ...
                lag_segments, rts_fps, current_state, ...
                current_corrected_covariance, ...
                current_predicted_covariance, current_transition, ...
                terminal_error, param, range_index, ...
                char(position_error_unit), nav_format);

            buffer_index = 1;
            state_buffer(:) = 0;
            corrected_covariance_buffer(:) = 0;
            predicted_covariance_buffer(:) = 0;
            transition_buffer(:) = 0;
        end

        if enable_feedback
            [kf, navstate] = feedback_function(kf, navstate);
        else
            kf.x(:) = 0;
        end
        range_index = range_index + 1;

        % Range 后的第一个 IMU 点作为新 RTS 区间首点保存。
        if enable_smoothing
            if buffer_index > maximum_segment_samples
                error('RTS缓存不足，请增大 maximum_segment_samples。');
            end
            corrected_covariance_buffer(buffer_index, :) = kf.P(:)';
        end
        navstate = InsMech(navstate, last_imu, this_imu);
        kf = propagation_function(navstate, this_imu, imu_dt, kf);
        if enable_smoothing
            state_buffer(buffer_index, :) = [navstate.time, ...
                navstate.pos', navstate.vel', navstate.att'];
            predicted_covariance_buffer(buffer_index, :) = kf.P(:)';
            transition_buffer(buffer_index, :) = kf.phi(:)';
            buffer_index = buffer_index + 1;
        end

    elseif range_inside_interval && cfg.userange == 1
        %% 6.2 测距位于两个 IMU 历元之间：拆分增量后精确更新
        range_time = rangedata(range_index, 1);
        [first_imu, second_imu] = interpolate( ...
            last_imu, this_imu, range_time);

        % 先传播到精确测距时刻，并将该点加入当前 RTS 区间。
        first_dt = first_imu(1) - last_imu(1);
        if enable_smoothing
            if buffer_index > maximum_segment_samples
                error('RTS缓存不足，请增大 maximum_segment_samples。');
            end
            corrected_covariance_buffer(buffer_index, :) = kf.P(:)';
        end
        navstate = InsMech(navstate, last_imu, first_imu);
        kf = propagation_function(navstate, first_imu, first_dt, kf);
        if enable_smoothing
            state_buffer(buffer_index, :) = [navstate.time, ...
                navstate.pos', navstate.vel', navstate.att'];
            predicted_covariance_buffer(buffer_index, :) = kf.P(:)';
            transition_buffer(buffer_index, :) = kf.phi(:)';
            buffer_index = buffer_index + 1;
        end

        % 在精确测距时刻执行距离+深度联合更新。
        range_height = [range_time, interp1(height(:, 1), height(:, 2), ...
            range_time, 'linear', 'extrap')];
        kf = range_update_function(navstate, rangedata(range_index, :), ...
            range_height, kf);
        if enable_iterated_range_update
            range_update_diagnostics(range_index, :) = ...
                collect_range_update_diagnostic(range_time, kf);
        end
        terminal_error = kf.x;

        if enable_smoothing && buffer_index > 1
            valid_length = buffer_index - 1;
            current_state = state_buffer(1:valid_length, :);
            current_corrected_covariance = ...
                corrected_covariance_buffer(1:valid_length, :);
            current_predicted_covariance = ...
                predicted_covariance_buffer(1:valid_length, :);
            current_transition = transition_buffer(1:valid_length, :);
            lag_queue = update_lag_smoothers(lag_queue, ...
                lag_segments, rts_fps, current_state, ...
                current_corrected_covariance, ...
                current_predicted_covariance, current_transition, ...
                terminal_error, param, range_index, ...
                char(position_error_unit), nav_format);

            buffer_index = 1;
            state_buffer(:) = 0;
            corrected_covariance_buffer(:) = 0;
            predicted_covariance_buffer(:) = 0;
            transition_buffer(:) = 0;
        end

        if enable_feedback
            [kf, navstate] = feedback_function(kf, navstate);
        else
            kf.x(:) = 0;
        end
        range_index = range_index + 1;

        % 完成测距时刻至当前 IMU 历元的剩余传播。
        second_dt = second_imu(1) - first_imu(1);
        if enable_smoothing
            if buffer_index > maximum_segment_samples
                error('RTS缓存不足，请增大 maximum_segment_samples。');
            end
            corrected_covariance_buffer(buffer_index, :) = kf.P(:)';
        end
        navstate = InsMech(navstate, first_imu, second_imu);
        kf = propagation_function(navstate, second_imu, second_dt, kf);
        if enable_smoothing
            state_buffer(buffer_index, :) = [navstate.time, ...
                navstate.pos', navstate.vel', navstate.att'];
            predicted_covariance_buffer(buffer_index, :) = kf.P(:)';
            transition_buffer(buffer_index, :) = kf.phi(:)';
            buffer_index = buffer_index + 1;
        end

    else
        %% 6.3 普通历元：惯导、深度更新和误差传播
        navstate = InsMech(navstate, last_imu, this_imu);
        [kf, navstate] = height_update_function( ...
            kf, navstate, height(imu_index, :));

        if enable_smoothing
            if buffer_index > maximum_segment_samples
                error('RTS缓存不足，请增大 maximum_segment_samples。');
            end
            corrected_covariance_buffer(buffer_index, :) = kf.P(:)';
        end
        kf = propagation_function(navstate, this_imu, imu_dt, kf);
        if enable_smoothing
            state_buffer(buffer_index, :) = [navstate.time, ...
                navstate.pos', navstate.vel', navstate.att'];
            predicted_covariance_buffer(buffer_index, :) = kf.P(:)';
            transition_buffer(buffer_index, :) = kf.phi(:)';
            buffer_index = buffer_index + 1;
        end
    end

    %% 6.4 保存组合导航和独立纯惯导结果
    nav_row = [0; navstate.time; navstate.pos(1:2) * param.R2D; navstate.pos(3); navstate.vel; navstate.att * param.R2D];
    fprintf(forward_fp, nav_format, nav_row);
    pure_ins_row = [0; pure_ins_navstate.time; pure_ins_navstate.pos(1:2) * param.R2D; pure_ins_navstate.pos(3); pure_ins_navstate.vel; pure_ins_navstate.att * param.R2D];
    fprintf(pure_ins_fp, nav_format, pure_ins_row);
    progress = floor(10 * imu_index / size(imudata, 1)) * 10;
    if progress > last_progress && mod(progress, 20) == 0
        fprintf('处理进度：%d %%\n', progress);
        last_progress = progress;
    end
end
%% 7. 写入最后一段并关闭输出
if enable_smoothing
    flush_lag_queue(lag_queue, lag_segments, rts_fps, nav_format);
end
fclose(pure_ins_fp);
fclose(forward_fp);
close_open_files(rts_fps);
elapsed_time_s = toc;
if enable_iterated_range_update
    write_range_update_diagnostics(range_iteration_diagnostic_path, ...
        range_update_diagnostics);
end
fprintf('处理完成，耗时 %.2f s。\n', elapsed_time_s);
fprintf('参考真值：%s\n', truth_copy_path);
fprintf('纯惯导：%s\n', pure_ins_path);
fprintf('前向EKF：%s\n', forward_path);
if enable_smoothing
    for lag_index = 1:numel(lag_segments)
        fprintf('固定滞后RTS（lag_segments=%d）：%s\n', ...
            lag_segments(lag_index), rts_paths{lag_index});
    end
end
if enable_iterated_range_update
    fprintf('迭代测距诊断：%s\n', range_iteration_diagnostic_path);
end

function row = collect_range_update_diagnostic(time, kf)
    row = [time, kf.range_iteration_count, ...
        kf.range_initial_residual_m, kf.range_final_residual_m, ...
        kf.range_position_correction_m, kf.range_initial_nis, ...
        kf.range_converged];
end

function queue = empty_lag_queue()
    queue = struct('state', {}, 'corrected_covariance', {}, ...
        'predicted_covariance', {}, 'transition', {}, ...
        'range_index', {}, 'nav', {});
end

function lag_queue = update_lag_smoothers(lag_queue, ...
        lag_segments, rts_fps, state, corrected_covariance, ...
        predicted_covariance, transition, terminal_error, param, ...
        range_index, position_error_unit, nav_format)
    new_segment = struct( ...
        'state', state, ...
        'corrected_covariance', corrected_covariance, ...
        'predicted_covariance', predicted_covariance, ...
        'transition', transition, ...
        'range_index', range_index, ...
        'nav', []);

    lag_queue(end + 1) = new_segment;
    bridge_error = terminal_error;

    for segment_index = numel(lag_queue):-1:1
        [segment_nav, bridge_error, smoothed_state] = ...
            perform_unified_smoothing( ...
            lag_queue(segment_index).state, bridge_error, param, ...
            lag_queue(segment_index).range_index, 'RTS', ...
            position_error_unit, ...
            lag_queue(segment_index).corrected_covariance, ...
            lag_queue(segment_index).predicted_covariance, ...
            lag_queue(segment_index).transition);
        lag_queue(segment_index).state = smoothed_state;
        lag_queue(segment_index).nav = segment_nav;
    end

    for lag_index = 1:numel(lag_segments)
        lag = lag_segments(lag_index);
        if numel(lag_queue) >= lag
            output_index = numel(lag_queue) - lag + 1;
            fprintf(rts_fps(lag_index), nav_format, ...
                lag_queue(output_index).nav);
        end
    end
    if numel(lag_queue) >= max(lag_segments)
        lag_queue(1) = [];
    end
end

function flush_lag_queue(lag_queue, lag_segments, rts_fps, nav_format)
    queue_length = numel(lag_queue);
    for lag_index = 1:numel(lag_segments)
        remaining_count = min(lag_segments(lag_index) - 1, queue_length);
        first_index = queue_length - remaining_count + 1;
        for segment_index = first_index:queue_length
            fprintf(rts_fps(lag_index), nav_format, ...
                lag_queue(segment_index).nav);
        end
    end
end

function close_open_files(file_ids)
    for file_index = 1:numel(file_ids)
        if file_ids(file_index) >= 0
            fclose(file_ids(file_index));
        end
    end
end

function write_range_update_diagnostics(file_path, diagnostics)
    valid = all(isfinite(diagnostics(:, 1:6)), 2);
    diagnostics = diagnostics(valid, :);
    table_value = array2table(diagnostics, 'VariableNames', { ...
        'time_s', 'iterations', 'initial_range_residual_m', ...
        'final_range_residual_m', 'position_correction_m', ...
        'initial_nis', 'converged'});
    writetable(table_value, file_path);
end


