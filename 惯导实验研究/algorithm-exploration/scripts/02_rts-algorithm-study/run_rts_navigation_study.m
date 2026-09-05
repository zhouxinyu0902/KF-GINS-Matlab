clear;
close all;
clc;
%% RTS算法统一研究主程序：纯惯导、前向 ES-EKF 与一次/二次 RTS
% 输出：truth、pure-ins、forward EKF、single RTS、double RTS
%% 1. 用户配置
data_source = "experiment";            % "simulation" 或 "experiment"
simulation_case = 'case-00';
position_error_unit = "rad";           % "rad" 或 "m"
range_interval_s = 420;                % 测距间隔：7 min
duration_s = 4621;                     % 从数据起点开始处理的时长
beacon_order = [1, 2, 3];              % 三个信标固定轮换顺序
simulation_range_noise_std_m = 6;
simulation_depth_noise_std_m = 0.4;
experiment_range_std_m = 6;
experiment_depth_std_m = 0.4;
enable_feedback = true;
enable_smoothing = true;
enable_second_rts = true;
random_seed = 1;
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
    case_name = char(simulation_case);
    id = str2double(case_name(end));
    input_dir = paths.simulation_input(id);
    cfg = load_algorithm_exploration_config("simulation", position_error_unit, input_dir);
    output_dir = fullfile(cfg.outputfolder, sprintf('simple-ekf-rts-%s', position_error_unit));
    filter_range_std_m = simulation_range_noise_std_m;
    filter_depth_std_m = simulation_depth_noise_std_m;
else
    case_name = 'case-06';
    id = str2double(case_name(end));
    cfg = load_algorithm_exploration_config("experiment", position_error_unit, []);
    input_dir = paths.experiment_input(id);
    output_dir = fullfile(cfg.outputfolder, sprintf('simple-ekf-rts-%s', position_error_unit));
    filter_range_std_m = experiment_range_std_m;
    filter_depth_std_m = experiment_depth_std_m;
end
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
    range_update_function = @myRangeUpdate;
    feedback_function = @myErrorFeedback_range;
    height_update_function = @update_decoupled_height;
    propagation_function = @myInsPropagate_15state;
else
    range_update_function = @myRangeUpdate_m;
    feedback_function = @myErrorFeedback_range_m;
    height_update_function = @update_decoupled_height_m;
    propagation_function = @myInsPropagate_15state_m;
end
%% 3. 导入并整理 IMU、真值、距离和高度数据
imudata_all = readmatrix(cfg.imufilepath, 'FileType', 'text');
truth = readmatrix(cfg.truthpath, 'FileType', 'text');
if data_source == "simulation"
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
    % range_path = fullfile(input_dir, 'rangedata_noised.txt');
    % height_path = fullfile(input_dir, 'height_noised.txt');
    % if ~isfile(range_path) || ~isfile(height_path)
    %     error('实测预处理距离或高度文件缺失：%s', input_dir);
    % end
    % rangedata = readmatrix(range_path, 'FileType', 'text');
    % height_source = readmatrix(height_path, 'FileType', 'text');
    % if any(abs(diff(rangedata(:, 1)) - range_interval_s) > 1e-6)
    %     error('实测距离数据不是固定 %.0f s 间隔。', range_interval_s);
    % end
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
    rangedata(:, 3) = rangedata(:, 3) + experiment_range_std_m * randn(size(rangedata, 1), 1);
    height_source = [];
end
start_time = max([cfg.starttime, imudata_all(1, 1), truth(1, 2)]);
end_time = min([start_time + duration_s, cfg.endtime, imudata_all(end, 1), truth(end, 2)]);
cfg.starttime = start_time;
cfg.endtime = end_time;
imu_mask = imudata_all(:, 1) >= start_time & imudata_all(:, 1) <= end_time;
imudata = imudata_all(imu_mask, :);
range_mask = rangedata(:, 1) >= start_time & rangedata(:, 1) <= end_time;
rangedata = rangedata(range_mask, :);
if data_source == "simulation"
    height_value = interp1(truth(:, 2), truth(:, 5), imudata(:, 1), 'linear', 'extrap');
    height = [imudata(:, 1), height_value + simulation_depth_noise_std_m * randn(size(height_value))];
else
    height_value = interp1(truth(:, 2), truth(:, 5), imudata(:, 1), 'linear', 'extrap');
    height = [imudata(:, 1), height_value + experiment_depth_std_m * randn(size(height_value))];
    % height_value = interp1(height_source(:, 1), height_source(:, 2), imudata(:, 1), 'linear', 'extrap');
    % height = [imudata(:, 1), height_value];
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
single_rts_path = fullfile(output_dir, sprintf('simple-rts-single-%s.nav', position_error_unit));
double_rts_path = fullfile(output_dir, sprintf('simple-rts-double-%s.nav', position_error_unit));
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
single_rts_fp = -1;
double_rts_fp = -1;
if enable_smoothing
    single_rts_fp = fopen(single_rts_path, 'wt');
    if single_rts_fp < 0
        fclose(pure_ins_fp);
        fclose(forward_fp);
        error('无法创建一次RTS结果：%s', single_rts_path);
    end
    if enable_second_rts
        double_rts_fp = fopen(double_rts_path, 'wt');
        if double_rts_fp < 0
            fclose(pure_ins_fp);
            fclose(forward_fp);
            fclose(single_rts_fp);
            error('无法创建二次RTS结果：%s', double_rts_path);
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
previous_single_state = [];
previous_single_nav = [];
previous_corrected_covariance = [];
previous_predicted_covariance = [];
previous_transition = [];
previous_range_index = 0;
last_progress = -1;
fprintf('开始处理 %s/%s（%s位置误差状态）：纯惯导=1，前向EKF=%d，一次RTS=%d，二次RTS=%d。\n', data_source, case_name, position_error_unit, enable_feedback, enable_smoothing, enable_smoothing && enable_second_rts);
tic;
%% 6. 主循环：结构与 run_all_real_datasets_rtsfix.m 保持一致
for imu_index = 2:size(imudata, 1)
    last_imu = this_imu;
    this_imu = imudata(imu_index, :)';
    imu_dt = this_imu(1) - last_imu(1);
    time_tolerance = max(1e-8, abs(imu_dt) * 0.25);

    % 纯惯导链独立运行，不接受距离或深度反馈。
    pure_ins_navstate = InsMech( ...
        pure_ins_navstate, last_imu, this_imu);

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
        terminal_error = kf.x;

        if enable_smoothing && buffer_index > 1
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
                'RTS', char(position_error_unit), ...
                current_corrected_covariance, ...
                current_predicted_covariance, current_transition);
            fprintf(single_rts_fp, nav_format, single_nav);

            if enable_second_rts
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
                        previous_range_index, 'RTS', ...
                        char(position_error_unit), ...
                        previous_corrected_covariance, ...
                        previous_predicted_covariance, previous_transition);
                    fprintf(double_rts_fp, nav_format, double_nav);

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
        terminal_error = kf.x;

        if enable_smoothing && buffer_index > 1
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
                'RTS', char(position_error_unit), ...
                current_corrected_covariance, ...
                current_predicted_covariance, current_transition);
            fprintf(single_rts_fp, nav_format, single_nav);

            if enable_second_rts
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
                        previous_range_index, 'RTS', ...
                        char(position_error_unit), ...
                        previous_corrected_covariance, ...
                        previous_predicted_covariance, previous_transition);
                    fprintf(double_rts_fp, nav_format, double_nav);

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
if enable_smoothing && enable_second_rts && ~isempty(previous_single_nav)
    fprintf(double_rts_fp, nav_format, previous_single_nav);
end
fclose(pure_ins_fp);
fclose(forward_fp);
if single_rts_fp >= 0
    fclose(single_rts_fp);
end
if double_rts_fp >= 0
    fclose(double_rts_fp);
end
elapsed_time_s = toc;
fprintf('处理完成，耗时 %.2f s。\n', elapsed_time_s);
fprintf('参考真值：%s\n', truth_copy_path);
fprintf('纯惯导：%s\n', pure_ins_path);
fprintf('前向EKF：%s\n', forward_path);
if enable_smoothing
    fprintf('一次RTS：%s\n', single_rts_path);
end
if enable_smoothing && enable_second_rts
    fprintf('二次RTS：%s\n', double_rts_path);
end
