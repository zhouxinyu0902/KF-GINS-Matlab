clear;
close all;
clc;
%% 四方法导航结果统一生成脚本
% 方法1：前向 ES-EKF；方法2：二次 RTS；
% 方法3：2RTS 分段旋转收缩；方法4：2RTS 水平位置速度约束重放。
% 导航核心与 run_rts_navigation_study.m 保持同步。
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
force_rerun_navigation_core = false;   % false时优先复用已有前向和二次RTS
overwrite_method_results = false;      % true时强制重算方法3和方法4
enable_rotation_contraction = true;
enable_position_velocity_replay = true;
rts_update_interval_s = 1;
rts_position_std_m = 30;
rts_velocity_std_mps = 0.3;
boundary_guard_s = 60;
boundary_position_std_m = 80;
boundary_velocity_std_mps = 1.0;
innovation_gate_sigma = 3;
robust_gate_sigma = 6;
robust_std_scale = 3;
%% 2. 初始化路径、算法配置和输出目录
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
paths = setup_inertial_experiment();
if exist('glvs', 'file') ~= 2
    psins_root = fullfile('D:\Github\PSINS', 'psins2401');
    if ~isfolder(psins_root)
        error('找不到PSINS工具箱：%s', psins_root);
    end
    addpath(genpath(psins_root));
end
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

forward_path = fullfile(output_dir, sprintf( ...
    'simple-forward-ekf-%s.nav', position_error_unit));
single_rts_path = fullfile(output_dir, sprintf( ...
    'simple-rts-single-%s.nav', position_error_unit));
double_rts_path = fullfile(output_dir, sprintf( ...
    'simple-rts-double-%s.nav', position_error_unit));
rotation_path = fullfile(output_dir, sprintf( ...
    'four-method-rts-double-rotation-%s.nav', position_error_unit));
replay_path = fullfile(output_dir, sprintf( ...
    'four-method-rts-double-position-velocity-%s.nav', ...
    position_error_unit));

navigation_results_ready = nav_file_is_ready(forward_path) && ...
    nav_file_is_ready(double_rts_path);
run_navigation_core = force_rerun_navigation_core || ...
    ~navigation_results_ready;
if run_navigation_core
    fprintf('前向或二次RTS结果缺失（或要求强制重算），开始执行导航核心。\n');
else
    fprintf('已有前向和二次RTS结果，跳过导航核心。\n');
    fprintf('复用前向结果：%s\n', forward_path);
    fprintf('复用二次RTS：%s\n', double_rts_path);
end

if run_navigation_core
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
end

%% 8. 读取方法1和方法2结果
if ~nav_file_is_ready(forward_path) || ~nav_file_is_ready(double_rts_path)
    error('导航核心结束后仍缺少前向或二次RTS结果。');
end
forward_nav = readmatrix(forward_path, 'FileType', 'text');
double_rts_nav = readmatrix(double_rts_path, 'FileType', 'text');
validate_nav_matrix(forward_nav, '前向EKF');
validate_nav_matrix(double_rts_nav, '二次RTS');

%% 9. 方法3：二次RTS分段旋转收缩
if enable_rotation_contraction
    rotation_is_current = method_result_is_current( ...
        rotation_path, double_rts_path);
    if ~overwrite_method_results && rotation_is_current
        fprintf('方法3结果已存在且不早于二次RTS，直接复用：%s\n', ...
            rotation_path);
    else
        fprintf('开始生成方法3：2RTS分段旋转收缩。\n');
        rotation_nav = double_rts_nav;
        segment_ids = unique(double_rts_nav(:, 1), 'stable');
        for segment_index = 1:max(0, numel(segment_ids) - 1)
            current_mask = double_rts_nav(:, 1) == segment_ids(segment_index);
            next_mask = double_rts_nav(:, 1) == segment_ids(segment_index + 1);
            current_segment = double_rts_nav(current_mask, :);
            next_segment = double_rts_nav(next_mask, :);
            target_position = nav_row_to_position(next_segment(1, :), param);
            rotation_nav(current_mask, :) = rotate_double_rts_segment( ...
                current_segment, target_position, param);
        end
        write_nav_file(rotation_path, rotation_nav);
        fprintf('方法3结果：%s\n', rotation_path);
    end
else
    fprintf('已关闭方法3，不生成旋转收缩结果。\n');
end

%% 10. 方法4：二次RTS水平位置速度约束的固定滞后重放
if enable_position_velocity_replay
    replay_is_current = method_result_is_current( ...
        replay_path, double_rts_path);
    if ~overwrite_method_results && replay_is_current
        fprintf('方法4结果已存在且不早于二次RTS，直接复用：%s\n', ...
            replay_path);
    else
        fprintf('开始生成方法4：2RTS水平位置速度约束重放。\n');

        % 最末段没有后一测距区间提供桥接误差，不作为固定滞后可发布段。
        final_segment_id = double_rts_nav(end, 1);
        replay_reference_nav = double_rts_nav( ...
            double_rts_nav(:, 1) < final_segment_id, :);
        if isempty(replay_reference_nav)
            error('二次RTS完整区间不足，无法执行位置速度约束重放。');
        end

        imu_all = readmatrix(cfg.imufilepath, 'FileType', 'text');
        truth_all = readmatrix(cfg.truthpath, 'FileType', 'text');
        replay_start_time = replay_reference_nav(1, 2);
        replay_end_time = replay_reference_nav(end, 2);
        imu_mask = imu_all(:, 1) >= replay_start_time - 1e-8 & ...
            imu_all(:, 1) <= replay_end_time + 1e-8;
        replay_imu = imu_all(imu_mask, :);
        if size(replay_imu, 1) ~= size(replay_reference_nav, 1) || ...
                max(abs(replay_imu(:, 1) - replay_reference_nav(:, 2))) > 1e-7
            error(['历史IMU与二次RTS时间轴不一致，不能安全重放。', ...
                '\nIMU点数=%d，RTS点数=%d。'], ...
                size(replay_imu, 1), size(replay_reference_nav, 1));
        end

        % 与 run_rts_navigation_study 保持相同随机数顺序：先生成测距噪声，
        % 再生成重放使用的深度噪声。
        rng(random_seed, 'twister');
        range_sources_for_rng = { ...
            readmatrix(cfg.rangefile1path, 'FileType', 'text'), ...
            readmatrix(cfg.rangefile2path, 'FileType', 'text'), ...
            readmatrix(cfg.rangefile3path, 'FileType', 'text')};
        source_interval_s = median(diff(range_sources_for_rng{1}(:, 1)));
        range_stride = round(range_interval_s / source_interval_s);
        for source_index = 1:numel(range_sources_for_rng)
            range_sources_for_rng{source_index} = ...
                range_sources_for_rng{source_index}( ...
                range_stride:range_stride:end, :);
        end
        event_count = min(cellfun(@(data) size(data, 1), ...
            range_sources_for_rng));
        discarded_range_noise = randn(event_count, 1);
        if numel(discarded_range_noise) ~= event_count
            error('测距噪声随机序列长度异常。');
        end
        replay_height_truth = interp1(truth_all(:, 2), truth_all(:, 5), ...
            replay_imu(:, 1), 'linear', 'extrap');
        if data_source == "simulation"
            depth_noise_std_m = simulation_depth_noise_std_m;
        else
            depth_noise_std_m = experiment_depth_std_m;
        end
        replay_height = [replay_imu(:, 1), replay_height_truth + ...
            depth_noise_std_m * randn(size(replay_height_truth))];

        replay_options = struct( ...
            'position_error_unit', position_error_unit, ...
            'filter_depth_std_m', filter_depth_std_m, ...
            'range_interval_s', range_interval_s, ...
            'rts_update_interval_s', rts_update_interval_s, ...
            'rts_position_std_m', rts_position_std_m, ...
            'rts_velocity_std_mps', rts_velocity_std_mps, ...
            'boundary_guard_s', boundary_guard_s, ...
            'boundary_position_std_m', boundary_position_std_m, ...
            'boundary_velocity_std_mps', boundary_velocity_std_mps, ...
            'innovation_gate_sigma', innovation_gate_sigma, ...
            'robust_gate_sigma', robust_gate_sigma, ...
            'robust_std_scale', robust_std_scale);
        replay_nav = replay_position_velocity( ...
            replay_imu, replay_height, replay_reference_nav, cfg, ...
            replay_options, param);
        write_nav_file(replay_path, replay_nav);
        fprintf('方法4结果：%s\n', replay_path);
    end
else
    fprintf('已关闭方法4，不生成位置速度约束重放结果。\n');
end

fprintf('\n四方法结果生成流程结束。\n');
fprintf('方法1 前向EKF：%s\n', forward_path);
fprintf('方法2 二次RTS：%s\n', double_rts_path);
if enable_rotation_contraction
    fprintf('方法3 旋转收缩：%s\n', rotation_path);
end
if enable_position_velocity_replay
    fprintf('方法4 位置速度约束：%s\n', replay_path);
end

%% 局部辅助函数
function ready = nav_file_is_ready(file_path)
    file_info = dir(file_path);
    ready = isfile(file_path) && ~isempty(file_info) && file_info.bytes > 0;
end

function current = method_result_is_current(result_path, source_path)
    if ~nav_file_is_ready(result_path) || ~nav_file_is_ready(source_path)
        current = false;
        return;
    end
    result_info = dir(result_path);
    source_info = dir(source_path);
    current = result_info.datenum >= source_info.datenum;
end

function validate_nav_matrix(nav_data, label)
    if isempty(nav_data) || size(nav_data, 2) < 11 || ...
            any(~isfinite(nav_data(:, 2))) || any(diff(nav_data(:, 2)) <= 0)
        error('%s结果为空、列数不足或时间轴不严格递增。', label);
    end
end

function write_nav_file(file_path, nav_data)
    file_id = fopen(file_path, 'wt');
    if file_id < 0
        error('无法创建导航结果：%s', file_path);
    end
    cleaner = onCleanup(@() fclose(file_id));
    format = ['%2d %12.6f %12.8f %12.8f %8.4f %8.4f ', ...
        '%8.4f %8.4f %8.4f %8.4f %8.4f\n'];
    fprintf(file_id, format, nav_data');
    clear cleaner;
end

function position = nav_row_to_position(nav_row, param)
    position = [nav_row(3:4)' * param.D2R; nav_row(5)];
end

function rotated_nav = rotate_double_rts_segment( ...
        double_rts_nav, target_position, param)
    position = [double_rts_nav(:, 3:4)' * param.D2R; ...
        double_rts_nav(:, 5)'];
    position(3, :) = 0;
    target_position(3) = 0;
    rotated_position = rotateAndScaleTrajectory(position, target_position);
    rotated_nav = double_rts_nav;
    rotated_nav(:, 3:4) = rotated_position(1:2, :)' * param.R2D;
end

function replay_nav = replay_position_velocity( ...
        imudata, height, reference_nav, cfg, options, param)
    cfg.initpos(1:2) = reference_nav(1, 3:4)' * param.D2R;
    cfg.initpos(3) = height(1, 2);
    cfg.initvel = reference_nav(1, 6:8)';
    [kf, navstate] = myInitialize_15state(cfg);
    kf.depthstd = options.filter_depth_std_m;
    horizontal_scale = [navstate.Rm + navstate.pos(3); ...
        (navstate.Rn + navstate.pos(3)) * cos(navstate.pos(1))];
    if options.position_error_unit == "rad"
        kf.P(1:2, 1:2) = diag((options.rts_position_std_m ...
            ./ horizontal_scale) .^ 2);
    else
        kf.P(1:2, 1:2) = eye(2) * options.rts_position_std_m ^ 2;
    end
    kf.P(4:5, 4:5) = eye(2) * options.rts_velocity_std_mps ^ 2;

    sample_count = size(imudata, 1);
    replay_nav = nan(sample_count, 11);
    navstate.time = imudata(1, 1);
    navstate.pos(3) = height(1, 2);
    replay_nav(1, :) = state_to_nav_row(navstate, param);
    next_update_time = ceil(imudata(1, 1) / ...
        options.rts_update_interval_s) * options.rts_update_interval_s;
    if next_update_time <= imudata(1, 1) + 1e-9
        next_update_time = next_update_time + options.rts_update_interval_s;
    end
    this_imu = imudata(1, :)';
    last_progress = -1;
    for imu_index = 2:sample_count
        last_imu = this_imu;
        this_imu = imudata(imu_index, :)';
        imu_dt = this_imu(1) - last_imu(1);
        time_tolerance = max(1e-8, abs(imu_dt) * 0.25);

        if last_imu(1) >= next_update_time - time_tolerance
            measurement_nav = interp1(reference_nav(:, 2), ...
                reference_nav(:, 3:7), last_imu(1), 'linear');
            [position_std_m, velocity_std_mps] = ...
                select_replay_measurement_std(last_imu(1), options);
            [kf, accepted] = update_replay_position_velocity( ...
                navstate, measurement_nav(1:2)' * param.D2R, ...
                measurement_nav(4:5)', kf, position_std_m, ...
                velocity_std_mps, options);
            if accepted
                if options.position_error_unit == "rad"
                    [kf, navstate] = myErrorFeedback_range(kf, navstate);
                else
                    [kf, navstate] = myErrorFeedback_range_m(kf, navstate);
                end
                replay_nav(imu_index - 1, :) = state_to_nav_row(navstate, param);
            end
            next_update_time = next_update_time + options.rts_update_interval_s;
        end

        navstate = InsMech(navstate, last_imu, this_imu);
        if options.position_error_unit == "rad"
            [kf, navstate] = update_decoupled_height( ...
                kf, navstate, height(imu_index, :));
            navstate.time = this_imu(1);
            kf = myInsPropagate_15state(navstate, this_imu, imu_dt, kf);
        else
            [kf, navstate] = update_decoupled_height_m( ...
                kf, navstate, height(imu_index, :));
            navstate.time = this_imu(1);
            kf = myInsPropagate_15state_m(navstate, this_imu, imu_dt, kf);
        end
        replay_nav(imu_index, :) = state_to_nav_row(navstate, param);

        progress = floor(10 * imu_index / sample_count) * 10;
        if progress > last_progress && mod(progress, 20) == 0
            fprintf('方法4重放进度：%d %%\n', progress);
            last_progress = progress;
        end
    end
end

function [position_std_m, velocity_std_mps] = ...
        select_replay_measurement_std(time_s, options)
    distance_to_boundary = abs(time_s - round(time_s ...
        / options.range_interval_s) * options.range_interval_s);
    in_boundary_guard = distance_to_boundary <= options.boundary_guard_s ...
        && time_s > options.boundary_guard_s;
    if in_boundary_guard
        normalized_distance = distance_to_boundary / options.boundary_guard_s;
        smooth_weight = 10 * normalized_distance ^ 3 ...
            - 15 * normalized_distance ^ 4 + 6 * normalized_distance ^ 5;
        position_std_m = options.boundary_position_std_m + ...
            (options.rts_position_std_m - options.boundary_position_std_m) ...
            * smooth_weight;
        velocity_std_mps = options.boundary_velocity_std_mps + ...
            (options.rts_velocity_std_mps ...
            - options.boundary_velocity_std_mps) * smooth_weight;
    else
        position_std_m = options.rts_position_std_m;
        velocity_std_mps = options.rts_velocity_std_mps;
    end
end

function [kf, accepted] = update_replay_position_velocity( ...
        navstate, measurement_position, measurement_velocity, kf, ...
        position_std_m, velocity_std_mps, options)
    horizontal_scale = [navstate.Rm + navstate.pos(3); ...
        (navstate.Rn + navstate.pos(3)) * cos(navstate.pos(1))];
    position_residual = navstate.pos(1:2) - measurement_position;
    if options.position_error_unit == "m"
        position_residual = horizontal_scale .* position_residual;
    end
    measurement_residual = [position_residual; ...
        navstate.vel(1:2) - measurement_velocity];
    measurement_matrix = zeros(4, kf.RANK);
    measurement_matrix(1:2, 1:2) = eye(2);
    measurement_matrix(3:4, 4:5) = eye(2);
    innovation = measurement_residual - measurement_matrix * kf.x;
    if options.position_error_unit == "m"
        innovation_m = norm(innovation(1:2));
    else
        innovation_m = norm(horizontal_scale .* innovation(1:2));
    end
    velocity_innovation_mps = norm(innovation(3:4));
    normalized_innovation = max( ...
        innovation_m / max(position_std_m, eps), ...
        velocity_innovation_mps / max(velocity_std_mps, eps));
    accepted = normalized_innovation <= options.robust_gate_sigma;
    robust_downweighted = accepted && ...
        normalized_innovation > options.innovation_gate_sigma;
    if robust_downweighted
        position_std_m = position_std_m * options.robust_std_scale;
        velocity_std_mps = velocity_std_mps * options.robust_std_scale;
    end
    if accepted
        if options.position_error_unit == "m"
            measurement_std = repmat(position_std_m, 2, 1);
        else
            measurement_std = position_std_m ./ horizontal_scale;
        end
        measurement_covariance = diag([measurement_std .^ 2; ...
            repmat(velocity_std_mps ^ 2, 2, 1)]);
        innovation_covariance = measurement_matrix * kf.P ...
            * measurement_matrix' + measurement_covariance;
        kalman_gain = kf.P * measurement_matrix' / innovation_covariance;
        kf.x = kf.x + kalman_gain * innovation;
        identity = eye(kf.RANK);
        kf.P = (identity - kalman_gain * measurement_matrix) * kf.P ...
            * (identity - kalman_gain * measurement_matrix)' ...
            + kalman_gain * measurement_covariance * kalman_gain';
        kf.P = (kf.P + kf.P') / 2;
    end
end

function row = state_to_nav_row(navstate, param)
    row = zeros(1, 11);
    row(2) = navstate.time;
    row(3:5) = [navstate.pos(1:2)' * param.R2D, navstate.pos(3)];
    row(6:8) = navstate.vel';
    row(9:11) = navstate.att' * param.R2D;
end
