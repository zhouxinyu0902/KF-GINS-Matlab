clear;
close all;
clc;
%% 长时时变潜标影响：未补偿/理想坐标导航
% 距离由真实时变潜标位置生成；滤波器始终使用固定的初始名义坐标。
% beacon_position_source="fixed-initial"时为未补偿结果；
% beacon_position_source="truth"时将真实时变坐标提供给滤波器，作为理想参考。
%% 1. 用户配置
data_source = "experiment";           % "simulation" / "experiment"
dataset_id = 'case-09';               % 数据目录名；不再限制为case-06
motion_region_mode = 'circle';         % 'circle' 或 'annulus'
activity_radius_m = 61;                % circle：0~R [m]
annulus_inner_radius_m = 40;           % annulus内半径 [m]
annulus_outer_radius_m = 200;          % annulus外半径 [m]
initial_measurement_error_max_m = 10;  % 必须与生成脚本一致 [m]
beacon_position_source = "fixed-initial"; % 默认未补偿；"truth"仅作理想参考
position_error_unit = "rad";           % "rad" 或 "m"
range_interval_s = 420;                % 测距间隔：7 min
start_time_s = [];                     % []：自动取公共起点
duration_s = 24*3600;                  % []：使用全部公共时段
beacon_order = [1, 2, 3];              % 三个信标固定轮换顺序
range_noise_std_m = 6;
depth_noise_std_m = 0.4;
enable_feedback = true;
enable_smoothing = true;
enable_second_rts = true;
random_seed = 1;
if initial_measurement_error_max_m < 0
    error('initial_measurement_error_max_m不能为负数。');
end
beacon_position_source = lower(beacon_position_source);
switch beacon_position_source
    case "fixed-initial"
        result_variant = 'uncompensated';
        compensation_mode = 'uncompensated-fixed-initial-beacon';
    case "truth"
        result_variant = 'truth-beacon-reference';
        compensation_mode = 'ideal-true-time-varying-beacon';
    otherwise
        error('beacon_position_source只能设置为"fixed-initial"或"truth"。');
end
switch lower(motion_region_mode)
    case 'circle'
        if activity_radius_m <= 0
            error('activity_radius_m必须大于0。');
        end
        study_id = sprintf( ...
            'beacon-position-time-varying-circle-%gm-initial%gm', ...
            activity_radius_m, initial_measurement_error_max_m);
    case 'annulus'
        if annulus_inner_radius_m < 0 || ...
                annulus_outer_radius_m <= annulus_inner_radius_m
            error('圆环内外半径设置错误。');
        end
        study_id = sprintf( ...
            'beacon-position-time-varying-annulus-%g-%gm-initial%gm', ...
            annulus_inner_radius_m, annulus_outer_radius_m, ...
            initial_measurement_error_max_m);
    otherwise
        error('motion_region_mode只能设置为circle或annulus。');
end
%% 2. 初始化路径、算法配置和输出目录
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));
addpath(topic_dir);
setup_inertial_experiment();
param = Param();
glvs;
rng(random_seed, 'twister');
position_error_unit = lower(position_error_unit);
if ~ismember(position_error_unit, ["rad", "m"])
    error('position_error_unit只能设置为"rad"或"m"。');
end
if enable_smoothing && ~enable_feedback
    error('执行 RTS 时必须启用 enable_feedback。');
end
if enable_second_rts && ~enable_smoothing
    error('enable_second_rts=true 时必须启用 enable_smoothing。');
end
dataset = resolve_beacon_position_dataset( ...
    data_source, dataset_id, position_error_unit);
case_name = sprintf('%s/%s',dataset.data_source,dataset.dataset_id);
input_dir = dataset.input_dir;
study_input_dir = fullfile(input_dir, study_id);
cfg = dataset.cfg;
output_dir = fullfile(cfg.outputfolder, study_id, sprintf( ...
    '%s-%s', result_variant, position_error_unit));
filter_range_std_m = range_noise_std_m;
filter_depth_std_m = depth_noise_std_m;
if ~isfolder(input_dir)
    error('数据集输入目录不存在：%s', input_dir);
end
if ~isfolder(study_input_dir)
    error(['缺少时变潜标输入：%s\n请先运行 ', ...
        'generate_case06_time_varying_beacon_data.m。'], study_input_dir);
end
generation_context_path = fullfile(study_input_dir, 'generation-context.mat');
if ~isfile(generation_context_path)
    error('缺少数据生成配置：%s', generation_context_path);
end
loaded_generation = load(generation_context_path, 'generation_context');
generation_context = loaded_generation.generation_context;
if ~isfield(generation_context, 'version') || generation_context.version < 4 || ...
        ~strcmp(generation_context.data_source, dataset.data_source) || ...
        ~strcmp(generation_context.dataset_id, dataset.dataset_id)
    error('生成配置与当前data_source/dataset_id不一致，请重新生成该工况。');
end
if abs(generation_context.initial_measurement_error_max_m- ...
        initial_measurement_error_max_m) > 1e-12
    error('初始潜标误差上限与generation-context.mat不一致。');
end

if ~isfolder(output_dir)
    mkdir(output_dir);
end
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
range_sources = {
    readmatrix(fullfile(study_input_dir, 'range1.txt'), ...
        'FileType', 'text'), ...
    readmatrix(fullfile(study_input_dir, 'range2.txt'), ...
        'FileType', 'text'), ...
    readmatrix(fullfile(study_input_dir, 'range3.txt'), ...
        'FileType', 'text')};
beacon_truth_paths = strings(3, 1);
beacon_truth_sources = cell(3, 1);
if beacon_position_source == "truth"
    for source_index = 1:3
        beacon_truth_paths(source_index) = fullfile(study_input_dir, ...
            sprintf('beacon%d-position-truth.txt', source_index));
        if ~isfile(beacon_truth_paths(source_index))
            error('缺少潜标%d真实位置文件：%s',source_index, ...
                beacon_truth_paths(source_index));
        end
        beacon_truth_sources{source_index} = readmatrix( ...
            beacon_truth_paths(source_index),'FileType','text');
    end
end
source_interval_s = median(diff(range_sources{1}(:, 1)));
range_stride = round(range_interval_s/source_interval_s);
if range_stride < 1 || ...
        abs(range_stride*source_interval_s-range_interval_s) > 1e-6
    error('测距间隔 %.3f s 不是时变潜标数据间隔 %.3f s 的整数倍。', ...
        range_interval_s, source_interval_s);
end
for source_index = 1:numel(range_sources)
    if size(range_sources{source_index}, 2) < 6 || ...
            any(diff(range_sources{source_index}(:, 1)) <= 0)
        error('时变潜标 range%d.txt 格式或时间轴无效。', source_index);
    end
    if beacon_position_source == "truth"
        beacon_truth = beacon_truth_sources{source_index};
        if size(beacon_truth, 2) < 4 || ...
                size(beacon_truth, 1) ~= size(range_sources{source_index}, 1) || ...
                any(~isfinite(beacon_truth(:, 1:4)), 'all') || ...
                any(abs(beacon_truth(:, 1)- ...
                range_sources{source_index}(:, 1)) > 1e-8)
            error('潜标%d真实位置文件格式或时间轴无效。',source_index);
        end
        beacon_truth_sources{source_index} = beacon_truth( ...
            range_stride:range_stride:end, :);
    end
    range_sources{source_index} = range_sources{source_index}( ...
        range_stride:range_stride:end, :);
end
event_count = min(cellfun(@(data) size(data, 1), range_sources));
rangedata = zeros(event_count, size(range_sources{1}, 2));
for event_index = 1:event_count
    order_index = mod(event_index-1, numel(beacon_order))+1;
    source_index = beacon_order(order_index);
    rangedata(event_index, :) = range_sources{source_index}(event_index, :);
    if beacon_position_source == "truth"
        % 真值文件第2~4列为当前时刻潜标的[纬度rad，经度rad，高度m]。
        rangedata(event_index, 4:6) = ...
            beacon_truth_sources{source_index}(event_index, 2:4);
    end
end
% 第3列在生成阶段仍为理想真实距离；这里加入与基准实验相同的白噪声。
% fixed-initial模式保持第4~6列不变；truth模式已替换为同历元真实坐标。
rangedata(:, 3) = rangedata(:, 3)+ ...
    range_noise_std_m*randn(size(rangedata, 1), 1);
% cfg中的初始位置、速度和姿态只对应cfg.starttime。首个测距通常晚于
% 初始化时刻，不能用rangedata(1,1)推迟导航起点，否则会在错误时刻套用
% 旧初始状态。测距只需落在导航时段内，不参与起止时刻的确定。
data_start_time = max(imudata_all(1, 1), truth(1, 2));
initialization_time = max(cfg.starttime, data_start_time);
time_tolerance_s = max(1e-8, median(diff(imudata_all(:, 1)))*0.25);
if isempty(start_time_s)
    start_time = initialization_time;
else
    if abs(start_time_s-cfg.starttime) > time_tolerance_s
        error(['start_time_s=%.6f与配置初始状态时刻cfg.starttime=%.6f不一致。' ...
            '若要更换起点，必须同时在navigation-config.mat中更新starttime、' ...
            'initpos、initvel和initatt。'],start_time_s,cfg.starttime);
    end
    start_time = max(start_time_s, data_start_time);
end
available_end_time = min(imudata_all(end, 1), truth(end, 2));
if isempty(duration_s)
    end_time = available_end_time;
else
    end_time = min(start_time+duration_s, available_end_time);
end
if start_time >= end_time
    error('配置初始时刻不在IMU和真值数据的公共时间范围内。');
end
cfg.starttime = start_time;
cfg.endtime = end_time;
imu_mask = imudata_all(:, 1) >= start_time & imudata_all(:, 1) <= end_time;
imudata = imudata_all(imu_mask, :);
range_mask = rangedata(:, 1) >= start_time & rangedata(:, 1) <= end_time;
rangedata = rangedata(range_mask, :);
height_value = interp1(truth(:, 2), truth(:, 5), imudata(:, 1), ...
    'linear', 'extrap');
height = [imudata(:, 1), height_value+ ...
    depth_noise_std_m*randn(size(height_value))];
if isempty(rangedata)
    error('当前时间范围内没有测距事件。');
end
fprintf('导航初始化：%.2f s；首个测距：%.2f s（延迟%.2f s）。\n', ...
    start_time,rangedata(1,1),rangedata(1,1)-start_time);
%% 4. 设置导航结果文件（不复制truth）
forward_path = fullfile(output_dir, sprintf('simple-forward-ekf-%s.nav', position_error_unit));
single_rts_path = fullfile(output_dir, sprintf('simple-rts-single-%s.nav', position_error_unit));
double_rts_path = fullfile(output_dir, sprintf('simple-rts-double-%s.nav', position_error_unit));
nav_format = ['%2d %12.6f %12.8f %12.8f %8.4f %8.4f ', '%8.4f %8.4f %8.4f %8.4f %8.4f\n'];
forward_fp = fopen(forward_path, 'wt');
if forward_fp < 0
    error('无法创建前向导航结果：%s', forward_path);
end
single_rts_fp = -1;
double_rts_fp = -1;
if enable_smoothing
    single_rts_fp = fopen(single_rts_path, 'wt');
    if single_rts_fp < 0
        fclose(forward_fp);
        error('无法创建一次RTS结果：%s', single_rts_path);
    end
    if enable_second_rts
        double_rts_fp = fopen(double_rts_path, 'wt');
        if double_rts_fp < 0
            fclose(forward_fp);
            fclose(single_rts_fp);
            error('无法创建二次RTS结果：%s', double_rts_path);
        end
    end
end
%% 5. 初始化ES-EKF和RTS缓存
[kf, navstate] = myInitialize_15state(cfg);
kf.rangstd = filter_range_std_m;
kf.depthstd = filter_depth_std_m;
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
fprintf(['开始处理 %s 时变潜标影响（%s位置误差状态）：', ...
    '前向EKF=%d，一次RTS=%d，二次RTS=%d。\n'], ...
    case_name, position_error_unit, enable_feedback, enable_smoothing, ...
    enable_smoothing && enable_second_rts);
tic;
%% 6. 主循环：结构与 run_all_real_datasets_rtsfix.m 保持一致
for imu_index = 2:size(imudata, 1)
    last_imu = this_imu;
    this_imu = imudata(imu_index, :)';
    imu_dt = this_imu(1) - last_imu(1);
    time_tolerance = max(1e-8, abs(imu_dt) * 0.25);

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

    %% 6.4 保存组合导航结果
    nav_row = [0; navstate.time; navstate.pos(1:2) * param.R2D; navstate.pos(3); navstate.vel; navstate.att * param.R2D];
    fprintf(forward_fp, nav_format, nav_row);
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
fclose(forward_fp);
if single_rts_fp >= 0
    fclose(single_rts_fp);
end
if double_rts_fp >= 0
    fclose(double_rts_fp);
end
elapsed_time_s = toc;
navigation_context = struct();
navigation_context.version = 3;
navigation_context.data_source = dataset.data_source;
navigation_context.dataset_id = dataset.dataset_id;
navigation_context.study_id = study_id;
navigation_context.compensation_mode = compensation_mode;
navigation_context.beacon_position_source = char(beacon_position_source);
navigation_context.beacon_truth_paths = beacon_truth_paths;
navigation_context.motion_region_mode = motion_region_mode;
navigation_context.initial_measurement_error_max_m = ...
    initial_measurement_error_max_m;
navigation_context.position_error_unit = char(position_error_unit);
navigation_context.start_time_s = cfg.starttime;
navigation_context.end_time_s = cfg.endtime;
navigation_context.duration_s = cfg.endtime-cfg.starttime;
navigation_context.range_interval_s = range_interval_s;
navigation_context.beacon_order = beacon_order;
navigation_context.range_noise_std_m = range_noise_std_m;
navigation_context.depth_noise_std_m = depth_noise_std_m;
navigation_context.study_input_dir = study_input_dir;
navigation_context.generation_context_path = generation_context_path;
navigation_context.truth_source_path = cfg.truthpath;
navigation_context.forward_path = forward_path;
navigation_context.single_rts_path = single_rts_path;
navigation_context.double_rts_path = double_rts_path;
save(fullfile(output_dir, 'navigation-context.mat'), ...
    'navigation_context');
fprintf('处理完成，耗时 %.2f s。\n', elapsed_time_s);
fprintf('前向EKF：%s\n', forward_path);
if enable_smoothing
    fprintf('一次RTS：%s\n', single_rts_path);
end
if enable_smoothing && enable_second_rts
    fprintf('二次RTS：%s\n', double_rts_path);
end
