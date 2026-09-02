clear; % 归档的BRC/延迟几何早期脚本
close all;

%% 事件驱动的轮换信标测距/惯导仿真
% 每个测距时刻的处理顺序：
%   1）利用当前测距观测更新前向 15 状态 EKF；
%   2）在当前测距点到达后，对“上一测距点—当前测距点”区间进行反向推算，
%      反向过程只执行惯导机械编排，不进行反向滤波或测距更新；
%   3）利用当前反推得到的 t0-420 s 节点，延迟修正上一段红色反推轨迹
%      [t0-840 s, t0-420 s]：固定其起点，把终点变换到红色反推节点。

rng(1);                              % 固定随机数种子，保证仿真噪声可以复现。
glvs;                                % 初始化 PSINS 使用的地球模型全局常量。
param = Param();

options.case_name = 'case-01';
options.range_interval_s = 420;
% 第11个测距点位于4620 s，再保留1 s用于观察更新后的导航响应。
options.end_time_s = 4621;
options.beacon_order = [1, 2, 3];
options.range_noise_std_m = 10;
options.depth_noise_std_m = 0.4;
options.filter_range_std_m = 10;
options.filter_depth_std_m = 0.4;
options.use_adaptive_forward_update = true;
options.show_input_figure = true;

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
project_root = fileparts(fileparts(topic_dir));
input_dir = fullfile(project_root, 'data', 'inertial-experiment', ...
    'algorithm-exploration', 'input', 'simulation', options.case_name);

cfg = ProcessConfigforSimu(input_dir);
cfg.userange = true;
cfg.outputfolder = fullfile(project_root, 'data', 'inertial-experiment', ...
    'algorithm-exploration', 'navigation-results', 'simulation', 'forward-backward');
if ~exist(cfg.outputfolder, 'dir')
    mkdir(cfg.outputfolder);
end

%% 读取并整理观测数据
imu_all = importdata(cfg.imufilepath);
truth_all = importdata(cfg.truthpath);
range_sources = {
    importdata(cfg.rangefile1path), ...
    importdata(cfg.rangefile2path), ...
    importdata(cfg.rangefile3path)};

source_dt = median(diff(range_sources{1}(:, 1)));
range_step = round(options.range_interval_s / source_dt);
if abs(range_step * source_dt - options.range_interval_s) > 1e-6
    error('测距间隔 %.3f s 不是原始测距采样间隔 %.3f s 的整数倍。', ...
        options.range_interval_s, source_dt);
end

% 三个信标先按相同时间间隔降采样，再按照 1→2→3→1……的固定顺序轮换。
for source_index = 1:numel(range_sources)
    range_sources{source_index} = range_sources{source_index}( ...
        range_step:range_step:end, :);
end
event_count = min(cellfun(@(x) size(x, 1), range_sources));
rangedata = zeros(event_count, size(range_sources{1}, 2));
for event_index = 1:event_count
    order_index = mod(event_index - 1, numel(options.beacon_order)) + 1;
    source_index = options.beacon_order(order_index);
    rangedata(event_index, :) = range_sources{source_index}(event_index, :);
end
rangedata(:, 3) = rangedata(:, 3) + ...
    options.range_noise_std_m * randn(event_count, 1);

start_time = max([cfg.starttime, imu_all(1, 1), truth_all(1, 2)]);
end_time = min([options.end_time_s, cfg.endtime, ...
    imu_all(end, 1), truth_all(end, 2)]);
cfg.starttime = start_time;
cfg.endtime = end_time;

imu_mask = imu_all(:, 1) >= start_time & imu_all(:, 1) <= end_time;
imudata = imu_all(imu_mask, :);
range_mask = rangedata(:, 1) >= start_time & rangedata(:, 1) <= end_time;
rangedata = rangedata(range_mask, :);

% 将真值高度插值到 IMU 时刻，避免真值文件和 IMU 文件行号不完全一致时错位。
height_values = interp1(truth_all(:, 2), truth_all(:, 5), imudata(:, 1), ...
    'linear', 'extrap');
height = [imudata(:, 1), height_values + ...
    options.depth_noise_std_m * randn(size(height_values))];

if options.show_input_figure
    figure('Name', 'Simulation input');
    plot(truth_all(:, 3), truth_all(:, 4), 'LineWidth', 1.2);
    hold on;
    plot(rangedata(:, 5) * param.R2D, rangedata(:, 4) * param.R2D, ...
        'o', 'MarkerSize', 5);
    axis equal;
    grid on;
    xlabel('经度（°）');
    ylabel('纬度（°）');
    legend('真实轨迹', '当前使用的信标', 'Location', 'best');
    title(sprintf('三信标轮换，测距间隔 = %.0f s', ...
        options.range_interval_s));
end

%% 初始化前向滤波器和结果缓存
[kf, navstate] = myInitialize_15state(cfg);
kf.rangstd = options.filter_range_std_m;
kf.depthstd = options.filter_depth_std_m;

sample_count = size(imudata, 1);
forward_nav = nan(sample_count, 11);
backward_position = nan(3, sample_count);
delayed_geometry_position = nan(3, sample_count);
segment_is_processed = false(sample_count, 1);
delayed_geometry_is_processed = false(sample_count, 1);

navstate.time = imudata(1, 1);
forward_nav(1, :) = state_to_nav_row(navstate, param);

range_index = find(rangedata(:, 1) >= imudata(1, 1), 1, 'first');
if isempty(range_index)
    range_index = size(rangedata, 1) + 1;
end

previous_anchor = struct('imu_index', [], 'position', []);
previous_segment = struct('indices', [], 'diagnostic_index', []);
diagnostics = repmat(empty_diagnostic(), 0, 1);
last_progress = -1;

%% 主循环：按 IMU 递推，在测距点到达时处理上一测距区间
this_imu = imudata(1, :)';
for imu_index = 2:sample_count
    last_imu = this_imu;
    this_imu = imudata(imu_index, :)';
    imu_dt = this_imu(1) - last_imu(1);

    % 当前导航状态对应 last_imu 时刻。使用时间容差判断测距事件，
    % 避免直接比较浮点时间造成漏检。
    time_tolerance = max(1e-8, abs(imu_dt) * 0.25);
    while range_index <= size(rangedata, 1) && ...
            rangedata(range_index, 1) < last_imu(1) - time_tolerance
        warning('测距时刻 %.6f s 未与 IMU 对齐，已跳过该观测。', rangedata(range_index, 1));
        range_index = range_index + 1;
    end

    is_range_epoch = cfg.userange && range_index <= size(rangedata, 1) && ...
        abs(last_imu(1) - rangedata(range_index, 1)) <= time_tolerance;

    if is_range_epoch
        anchor_imu_index = imu_index - 1;
        current_range = rangedata(range_index, :);
        current_depth = height(anchor_imu_index, :);

        % 当前测距事件：先校正前向轨迹的当前端点。
        kf = update_range_filter(navstate, current_range, current_depth, kf, ...
            options.use_adaptive_forward_update);
        [kf, navstate] = myErrorFeedback_range(kf, navstate);
        forward_nav(anchor_imu_index, :) = state_to_nav_row(navstate, param);
        current_anchor_position = navstate.pos;

        % 只有获得前后两个真实测距点后，才具备一个完整的双端点约束区间。
        if ~isempty(previous_anchor.imu_index)
            segment_indices = previous_anchor.imu_index:anchor_imu_index;
            [backward_segment, info] = ...
                process_range_event_segment( ...
                imudata(segment_indices, :), height(segment_indices, :), navstate, ...
                previous_anchor.position);

            % 延迟上一段红色轨迹修正：时刻 t0 得到当前红色反推终点
            % （t0-420 s 节点）后，不修改当前红色区间，而是修正上一周期
            % 已保存的红色反推轨迹 [t0-840 s, t0-420 s]。固定该红色
            % 轨迹的 t0-840 s 起点，并把其 t0-420 s 终点变换到当前
            % 反向推算得到的 t0-420 s 参考点。
            if ~isempty(previous_segment.indices)
                previous_backward_position = ...
                    backward_position(:, previous_segment.indices);
                [corrected_previous_segment, transform_info] = ...
                    transform_previous_backward_segment( ...
                    previous_backward_position, backward_segment(:, 1));
                write_indices = previous_segment.indices;
                write_columns = 1:numel(write_indices);
                % 相邻区间共享一个测距历元。该历元归属于较早区间的终点，
                % 避免下一段旋转再次覆盖同一个时间点。
                if delayed_geometry_is_processed(write_indices(1))
                    write_indices = write_indices(2:end);
                    write_columns = write_columns(2:end);
                end
                delayed_geometry_position(:, write_indices) = ...
                    corrected_previous_segment(:, write_columns);
                delayed_geometry_is_processed(write_indices) = true;

                diagnostic_index = previous_segment.diagnostic_index;
                diagnostics(diagnostic_index).delayed_rotation_angle_deg = ...
                    transform_info.rotation_angle_deg;
                diagnostics(diagnostic_index).delayed_scale = ...
                    transform_info.scale_factor;
                diagnostics(diagnostic_index).delayed_endpoint_gap_before_m = ...
                    transform_info.endpoint_gap_before_m;
                diagnostics(diagnostic_index).delayed_endpoint_gap_after_m = ...
                    transform_info.endpoint_gap_after_m;
            end

            backward_position(:, segment_indices) = backward_segment;
            segment_is_processed(segment_indices) = true;

            info.segment_index = numel(diagnostics) + 1;
            info.start_range_index = range_index - 1;
            info.end_range_index = range_index;
            diagnostics(end + 1, 1) = info; %#ok<SAGROW>
            previous_segment.indices = segment_indices;
            previous_segment.diagnostic_index = numel(diagnostics);
        end

        previous_anchor.imu_index = anchor_imu_index;
        previous_anchor.position = current_anchor_position;
        range_index = range_index + 1;
    end

    % 如果上一历元发生了测距更新，则从量测校正后的状态继续前向递推。
    last_state = navstate;
    navstate = InsMech(last_state, last_imu, this_imu);
    % 与 all_m.m 一致：非测距历元进行解耦高度 Kalman 更新，只反馈
    % 垂向位置和垂向速度；测距历元的高度已包含在距离联合量测中。
    if ~is_range_epoch
        [kf, navstate] = update_decoupled_height( ...
            kf, navstate, height(imu_index, :));
    end
    kf = myInsPropagate_15state(navstate, this_imu, imu_dt, kf);
    forward_nav(imu_index, :) = state_to_nav_row(navstate, param);

    progress = floor(10 * imu_index / sample_count) * 10;
    if progress > last_progress && mod(progress, 20) == 0
        fprintf('处理进度：%d %%\n', progress);
        last_progress = progress;
    end
end

%% 生成长度一致、可以直接比较的三类结果
% 没有前后两个测距点约束的区间直接保留前向结果。
backward_nav = forward_nav;
delayed_geometry_nav = forward_nav;
processed_indices = find(segment_is_processed);
backward_nav(processed_indices, 3:5) = position_to_output_units( ...
    backward_position(:, processed_indices), param);
% 当前区间保持红色纯反向结果；只有在下一个测距点到达后，上一段才被
% 紫色延迟几何结果覆盖。
delayed_geometry_nav(processed_indices, 3:5) = position_to_output_units( ...
    backward_position(:, processed_indices), param);
delayed_indices = find(delayed_geometry_is_processed);
delayed_geometry_nav(delayed_indices, 3:5) = position_to_output_units( ...
    delayed_geometry_position(:, delayed_indices), param);

forward_path = fullfile(cfg.outputfolder, 'range-ins-forward.nav');
backward_path = fullfile(cfg.outputfolder, 'range-ins-backward-constrained.nav');
delayed_geometry_path = fullfile(cfg.outputfolder, ...
    'range-ins-delayed-previous-geometry.nav');
diagnostic_path = fullfile(exploration_artifact_dir(cfg.outputfolder), ...
    'range-segment-diagnostics.csv');

write_nav_file(forward_path, forward_nav);
write_nav_file(backward_path, backward_nav);
write_nav_file(delayed_geometry_path, delayed_geometry_nav);
write_diagnostics(diagnostic_path, diagnostics);

fprintf('已完成 %d 个相邻测距区间的反向约束处理。\n', numel(diagnostics));
fprintf('前向滤波结果：%s\n', forward_path);
fprintf('纯反向重建结果：%s\n', backward_path);
fprintf('延迟上一段几何修正结果：%s\n', delayed_geometry_path);

%% 脚本局部辅助函数
function kf = update_range_filter(navstate, range_data, depth_data, kf, use_adaptive)
    if use_adaptive
        kf = myRangeUpdate_adap(navstate, range_data, depth_data, kf);
    else
        kf = myRangeUpdate(navstate, range_data, depth_data, kf);
    end
end

function row = state_to_nav_row(navstate, param)
    row = zeros(1, 11);
    row(2) = navstate.time;
    row(3:5) = [navstate.pos(1:2)' * param.R2D, navstate.pos(3)];
    row(6:8) = navstate.vel';
    row(9:11) = navstate.att' * param.R2D;
end

function values = position_to_output_units(position, param)
    values = [position(1:2, :)' * param.R2D, position(3, :)'];
end

function write_nav_file(file_path, nav_data)
    file_id = fopen(file_path, 'w');
    if file_id < 0
        error('无法打开导航结果文件：%s', file_path);
    end
    format = ['%2d %12.6f %12.8f %12.8f %8.4f %8.4f ', ...
        '%8.4f %8.4f %8.4f %8.4f %8.4f\n'];
    fprintf(file_id, format, nav_data');
    fclose(file_id);
end

function item = empty_diagnostic()
    item = struct( ...
        'segment_index', 0, ...
        'start_range_index', 0, ...
        'end_range_index', 0, ...
        'start_time', nan, ...
        'end_time', nan, ...
        'sample_count', 0, ...
        'backward_start_gap_m', nan, ...
        'delayed_scale', nan, ...
        'delayed_rotation_angle_deg', nan, ...
        'delayed_endpoint_gap_before_m', nan, ...
        'delayed_endpoint_gap_after_m', nan);
end

function write_diagnostics(file_path, diagnostics)
    file_id = fopen(file_path, 'w');
    if file_id < 0
        error('无法打开区间诊断文件：%s', file_path);
    end
    fprintf(file_id, ['segment,start-range,end-range,start-time,end-time,samples,', ...
        'backward-start-gap-m,delayed-rotation-angle-deg,', ...
        'delayed-gap-before-m,delayed-gap-after-m,delayed-scale\n']);
    for index = 1:numel(diagnostics)
        item = diagnostics(index);
        fprintf(file_id, ['%d,%d,%d,%.6f,%.6f,%d,%.9f,%.9f,%.9f,', ...
            '%.9f,%.9f\n'], ...
            item.segment_index, item.start_range_index, item.end_range_index, ...
            item.start_time, item.end_time, item.sample_count, ...
            item.backward_start_gap_m, ...
            item.delayed_rotation_angle_deg, ...
            item.delayed_endpoint_gap_before_m, ...
            item.delayed_endpoint_gap_after_m, ...
            item.delayed_scale);
    end
    fclose(file_id);
end
