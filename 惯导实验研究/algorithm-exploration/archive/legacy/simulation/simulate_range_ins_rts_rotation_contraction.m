clear; % 归档的二次RTS旋转收缩独立版本
close all;

%% 二次 RTS 轨迹基础上的残余桥接旋转收缩对比仿真
% 每个测距时刻的处理顺序：
%   1）利用当前测距观测更新前向 15 状态 EKF；
%   2）在当前测距点到达后，对“上一测距点—当前测距点”区间进行反向推算，
%      反向过程只执行惯导机械编排，不进行反向滤波或测距更新；
%   3）利用当前反推得到的 t0-420 s 节点，延迟修正上一段红色反推轨迹
%      [t0-840 s, t0-420 s]：固定其起点，把终点变换到红色反推节点。
%   4）参考 experiment/FGO_range_ins_rad.m 的分段桥接方法，对前向 AEKF
%      分别执行一次 RTS 和二次 RTS 平滑，并与前三种结果统一比较。
%   5）RC-2RTS 只以已经生成的二次 RTS 轨迹为输入：利用下一段二次
%      RTS 起点形成的残余桥接节点，旋转收缩前一段二次 RTS 轨迹，
%      固定前一段起点并将终点对齐到该节点。

rng(1);                              % 固定随机数种子，保证仿真噪声可以复现。
glvs;                                % 初始化 PSINS 使用的地球模型全局常量。
param = Param();

options.case_name = 'case-01';
options.range_interval_s = 420;
options.beacon_order = [1, 2, 3];
options.range_noise_std_m = 10;
options.depth_noise_std_m = 0.4;
options.filter_range_std_m = 10;
options.filter_depth_std_m = 0.4;
options.use_adaptive_forward_update = true;
options.show_input_figure = false;
% RTS 默认每 1 s 保存一个关键节点；协方差仍在 100 Hz IMU 历元连续传播。
% 改为 0.01 可执行逐 IMU 历元 RTS，但计算时间和内存占用会显著增加。
options.rts_node_interval_s = 1;
% 第11个测距点位于4620 s，再保留1 s用于观察更新后的导航响应。
options.end_time_s = 4621;

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
project_root = fileparts(fileparts(topic_dir));
% 允许从任意工作目录独立运行本脚本。
addpath(topic_dir);
setup_inertial_experiment();
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
rts_single_nav = nan(sample_count, 11);
rts_double_nav = nan(sample_count, 11);
rts_rotation_nav = nan(sample_count, 11);
rts_single_is_processed = false(sample_count, 1);
rts_double_is_processed = false(sample_count, 1);
rts_rotation_is_processed = false(sample_count, 1);

navstate.time = imudata(1, 1);
forward_nav(1, :) = state_to_nav_row(navstate, param);

range_index = find(rangedata(:, 1) >= imudata(1, 1), 1, 'first');
if isempty(range_index)
    range_index = size(rangedata, 1) + 1;
end

previous_anchor = struct('imu_index', [], 'position', []);
previous_segment = struct('indices', [], 'diagnostic_index', []);
diagnostics = repmat(empty_diagnostic(), 0, 1);
rts_diagnostics = repmat(empty_rts_diagnostic(), 0, 1);
rts_block = initialize_rts_block(1, kf.P, kf.RANK);
previous_rts_block = [];
previous_double_block = [];
rts_rotation_diagnostics = repmat(empty_rotation_diagnostic(), 0, 1);
rts_phi_accum = eye(kf.RANK);
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

        % 把测距时刻作为当前 RTS 区间的最后一个关键节点。这里保存的是
        % 测距更新前的预测协方差和名义状态。
        [rts_block, ~] = append_rts_node_if_needed( ...
            rts_block, anchor_imu_index, kf.P, rts_phi_accum);

        % 当前测距事件：先计算末端误差，但暂不反馈，供一次 RTS 使用。
        kf = update_range_filter(navstate, current_range, current_depth, kf, ...
            options.use_adaptive_forward_update);
        terminal_error = kf.x;

        % 一次 RTS：把当前测距更新产生的末端误差反向平滑到当前区间。
        rts_segment_indices = rts_block.start_index:anchor_imu_index;
        nominal_segment_nav = forward_nav(rts_segment_indices, :);
        [single_segment_nav, bridge_error, first_error_nodes] = ...
            smooth_rts_block(rts_block, terminal_error, nominal_segment_nav, ...
            rts_segment_indices, param);
        [rts_single_nav, rts_single_is_processed] = write_segment_result( ...
            rts_single_nav, rts_single_is_processed, rts_segment_indices, ...
            single_segment_nav);

        % 二次 RTS：当前区间一次平滑传到起点的桥接误差，作为上一段的
        % 末端条件，再对上一段已经一次平滑的轨迹执行一次 RTS。
        if ~isempty(previous_rts_block)
            [double_previous_nav, residual_bridge_error] = smooth_rts_block( ...
                previous_rts_block, bridge_error, ...
                previous_rts_block.single_nav, ...
                previous_rts_block.segment_indices, param);
            [rts_double_nav, rts_double_is_processed] = write_segment_result( ...
                rts_double_nav, rts_double_is_processed, ...
                previous_rts_block.segment_indices, double_previous_nav);
            previous_index = previous_rts_block.diagnostic_index;
            rts_diagnostics(previous_index).second_pass_terminal_error_m = ...
                horizontal_error_norm(bridge_error, ...
                previous_rts_block.end_position);

            % 残余桥接旋转收缩：previous_double_block.nav 是已经完成的
            % 二次 RTS 轨迹。RC-2RTS 仅在它的基础上执行几何变换，不以
            % 前向 EKF、一次 RTS 或 BRC 轨迹作为旋转收缩输入。
            if ~isempty(previous_double_block)
                target_position = nav_row_to_position( ...
                    double_previous_nav(1, :), param);
                [rotated_previous_nav, rotation_info] = ...
                    rotate_double_rts_segment( ...
                    previous_double_block.nav, target_position, param);
                [rts_rotation_nav, rts_rotation_is_processed] = ...
                    write_segment_result(rts_rotation_nav, ...
                    rts_rotation_is_processed, ...
                    previous_double_block.segment_indices, ...
                    rotated_previous_nav);

                rotation_item = empty_rotation_diagnostic();
                rotation_item.block_index = ...
                    previous_double_block.block_index;
                rotation_item.start_time = ...
                    previous_double_block.nav(1, 2);
                rotation_item.end_time = ...
                    previous_double_block.nav(end, 2);
                rotation_item.residual_bridge_error_m = ...
                    horizontal_error_norm(residual_bridge_error, ...
                    target_position);
                rotation_item.rotation_angle_deg = ...
                    rotation_info.rotation_angle_deg;
                rotation_item.scale_factor = rotation_info.scale_factor;
                rotation_item.endpoint_gap_before_m = ...
                    rotation_info.endpoint_gap_before_m;
                rotation_item.endpoint_gap_after_m = ...
                    rotation_info.endpoint_gap_after_m;
                rts_rotation_diagnostics(end + 1, 1) = ...
                    rotation_item; %#ok<SAGROW>
            end

            previous_double_block = struct( ...
                'nav', double_previous_nav, ...
                'segment_indices', previous_rts_block.segment_indices, ...
                'block_index', previous_index);
        end

        rts_info = empty_rts_diagnostic();
        rts_info.block_index = numel(rts_diagnostics) + 1;
        rts_info.start_time = imudata(rts_segment_indices(1), 1);
        rts_info.end_time = imudata(rts_segment_indices(end), 1);
        rts_info.sample_count = numel(rts_segment_indices);
        rts_info.node_count = numel(rts_block.node_indices);
        rts_info.terminal_error_m = horizontal_error_norm( ...
            terminal_error, navstate.pos);
        rts_info.first_pass_bridge_error_m = horizontal_error_norm( ...
            bridge_error, nav_row_to_position(single_segment_nav(1, :), param));
        rts_diagnostics(end + 1, 1) = rts_info; %#ok<SAGROW>

        previous_rts_block = rts_block;
        previous_rts_block.single_nav = single_segment_nav;
        previous_rts_block.segment_indices = rts_segment_indices;
        previous_rts_block.diagnostic_index = numel(rts_diagnostics);
        previous_rts_block.end_position = navstate.pos;
        previous_rts_block.first_error_nodes = first_error_nodes;

        % 将当前测距误差反馈到名义导航状态，供后续前向和反向推算使用。
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

        % 以测距反馈后的状态和后验协方差建立下一 RTS 区间。
        rts_block = initialize_rts_block(anchor_imu_index, kf.P, kf.RANK);
        rts_phi_accum = eye(kf.RANK);
        range_index = range_index + 1;
    end

    % 如果上一历元发生了测距更新，则从量测校正后的状态继续前向递推。
    last_state = navstate;
    navstate = InsMech(last_state, last_imu, this_imu);
    height_transition = eye(kf.RANK);
    % 与 all_m.m 一致：非测距历元进行解耦高度 Kalman 更新，只反馈
    % 垂向位置和垂向速度；测距历元的高度已包含在距离联合量测中。
    if ~is_range_epoch
        [kf, navstate, height_transition] = update_decoupled_height( ...
            kf, navstate, height(imu_index, :));
    end
    kf = myInsPropagate_15state(navstate, this_imu, imu_dt, kf);
    forward_nav(imu_index, :) = state_to_nav_row(navstate, param);

    % 聚合相邻 RTS 关键节点之间的状态转移矩阵，并按配置间隔保存节点。
    rts_phi_accum = kf.phi * height_transition * rts_phi_accum;
    last_node_time = imudata(rts_block.node_indices(end), 1);
    if imudata(imu_index, 1) - last_node_time >= ...
            options.rts_node_interval_s - max(1e-8, abs(imu_dt) * 0.25)
        [rts_block, rts_phi_accum] = append_rts_node_if_needed( ...
            rts_block, imu_index, kf.P, rts_phi_accum);
    end

    progress = floor(10 * imu_index / sample_count) * 10;
    if progress > last_progress && mod(progress, 20) == 0
        fprintf('处理进度：%d %%\n', progress);
        last_progress = progress;
    end
end

%% 生成长度一致、可以直接比较的五类结果
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

% 最后一个完整 RTS 区间没有下一段桥接误差，二次结果按 experiment 的
% 处理原则退化为该区间的一次 RTS 结果。
if ~isempty(previous_rts_block)
    [rts_double_nav, rts_double_is_processed] = write_segment_result( ...
        rts_double_nav, rts_double_is_processed, ...
        previous_rts_block.segment_indices, previous_rts_block.single_nav);
end
rts_single_nav(~rts_single_is_processed, :) = ...
    forward_nav(~rts_single_is_processed, :);
rts_double_nav(~rts_double_is_processed, :) = ...
    forward_nav(~rts_double_is_processed, :);
% 没有获得后续二次 RTS 残余桥接节点的末尾区间，保留二次 RTS 结果。
rts_rotation_nav(~rts_rotation_is_processed, :) = ...
    rts_double_nav(~rts_rotation_is_processed, :);

forward_path = fullfile(cfg.outputfolder, 'range-ins-forward.nav');
backward_path = fullfile(cfg.outputfolder, 'range-ins-backward-constrained.nav');
delayed_geometry_path = fullfile(cfg.outputfolder, ...
    'range-ins-delayed-previous-geometry.nav');
artifact_dir = exploration_artifact_dir(cfg.outputfolder);
diagnostic_path = fullfile(artifact_dir, 'range-segment-diagnostics.csv');
rts_single_path = fullfile(cfg.outputfolder, 'range-ins-rts-single.nav');
rts_double_path = fullfile(cfg.outputfolder, 'range-ins-rts-double.nav');
rts_diagnostic_path = fullfile(artifact_dir, 'range-rts-diagnostics.csv');
rts_rotation_path = fullfile(cfg.outputfolder, ...
    'range-ins-rts-double-bridge-rotation.nav');
rts_rotation_diagnostic_path = fullfile(artifact_dir, ...
    'range-rts-bridge-rotation-diagnostics.csv');

write_nav_file(forward_path, forward_nav);
write_nav_file(backward_path, backward_nav);
write_nav_file(delayed_geometry_path, delayed_geometry_nav);
write_diagnostics(diagnostic_path, diagnostics);
write_nav_file(rts_single_path, rts_single_nav);
write_nav_file(rts_double_path, rts_double_nav);
write_rts_diagnostics(rts_diagnostic_path, rts_diagnostics);
write_nav_file(rts_rotation_path, rts_rotation_nav);
write_rotation_diagnostics(rts_rotation_diagnostic_path, ...
    rts_rotation_diagnostics);

% 主对比只保留用户当前关心的三种结果。
result_names = {'前向 EKF', '二次 RTS', '2RTS+旋转收缩'};
result_nav = {forward_nav, rts_double_nav, rts_rotation_nav};
evaluate_and_plot_results(truth_all, result_names, result_nav, ...
    segment_is_processed, cfg.outputfolder, options.case_name);
plot_rotation_diagnostics(rts_rotation_diagnostics, ...
    cfg.outputfolder, options.case_name);

fprintf('已完成 %d 个相邻测距区间的反向约束处理。\n', numel(diagnostics));
fprintf('前向滤波结果：%s\n', forward_path);
fprintf('纯反向重建结果：%s\n', backward_path);
fprintf('延迟上一段几何修正结果：%s\n', delayed_geometry_path);
fprintf('一次 RTS 平滑结果：%s\n', rts_single_path);
fprintf('二次 RTS 平滑结果：%s\n', rts_double_path);
fprintf('二次 RTS 残余桥接旋转收缩结果：%s\n', rts_rotation_path);

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

function block = initialize_rts_block(start_index, start_covariance, rank)
%INITIALIZE_RTS_BLOCK 创建一个以测距反馈状态为起点的 RTS 区间缓存。
    block = struct( ...
        'start_index', start_index, ...
        'node_indices', start_index, ...
        'covariances', reshape(start_covariance, rank, rank, 1), ...
        'transitions', zeros(rank, rank, 0));
end

function [block, phi_accum] = append_rts_node_if_needed( ...
        block, imu_index, covariance, phi_accum)
%APPEND_RTS_NODE_IF_NEEDED 保存关键节点及上一关键节点到该点的状态转移。
    if block.node_indices(end) == imu_index
        return;
    end
    transition_index = numel(block.node_indices);
    block.transitions(:, :, transition_index) = phi_accum;
    block.node_indices(end + 1) = imu_index;
    block.covariances(:, :, end + 1) = covariance;
    phi_accum = eye(size(covariance, 1));
end

function [smoothed_nav, bridge_error, error_nodes] = smooth_rts_block( ...
        block, terminal_error, nominal_nav, segment_indices, param)
%SMOOTH_RTS_BLOCK 对一个测距区间执行误差状态 RTS 反向平滑。
%
% block 中保存的协方差和状态转移矩阵来自连续 100 Hz AEKF；默认每
% 1 s 形成一个 RTS 节点。节点误差平滑完成后，线性插值回原始 IMU
% 历元，仅改变输出采样密度，不改变 RTS 节点上的递推关系。
    rank = numel(terminal_error);
    node_count = numel(block.node_indices);
    if node_count < 2
        error('RTS 区间至少需要两个关键节点。');
    end
    if size(nominal_nav, 1) ~= numel(segment_indices)
        error('RTS 名义轨迹长度与区间索引长度不一致。');
    end

    error_nodes = zeros(node_count, rank);
    error_nodes(end, :) = terminal_error(:)';
    for node_index = node_count - 1:-1:1
        corrected_covariance = block.covariances(:, :, node_index);
        predicted_covariance = block.covariances(:, :, node_index + 1);
        transition = block.transitions(:, :, node_index);

        predicted_covariance = ...
            (predicted_covariance + predicted_covariance') / 2;
        regularization = max(1e-12, ...
            1e-12 * trace(predicted_covariance) / rank);
        predicted_covariance = predicted_covariance + ...
            eye(rank) * regularization;
        smoothing_gain = corrected_covariance * transition' * ...
            pinv(predicted_covariance);
        error_nodes(node_index, :) = ...
            (smoothing_gain * error_nodes(node_index + 1, :)')';
    end

    full_error = interp1(double(block.node_indices(:)), error_nodes, ...
        double(segment_indices(:)), 'linear');
    smoothed_nav = apply_error_to_nav(nominal_nav, full_error, param);
    bridge_error = error_nodes(1, :)';
end

function corrected_nav = apply_error_to_nav(nominal_nav, error_state, param)
%APPLY_ERROR_TO_NAV 按当前 15 状态滤波器的反馈符号修正导航输出。
    corrected_nav = nominal_nav;
    corrected_nav(:, 3) = nominal_nav(:, 3) - ...
        error_state(:, 1) * param.R2D;
    corrected_nav(:, 4) = nominal_nav(:, 4) - ...
        error_state(:, 2) * param.R2D;
    corrected_nav(:, 5) = nominal_nav(:, 5) - error_state(:, 3);
    corrected_nav(:, 6:8) = nominal_nav(:, 6:8) - error_state(:, 4:6);
    % 与 myErrorFeedback_range 保持一致：当前版本不反馈姿态误差。
end

function [result_nav, is_processed] = write_segment_result( ...
        result_nav, is_processed, segment_indices, segment_nav)
%WRITE_SEGMENT_RESULT 写入分段结果，并避免公共测距历元被后一区间覆盖。
    write_columns = 1:numel(segment_indices);
    if is_processed(segment_indices(1))
        segment_indices = segment_indices(2:end);
        write_columns = write_columns(2:end);
    end
    result_nav(segment_indices, :) = segment_nav(write_columns, :);
    is_processed(segment_indices) = true;
end

function position = nav_row_to_position(nav_row, param)
%NAV_ROW_TO_POSITION 将输出格式位置恢复为 [纬度(rad); 经度(rad); 高度(m)]。
    position = [nav_row(3:4)' * param.D2R; nav_row(5)];
end

function value = horizontal_error_norm(error_state, position)
%HORIZONTAL_ERROR_NORM 将误差状态前两维转换为水平米制模长。
    param = Param();
    [rm, rn] = getRmRn(position(1), param);
    north_error = error_state(1) * (rm + position(3));
    east_error = error_state(2) * (rn + position(3)) * cos(position(1));
    value = hypot(north_error, east_error);
end

function item = empty_rts_diagnostic()
%EMPTY_RTS_DIAGNOSTIC 创建 RTS 区间诊断记录。
    item = struct( ...
        'block_index', 0, ...
        'start_time', nan, ...
        'end_time', nan, ...
        'sample_count', 0, ...
        'node_count', 0, ...
        'terminal_error_m', nan, ...
        'first_pass_bridge_error_m', nan, ...
        'second_pass_terminal_error_m', nan);
end

function write_rts_diagnostics(file_path, diagnostics)
%WRITE_RTS_DIAGNOSTICS 保存一次、二次 RTS 的区间桥接误差。
    file_id = fopen(file_path, 'w');
    if file_id < 0
        error('无法打开 RTS 诊断文件：%s', file_path);
    end
    cleaner = onCleanup(@() fclose(file_id));
    fprintf(file_id, ['block,start-time,end-time,samples,nodes,', ...
        'terminal-error-m,first-pass-bridge-error-m,', ...
        'second-pass-terminal-error-m\n']);
    for index = 1:numel(diagnostics)
        item = diagnostics(index);
        fprintf(file_id, '%d,%.6f,%.6f,%d,%d,%.9f,%.9f,%.9f\n', ...
            item.block_index, item.start_time, item.end_time, ...
            item.sample_count, item.node_count, item.terminal_error_m, ...
            item.first_pass_bridge_error_m, ...
            item.second_pass_terminal_error_m);
    end
    clear cleaner;
end

function [rotated_nav, info] = rotate_double_rts_segment( ...
        double_rts_nav, target_position, param)
%ROTATE_DOUBLE_RTS_SEGMENT 用残余桥接节点旋转收缩上一段二次 RTS 轨迹。
%
% 固定上一段二次 RTS 在较早测距时刻的起点，将其较晚测距时刻的
% 终点对齐到下一段二次 RTS 的起点。该起点由二次 RTS 残余桥接误差
% 形成，因此几何变换只吸收未继续执行三次 RTS 的残余端点约束。
    position = [double_rts_nav(:, 3:4)' * param.D2R; ...
        double_rts_nav(:, 5)'];
    original_endpoint = position(:, end);
    horizontal_position = position;
    horizontal_position(3, :) = 0;
    horizontal_target = target_position;
    horizontal_target(3) = 0;
    [rotated_position, scale_factor, rotation_angle_deg] = ...
        rotateAndScaleTrajectory(horizontal_position, horizontal_target);

    rotated_nav = double_rts_nav;
    rotated_nav(:, 3:4) = rotated_position(1:2, :)' * param.R2D;
    info = struct( ...
        'scale_factor', scale_factor, ...
        'rotation_angle_deg', rotation_angle_deg, ...
        'endpoint_gap_before_m', horizontal_position_gap( ...
            original_endpoint, target_position), ...
        'endpoint_gap_after_m', horizontal_position_gap( ...
            rotated_position(:, end), target_position));
end

function gap = horizontal_position_gap(position, reference)
%HORIZONTAL_POSITION_GAP 计算两个大地坐标点的水平间距。
    param = Param();
    [rm, rn] = getRmRn(reference(1), param);
    north_gap = (position(1) - reference(1)) * ...
        (rm + reference(3));
    east_gap = (position(2) - reference(2)) * ...
        (rn + reference(3)) * cos(reference(1));
    gap = hypot(north_gap, east_gap);
end

function item = empty_rotation_diagnostic()
%EMPTY_ROTATION_DIAGNOSTIC 创建残余桥接旋转收缩诊断记录。
    item = struct( ...
        'block_index', 0, ...
        'start_time', nan, ...
        'end_time', nan, ...
        'residual_bridge_error_m', nan, ...
        'rotation_angle_deg', nan, ...
        'scale_factor', nan, ...
        'endpoint_gap_before_m', nan, ...
        'endpoint_gap_after_m', nan);
end

function write_rotation_diagnostics(file_path, diagnostics)
%WRITE_ROTATION_DIAGNOSTICS 保存残余桥接旋转收缩的几何参数。
    file_id = fopen(file_path, 'w');
    if file_id < 0
        error('无法打开残余桥接旋转收缩诊断文件：%s', file_path);
    end
    cleaner = onCleanup(@() fclose(file_id));
    fprintf(file_id, ['block,start-time,end-time,residual-bridge-error-m,', ...
        'rotation-angle-deg,scale-factor,endpoint-gap-before-m,', ...
        'endpoint-gap-after-m\n']);
    for index = 1:numel(diagnostics)
        item = diagnostics(index);
        fprintf(file_id, '%d,%.6f,%.6f,%.9f,%.9f,%.9f,%.9f,%.9f\n', ...
            item.block_index, item.start_time, item.end_time, ...
            item.residual_bridge_error_m, item.rotation_angle_deg, ...
            item.scale_factor, item.endpoint_gap_before_m, ...
            item.endpoint_gap_after_m);
    end
    clear cleaner;
end

function evaluate_and_plot_results(truth, names, nav_results, ...
        effective_mask, output_dir, case_name)
%EVALUATE_AND_PLOT_RESULTS 评价前向 EKF、二次 RTS 和 RC-2RTS。
    figure_dir = exploration_artifact_dir(output_dir);
    if ~exist(figure_dir, 'dir')
        mkdir(figure_dir);
    end

    time = nav_results{1}(:, 2);
    truth_position = interp1(truth(:, 2), truth(:, 3:5), time, ...
        'linear', 'extrap');
    result_count = numel(nav_results);
    radial_error = zeros(numel(time), result_count);
    for result_index = 1:result_count
        [~, ~, radial_error(:, result_index)] = calculate_horizontal_error( ...
            nav_results{result_index}(:, 3:5), truth_position);
    end

    statistics = table('Size', [result_count * 2, 7], ...
        'VariableTypes', {'string', 'string', 'double', 'double', ...
        'double', 'double', 'double'}, ...
        'VariableNames', {'System', 'Scope', 'RMSE_m', 'Max_m', ...
        'Mean_m', 'Median_m', 'P95_m'});
    row_index = 0;
    for result_index = 1:result_count
        for scope_index = 1:2
            row_index = row_index + 1;
            if scope_index == 1
                mask = true(size(time));
                scope = "全时段";
            else
                mask = effective_mask;
                scope = "相邻测距有效区间";
            end
            values = radial_error(mask, result_index);
            statistics(row_index, :) = {string(names{result_index}), scope, ...
                sqrt(mean(values .^ 2)), max(values), mean(values), ...
                median(values), prctile(values, 95)};
        end
    end
    writetable(statistics, fullfile(figure_dir, ...
        'range-rts-bridge-rotation-statistics.csv'));

    colors = [ ...
        0.10, 0.35, 0.75; ...
        0.05, 0.55, 0.55; ...
        0.80, 0.15, 0.55];
    line_styles = {'-', '--', '-.'};
    overview = figure('Color', 'w', 'Name', '二次 RTS 残余桥接旋转收缩对比', ...
        'Position', [80, 120, 1500, 600]);
    layout = tiledlayout(1, 2, 'TileSpacing', 'compact', ...
        'Padding', 'compact');
    title(layout, sprintf('%s：EKF、二次 RTS 与 2RTS+旋转收缩对比', ...
        case_name));

    nexttile;
    origin = truth_position(1, :);
    [truth_east, truth_north] = position_to_local_plane_rts( ...
        truth_position, origin);
    plot(truth_east / 1000, truth_north / 1000, 'k-', ...
        'LineWidth', 1.8, 'DisplayName', '真值');
    hold on;
    for result_index = 1:result_count
        [east, north] = position_to_local_plane_rts( ...
            nav_results{result_index}(:, 3:5), origin);
        plot(east / 1000, north / 1000, ...
            'Color', colors(result_index, :), ...
            'LineStyle', line_styles{result_index}, ...
            'LineWidth', 1.2, 'DisplayName', names{result_index});
    end
    % 本算例东西向跨度远大于南北向跨度，独立缩放便于观察轨迹差异。
    axis tight;
    grid on;
    xlabel('东向位置（km）');
    ylabel('北向位置（km）');
    title('平面轨迹（横纵轴独立缩放）');
    legend('Location', 'best');

    nexttile;
    for result_index = 1:result_count
        plot(time, radial_error(:, result_index), ...
            'Color', colors(result_index, :), ...
            'LineStyle', line_styles{result_index}, ...
            'LineWidth', 1.2, 'DisplayName', names{result_index});
        hold on;
    end
    grid on;
    xlabel('时间（s）');
    ylabel('水平径向误差（m）');
    title('水平径向误差');
    legend('Location', 'best');
    xlim([0, 4621]);

    exportgraphics(overview, fullfile(figure_dir, ...
        'range-rts-bridge-rotation-comparison.png'), 'Resolution', 200);
    savefig(overview, fullfile(figure_dir, ...
        'range-rts-bridge-rotation-comparison.fig'));
    disp('二次 RTS 残余桥接旋转收缩对比统计：');
    disp(statistics);
end

function plot_rotation_diagnostics(diagnostics, output_dir, case_name)
%PLOT_ROTATION_DIAGNOSTICS 绘制残余桥接旋转收缩的区间诊断图。
    if isempty(diagnostics)
        warning('没有可绘制的残余桥接旋转收缩诊断数据。');
        return;
    end
    figure_dir = exploration_artifact_dir(output_dir);
    block = [diagnostics.block_index];
    residual = [diagnostics.residual_bridge_error_m];
    angle = [diagnostics.rotation_angle_deg];
    scale = [diagnostics.scale_factor];
    gap_before = [diagnostics.endpoint_gap_before_m];
    gap_after = [diagnostics.endpoint_gap_after_m];

    diagnostic_figure = figure('Color', 'w', ...
        'Name', '二次 RTS 残余桥接旋转收缩诊断', ...
        'Position', [120, 120, 1300, 760]);
    layout = tiledlayout(2, 2, 'TileSpacing', 'compact', ...
        'Padding', 'compact');
    title(layout, sprintf('%s：二次 RTS 残余桥接旋转收缩诊断', ...
        case_name));

    nexttile;
    bar(block, residual, 0.65, 'FaceColor', [0.80, 0.15, 0.55]);
    grid on;
    xlabel('被修正的二次 RTS 区间');
    ylabel('残余桥接误差（m）');
    title('生成新参考节点的残余桥接误差');

    nexttile;
    bar(block, angle, 0.65, 'FaceColor', [0.35, 0.45, 0.80]);
    grid on;
    xlabel('被修正的二次 RTS 区间');
    ylabel('旋转角（°）');
    title('水平旋转角');

    nexttile;
    plot(block, scale, 'o-', 'Color', [0.10, 0.60, 0.45], ...
        'LineWidth', 1.3, 'MarkerFaceColor', [0.10, 0.60, 0.45]);
    yline(1, 'k:');
    grid on;
    xlabel('被修正的二次 RTS 区间');
    ylabel('尺度因子');
    title('水平收缩/伸展比例');

    nexttile;
    semilogy(block, max(gap_before, eps), 'o-', 'LineWidth', 1.2, ...
        'DisplayName', '修正前');
    hold on;
    semilogy(block, max(gap_after, eps), 's-', 'LineWidth', 1.2, ...
        'DisplayName', '修正后');
    grid on;
    xlabel('被修正的二次 RTS 区间');
    ylabel('端点间距（m，对数轴）');
    title('端点对齐残差');
    legend('Location', 'best');

    exportgraphics(diagnostic_figure, fullfile(figure_dir, ...
        'range-rts-bridge-rotation-diagnostics.png'), 'Resolution', 200);
    savefig(diagnostic_figure, fullfile(figure_dir, ...
        'range-rts-bridge-rotation-diagnostics.fig'));
end

function [north_error, east_error, radial_error] = ...
        calculate_horizontal_error(estimated_position, truth_position)
%CALCULATE_HORIZONTAL_ERROR 按 WGS-84 逐点曲率半径计算水平误差。
    latitude = deg2rad(truth_position(:, 1));
    height = truth_position(:, 3);
    [rm, rn] = wgs84_radii_rts(latitude);
    north_error = deg2rad(estimated_position(:, 1) - ...
        truth_position(:, 1)) .* (rm + height);
    east_error = deg2rad(estimated_position(:, 2) - ...
        truth_position(:, 2)) .* (rn + height) .* cos(latitude);
    radial_error = hypot(north_error, east_error);
end

function [rm, rn] = wgs84_radii_rts(latitude)
%WGS84_RADII_RTS 计算 WGS-84 子午圈和卯酉圈曲率半径。
    semi_major_axis = 6378137.0;
    flattening = 1 / 298.257223563;
    eccentricity_squared = flattening * (2 - flattening);
    denominator = sqrt(1 - eccentricity_squared .* sin(latitude) .^ 2);
    rn = semi_major_axis ./ denominator;
    rm = semi_major_axis * (1 - eccentricity_squared) ./ denominator .^ 3;
end

function [east, north] = position_to_local_plane_rts(position, origin)
%POSITION_TO_LOCAL_PLANE_RTS 将经纬度转换为相对起点的局部平面坐标。
    latitude = deg2rad(origin(1));
    [rm, rn] = wgs84_radii_rts(latitude);
    north = deg2rad(position(:, 1) - origin(1)) * (rm + origin(3));
    east = deg2rad(position(:, 2) - origin(2)) * ...
        (rn + origin(3)) * cos(latitude);
end
