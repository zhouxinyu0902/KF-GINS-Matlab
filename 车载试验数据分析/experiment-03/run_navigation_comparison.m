%% 第三次车载试验：EKF 与 RTS 导航结果对比
% 本文件直接完成数据读取、前向 EKF、一次 RTS、二次 RTS 和结果评价。
% 日常使用只需修改“运行设置”，不再调用单独的核心算法文件。
clear;
clc;
close all;

%% 运行设置
dataset_id = 'run-0818-noon';
position_unit = "rad";              % "rad" 或 "m"
selected_methods = ["ekf", "rts1", "rts2"];
% 第四种备选方法：在 selected_methods 中加入 "rts2-rotation"。
switch (dataset_id)
    case 'run-0817'
        duration_s = 10000;
    case 'run-0818'
        duration_s = 12000;
    case 'run-0818-noon'
        duration_s = 17000;
end
range_std_m = 6;
depth_std_m = 0.4;
rts_node_interval_s = 1;
show_input_figure = false;

position_unit = lower(position_unit);
selected_methods = unique(lower(selected_methods), 'stable');
allowed_units = ["rad", "m"];
allowed_methods = ["ekf", "rts1", "rts2", "rts2-rotation"];
if ~ismember(position_unit, allowed_units)
    error('position_unit 只能取 "rad" 或 "m"。');
end
if isempty(selected_methods) || any(~ismember(selected_methods, allowed_methods))
    error('selected_methods 只能包含：%s。', ...
        strjoin(cellstr(allowed_methods), ', '));
end
enable_rotation = ismember("rts2-rotation", selected_methods);

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir, '-begin');
paths = setup_all_real_data_preprocessing(dataset_id);
input_dir = paths.input;
result_dir = fullfile(paths.output, char(position_unit));
artifact_dir = fullfile(result_dir, 'artifacts');
if ~isfolder(result_dir), mkdir(result_dir); end
if ~isfolder(artifact_dir), mkdir(artifact_dir); end

%% 事件驱动的测距/惯导处理顺序
% 每个测距时刻的处理顺序：
%   1）利用当前测距观测更新前向 15 状态 EKF；
%   2）对刚结束的区间执行一次 RTS；
%   3）将当前区间的桥接误差作为上一段末端条件，生成二次 RTS；
%   4）若启用备选方法，对已完成的二次 RTS 区间执行旋转收缩。

glvs;                                % 初始化 PSINS 使用的地球模型全局常量。
param = Param();

options.case_name = sprintf('experiment-03-%s-%s', ...
    paths.dataset_name, position_unit);
options.range_interval_s = 420;
options.beacon_order = [1, 2, 3];
options.filter_range_std_m = range_std_m;
options.filter_depth_std_m = depth_std_m;
options.use_adaptive_forward_update = false;
options.show_input_figure = show_input_figure;
% RTS 默认每 1 s 保存一个关键节点；协方差仍在 100 Hz IMU 历元连续传播。
% 改为 0.01 可执行逐 IMU 历元 RTS，但计算时间和内存占用会显著增加。
options.rts_node_interval_s = rts_node_interval_s;
options.duration_s = duration_s;

cfg = Config(dataset_id, position_unit);
cfg.userange = true;
cfg.outputfolder = result_dir;

if position_unit == "rad"
    fn_range_update = @myRangeUpdate;
    fn_error_feedback = @myErrorFeedback_range;
    fn_height_update = @update_decoupled_height_rad;
    fn_ins_propagate = @myInsPropagate_15state;
else
    fn_range_update = @myRangeUpdate_m;
    fn_error_feedback = @myErrorFeedback_range_m;
    fn_height_update = @update_decoupled_height_m;
    fn_ins_propagate = @myInsPropagate_15state_m;
end

%% 读取并整理观测数据
imu_all = readmatrix(cfg.imufilepath, 'FileType', 'text');
truth_all = readmatrix(cfg.truthpath, 'FileType', 'text');
range_input_path = cfg.rangefilepath;
rangedata = readmatrix(range_input_path, 'FileType', 'text');
if isempty(rangedata) || size(rangedata, 2) < 6
    error('测距数据必须是非空的 N×6（或更多列）矩阵。');
end
if any(diff(rangedata(:, 1)) <= 0)
    error('测距时刻必须严格递增。');
end
height_all = readmatrix(cfg.heightfilepath, ...
    'FileType', 'text');
options.range_boundary_reference_time_s = rangedata(1, 1);

start_time = max([cfg.starttime, imu_all(1, 1), truth_all(1, 2)]);
end_time = min([cfg.endtime, cfg.starttime + options.duration_s, ...
    imu_all(end, 1), truth_all(end, 2)]);
cfg.starttime = start_time;
cfg.endtime = end_time;

imu_mask = imu_all(:, 1) >= start_time & imu_all(:, 1) <= end_time;
imudata = imu_all(imu_mask, :);
range_mask = rangedata(:, 1) >= start_time & rangedata(:, 1) <= end_time;
rangedata = rangedata(range_mask, :);
if isempty(imudata)
    error('所选时间段内没有 IMU 数据。');
end
if isempty(rangedata)
    error('所选时间段内没有测距数据。');
end

% 实测测距时间位于两个100 Hz IMU历元之间。统一映射到最近的IMU历元；
% 距离相同时选择后一历元，保证量测不会被提前使用。
original_range_times = rangedata(:, 1);
[aligned_range_times, range_time_offsets, range_imu_indices] = ...
    align_range_times_to_imu(imudata(:, 1), original_range_times);
rangedata(:, 1) = aligned_range_times;
alignment_table = table(original_range_times, aligned_range_times, ...
    range_time_offsets, range_imu_indices, ...
    'VariableNames', {'OriginalTime_s', 'AlignedImuTime_s', ...
    'AlignmentOffset_s', 'ImuIndex'});
writetable(alignment_table, fullfile(artifact_dir, ...
    'range-time-alignment.csv'));
fprintf('测距时间已映射到IMU历元：%d点，最大时间修正 %.6f s。\n', ...
    size(alignment_table, 1), max(abs(range_time_offsets)));

height_values = interp1(height_all(:, 1), height_all(:, 2), imudata(:, 1), ...
    'linear', 'extrap');
height = [imudata(:, 1), height_values];

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

rts_diagnostics = repmat(empty_rts_diagnostic(), 0, 1);
rts_block = initialize_rts_block(1, kf.P, kf.RANK);
previous_rts_block = [];
previous_double_block = [];
rts_rotation_diagnostics = repmat(empty_rotation_diagnostic(), 0, 1);
rts_phi_accum = eye(kf.RANK);
last_progress = -1;
processed_range_event_count = 0;

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
        processed_range_event_count = processed_range_event_count + 1;
        anchor_imu_index = imu_index - 1;
        current_range = rangedata(range_index, :);
        current_depth = height(anchor_imu_index, :);

        % 把测距时刻作为当前 RTS 区间的最后一个关键节点。这里保存的是
        % 测距更新前的预测协方差和名义状态。
        [rts_block, ~] = append_rts_node_if_needed( ...
            rts_block, anchor_imu_index, kf.P, rts_phi_accum);

        % 当前测距事件：先计算末端误差，但暂不反馈，供一次 RTS 使用。
        kf = fn_range_update(navstate, current_range, current_depth, kf);
        terminal_error = kf.x;

        % 一次 RTS：把当前测距更新产生的末端误差反向平滑到当前区间。
        rts_segment_indices = rts_block.start_index:anchor_imu_index;
        nominal_segment_nav = forward_nav(rts_segment_indices, :);
        [single_segment_nav, bridge_error, first_error_nodes] = ...
            smooth_rts_block(rts_block, terminal_error, nominal_segment_nav, ...
            rts_segment_indices, param, position_unit);
        [rts_single_nav, rts_single_is_processed] = write_segment_result( ...
            rts_single_nav, rts_single_is_processed, rts_segment_indices, ...
            single_segment_nav);

        % 二次 RTS：当前区间一次平滑传到起点的桥接误差，作为上一段的
        % 末端条件，再对上一段已经一次平滑的轨迹执行一次 RTS。
        if ~isempty(previous_rts_block)
            [double_previous_nav, residual_bridge_error] = smooth_rts_block( ...
                previous_rts_block, bridge_error, ...
                previous_rts_block.single_nav, ...
                previous_rts_block.segment_indices, param, position_unit);
            [rts_double_nav, rts_double_is_processed] = write_segment_result( ...
                rts_double_nav, rts_double_is_processed, ...
                previous_rts_block.segment_indices, double_previous_nav);

            previous_index = previous_rts_block.diagnostic_index;
            rts_diagnostics(previous_index).second_pass_terminal_error_m = ...
                horizontal_error_norm_m(bridge_error, ...
                previous_rts_block.end_position, position_unit);

            % 旋转收缩只在已经完成的二次 RTS 轨迹上执行。当前二次 RTS
            % 给出的新桥接节点用于修正更早一段轨迹，不重复运行 EKF/RTS。
            if enable_rotation && ~isempty(previous_double_block)
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
                rotation_item.block_index = previous_double_block.block_index;
                rotation_item.start_time = previous_double_block.nav(1, 2);
                rotation_item.end_time = previous_double_block.nav(end, 2);
                rotation_item.residual_bridge_error_m = ...
                    horizontal_error_norm_m(residual_bridge_error, ...
                    nav_row_to_position(previous_double_block.nav(1, :), param), ...
                    position_unit);
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

            if enable_rotation
                previous_double_block = struct( ...
                    'nav', double_previous_nav, ...
                    'segment_indices', previous_rts_block.segment_indices, ...
                    'block_index', previous_index);
            end
        end

        rts_info = empty_rts_diagnostic();
        rts_info.block_index = numel(rts_diagnostics) + 1;
        rts_info.start_time = imudata(rts_segment_indices(1), 1);
        rts_info.end_time = imudata(rts_segment_indices(end), 1);
        rts_info.sample_count = numel(rts_segment_indices);
        rts_info.node_count = numel(rts_block.node_indices);
        rts_info.terminal_error_m = horizontal_error_norm_m( ...
            terminal_error, navstate.pos, position_unit);
        rts_info.first_pass_bridge_error_m = ...
            horizontal_error_norm_m(bridge_error, ...
            nav_row_to_position(nominal_segment_nav(1, :), param), ...
            position_unit);
        rts_diagnostics(end + 1, 1) = rts_info; %#ok<SAGROW>

        previous_rts_block = rts_block;
        previous_rts_block.single_nav = single_segment_nav;
        previous_rts_block.segment_indices = rts_segment_indices;
        previous_rts_block.diagnostic_index = numel(rts_diagnostics);
        previous_rts_block.end_position = navstate.pos;
        previous_rts_block.first_error_nodes = first_error_nodes;

        % 将当前测距误差反馈到名义导航状态，供后续前向和反向推算使用。
        [kf, navstate] = fn_error_feedback(kf, navstate);
        forward_nav(anchor_imu_index, :) = state_to_nav_row(navstate, param);
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
        [kf, navstate, height_transition] = ...
            fn_height_update(kf, navstate, height(imu_index, :));
    end
    kf = fn_ins_propagate(navstate, this_imu, imu_dt, kf);
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

if processed_range_event_count ~= size(rangedata, 1)
    error('测距事件处理不完整：输入%d点，实际更新%d点。', ...
        size(rangedata, 1), processed_range_event_count);
end
%% 生成长度一致、可以直接比较的结果
rts_single_valid_mask = rts_single_is_processed;
rts_single_nav(~rts_single_is_processed, :) = ...
    forward_nav(~rts_single_is_processed, :);

% 最后一个完整区间还没有下一段桥接误差，二次结果在该区间退化为一次RTS。
rts_double_second_pass_mask = rts_double_is_processed;
if ~isempty(previous_rts_block)
    [rts_double_nav, rts_double_is_processed] = write_segment_result( ...
        rts_double_nav, rts_double_is_processed, ...
        previous_rts_block.segment_indices, previous_rts_block.single_nav);
end
rts_double_nav(~rts_double_is_processed, :) = ...
    forward_nav(~rts_double_is_processed, :);
if enable_rotation
    rts_rotation_valid_mask = rts_rotation_is_processed;
    rts_rotation_nav(~rts_rotation_is_processed, :) = ...
        rts_double_nav(~rts_rotation_is_processed, :);
else
    rts_rotation_valid_mask = false(sample_count, 1);
end

forward_path = fullfile(cfg.outputfolder, 'ekf.nav');
rts_single_path = fullfile(cfg.outputfolder, 'rts-once.nav');
rts_double_path = fullfile(cfg.outputfolder, 'rts-twice.nav');
rts_rotation_path = fullfile(cfg.outputfolder, 'rts-twice-rotation.nav');
rts_diagnostic_path = fullfile(artifact_dir, 'range-rts-diagnostics.csv');
rts_rotation_diagnostic_path = fullfile(artifact_dir, ...
    'range-rts-bridge-rotation-diagnostics.csv');
statistics_path = fullfile(artifact_dir, ...
    'navigation-comparison-statistics.csv');

method_names = cell(1, numel(selected_methods));
nav_results = cell(1, numel(selected_methods));
method_paths = cell(1, numel(selected_methods));
comparison_mask = true(sample_count, 1);
for method_index = 1:numel(selected_methods)
    switch selected_methods(method_index)
        case "ekf"
            method_names{method_index} = '前向 EKF';
            nav_results{method_index} = forward_nav;
            method_paths{method_index} = forward_path;
        case "rts1"
            method_names{method_index} = '一次 RTS';
            nav_results{method_index} = rts_single_nav;
            method_paths{method_index} = rts_single_path;
            comparison_mask = comparison_mask & rts_single_valid_mask;
        case "rts2"
            method_names{method_index} = '二次 RTS';
            nav_results{method_index} = rts_double_nav;
            method_paths{method_index} = rts_double_path;
            comparison_mask = comparison_mask & rts_double_second_pass_mask;
        case "rts2-rotation"
            if ~any(rts_rotation_valid_mask)
                error('没有形成有效的二次RTS旋转收缩区间。');
            end
            method_names{method_index} = '二次 RTS+旋转收缩';
            nav_results{method_index} = rts_rotation_nav;
            method_paths{method_index} = rts_rotation_path;
            comparison_mask = comparison_mask & rts_rotation_valid_mask;
    end
end
if ~any(comparison_mask)
    error('所选方法没有共同的有效评价区间。');
end

for method_index = 1:numel(selected_methods)
    write_nav_file(method_paths{method_index}, nav_results{method_index});
end
write_rts_diagnostics(rts_diagnostic_path, rts_diagnostics);
if enable_rotation
    write_rotation_diagnostics(rts_rotation_diagnostic_path, ...
        rts_rotation_diagnostics);
end
%%
% options.duration_s  = 12000;
statistics = evaluate_navigation_methods(truth_all, method_names, ...
    nav_results, comparison_mask, artifact_dir, options.case_name, ...
    statistics_path, options.duration_s);

fprintf('\n%s，%s 状态单位，处理完成。\n', ...
    paths.dataset_name, position_unit);
disp(statistics);
for method_index = 1:numel(selected_methods)
    fprintf('%s：%s\n', method_names{method_index}, ...
        method_paths{method_index});
end
%% 局部辅助函数
function [aligned_times, offsets, imu_indices] = ...
        align_range_times_to_imu(imu_times, range_times)
%ALIGN_RANGE_TIMES_TO_IMU 将测距时刻映射到最近的IMU历元。
% 当量测恰好位于两个历元中间时选择后一历元，避免提前使用量测。
    imu_times = imu_times(:);
    range_times = range_times(:);
    if numel(imu_times) < 2 || any(diff(imu_times) <= 0)
        error('IMU时间必须严格递增且至少包含两个历元。');
    end

    nominal_dt = median(diff(imu_times));
    maximum_offset = nominal_dt / 2 + max(1e-8, nominal_dt * 1e-6);
    tie_tolerance = max(1e-9, nominal_dt * 1e-6);
    imu_indices = zeros(size(range_times));
    aligned_times = zeros(size(range_times));

    for range_index = 1:numel(range_times)
        after_index = find(imu_times >= range_times(range_index), 1, 'first');
        if isempty(after_index)
            selected_index = numel(imu_times);
        elseif after_index == 1
            selected_index = 1;
        else
            before_index = after_index - 1;
            before_error = range_times(range_index) - imu_times(before_index);
            after_error = imu_times(after_index) - range_times(range_index);
            if after_error <= before_error + tie_tolerance
                selected_index = after_index;
            else
                selected_index = before_index;
            end
        end

        alignment_error = imu_times(selected_index) - range_times(range_index);
        if abs(alignment_error) > maximum_offset
            error(['测距时刻 %.6f s 距离最近IMU历元 %.6f s，相差 %.6f s，', ...
                '超过允许值 %.6f s。'], range_times(range_index), ...
                imu_times(selected_index), alignment_error, maximum_offset);
        end
        imu_indices(range_index) = selected_index;
        aligned_times(range_index) = imu_times(selected_index);
    end
    offsets = aligned_times - range_times;
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
    format = ['%2d %12.6f %12.8f %12.8f %8.4f %8.4f ', ...
        '%8.4f %8.4f %8.4f %8.4f %8.4f\n'];
    fprintf(file_id, format, nav_data');
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
        block, terminal_error, nominal_nav, segment_indices, param, ...
        position_unit)
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
    smoothed_nav = apply_error_to_nav( ...
        nominal_nav, full_error, param, position_unit);
    bridge_error = error_nodes(1, :)';
end

function corrected_nav = apply_error_to_nav( ...
        nominal_nav, error_state, param, position_unit)
%APPLY_ERROR_TO_NAV 按当前 15 状态滤波器的反馈符号修正导航输出。
    corrected_nav = nominal_nav;
    if position_unit == "rad"
        corrected_nav(:, 3:4) = nominal_nav(:, 3:4) ...
            - error_state(:, 1:2) * param.R2D;
        % rad链第三维为高度误差dH，因此高度修正为 h-dH。
        corrected_nav(:, 5) = nominal_nav(:, 5) - error_state(:, 3);
    else
        latitude = nominal_nav(:, 3) * param.D2R;
        height = nominal_nav(:, 5);
        [rm, rn] = wgs84_radii_rts(latitude);
        corrected_nav(:, 3) = nominal_nav(:, 3) ...
            - error_state(:, 1) ./ (rm + height) * param.R2D;
        corrected_nav(:, 4) = nominal_nav(:, 4) ...
            - error_state(:, 2) ./ ((rn + height) .* cos(latitude)) ...
            * param.R2D;
        % m链第三维为下向误差D，因此高度修正为 h+D。
        corrected_nav(:, 5) = nominal_nav(:, 5) + error_state(:, 3);
    end
    corrected_nav(:, 6:8) = nominal_nav(:, 6:8) - error_state(:, 4:6);
    % 与两套误差反馈函数保持一致：当前版本不反馈姿态误差。
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

function value = horizontal_error_norm_m( ...
        error_state, position, position_unit)
%HORIZONTAL_ERROR_NORM_M 将不同状态单位的水平误差统一换算为米。
    if position_unit == "rad"
        value = bridge_error_horizontal_m(error_state, position, "rad");
    else
        value = hypot(error_state(1), error_state(2));
    end
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
%ROTATE_DOUBLE_RTS_SEGMENT 用残余桥接节点旋转收缩上一段2RTS轨迹。
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
    north_gap = (position(1) - reference(1)) * (rm + reference(3));
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


% function statistics = evaluate_navigation_methods( ...
%         truth, names, nav_results, effective_mask, output_dir, ...
%         case_name, statistics_path, display_duration_s)
% %EVALUATE_NAVIGATION_METHODS 在共同有效区间评价所选导航结果。
%     time = nav_results{1}(:, 2);
%     elapsed_time = time - time(1);
%     truth_position = interp1(truth(:, 2), truth(:, 3:5), time, ...
%         'linear', 'extrap');
%     result_count = numel(nav_results);
%     for result_index = 1:result_count
%         effective_mask = effective_mask ...
%             & all(isfinite(nav_results{result_index}), 2);
%     end
%     radial_error = nan(numel(time), result_count);
%     rmse_m = zeros(result_count, 1);
%     mean_m = zeros(result_count, 1);
%     median_m = zeros(result_count, 1);
%     p95_m = zeros(result_count, 1);
%     maximum_m = zeros(result_count, 1);
%     for result_index = 1:result_count
%         [~, ~, radial_error(:, result_index)] = ...
%             calculate_horizontal_error( ...
%             nav_results{result_index}(:, 3:5), truth_position);
%         values = radial_error(effective_mask, result_index);
%         rmse_m(result_index) = sqrt(mean(values .^ 2));
%         mean_m(result_index) = mean(values);
%         median_m(result_index) = median(values);
%         p95_m(result_index) = prctile(values, 95);
%         maximum_m(result_index) = max(values);
%     end
%     method = string(names(:));
%     statistics = table(method, rmse_m, mean_m, median_m, p95_m, ...
%         maximum_m, 'VariableNames', {'Method', 'RMSE_m', 'Mean_m', ...
%         'Median_m', 'P95_m', 'Maximum_m'});
%     writetable(statistics, statistics_path);
% 
%     figure_dir = output_dir;
%     if ~exist(figure_dir, 'dir')
%         mkdir(figure_dir);
%     end
%     display_indices = find(effective_mask);
%     display_indices = display_indices(unique(round(linspace(1, ...
%         numel(display_indices), min(20000, numel(display_indices))))));
%     origin = truth_position(display_indices(1), :);
%     [truth_east, truth_north] = position_to_local_plane_rts( ...
%         truth_position(display_indices, :), origin);
%     colors = [0.10, 0.35, 0.75; 0.05, 0.55, 0.55; ...
%         0.78, 0.16, 0.52; 0.20, 0.65, 0.35];
%     line_styles = {'-', '--', ':', '-.'};
%     comparison_figure = figure('Color', 'w', ...
%         'Name', '导航方法对比', 'Position', [80, 120, 1500, 600]);
%     layout = tiledlayout(comparison_figure, 1, 2, ...
%         'TileSpacing', 'compact', 'Padding', 'compact');
%     title(layout, sprintf('%s：事件驱动导航方法对比', case_name));
%     nexttile;
%     plot(truth_east / 1000, truth_north / 1000, 'k-', ...
%         'LineWidth', 1.7, 'DisplayName', '真值');
%     hold on;
%     for result_index = 1:result_count
%         [east, north] = position_to_local_plane_rts( ...
%             nav_results{result_index}(display_indices, 3:5), origin);
%         plot(east / 1000, north / 1000, ...
%             'Color', colors(result_index, :), ...
%             'LineStyle', line_styles{result_index}, ...
%             'LineWidth', 1.25, 'DisplayName', names{result_index});
%     end
%     grid on; box on; axis tight;
%     xlabel('东向位置（km）'); ylabel('北向位置（km）');
%     title('共同有效区间轨迹');
%     legend('Location', 'best', 'Interpreter', 'none');
% 
%     nexttile;
%     hold on;
%     for result_index = 1:result_count
%         plot(elapsed_time(display_indices), ...
%             radial_error(display_indices, result_index), ...
%             'Color', colors(result_index, :), ...
%             'LineStyle', line_styles{result_index}, ...
%             'LineWidth', 1.2, 'DisplayName', names{result_index});
%     end
%     grid on; box on; axis tight;
%     xlabel('时间（s）'); ylabel('水平径向误差（m）');
%     title('共同有效区间水平径向误差');
%     legend('Location', 'best', 'Interpreter', 'none');
%     xlim([0, min(display_duration_s, elapsed_time(end))]);
%     exportgraphics(comparison_figure, fullfile(figure_dir, ...
%         'navigation-comparison.png'), 'Resolution', 300);
%     savefig(comparison_figure, fullfile(figure_dir, ...
%         'navigation-comparison.fig'));
% end
function statistics = evaluate_navigation_methods( ...
        truth, names, nav_results, effective_mask, output_dir, ...
        case_name, statistics_path, display_duration_s)

%EVALUATE_NAVIGATION_METHODS 在统一评价区间评价所选导航结果。

    time = nav_results{1}(:, 2);
    elapsed_time = time - time(1);

    % 插值得到对应时刻真值位置
    truth_position = interp1(truth(:, 2), truth(:, 3:5), time, ...
        'linear', 'extrap');

    result_count = numel(nav_results);

    % 所有算法共同有效区间
    for result_index = 1:result_count
        effective_mask = effective_mask ...
            & all(isfinite(nav_results{result_index}), 2);
    end

    % ==========================================================
    % 增加评价时间窗口限制
    % display_duration_s 同时用于统计和显示
    % ==========================================================
    if ~isempty(display_duration_s)

        duration_mask = elapsed_time <= display_duration_s;

        effective_mask = effective_mask & duration_mask;

    end


    % ==========================================================
    % 误差计算
    % ==========================================================
    radial_error = nan(numel(time), result_count);

    rmse_m = zeros(result_count, 1);
    mean_m = zeros(result_count, 1);
    median_m = zeros(result_count, 1);
    p95_m = zeros(result_count, 1);
    maximum_m = zeros(result_count, 1);


    for result_index = 1:result_count

        [~, ~, radial_error(:, result_index)] = ...
            calculate_horizontal_error( ...
            nav_results{result_index}(:, 3:5), ...
            truth_position);


        % 当前评价窗口
        values = radial_error(effective_mask, result_index);


        rmse_m(result_index) = sqrt(mean(values.^2));
        mean_m(result_index) = mean(values);
        median_m(result_index) = median(values);
        p95_m(result_index) = prctile(values,95);
        maximum_m(result_index) = max(values);

    end


    % ==========================================================
    % 输出统计结果
    % ==========================================================
    method = string(names(:));

    statistics = table(method, ...
        rmse_m, ...
        mean_m, ...
        median_m, ...
        p95_m, ...
        maximum_m, ...
        'VariableNames', ...
        {'Method','RMSE_m','Mean_m',...
        'Median_m','P95_m','Maximum_m'});


    writetable(statistics, statistics_path);



    % ==========================================================
    % 绘图
    % ==========================================================
    figure_dir = output_dir;

    if ~exist(figure_dir,'dir')
        mkdir(figure_dir);
    end


    display_indices = find(effective_mask);

    % 控制绘图点数量
    display_indices = display_indices(unique(round(linspace(1,...
        numel(display_indices),...
        min(20000,numel(display_indices))))));


    origin = truth_position(display_indices(1),:);


    [truth_east, truth_north] = ...
        position_to_local_plane_rts( ...
        truth_position(display_indices,:), ...
        origin);


    colors = [
        0.10,0.35,0.75;
        0.05,0.55,0.55;
        0.78,0.16,0.52;
        0.20,0.65,0.35
        ];


    line_styles = {'-','--',':','-.'};


    comparison_figure = figure( ...
        'Color','w',...
        'Name','导航方法对比',...
        'Position',[80,120,1500,600]);


    layout = tiledlayout(comparison_figure,1,2,...
        'TileSpacing','compact',...
        'Padding','compact');


    title(layout,...
        sprintf('%s：事件驱动导航方法对比',case_name));


    % ==========================================================
    % 左图：轨迹
    % ==========================================================

    nexttile;

    plot(truth_east/1000,...
        truth_north/1000,...
        'k-',...
        'LineWidth',1.7,...
        'DisplayName','真值');

    hold on;


    for result_index = 1:result_count

        [east,north] = ...
            position_to_local_plane_rts( ...
            nav_results{result_index}(display_indices,3:5),...
            origin);


        plot(east/1000,...
            north/1000,...
            'Color',colors(result_index,:),...
            'LineStyle',line_styles{result_index},...
            'LineWidth',1.25,...
            'DisplayName',names{result_index});

    end


    grid on;
    box on;
    axis tight;

    xlabel('东向位置（km）');
    ylabel('北向位置（km）');

    title('评价区间轨迹');

    legend('Location','best',...
        'Interpreter','none');



    % ==========================================================
    % 右图：误差
    % ==========================================================

    nexttile;

    hold on;


    for result_index = 1:result_count

        plot(elapsed_time(display_indices),...
            radial_error(display_indices,result_index),...
            'Color',colors(result_index,:),...
            'LineStyle',line_styles{result_index},...
            'LineWidth',1.2,...
            'DisplayName',names{result_index});

    end


    grid on;
    box on;
    axis tight;


    xlabel('时间（s）');
    ylabel('水平径向误差（m）');

    title('评价区间水平径向误差');


    legend('Location','best',...
        'Interpreter','none');


    if ~isempty(display_duration_s)

        xlim([0,...
            min(display_duration_s,elapsed_time(end))]);

    end


    exportgraphics(comparison_figure,...
        fullfile(figure_dir,...
        'navigation-comparison.png'),...
        'Resolution',300);


    savefig(comparison_figure,...
        fullfile(figure_dir,...
        'navigation-comparison.fig'));

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
