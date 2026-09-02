function outputs = run_experiment_rts_core(result_dir)
%RUN_EXPERIMENT_RTS_CORE 处理实测预处理数据，生成EKF、2RTS和旋转收缩结果。
% 主链采用米制误差状态：[位置N/E/D(m), 速度N/E/D(m/s), ...]；
% 名义位置 navstate.pos 始终保持 [纬度rad, 经度rad, 高度m]。

enable_fixed_lag_replay = true;

%% 事件驱动的实测测距/惯导四方法对比
% 每个测距时刻的处理顺序：
%   1）利用当前测距观测更新前向 15 状态 EKF；
%   2）对刚结束的区间执行一次 RTS；
%   3）当前区间产生的桥接误差作为上一段的末端条件，生成二次 RTS；
%   4）二次 RTS 一生成就立即触发同区间历史 IMU 重放；
%   5）对已完成的二次 RTS 区间执行残余桥接误差旋转收缩。

glvs;                                % 初始化 PSINS 使用的地球模型全局常量。
param = Param();

options.case_name = 'experiment';
options.range_interval_s = 420;
options.beacon_order = [1, 2, 3];
options.filter_range_std_m = 6;
options.filter_depth_std_m = 0.4;
options.use_adaptive_forward_update = false;
% RTS 默认每 1 s 保存一个关键节点；协方差仍在 100 Hz IMU 历元连续传播。
% 改为 0.01 可执行逐 IMU 历元 RTS，但计算时间和内存占用会显著增加。
options.rts_node_interval_s = 1;
% 第11个测距点位于4620 s，再保留1 s用于观察更新后的导航响应。
options.duration_s = 4621;
options.rts_update_interval_s = 1;
options.rts_position_std_m = 30;
options.rts_velocity_std_mps = 0.3;
options.boundary_guard_s = 60;
options.boundary_position_std_m = 80;
options.boundary_velocity_std_mps = 1.0;
options.innovation_gate_sigma = 3;
options.robust_gate_sigma = 6;
options.robust_std_scale = 3;

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
project_root = fileparts(fileparts(topic_dir));
input_dir = fullfile(project_root, 'data', 'inertial-experiment', ...
    'algorithm-exploration', 'input', 'experiment-preprocessed');

if nargin < 1 || isempty(result_dir)
    result_dir = fullfile(project_root, 'data', 'inertial-experiment', ...
        'algorithm-exploration', 'navigation-results', 'experiment', 'four-method-comparison');
end
result_dir = char(string(result_dir));

cfg = ProcessConfig_exper_m();
cfg.userange = true;
cfg.outputfolder = result_dir;
if ~exist(cfg.outputfolder, 'dir')
    mkdir(cfg.outputfolder);
end

%% 读取并整理观测数据
imu_all = readmatrix(cfg.imufilepath, 'FileType', 'text');
truth_all = readmatrix(cfg.truthpath, 'FileType', 'text');
rangedata = readmatrix(fullfile(input_dir, 'rangedata_noised.txt'), ...
    'FileType', 'text');
height_all = readmatrix(fullfile(input_dir, 'height_noised.txt'), ...
    'FileType', 'text');
options.range_boundary_reference_time_s = rangedata(1, 1);

start_time = max([cfg.starttime, imu_all(1, 1), truth_all(1, 2)]);
end_time = min([start_time + options.duration_s, cfg.endtime, ...
    imu_all(end, 1), truth_all(end, 2)]);
cfg.starttime = start_time;
cfg.endtime = end_time;

imu_mask = imu_all(:, 1) >= start_time & imu_all(:, 1) <= end_time;
imudata = imu_all(imu_mask, :);
range_mask = rangedata(:, 1) >= start_time & rangedata(:, 1) <= end_time;
rangedata = rangedata(range_mask, :);

height_values = interp1(height_all(:, 1), height_all(:, 2), imudata(:, 1), ...
    'linear', 'extrap');
height = [imudata(:, 1), height_values];

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
fixed_lag_replay_nav = nan(sample_count, 11);
fixed_lag_replay_is_processed = false(sample_count, 1);

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
fixed_lag_context = [];
fixed_lag_update_diagnostics = repmat( ...
    empty_fixed_lag_update_diagnostic(), 0, 1);
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

            % 二次 RTS 区间一生成就立刻重放同一段历史 IMU。重放器的
            % 完整导航状态和协方差在相邻区间间继承，不等待全时段结束。
            if enable_fixed_lag_replay
                replay_indices = previous_rts_block.segment_indices;
                [fixed_lag_segment, fixed_lag_context, ...
                        segment_update_diagnostics] = ...
                    replay_fixed_lag_segment( ...
                    imudata(replay_indices, :), height(replay_indices, :), ...
                    double_previous_nav, fixed_lag_context, cfg, options, param);
                [fixed_lag_replay_nav, fixed_lag_replay_is_processed] = ...
                    write_segment_result(fixed_lag_replay_nav, ...
                    fixed_lag_replay_is_processed, replay_indices, ...
                    fixed_lag_segment);
                fixed_lag_update_diagnostics = [ ...
                    fixed_lag_update_diagnostics; ...
                    segment_update_diagnostics]; %#ok<AGROW>
            end
            previous_index = previous_rts_block.diagnostic_index;
            rts_diagnostics(previous_index).second_pass_terminal_error_m = ...
                horizontal_error_norm_m(bridge_error);

            % 旋转收缩只在已经完成的二次 RTS 轨迹上执行。当前二次 RTS
            % 给出的新桥接节点用于修正更早一段轨迹，不重复运行 EKF/RTS。
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
                rotation_item.block_index = previous_double_block.block_index;
                rotation_item.start_time = previous_double_block.nav(1, 2);
                rotation_item.end_time = previous_double_block.nav(end, 2);
                rotation_item.residual_bridge_error_m = ...
                    horizontal_error_norm_m(residual_bridge_error);
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
        rts_info.terminal_error_m = horizontal_error_norm_m(terminal_error);
        rts_info.first_pass_bridge_error_m = ...
            horizontal_error_norm_m(bridge_error);
        rts_diagnostics(end + 1, 1) = rts_info; %#ok<SAGROW>

        previous_rts_block = rts_block;
        previous_rts_block.single_nav = single_segment_nav;
        previous_rts_block.segment_indices = rts_segment_indices;
        previous_rts_block.diagnostic_index = numel(rts_diagnostics);
        previous_rts_block.end_position = navstate.pos;
        previous_rts_block.first_error_nodes = first_error_nodes;

        % 将当前测距误差反馈到名义导航状态，供后续前向和反向推算使用。
        [kf, navstate] = myErrorFeedback_range_m(kf, navstate);
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
            update_decoupled_height_m( ...
            kf, navstate, height(imu_index, :));
    end
    kf = myInsPropagate_15state_m(navstate, this_imu, imu_dt, kf);
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

%% 生成长度一致、可以直接比较的结果
% 最后一个完整 RTS 区间没有下一段桥接误差，二次结果按 experiment 的
% 处理原则退化为该区间的一次 RTS 结果。
if ~isempty(previous_rts_block)
    [rts_double_nav, rts_double_is_processed] = write_segment_result( ...
        rts_double_nav, rts_double_is_processed, ...
        previous_rts_block.segment_indices, previous_rts_block.single_nav);
end
rts_double_nav(~rts_double_is_processed, :) = ...
    forward_nav(~rts_double_is_processed, :);
rts_rotation_nav(~rts_rotation_is_processed, :) = ...
    rts_double_nav(~rts_rotation_is_processed, :);

forward_path = fullfile(cfg.outputfolder, 'range-ins-forward.nav');
rts_double_path = fullfile(cfg.outputfolder, 'range-ins-rts-double.nav');
artifact_dir = exploration_artifact_dir(cfg.outputfolder);
rts_diagnostic_path = fullfile(artifact_dir, 'range-rts-diagnostics.csv');
rts_rotation_path = fullfile(cfg.outputfolder, ...
    'range-ins-rts-double-bridge-rotation.nav');
rts_rotation_diagnostic_path = fullfile(artifact_dir, ...
    'range-rts-bridge-rotation-diagnostics.csv');
fixed_lag_replay_path = fullfile(cfg.outputfolder, ...
    'range-ins-double-rts-position-velocity-fixed-lag-replay.nav');
fixed_lag_update_path = fullfile(artifact_dir, ...
    'range-ins-double-rts-position-velocity-fixed-lag-updates.csv');
fixed_lag_statistics_path = fullfile(artifact_dir, ...
    'fixed-lag-four-method-statistics.csv');

write_nav_file(forward_path, forward_nav);
write_nav_file(rts_double_path, rts_double_nav);
write_rts_diagnostics(rts_diagnostic_path, rts_diagnostics);
write_nav_file(rts_rotation_path, rts_rotation_nav);
write_rotation_diagnostics(rts_rotation_diagnostic_path, ...
    rts_rotation_diagnostics);

if enable_fixed_lag_replay
    write_nav_file(fixed_lag_replay_path, fixed_lag_replay_nav);
    writetable(struct2table(fixed_lag_update_diagnostics), ...
        fixed_lag_update_path);
end

outputs = struct( ...
    'case_name', options.case_name, ...
    'input_dir', input_dir, ...
    'truth_path', cfg.truthpath, ...
    'result_dir', cfg.outputfolder, ...
    'forward_path', forward_path, ...
    'double_rts_path', rts_double_path, ...
    'rotation_contraction_path', rts_rotation_path, ...
    'fixed_lag_replay_path', fixed_lag_replay_path, ...
    'fixed_lag_statistics_path', fixed_lag_statistics_path, ...
    'range_event_count', size(rangedata, 1), ...
    'fixed_lag_valid_count', nnz(fixed_lag_replay_is_processed), ...
    'fixed_lag_valid_time_s', imudata(fixed_lag_replay_is_processed, 1));

fprintf('前向滤波结果：%s\n', forward_path);
fprintf('二次 RTS 平滑结果：%s\n', rts_double_path);
fprintf('2RTS+旋转收缩结果：%s\n', rts_rotation_path);
if enable_fixed_lag_replay
    fprintf('固定滞后位置速度约束结果：%s\n', fixed_lag_replay_path);
end
end

%% 局部辅助函数
function kf = update_range_filter(navstate, range_data, depth_data, kf, ~)
    kf = myRangeUpdate_m(navstate, range_data, depth_data, kf);
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
    latitude = nominal_nav(:, 3) * param.D2R;
    height = nominal_nav(:, 5);
    [rm, rn] = wgs84_radii_rts(latitude);
    corrected_nav(:, 3) = nominal_nav(:, 3) ...
        - error_state(:, 1) ./ (rm + height) * param.R2D;
    corrected_nav(:, 4) = nominal_nav(:, 4) ...
        - error_state(:, 2) ./ ((rn + height) .* cos(latitude)) ...
        * param.R2D;
    % 米制第三维为下向误差D，故高度修正为 h + x_D。
    corrected_nav(:, 5) = nominal_nav(:, 5) + error_state(:, 3);
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

function value = horizontal_error_norm_m(error_state)
%HORIZONTAL_ERROR_NORM_M 米制状态前两维已是北、东位置误差（m）。
    value = hypot(error_state(1), error_state(2));
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

function [segment_nav, context, diagnostics] = replay_fixed_lag_segment( ...
        imudata, height, double_rts_nav, context, cfg, options, param)
%REPLAY_FIXED_LAG_SEGMENT 在2RTS区间生成时立即重放该段历史IMU。
% 相邻区间继承完整导航状态、误差状态和协方差，形成连续固定滞后轨迹。
    if isempty(context)
        cfg.initpos(1:2) = double_rts_nav(1, 3:4)' * param.D2R;
        cfg.initpos(3) = height(1, 2);
        cfg.initvel = double_rts_nav(1, 6:8)';
        [replay_kf, replay_state] = myInitialize_15state(cfg);
        replay_kf.depthstd = options.filter_depth_std_m;
        % 米制链的水平位置协方差保持m^2，不再误除地球曲率半径。
        replay_kf.P(1:2, 1:2) = eye(2) ...
            * options.rts_position_std_m ^ 2;
        replay_kf.P(4:5, 4:5) = eye(2) ...
            * options.rts_velocity_std_mps ^ 2;
    else
        replay_kf = context.kf;
        replay_state = context.navstate;
    end

    sample_count = size(imudata, 1);
    segment_nav = nan(sample_count, 11);
    replay_state.time = imudata(1, 1);
    segment_nav(1, :) = state_to_nav_row(replay_state, param);
    diagnostics = repmat(empty_fixed_lag_update_diagnostic(), 0, 1);
    next_update_time = ceil(imudata(1, 1) ...
        / options.rts_update_interval_s) * options.rts_update_interval_s;
    if next_update_time <= imudata(1, 1) + 1e-9
        next_update_time = next_update_time + options.rts_update_interval_s;
    end

    this_imu = imudata(1, :)';
    for imu_index = 2:sample_count
        last_imu = this_imu;
        this_imu = imudata(imu_index, :)';
        imu_dt = this_imu(1) - last_imu(1);
        time_tolerance = max(1e-8, abs(imu_dt) * 0.25);

        if last_imu(1) >= next_update_time - time_tolerance
            measurement_nav = interp1(double_rts_nav(:, 2), ...
                double_rts_nav(:, 3:7), last_imu(1), 'linear');
            [position_std_m, velocity_std_mps, is_boundary_guard] = ...
                select_fixed_lag_measurement_std(last_imu(1), options);
            [replay_kf, update_info] = ...
                update_fixed_lag_position_velocity( ...
                replay_state, measurement_nav(1:2)' * param.D2R, ...
                measurement_nav(4:5)', replay_kf, ...
                position_std_m, velocity_std_mps, options);

            if update_info.accepted
                [replay_kf, replay_state] = ...
                    feedback_fixed_lag_error(replay_kf, replay_state);
                segment_nav(imu_index - 1, :) = ...
                    state_to_nav_row(replay_state, param);
            end

            item = empty_fixed_lag_update_diagnostic();
            item.time_s = last_imu(1);
            item.position_std_m = update_info.effective_std_m;
            item.velocity_std_mps = ...
                update_info.effective_velocity_std_mps;
            item.in_boundary_guard = is_boundary_guard;
            item.innovation_m = update_info.innovation_m;
            item.velocity_innovation_mps = ...
                update_info.velocity_innovation_mps;
            item.normalized_innovation = update_info.normalized_innovation;
            item.robust_downweighted = update_info.robust_downweighted;
            item.accepted = update_info.accepted;
            diagnostics(end + 1, 1) = item; %#ok<AGROW>
            next_update_time = next_update_time ...
                + options.rts_update_interval_s;
        end

        last_state = replay_state;
        replay_state = InsMech(last_state, last_imu, this_imu);
        [replay_kf, replay_state] = update_decoupled_height_m( ...
            replay_kf, replay_state, height(imu_index, :));
        replay_state.time = this_imu(1);
        replay_kf = myInsPropagate_15state_m( ...
            replay_state, this_imu, imu_dt, replay_kf);
        segment_nav(imu_index, :) = state_to_nav_row(replay_state, param);
    end

    context = struct('kf', replay_kf, 'navstate', replay_state);
end

function [position_std_m, velocity_std_mps, is_boundary_guard] = ...
        select_fixed_lag_measurement_std(time_s, options)
%SELECT_FIXED_LAG_MEASUREMENT_STD 在2RTS交界附近连续降低伪量测权重。
    boundary_index = round((time_s ...
        - options.range_boundary_reference_time_s) ...
        / options.range_interval_s);
    nearest_boundary_time = options.range_boundary_reference_time_s ...
        + boundary_index * options.range_interval_s;
    distance_to_boundary = abs(time_s - nearest_boundary_time);
    is_boundary_guard = distance_to_boundary <= options.boundary_guard_s ...
        && time_s > options.boundary_guard_s;
    if is_boundary_guard
        normalized_distance = distance_to_boundary ...
            / options.boundary_guard_s;
        smooth_weight = 10 * normalized_distance ^ 3 ...
            - 15 * normalized_distance ^ 4 + 6 * normalized_distance ^ 5;
        position_std_m = options.boundary_position_std_m ...
            + (options.rts_position_std_m ...
            - options.boundary_position_std_m) * smooth_weight;
        velocity_std_mps = options.boundary_velocity_std_mps ...
            + (options.rts_velocity_std_mps ...
            - options.boundary_velocity_std_mps) * smooth_weight;
    else
        position_std_m = options.rts_position_std_m;
        velocity_std_mps = options.rts_velocity_std_mps;
    end
end

function [kf, info] = update_fixed_lag_position_velocity( ...
        navstate, measurement_position, measurement_velocity, kf, ...
        position_std_m, velocity_std_mps, options)
%UPDATE_FIXED_LAG_POSITION_VELOCITY 用2RTS水平位置速度更新15状态滤波器。
    horizontal_scale = [navstate.Rm + navstate.pos(3); ...
        (navstate.Rn + navstate.pos(3)) * cos(navstate.pos(1))];
    position_difference = navstate.pos(1:2) - measurement_position;
    position_residual_m = [ ...
        position_difference(1) * horizontal_scale(1); ...
        position_difference(2) * horizontal_scale(2)];
    measurement_residual = [ ...
        position_residual_m; ...
        navstate.vel(1:2) - measurement_velocity];
    measurement_matrix = zeros(4, kf.RANK);
    measurement_matrix(1:2, 1:2) = eye(2);
    measurement_matrix(3:4, 4:5) = eye(2);
    innovation = measurement_residual - measurement_matrix * kf.x;
    innovation_m = norm(innovation(1:2));
    velocity_innovation_mps = norm(innovation(3:4));
    normalized_innovation = max( ...
        innovation_m / max(position_std_m, eps), ...
        velocity_innovation_mps / max(velocity_std_mps, eps));

    accepted = normalized_innovation <= options.robust_gate_sigma;
    robust_downweighted = accepted && ...
        normalized_innovation > options.innovation_gate_sigma;
    effective_std_m = position_std_m;
    effective_velocity_std_mps = velocity_std_mps;
    if robust_downweighted
        effective_std_m = effective_std_m * options.robust_std_scale;
        effective_velocity_std_mps = effective_velocity_std_mps ...
            * options.robust_std_scale;
    end

    if accepted
        measurement_covariance = diag([ ...
            repmat(effective_std_m ^ 2, 2, 1); ...
            repmat(effective_velocity_std_mps ^ 2, 2, 1)]);
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

    info = struct('accepted', accepted, ...
        'robust_downweighted', robust_downweighted, ...
        'effective_std_m', effective_std_m, ...
        'effective_velocity_std_mps', effective_velocity_std_mps, ...
        'innovation_m', innovation_m, ...
        'velocity_innovation_mps', velocity_innovation_mps, ...
        'normalized_innovation', normalized_innovation);
end

function [kf, navstate] = feedback_fixed_lag_error(kf, navstate)
%FEEDBACK_FIXED_LAG_ERROR 反馈位置、速度及IMU零偏误差。
    [kf, navstate] = myErrorFeedback_range_m(kf, navstate);
end

function item = empty_fixed_lag_update_diagnostic()
%EMPTY_FIXED_LAG_UPDATE_DIAGNOSTIC 创建固定滞后伪量测诊断记录。
    item = struct('time_s', nan, 'position_std_m', nan, ...
        'velocity_std_mps', nan, 'in_boundary_guard', false, ...
        'innovation_m', nan, 'velocity_innovation_mps', nan, ...
        'normalized_innovation', nan, 'robust_downweighted', false, ...
        'accepted', false);
end
