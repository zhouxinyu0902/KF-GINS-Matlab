%% REALTIME FGO RANGE/INS/HEIGHT 360 s 固定滞后 15维预积分因子图组合导航
% 目标：在线 INS 机械编排 + 1 s 高度垂向约束 + 测距触发的 360 s 水平回溯优化。
% 逻辑：
% 1) 变量节点为 X_k = {pos, vel, att, bg, ba}，状态维数为 15。
% 2) IMU 因子为预积分式二元因子：由 X_i 和 IMU 片段预测 X_j，残差为 {pos, vel, att}。
% 3) bias 随机游走因子连接相邻节点的陀螺/加计零偏。
% 4) 高度因子按 1 s 关键帧加入，只反馈垂向位置 h 和垂向速度 vD。
% 5) 距离因子只在测距节点触发，用当前测距约束回溯优化前 360 s 的水平位置/水平速度。
% 6) 高度和距离分别走不同优化入口、不同因子集合、不同反馈自由度。
% 7) 优化后的当前节点继续用于后续 INS 机械编排；不先生成全程纯 INS 后处理。
%
% 注意：
% - 本程序只使用 myInitialize_15state 取得初始 navstate，不使用 KF P、KF phi 或 myInsPropagate_15state。
% - IMU 预积分因子的 Jacobian 默认使用数值差分，避免 Ji=-I 的过度简化。
% - 默认仍严格保留选择性量测反馈：高度只改 h/vD，距离只改水平位置/水平速度。

clear; clc; close all;

%% 1. 路径、配置与参数
this_dir = fileparts(mfilename('fullpath'));
project_root = fullfile(this_dir, '..', '..');
input_dir = fullfile(this_dir, 'input');
output_dir = fullfile(this_dir, 'output');

addpath(genpath(project_root));
addpath(input_dir);

param = Param();
cfg = config_simu(input_dir);
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

opts.node_interval_sec = 1;              % 关键帧间隔。若要每个 IMU 样点建节点，置 use_imu_sample_node = true。
opts.use_imu_sample_node = false;        % true 时节点频率等于 IMU 频率，计算量较大。

% 高度更新与距离更新采用不同窗口、不同因子集合、不同反馈掩码。
% 高度：高频小窗口，只反馈垂向位置/垂向速度。
% 距离：低频长窗口，只反馈水平位置/水平速度。
opts.height_window_nodes = 2;            % 高频高度窗口：默认相邻两个节点，对应每 1 s 垂向小优化。
opts.range_lag_sec = 360;                % 测距触发后回溯优化前 360 s 节点。
opts.range_window_nodes = [];            % 自动由 range_lag_sec/node_interval_sec 计算。
opts.max_iterations_height = 1;
opts.max_iterations_range = 5;
opts.lm_lambda = 1e-6;
opts.max_time_sec = inf;
opts.range_std = 6;
opts.height_std = 0.4;
opts.random_seed = 1;
opts.max_horizontal_step_m = 1000;
opts.max_vertical_step_m = 5;
opts.optimize_height_every_node = true;  % true：每个节点做高度/IMU小窗口优化。
opts.range_include_height_factors = true; % false：测距窗口只用IMU+距离；垂向由高度窗口独立修正。
opts.range_factor_scope = 'current';     % 'current'：测距更新只加入当前测距因子；'lag_all'：加入滞后区间内所有测距因子。
opts.print_every_node = 180;
opts.range_factor_mode = 'horizontal';   % '3d'、'horizontal'、'horizontal_from_slant'
opts.use_numeric_imu_jacobian = false;    % true：预积分因子对 X_i 数值求导；false：退回近似导数。
opts.manual_bias_correction = false;     % 若 InsMech 不使用 navstate.gyrbias/accbias，可置 true 手动扣除零偏。
opts.imu_mode = 'increment';             % 'increment'：[t,dtheta,dvel]；'rate'：[t,gyro,acc]。仅 manual_bias_correction=true 时使用。
opts.gyro_cols = 2:4;
opts.acc_cols = 5:7;

% 15维状态：[lat, lon, h, vN, vE, vD, roll, pitch, yaw, bgx,bgy,bgz, bax,bay,baz]
rank = 15;
opts.state_scale = [ ...
    1e-6; 1e-6; 1; ...
    0.1; 0.1; 0.1; ...
    1e-3; 1e-3; 1e-3; ...
    1e-5; 1e-5; 1e-5; ...
    1e-4; 1e-4; 1e-4];

% 因子触发的反馈自由度。默认严格保留你的架构约束：
% 高度因子只反馈垂向；距离因子只反馈水平位置/水平速度。
opts.height_feedback_mask = false(rank, 1);
opts.height_feedback_mask([3, 6]) = true;
opts.range_feedback_mask = false(rank, 1);
opts.range_feedback_mask([1, 2, 4, 5]) = true;
% 若调试发现主要误差来自零偏，可临时打开下面两行，不建议作为默认架构。
% opts.range_feedback_mask(10:15) = true;
% opts.height_feedback_mask(13:15) = true;

% 先验标准差，单位与状态一致。窗口首节点用它作为锚点。
opts.prior_std = [ ...
    5 / 6378137; 5 / 6378137; 1.0; ...
    0.05; 0.05; 0.05; ...
    1e-3; 1e-3; 1e-3; ...
    1e-7; 1e-7; 1e-7; ...
    1e-4; 1e-4; 1e-4];

% IMU预积分因子的等效噪声。这里是调参入口，不使用KF协方差。
% 这里不是原始 IMU 白噪声，而是 1 s 关键帧间机械编排残差的等效权重。
opts.imu_pos_std_m_per_s = 10;       % 原 0.5 太松，先降到 0.08
opts.imu_h_std_m_per_s   = 5;       % 垂向由高度强约束，可更小
opts.imu_vel_std_mps_per_s = 1;    % 原 0.02 偏松
opts.imu_att_std_rad_per_s = 1e-3;     % 原 5e-4 偏松

% bias 随机游走。不要用 1e-6 / 1e-5 那么大。
% 这里设置为“360 s 内允许 bias 有小幅变化”，主要避免 bias 在窗口内乱漂。
opts.gyrbias_rw_std = 5e-10;           % [rad/s/sqrt(s)]
opts.accbias_rw_std = 5e-7;            % [m/s^2/sqrt(s)]
%% 2. 读取 IMU、高度、距离量测
rng(opts.random_seed);
imudata = importdata(cfg.imufilepath);
truth = importdata(cfg.truthpath);
heightdata = truth(:, [2, 5]);
range_streams = { ...
    importdata(cfg.rangefile1path), ...
    importdata(cfg.rangefile2path), ...
    importdata(cfg.rangefile3path)};
rangedata = buildRange360(range_streams);

cfg.starttime = max([cfg.starttime, imudata(1, 1), heightdata(1, 1)]);
cfg.endtime = min([cfg.endtime, imudata(end, 1), heightdata(end, 1), rangedata(end, 1)]);
if isfinite(opts.max_time_sec)
    cfg.endtime = min(cfg.endtime, cfg.starttime + opts.max_time_sec);
end

imudata = imudata(imudata(:, 1) >= cfg.starttime & imudata(:, 1) <= cfg.endtime, :);
heightdata = heightdata(heightdata(:, 1) >= cfg.starttime & heightdata(:, 1) <= cfg.endtime, :);
rangedata = rangedata(rangedata(:, 1) >= cfg.starttime & rangedata(:, 1) <= cfg.endtime, :);

heightdata(:, 2) = heightdata(:, 2) + opts.height_std * randn(size(heightdata, 1), 1);
rangedata(:, 3) = rangedata(:, 3) + opts.range_std * randn(size(rangedata, 1), 1);

imu_times = imudata(:, 1);
if opts.use_imu_sample_node
    node_indices = (1:length(imu_times))';
    node_times = imu_times;
else
    node_times = (cfg.starttime:opts.node_interval_sec:cfg.endtime)';
    node_indices = round(interp1(imu_times, (1:length(imu_times))', node_times, 'nearest', 'extrap'));
    [node_indices, keep] = unique(node_indices, 'stable'); 
    node_times = imu_times(node_indices);
end
num_nodes = length(node_indices);

range_indices = round(interp1(imu_times, (1:length(imu_times))', rangedata(:, 1), 'nearest', 'extrap'));
[found, range_nodes] = ismember(range_indices, node_indices);
rangedata = rangedata(found, :);
range_nodes = range_nodes(found);

height_factors = buildHeightFactors(node_times, heightdata, opts.height_std);
range_factors = buildRangeFactors(rangedata, range_nodes, opts.range_std);

fprintf('\n========== REALTIME FGO RANGE/INS/HEIGHT 360s ==========' );
fprintf('\n处理时间: %.2f -> %.2f s\n', cfg.starttime, cfg.endtime);
if isempty(opts.range_window_nodes)
    opts.range_window_nodes = round(opts.range_lag_sec / opts.node_interval_sec) + 1;
end
fprintf('节点数: %d, 高度因子: %d, 距离因子: %d\n', ...
    num_nodes, numel(height_factors), numel(range_factors));
fprintf('高度更新窗口: %d 节点；测距回溯窗口: %d 节点，约 %.1f s；测距因子范围: %s\n', ...
    opts.height_window_nodes, opts.range_window_nodes, ...
    (opts.range_window_nodes - 1) * opts.node_interval_sec, opts.range_factor_scope);

%% 3. 在线机械编排 + 基础因子图滑动窗口优化
[~, navstate] = myInitialize_15state(cfg);
navstate = ensureBiasFields(navstate);
nav_nodes = cell(num_nodes, 1);
nav_nodes{1} = navstate;

% 仅用首个高度量测初始化首节点高程；之后高度作为因子处理。
nav_nodes(1) = applyHeightToNodes(nav_nodes(1), height_factors(1));
navstate = nav_nodes{1};

history = [];
range_ptr = 1;

for k = 2:num_nodes
    % 3.1 从上一个优化后的节点继续 INS 机械编排到当前节点。
    navstate = nav_nodes{k - 1};
    navstate = propagateNav(navstate, node_indices(k - 1), node_indices(k), imudata, opts);
    nav_nodes{k} = navstate;

    has_range = range_ptr <= numel(range_nodes) && k == range_nodes(range_ptr);

    % 3.2 高频高度更新：只构建 IMU + 高度因子的小窗口，只反馈 h 和 vD。
    if opts.optimize_height_every_node
        height_first = max(1, k - opts.height_window_nodes + 1);
        local_height = selectHeightFactors(height_factors, height_first, k);
        local_range = [];

        [nav_nodes, local_history] = optimizeGraphWindow(nav_nodes, height_first, k, node_indices, imudata, ...
            local_height, local_range, opts.height_feedback_mask, opts.max_iterations_height, opts, rank, param);
        navstate = nav_nodes{k};

        if ~isempty(local_history)
            history = [history; [k * ones(size(local_history, 1), 1), zeros(size(local_history, 1), 1), local_history]]; %#ok<AGROW>
        end

        if mod(k, opts.print_every_node) == 0 && ~isempty(local_history)
            row = local_history(end, :);
            fprintf('height update node %d/%d: %.2f s, preint %.3g, bias %.3g, height %.3g, maxH %.3f m, maxZ %.3f m\n', ...
                k, num_nodes, node_times(k), row(3), row(4), row(5), row(7), row(8));
        end
    end

    % 3.3 低频距离更新：到测距节点时回溯前 range_lag_sec 秒，只反馈水平位置/水平速度。
    if has_range
        range_first = max(1, k - opts.range_window_nodes + 1);
        switch lower(opts.range_factor_scope)
            case 'current'
                local_range = range_factors(range_ptr);
            case 'lag_all'
                local_range = selectRangeFactors(range_factors, range_first, k);
            otherwise
                error('未知测距因子范围选项: %s', opts.range_factor_scope);
        end
        if opts.range_include_height_factors
            local_height = selectHeightFactors(height_factors, range_first, k);
        else
            local_height = [];
        end

        [nav_nodes, local_history] = optimizeGraphWindow(nav_nodes, range_first, k, node_indices, imudata, ...
            local_height, local_range, opts.range_feedback_mask, opts.max_iterations_range, opts, rank, param);
        navstate = nav_nodes{k};  

        fprintf('\nrange update %d/%d: %.2f s, window nodes %d, range factors %d\n', ...
            range_ptr, numel(range_nodes), node_times(k), k - range_first + 1, numel(local_range));
        printHistory(local_history, opts.max_iterations_range);

        if ~isempty(local_history)
            history = [history; [k * ones(size(local_history, 1), 1), ones(size(local_history, 1), 1), local_history]]; %#ok<AGROW>
        end

        range_ptr = range_ptr + 1;
    end
end

%% 4. 输出
key_path = fullfile(output_dir, 'FGO-RANGE-INS-HEIGHT-keyframes.nav');
smooth_path = fullfile(output_dir, 'FGO-RANGE-INS-accurate.nav');
% writeNavRows(key_path, nav_nodes, param);
writeFullRateFromNodes(smooth_path, nav_nodes, node_indices, imudata, param);

result.opts = opts;
result.history = history;
result.key_path = key_path;
result.smooth_path = smooth_path;
% save(fullfile(output_dir, 'realtime_fgo_range_ins_height_360s_selective_result.mat'), 'result');

fprintf('\nREALTIME FGO 360s preint15 selective 完成。\n');
fprintf('关键帧结果: %s\n', key_path);
fprintf('100 Hz 结果: %s\n', smooth_path);

%% 局部函数

function [nav_nodes, history] = optimizeGraphWindow(nav_nodes, first_node, last_node, node_indices, imudata, ...
        height_factors, range_factors, feedback_mask, max_iterations, opts, rank, param)
    local_nodes = nav_nodes(first_node:last_node);
    local_node_indices = node_indices(first_node:last_node) - node_indices(first_node) + 1;
    local_imudata = imudata(node_indices(first_node):node_indices(last_node), :);

    % 将全局节点编号转为局部节点编号。
    for i = 1:numel(height_factors)
        height_factors(i).node = height_factors(i).node - first_node + 1;
    end
    for i = 1:numel(range_factors)
        range_factors(i).node = range_factors(i).node - first_node + 1;
    end

    history = [];
    prior_nav = local_nodes{1};

    for iter = 1:max_iterations
        [A, b, costs] = assembleGraph(local_nodes, local_node_indices, local_imudata, ...
            height_factors, range_factors, prior_nav, opts, rank);

        dx_vec = solveMaskedScaledNormalEquation(A, b, opts.state_scale, feedback_mask, ...
            numel(local_nodes), rank, opts.lm_lambda);
        dx = reshape(dx_vec, rank, numel(local_nodes));

        max_step = maxHorizontalStep(local_nodes, dx, param);
        max_vertical_step = maxVerticalStep(dx);
        scale = 1;
        if max_step > opts.max_horizontal_step_m
            scale = min(scale, opts.max_horizontal_step_m / max_step);
        end
        if max_vertical_step > opts.max_vertical_step_m
            scale = min(scale, opts.max_vertical_step_m / max_vertical_step);
        end
        if scale < 1
            dx = dx * scale;
            max_step = maxHorizontalStep(local_nodes, dx, param);
            max_vertical_step = maxVerticalStep(dx);
        end

        for kk = 1:numel(local_nodes)
            local_nodes{kk} = retractNavState(local_nodes{kk}, dx(:, kk));
        end

        history(end + 1, :) = [iter, costs.prior, costs.preint, costs.bias, costs.height, costs.range, max_step, max_vertical_step]; %#ok<AGROW>
    end

    nav_nodes(first_node:last_node) = local_nodes;
end

function [A, b, costs] = assembleGraph(local_nodes, local_node_indices, local_imudata, ...
        height_factors, range_factors, prior_nav, opts, rank)
    num_nodes = numel(local_nodes);
    num_var = rank * num_nodes;
    A = sparse(num_var, num_var);
    b = zeros(num_var, 1);

    costs.prior = 0;
    costs.preint = 0;
    costs.bias = 0;
    costs.height = 0;
    costs.range = 0;

    % 先验因子：锚定窗口首节点，避免窗口内整体漂移。
    idx = nodeBlock(1, rank);
    r = navError15(local_nodes{1}, prior_nav);
    H = eye(rank);
    W = diag(1 ./ (opts.prior_std(:) .^ 2));
    A(idx, idx) = A(idx, idx) + H' * W * H;
    b(idx) = b(idx) + H' * W * r;
    costs.prior = costs.prior + r' * W * r;

    % IMU 预积分因子：连接相邻状态节点，残差为位置/速度/姿态 9 维。
    for k = 1:(num_nodes - 1)
        idx_i = nodeBlock(k, rank);
        idx_j = nodeBlock(k + 1, rank);
        [r, Ji, Jj, sigma] = imuPreintFactorResidualJacobian(local_nodes{k}, local_nodes{k + 1}, ...
            local_node_indices(k), local_node_indices(k + 1), local_imudata, opts, rank);
        W = diag(1 ./ (sigma(:) .^ 2));

        A(idx_i, idx_i) = A(idx_i, idx_i) + Ji' * W * Ji;
        A(idx_i, idx_j) = A(idx_i, idx_j) + Ji' * W * Jj;
        A(idx_j, idx_i) = A(idx_j, idx_i) + Jj' * W * Ji;
        A(idx_j, idx_j) = A(idx_j, idx_j) + Jj' * W * Jj;
        b(idx_i) = b(idx_i) + Ji' * W * r;
        b(idx_j) = b(idx_j) + Jj' * W * r;
        costs.preint = costs.preint + r' * W * r;

        % bias 随机游走因子：连接 bg/ba。
        [rb, Jbi, Jbj, sigmab] = biasRandomWalkFactor(local_nodes{k}, local_nodes{k + 1}, ...
            local_node_indices(k), local_node_indices(k + 1), local_imudata, opts, rank);
        Wb = diag(1 ./ (sigmab(:) .^ 2));
        A(idx_i, idx_i) = A(idx_i, idx_i) + Jbi' * Wb * Jbi;
        A(idx_i, idx_j) = A(idx_i, idx_j) + Jbi' * Wb * Jbj;
        A(idx_j, idx_i) = A(idx_j, idx_i) + Jbj' * Wb * Jbi;
        A(idx_j, idx_j) = A(idx_j, idx_j) + Jbj' * Wb * Jbj;
        b(idx_i) = b(idx_i) + Jbi' * Wb * rb;
        b(idx_j) = b(idx_j) + Jbj' * Wb * rb;
        costs.bias = costs.bias + rb' * Wb * rb;
    end

    % 高度因子：一元因子，只连接一个状态节点。
    for m = 1:numel(height_factors)
        f = height_factors(m);
        idx = nodeBlock(f.node, rank);
        [r, H, sigma] = heightFactorResidualJacobian(local_nodes{f.node}, f, rank);
        W = 1 / sigma ^ 2;
        A(idx, idx) = A(idx, idx) + W * (H' * H);
        b(idx) = b(idx) + W * H' * r;
        costs.height = costs.height + W * r ^ 2;
    end

    % 距离因子：一元因子，只连接一个状态节点。
    for m = 1:numel(range_factors)
        f = range_factors(m);
        idx = nodeBlock(f.node, rank);
        [r, H, sigma] = rangeFactorResidualJacobian(local_nodes{f.node}, f, opts, rank);
        W = 1 / sigma ^ 2;
        A(idx, idx) = A(idx, idx) + W * (H' * H);
        b(idx) = b(idx) + W * H' * r;
        costs.range = costs.range + W * r ^ 2;
    end
end

function [r, Ji, Jj, sigma] = imuPreintFactorResidualJacobian(nav_i, nav_j, idx_i, idx_j, imudata, opts, rank)
    r = imuPreintResidual(nav_i, nav_j, idx_i, idx_j, imudata, opts);

    % r = X_j - f(X_i, imu, b_i)。J_j 只对 pos/vel/att 有单位导数。
    Jj = zeros(9, rank);
    Jj(:, 1:9) = eye(9);

    if opts.use_numeric_imu_jacobian
        Ji = numericPreintJacobian(nav_i, nav_j, idx_i, idx_j, imudata, opts, rank);
    else
        Ji = zeros(9, rank);
        Ji(:, 1:9) = -eye(9);
    end

    dt = max(imudata(idx_j, 1) - imudata(idx_i, 1), eps);
    param = Param();
    DR = positionDr(nav_j.pos, param);
    pos_std_m = opts.imu_pos_std_m_per_s * max(dt, 1);
    sigma = [ ...
        pos_std_m / abs(DR(1, 1)); ...
        pos_std_m / abs(DR(2, 2)); ...
        opts.imu_h_std_m_per_s * max(dt, 1); ...
        opts.imu_vel_std_mps_per_s * max(dt, 1); ...
        opts.imu_vel_std_mps_per_s * max(dt, 1); ...
        opts.imu_vel_std_mps_per_s * max(dt, 1); ...
        opts.imu_att_std_rad_per_s * max(dt, 1); ...
        opts.imu_att_std_rad_per_s * max(dt, 1); ...
        opts.imu_att_std_rad_per_s * max(dt, 1)];
end

function r = imuPreintResidual(nav_i, nav_j, idx_i, idx_j, imudata, opts)
    nav_pred = propagateNav(nav_i, idx_i, idx_j, imudata, opts);
    r = navErrorImu(nav_j, nav_pred);
end

function Ji = numericPreintJacobian(nav_i, nav_j, idx_i, idx_j, imudata, opts, rank)
    eps_vec = [ ...
        1e-8; 1e-8; 1e-3; ...
        1e-4; 1e-4; 1e-4; ...
        1e-6; 1e-6; 1e-6; ...
        1e-8; 1e-8; 1e-8; ...
        1e-6; 1e-6; 1e-6];
    Ji = zeros(9, rank);
    for d = 1:rank
        dp = zeros(rank, 1);
        dp(d) = eps_vec(d);
        nav_plus = perturbNavState(nav_i, dp);
        nav_minus = perturbNavState(nav_i, -dp);
        r_plus = imuPreintResidual(nav_plus, nav_j, idx_i, idx_j, imudata, opts);
        r_minus = imuPreintResidual(nav_minus, nav_j, idx_i, idx_j, imudata, opts);
        Ji(:, d) = (r_plus - r_minus) / (2 * eps_vec(d));
    end
end

function [r, Ji, Jj, sigma] = biasRandomWalkFactor(nav_i, nav_j, idx_i, idx_j, imudata, opts, rank)
    nav_i = ensureBiasFields(nav_i);
    nav_j = ensureBiasFields(nav_j);
    r = [nav_j.gyrbias - nav_i.gyrbias; nav_j.accbias - nav_i.accbias];
    Ji = zeros(6, rank);
    Jj = zeros(6, rank);
    Ji(:, 10:15) = -eye(6);
    Jj(:, 10:15) = eye(6);
    dt = max(imudata(idx_j, 1) - imudata(idx_i, 1), eps);
    sigma = [opts.gyrbias_rw_std * sqrt(dt) * ones(3, 1); ...
             opts.accbias_rw_std * sqrt(dt) * ones(3, 1)];
end

function [r, H, sigma] = heightFactorResidualJacobian(nav, factor, rank)
    r = nav.pos(3) - factor.z;
    H = zeros(1, rank);
    H(3) = 1;
    sigma = factor.sigma;
end

function [r, H, sigma] = rangeFactorResidualJacobian(nav, factor, opts, rank)
    param = Param();
    beacon = factor.beacon(:);
    [rm, rn] = getRmRn(beacon(1), param);
    DR = diag([rm + beacon(3), (rn + beacon(3)) * cos(beacon(1)), -1]);
    delta = DR * (nav.pos - beacon);
    H = zeros(1, rank);

    switch lower(opts.range_factor_mode)
        case '3d'
            predicted = max(norm(delta), 1e-6);
            measured = factor.range;
            derivative = (nav.pos' - beacon') * (DR ^ 2) / predicted;
            H(1, 1:3) = derivative(1:3);
        case 'horizontal'
            predicted = max(norm(delta(1:2)), 1e-6);
            measured = factor.range;
            derivative = (nav.pos' - beacon') * (DR ^ 2) / predicted;
            H(1, 1:2) = derivative(1:2);
        case 'horizontal_from_slant'
            predicted = max(norm(delta(1:2)), 1e-6);
            measured = sqrt(max(factor.range ^ 2 - delta(3) ^ 2, 0));
            derivative = (nav.pos' - beacon') * (DR ^ 2) / predicted;
            H(1, 1:2) = derivative(1:2);
        otherwise
            error('未知距离因子模式: %s', opts.range_factor_mode);
    end

    r = predicted - measured;
    sigma = factor.sigma;
end

function nav = propagateNav(nav, idx_i, idx_j, imudata, opts)
    nav = ensureBiasFields(nav);
    for imuindex = (idx_i + 1):idx_j
        lastimu = imudata(imuindex - 1, :)';
        thisimu = imudata(imuindex, :)';
        if nargin >= 5 && isfield(opts, 'manual_bias_correction') && opts.manual_bias_correction
            [lastimu, thisimu] = biasCorrectImuPair(lastimu, thisimu, nav, opts);
        end
        nav = InsMech(nav, lastimu, thisimu);
        nav = ensureBiasFields(nav);
    end
end

function [lastimu_corr, thisimu_corr] = biasCorrectImuPair(lastimu, thisimu, nav, opts)
    lastimu_corr = lastimu;
    thisimu_corr = thisimu;
    dt = thisimu(1) - lastimu(1);
    if dt <= 0
        return;
    end
    switch lower(opts.imu_mode)
        case 'increment'
            thisimu_corr(opts.gyro_cols) = thisimu_corr(opts.gyro_cols) - nav.gyrbias(:) * dt;
            thisimu_corr(opts.acc_cols) = thisimu_corr(opts.acc_cols) - nav.accbias(:) * dt;
        case 'rate'
            thisimu_corr(opts.gyro_cols) = thisimu_corr(opts.gyro_cols) - nav.gyrbias(:);
            thisimu_corr(opts.acc_cols) = thisimu_corr(opts.acc_cols) - nav.accbias(:);
        otherwise
            error('未知 IMU 模式: %s', opts.imu_mode);
    end
end

function r = navErrorImu(nav_current, nav_ref)
    r = zeros(9, 1);
    r(1:3) = nav_current.pos - nav_ref.pos;
    r(4:6) = nav_current.vel - nav_ref.vel;
    r(7:9) = wrapAngle(nav_current.att - nav_ref.att);
end

function r = navError15(nav_current, nav_ref)
    nav_current = ensureBiasFields(nav_current);
    nav_ref = ensureBiasFields(nav_ref);
    r = zeros(15, 1);
    r(1:3) = nav_current.pos - nav_ref.pos;
    r(4:6) = nav_current.vel - nav_ref.vel;
    r(7:9) = wrapAngle(nav_current.att - nav_ref.att);
    r(10:12) = nav_current.gyrbias - nav_ref.gyrbias;
    r(13:15) = nav_current.accbias - nav_ref.accbias;
end

function nav = retractNavState(nav, dx)
    nav = ensureBiasFields(nav);
    nav.pos = nav.pos - dx(1:3);
    nav.vel = nav.vel - dx(4:6);
    nav.att = wrapAngle(nav.att - dx(7:9));
    nav.gyrbias = nav.gyrbias - dx(10:12);
    nav.accbias = nav.accbias - dx(13:15);
    param = Param();
    [nav.Rm, nav.Rn] = getRmRn(nav.pos(1), param);
    nav.gravity = getGravity(nav.pos);
end

function nav = perturbNavState(nav, dx)
    nav = ensureBiasFields(nav);
    nav.pos = nav.pos + dx(1:3);
    nav.vel = nav.vel + dx(4:6);
    nav.att = wrapAngle(nav.att + dx(7:9));
    nav.gyrbias = nav.gyrbias + dx(10:12);
    nav.accbias = nav.accbias + dx(13:15);
    param = Param();
    [nav.Rm, nav.Rn] = getRmRn(nav.pos(1), param);
    nav.gravity = getGravity(nav.pos);
end

function nav = ensureBiasFields(nav)
    if ~isfield(nav, 'gyrbias') || isempty(nav.gyrbias)
        nav.gyrbias = zeros(3, 1);
    end
    if ~isfield(nav, 'accbias') || isempty(nav.accbias)
        nav.accbias = zeros(3, 1);
    end
    nav.gyrbias = nav.gyrbias(:);
    nav.accbias = nav.accbias(:);
end

function factors = buildHeightFactors(node_times, heightdata, height_std)
    if numel(node_times) == 1
        edges = [-inf; inf];
    else
        edges = [-inf; (node_times(1:end-1) + node_times(2:end)) / 2; inf];
    end
    bin = discretize(heightdata(:, 1), edges);
    template = struct('node', 0, 'z', 0, 'sigma', 0, 'count', 0);
    factors = repmat(template, numel(node_times), 1);
    for k = 1:numel(node_times)
        values = heightdata(bin == k, 2);
        if isempty(values)
            [~, idx] = min(abs(heightdata(:, 1) - node_times(k)));
            values = heightdata(idx, 2);
        end
        factors(k).node = k;
        factors(k).z = mean(values);
        factors(k).sigma = height_std / sqrt(max(1, numel(values)));
        factors(k).count = numel(values);
    end
end

function factors = buildRangeFactors(rangedata, range_nodes, range_std)
    template = struct('node', 0, 'range', 0, 'beacon', zeros(3, 1), 'sigma', 0, 'time', 0);
    factors = repmat(template, size(rangedata, 1), 1);
    for k = 1:size(rangedata, 1)
        factors(k).node = range_nodes(k);
        factors(k).time = rangedata(k, 1);
        factors(k).range = rangedata(k, 3);
        factors(k).beacon = rangedata(k, 4:6)';
        factors(k).sigma = range_std;
    end
end

function local = selectHeightFactors(factors, first_node, last_node)
    mask = [factors.node] >= first_node & [factors.node] <= last_node;
    local = factors(mask);
end

function local = selectRangeFactors(factors, first_node, last_node)
    if isempty(factors)
        local = factors;
        return;
    end
    mask = [factors.node] >= first_node & [factors.node] <= last_node;
    local = factors(mask);
end

function rangedata = buildRange360(range_streams)
    sample_step = 360;
    for i = 1:numel(range_streams)
        range_streams{i} = range_streams{i}(sample_step:sample_step:end, :);
    end
    n = min(cellfun(@(x) size(x, 1), range_streams));
    rangedata = zeros(n, size(range_streams{1}, 2));
    seq = [1, 2, 3];
    for i = 1:3
        rows = i:3:n;
        rangedata(rows, :) = range_streams{seq(i)}(rows, :);
    end
    rangedata = rangedata(any(rangedata, 2), :);
end

function nav_nodes = applyHeightToNodes(nav_nodes, height_factors)
    param = Param();
    for k = 1:numel(height_factors)
        node = height_factors(k).node;
        if node < 1 || node > numel(nav_nodes)
            continue;
        end
        nav_nodes{node}.pos(3) = height_factors(k).z;
        [nav_nodes{node}.Rm, nav_nodes{node}.Rn] = getRmRn(nav_nodes{node}.pos(1), param);
        nav_nodes{node}.gravity = getGravity(nav_nodes{node}.pos);
    end
end

function dx_vec = solveScaledNormalEquation(A, b, state_scale, num_nodes, lambda)
    scale_vec = repmat(state_scale(:), num_nodes, 1);
    S = spdiags(scale_vec, 0, length(scale_vec), length(scale_vec));
    A = (A + A') / 2;
    A_scaled = S * A * S;
    damping = lambda * max(1, full(max(abs(diag(A_scaled)))));
    y = (A_scaled + speye(size(A_scaled)) * damping) \ (S * b);
    dx_vec = scale_vec .* y;
end

function dx_vec = solveMaskedScaledNormalEquation(A, b, state_scale, feedback_mask, num_nodes, rank, lambda)
    free_idx = false(rank * num_nodes, 1);
    feedback_mask = feedback_mask(:);
    for node = 1:num_nodes
        idx = nodeBlock(node, rank);
        free_idx(idx(feedback_mask)) = true;
    end

    dx_vec = zeros(rank * num_nodes, 1);
    if ~any(free_idx)
        return;
    end

    dx_free = solveScaledNormalEquation(A(free_idx, free_idx), b(free_idx), ...
        state_scale(feedback_mask), num_nodes, lambda);
    dx_vec(free_idx) = dx_free;
end

function max_step = maxHorizontalStep(nav_nodes, dx, param)
    max_step = 0;
    for k = 1:numel(nav_nodes)
        DR = positionDr(nav_nodes{k}.pos, param);
        max_step = max(max_step, norm(DR(1:2, 1:2) * dx(1:2, k)));
    end
end

function max_step = maxVerticalStep(dx)
    if isempty(dx)
        max_step = 0;
    else
        max_step = max(abs(dx(3, :)));
    end
end

function DR = positionDr(pos, param)
    [rm, rn] = getRmRn(pos(1), param);
    DR = diag([rm + pos(3), (rn + pos(3)) * cos(pos(1)), -1]);
end

function idx = nodeBlock(node, rank)
    idx = (node - 1) * rank + (1:rank);
end

function printHistory(local_history, max_iterations)
    for ii = 1:size(local_history, 1)
        fprintf('  iter %d/%d: prior %.3g, preint %.3g, bias %.3g, height %.3g, range %.3g, maxH %.3f m, maxZ %.3f m\n', ...
            local_history(ii, 1), max_iterations, local_history(ii, 2), local_history(ii, 3), ...
            local_history(ii, 4), local_history(ii, 5), local_history(ii, 6), local_history(ii, 7), local_history(ii, 8));
    end
end

function angle = wrapAngle(angle)
    angle = mod(angle + pi, 2 * pi) - pi;
end

function writeFullRateFromNodes(filepath, nav_nodes, node_indices, imudata, param)
    fp = fopen(filepath, 'wt');
    if fp < 0
        error('无法打开输出文件: %s', filepath);
    end
    cleaner = onCleanup(@() fclose(fp));  

    row_id = 1;
    for seg = 1:(numel(nav_nodes) - 1)
        first_idx = node_indices(seg);
        last_idx = node_indices(seg + 1);
        start_nav = nav_nodes{seg};
        end_nav = nav_nodes{seg + 1};
        start_local = 1;
        if seg > 1
            start_local = 2;
        end
        denom = max(1, last_idx - first_idx);
        for local = start_local:(last_idx - first_idx + 1)
            global_idx = first_idx + local - 1;
            alpha = (global_idx - first_idx) / denom;
            pos = (1 - alpha) * start_nav.pos + alpha * end_nav.pos;
            vel = (1 - alpha) * start_nav.vel + alpha * end_nav.vel;
            att = wrapAngle(start_nav.att + alpha * wrapAngle(end_nav.att - start_nav.att));
            row = [row_id; imudata(global_idx, 1); pos(1) * param.R2D; pos(2) * param.R2D; ...
                pos(3); vel; att * param.R2D];
            fprintf(fp, ['%6d %12.6f %14.9f %14.9f %10.4f ', ...
                '%10.5f %10.5f %10.5f %10.6f %10.6f %10.6f\n'], row);
            row_id = row_id + 1;
        end
    end
end

function writeNavRows(filepath, navs, param)
    fp = fopen(filepath, 'wt');
    if fp < 0
        error('无法打开输出文件: %s', filepath);
    end
    cleaner = onCleanup(@() fclose(fp));  
    for k = 1:length(navs)
        nav = navs{k};
        row = [k; nav.time; nav.pos(1) * param.R2D; nav.pos(2) * param.R2D; ...
            nav.pos(3); nav.vel; nav.att * param.R2D];
        fprintf(fp, ['%6d %12.6f %14.9f %14.9f %10.4f ', ...
            '%10.5f %10.5f %10.5f %10.6f %10.6f %10.6f\n'], row);
    end
end
