%% REALTIME FGO RANGE/INS/HEIGHT - full 15-state fixed-lag smoother
% 改进要点：
% 1) 采用米制局部误差状态 [dN,dE,dD,dvN,dvE,dvD,droll,dpitch,dyaw,dbg,dba]，改善数值条件。
% 2) IMU 因子为完整 15 维离散机械编排因子；对起始状态进行数值线性化，保留位置/速度/姿态/零偏耦合。
% 3) 对 IMU 测量白噪声进行数值灵敏度传播，形成每个关键帧区间的 15x15 协方差；零偏随机游走并入同一因子。
% 4) 测距窗口默认联合使用窗口内全部测距因子，并释放完整 15 维误差状态。
% 5) 使用 Schur 补顺序边缘化移出窗口的节点，保留历史量测形成的边缘先验。
% 6) 使用鲁棒测距核、自适应 LM、步长限制和收敛判据。
%
% 重要说明：
% - 本程序沿用工程中的 InsMech、Param、getRmRn、getGravity、
%   config_factor_graph_simulation、myInitialize_15state。
% - 这里的“预积分因子”是与现有 InsMech 完全一致的离散区间传播因子，不是 GTSAM/Forster 的旋转流形预积分类。
% - 必须按实际 IMU 数据手册核对 gyro_noise_density、acc_noise_density、gyrbias_rw_std、accbias_rw_std。
% - 必须确认 InsMech 是否已在内部扣除 navstate.gyrbias/accbias，避免重复补偿。

clear; clc; close all;

%% 1. 路径、配置与参数
topic_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(topic_root);
paths = setup_factor_graph_navigation();
input_dir = paths.input;
output_dir = paths.navigation_results;

param = Param();
cfg = config_factor_graph_simulation(input_dir);
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

rank = 15;
opts.node_interval_sec = 1;
opts.use_imu_sample_node = false;
opts.max_time_sec = inf;
opts.random_seed = 1;

% 高度：1 s 两节点小窗口；测距：默认 720 s，可同时容纳至少两到三次 360 s 测距。
opts.height_window_nodes = 2;
opts.range_lag_sec = 720;
opts.range_window_nodes = [];
opts.optimize_height_every_node = true;
opts.range_include_height_factors = true;
opts.range_factor_scope = 'lag_all';
opts.range_factor_mode = 'horizontal';       % '3d' | 'horizontal' | 'horizontal_from_slant'

% 迭代与数值求解。
opts.max_iterations_height = 1;
opts.max_iterations_range = 6;
opts.max_lm_trials = 5;
opts.lm_lambda_initial = 1e-4;
opts.lm_lambda_min = 1e-10;
opts.lm_lambda_max = 1e8;
opts.lm_decrease = 0.25;
opts.lm_increase = 10;
opts.relative_cost_tolerance = 1e-5;
opts.scaled_step_tolerance = 1e-4;
opts.max_horizontal_step_m = 300;
opts.max_vertical_step_m = 5;
opts.max_attitude_step_rad = 2 * pi / 180;
opts.print_every_node = 180;

% 量测噪声。
opts.range_std = 6;
opts.height_std = 0.4;
opts.range_robust_kernel = 'huber';
opts.range_huber_delta = 2.5;               % 标准化残差阈值

% IMU 数据格式与零偏处理。
opts.imu_mode = 'increment';                 % 'increment': [t,dtheta,dvel]；'rate': [t,gyro,acc]
opts.gyro_cols = 2:4;
opts.acc_cols = 5:7;
opts.insmech_uses_bias_fields = false;       % 若 InsMech 已自行扣除零偏，改为 true
opts.manual_bias_correction = ~opts.insmech_uses_bias_fields;

% 连续时间噪声密度。优先尝试读取 Param 中同名/近似字段，否则使用保守默认值。
opts.gyro_noise_density = readParamScalar(param, ...
    {'gyro_noise_density','gyr_noise_density','gyr_arw','gyro_arw'}, 1.0e-5);       % rad/sqrt(s)
opts.acc_noise_density = readParamScalar(param, ...
    {'acc_noise_density','acc_vrw','acc_arw'}, 1.0e-3);                             % m/s/sqrt(s)
opts.gyrbias_rw_std = readParamScalar(param, ...
    {'gyrbias_rw_std','gyro_bias_rw','gyr_bias_rw'}, 5.0e-10);                      % rad/s/sqrt(s)
opts.accbias_rw_std = readParamScalar(param, ...
    {'accbias_rw_std','acc_bias_rw'}, 5.0e-7);                                     % m/s^2/sqrt(s)

% 数值线性化。FEJ：每次长窗口优化前重建一次，迭代过程中固定雅可比和协方差。
opts.refresh_preint_before_range = true;
opts.refresh_preint_for_marginalization = true;
opts.numeric_state_jacobian = true;
opts.numeric_noise_jacobian = true;
opts.state_jacobian_eps = [ ...
    0.02; 0.02; 0.02; ...                  % dN,dE,dD [m]
    2e-4; 2e-4; 2e-4; ...                  % dv [m/s]
    2e-6; 2e-6; 2e-6; ...                  % datt [rad]
    2e-8; 2e-8; 2e-8; ...                  % dbg [rad/s]
    2e-6; 2e-6; 2e-6];                     % dba [m/s^2]
opts.gyro_noise_jacobian_eps = 2e-7;        % 恒定角速度扰动 [rad/s]
opts.acc_noise_jacobian_eps = 2e-5;         % 恒定加速度扰动 [m/s^2]

% 协方差下限，防止数值奇异；单位对应米制误差状态。
opts.preint_cov_floor_std = [ ...
    0.005; 0.005; 0.005; ...
    5e-4; 5e-4; 5e-4; ...
    2e-6; 2e-6; 2e-6; ...
    1e-10; 1e-10; 1e-10; ...
    1e-8; 1e-8; 1e-8];
opts.covariance_eigen_floor = 1e-18;
opts.information_jitter = 1e-12;

% 米制状态缩放，用于求解器预条件。
opts.state_scale = [ ...
    10; 10; 2; ...
    0.5; 0.5; 0.2; ...
    2e-3; 2e-3; 2e-3; ...
    1e-6; 1e-6; 1e-6; ...
    1e-4; 1e-4; 1e-4];

% 测距长窗口释放完整 15 维状态；高度小窗口只反馈 D 和 vD。
opts.range_feedback_mask = true(rank, 1);
opts.height_feedback_mask = false(rank, 1);
opts.height_feedback_mask([3, 6]) = true;

% 初始先验标准差（米制误差状态）。
opts.initial_prior_std = [ ...
    5; 5; 1; ...
    0.05; 0.05; 0.05; ...
    1e-3; 1e-3; 1e-3; ...
    1e-7; 1e-7; 1e-7; ...
    1e-4; 1e-4; 1e-4];

% 高度两节点小窗口首节点锚定，仅第 3、6 维实际参与求解。
opts.height_anchor_std = 1e6 * ones(rank, 1);
opts.height_anchor_std(3) = 0.02;
opts.height_anchor_std(6) = 0.02;

%% 2. 读取 IMU、高度与距离量测
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

if isempty(opts.range_window_nodes)
    opts.range_window_nodes = round(opts.range_lag_sec / opts.node_interval_sec) + 1;
end

fprintf('\n========== FULL-STATE FIXED-LAG FGO RANGE/INS/HEIGHT ==========\n');
fprintf('处理时间: %.2f -> %.2f s\n', cfg.starttime, cfg.endtime);
fprintf('节点数: %d，高度因子: %d，距离因子: %d\n', num_nodes, numel(height_factors), numel(range_factors));
fprintf('测距窗口: %d 节点，约 %.1f s；窗口内使用全部测距因子。\n', ...
    opts.range_window_nodes, (opts.range_window_nodes - 1) * opts.node_interval_sec);
fprintf('IMU 噪声密度: gyro %.3g rad/sqrt(s), acc %.3g m/s/sqrt(s)\n', ...
    opts.gyro_noise_density, opts.acc_noise_density);
fprintf('InsMech 内部零偏补偿: %d；脚本手动零偏补偿: %d\n', ...
    opts.insmech_uses_bias_fields, opts.manual_bias_correction);

%% 3. 在线传播、局部高度优化和测距固定滞后优化
[~, navstate] = myInitialize_15state(cfg);
navstate = ensureBiasFields(navstate);
nav_nodes = cell(num_nodes, 1);
nav_nodes{1} = navstate;
nav_nodes(1) = applyHeightToNodes(nav_nodes(1), height_factors(1));
navstate = nav_nodes{1};

% 每条关键帧边保存一次 FEJ 线性化与协方差。
edge_cache = cell(max(num_nodes - 1, 1), 1);

% 真正的边缘先验：初始时由系统初始化先验给出，之后由 Schur 补推进。
marginal_prior = makeGaussianPrior(1, nav_nodes{1}, opts.initial_prior_std);

history = [];
range_ptr = 1;

for k = 2:num_nodes
    % 3.1 从上一个优化后的节点继续机械编排。
    navstate = nav_nodes{k - 1};
    navstate = propagateNav(navstate, node_indices(k - 1), node_indices(k), imudata, opts);
    nav_nodes{k} = navstate;

    % 为新边建立完整 15 维 FEJ 因子。
    edge_cache{k - 1} = linearizeImuEdge(nav_nodes{k - 1}, nav_nodes{k}, ...
        node_indices(k - 1), node_indices(k), imudata, opts, param);

    % 3.2 高频高度更新：两节点、垂向自由度，避免每秒建立长图。
    if opts.optimize_height_every_node
        height_first = max(1, k - opts.height_window_nodes + 1);
        local_height = selectHeightFactors(height_factors, height_first, k);
        local_range = range_factors([]);
        height_prior = makeGaussianPrior(height_first, nav_nodes{height_first}, opts.height_anchor_std);

        [nav_nodes, local_history] = optimizeGraphWindow(nav_nodes, height_first, k, ...
            node_indices, imudata, local_height, local_range, height_prior, edge_cache, ...
            opts.height_feedback_mask, opts.max_iterations_height, opts, rank, param);
        navstate = nav_nodes{k};

        if ~isempty(local_history)
            history = [history; [k * ones(size(local_history, 1), 1), ...
                zeros(size(local_history, 1), 1), local_history]]; %#ok<AGROW>
        end
    end

    % 3.3 一个节点可能对应一个或多个距离量测。
    has_range = range_ptr <= numel(range_nodes) && k == range_nodes(range_ptr);
    if has_range
        range_first = max(1, k - opts.range_window_nodes + 1);

        % 将已经移出固定滞后窗口的历史节点顺序边缘化，保留其全部历史信息。
        if marginal_prior.node < range_first
            [marginal_prior, edge_cache] = advanceMarginalPrior(marginal_prior, range_first, ...
                nav_nodes, node_indices, imudata, height_factors, range_factors, ...
                edge_cache, opts, rank, param);
        elseif marginal_prior.node > range_first
            error('边缘先验节点 %d 晚于窗口首节点 %d。', marginal_prior.node, range_first);
        end

        % 长窗口优化前按当前轨迹重建一次 FEJ 雅可比与区间协方差。
        if opts.refresh_preint_before_range
            edge_cache = refreshEdgeCaches(edge_cache, nav_nodes, range_first, k, ...
                node_indices, imudata, opts, param);
        end

        local_range = selectRangeFactors(range_factors, range_first, k);
        if opts.range_include_height_factors
            local_height = selectHeightFactors(height_factors, range_first, k);
        else
            local_height = height_factors([]);
        end

        if numel(local_range) < 2
            fprintf('警告：当前窗口仅有 %d 个测距因子，单距离仍只有一个径向信息方向。\n', numel(local_range));
        end

        [nav_nodes, local_history] = optimizeGraphWindow(nav_nodes, range_first, k, ...
            node_indices, imudata, local_height, local_range, marginal_prior, edge_cache, ...
            opts.range_feedback_mask, opts.max_iterations_range, opts, rank, param);
        navstate = nav_nodes{k};

        % 优化后将边缘先验重定位到新的窗口首节点线性化点，避免下次重复或丢失信息。
        marginal_prior = rebaseCanonicalPrior(marginal_prior, nav_nodes{range_first}, param);

        fprintf('\nrange update %d/%d: t=%.2f s, nodes=%d, range factors=%d, prior node=%d\n', ...
            range_ptr, numel(range_nodes), node_times(k), k - range_first + 1, ...
            numel(local_range), marginal_prior.node);
        printHistory(local_history, max(opts.max_iterations_range, 1));

        if ~isempty(local_history)
            history = [history; [k * ones(size(local_history, 1), 1), ...
                ones(size(local_history, 1), 1), local_history]]; %#ok<AGROW>
        end

        range_ptr = range_ptr + 1;
    end

    if mod(k, opts.print_every_node) == 0
        fprintf('processed node %d/%d, t=%.2f s\n', k, num_nodes, node_times(k));
    end
end

%% 4. 输出
key_path = fullfile(output_dir, 'FGO-RANGE-INS-HEIGHT-full15-keyframes.nav');
smooth_path = fullfile(output_dir, 'FGO-RANGE-INS-HEIGHT-full15-accurate.nav');
% writeNavRows(key_path, nav_nodes, param);
writeFullRateFromNodes(smooth_path, nav_nodes, node_indices, imudata, param);

result.opts = opts;
result.history = history;
result.key_path = key_path;
result.smooth_path = smooth_path;
result.marginal_prior = marginal_prior;
% save(fullfile(output_dir, 'fgo_range_ins_fixedlag_full15_result.mat'), 'result');

fprintf('\nFULL-STATE FIXED-LAG FGO 完成。\n');
fprintf('关键帧结果: %s\n', key_path);
fprintf('100 Hz 结果: %s\n', smooth_path);

%% 局部函数

function [nav_nodes, history] = optimizeGraphWindow(nav_nodes, first_node, last_node, ...
        node_indices, imudata, height_factors, range_factors, prior, edge_cache, ...
        feedback_mask, max_iterations, opts, rank, param)
    local_nodes = nav_nodes(first_node:last_node);
    num_local = numel(local_nodes);
    history = [];
    lambda = opts.lm_lambda_initial;

    for iter = 1:max_iterations
        [H, b, costs] = assembleGraph(local_nodes, first_node, node_indices, imudata, ...
            height_factors, range_factors, prior, edge_cache, opts, rank, param);
        current_cost = costs.total;
        accepted = false;
        best_dx = zeros(rank, num_local);
        best_costs = costs;

        for trial = 1:opts.max_lm_trials
            dx_vec = solveMaskedScaledNormalEquation(H, b, opts.state_scale, ...
                feedback_mask, num_local, rank, lambda);
            dx = reshape(dx_vec, rank, num_local);
            dx = limitStateStep(dx, opts);
            trial_nodes = applyStateCorrection(local_nodes, dx, param);

            [~, ~, trial_costs] = assembleGraph(trial_nodes, first_node, node_indices, imudata, ...
                height_factors, range_factors, prior, edge_cache, opts, rank, param);

            if trial_costs.total <= current_cost || maxScaledStep(dx, opts.state_scale) < opts.scaled_step_tolerance
                accepted = true;
                best_dx = dx;
                best_costs = trial_costs;
                local_nodes = trial_nodes;
                lambda = max(opts.lm_lambda_min, lambda * opts.lm_decrease);
                break;
            end
            lambda = min(opts.lm_lambda_max, lambda * opts.lm_increase);
        end

        if ~accepted
            history(end + 1, :) = [iter, current_cost, costs.prior, costs.imu, ...
                costs.height, costs.range, 0, 0, 0, lambda, 0]; %#ok<AGROW>
            break;
        end

        max_h = maxHorizontalCorrection(best_dx);
        max_z = max(abs(best_dx(3, :)));
        max_att = max(max(abs(best_dx(7:9, :))));
        step_scaled = maxScaledStep(best_dx, opts.state_scale);
        relative_drop = (current_cost - best_costs.total) / max(1, abs(current_cost));

        history(end + 1, :) = [iter, best_costs.total, best_costs.prior, best_costs.imu, ...
            best_costs.height, best_costs.range, max_h, max_z, max_att, lambda, 1]; %#ok<AGROW>

        if step_scaled < opts.scaled_step_tolerance || relative_drop < opts.relative_cost_tolerance
            break;
        end
    end

    nav_nodes(first_node:last_node) = local_nodes;
end

function [H, b, costs] = assembleGraph(local_nodes, first_node, node_indices, imudata, ...
        height_factors, range_factors, prior, edge_cache, opts, rank, param)
    num_nodes = numel(local_nodes);
    num_var = rank * num_nodes;
    H = sparse(num_var, num_var);
    b = zeros(num_var, 1);

    costs.prior = 0;
    costs.imu = 0;
    costs.height = 0;
    costs.range = 0;

    % 边缘/初始化先验，采用 canonical 信息形式。
    if prior.node ~= first_node
        error('先验节点 %d 与窗口首节点 %d 不一致。', prior.node, first_node);
    end
    idx = nodeBlock(1, rank);
    e = navError15Metric(local_nodes{1}, prior.anchor, param);
    prior_rhs = prior.b + prior.H * e;
    H(idx, idx) = H(idx, idx) + prior.H;
    b(idx) = b(idx) + prior_rhs;
    costs.prior = e' * prior.H * e - 2 * prior.b' * e;

    % 完整 15 维 IMU 区间因子。
    for kk = 1:(num_nodes - 1)
        global_edge = first_node + kk - 1;
        cache = edge_cache{global_edge};
        if isempty(cache)
            error('IMU edge cache %d 为空。', global_edge);
        end
        nav_i = local_nodes{kk};
        nav_j = local_nodes{kk + 1};
        r = imuEdgeResidual(nav_i, nav_j, node_indices(global_edge), ...
            node_indices(global_edge + 1), imudata, opts, param);
        Ji = cache.Ji;
        Jj = cache.Jj;
        W = informationFromCovariance(cache.Q, opts);

        idx_i = nodeBlock(kk, rank);
        idx_j = nodeBlock(kk + 1, rank);
        H(idx_i, idx_i) = H(idx_i, idx_i) + Ji' * W * Ji;
        H(idx_i, idx_j) = H(idx_i, idx_j) + Ji' * W * Jj;
        H(idx_j, idx_i) = H(idx_j, idx_i) + Jj' * W * Ji;
        H(idx_j, idx_j) = H(idx_j, idx_j) + Jj' * W * Jj;
        b(idx_i) = b(idx_i) + Ji' * W * r;
        b(idx_j) = b(idx_j) + Jj' * W * r;
        costs.imu = costs.imu + r' * W * r;
    end

    % 高度因子。
    for m = 1:numel(height_factors)
        local_node = height_factors(m).node - first_node + 1;
        if local_node < 1 || local_node > num_nodes
            continue;
        end
        idx = nodeBlock(local_node, rank);
        [r, J, sigma] = heightFactorResidualJacobian(local_nodes{local_node}, height_factors(m), rank);
        W = 1 / sigma^2;
        H(idx, idx) = H(idx, idx) + W * (J' * J);
        b(idx) = b(idx) + W * J' * r;
        costs.height = costs.height + W * r^2;
    end

    % 窗口内全部距离因子，带 Huber 鲁棒核。
    for m = 1:numel(range_factors)
        local_node = range_factors(m).node - first_node + 1;
        if local_node < 1 || local_node > num_nodes
            continue;
        end
        idx = nodeBlock(local_node, rank);
        [r, J, sigma] = rangeFactorResidualJacobian(local_nodes{local_node}, ...
            range_factors(m), opts, rank, param);
        [robust_weight, robust_cost] = robustScalarWeightCost(r, sigma, ...
            opts.range_robust_kernel, opts.range_huber_delta);
        W = robust_weight / sigma^2;
        H(idx, idx) = H(idx, idx) + W * (J' * J);
        b(idx) = b(idx) + W * J' * r;
        costs.range = costs.range + robust_cost;
    end

    H = (H + H') / 2;
    costs.total = costs.prior + costs.imu + costs.height + costs.range;
end

function cache = linearizeImuEdge(nav_i, nav_j, idx_i, idx_j, imudata, opts, param)
    nav_i = ensureBiasFields(nav_i);
    nav_j = ensureBiasFields(nav_j);
    pred0 = propagateNav(nav_i, idx_i, idx_j, imudata, opts);
    r0 = navError15Metric(nav_j, pred0, param);

    Ji = zeros(15, 15);
    if opts.numeric_state_jacobian
        for d = 1:15
            eps_d = opts.state_jacobian_eps(d);
            step = zeros(15, 1);
            step(d) = eps_d;
            nav_i_plus = perturbNavStateMetric(nav_i, step, param);
            pred_plus = propagateNav(nav_i_plus, idx_i, idx_j, imudata, opts);
            r_plus = navError15Metric(nav_j, pred_plus, param);
            Ji(:, d) = (r_plus - r0) / eps_d;
        end
    else
        dt = max(imudata(idx_j, 1) - imudata(idx_i, 1), eps);
        Phi = eye(15);
        Phi(1:3, 4:6) = eye(3) * dt;
        Ji = -Phi;
    end
    Jj = eye(15);

    dt = max(imudata(idx_j, 1) - imudata(idx_i, 1), eps);
    G = zeros(15, 6);
    if opts.numeric_noise_jacobian
        for d = 1:6
            if d <= 3
                eps_n = opts.gyro_noise_jacobian_eps;
            else
                eps_n = opts.acc_noise_jacobian_eps;
            end
            noise_rate = zeros(6, 1);
            noise_rate(d) = eps_n;
            pred_plus = propagateNavWithImuOffset(nav_i, idx_i, idx_j, imudata, opts, noise_rate);
            endpoint_error = navError15Metric(pred_plus, pred0, param);
            G(:, d) = endpoint_error / eps_n;
        end
    else
        G(7:9, 1:3) = eye(3) * dt;
        G(4:6, 4:6) = eye(3) * dt;
        G(1:3, 4:6) = 0.5 * eye(3) * dt^2;
    end

    % 恒定区间噪声的方差取 density^2/dt，使积分后方差与白噪声 density^2*dt 一致。
    q_const = [ ...
        (opts.gyro_noise_density^2 / dt) * ones(3, 1); ...
        (opts.acc_noise_density^2 / dt) * ones(3, 1)];
    Q = G * diag(q_const) * G';
    Q(10:12, 10:12) = Q(10:12, 10:12) + ...
        eye(3) * opts.gyrbias_rw_std^2 * dt;
    Q(13:15, 13:15) = Q(13:15, 13:15) + ...
        eye(3) * opts.accbias_rw_std^2 * dt;
    Q = Q + diag(opts.preint_cov_floor_std(:).^2);
    Q = regularizeCovariance(Q, opts.covariance_eigen_floor);

    cache.Ji = Ji;
    cache.Jj = Jj;
    cache.Q = Q;
    cache.dt = dt;
    cache.anchor_i = nav_i;
    cache.bias_coupling_norm = norm(Ji(:, 10:15), 'fro');
end

function r = imuEdgeResidual(nav_i, nav_j, idx_i, idx_j, imudata, opts, param)
    pred = propagateNav(nav_i, idx_i, idx_j, imudata, opts);
    r = navError15Metric(nav_j, pred, param);
end

function edge_cache = refreshEdgeCaches(edge_cache, nav_nodes, first_node, last_node, ...
        node_indices, imudata, opts, param)
    for g = first_node:(last_node - 1)
        edge_cache{g} = linearizeImuEdge(nav_nodes{g}, nav_nodes{g + 1}, ...
            node_indices(g), node_indices(g + 1), imudata, opts, param);
    end
end

function [prior, edge_cache] = advanceMarginalPrior(prior, target_node, nav_nodes, ...
        node_indices, imudata, height_factors, range_factors, edge_cache, opts, rank, param)
    if target_node < prior.node
        error('不能将边缘先验从节点 %d 反向推进到节点 %d。', prior.node, target_node);
    end

    for g = prior.node:(target_node - 1)
        if opts.refresh_preint_for_marginalization || isempty(edge_cache{g})
            edge_cache{g} = linearizeImuEdge(nav_nodes{g}, nav_nodes{g + 1}, ...
                node_indices(g), node_indices(g + 1), imudata, opts, param);
        end
        cache = edge_cache{g};

        Hpair = zeros(2 * rank, 2 * rank);
        bpair = zeros(2 * rank, 1);
        idx_i = 1:rank;
        idx_j = rank + (1:rank);

        % 旧 canonical 先验重线性化到当前第 g 节点。
        e = navError15Metric(nav_nodes{g}, prior.anchor, param);
        Hpair(idx_i, idx_i) = Hpair(idx_i, idx_i) + prior.H;
        bpair(idx_i) = bpair(idx_i) + prior.b + prior.H * e;

        % g -> g+1 的完整 IMU 因子。
        r = imuEdgeResidual(nav_nodes{g}, nav_nodes{g + 1}, ...
            node_indices(g), node_indices(g + 1), imudata, opts, param);
        W = informationFromCovariance(cache.Q, opts);
        Ji = cache.Ji;
        Jj = cache.Jj;
        Hpair(idx_i, idx_i) = Hpair(idx_i, idx_i) + Ji' * W * Ji;
        Hpair(idx_i, idx_j) = Hpair(idx_i, idx_j) + Ji' * W * Jj;
        Hpair(idx_j, idx_i) = Hpair(idx_j, idx_i) + Jj' * W * Ji;
        Hpair(idx_j, idx_j) = Hpair(idx_j, idx_j) + Jj' * W * Jj;
        bpair(idx_i) = bpair(idx_i) + Ji' * W * r;
        bpair(idx_j) = bpair(idx_j) + Jj' * W * r;

        % 只吸收被消元节点 g 上的一元高度和距离因子；保留节点上的因子留给后续窗口。
        h_idx = find([height_factors.node] == g);
        for m = h_idx
            [rh, Jh, sigmah] = heightFactorResidualJacobian(nav_nodes{g}, height_factors(m), rank);
            Wh = 1 / sigmah^2;
            Hpair(idx_i, idx_i) = Hpair(idx_i, idx_i) + Wh * (Jh' * Jh);
            bpair(idx_i) = bpair(idx_i) + Wh * Jh' * rh;
        end

        r_idx = find([range_factors.node] == g);
        for m = r_idx
            [rr, Jr, sigmar] = rangeFactorResidualJacobian(nav_nodes{g}, range_factors(m), opts, rank, param);
            [wr, ~] = robustScalarWeightCost(rr, sigmar, opts.range_robust_kernel, opts.range_huber_delta);
            Wr = wr / sigmar^2;
            Hpair(idx_i, idx_i) = Hpair(idx_i, idx_i) + Wr * (Jr' * Jr);
            bpair(idx_i) = bpair(idx_i) + Wr * Jr' * rr;
        end

        Hpair = (Hpair + Hpair') / 2;
        Hii = Hpair(idx_i, idx_i);
        Hij = Hpair(idx_i, idx_j);
        Hji = Hpair(idx_j, idx_i);
        Hjj = Hpair(idx_j, idx_j);
        bi = bpair(idx_i);
        bj = bpair(idx_j);

        Hii = Hii + eye(rank) * opts.information_jitter * max(1, max(abs(diag(Hii))));
        solved = Hii \ [Hij, bi];
        Hm = Hjj - Hji * solved(:, 1:rank);
        bm = bj - Hji * solved(:, end);
        Hm = regularizeInformation(Hm, opts.information_jitter);

        prior.node = g + 1;
        prior.anchor = nav_nodes{g + 1};
        prior.H = Hm;
        prior.b = bm;
    end
end

function prior = makeGaussianPrior(node, anchor, std_vec)
    prior.node = node;
    prior.anchor = anchor;
    prior.H = diag(1 ./ max(std_vec(:), eps).^2);
    prior.b = zeros(numel(std_vec), 1);
end

function prior = rebaseCanonicalPrior(prior, new_anchor, param)
    e = navError15Metric(new_anchor, prior.anchor, param);
    prior.b = prior.b + prior.H * e;
    prior.anchor = new_anchor;
end

function [r, J, sigma] = heightFactorResidualJacobian(nav, factor, rank)
    r = nav.pos(3) - factor.z;
    J = zeros(1, rank);
    J(3) = -1;                           % dD>0 表示向下，h 减小
    sigma = factor.sigma;
end

function [r, J, sigma] = rangeFactorResidualJacobian(nav, factor, opts, rank, param)
    beacon = factor.beacon(:);
    delta_ned = geodeticDeltaNed(nav.pos, beacon, param);
    J = zeros(1, rank);

    switch lower(opts.range_factor_mode)
        case '3d'
            predicted = max(norm(delta_ned), 1e-8);
            measured = factor.range;
            J(1, 1:3) = delta_ned' / predicted;
        case 'horizontal'
            predicted = max(norm(delta_ned(1:2)), 1e-8);
            measured = factor.range;
            J(1, 1:2) = delta_ned(1:2)' / predicted;
        case 'horizontal_from_slant'
            predicted = max(norm(delta_ned(1:2)), 1e-8);
            horizontal_measured = sqrt(max(factor.range^2 - delta_ned(3)^2, 1e-12));
            measured = horizontal_measured;
            J(1, 1:2) = delta_ned(1:2)' / predicted;
            J(1, 3) = delta_ned(3) / horizontal_measured;
        otherwise
            error('未知距离因子模式: %s', opts.range_factor_mode);
    end

    r = predicted - measured;
    sigma = factor.sigma;
end

function [weight, cost] = robustScalarWeightCost(r, sigma, kernel, delta)
    u = abs(r) / max(sigma, eps);
    switch lower(kernel)
        case 'none'
            weight = 1;
            cost = u^2;
        case 'huber'
            if u <= delta
                weight = 1;
                cost = u^2;
            else
                weight = delta / max(u, eps);
                cost = 2 * delta * u - delta^2;
            end
        otherwise
            error('未知鲁棒核: %s', kernel);
    end
end

function nav = propagateNav(nav, idx_i, idx_j, imudata, opts)
    nav = ensureBiasFields(nav);
    for imuindex = (idx_i + 1):idx_j
        lastimu = imudata(imuindex - 1, :)';
        thisimu = imudata(imuindex, :)';
        if opts.manual_bias_correction
            [lastimu, thisimu] = biasCorrectImuPair(lastimu, thisimu, nav, opts);
        end
        nav = InsMech(nav, lastimu, thisimu);
        nav = ensureBiasFields(nav);
    end
end

function nav = propagateNavWithImuOffset(nav, idx_i, idx_j, imudata, opts, noise_rate)
    nav = ensureBiasFields(nav);
    for imuindex = (idx_i + 1):idx_j
        lastimu = imudata(imuindex - 1, :)';
        thisimu = imudata(imuindex, :)';
        dt_last = localSampleDt(imudata, imuindex - 1);
        dt_this = localSampleDt(imudata, imuindex);
        lastimu = addImuRateOffset(lastimu, dt_last, noise_rate, opts);
        thisimu = addImuRateOffset(thisimu, dt_this, noise_rate, opts);
        if opts.manual_bias_correction
            [lastimu, thisimu] = biasCorrectImuPair(lastimu, thisimu, nav, opts);
        end
        nav = InsMech(nav, lastimu, thisimu);
        nav = ensureBiasFields(nav);
    end
end

function row = addImuRateOffset(row, dt, noise_rate, opts)
    switch lower(opts.imu_mode)
        case 'increment'
            row(opts.gyro_cols) = row(opts.gyro_cols) + noise_rate(1:3) * dt;
            row(opts.acc_cols) = row(opts.acc_cols) + noise_rate(4:6) * dt;
        case 'rate'
            row(opts.gyro_cols) = row(opts.gyro_cols) + noise_rate(1:3);
            row(opts.acc_cols) = row(opts.acc_cols) + noise_rate(4:6);
        otherwise
            error('未知 IMU 模式: %s', opts.imu_mode);
    end
end

function dt = localSampleDt(imudata, idx)
    if idx <= 1
        dt = imudata(2, 1) - imudata(1, 1);
    else
        dt = imudata(idx, 1) - imudata(idx - 1, 1);
    end
    dt = max(dt, eps);
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
            % 两子样机械编排同时使用 lastimu/thisimu，因此两行都应按各自区间扣除零偏。
            lastimu_corr(opts.gyro_cols) = lastimu_corr(opts.gyro_cols) - nav.gyrbias(:) * dt;
            lastimu_corr(opts.acc_cols) = lastimu_corr(opts.acc_cols) - nav.accbias(:) * dt;
            thisimu_corr(opts.gyro_cols) = thisimu_corr(opts.gyro_cols) - nav.gyrbias(:) * dt;
            thisimu_corr(opts.acc_cols) = thisimu_corr(opts.acc_cols) - nav.accbias(:) * dt;
        case 'rate'
            lastimu_corr(opts.gyro_cols) = lastimu_corr(opts.gyro_cols) - nav.gyrbias(:);
            lastimu_corr(opts.acc_cols) = lastimu_corr(opts.acc_cols) - nav.accbias(:);
            thisimu_corr(opts.gyro_cols) = thisimu_corr(opts.gyro_cols) - nav.gyrbias(:);
            thisimu_corr(opts.acc_cols) = thisimu_corr(opts.acc_cols) - nav.accbias(:);
        otherwise
            error('未知 IMU 模式: %s', opts.imu_mode);
    end
end

function r = navError15Metric(nav_current, nav_ref, param)
    nav_current = ensureBiasFields(nav_current);
    nav_ref = ensureBiasFields(nav_ref);
    r = zeros(15, 1);
    r(1:3) = geodeticDeltaNed(nav_current.pos, nav_ref.pos, param);
    r(4:6) = nav_current.vel - nav_ref.vel;
    r(7:9) = wrapAngle(nav_current.att - nav_ref.att);
    r(10:12) = nav_current.gyrbias - nav_ref.gyrbias;
    r(13:15) = nav_current.accbias - nav_ref.accbias;
end

function delta_ned = geodeticDeltaNed(pos, ref_pos, param)
    [rm, rn] = getRmRn(ref_pos(1), param);
    dlat = pos(1) - ref_pos(1);
    dlon = wrapAngle(pos(2) - ref_pos(2));
    delta_ned = [ ...
        (rm + ref_pos(3)) * dlat; ...
        (rn + ref_pos(3)) * cos(ref_pos(1)) * dlon; ...
        -(pos(3) - ref_pos(3))];
end

function nav = perturbNavStateMetric(nav, dx, param)
    nav = ensureBiasFields(nav);
    [rm, rn] = getRmRn(nav.pos(1), param);
    cos_lat = max(abs(cos(nav.pos(1))), 1e-8) * signNonzero(cos(nav.pos(1)));
    nav.pos(1) = nav.pos(1) + dx(1) / (rm + nav.pos(3));
    nav.pos(2) = nav.pos(2) + dx(2) / ((rn + nav.pos(3)) * cos_lat);
    nav.pos(3) = nav.pos(3) - dx(3);
    nav.vel = nav.vel + dx(4:6);
    nav.att = wrapAngle(nav.att + dx(7:9));
    nav.gyrbias = nav.gyrbias + dx(10:12);
    nav.accbias = nav.accbias + dx(13:15);
    [nav.Rm, nav.Rn] = getRmRn(nav.pos(1), param);
    nav.gravity = getGravity(nav.pos);
end

function nav = retractNavStateMetric(nav, dx, param)
    nav = perturbNavStateMetric(nav, -dx, param);
end

function s = signNonzero(x)
    if x >= 0
        s = 1;
    else
        s = -1;
    end
end

function nodes = applyStateCorrection(nodes, dx, param)
    for k = 1:numel(nodes)
        nodes{k} = retractNavStateMetric(nodes{k}, dx(:, k), param);
    end
end

function dx = limitStateStep(dx, opts)
    scale = 1;
    max_h = maxHorizontalCorrection(dx);
    max_z = max(abs(dx(3, :)));
    max_att = max(max(abs(dx(7:9, :))));
    if max_h > opts.max_horizontal_step_m
        scale = min(scale, opts.max_horizontal_step_m / max_h);
    end
    if max_z > opts.max_vertical_step_m
        scale = min(scale, opts.max_vertical_step_m / max_z);
    end
    if max_att > opts.max_attitude_step_rad
        scale = min(scale, opts.max_attitude_step_rad / max_att);
    end
    if scale < 1
        dx = dx * scale;
    end
end

function value = maxHorizontalCorrection(dx)
    if isempty(dx)
        value = 0;
    else
        value = max(sqrt(sum(dx(1:2, :).^2, 1)));
    end
end

function value = maxScaledStep(dx, state_scale)
    if isempty(dx)
        value = 0;
    else
        value = max(max(abs(dx ./ repmat(state_scale(:), 1, size(dx, 2)))));
    end
end

function dx_vec = solveMaskedScaledNormalEquation(H, b, state_scale, feedback_mask, ...
        num_nodes, rank, lambda)
    free_idx = false(rank * num_nodes, 1);
    for node = 1:num_nodes
        idx = nodeBlock(node, rank);
        free_idx(idx(feedback_mask(:))) = true;
    end

    dx_vec = zeros(rank * num_nodes, 1);
    if ~any(free_idx)
        return;
    end

    scale_all = repmat(state_scale(:), num_nodes, 1);
    scale_free = scale_all(free_idx);
    S = spdiags(scale_free, 0, numel(scale_free), numel(scale_free));
    Hf = (H(free_idx, free_idx) + H(free_idx, free_idx)') / 2;
    bf = b(free_idx);
    Hs = S * Hf * S;
    bs = S * bf;
    diag_scale = max(abs(diag(Hs)), 1);
    damping = spdiags(lambda * diag_scale, 0, numel(diag_scale), numel(diag_scale));
    y = (Hs + damping) \ bs;
    dx_vec(free_idx) = scale_free .* y;
end

function W = informationFromCovariance(Q, opts)
    Q = regularizeCovariance(Q, opts.covariance_eigen_floor);
    W = Q \ eye(size(Q));
    W = (W + W') / 2;
end

function Q = regularizeCovariance(Q, floor_value)
    Q = (Q + Q') / 2;
    [V, D] = eig(full(Q));
    d = real(diag(D));
    d(~isfinite(d)) = floor_value;
    d = max(d, floor_value);
    Q = V * diag(d) * V';
    Q = real((Q + Q') / 2);
end

function H = regularizeInformation(H, jitter)
    H = real((H + H') / 2);
    scale = max(1, max(abs(diag(H))));
    H = H + eye(size(H)) * jitter * scale;
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
    if isempty(factors)
        local = factors;
        return;
    end
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

function idx = nodeBlock(node, rank)
    idx = (node - 1) * rank + (1:rank);
end

function printHistory(local_history, max_iterations)
    for ii = 1:size(local_history, 1)
        fprintf(['  iter %d/%d: total %.4g, prior %.3g, imu %.3g, height %.3g, range %.3g, ' ...
            'maxH %.3f m, maxD %.3f m, maxAtt %.4g rad, lambda %.3g, accepted %d\n'], ...
            local_history(ii, 1), max_iterations, local_history(ii, 2), local_history(ii, 3), ...
            local_history(ii, 4), local_history(ii, 5), local_history(ii, 6), ...
            local_history(ii, 7), local_history(ii, 8), local_history(ii, 9), ...
            local_history(ii, 10), local_history(ii, 11));
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

function value = readParamScalar(param, names, default_value)
    value = default_value;
    for k = 1:numel(names)
        name = names{k};
        try
            if isstruct(param) && isfield(param, name)
                candidate = param.(name);
            elseif isobject(param) && isprop(param, name)
                candidate = param.(name);
            else
                continue;
            end
            if isnumeric(candidate) && isscalar(candidate) && isfinite(candidate) && candidate > 0
                value = double(candidate);
                return;
            end
        catch
            % 保留默认值。
        end
    end
end
