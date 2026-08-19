%% FGO RANGE/INS 组合导航
% 图节点：1 s 一个节点。
% 高度量测：每个节点汇总该秒内的 100 Hz 高度量测，形成高度因子。
% 距离量测：约 360 s 一个水平距离因子。
% 迭代次数：2 次。

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

opts.node_interval_sec = 1;
opts.max_iterations = 1;
opts.max_time_sec = inf;              % 调试时可改成 1800
opts.range_std = 6;
opts.height_std = 0.4;
opts.random_seed = 1;
opts.max_position_step_m = 100;       % 稀疏距离因子可观性弱，限制单次 GN 步长防止发散
opts.process_std_floor = [ ...
    1e-11; 1e-11; 1e-4; ...
    1e-5;  1e-5;  1e-5; ...
    1e-8;  1e-8;  1e-8; ...
    1e-11; 1e-11; 1e-11; ...
    1e-8;  1e-8;  1e-8];
opts.state_scale = [ ...
    1e-6; 1e-6; 1; ...
    0.1; 0.1; 0.1; ...
    1e-3; 1e-3; 1e-3; ...
    1e-6; 1e-6; 1e-6; ...
    1e-4; 1e-4; 1e-4];

%% 2. 读取 IMU、高度和距离量测
rng(opts.random_seed);
imudata = importdata(cfg.imufilepath);
truth = importdata(cfg.truthpath);
heightdata = truth(:, [2, 5]);
range_streams = { ...
    importdata(cfg.rangefile1path), ...
    importdata(cfg.rangefile2path), ...
    importdata(cfg.rangefile3path)};
rangedata = buildRange360(range_streams);

% 距离量测是 360 s 稀疏约束，不能拿第一条距离时刻作为初始化时刻；
% 否则 navstate 的初始 PVA 仍来自统一配置的 0.01 s，时间却跳到 359 s。
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
node_times = (cfg.starttime:opts.node_interval_sec:cfg.endtime)';
node_indices = round(interp1(imu_times, (1:length(imu_times))', node_times, 'nearest', 'extrap'));
[node_indices, keep] = unique(node_indices, 'stable');
node_times = imu_times(node_indices);
num_nodes = length(node_indices);
rank = 15;

range_indices = round(interp1(imu_times, (1:length(imu_times))', rangedata(:, 1), 'nearest', 'extrap'));
[found, range_nodes] = ismember(range_indices, node_indices);
rangedata = rangedata(found, :);
range_nodes = range_nodes(found);

height_factors = buildHeightFactors(node_times, heightdata, opts.height_std);

fprintf('\n========== FGO RANGE/INS Processing ==========\n');
fprintf('处理时间: %.2f -> %.2f s\n', cfg.starttime, cfg.endtime);
fprintf('图节点数: %d, 高度因子: %d, 距离因子: %d\n', ...
    num_nodes, numel(height_factors), size(rangedata, 1));

%% 3. 用 KF RANGE/INS 生成初始轨迹
[kf_template, navstate] = myInitialize_15state(cfg);
kf = kf_template;
kf.rangstd = opts.range_std;
kf.depthstd = opts.height_std;

nav_nodes = cell(num_nodes, 1);
nav_nodes{1} = navstate;
lastimu = imudata(1, :)';
thisimu = imudata(1, :)';
next_node = 2;
rangeindex = 1;

for imuindex = 2:node_indices(end)
    lastimu = thisimu;
    thisimu = imudata(imuindex, :)';
    dt = thisimu(1) - lastimu(1);

    navstate = InsMech(navstate, lastimu, thisimu);
    kf = heightOnlyUpdate(navstate, heightdata(imuindex, :), kf);
    navstate.pos(3) = navstate.pos(3) - kf.x(3);
    navstate.vel(3) = navstate.vel(3) - kf.x(6);
    kf.x(3) = 0;
    kf.x(6) = 0;

    while rangeindex <= size(rangedata, 1) && rangedata(rangeindex, 1) <= thisimu(1) + 5e-4
        kf = rangeOnlyUpdate(navstate, rangedata(rangeindex, :), kf);
        [kf, navstate] = myErrorFeedback_range(kf, navstate);
        rangeindex = rangeindex + 1;
    end

    kf = myInsPropagate_15state(navstate, thisimu, dt, kf);

    while next_node <= num_nodes && imuindex >= node_indices(next_node)
        nav_nodes{next_node} = navstate;
        next_node = next_node + 1;
    end
end

%% 4. 两次 Gauss-Newton 因子图迭代
history = zeros(opts.max_iterations, 4);
for iter = 1:opts.max_iterations
    fprintf('\nFGO iteration %d/%d: 重线性化 IMU/高度/距离因子...\n', iter, opts.max_iterations);

    [Phi_blocks, Q_blocks, process_z] = buildImuFactors(nav_nodes, node_indices, imudata, kf_template, opts, rank);
    [A, b, costs] = assembleGraph(nav_nodes, Phi_blocks, Q_blocks, process_z, ...
        height_factors, rangedata, range_nodes, kf_template.P0, opts, rank);
    dx_vec = solveScaledNormalEquation(A, b, opts.state_scale, num_nodes);
    dx = reshape(dx_vec, rank, num_nodes);

    max_pos_dx = 0;
    for k = 1:num_nodes
        DR = positionDr(nav_nodes{k}.pos, param);
        max_pos_dx = max(max_pos_dx, norm(DR * dx(1:3, k)));
    end
    if max_pos_dx > opts.max_position_step_m
        scale = opts.max_position_step_m / max_pos_dx;
        dx = dx * scale;
        fprintf('  GN 步长 %.3f m 过大，缩放为 %.3f m\n', max_pos_dx, opts.max_position_step_m);
        max_pos_dx = opts.max_position_step_m;
    end

    for k = 1:num_nodes
        nav_nodes{k} = retractNavState(nav_nodes{k}, dx(:, k));
    end

    history(iter, :) = [costs.process, costs.height, costs.range, max_pos_dx];
    fprintf('  process %.3g, height %.3g, range %.3g, max dx %.4f m\n', ...
        costs.process, costs.height, costs.range, max_pos_dx);
end

%% 5. 输出
navpath = fullfile(output_dir, 'FGO-RANGE-INS-keyframes-single.nav');
writeNavRows(navpath, nav_nodes, param);

fprintf('\nFGO RANGE/INS 完成，结果文件: %s\n', navpath);

%% 局部函数
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

function [Phi_blocks, Q_blocks, process_z] = buildImuFactors(nav_nodes, node_indices, imudata, kf_template, opts, rank)
    num_nodes = length(nav_nodes);
    Phi_blocks = cell(num_nodes - 1, 1);
    Q_blocks = cell(num_nodes - 1, 1);
    process_z = zeros(rank, num_nodes - 1);
    for k = 1:(num_nodes - 1)
        nav_pred = nav_nodes{k};
        Phi_acc = eye(rank);
        Q_acc = zeros(rank);
        for imuindex = (node_indices(k) + 1):node_indices(k + 1)
            lastimu = imudata(imuindex - 1, :)';
            thisimu = imudata(imuindex, :)';
            dt = thisimu(1) - lastimu(1);
            nav_pred = InsMech(nav_pred, lastimu, thisimu);
            kf_step = kf_template;
            kf_step.x = zeros(rank, 1);
            kf_step.P = zeros(rank);
            kf_step = myInsPropagate_15state(nav_pred, thisimu, dt, kf_step);
            Phi_acc = kf_step.phi * Phi_acc;
            Q_acc = kf_step.phi * Q_acc * kf_step.phi' + (kf_step.P + kf_step.P') / 2;
        end
        Phi_blocks{k} = Phi_acc;
        Q_blocks{k} = regularizeCov(Q_acc, opts.process_std_floor);
        process_z(:, k) = localError(nav_nodes{k + 1}, nav_pred);
    end
end

function [A, b, costs] = assembleGraph(nav_nodes, Phi_blocks, Q_blocks, process_z, height_factors, rangedata, range_nodes, P0, opts, rank)
    num_nodes = length(nav_nodes);
    num_var = rank * num_nodes;
    A = sparse(num_var, num_var);
    b = zeros(num_var, 1);
    costs.process = 0;
    costs.height = 0;
    costs.range = 0;

    idx0 = nodeBlock(1, rank);
    W0 = symInv(P0);
    A(idx0, idx0) = A(idx0, idx0) + W0;

    for k = 1:(num_nodes - 1)
        idx_i = nodeBlock(k, rank);
        idx_j = nodeBlock(k + 1, rank);
        Phi = Phi_blocks{k};
        W = symInv(Q_blocks{k});
        z = process_z(:, k);
        Ji = -Phi;
        Jj = eye(rank);
        A(idx_i, idx_i) = A(idx_i, idx_i) + Ji' * W * Ji;
        A(idx_i, idx_j) = A(idx_i, idx_j) + Ji' * W * Jj;
        A(idx_j, idx_i) = A(idx_j, idx_i) + Jj' * W * Ji;
        A(idx_j, idx_j) = A(idx_j, idx_j) + Jj' * W * Jj;
        b(idx_i) = b(idx_i) + Ji' * W * z;
        b(idx_j) = b(idx_j) + Jj' * W * z;
        costs.process = costs.process + z' * W * z;
    end

    Hh = zeros(1, rank);
    Hh(3) = 1;
    for m = 1:numel(height_factors)
        f = height_factors(m);
        idx = nodeBlock(f.node, rank);
        z = nav_nodes{f.node}.pos(3) - f.z;
        weight = 1 / (f.sigma ^ 2);
        A(idx, idx) = A(idx, idx) + weight * (Hh' * Hh);
        b(idx) = b(idx) + weight * Hh' * z;
        costs.height = costs.height + weight * z ^ 2;
    end

    for m = 1:size(rangedata, 1)
        node = range_nodes(m);
        idx = nodeBlock(node, rank);
        [z, H] = rangeResidualAndJacobian(nav_nodes{node}, rangedata(m, :), rank);
        weight = 1 / (opts.range_std ^ 2);
        A(idx, idx) = A(idx, idx) + weight * (H' * H);
        b(idx) = b(idx) + weight * H' * z;
        costs.range = costs.range + weight * z ^ 2;
    end
end

function kf = heightOnlyUpdate(navstate, heightrow, kf)
    z = navstate.pos(3) - heightrow(2);
    H = zeros(1, kf.RANK);
    H(3) = 1;
    R = kf.depthstd ^ 2;
    K = kf.P * H' / (H * kf.P * H' + R);
    mask = zeros(kf.RANK, 1);
    mask(3) = 1;
    mask(6) = 1;
    K = K .* mask;
    kf.x = kf.x + K * (z - H * kf.x);
    I = eye(kf.RANK);
    kf.P = (I - K * H) * kf.P * (I - K * H)' + K * R * K';
end

function kf = rangeOnlyUpdate(navstate, rangerow, kf)
    [z, H] = rangeResidualAndJacobian(navstate, rangerow, kf.RANK);
    R = kf.rangstd ^ 2;
    K = kf.P * H' / (H * kf.P * H' + R);
    kf.x = kf.x + K * (z - H * kf.x);
    I = eye(kf.RANK);
    kf.P = (I - K * H) * kf.P * (I - K * H)' + K * R * K';
end

function [z, H] = rangeResidualAndJacobian(navstate, rangerow, rank)
    param = Param();
    beacon = rangerow(4:6)';
    [rm, rn] = getRmRn(beacon(1), param);
    DR = diag([rm + beacon(3), (rn + beacon(3)) * cos(beacon(1)), -1]);
    delta = DR * (navstate.pos - beacon);
    predicted = max(norm(delta(1:2)), 1e-6);
    z = predicted - rangerow(3);
    H = zeros(1, rank);
    derivative = (navstate.pos' - beacon') * (DR ^ 2) / predicted;
    H(1, 1:2) = derivative(1:2);
end

function dx = localError(nav_current, nav_pred)
    dx = zeros(15, 1);
    dx(1:3) = nav_current.pos - nav_pred.pos;
    dx(4:6) = nav_current.vel - nav_pred.vel;
    dx(7:9) = nav_current.att - nav_pred.att;
    dx(10:12) = nav_current.gyrbias - nav_pred.gyrbias;
    dx(13:15) = nav_current.accbias - nav_pred.accbias;
end

function nav = retractNavState(nav, dx)
    nav.pos = nav.pos - dx(1:3);
    nav.vel = nav.vel - dx(4:6);
    qpn = rotvec2quat(dx(7:9));
    nav.qbn = quatProd(qpn, nav.qbn);
    nav.qbn = quatNormalized(nav.qbn);
    nav.cbn = quat2dcm(nav.qbn);
    nav.att = dcm2euler(nav.cbn);
    nav.gyrbias = nav.gyrbias + dx(10:12);
    nav.accbias = nav.accbias + dx(13:15);
    param = Param();
    [nav.Rm, nav.Rn] = getRmRn(nav.pos(1), param);
    nav.gravity = getGravity(nav.pos);
end

function dx_vec = solveScaledNormalEquation(A, b, state_scale, num_nodes)
    scale_vec = repmat(state_scale(:), num_nodes, 1);
    S = spdiags(scale_vec, 0, length(scale_vec), length(scale_vec));
    A = (A + A') / 2;
    y = ((S * A * S) + speye(size(A)) * 1e-12) \ (S * b);
    dx_vec = scale_vec .* y;
end

function DR = positionDr(pos, param)
    [rm, rn] = getRmRn(pos(1), param);
    DR = diag([rm + pos(3), (rn + pos(3)) * cos(pos(1)), -1]);
end

function idx = nodeBlock(node, rank)
    idx = (node - 1) * rank + (1:rank);
end

function Q = regularizeCov(Q, std_floor)
    Q = (Q + Q') / 2 + diag(std_floor(:) .^ 2);
end

function W = symInv(P)
    P = (P + P') / 2;
    jitter = max(1e-20, 1e-12 * max(1, max(abs(diag(P)))));
    for i = 1:12
        [L, flag] = chol(P + eye(size(P)) * jitter, 'lower');
        if flag == 0
            Linv = L \ eye(size(L));
            W = Linv' * Linv;
            return;
        end
        jitter = jitter * 10;
    end
    W = pinv(full(P));
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
