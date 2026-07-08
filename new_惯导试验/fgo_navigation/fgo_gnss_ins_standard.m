%% 标准 GNSS/INS 因子图流程：迭代重线性化 + 批量平滑
% 变量：每个 GNSS 历元一个 15 维误差状态。
% 因子：初始先验因子、IMU 过程因子、GNSS 位置因子。
% 流程：KF 初值 -> 建图 -> Gauss-Newton 求解 -> 误差反馈 -> 重新线性化。

clear; clc; close all;

%% 1. 路径、配置与因子图参数
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

opts.gnss_file = fullfile(output_dir, 'simulated_gnss.txt');
opts.max_time_sec = inf;             % 改成 inf 可跑完整数据
opts.max_iterations = 4;              % 标准 FGO 至少要多轮重线性化
opts.gnss_std_m = [2; 2; 3];          % N/E/U, m
opts.stop_dx_m = 1e-3;                % 节点位置最大改正量收敛阈值
opts.process_std_floor = [ ...
    1e-11; 1e-11; 1e-4; ...
    1e-5; 1e-5; 1e-5; ...
    1e-8; 1e-8; 1e-8; ...
    1e-11; 1e-11; 1e-11; ...
    1e-8; 1e-8; 1e-8];
opts.state_scale = [ ...
    1e-6; 1e-6; 1; ...
    0.1; 0.1; 0.1; ...
    1e-3; 1e-3; 1e-3; ...
    1e-6; 1e-6; 1e-6; ...
    1e-4; 1e-4; 1e-4];

if ~exist(opts.gnss_file, 'file')
    error('找不到 GNSS 文件: %s。请先运行 fgo_gnss_ins_demo.m 或提供 simulated_gnss.txt。', opts.gnss_file);
end

fprintf('\n========== Standard GNSS/INS FGO ==========\n');
fprintf('GNSS 文件: %s\n', opts.gnss_file);

%% 2. 读取数据并截取公共时间段
imudata = importdata(cfg.imufilepath);
gnss = importdata(opts.gnss_file);
truth = importdata(cfg.truthpath);

gnss = gnss(:, 1:4);
gnss(:, 2:3) = gnss(:, 2:3) * param.D2R;

cfg.starttime = max([cfg.starttime, imudata(1, 1), gnss(1, 1), truth(1, 2)]);
cfg.endtime = min([cfg.endtime, imudata(end, 1), gnss(end, 1), truth(end, 2)]);
if isfinite(opts.max_time_sec)
    cfg.endtime = min(cfg.endtime, cfg.starttime + opts.max_time_sec);
end

imudata = imudata(imudata(:, 1) >= cfg.starttime & imudata(:, 1) <= cfg.endtime, :);
gnss = gnss(gnss(:, 1) >= cfg.starttime & gnss(:, 1) <= cfg.endtime, :);
truth = truth(truth(:, 2) >= cfg.starttime & truth(:, 2) <= cfg.endtime, :);
if isempty(imudata) || isempty(gnss)
    error('IMU 与 GNSS 数据没有重叠。');
end

imu_times = imudata(:, 1);
node_indices = round(interp1(imu_times, (1:length(imu_times))', gnss(:, 1), 'nearest', 'extrap'));
[node_indices, keep] = unique(node_indices, 'stable');
gnss = gnss(keep, :);
num_nodes = length(node_indices);
rank = 15;

fprintf('处理时间: %.2f -> %.2f s, 节点数: %d\n', cfg.starttime, cfg.endtime, num_nodes);

%% 3. 用递推 KF 生成初始轨迹
% 标准非线性图优化需要一个足够好的线性化点。GNSS/INS 通常用前向 EKF
% 或纯 INS + 粗对准结果初始化；这里直接用同一套 GNSS 量测跑一遍 KF。
[kf_template, navstate] = myInitialize_15state(cfg);
kf = kf_template;
nav_nodes = cell(num_nodes, 1);
nav_nodes{1} = navstate;

lastimu = imudata(1, :)';
thisimu = imudata(1, :)';
next_node = 2;

for imuindex = 2:node_indices(end)
    lastimu = thisimu;
    thisimu = imudata(imuindex, :)';
    dt = thisimu(1) - lastimu(1);

    navstate = InsMech(navstate, lastimu, thisimu);
    kf = myInsPropagate_15state(navstate, thisimu, dt, kf);

    if next_node <= num_nodes && imuindex == node_indices(next_node)
        kf = myGNSSUpdate_15state(navstate, gnss(next_node, :)', kf);
        [kf, navstate] = myErrorFeedback_15state(kf, navstate);
        nav_nodes{next_node} = navstate;
        next_node = next_node + 1;
    end
end

%% 4. Gauss-Newton 迭代：重建因子、求解、反馈
history = zeros(opts.max_iterations, 3);
for iter = 1:opts.max_iterations
    fprintf('\nFGO iteration %d/%d: 重新线性化 IMU/GNSS 因子...\n', iter, opts.max_iterations);

    [Phi_blocks, Q_blocks, process_z] = buildImuFactors( ...
        nav_nodes, node_indices, imudata, kf_template, opts, rank);

    num_var = rank * num_nodes;
    A = sparse(num_var, num_var);
    b = zeros(num_var, 1);
    total_cost = 0;

    % 初始先验因子：固定第一节点，避免整体漂移自由度。
    idx0 = nodeBlock(1, rank);
    W0 = symInv(kf_template.P0);
    A(idx0, idx0) = A(idx0, idx0) + W0;

    % IMU 过程因子：z_ij = dx_j - Phi_ij * dx_i。
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
        total_cost = total_cost + z' * W * z;
    end

    % GNSS 位置因子：z_g = DR * (pos_nominal - pos_gnss) = H * dx。
    Wg = diag(1 ./ (opts.gnss_std_m(:) .^ 2));
    gnss_cost = 0;
    for k = 1:num_nodes
        idx = nodeBlock(k, rank);
        DR = positionDr(nav_nodes{k}.pos, param);
        z = DR * (nav_nodes{k}.pos - gnss(k, 2:4)');
        H = zeros(3, rank);
        H(:, 1:3) = DR;

        A(idx, idx) = A(idx, idx) + H' * Wg * H;
        b(idx) = b(idx) + H' * Wg * z;
        gnss_cost = gnss_cost + z' * Wg * z;
    end

    dx_vec = solveScaledNormalEquation(A, b, opts.state_scale, num_nodes);
    dx = reshape(dx_vec, rank, num_nodes);

    max_pos_dx = 0;
    for k = 1:num_nodes
        DR = positionDr(nav_nodes{k}.pos, param);
        max_pos_dx = max(max_pos_dx, norm(DR * dx(1:3, k)));
        nav_nodes{k} = retractNavState(nav_nodes{k}, dx(:, k));
    end

    history(iter, :) = [total_cost, gnss_cost, max_pos_dx];
    fprintf('  process cost %.3g, GNSS cost %.3g, max position update %.6f m\n', ...
        total_cost, gnss_cost, max_pos_dx);

    if max_pos_dx < opts.stop_dx_m
        history = history(1:iter, :);
        fprintf('  收敛：最大位置改正小于 %.3g m。\n', opts.stop_dx_m);
        break;
    end
end

%% 5. 输出结果与误差统计
truth_pos = [truth(:, 3) * param.D2R, truth(:, 4) * param.D2R, truth(:, 5)];
truth_node = interp1(truth(:, 2), truth_pos, gnss(:, 1), 'linear', 'extrap');
err_ned = zeros(num_nodes, 3);
for k = 1:num_nodes
    DR = positionDr(truth_node(k, :)', param);
    err_ned(k, :) = (DR * (nav_nodes{k}.pos - truth_node(k, :)'))';
end

writeNavRows(fullfile(output_dir, 'Standard-FGO-GNSS-INS-keyframes.nav'), nav_nodes, param);
% writematrix([gnss(:, 1), err_ned], fullfile(output_dir, 'standard_fgo_error_ned_m.txt'), 'Delimiter', 'space');
% result.opts = opts;
% result.history = history;
% result.node_times = gnss(:, 1);
% result.err_ned_m = err_ned;
% save(fullfile(output_dir, 'standard_fgo_gnss_ins_result.mat'), 'result');
% 
% fig = figure('Visible', 'off');
% t = gnss(:, 1) - gnss(1, 1);
% plot(t, vecnorm(err_ned(:, 1:2), 2, 2), 'b', 'LineWidth', 1.2);
% grid on;
% xlabel('Time (s)');
% ylabel('Horizontal error (m)');
% title('Standard GNSS/INS FGO');
% saveas(fig, fullfile(output_dir, 'standard_fgo_gnss_ins_error.png'));
% close(fig);
% 
% fprintf('\nStandard FGO 完成。水平 RMS = %.3f m, 高程 RMS = %.3f m\n', ...
%     rms(vecnorm(err_ned(:, 1:2), 2, 2)), rms(err_ned(:, 3)));
% fprintf('输出: %s\n', fullfile(output_dir, 'Standard-FGO-GNSS-INS-keyframes.nav'));

%% 局部函数
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
