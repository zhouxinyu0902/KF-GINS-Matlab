%% GNSS/INS 因子图组合导航示例
% 架构参考 exper 脚本的顺序：参数 -> 数据 -> 主循环 -> 图优化 -> 输出。
% 这里故意把主要逻辑放在一个脚本内，便于先看清 FGO 的处理流程。

clear; clc; close all;

%% 1. 参数与路径
topic_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(topic_root);
paths = setup_factor_graph_navigation();
input_dir = paths.input;
output_dir = paths.navigation_results;
derived_input_dir = paths.derived_input;

param = Param();
cfg = config_factor_graph_simulation(input_dir);
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

opts.gnss_rate_hz = 1;                 % GNSS 频率示例：1 Hz
opts.max_time_sec = inf;              % 设为 inf 可跑完整数据
opts.random_seed = 1;
opts.gnss_pos_std_m = [2; 2; 3];       % N/E/U 位置噪声标准差，单位 m
opts.process_std_floor = [ ...
    1e-12; 1e-12; 1e-4; ...
    1e-5; 1e-5; 1e-5; ...
    1e-8; 1e-8; 1e-8; ...
    1e-12; 1e-12; 1e-12; ...
    1e-8; 1e-8; 1e-8];
opts.state_scale = [ ...
    1e-6; 1e-6; 1; ...
    0.1; 0.1; 0.1; ...
    1e-3; 1e-3; 1e-3; ...
    1e-6; 1e-6; 1e-6; ...
    1e-4; 1e-4; 1e-4];

fprintf('\n========== GNSS/INS 因子图组合导航 ==========\n');
fprintf('输入目录: %s\n', input_dir);
fprintf('输出目录: %s\n', output_dir);

%% 2. 导入 IMU 与 truth，并构造 GNSS 量测
imudata = importdata(cfg.imufilepath);
truth = importdata(cfg.truthpath);

cfg.starttime = max([cfg.starttime, imudata(1, 1), truth(1, 2)]);
cfg.endtime = min([cfg.endtime, imudata(end, 1), truth(end, 2)]);
if isfinite(opts.max_time_sec)
    cfg.endtime = min(cfg.endtime, cfg.starttime + opts.max_time_sec);
end

imudata = imudata(imudata(:, 1) >= cfg.starttime & imudata(:, 1) <= cfg.endtime, :);
truth = truth(truth(:, 2) >= cfg.starttime & truth(:, 2) <= cfg.endtime, :);
if isempty(imudata) || isempty(truth)
    error('IMU 与 truth 数据在处理时间段内没有重叠。');
end

rng(opts.random_seed);
gnss_time = (cfg.starttime : 1 / opts.gnss_rate_hz : cfg.endtime)';
truth_pos_rad = [truth(:, 3) * param.D2R, truth(:, 4) * param.D2R, truth(:, 5)];
truth_pos_node = interp1(truth(:, 2), truth_pos_rad, gnss_time, 'linear', 'extrap');
gnss_pos = truth_pos_node;

for k = 1:size(gnss_pos, 1)
    DR = positionDr(truth_pos_node(k, :)', param);
    noise_neu = opts.gnss_pos_std_m(:) .* randn(3, 1);
    noise_ned = [noise_neu(1); noise_neu(2); -noise_neu(3)];
    gnss_pos(k, :) = truth_pos_node(k, :)' + (DR \ noise_ned);
end

gnss = [gnss_time, gnss_pos];
writematrix([gnss_time, gnss_pos(:, 1:2) * param.R2D, gnss_pos(:, 3)], ...
    fullfile(derived_input_dir, 'simulated_gnss.txt'), 'Delimiter', 'space');

fprintf('处理时间: %.2f -> %.2f s, GNSS 节点数: %d\n', ...
    cfg.starttime, cfg.endtime, size(gnss, 1));

%% 3. 名义惯导递推，并在 GNSS 历元建立图节点
[kf0, navstate] = myInitialize_15state(cfg);
imu_times = imudata(:, 1);
node_indices = round(interp1(imu_times, (1:length(imu_times))', ...
    gnss(:, 1), 'nearest', 'extrap'));
[node_indices, keep] = unique(node_indices, 'stable');
gnss = gnss(keep, :);
truth_pos_node = truth_pos_node(keep, :);

num_nodes = length(node_indices);
state_rank = 15;
nav_nodes = cell(num_nodes, 1);
Phi_blocks = cell(num_nodes - 1, 1);
Q_blocks = cell(num_nodes - 1, 1);

nav_nodes{1} = navstate;
Phi_acc = eye(state_rank);
Q_acc = zeros(state_rank);
next_node = 2;

fprintf('开始惯导递推并累计节点间 INS 因子...\n');
for imu_index = 2:node_indices(end)
    lastimu = imudata(imu_index - 1, :)';
    thisimu = imudata(imu_index, :)';
    dt = thisimu(1) - lastimu(1);
    if dt <= 0
        error('IMU 时间戳必须严格递增。');
    end

    navstate = InsMech(navstate, lastimu, thisimu);

    kf_step = kf0;
    kf_step.x = zeros(state_rank, 1);
    kf_step.P = zeros(state_rank);
    kf_step = myInsPropagate_15state(navstate, thisimu, dt, kf_step);

    Phi_acc = kf_step.phi * Phi_acc;
    Q_acc = kf_step.phi * Q_acc * kf_step.phi' + (kf_step.P + kf_step.P') / 2;

    if next_node <= num_nodes && imu_index == node_indices(next_node)
        nav_nodes{next_node} = navstate;
        Phi_blocks{next_node - 1} = Phi_acc;
        Q_blocks{next_node - 1} = regularizeCov(Q_acc, opts.process_std_floor);
        Phi_acc = eye(state_rank);
        Q_acc = zeros(state_rank);
        next_node = next_node + 1;
    end
end

if next_node <= num_nodes
    error('惯导递推没有覆盖所有 GNSS 图节点。');
end

%% 4. 组装因子图：先验因子 + INS 过程因子 + GNSS 位置因子
fprintf('组装稀疏因子图...\n');
num_var = state_rank * num_nodes;
A = sparse(num_var, num_var);
b = zeros(num_var, 1);

idx0 = nodeBlock(1, state_rank);
A(idx0, idx0) = A(idx0, idx0) + symInv(kf0.P0);

for k = 1:(num_nodes - 1)
    idx_i = nodeBlock(k, state_rank);
    idx_j = nodeBlock(k + 1, state_rank);
    Phi = Phi_blocks{k};
    W = symInv(Q_blocks{k});

    % INS 因子：dx_{k+1} = Phi_k * dx_k
    A(idx_i, idx_i) = A(idx_i, idx_i) + Phi' * W * Phi;
    A(idx_i, idx_j) = A(idx_i, idx_j) - Phi' * W;
    A(idx_j, idx_i) = A(idx_j, idx_i) - W * Phi;
    A(idx_j, idx_j) = A(idx_j, idx_j) + W;
end

for k = 1:num_nodes
    idx = nodeBlock(k, state_rank);
    DR = positionDr(nav_nodes{k}.pos, param);
    z = DR * (nav_nodes{k}.pos - gnss(k, 2:4)');
    H = zeros(3, state_rank);
    H(:, 1:3) = DR;
    W = diag(1 ./ (opts.gnss_pos_std_m(:) .^ 2));

    % GNSS 因子：z = H * dx，dx 是名义位置误差，后面用 nav - dx 做反馈。
    A(idx, idx) = A(idx, idx) + H' * W * H;
    b(idx) = b(idx) + H' * W * z;
end

A = (A + A') / 2;

%% 5. 求解图优化，并把误差状态反馈到导航节点
fprintf('求解 %d 个节点、%d 维变量的线性因子图...\n', num_nodes, num_var);
scale_vec = repmat(opts.state_scale(:), num_nodes, 1);
S = spdiags(scale_vec, 0, num_var, num_var);
dx_vec = scale_vec .* (((S * A * S) + speye(num_var) * 1e-12) \ (S * b));
dx = reshape(dx_vec, state_rank, num_nodes);

nav_fgo = cell(num_nodes, 1);
for k = 1:num_nodes
    nav_fgo{k} = nav_nodes{k};
    nav_fgo{k}.pos = nav_fgo{k}.pos - dx(1:3, k);
    nav_fgo{k}.vel = nav_fgo{k}.vel - dx(4:6, k);
    [nav_fgo{k}.Rm, nav_fgo{k}.Rn] = getRmRn(nav_fgo{k}.pos(1), param);
    nav_fgo{k}.gravity = getGravity(nav_fgo{k}.pos);
end

%% 6. 输出与简单诊断
nominal_err = zeros(num_nodes, 3);
fgo_err = zeros(num_nodes, 3);
for k = 1:num_nodes
    DR = positionDr(truth_pos_node(k, :)', param);
    nominal_err(k, :) = (DR * (nav_nodes{k}.pos - truth_pos_node(k, :)'))';
    fgo_err(k, :) = (DR * (nav_fgo{k}.pos - truth_pos_node(k, :)'))';
end

writeNavRows(fullfile(output_dir, 'INS-nominal-keyframes.nav'), nav_nodes, param);
% writeNavRows(fullfile(output_dir, 'FGO-GNSS-INS-keyframes.nav'), nav_fgo, param);
% writematrix([gnss(:, 1), nominal_err, fgo_err], ...
%     fullfile(output_dir, 'fgo_error_ned_m.txt'), 'Delimiter', 'space');
% 
% result.opts = opts;
% result.cfg = cfg;
% result.gnss = gnss;
% result.node_times = gnss(:, 1);
% result.dx = dx;
% result.nominal_err_ned_m = nominal_err;
% result.fgo_err_ned_m = fgo_err;
% save(fullfile(output_dir, 'fgo_gnss_ins_result.mat'), 'result');
% 
% fig = figure('Visible', 'off');
% t = gnss(:, 1) - gnss(1, 1);
% plot(t, vecnorm(nominal_err(:, 1:2), 2, 2), 'Color', [0.60 0.60 0.60], 'LineWidth', 1.0);
% hold on;
% plot(t, vecnorm(fgo_err(:, 1:2), 2, 2), 'b', 'LineWidth', 1.3);
% grid on;
% xlabel('Time (s)');
% ylabel('Horizontal error (m)');
% legend('Nominal INS', 'FGO GNSS/INS', 'Location', 'best');
% title('GNSS/INS factor graph result');
% saveas(fig, fullfile(output_dir, 'fgo_gnss_ins_error.png'));
% close(fig);
% 
% fprintf('完成。名义水平 RMS = %.3f m, FGO 水平 RMS = %.3f m\n', ...
%     rms(vecnorm(nominal_err(:, 1:2), 2, 2)), rms(vecnorm(fgo_err(:, 1:2), 2, 2)));
% fprintf('结果已写入: %s\n', output_dir);

%% 本脚本只保留少量小工具函数，主线逻辑都在上面。
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
