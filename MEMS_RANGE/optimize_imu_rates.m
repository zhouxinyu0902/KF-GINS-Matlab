function [w_opt, f_opt] = optimize_imu_rates(w_tilde, f_tilde, b_g, b_a, state, beacon_pos, R_meas, params, dt)
% 输入 w_tilde, f_tilde 为原始 IMU 输出（deg/s, m/s^2）
% dt = 1 (测距周期)

% 1. 设置优化初值 (角速度和加速度形式)
w_init = w_tilde - b_g*[1,1,-1]';
f_init = f_tilde - b_a*[1,1,-1]';
x0 = [w_init; f_init];

% 2. 求解设置
options = optimoptions('fsolve', 'Display', 'off', ...
    'FunctionTolerance', 1e-10, ...
    'StepTolerance', 1e-10, ...
    'FiniteDifferenceStepSize', 1e-6, ...
    'Algorithm', 'levenberg-marquardt');

target_fun = @(x) rates_residual_equations(x, w_init, f_init, state, beacon_pos, R_meas, params, dt);

[x_opt, ~, exitflag] = fsolve(target_fun, x0, options);

if exitflag > 0

    w_opt = x_opt(1:3);
    f_opt = x_opt(4:6);
    % fprintf('角速度修正量: %.4e rad/s\n', norm(w_opt - w_init));
    % fprintf('加速度修正量: %.4e m/s^2\n', norm(f_opt - f_init));
else
    % 失败则返回初值
    w_opt = w_init;
    f_opt = f_init;
end
% --- 验证优化效果 ---
% g_n = [0; 0; 9.81];
% [rm, rn] = getRmRn(state.p0(1), params);
% Rc = diag([1/(rm + state.p0(3)), 1/((rn + state.p0(3))*cos(state.p0(1))), -1]);
% % 1. 计算优化前的预测位置和残差
% pk_init = state.p0 + Rc * state.v0 * dt + 0.5 * Rc * (state.Cb0_n * f_tilde + g_n) * dt^2; % 简化示意
% dist_init = norm(pos2dxyz(pk_init', beacon_pos));
% res_init_none = abs(R_meas - dist_init);
% 
% % 2. 计算优化后的预测位置和残差
% pk_opt = state.p0 + Rc * state.v0 * dt + 0.5 * Rc * (state.Cb0_n * f_opt + g_n) * dt^2;
% dist_opt = norm(pos2dxyz(pk_opt', beacon_pos));
% 
% % 3. 计算优化前的预测位置和残差
% pk_init = state.p0 + Rc * state.v0 * dt + 0.5 * Rc * (state.Cb0_n * (f_tilde-b_g) + g_n) * dt^2; % 简化示意
% dist_init = norm(pos2dxyz(pk_init', beacon_pos));
% 
% res_init = abs(R_meas - dist_init);
% res_opt = abs(R_meas - dist_opt);
% 3. 打印对比
% fprintf('减去偏值后的残差改善量:     %.10f m\n', res_init_none - res_init);
% fprintf('优化后的残差改善量:     %.10f m\n', res_init - res_opt);
%
% if (res_init - res_opt) > 0
%     disp('✅ 逻辑闭环：优化器成功降低了观测残差。');
% else
%     disp('❌ 逻辑可能存在问题：残差未下降或反而上升，检查梯度符号。');
% end
end

function residuals = rates_residual_equations(x, w_prior, f_prior, state, beacon_pos, R_meas, params, dt)
% x: [w; f] 待优化的角速度和加速度
w = x(1:3); f = x(4:6);

p0 = state.p0;
v0 = state.v0;
Cb0_n = state.Cb0_n; Cn_b = Cb0_n';
g_n = [0; 0; -9.81]; % 修正：NED系重力向下，如果是减去g项，注意符号
wnin = state.wnin;

% 1. Rc 矩阵
[rm, rn] = getRmRn(p0(1), params);
Rc = diag([1/(rm + p0(3)), 1/((rn + p0(3))*cos(p0(1))), -1]);

% 2. 位置推算 (简化模型)
% pk = p0 + (Rc * v0) * dt + 0.5 * Rc * (Cb0_n * f + g_n) * dt^2
% 注意：如果 state.v0 已经是导航系速度，Rc 转换是正确的
% pk = p0 + Rc * v0 * dt + 0.5 * Rc *  g_n * dt^2;
pk = p0 + Rc * v0 * dt + 0.5 * Rc * (Cb0_n * f + g_n) * dt^2;

% 3. 观测残差计算
rang_vec_m = pos2dxyz(pk', beacon_pos); % 预测点到基站的米制矢量
dist_est = norm(rang_vec_m);

% M 系数：观测项的增益
% 这里 dist_est 不能为 0
M = (R_meas - dist_est) / (params.sigma_R^2 * dist_est);

% 4. 构建残差 (采取权重平衡方案)
% 我们将方程两边乘以 params.sigma^2，这样 IMU 项系数变为 1

skew_w = [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];
skew_f = [0, -f(3), f(2); f(3), 0, -f(1); -f(2), f(1), 0];
skew_wnin = [0, -wnin(3), wnin(2); wnin(3), 0, -wnin(1); -wnin(2), wnin(1), 0];

% dp 建议直接用预测点到基站的米制矢量，因为它代表了“观测距离误差”的方向
dp = rang_vec_m';

% --- 关键：平衡后的残差方程 ---
% res_w: 角速度误差 + 观测反馈
% 反馈系数增加了 params.sigma_g^2
res_w = (w - w_prior) + ...
    (params.sigma_g^2 * M) * (skew_f * Cn_b * dp * dt^3);

% res_f: 加速度误差 + 观测反馈
term1 = Cn_b * dt^2;
term2 = skew_w * Cn_b * dt^3;
term3 = Cn_b * skew_wnin * dt^3;

res_f = (f - f_prior) + ...
    (params.sigma_a^2 * M) * (term1 - term2 + term3) * dp;

residuals = [res_w ; res_f];
end