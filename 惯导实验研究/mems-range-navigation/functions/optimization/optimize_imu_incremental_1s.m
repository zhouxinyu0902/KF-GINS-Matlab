function [dtheta_opt, dv_opt] = optimize_imu_incremental_1s(dtheta_tilde, dv_tilde, state, beacon_pos, R_meas, params, Hz)
    dt = 1/Hz; % 1s 周期
    
    % 1. 设置优化初值 (1s内的累加增量)
    x0 = [dtheta_tilde; dv_tilde]; 
    
    % 2. 配置优化参数 (1s尺度下精度可以略微放宽，增强收敛速度)
options = optimoptions('fsolve', 'Display', 'off',...
    'FunctionTolerance', 1e-15, ... % 远小于你当前的残差量级 (1e-7)
    'StepTolerance', 1e-12,     ... % 允许极小的变量更新
    'OptimalityTolerance', 1e-9);  % 一阶最优性条件的阈值
    
    target_fun = @(x) incremental_residual_equations(x, dtheta_tilde, dv_tilde, state, beacon_pos, R_meas, params, dt);
    
    [x_opt, ~, exitflag, ~] = fsolve(target_fun, x0, options);
    
    if exitflag > 0
        dtheta_opt = x_opt(1:3);
        dv_opt = x_opt(4:6);
    else
        dtheta_opt = dtheta_tilde; % 失败则保持原状
        dv_opt = dv_tilde;
    end
end

function residuals = incremental_residual_equations(x, dtheta_tilde, dv_tilde, state, beacon_pos, R_meas, params, dt)
    % 1. 变量提取
    dtheta = x(1:3); dv = x(4:6);
    w = dtheta / dt; f = dv / dt; % 速率
    Cn_b = state.Cb0_n';
    Cb_n = state.Cb0_n;
    wnin =state.wnin;
    
    [rm, rn] = getRmRn(state.p0(1), params);
    Rc = diag([1/(rm + state.p0(3)), 1/((rn + state.p0(3))*cos(state.p0(1))), -1]);
    skew_f = [0, -f(3), f(2); f(3), 0, -f(1); -f(2), f(1), 0];
    skew_w = [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];
    skew_wnin = [0, -wnin(3), wnin(2); wnin(3), 0, -wnin(1); -wnin(2), wnin(1), 0];
    
    % 2. 预测位置 pk (1s 跨度)
    g_n = [0; 0; 9.81]; % 确保重力方向与 state.p0 坐标系一致 (NED为正)
    G = state.p0 + Rc * state.v0 * dt + Rc *g_n* dt^2;
    temp1 = Rc * Cb_n * f * dt^2;
    temp2 = Rc * Cb_n * skew_w * f * dt^3;
    temp3 = -Rc * skew_wnin * Cb_n * f * dt^3;
    pk = G + temp1 + temp2 + temp3;

    % 3. 核心修正：使用统一的米制矢量 dp
    % rang_vec_m 得到 [dN, dE, dD]，这正是论文中的位移向量
    dp_n = pos2dxyz(pk', beacon_pos)'; 
    dist_est = norm(dp_n);
    dp_n =Rc^-1 * (pk-beacon_pos);
    % 4. 观测增益 M (包含 1/sigma_R^2)
    M = (R_meas - dist_est) / (params.sigma_R^2 * dist_est);

    res_w = (w - dtheta_tilde/dt) - (params.sigma_g^2 * M) * (skew_f * Cn_b * dp_n * dt^3);
    term1 = Cn_b * dt^2;
    term2 = skew_w * Cn_b * dt^3; 
    term3 = Cn_b * skew_wnin * dt^3;
    res_f = (f - dv_tilde/dt) - (params.sigma_a^2 * M) * (term1 - term2 + term3) * dp_n;
            
    residuals = [res_w; res_f]; 
end