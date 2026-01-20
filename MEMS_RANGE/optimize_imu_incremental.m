function [dtheta_opt, dv_opt] = optimize_imu_incremental(dtheta_tilde, dv_tilde, state, beacon_pos, R_meas, params, Hz)
    dt = 1/Hz;
    
    % 1. 设置优化初值 (增量形式)
    dtheta_init = dtheta_tilde ;
    dv_init = dv_tilde;
    x0 = [dtheta_init; dv_init]; 

    % 2. 求解非线性方程组
    options = optimoptions('fsolve', 'Display', 'off',...
        'FunctionTolerance', 1e-15, 'StepTolerance', 1e-12);
    target_fun = @(x) incremental_residual_equations(x, dtheta_tilde, dv_tilde,state, beacon_pos, R_meas, params, dt);
    [x_opt, ~, exitflag, ~] = fsolve(target_fun, x0, options);

    if exitflag > 0
        dtheta_opt = x_opt(1:3);
        dv_opt = x_opt(4:6);
    else
        dtheta_opt = dtheta_init;
        dv_opt = dv_init;
    end
    
end

% function residuals = incremental_residual_equations(x, dtheta_tilde, dv_tilde,state, beacon_pos, R_meas, params, dt)
%     % 1. 状态提取
%     dtheta = x(1:3); dv = x(4:6);
%     w = dtheta / dt; f = dv / dt;
%     p0 = state.p0; 
%     v0 = state.v0;
%     Cb0_n = state.Cb0_n; Cn_b = Cb0_n';     
%     g_n = [0; 0; -9.81]; % NED系，向下为正
%     wnin = state.wnin;
% 
%     % 2. Rc 矩阵（仅用于位置更新 pk）
%     [rm, rn] = getRmRn(p0(1), params);
%     Rc = diag([1/(rm + p0(3)), 1/((rn + p0(3))*cos(p0(1))), -1]);
% 
%     % 3. 位置推算 (公式 3.13 & 3.14)
%     skew_w = [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];
%     skew_wnin = [0, -wnin(3), wnin(2); wnin(3), 0, -wnin(1); -wnin(2), wnin(1), 0];
% 
%     G = p0 + Rc * v0 * dt + 0.5 * Rc * g_n * dt^2;
%     term_f = Rc * Cb0_n * f * dt^2;
%     pk = G + term_f; 
%     % 4. 数值对齐：计算导航系下的米制差值 (NED)
%     rang_vec_m = pos2dxyz(pk', beacon_pos); % 预测点到基站的 [dN, dE, dD] (m)
%     dist_est = norm(rang_vec_m);            % 3D 距离 (m)
% 
%     dp = Rc^-1 *( pk -beacon_pos);             % 预测点相对于起点的位移 (m)
% 
%     % 5. 残差系数 M
%     M = (R_meas - dist_est) / (params.sigma_R^2 * dist_est);
% 
%     % 6. 偏导数方程 (统一使用米制矢量)
%     skew_f = [0, -f(3), f(2); f(3), 0, -f(1); -f(2), f(1), 0];  
%     res_w = (1/params.sigma_g^2) * (dtheta_tilde/dt-w) - ...
%             M * skew_f * Cn_b * dp * dt^3;
% 
% 
%     term1 = Cn_b * dt^2;
%     term2 = skew_w * Cn_b  * dt^3; 
%     term3 = Cn_b * skew_wnin  * dt^3; 
%     res_f = (1/params.sigma_a^2) * (dv_tilde/dt - f  ) - ...
%             M * (term1 - term2 + term3) * dp;
% 
% 
%     residuals = [res_w ; res_f]; 
% end

function residuals = incremental_residual_equations(x, dtheta_tilde, dv_tilde, state, beacon_pos, R_meas, params, dt)
    % 1. 变量提取
    dtheta = x(1:3); dv = x(4:6);
    w = dtheta / dt; f = dv / dt;
    
    % 2. 位置推算 (必须包含 f，优化器才能看到 f 的影响)
    [rm, rn] = getRmRn(state.p0(1), params);
    Rc = diag([1/(rm + state.p0(3)), 1/((rn + state.p0(3))*cos(state.p0(1))), -1]);
    
    % pk = p0 + v*dt + 0.5*g*dt^2 + 0.5*Rc*Cb_n*f*dt^2
    pk = state.p0 + Rc * state.v0 * dt + 0.5 * Rc * [0; 0; 9.81] * dt^2 + 0.5 * Rc * (state.Cb0_n * f) * dt^2;
    
    % 3. 关键修正：获取米制误差矢量
    % rang_vec_m 是从基站指向飞机的矢量 [dN; dE; dD]，单位：米
    dp_n = pos2dxyz(pk', beacon_pos)'; 
    dist_est = norm(dp_n);
    
    % 4. 观测增益 M (包含 1/sigma_R^2)
    M = (R_meas - dist_est) / (params.sigma_R^2 * dist_est);
    
    % 5. 关键修正：归一化残差方程 (乘以 sigma^2 以平衡量级)
    Cn_b = state.Cb0_n';
    skew_f = [0, -f(3), f(2); f(3), 0, -f(1); -f(2), f(1), 0];
    skew_w = [0, -w(3), w(2); w(3), 0, -w(1); -w(2), w(1), 0];
    
    % 这一步将两项量级拉到 1e-3 左右。注意 M 前面是 + 还是 - 需根据 dp_n 方向确定。
    % 按照论文梯度下降逻辑，如果 R > dist，M 为正，修正量应沿梯度正向。
    res_w = (w - dtheta_tilde/dt) + (params.sigma_g^2 * M) * (skew_f * Cn_b * dp_n * dt^3);
    
    term1 = Cn_b * dt^2;
    term2 = skew_w * Cn_b * dt^3; 
    res_f = (f - dv_tilde/dt) + (params.sigma_a^2 * M) * (term1 - term2) * dp_n;
            
    residuals = [res_w ; res_f]; 
end