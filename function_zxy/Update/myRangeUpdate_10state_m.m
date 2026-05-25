function [kf, navstate] = myRangeUpdate_10state_m(navstate, RangeData_3x6, d_sub, kf)
% -------------------------------------------------------------------------
% 功能：严格遵循原版 [Z = 理论(INS) - 实测(meas)] 逻辑重构的 10维三距离并联水平测距更新函数
% 
% 状态量映射 (kf.RANK = 10):
%   kf.x = [drN; drE; dvN; dvE; psi_z; del_ax; del_ay; eps_uz; theta; phi]
% -------------------------------------------------------------------------

   param = Param();
    
    %% 1. 【核心保留】以载体当前纬度计算地球半径，维持原有几何基准
    [rm, rn] = getRmRn(navstate.pos(1), param);
    h = navstate.pos(3);
    
    % 提取当前滤波器中的潜标流场绝对先验状态
    % theta_e = kf.x(9);
    % phi_e   = kf.x(10);
    
    theta_e = navstate.theta_calib; 
    phi_e   = navstate.phi_calib;

    % 提取 3 个潜标的实测水平投影距离
    R_h_meas = RangeData_3x6(1:3, 3); 
    P_surface_llh = RangeData_3x6(1:3, 4:6);

    % 量测噪声协方差 (沿用你原代码的 kf.rangstd 变量)
    R = diag(25 * ones(3, 1));
    
    %% 2. 预分配 3 维量测残差与 3 x 10 观测雅可比矩阵
    Z = zeros(3, 1);
    H = zeros(3, kf.RANK); % kf.RANK = 10

    % 地理坐标差投影阵 (2x2 水平裁剪，剥离原本高度维的 -1)
    DR_2d = diag([rm + h, (rn + h)*cos(navstate.pos(1))]); 

    %% 3. 循环并行构建 3 个基站的量测几何 (严格保持你原本的计算流)
    for k = 1:3
        % A. 将米制的流场钟摆形变转换为大圆弧弧度偏置
        delta_lat = (d_sub(k) * tan(theta_e) * cos(phi_e)) / (rm + h);
        delta_lon = (d_sub(k) * tan(theta_e) * sin(phi_e)) / ((rn + h) * cos(navstate.pos(1)));
        
        % B. 合成当前时刻该潜标发声点的实际绝对经纬度位置
        bcn_actual = [P_surface_llh(k, 1) + delta_lat;  % 实际发声点纬度
                      P_surface_llh(k, 2) + delta_lon]; % 实际发声点经度
        
        % C. 【原版逻辑复刻】绝对地理坐标差经由 DR 转换为米制直角差
        delta_pos = (DR_2d * (navstate.pos(1:2) - bcn_actual))';
        
        % D. 计算理论估计水平距离
        HorizR = sqrt(sum(delta_pos(1:2).^2));
        
        % E. 【原版逻辑】：Z = 理论估计距离 - 实测距离
        Z(k) = HorizR - R_h_meas(k);
        
        % F. 【原版正号保留】：构建 H 矩阵的前两维位置偏导 (形式与原本完全相同)
        H(k, 1) = delta_pos(1) / HorizR; % 北向位置偏导
        H(k, 2) = delta_pos(2) / HorizR; % 东向位置偏导
        % H(k, 3:8) 速度与惯导零偏通道几何偏导严格置 0
        
        % G. 【符号对齐重构】：引入流场参数对 HorizR 的解析偏导项
        sec_theta2 = 1 / power(cos(theta_e), 2);
        
        % 9: 对潜标流场倾角 theta 的偏导
        H(k, 9) = d_sub(k) * sec_theta2 * (delta_pos(1) * cos(phi_e) + delta_pos(2) * sin(phi_e)) / HorizR;
        
        % 10: 对潜标偏移方位角 phi 的偏导
        H(k, 10) = -d_sub(k) * tan(theta_e) * (delta_pos(1) * sin(phi_e) - delta_pos(2) * cos(phi_e)) / HorizR;
    end
    
    kf.Z = Z;
    kf.Zkk_1 = H * kf.x;

    %% 4. 经典的卡尔曼滤波更新大盘
    K = kf.P * H' / (H * kf.P * H' + R);
    kf.x = kf.x + K * (Z - kf.Zkk_1);
    
    % 沿用你原本的经典抗截断约瑟夫更新形式
    kf.P = (eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
    kf.P = (kf.P + kf.P') / 2;

end