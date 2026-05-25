function [kf, navstate] = myErrorFeedback_range_m_10(kf, navstate)
% -------------------------------------------------------------------------
% 功能：基于 10 维联合估计滤波器的惯导全反馈与分阵营状态重置函数
% 
% 状态量映射 (kf.RANK = 10):
%   kf.x = [drN; drE; dvN; dvE; psi_z; del_ax; del_ay; eps_uz; theta; phi]
% -------------------------------------------------------------------------

    param = Param();
    h = navstate.pos(3); % 提取当前高度

    %% 1. 【核心保留】以载体当前纬度重新计算地球半径，维持原有几何基准
    [navstate.Rm, navstate.Rn] = getRmRn(navstate.pos(1), param);
    navstate.gravity = getGravity(navstate.pos);

    %% 2. 【分阵营精准反馈】将 1~8 维惯导误差全额对冲补偿给绝对编排大盘
    
    % A. 水平位置反馈：将水平米制误差 [drN; drE] 转换为大圆弧经纬度弧度扣除
    navstate.pos(1) = navstate.pos(1) - kf.x(1) / (navstate.Rm + h);
    navstate.pos(2) = navstate.pos(2) - kf.x(2) / ((navstate.Rn + h) * cos(navstate.pos(1)));
    % 注：navstate.pos(3) 天向高度维已被安全割离，不受任何水平标定噪声的污染

    % B. 水平速度反馈：对冲减去北速误差与东速误差
    navstate.vel(1) = navstate.vel(1) - kf.x(3);
    navstate.vel(2) = navstate.vel(2) - kf.x(4);

    % % C. 地理航向角反馈与姿态阵刷新
    navstate.att(3) = navstate.att(3) - kf.x(5);       % 修正地理方位航向
    navstate.cbn    = euler2dcm(navstate.att);         % 【关键】立刻刷新系统名义姿态阵
    % 
    % % D. IMU 硬件零偏常值累加反馈
    % navstate.accbias(1:2) = navstate.accbias(1:2) + kf.x(6:7)'; % 水平加计偏置
    % navstate.gyrbias(3)   = navstate.gyrbias(3) + kf.x(8);      % 天向陀螺偏置

    %% 3. 【核心更迭】状态向量分步重置锁（Reset）
    if ~isfield(navstate, 'theta_calib')
        navstate.theta_calib = 0; % 弧度
        navstate.phi_calib   = 0; % 弧度
    end

    % 滚动累加滤波辨识出的增量
    navstate.theta_calib = navstate.theta_calib + kf.x(9);
    navstate.phi_calib   = navstate.phi_calib + kf.x(10);

    kf.x = zeros(10, 1);

    % kf.x(1:8) = zeros(8, 1);
end