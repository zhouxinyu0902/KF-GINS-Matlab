function [kf, navstate] = myInitialize_10state(cfg)
% -------------------------------------------------------------------------
% 功能：基于 15 维原版改造的 10 维惯导-潜标流场联合估计卡尔曼滤波器初始化
% 状态向量定义：[rN; rE; vN; vE; psi_z; del_ax; del_ay; eps_uz; theta; phi]
% -------------------------------------------------------------------------
    
    glvs; % 确保高精度标定常量（如 glv.deg）可用

    % 1. 严格锁定 10 维状态与 5 维过程噪声
    kf.RANK = 10;
    kf.NOISE_RANK = 5;
    kf.P = zeros(kf.RANK, kf.RANK);
    kf.Qc = zeros(kf.NOISE_RANK, kf.NOISE_RANK);
    kf.x = zeros(kf.RANK, 1);
    
    %% 2. 连续过程噪声功率谱 Qc (5 x 5) 完美对齐
    kf.Qc(1:2, 1:2) = power(cfg.accvrw, 2) * eye(2, 2);  % 水平加速度计高频白噪声
    kf.Qc(3, 3)     = power(cfg.gyrarw, 2);              % 垂直陀螺高频白噪声
    kf.Qc(4:5, 4:5) = 2 * power(cfg.accbiasstd, 2) / cfg.corrtime * eye(2, 2); % 水平加计马尔氏驱动

    %% 3. 初始协方差矩阵 P0 (10 x 10) 全要素重构
    kf.P(1:2, 1:2) = diag(power(cfg.initposstd(1:2), 2));     % 1,2: 初始水平位置不确定度 (m)
    kf.P(3:4, 3:4) = diag(power(cfg.initvelstd(1:2), 2));     % 3,4: 初始水平速度不确定度 (m/s)
    kf.P(5, 5)     = power(cfg.initattstd(3), 2);              % 5:   初始航向失准角置信度 (rad)
    kf.P(6:7, 6:7) = diag(power(cfg.initaccbiasstd(1:2), 2)); % 6,7: 载体水平加计常值零偏 (m/s^2)
    kf.P(8, 8)     = power(cfg.initgyrbiasstd(3), 2);          % 8:   载体垂直陀螺常值零偏 (rad/s)
    
    % --- 【手写扩展核心】：并入潜标静态流场不确定度 ---
    kf.P(9, 9)     = power(30.0 * glv.deg, 2);                  % 9:   潜标共同流场倾角（放宽到3度）
    kf.P(10, 10)   = power(180.0 * glv.deg, 2);                % 10:  潜标方位角不确定度（全向360度）
    kf.P0 = kf.P;

    %% 4. 纯惯导机械编排所需的绝对解算参数初始化 (保持不变)
    navstate.time = cfg.starttime;
    navstate.pos = cfg.initpos;
    navstate.vel = cfg.initvel;
    navstate.att = cfg.initatt;
    navstate.cbn = euler2dcm(cfg.initatt);
    navstate.qbn = euler2quat(cfg.initatt);
    navstate.gyrbias = cfg.initgyrbias;
    navstate.accbias = cfg.initaccbias;
    navstate.gyrscale = cfg.initgyrscale;
    navstate.accscale = cfg.initaccscale;
    
    navstate.theta_calib = 10/180*pi; % 弧度
    navstate.phi_calib   = 0; % 弧度


    param = Param();
    [navstate.Rm, navstate.Rn] = getRmRn(cfg.initpos(1), param);
    navstate.gravity = getGravity(cfg.initpos);
end