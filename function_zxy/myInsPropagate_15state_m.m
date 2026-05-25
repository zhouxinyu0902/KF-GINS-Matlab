% -------------------------------------------------------------------------
% 基于 KF-GINS 裁剪的 15 维全米制惯导一步预测函数
% 状态量：[位置_m(3), 速度_m/s(3), 姿态失准角_rad(3), 陀螺零偏_rad/s(3), 加计零偏_m/s^2(3)]
% -------------------------------------------------------------------------

function kf = myInsPropagate_15state_m(navstate, thisimu, dt, kf)
    
    param = Param();
    corrtime = 3600; % 设定一阶高斯马尔可夫相关时间（3600秒）
    
    %% 1. 复制绝对导航参数
    % 注意：navstate.pos 内部仍保持绝对大地坐标 [纬度(rad), 经度(rad), 高度(m)]
    pos = navstate.pos;
    vel = navstate.vel;
    cbn = navstate.cbn;
    rm = navstate.Rm;
    rn = navstate.Rn;
    gravity = navstate.gravity;

    % IMU 数据（增量除以时间，转换为角速度和加速度）
    omega = thisimu(2:4, 1) / dt;
    accel = thisimu(5:7, 1) / dt;

    % 地理系空间几何参数
    rmh = rm + pos(3);
    rnh = rn + pos(3);
    wie_n = [param.WGS84_WIE * cos(pos(1)); 0; -param.WGS84_WIE * sin(pos(1))];
    wen_n = [vel(2) / rnh; -vel(1) / rmh; -vel(2) * tan(pos(1)) / rnh];

    %% 2. 构造连续时间系统矩阵 F（严格限定为 15 维）
    F = zeros(kf.RANK, kf.RANK);       % kf.RANK 应为 15
    PHI = eye(kf.RANK, kf.RANK);

    % --- 2.1 位置误差动力学 (米制) ---
    Frr = zeros(3, 3);
    Frr(1, 1) = -vel(3) / rmh;
    Frr(1, 3) = vel(1) / rmh;
    Frr(2, 1) = vel(2) * tan(pos(1)) / rnh;
    Frr(2, 2) = -(vel(3) + vel(1) * tan(pos(1))) / rnh;
    Frr(2, 3) = vel(2) / rnh;
    F(1:3, 1:3) = Frr;
    F(1:3, 4:6) = eye(3); % 速度直接驱动米制位置
    % F(1:3, 4:6) = diag([1,1,-1]);

    % --- 2.2 速度误差动力学 ---
    Fvr = zeros(3, 3);
    % 替换原源码中的 pow2 为 ^2
    Fvr(1, 1) = -2 * vel(2) * param.WGS84_WIE * cos(pos(1)) / rmh - (vel(2))^2 / rmh / rnh / (cos(pos(1)))^2;
    Fvr(1, 3) = vel(1) * vel(3) / rmh / rmh - (vel(2))^2 * tan(pos(1)) / rnh / rnh;
    Fvr(2, 1) = 2 * param.WGS84_WIE * (vel(1) * cos(pos(1)) - vel(3) * sin(pos(1))) / rmh + vel(1) * vel(2) / rmh / rnh / (cos(pos(1)))^2;
    Fvr(2, 3) = (vel(2) * vel(3) + vel(1) * vel(2) * tan(pos(1))) / rnh / rnh;
    Fvr(3, 1) = 2 * param.WGS84_WIE * vel(2) * sin(pos(1)) / rmh;
    Fvr(3, 3) = -(vel(2))^2 / rnh / rnh - (vel(1))^2 / rmh / rmh + 2 * gravity / (sqrt(rm * rn) + pos(3));
    F(4:6, 1:3) = Fvr;
    
    Fvv = zeros(3, 3);
    Fvv(1, 1) = vel(3) / rmh;
    Fvv(1, 2) = -2 * (param.WGS84_WIE * sin(pos(1)) + vel(2) * tan(pos(1)) / rnh);
    Fvv(1, 3) = vel(1) / rmh;
    Fvv(2, 1) = 2 * param.WGS84_WIE * sin(pos(1)) + vel(2) * tan(pos(1)) / rnh;
    Fvv(2, 2) = (vel(3) + vel(1) * tan(pos(1))) / rnh;
    Fvv(2, 3) = 2 * param.WGS84_WIE * cos(pos(1)) + vel(2) / rnh;
    Fvv(3, 1) = -2 * vel(1) / rmh;
    Fvv(3, 2) = -2 * (param.WGS84_WIE * cos(pos(1)) + vel(2) / rnh);
    F(4:6, 4:6) = Fvv;
    
    % 核心交联项（限 15 维）
    F(4:6, 7:9) = skew(cbn * accel);   % 姿态失准角污染速度
    F(4:6, 13:15) = cbn;               % 加速度计零偏污染速度

    % --- 2.3 姿态误差动力学 ---
    Fphir = zeros(3, 3);
    Fphir(1, 1) = -param.WGS84_WIE * sin(pos(1)) / rmh;
    Fphir(1, 3) = vel(2) / rnh / rnh;
    Fphir(2, 3) = -vel(1) / rmh / rmh;
    Fphir(3, 1) = -param.WGS84_WIE * cos(pos(1)) / rmh - vel(2) / rmh / rnh / (cos(pos(1)))^2;
    Fphir(3, 3) = -vel(2) * tan(pos(1)) / rnh / rnh;
    F(7:9, 1:3) = Fphir;
    
    Fphiv = zeros(3, 3);
    Fphiv(1, 2) = 1 / rnh;
    Fphiv(2, 1) = -1 / rmh;
    Fphiv(3, 2) = -tan(pos(1)) / rnh;
    F(7:9, 4:6) = Fphiv;
    F(7:9, 7:9) = -skew(wie_n + wen_n);
    F(7:9, 10:12) = -cbn;              % 陀螺仪零偏污染姿态

    % --- 2.4 传感器零偏（一阶高斯-马尔可夫过程） ---
    F(10:12, 10:12) = -1 / corrtime * eye(3); 
    F(13:15, 13:15) = -1 / corrtime * eye(3); 

    %% 3. 离散化状态转移矩阵 
    F_dt = F * dt;
    PHI = PHI + F_dt ;

    %% 4. 噪声驱动矩阵与离散传播（限 12 维连续过程噪声）
    G = zeros(kf.RANK, kf.NOISE_RANK); % 15 * 12 矩阵
    G(4:6, 1:3) = cbn;                 % 速度项注入加计白噪声
    G(7:9, 4:6) = cbn;                 % 姿态项注入陀螺白噪声
    G(10:12, 7:9) = eye(3);            % 陀螺零偏驱动
    G(13:15, 10:12) = eye(3);          % 加计零偏驱动
    
    % 计算离散过程噪声 Qd (梯形积分改良)
    Qd = G * kf.Qc * G' * dt;
    Qd = (PHI * Qd * PHI' + Qd) / 2;

    %% 5. 协方差与状态一步预测
    kf.P = PHI * kf.P * PHI' + Qd;
    
    % 强制对称化，消除有限浮点数带来的数值下溢和截断误差
    kf.P = (kf.P + kf.P') / 2; 
    % if kf.P(1,1) >1e4
    %     keyboard;
    % end
    kf.Pk_k1 = kf.P;
    kf.x = PHI * kf.x;
    kf.phi = PHI;
end