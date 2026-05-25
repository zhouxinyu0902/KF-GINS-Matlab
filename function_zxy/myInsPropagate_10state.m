function kf = myInsPropagate_10state(navstate, thisimu, dt, kf)
   
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
    % 【核心添加：手写分块转换落地】重构并提取出 10x10 的联合矩阵
    F_new = zeros(kf.RANK, kf.RANK);  % kf.RANK 目前为 10
    PHI = eye(kf.RANK, kf.RANK);
    
    % 严格按照手写板书规划的物理通道抽取规则：
    %        rN, rE, vN, vE, psi_z, nabla_x, nabla_y, eps_z
    idx = [  1,  2,  4,  5,    9,     13,      14,     12  ];
    
    % 一行代码，直接把 15维中的核心水平与垂直维度抽取至新阵前 8 维
    F_new(1:8, 1:8) = F(idx, idx);
    % 至于最后的第 9, 10 行/列（theta, phi），全自动留白为 0，保持常数常态
    
    % 状态转移矩阵一阶离散化
    PHI = PHI + F_new * dt;

    %% 4. 噪声驱动矩阵与离散传播（限 12 维连续过程噪声）
    
     % 噪声驱动矩阵 G 与 5维连续过程噪声 Qd 重构
    G = zeros(kf.RANK, kf.NOISE_RANK); % 10 x 5 矩阵
    G(3:4, 1:2) = cbn(1:2, 1:2);       % 水平高频加计白噪声驱动水平速度
    G(5, 3)     = -cbn(3, 3);          % 垂直高频陀螺白噪声驱动航向角误差
    G(6:7, 4:5) = eye(2, 2);           % 一阶马尔可夫过程噪声驱动水平加计零偏
    
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