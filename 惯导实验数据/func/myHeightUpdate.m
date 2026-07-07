function kf = myHeightUpdate(navstate, depthdata, kf)
% navstate: 导航状态结构体
% depthdata: [时间, 深度/高度观测值]
% kf: 滤波器结构体
%% 1. 计算观测残差 (Innovation)
Z = navstate.pos(3) - depthdata(2);
kf.Z = Z;
%% 2. 构造量测矩阵 H
H = zeros(1, kf.RANK);
H(1, 3) = 1; 
% 计算预测观测值 
kf.Zkk_1 = H * kf.x;
%% 3. 计算卡尔曼增益 K
R = kf.depthstd ^ 2;
K_full = kf.P * H' / (H * kf.P * H' + R);
%% 4. 实施“解耦反馈”掩码 (Decoupled Feedback Mask)
% 核心逻辑：只允许增益流向天向位置和天向速度
pos_u_idx = 3; 
vel_u_idx = 6; % 在大多数15维滤波器中，6是天向速度

mask = zeros(kf.RANK, 1);
mask(pos_u_idx) = 1; 
mask(vel_u_idx) = 1; 
% K = K_full;
% 强制截断增益
K = K_full .* mask;
%% 5. 更新状态量与协方差矩阵
kf.x = kf.x + K * (Z - kf.Zkk_1);
I = eye(kf.RANK);
kf.P = (I - K * H) * kf.P * (I - K * H)' + K * R * K';

end