function kf = myHeightUpdate(navstate, depthdata, kf)
% navstate: 导航状态结构体
% depthdata: [时间, 深度/高度观测值]
% kf: 滤波器结构体

%% 1. 计算观测残差 (Innovation)
% 假设 navstate.pos(3) 是高度，depthdata(2) 是外部参考高度/深度
Z = navstate.pos(3) - depthdata(2);
kf.Z = Z;

%% 2. 构造量测矩阵 H
% 仅观测天向位置 (状态量索引为 3)
H = zeros(1, kf.RANK);
H(1, 3) = 1; 

% 计算预测观测值 (对于高度观测，通常就是状态量中的高度误差)
kf.Zkk_1 = H * kf.x;

%% 3. 计算卡尔曼增益 K
R = kf.depthstd^2;
K_full = kf.P * H' / (H * kf.P * H' + R);

%% 4. 实施“解耦反馈”掩码 (Decoupled Feedback Mask)
% 核心逻辑：只允许增益流向天向位置和天向速度
% 假设状态量定义：1:3 为位置误差(L, lam, h)，4:6 为速度误差(vN, vE, vU)
% 如果你的速度索引不同，请修改 vel_u_idx
pos_u_idx = 3; 
vel_u_idx = 6; % 在大多数15维滤波器中，6是天向速度

mask = zeros(kf.RANK, 1);
mask(pos_u_idx) = 1; 
mask(vel_u_idx) = 1; 

% 强制截断增益
K = K_full .* mask;

%% 5. 更新状态量与协方差矩阵
% 更新状态量 (此时只有位置和速度的天向分量会被修改)
kf.x = kf.x + K * (Z - kf.Zkk_1);
% 更新协方差矩阵
% 使用 Joseph 形式保证 P 的正定性和对称性，这对高频更新尤为重要
I = eye(kf.RANK);
kf.P = (I - K * H) * kf.P * (I - K * H)' + K * R * K';

end