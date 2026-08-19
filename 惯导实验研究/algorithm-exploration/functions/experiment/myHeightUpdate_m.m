function kf = myHeightUpdate_m(navstate, depthdata, kf)
%MYHEIGHTUPDATE_M 米制NED误差状态的高度更新（仅更新kf，不执行反馈）。
% x(1:3)=[dN,dE,dD]，第三维是下向误差；名义高度仍为pos(3)。
% 新代码若需要同步反馈名义状态并为RTS提供闭环矩阵，请优先调用
% update_decoupled_height_m。

%% 1. 计算观测残差 (Innovation)
% navstate.pos(3)是高度，depthdata(2)是外部高度/深度观测。
Z = navstate.pos(3) - depthdata(2);
kf.Z = Z;

%% 2. 构造量测矩阵 H
% 高度残差对下向位置误差的偏导为-1。
H = zeros(1, kf.RANK);
H(1, 3) = -1; 

% 计算预测观测值。
kf.Zkk_1 = H * kf.x;

%% 3. 计算卡尔曼增益 K
R = kf.depthstd ^ 2;
K_full = kf.P * H' / (H * kf.P * H' + R);

%% 4. 实施“解耦反馈”掩码 (Decoupled Feedback Mask)
% 只允许增益流向下向位置和下向速度。
pos_u_idx = 3;
vel_u_idx = 6;

mask = zeros(kf.RANK, 1);
mask(pos_u_idx) = 1; 
mask(vel_u_idx) = 1; 

% 强制截断增益
K = K_full .* mask;
% K = K_full;
%% 5. 更新状态量与协方差矩阵
% 更新状态量 (此时只有位置和速度的天向分量会被修改)
kf.x = kf.x + K * (Z - kf.Zkk_1);
% 更新协方差矩阵
% 使用 Joseph 形式保证 P 的正定性和对称性，这对高频更新尤为重要
I = eye(kf.RANK);
kf.P = (I - K * H) * kf.P * (I - K * H)' + K * R * K';

end
