function kf = myRange_azi_pos_Update_Adaptive(navstate, Rangedata, depthdata, kf)
% 自适应测量更新函数 (Adaptive EKF Update)
% 核心思想：基于观测残差的马氏距离，动态调整观测噪声阵 R
% 既能利用好数据，又能抵抗异常值的干扰

global rangstd depstd posstd; % 全局基础噪声配置
param = Param();

%% 1. 数据解析与几何计算
bcn = Rangedata(4:6)'; % 信标位置
[rm, rn] = getRmRn(bcn(1), param);
h_earth = bcn(3);

% 计算相对距离向量 (NED系投影)
dN = (bcn(1) - navstate.pos(1)) * (rm + h_earth);
dE = (bcn(2) - navstate.pos(2)) * (rn + h_earth) * cos(navstate.pos(1));

% 计算预测的观测值 (Predicted Measurements)
HorizR2 = dN^2 + dE^2;
HorizR_cal = sqrt(HorizR2);

% 计算方位角 (Azimuth)
% 注意：确保 navstate.att(3) 是航向角 (Yaw)
azi_cal = pos2azimuth(-[dE,dN], [0,0], r2d(navstate.att(3))); 

% --- [警告] 关于位置观测 (pos_pos) ---
% 直接使用 Range+Azi 推算的 Position 再放入 KF 更新会导致“双重计数”(Double Counting)
% 因为 Range 和 Azi 已经在前两个维度被使用了。
% 建议：通常只用 [Range, Depth, Azi]。如果必须保留 Position，需给予极大的 R。
% 这里保留你的逻辑，但要注意由此带来的相关性风险。
pos_xy = calc_position_from_beacon([0,0], Rangedata(3), Rangedata(7), r2d(navstate.att(3)));
pos_rad1 = dxyz2pos([pos_xy,0], Rangedata(4:6)');

%% 2. 计算残差 (Innovation / Residuals)
res_range = HorizR_cal - Rangedata(3);
res_depth = navstate.pos(3) - depthdata(2);

res_azi = azi_cal - Rangedata(7);
res_azi = mod(res_azi + 180, 360) - 180; % 归一化到 [-180, 180]

res_pos_lat = navstate.pos(1) - pos_rad1(1);
res_pos_lon = navstate.pos(2) - pos_rad1(2);

% 组装残差向量 Z
Z = [res_range;
     res_depth;
     res_azi;
     res_pos_lat;
     res_pos_lon]; % 将Pos拆分为两维处理更清晰

kf.Z = Z; % 记录残差

%% 3. 构建观测矩阵 H (Jacobian)
H = zeros(5, kf.RANK);
deg_per_rad = 180 / pi;

% (1) Range H
H(1, 1) = (-dN / HorizR_cal) * (rm + h_earth);     
H(1, 2) = (-dE / HorizR_cal) * (rn + h_earth) * cos(navstate.pos(1));

% (2) Depth H
H(2, 3) = 1;

% (3) Azimuth H
% 注意：H(3,9) 对应姿态误差，你的状态定义如果是 [phi_N, phi_E, phi_D]，
% 且第9维是 phi_D (Heading Error)，系数应为 -1 (视具体坐标定义而定)
H(3, 1) = (dE / HorizR2) * (rm + h_earth) * deg_per_rad;
H(3, 2) = (-dN / HorizR2) * (rn + h_earth) * cos(navstate.pos(1)) * deg_per_rad;
H(3, 9) = -1 * deg_per_rad; 

% (4) Position H (Lat)
H(4, 1) = 1; 

% (5) Position H (Lon)
H(5, 2) = 1;

%% 4. 构建基础噪声矩阵 R_base
% 你的方位角自适应模型 (物理模型)
% 0度时精度高(0.1)，90度时精度低(0.5)
azistd_phys = 0.5 - abs(abs(azi_cal)-90)/90 * 0.4; 

% 基础标准差向量
sigma_base = [rangstd; 
              depstd; 
              azistd_phys; 
              posstd/(rm + h_earth); 
              posstd/((rn + h_earth) * cos(navstate.pos(1)))];

R_base = diag(sigma_base.^2);

%% 5. === 自适应抗差逻辑 (Adaptive Robust) ===
% 使用 IGG-III 方案或 简单的马氏距离膨胀

% 计算理论上的残差协方差 S = HPH' + R
S = H * kf.P * H' + R_base;

% 为了数值稳定性，取 S 的对角线元素计算标准化残差
% (也可以用完整的 inv(S)，但在高维时容易出错)
sqrt_S_diag = sqrt(diag(S)); 

% 阈值设置 (通常取 2.0 ~ 3.0 倍标准差)
k0 = 2.0; % 正常范围
k1 = 10.0; % 异常范围

adaptive_factor = ones(5, 1);

for i = 1:5
    % 标准化残差 (Standardized Residual)
    % 衡量当前残差相对于"预测不确定度"有多大
    std_res = abs(Z(i)) / sqrt_S_diag(i);
    
    if std_res <= k0
        % 正常误差范围内，信任该数据
        adaptive_factor(i) = 1;
    elseif std_res > k0 && std_res <= k1
        % 误差略大，怀疑有异常，降权 (增大R)
        % 膨胀因子：使得 R 变大，从而 K 变小
        adaptive_factor(i) = std_res / k0; 
        % 或者使用更激进的平方: (std_res / k0)^2
    else
        % 误差巨大，认为是粗差 (Outlier)，极大抑制
        adaptive_factor(i) = 10000; % 几乎切断该通道更新
        % 或者直接令 adaptive_factor(i) = inf;
    end
end

% 构造自适应 R 矩阵
% R_adaptive = R_base * diag(adaptive_factor.^2); 
% 注意：adaptive_factor 是乘在 sigma 上的还是 R 上的？
% 上面逻辑是乘在 R 上的膨胀系数。
% R_adaptive = diag((sigma_base .* adaptive_factor).^2);
R_adaptive = R_base;
%% 6. 执行 EKF 更新
% 使用调整后的 R_adaptive 计算卡尔曼增益
% 采用 Joseph 形式更新 P 以保证对称正定

try
    % 计算增益 K
    K = kf.P * H' / (H * kf.P * H' + R_adaptive);
    
    % 更新状态
    kf.x = kf.x + K * Z;
    
    % 更新协方差 (Joseph form)
    I_KH = eye(kf.RANK) - K * H;
    kf.P = I_KH * kf.P * I_KH' + K * R_adaptive * K';
    
catch ME
    warning('Matrix singularity in Adaptive Update. Skipping this step.');
    % 如果矩阵求逆失败，跳过更新，防止程序崩溃
end

end