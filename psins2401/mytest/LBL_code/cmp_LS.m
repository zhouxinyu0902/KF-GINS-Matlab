% saveAllFiguresAsPng
clear
%% 1、  定义系统参数
L = 3000; % 边长 L
H = L / 100; % 水听器的 z 坐标

% 四个水听器的位置 (x, y, z)
sensor_positions = [
    L/2,  L/2,  H;
    -L/2,  L/2,  H;
    L/2, -L/2,  H;
    -L/2, -L/2,  H
    ];
% sensor_positions = [
%     -L/2,  -L/2/sqrt(3),  H;
%     L/2,  -L/2/sqrt(3),  H;
%     0, L/sqrt(3),  H;
%     ];
N=size(sensor_positions,1);
%% 2、 生成轨迹
close all
dt=1;
rng(1)
% [trajectory_coords,velocity_info]=trjgen();
[trajectory_coords,velocity_info]=trjgen_with_acceleration();
plot_TrajectoryAndTransponders(trajectory_coords, sensor_positions)
estimated_positions = zeros(length(trajectory_coords), 2); % 存储 [x_est, y_est]
estimated_positions1 = zeros(length(trajectory_coords), 2); % 存储 [x_est, y_est]
ll=length(trajectory_coords);
time_vector_trj=1:1:ll;
%% 3、 卡尔曼滤波器参数初始化
sigma_lbl_pos = 3; % 米，LBL解算结果的位置标准差
R_kf = eye(2) * sigma_lbl_pos^2; % 卡尔曼滤波器的观测量噪声协方差矩阵
% 状态向量: [x; y; vx; vy]
% 维度: 4x1
x_est = zeros(4, ll);     % 估计的状态
P_est = zeros(4, 4, ll);  % 估计的协方差矩阵

% 初始状态估计 (从真实轨迹的起点稍微加点噪声作为初始值)
initial_pos_noise = 3; % 初始位置误差（m）
initial_vel_noise = 0.1; % 初始速度误差（m/s）
x_est(:, 1) = [trajectory_coords(1,1) + randn*initial_pos_noise; 
              trajectory_coords(1,2) + randn*initial_pos_noise;
               velocity_info(1,2)+ randn*initial_vel_noise;
               velocity_info(1,2) + randn*initial_vel_noise];

% 初始误差协方差矩阵
P_est(:, :, 1) = diag([10^2, 10^2, 1^2, 1^2]); % 对位置和速度的初始不确定性

% 状态转移矩阵 F
F = [1 0 dt 0;
     0 1 0 dt;
     0 0 1 0;
     0 0 0 1];

% 观测矩阵 H
% 观测量是位置 (x, y)，所以H只提取状态向量中的位置分量
H = [1 0 0 0;
     0 1 0 0];

% 过程噪声协方差矩阵 Q
% Q反映了模型的不确定性，即我们的匀速模型未能解释的运动部分。
% 可以通过加速度噪声来建模。
sigma_accel_noise = 0.008; % m/s^2, 假设的加速度噪声标准差
Q = [(dt^3/3)*sigma_accel_noise^2, 0, (dt^2/2)*sigma_accel_noise^2, 0;
     0, (dt^3/3)*sigma_accel_noise^2, 0, (dt^2/2)*sigma_accel_noise^2;
     (dt^2/2)*sigma_accel_noise^2, 0, dt*sigma_accel_noise^2, 0;
     0, (dt^2/2)*sigma_accel_noise^2, 0, dt*sigma_accel_noise^2];
%% 4、 解算
% 控制变量为测距误差
% 测距误差的方差 (sigma^2)
sigma_r = 4; % 标准差
sigma_c = 2;
sigma=[4,2,0.5,0.15];
for kk=1:4
    sigma_r=sigma(kk);
for i = 1:ll
    % 最小二乘解算
    % 1. 计算理论距离 (真实距离)
    true_ranges = zeros(N, 1);
    for j = 1:N
        true_ranges(j) = norm(trajectory_coords(i,:) - sensor_positions(j,:));
        % if j==1
        %     % true_ranges(j) = norm(trajectory_coords-poserr{:}- sensor_positions(j,:));
        %     true_ranges(j) = norm(trajectory_coords(i,:)-[-5,0,0]- sensor_positions(j,:));
        %     % true_ranges(j) = norm(true_target_pos- [-5,0,0] - sensor_positions(j,:));
        %     % true_ranges(j) = norm(true_target_pos- [0,5,0] - sensor_positions(j,:));
        %     % true_ranges(j) = norm(true_target_pos- [5/sqrt(2),5/sqrt(2),0] - sensor_positions(j,:));
        % else
        %     true_ranges(j) = norm(trajectory_coords(i,:) - sensor_positions(j,:));
        % end
    end
    % 2. 模拟测量值 (加入随机误差)
    % random_errors = randn(4, 1) * sigma_r; % 高斯分布随机误差
    % 如果是高斯分布，标准差是 sqrt(0.15)
    measured_ranges = true_ranges + randn(4, 1) * sigma_r + sigma_c;
    % measured_ranges = true_ranges + randn(N, 1) * sigma_r ;
    % measured_ranges = true_ranges ;
   
    % 3、构建矩阵
    % A 矩阵的构建与测量值无关，只与传感器几何有关
    A_matrix = zeros(N, 2);
    B_matrix = zeros(N, 1);
    D_matrix = zeros(N, 1);
    C_matrix = zeros(N, N);
    for ii=1:N
        jj=mod(ii,N)+1;
        di_dj=norm(sensor_positions(jj,:))-norm(sensor_positions(ii,:));
        zi_zj=sensor_positions(jj,3)-sensor_positions(ii,3);
        A_matrix(ii, :)=sensor_positions(jj,1:2) - sensor_positions(ii,1:2);
        B_matrix(ii, :)= 0.5*(measured_ranges(ii)^2-measured_ranges(jj)^2+di_dj)-trajectory_coords(i,3)*zi_zj;
        D_matrix(ii, :)= 0.5*di_dj-trajectory_coords(i,3)*zi_zj;
    end
    % 构建 C 矩阵
    C_matrix = C_matrix + diag(-ones(N-1, 1), 1); % 第二个参数1表示上一次对角线
    C_matrix(N, 1) = -1;
    % --- 构建 Q 矩阵 ---
    Q_matrix = diag(measured_ranges);
    % --- 构建 DRR 矩阵 ---
    % DRR 是对角矩阵，对角线元素为 sigma_ri^2
    DRR = diag(ones(4,1) * (sigma_r+0.2).^2);
    % --- 计算 K 矩阵 ---+0.2
    % K = (A^T A)^(-1) A^T C Q
    ATA_inv = inv(A_matrix' * A_matrix); % (A^T A)^(-1)
    K = ATA_inv * A_matrix' * C_matrix * Q_matrix;
    % --- 计算 D_XX 矩阵 ---
    D_XX = K * DRR * K'; % K' 是 K 的转置
    rxy(i,:)=diag(D_XX);

    % 4. 计算估计位置和速度(使用最小二乘或伪逆)
    % X = (A^T A)^(-1) A^T B
    % 目标位置估计为 [x_est, y_est]
    estimated_pos_xy = (A_matrix' * A_matrix)^-1 * A_matrix' * B_matrix;
    estimated_positions(i, :) = estimated_pos_xy'; % 存储估计的 X, Y 坐标
end
result{kk}=estimated_positions;
end
%% 多条轨迹对比以及误差参数打印
close all
legend={'测距随机标准差4m','测距随机标准差2m','测距随机标准差0.5m','测距随机标准差0.15m'};
compareTrajectories(trajectory_coords, result{1}, result{2},...
    result{3},result{4}, sensor_positions, time_vector_trj, sigma_r,legend)
%%
sigma_r = 4; % 标准差
sigma_c = 2;
err{1}=[-1,0,0;-1,0,0;-1,0,0;-1,0,0]*0;
err{2}=[-1,0,0;-1,0,0;-1,0,0;-1,0,0]*5;
err{3}=[-1,0,0;-1,0,0;-1,0,0;-1,0,0]*10;
err{4}=[1,1,0;-1,1,0;1,-1,0;-1,-1,0]*5/sqrt(2);
for kk=1:4
for i = 1:ll
    % 最小二乘解算
    % 1. 计算理论距离 (真实距离)
    true_ranges = zeros(N, 1);
    for j = 1:N
        true_ranges(j) = norm(trajectory_coords(i,:) -err{kk}(j,:)-sensor_positions(j,:));
    end
    % 2. 模拟测量值 (加入随机误差)
    % random_errors = randn(4, 1) * sigma_r; % 高斯分布随机误差
    % 如果是高斯分布，标准差是 sqrt(0.15)
    measured_ranges = true_ranges + randn(4, 1) * sigma_r + sigma_c;
    % measured_ranges = true_ranges + randn(N, 1) * sigma_r ;
    % measured_ranges = true_ranges ;
   
    % 3、构建矩阵
    % A 矩阵的构建与测量值无关，只与传感器几何有关
    A_matrix = zeros(N, 2);
    B_matrix = zeros(N, 1);
    D_matrix = zeros(N, 1);
    C_matrix = zeros(N, N);
    for ii=1:N
        jj=mod(ii,N)+1;
        di_dj=norm(sensor_positions(jj,:))-norm(sensor_positions(ii,:));
        zi_zj=sensor_positions(jj,3)-sensor_positions(ii,3);
        A_matrix(ii, :)=sensor_positions(jj,1:2) - sensor_positions(ii,1:2);
        B_matrix(ii, :)= 0.5*(measured_ranges(ii)^2-measured_ranges(jj)^2+di_dj)-trajectory_coords(i,3)*zi_zj;
        D_matrix(ii, :)= 0.5*di_dj-trajectory_coords(i,3)*zi_zj;
    end
    % 构建 C 矩阵
    C_matrix = C_matrix + diag(-ones(N-1, 1), 1); % 第二个参数1表示上一次对角线
    C_matrix(N, 1) = -1;
    % --- 构建 Q 矩阵 ---
    Q_matrix = diag(measured_ranges);
    % --- 构建 DRR 矩阵 ---
    % DRR 是对角矩阵，对角线元素为 sigma_ri^2
    DRR = diag(ones(4,1) * (sigma_r+0.2).^2);
    % --- 计算 K 矩阵 ---+0.2
    % K = (A^T A)^(-1) A^T C Q
    ATA_inv = inv(A_matrix' * A_matrix); % (A^T A)^(-1)
    K = ATA_inv * A_matrix' * C_matrix * Q_matrix;
    % --- 计算 D_XX 矩阵 ---
    D_XX = K * DRR * K'; % K' 是 K 的转置
    rxy(i,:)=diag(D_XX);

    % 4. 计算估计位置和速度(使用最小二乘或伪逆)
    % X = (A^T A)^(-1) A^T B
    % 目标位置估计为 [x_est, y_est]
    estimated_pos_xy = (A_matrix' * A_matrix)^-1 * A_matrix' * B_matrix;
    estimated_positions(i, :) = estimated_pos_xy'; % 存储估计的 X, Y 坐标
end
result11{kk}=estimated_positions;
end
%% 绘图
close all
plottrj(trajectory_coords,estimated_positions,sensor_positions,time_vector_trj,sigma_r)
%%
close all
legend={'往外扩散5m','往西偏10m','往西偏5m','不偏'};
compareTrajectories(trajectory_coords, result11{4}, result11{3},...
    result11{2},result11{1}, sensor_positions, time_vector_trj, sigma_r,legend)
%%
estimated_vz = zeros(length(trajectory_coords) - 1, 1); % 假设Z方向速度为零
estimated_velocity_info = [x_est(3:4,2:end)',estimated_vz];
plotvel(estimated_velocity_info,velocity_info,time_vector_trj,sigma_r);
