%%
%% 1、 定义系统参数
clear
L = 3000; % 边长 L
H = 40; % 水听器的 z 坐标

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
estimated_positions_sd = zeros(length(trajectory_coords), 2); % 存储 [x_est, y_est]
estimated_positions_kf1 = zeros(length(trajectory_coords), 2); % 存储 [x_est, y_est]
ll=length(trajectory_coords);
time_vector_trj=1:1:ll;
%% 3、卡尔曼滤波器参数初始化
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

% 过程噪声协方差矩阵 Q, 通过加速度噪声来建模。
sigma_accel_noise = 0.008; % m/s^2, 假设的加速度噪声标准差
Q = [(dt^3/3)*sigma_accel_noise^2, 0, (dt^2/2)*sigma_accel_noise^2, 0;
    0, (dt^3/3)*sigma_accel_noise^2, 0, (dt^2/2)*sigma_accel_noise^2;
    (dt^2/2)*sigma_accel_noise^2, 0, dt*sigma_accel_noise^2, 0;
    0, (dt^2/2)*sigma_accel_noise^2, 0, dt*sigma_accel_noise^2];
%% 4、 解算
% 测距误差的方差 (sigma^2)
sigma_r = 4; % 标准差
sigma_c = 2;
for i = 1:ll
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
    %% 最小二乘解算
    % 3、构建矩阵
    % A 矩阵的构建与测量值无关，只与传感器几何有关
    A_matrix = zeros(N, 2);
    B_matrix = zeros(N, 1);
    for ii=1:N
        jj=mod(ii,N)+1;
        di_dj=norm(sensor_positions(jj,:))-norm(sensor_positions(ii,:));
        zi_zj=sensor_positions(jj,3)-sensor_positions(ii,3);
        A_matrix(ii, :)=sensor_positions(jj,1:2) - sensor_positions(ii,1:2);
        B_matrix(ii, :)= 0.5*(measured_ranges(ii)^2-measured_ranges(jj)^2+di_dj)-trajectory_coords(i,3)*zi_zj;
    end
    % 4. 计算估计位置和速度(使用最小二乘或伪逆)
    % X = (A^T A)^(-1) A^T B
    % 目标位置估计为 [x_est, y_est]
    estimated_pos_xy = (A_matrix' * A_matrix)^-1 * A_matrix' * B_matrix;
    estimated_positions(i, :) = estimated_pos_xy'; % 存储估计的 X, Y 坐标

    % 使用矩阵计算误差
    D_matrix = zeros(N, 1);
    for ii=1:N
        jj=mod(ii,N)+1;
        di_dj=norm(sensor_positions(jj,:))-norm(sensor_positions(ii,:));
        zi_zj=sensor_positions(jj,3)-sensor_positions(ii,3);
        D_matrix(ii, :)= 0.5*di_dj-trajectory_coords(i,3)*zi_zj;
    end
    % 构建 C 矩阵
    C_matrix = diag(ones(1,4));
    C_matrix = C_matrix + diag(-ones(N-1, 1), 1); % 第二个参数1表示上一次对角线
    C_matrix(N, 1) = -1;
    % --- 构建 Q 矩阵 ---
    Q_matrix = diag(measured_ranges);
    % --- 构建 DRR 矩阵 ---
    DRR = diag(ones(4,1) * sigma_r^2);
    % --- 计算 K 矩阵 ---
    K =  inv(A_matrix' * A_matrix) * A_matrix' * C_matrix * Q_matrix;
    % --- 计算 D_XX 矩阵 ---
    D_XX = K * DRR * K'; % K' 是 K 的转置
    rxy(i,:)=diag(D_XX);
    sigma_x_sq = D_XX(1,1);
    sigma_y_sq = D_XX(2,2);
    error(i) = sqrt(sigma_x_sq + sigma_y_sq);

    %% 牛顿梯度下降法
    % 初始猜测 XY
    if i == 1
        init_guess_xy = mean(sensor_positions(:,1:2),1);
    else
        init_guess_xy = estimated_positions1(i-1,1:2);
    end
    % 优化 XY，Z 固定
    options = optimoptions('lsqnonlin','Display','off','Algorithm','levenberg-marquardt');
    est_xy = lsqnonlin(@(xy) range_residuals_fixed_depth(xy, trajectory_coords(i,3), sensor_positions, measured_ranges),...
        init_guess_xy,[],[],options);
    estimated_positions1(i,:) = est_xy;

    %% 最大梯度下降法
    true_target_pos = trajectory_coords(i,:); % 当前轨迹点的真实位置
    % 初始猜测 XY
    if i == 1
        % 第一次迭代使用传感器中心作为初始猜测
        init_guess_xy = estimated_positions(i, :)'; % 列向量
    else
        % 后续迭代使用前一时刻的估计位置作为初始猜测
        init_guess_xy = estimated_positions_sd(i-1,1:2)'; % 列向量
    end
    % 最速下降法的优化参数
    sd_learning_rate = 0.02; % ！！关键参数：需要仔细调整，过大可能发散，过小收敛慢
    sd_max_iterations = 5000; % 最速下降法通常需要更多迭代
    sd_tolerance = 1e-8;     % 收敛容差
    % 调用最速下降法进行优化
    [est_xy, ~] = steepest_descent_for_localization(init_guess_xy, ...
        trajectory_coords(i,3), ... % 目标深度
        sensor_positions, ...
        measured_ranges, ...
        sd_learning_rate, ...
        sd_max_iterations, ...
        sd_tolerance);
    estimated_positions_sd(i,:) = est_xy'; % 存储估计的 X, Y 坐标 (转为行向量)

    %% KALMAN
    lbl_pos_measured(:, i) = estimated_positions(i,1:2);
    % -------------------------------------------------------------
    % 卡尔曼滤波器核心算法
    % -------------------------------------------------------------
    if i > 1
        R_kf = diag(rxy(i,:));
        % R_kf = [9,0;0,9];
        % 预测步
        x_pred = F * x_est(:, i-1);
        P_pred = F * P_est(:, :, i-1) * F' + Q;

        % 更新步
        z_k = lbl_pos_measured(:, i); % 当前LBL解算出的位置观测量

        y_k = z_k - H * x_pred; % 观测残差
        S_k = H * P_pred * H' + R_kf; % 观测残差协方差
        K_k = P_pred * H' * inv(S_k); % 卡尔曼增益

        x_est(:, i) = x_pred + K_k * y_k; % 状态更新
        P_est(:, :, i) = (eye(size(F)) - K_k * H) * P_pred; % 误差协方差更新
    end
    estimated_positions_kf1(i,:)=x_est(1:2, i)' ;
end
% %% 5、 单独卡尔曼
% % 预分配LBL解算结果存储
% lbl_pos_measured = zeros(2, ll);
% for k = 1:length(trajectory_coords)
%     % 为了简化，我们假设LBL解算器已经完成了距离到位置的转换和加权。
%     lbl_pos_measured(:, k) = estimated_positions(k,1:2);
%     % -------------------------------------------------------------
%     % 卡尔曼滤波器核心算法
%     % -------------------------------------------------------------
%     if k > 1
%         % R_kf = diag(rxy(k,:));
%         R_kf = [10,0;0,10];
%         % 预测步
%         x_pred = F * x_est(:, k-1);
%         P_pred = F * P_est(:, :, k-1) * F' + Q;
%
%         % 更新步
%         z_k = lbl_pos_measured(:, k); % 当前LBL解算出的位置观测量
%
%         y_k = z_k - H * x_pred; % 观测残差
%         S_k = H * P_pred * H' + R_kf; % 观测残差协方差
%         K_k = P_pred * H' * inv(S_k); % 卡尔曼增益
%         x_est(:, k) = x_pred + K_k * y_k; % 状态更新
%         P_est(:, :, k) = (eye(size(F)) - K_k * H) * P_pred; % 误差协方差更新
%     end
% end
% estimated_positions_kf=x_est(1:2,:)';
% fprintf('仿真和卡尔曼滤波完成。\n');
%% 6、 绘图
% 单个结果
close all
plottrj(trajectory_coords,estimated_positions,sensor_positions,time_vector_trj,sigma_r)
plottrj(trajectory_coords,estimated_positions1,sensor_positions,time_vector_trj,sigma_r)
plottrj(trajectory_coords,estimated_positions_sd,sensor_positions,time_vector_trj,sigma_r)
plottrj(trajectory_coords,estimated_positions_kf1,sensor_positions,time_vector_trj,sigma_r)
%% 多条轨迹对比以及误差参数打印
close all
legend1={'LS解算轨迹','GN解算轨迹','GD解算轨迹','KF解算轨迹'};
plot_compareTrajectories(trajectory_coords, estimated_positions, estimated_positions1,...
    estimated_positions_sd,estimated_positions_kf1, sensor_positions, time_vector_trj, sigma_r,legend1)
%% 位置差分计算速度
estimated_velocitys=[diff(estimated_positions(:,1)/dt),...
    diff(estimated_positions(:,2))/dt];
plotvel(estimated_velocitys,velocity_info,time_vector_trj,sigma_r);

estimated_velocitys=[diff(estimated_positions1(:,1)/dt),...
    diff(estimated_positions1(:,2))/dt];
plotvel(estimated_velocitys,velocity_info,time_vector_trj,sigma_r);

estimated_velocitys=[diff(estimated_positions_sd(:,1)/dt),...
    diff(estimated_positions_sd(:,2))/dt];
plotvel(estimated_velocitys,velocity_info,time_vector_trj,sigma_r);

estimated_velocity_kf = x_est(3:4,2:end)';
plotvel(estimated_velocity_kf,velocity_info,time_vector_trj,sigma_r);
%%
function res = range_residuals_fixed_depth(xy, known_depth, BCNddm, ranges)
% 将已知深度合并为完整位置
pos = [xy(:)', known_depth]; % 行向量
% 计算预测距离
predicted_ranges = sqrt(sum((BCNddm - pos).^2, 2));
% 残差
res = abs(predicted_ranges - ranges(:));
end