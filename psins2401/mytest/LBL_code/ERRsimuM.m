% 用于生成二维定位误差分布图（含蒙特卡洛仿真）
clear
%% 1. 定义系统参数
L = 5000; % 边长 L
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
% 测距误差的方差 (sigma^2)
sigma_r = 0.15; % 标准差
num_simulations = 100; % 每个网格点进行 100 次独立统计
%% 2. 定义空间网格
LL = 4000;
x_min = -LL; % 米
x_max = LL;  % 米
y_min = -LL; % 米
y_max = LL;  % 米

grid_resolution = 50; % 米 (调整以获得更精细或更粗糙的图)
x_grid_vals = x_min:grid_resolution:x_max;
y_grid_vals = y_min:grid_resolution:y_max;

[X_grid, Y_grid] = meshgrid(x_grid_vals, y_grid_vals);

% 初始化矩阵以存储定位误差值
localization_error_map = zeros(size(X_grid));
localization_error_map1 = zeros(size(X_grid));
% for sigma_r=[0.15,0.5,1,2]
% for poserr={[-5,0,0],[5,0,0],[0,5,0],[5,5,0]/sqrt(2)}
%% 3. 在每个网格点上进行蒙特卡洛仿真计算定位误差
fprintf('开始进行蒙特卡洛仿真，总共 %d 个网格点，每个点 %d 次仿真...\n', numel(X_grid), num_simulations);
for i = 1:numel(X_grid)
    true_target_x = X_grid(i);
    true_target_y = Y_grid(i);
    true_target_z = 30; % 目标的真实 Z 坐标为 30m
    true_target_pos = [true_target_x, true_target_y, true_target_z];
    % 存储每次仿真得到的估计位置，用于后续统计
    estimated_positions_at_current_grid_point = zeros(num_simulations, 2); % 存储 [x_est, y_est]
    for k = 1:num_simulations
        % 1. 计算理论距离 (真实距离)
        true_ranges = zeros(N, 1);
        for j = 1:N
            true_ranges(j) = norm(true_target_pos - sensor_positions(j,:));
            % if j==1 
            %     true_ranges(j) = norm(true_target_pos-poserr{:}- sensor_positions(j,:));
            %     % true_ranges(j) = norm(true_target_pos-[-5,0,0]- sensor_positions(j,:));
            %     % true_ranges(j) = norm(true_target_pos- [-5,0,0] - sensor_positions(j,:));
            %     % true_ranges(j) = norm(true_target_pos- [0,5,0] - sensor_positions(j,:));
            %     % true_ranges(j) = norm(true_target_pos- [5/sqrt(2),5/sqrt(2),0] - sensor_positions(j,:));
            % else
            %     true_ranges(j) = norm(true_target_pos - sensor_positions(j,:)); 
            % end
        end

        % 2. 模拟测量值 (加入随机误差)
        % random_errors = randn(4, 1) * sigma_r; % 高斯分布随机误差
        % 如果是高斯分布，标准差是 sqrt(0.15)
        % measured_ranges = true_ranges + randn(4, 1) * sigma_r + true_ranges/1500*0.05;
        measured_ranges = true_ranges + randn(N, 1) * sigma_r ;
        % measured_ranges = true_ranges ;
        % A 矩阵的构建与测量值无关，只与传感器几何有关
        A_matrix = zeros(N, 2);
        B_matrix = zeros(N, 1);
        for ii=1:N
            jj=mod(ii,N)+1;
            di_dj=norm(sensor_positions(jj,:))-norm(sensor_positions(ii,:));
            zi_zj=sensor_positions(jj,3)-sensor_positions(ii,3);
            A_matrix(ii, :)=sensor_positions(jj,1:2) - sensor_positions(ii,1:2);
            B_matrix(ii, :)= 0.5*(measured_ranges(ii)^2-measured_ranges(jj)^2+di_dj)-true_target_z*zi_zj;
        end
        % 4. 计算估计位置 (使用最小二乘或伪逆)
        % X = (A^T A)^(-1) A^T B
        % 目标位置估计为 [x_est, y_est]
        estimated_pos_xy = (A_matrix' * A_matrix)^-1 * A_matrix' * B_matrix;
        estimated_positions_at_current_grid_point(k, :) = estimated_pos_xy'; % 存储估计的 X, Y 坐标
        
        % 使用公式计算误差
        D = zeros(N, 1);
        for ii=1:N
            jj=mod(ii,N)+1;
            di_dj=norm(sensor_positions(jj,:))-norm(sensor_positions(ii,:));
            zi_zj=sensor_positions(jj,3)-sensor_positions(ii,3);
            D(ii, :)= 0.5*di_dj-true_target_z*zi_zj;
        end
        % 构建 C 矩阵
        C_matrix = diag(ones(1,4));
        C_matrix = C_matrix + diag(-ones(N-1, 1), 1); % 第二个参数1表示上一次对角线
        C_matrix(N, 1) = -1;
        % --- 构建 Q 矩阵 ---
        Q = diag(measured_ranges);
        % --- 构建 DRR 矩阵 ---
        DRR = diag(ones(4,1) * sigma_r^2);
        % --- 计算 K 矩阵 ---
        K =  inv(A_matrix' * A_matrix) * A_matrix' * C_matrix * Q;
        % --- 计算 D_XX 矩阵 ---
        D_XX = K * DRR * K'; % K' 是 K 的转置
        sigma_x_sq = D_XX(1,1);
        sigma_y_sq = D_XX(2,2);
        localization_error_map(k) = sqrt(sigma_x_sq + sigma_y_sq);
        
    end % 结束 100 次仿真循环

    % 5. 统计定位误差
    % 计算 100 次估计位置与真实位置的偏差
    errors_x = estimated_positions_at_current_grid_point(:, 1) - true_target_x;
    errors_y = estimated_positions_at_current_grid_point(:, 2) - true_target_y;

    % 计算每次仿真的水平误差
    horizontal_errors = sqrt(errors_x.^2 + errors_y.^2);

    % 取这 100 次水平误差的平均值作为该网格点的定位精度
    localization_error_map(i) = mean(horizontal_errors);
    localization_error_map1(i) = mean(localization_error_map(k));
    % 或者，也可以计算 100 次估计位置的 RMSE (Root Mean Square Error)
    % rmse_at_point = sqrt(mean(errors_x.^2 + errors_y.^2));
    % localization_error_map(i) = rmse_at_point;

    % % 打印进度 (可选)
    % if mod(i, 100) == 0 || i == numel(X_grid)
    %     fprintf('  已处理 %d/%d 个网格点...\n', i, numel(X_grid));
    % end
end % 结束网格点循环
fprintf('蒙特卡洛仿真完成。\n');
%% 4. 绘制定位误差图 (使用 contourf)
plot_contouf(localization_error_map,sensor_positions,X_grid, Y_grid,sigma_r,L)
plot_contouf(localization_error_map1,sensor_positions,X_grid, Y_grid,sigma_r,L)
% end