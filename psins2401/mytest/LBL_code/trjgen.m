% MATLAB 代码：生成水平匀速直线轨迹
function [trajectory_coords,velocity_info]=trjgen()

% MATLAB 代码：生成斜向匀速直线轨迹

% --- 轨迹参数定义 ---
start_pos = [1800, 1800, 30]; % 起点坐标 [X0, Y0, Z0]
end_pos = [0, 0, 30]; % 终点坐标 [Xf, Yf, Zf]
speed_magnitude = 1;         % 速度大小 (m/s)
sample_time = 1;             % 采样时间间隔 (s)

fprintf('正在生成斜向轨迹信息...\n');

% --- 计算轨迹总距离 ---
delta_x = end_pos(1) - start_pos(1);
delta_y = end_pos(2) - start_pos(2);
total_distance = sqrt(delta_x^2 + delta_y^2);

% --- 计算总时间 ---
total_time = total_distance / speed_magnitude;

% --- 生成时间序列 ---
% linspace 可以确保起点和终点准确包含，并均匀分布
time_vector = 0:sample_time:total_time;
num_samples = length(time_vector); % 实际的采样点数量

% --- 生成坐标点信息 ---
% X 坐标：线性插值
x_coords = start_pos(1) + (delta_x / total_time) * time_vector;
% Y 坐标：线性插值
y_coords = start_pos(2) + (delta_y / total_time) * time_vector;
% Z 坐标：始终保持为起点 Z
z_coords = repmat(start_pos(3), 1, num_samples);

% 将坐标点组织成一个 N x 3 的矩阵
trajectory_coords = [x_coords', y_coords', z_coords'];

% --- 生成速度信息 ---
% 计算 X 和 Y 方向的匀速分量
vx = delta_x / total_time;
vy = delta_y / total_time;
vz = 0; % Z 方向速度为 0

% 将速度信息组织成一个 N x 3 的矩阵 (每行相同)
velocity_info = repmat([vx, vy, vz], num_samples, 1);

fprintf('斜向轨迹生成完成。\n');
fprintf('总距离: %.4f m\n', total_distance);
fprintf('总时间: %.4f s\n', total_time);
fprintf('采样点数量: %d\n', num_samples);
fprintf('X方向速度分量: %.4f m/s\n', vx);
fprintf('Y方向速度分量: %.4f m/s\n', vy);
fprintf('Z方向速度分量: %.4f m/s\n', vz);
fprintf('合成速度大小: %.4f m/s (应与输入速度大小一致)\n', norm([vx, vy, vz]));
% 
% % --- 显示部分结果 (可选) ---
% disp('前5个坐标点 (X, Y, Z):');
% disp(trajectory_coords(1:min(5, num_samples), :));
% 
% disp('后5个坐标点 (X, Y, Z):');
% disp(trajectory_coords(max(1, num_samples-4):num_samples, :));
% 
% disp('速度信息 (前5个) (X, Y, Z):');
% disp(velocity_info(1:min(5, num_samples), :));

% --- 绘制轨迹图 (2D 水平平面图) ---
figure('Position',[100,100,500,500]);
plot(trajectory_coords(:,1), trajectory_coords(:,2), 'b-', 'LineWidth', 1.5);
hold on;
plot(start_pos(1), start_pos(2), 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', '起点');
plot(end_pos(1), end_pos(2), 'rx', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', '终点');
grid on;
axis equal; % 确保X和Y轴比例一致
xlabel('X 坐标 (m)');
ylabel('Y 坐标 (m)');
title('斜向匀速直线航行轨迹 (XY 平面)');
legend('show', 'Location', 'best');
hold off;
% 
% % --- 绘制轨迹图 (3D 图) ---
% figure;
% plot3(trajectory_coords(:,1), trajectory_coords(:,2), trajectory_coords(:,3), 'b-', 'LineWidth', 1.5);
% hold on;
% plot3(start_pos(1), start_pos(2), start_pos(3), 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', '起点');
% plot3(end_pos(1), end_pos(2), end_pos(3), 'rx', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', '终点');
% grid on;
% xlabel('X 坐标 (m)');
% ylabel('Y 坐标 (m)');
% zlabel('Z 坐标 (m)');
% title('斜向匀速直线航行轨迹 (3D)');
% legend('show', 'Location', 'best');
% view(3); % 3D 视角
% hold off;