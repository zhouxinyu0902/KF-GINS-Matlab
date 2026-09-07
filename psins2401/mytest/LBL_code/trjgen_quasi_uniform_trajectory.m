function [trajectory_coords,velocity_info]=trjgen_quasi_uniform_trajectory()
% MATLAB 代码：生成斜向拟匀速直线轨迹，带微小加速度偏差，不偏离太多

% --- 固定随机数种子 ---
% rng(42); % 使用一个固定的种子，确保每次运行结果相同

% --- 轨迹参数定义 ---
start_pos = [1800, 1800, 30]; % 起点坐标 [X0, Y0, Z0]
end_pos = [0, 0, 30];         % 目标方向点/终点（如果时间足够） [Xf, Yf, Zf]
initial_speed_magnitude = 1;  % 初始速度大小 (m/s)
sample_time = 1;              % 采样时间间隔 (s)
acceleration_magnitude = 0.006; % 最大加速度大小 (m/s^2)，相比之前0.05略微减小，减少偏差
fprintf('正在生成带微小加速度偏差的拟匀速轨迹信息...\n');

% --- 设置总时间 ---

total_time_base = 3600; % 用户指定固定总时间
time_vector = 0:sample_time:total_time_base;
num_samples = length(time_vector); % 实际的采样点数量

% --- 初始化坐标和速度 ---
x_coords = zeros(1, num_samples);
y_coords = zeros(1, num_samples);
z_coords = repmat(start_pos(3), 1, num_samples);
vx_coords = zeros(1, num_samples);
vy_coords = zeros(1, num_samples);
vz_coords = zeros(1, num_samples); % Z方向速度始终为0

x_coords(1) = start_pos(1);
y_coords(1) = start_pos(2);

% 计算初始速度分量，使其指向 end_pos 方向
direction_vector = end_pos(1:2) - start_pos(1:2);
direction_norm = norm(direction_vector);

if direction_norm > 0
    initial_vx = (direction_vector(1) / direction_norm) * initial_speed_magnitude;
    initial_vy = (direction_vector(2) / direction_norm) * initial_speed_magnitude;
else % 如果起点和终点相同，则初始速度为0
    initial_vx = 0;
    initial_vy = 0;
end

vx_coords(1) = initial_vx;
vy_coords(1) = initial_vy;
vz_coords(1) = 0;

% --- 生成坐标点和速度信息 ---
for i = 2:num_samples
    % 生成随机加速度分量
    % rand()生成[0,1]的随机数，(2*rand()-1)生成[-1,1]的随机数
    ax = acceleration_magnitude * (2*rand() - 1);
    ay = acceleration_magnitude * (2*rand() - 1);
    
    % 计算当前时刻的速度
    vx_coords(i) = vx_coords(i-1) + ax * sample_time;
    vy_coords(i) = vy_coords(i-1) + ay * sample_time;
    
    % 限制速度大小，确保其在初始速度附近波动
    current_speed = norm([vx_coords(i), vy_coords(i)]);
    % 允许速度在初始速度的 0.8 到 1.2 倍之间波动，以实现“拟匀速”
    max_allowed_speed = initial_speed_magnitude * 1.2; 
    min_allowed_speed = initial_speed_magnitude * 0.8; 

    if current_speed > max_allowed_speed
        % 如果速度过快，按比例缩回最大允许速度
        vx_coords(i) = vx_coords(i) * (max_allowed_speed / current_speed);
        vy_coords(i) = vy_coords(i) * (max_allowed_speed / current_speed);
    elseif current_speed < min_allowed_speed && current_speed ~= 0
        % 如果速度过慢，按比例提升到最小允许速度
        vx_coords(i) = vx_coords(i) * (min_allowed_speed / current_speed);
        vy_coords(i) = vy_coords(i) * (min_allowed_speed / current_speed);
    elseif current_speed == 0 % 避免除以零
         vx_coords(i) = initial_vx; % 如果速度为0，则重置为初始方向速度
         vy_coords(i) = initial_vy;
    end

    % 计算当前时刻的坐标
    x_coords(i) = x_coords(i-1) + vx_coords(i-1) * sample_time + 0.5 * ax * sample_time^2;
    y_coords(i) = y_coords(i-1) + vy_coords(i-1) * sample_time + 0.5 * ay * sample_time^2;
end

% 将坐标点组织成一个 N x 3 的矩阵
trajectory_coords = [x_coords', y_coords', z_coords'];
% 将速度信息组织成一个 N x 3 的矩阵
velocity_info = [vx_coords', vy_coords', vz_coords'];

% --- 计算实际的总距离和时间 ---
actual_total_distance = sum(sqrt(diff(x_coords).^2 + diff(y_coords).^2));
actual_total_time = (num_samples - 1) * sample_time;

fprintf('带加速度偏差的拟匀速轨迹生成完成。\n');
fprintf('实际总距离: %.4f m\n', actual_total_distance);
fprintf('总仿真时间: %.4f s\n', total_time_base); % 这里的total_time_base就是仿真时间
fprintf('实际仿真时间: %.4f s\n', actual_total_time);
fprintf('采样点数量: %d\n', num_samples);
fprintf('初始X方向速度分量: %.4f m/s\n', vx_coords(1));
fprintf('初始Y方向速度分量: %.4f m/s\n', vy_coords(1));
fprintf('Z方向速度分量: %.4f m/s\n', 0);
fprintf('平均合成速度大小: %.4f m/s\n', actual_total_distance / actual_total_time);

% --- 绘制轨迹图 (2D 水平平面图) ---
figure('Position',[100,100,600,800]); % 调整图形窗口大小以容纳两个子图

subplot(2,1,1); % 创建第一个子图，2行1列中的第1个
plot(trajectory_coords(:,1), trajectory_coords(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName', '实际轨迹');
hold on;
plot(start_pos(1), start_pos(2), 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', '起点');
plot(end_pos(1), end_pos(2), 'rx', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', '目标方向点');
% 绘制理想直线轨迹作为参考
ideal_x = linspace(start_pos(1), end_pos(1), 100);
ideal_y = linspace(start_pos(2), end_pos(2), 100);
plot(ideal_x, ideal_y, 'k--', 'LineWidth', 1, 'DisplayName', '理想直线方向');
grid on;
axis equal; % 确保X和Y轴比例一致
xlabel('X 坐标 (m)');
ylabel('Y 坐标 (m)');
title('带微小加速度偏差的拟匀速航行轨迹 (XY 平面)');
legend('show', 'Location', 'best');
hold off;

% --- 绘制速度图 ---
subplot(2,1,2); % 创建第二个子图，2行1列中的第2个
time_axis = (0:num_samples-1) * sample_time;
plot(time_axis, velocity_info(:,1), 'r-', 'LineWidth', 1.5, 'DisplayName', 'X方向速度 (Vx)');
hold on;
plot(time_axis, velocity_info(:,2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Y方向速度 (Vy)');
plot(time_axis, sqrt(velocity_info(:,1).^2 + velocity_info(:,2).^2), 'b-', 'LineWidth', 1.5, 'DisplayName', '合成速度大小');
% 绘制理想匀速线作为参考
plot(time_axis, repmat(initial_speed_magnitude, 1, num_samples), 'k--', 'LineWidth', 1, 'DisplayName', '理想匀速');
grid on;
xlabel('时间 (s)');
ylabel('速度 (m/s)');
title('速度分量及合成速度大小随时间变化');
legend('show', 'Location', 'best');
hold off;

end