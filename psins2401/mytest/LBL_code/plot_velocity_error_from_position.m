function plot_velocity_error_from_position(all_estimated_positions, method_names, true_positions, true_velocities, time_vector_trj, sigma_r)
%% plot_velocity_error_from_position:
% 根据估计的水平位置和真实的水平位置，计算并绘制速度误差，并量化误差。
%
%   输入:
%     all_estimated_positions: cell 数组，每个 cell 包含一个方法的估计位置信息 (N x 3)
%                              例如: {estimated_positions_LS, estimated_positions_GN, ...}
%                              其中 N 是时间点数，3 代表 [X, Y, Z]
%     method_names: cell 数组，包含每个方法的名称字符串，与 all_estimated_positions 对应
%                   例如: {'LS', 'GN', 'GD', 'KF-LS'}
%     true_positions: 真实的轨迹位置信息 (N x 3) [X, Y, Z]
%     true_velocities: 真实的轨迹速度信息 (N x 3) [Vx, Vy, Vz]
%     time_vector_trj: 轨迹的时间向量 (N x 1)
%     sigma_r: 测距标准差，用于图标题显示

% 检查输入参数数量
if nargin < 6
    error('所有输入参数都必须提供：all_estimated_positions, method_names, true_positions, true_velocities, time_vector_trj, sigma_r');
end

num_methods = length(method_names);
if num_methods ~= length(all_estimated_positions)
    error('方法名称的数量与估计位置信息的数量不匹配。');
end

% 计算时间步长 dt
dt = time_vector_trj(2) - time_vector_trj(1);

% 初始化存储速度误差指标的结构体或数组
velocity_error_metrics = struct();
velocity_error_metrics.Method = cell(num_methods, 1);
velocity_error_metrics.RMSE_Vel = zeros(num_methods, 1);
velocity_error_metrics.ME_Vel = zeros(num_methods, 1);
velocity_error_metrics.STD_Vel = zeros(num_methods, 1);
velocity_error_metrics.MaxError_Vel = zeros(num_methods, 1);

% 预定义绘图颜色和线型，以便区分不同方法
colors = {'r-', 'g--', 'm:', 'c-.'}; % 红色实线, 绿色虚线, 品红点线, 青色点划线
if num_methods > length(colors)
    warning('预定义颜色不足以区分所有方法，将循环使用颜色。');
end

%% 1. 从估计位置计算估计速度
% 注意：通过位置差分计算的速度会比位置点少一个
time_vector_vel_plots = time_vector_trj(1:end-1);
true_velocities_for_comp = true_velocities(1:end-1, :); % 截取与估计速度相同的长度

all_derived_estimated_velocities = cell(1, num_methods); % 存储从位置导出的速度

for i = 1:num_methods
    current_estimated_positions = all_estimated_positions{i}; % Nx3 [X, Y, Z]

    % 计算X方向速度 (Vx)
    derived_vx = diff(current_estimated_positions(:,1)) / dt;
    % 计算Y方向速度 (Vy)
    derived_vy = diff(current_estimated_positions(:,2)) / dt;
    % 假设Z方向速度为零或者不考虑Z轴误差，因为通常更关注水平速度
    % 如果需要Z方向速度，可以从位置计算 diff(current_estimated_positions(:,3)) / dt;
    
    % 将Vx和Vy合并，Z方向速度可忽略或设为零，取决于实际需求
    all_derived_estimated_velocities{i} = [derived_vx, derived_vy]; % (N-1)x2
end


%% 2. 绘制速度对比图 (X方向和Y方向)
figure('Name', 'X/Y方向速度对比图', 'Position', [50, 50, 1000, 600]);

% 子图1: X方向速度
subplot(2,1,1);
plot(time_vector_vel_plots, true_velocities_for_comp(:,1), 'k-', 'LineWidth', 2, 'DisplayName', '真实 Vx');
hold on;
for i = 1:num_methods
    plot(time_vector_vel_plots, all_derived_estimated_velocities{i}(:,1), ...
         colors{mod(i-1, length(colors)) + 1}, 'LineWidth', 1.2, 'DisplayName', sprintf('估计 Vx (%s)', method_names{i}));
end
grid on;
xlabel('时间 (s)');
ylabel('X方向速度 (m/s)');
title('X方向速度：真实 vs 估计');
legend('show', 'Location', 'best');
hold off;
xlim([1400,2300])
% 子图2: Y方向速度
subplot(2,1,2);
plot(time_vector_vel_plots, true_velocities_for_comp(:,2), 'k-', 'LineWidth', 2, 'DisplayName', '真实 Vy');
hold on;
for i = 1:num_methods
    plot(time_vector_vel_plots, all_derived_estimated_velocities{i}(:,2), ...
         colors{mod(i-1, length(colors)) + 1}, 'LineWidth', 1.2, 'DisplayName', sprintf('估计 Vy (%s)', method_names{i}));
end
grid on;
xlabel('时间 (s)');
ylabel('Y方向速度 (m/s)');
title('Y方向速度：真实 vs 估计');
legend('show', 'Location', 'best');
hold off;
xlim([1400,2300])
% sgtitle(sprintf('四种解算方法的速度对比图 (基于位置差分, 测距标准差 = %.2f m)', sigma_r));
sgtitle('四种解算方法的速度对比图 (基于位置差分)');
fprintf('X/Y方向速度对比图绘制完成。\n');


%% 3. 绘制水平速度误差图
figure('Name', '水平速度误差图', 'Position', [1300, 50, 350, 350]);
hold on;

for i = 1:num_methods
    current_derived_vel = all_derived_estimated_velocities{i};

    % 计算水平速度误差
    velocity_errors_x = current_derived_vel(:,1) - true_velocities_for_comp(:,1);
    velocity_errors_y = current_derived_vel(:,2) - true_velocities_for_comp(:,2);
    velocity_error_magnitude = sqrt(velocity_errors_x.^2 + velocity_errors_y.^2);

    % 存储误差指标
    velocity_error_metrics.Method{i} = method_names{i};
    velocity_error_metrics.RMSE_Vel(i) = sqrt(nanmean(velocity_error_magnitude.^2));
    velocity_error_metrics.ME_Vel(i) = nanmean(abs(velocity_error_magnitude)); % 平均绝对误差
    velocity_error_metrics.STD_Vel(i) = nanstd(velocity_error_magnitude);
    velocity_error_metrics.MaxError_Vel(i) = nanmax(velocity_error_magnitude);

    % 绘制每种方法的水平速度误差曲线
    plot(time_vector_vel_plots, velocity_error_magnitude, ...
         colors{mod(i-1, length(colors)) + 1}, 'LineWidth', 1.5, ...
         'DisplayName', sprintf('%s 水平速度误差', method_names{i}));

    % 绘制平均速度误差线
    mean_error_vel = nanmean(velocity_error_magnitude);
    plot([time_vector_vel_plots(1), time_vector_vel_plots(end)], [mean_error_vel, mean_error_vel], ...
         '--', 'Color', colors{mod(i-1, length(colors)) + 1}(1), 'LineWidth', 1.0, ... % 使用曲线的第一个颜色字符作为虚线颜色
         'DisplayName', sprintf('%s 平均误差: %.4f m/s', method_names{i}, mean_error_vel));
end

grid on;
xlabel('时间 (s)');
ylabel('水平速度误差 (m/s)');
title(sprintf('轨迹水平速度误差对比 (基于位置差分, 测距标准差 = %.2f m)', sigma_r));
legend('show', 'Location', 'best');
hold off;
xlim([time_vector_vel_plots(1), time_vector_vel_plots(end)]); % 设置X轴范围与速度时间向量匹配
fprintf('水平速度误差图绘制完成。\n');

%% 4. 显示速度误差指标表格
fprintf('\n--- 速度误差指标 (基于位置差分) ---\n');
fprintf('%-10s %-10s %-10s %-10s %-12s\n', '方法', 'RMSE (m/s)', 'ME (m/s)', 'STD (m/s)', 'Max Error (m/s)');
for i = 1:num_methods
    fprintf('%-10s %-10.4f %-10.4f %-10.4f %-12.4f\n', ...
            velocity_error_metrics.Method{i}, ...
            velocity_error_metrics.RMSE_Vel(i), ...
            velocity_error_metrics.ME_Vel(i), ...
            velocity_error_metrics.STD_Vel(i), ...
            velocity_error_metrics.MaxError_Vel(i));
end
fprintf('------------------------------------\n');

end