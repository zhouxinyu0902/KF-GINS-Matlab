function plot_multi_vel(all_estimated_velocity_info, method_names, true_velocities, time_vector_trj, sigma_r)
%% plot_multi_vel_direct: 绘制四种解算方法直接估计的X、Y方向速度对比图和速度误差图。
%
%   输入:
%     all_estimated_velocity_info: cell 数组，每个 cell 包含一个方法的估计速度信息 (N x 3 或 (N-1) x 3)
%                                  例如: {estimated_velocity_info_LS, estimated_velocity_info_GN, ...}
%                                  其中 N 是时间点数，3 代表 [Vx, Vy, Vz]
%     method_names: cell 数组，包含每个方法的名称字符串，与 all_estimated_velocity_info 对应
%                   例如: {'LS', 'GN', 'GD', 'KF-LS'}
%     true_velocities: 真实的轨迹速度信息 (N x 3) [Vx, Vy, Vz]
%     time_vector_trj: 轨迹的时间向量 (N x 1)
%     sigma_r: 测距标准差，用于图标题显示

% 检查输入参数数量
if nargin < 5
    error('所有输入参数都必须提供：all_estimated_velocity_info, method_names, true_velocities, time_vector_trj, sigma_r');
end

num_methods = length(method_names);
if num_methods ~= length(all_estimated_velocity_info)
    error('方法名称的数量与估计速度信息的数量不匹配。');
end

% 确定用于绘图的时间向量和真实速度的长度。
% 注意：有些估计速度可能比真实速度少一个点（例如，如果它们是由位置差分得到的）。
% 为了统一，我们以最短的估计速度序列长度为准。
min_vel_len = length(time_vector_trj); % 初始设置为真实速度的长度
for i = 1:num_methods
    if size(all_estimated_velocity_info{i}, 1) < min_vel_len
        min_vel_len = size(all_estimated_velocity_info{i}, 1);
    end
end

% 截取用于对比的真实速度和时间向量
true_velocities_for_comp = true_velocities(1:min_vel_len, :);
time_vector_vel_plots = time_vector_trj(1:min_vel_len);


% 初始化存储速度误差指标的结构体
velocity_error_metrics = struct();
velocity_error_metrics.Method = cell(num_methods, 1);
velocity_error_metrics.RMSE_Vel = zeros(num_methods, 1);
velocity_error_metrics.ME_Vel = zeros(num_methods, 1);
velocity_error_metrics.STD_Vel = zeros(num_methods, 1);
velocity_error_metrics.MaxError_Vel = zeros(num_methods, 1);

% 预定义绘图颜色和线型
colors = {'r-', 'g--', 'm:', 'c-.', 'b:', 'k--'}; % 可以添加更多颜色以支持更多方法
if num_methods > length(colors)
    warning('预定义颜色不足以区分所有方法，将循环使用颜色。');
end

%% 1. 绘制速度对比图 (X方向和Y方向)
figure('Name', 'X/Y方向速度对比图', 'Position', [50, 50, 1000, 600]);

% 子图1: X方向速度
subplot(2,1,1);
plot(time_vector_vel_plots, true_velocities_for_comp(:,1), 'k-', 'LineWidth', 2, 'DisplayName', '真实 Vx');
hold on;
for i = 1:num_methods
    % 确保估计速度长度与时间向量匹配
    current_estimated_vx = all_estimated_velocity_info{i}(1:min_vel_len, 1);
    plot(time_vector_vel_plots, current_estimated_vx, ...
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
    % 确保估计速度长度与时间向量匹配
    current_estimated_vy = all_estimated_velocity_info{i}(1:min_vel_len, 2);
    plot(time_vector_vel_plots, current_estimated_vy, ...
         colors{mod(i-1, length(colors)) + 1}, 'LineWidth', 1.2, 'DisplayName', sprintf('估计 Vy (%s)', method_names{i}));
end
grid on;
xlabel('时间 (s)');
ylabel('Y方向速度 (m/s)');
title('Y方向速度：真实 vs 估计');
legend('show', 'Location', 'best');
hold off;
xlim([1400,2300])
% sgtitle(sprintf('四种解算方法的速度对比图 (直接估计, 测距标准差 = %.2f m)', sigma_r));
sgtitle('四种解算方法的速度对比图(直接估计)');
fprintf('X/Y方向速度对比图绘制完成。\n');


%% 2. 绘制水平速度误差图
figure('Name', '水平速度误差图', 'Position', [1300, 50, 350, 350]);
hold on;

for i = 1:num_methods
    current_estimated_vel = all_estimated_velocity_info{i}(1:min_vel_len, :); % 截取与时间向量匹配的长度

    % 计算水平速度误差
    velocity_errors_x = current_estimated_vel(:,1) - true_velocities_for_comp(:,1);
    velocity_errors_y = current_estimated_vel(:,2) - true_velocities_for_comp(:,2);
    velocity_error_magnitude = sqrt(velocity_errors_x.^2 + velocity_errors_y.^2);

    % 存储误差指标
    velocity_error_metrics.Method{i} = method_names{i};
    % RMSE：均方根误差
    velocity_error_metrics.RMSE_Vel(i) = sqrt(nanmean(velocity_error_magnitude.^2));
    % ME：平均绝对误差
    velocity_error_metrics.ME_Vel(i) = nanmean(abs(velocity_error_magnitude));
    % STD：标准差
    velocity_error_metrics.STD_Vel(i) = nanstd(velocity_error_magnitude);
    % Max Error：最大误差
    velocity_error_metrics.MaxError_Vel(i) = nanmax(velocity_error_magnitude);

    % 绘制每种方法的水平速度误差曲线
    plot(time_vector_vel_plots, velocity_error_magnitude, ...
         colors{mod(i-1, length(colors)) + 1}, 'LineWidth', 1.5, ...
         'DisplayName', sprintf('%s 水平速度误差', method_names{i}));

    % 绘制平均速度误差线
    mean_error_vel = nanmean(velocity_error_magnitude);
    plot([time_vector_vel_plots(1), time_vector_vel_plots(end)], [mean_error_vel, mean_error_vel], ...
         '--', 'Color', colors{mod(i-1, length(colors)) + 1}(1), 'LineWidth', 1.0, ...
         'DisplayName', sprintf('%s 平均误差: %.4f m/s', method_names{i}, mean_error_vel));
end

grid on;
xlabel('时间 (s)');
ylabel('水平速度误差 (m/s)');
title(sprintf('轨迹水平速度误差对比 (直接估计, 测距标准差 = %.2f m)', sigma_r));
legend('show', 'Location', 'best');
hold off;
xlim([time_vector_vel_plots(1), time_vector_vel_plots(end)]);
fprintf('水平速度误差图绘制完成。\n');

%% 3. 显示速度误差指标表格
fprintf('\n--- 速度误差指标 (直接估计) ---\n');
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