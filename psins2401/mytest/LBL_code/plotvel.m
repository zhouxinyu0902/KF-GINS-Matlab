function plotvel(estimated_velocity_info,velocity_info,time_vector_trj,sigma_r)
%% 6. 从估计位置中估计速度
% 使用数值微分 (有限差分法)
% % 估计的速度点数会比位置点数少一个
% estimated_vx = diff(estimated_positions(:,1)) / dt;
% estimated_vy = diff(estimated_positions(:,2)) / dt;
% estimated_vz = zeros(length(trajectory_coords) - 1, 1); % 假设Z方向速度为零
% 
% % estimated_velocity_info = [estimated_vx, estimated_vy, estimated_vz];
% estimated_velocity_info = [x_est(3:4,2:end)',estimated_vz];
% 用于对比的真实速度 (截取与估计速度相同长度)
true_velocity_for_comp = velocity_info(1:end-1, :);
% 用于速度误差图和速度对比图的时间向量 (比位置时间向量少一个点)
time_vector_vel_plots = time_vector_trj(1:end-1);

% 计算速度误差
velocity_errors_x = estimated_velocity_info(:,1) - true_velocity_for_comp(:,1);
velocity_errors_y = estimated_velocity_info(:,2)- true_velocity_for_comp(:,2);
velocity_error_magnitude = sqrt(velocity_errors_x.^2 + velocity_errors_y.^2);
%% 图3: 速度对比图
figure('Name', '速度对比图', 'Position', [100, 100, 800, 600]); % 新图窗的位置

% 子图1: X方向速度
subplot(2,1,1); % 2行1列的子图中的第1个
plot(time_vector_vel_plots, true_velocity_for_comp(:,1), 'b-', 'LineWidth', 1.5, 'DisplayName', '真实 Vx');
hold on;
plot(time_vector_vel_plots, estimated_velocity_info(:,1), 'r--', 'LineWidth', 1.0, 'DisplayName', '估计 Vx');
grid on;
xlabel('时间 (s)');
ylabel('X方向速度 (m/s)');
title('X方向速度：真实 vs 估计');
legend('show', 'Location', 'best');
hold off;

% 子图2: Y方向速度
subplot(2,1,2); % 2行1列的子图中的第2个
plot(time_vector_vel_plots, true_velocity_for_comp(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName', '真实 Vy');
hold on;
plot(time_vector_vel_plots, estimated_velocity_info(:,2), 'r--', 'LineWidth', 1.0, 'DisplayName', '估计 Vy');
grid on;
xlabel('时间 (s)');
ylabel('Y方向速度 (m/s)');
title('Y方向速度：真实 vs 估计');
legend('show', 'Location', 'best');
hold off;

sgtitle('速度对比图'); % 设置整个 Figure 的总标题
fprintf('速度对比图绘制完成。\n');

% 图4: 速度误差图
figure('Name', '速度误差图', 'Position', [950, 550, 600, 400]); % 新图窗的位置
plot(time_vector_vel_plots, velocity_error_magnitude, 'c-', 'LineWidth', 1.2, 'DisplayName', '水平速度误差');
hold on;
% 绘制平均速度误差线
mean_error_vel = nanmean(velocity_error_magnitude); % 使用nanmean忽略NaN值计算平均值
plot([time_vector_vel_plots(1), time_vector_vel_plots(end)], [mean_error_vel, mean_error_vel], 'k--', 'LineWidth', 1.0, 'DisplayName', sprintf('平均误差: %.4f m/s', mean_error_vel));
grid on;
xlabel('时间 (s)');
ylabel('水平速度误差 (m/s)');
title(sprintf('轨迹速度水平误差 (测距标准差 = %.2f m)', sigma_r));
legend('show', 'Location', 'best');
hold off;
xlim([time_vector_vel_plots(1), time_vector_vel_plots(end)]); % 设置X轴范围与速度时间向量匹配

fprintf('速度误差图绘制完成。\n');