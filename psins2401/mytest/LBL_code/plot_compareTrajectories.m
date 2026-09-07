function plot_compareTrajectories(trajectory_coords, estimated_positions, estimated_positions1,...
    estimated_positions_sd, estimated_positions_new, sensor_positions, time_vector_trj, sigma_r,legend1)
% compareTrajectories: 对比多条解算轨迹与真实轨迹，并计算误差参数。
%
%   输入:
%     trajectory_coords      : N x 3 矩阵，真实轨迹的 (X, Y, Z) 坐标。
%     estimated_positions    : N x 3 矩阵，第一条解算轨迹 (LS) 的 (X, Y, Z) 坐标。
%     estimated_positions1   : N x 3 矩阵，第二条解算轨迹 (GN) 的 (X, Y, Z) 坐标。
%     estimated_positions_sd : N x 3 矩阵，第三条解算轨迹 (GD) 的 (X, Y, Z) 坐标。
%     estimated_positions_new: N x 3 矩阵，第四条解算轨迹的 (X, Y, Z) 坐标。
%     sensor_positions       : M x 3 矩阵，M个LBL信标的 (X, Y, Z) 坐标。
%     time_vector_trj        : 1 x N 向量，轨迹对应的时间序列。
%     sigma_r                : 标量，距离测量噪声的标准差 (用于误差计算说明)。

% 确保所有轨迹的长度一致，以进行逐点比较
num_samples = size(trajectory_coords, 1);
if size(estimated_positions, 1) ~= num_samples || ...
   size(estimated_positions1, 1) ~= num_samples || ...
   size(estimated_positions_sd, 1) ~= num_samples || ...
   size(estimated_positions_new, 1) ~= num_samples
    error('所有轨迹的采样点数量必须一致！');
end

% --- 绘制所有轨迹和信标对比图 ---
figure('Position',[100,100,800,400]); % 调整图形窗口大小，为两个子图留出空间

% --- 子图 1: 轨迹对比图 ---
subplot(1, 2, 1); % 1行2列的子图，当前是第1个

% 绘制真实轨迹
plot(trajectory_coords(:,1), trajectory_coords(:,2), 'k-', 'LineWidth', 2, 'DisplayName', '真实轨迹');
hold on;

% 绘制四条解算轨迹
plot(estimated_positions(:,1), estimated_positions(:,2), 'r--', 'LineWidth', 1.2, 'DisplayName', legend1{1});
plot(estimated_positions1(:,1), estimated_positions1(:,2), 'g-.', 'LineWidth', 1.2, 'DisplayName', legend1{2});
plot(estimated_positions_sd(:,1), estimated_positions_sd(:,2), 'c:', 'LineWidth', 1.2, 'DisplayName', legend1{3});
plot(estimated_positions_new(:,1), estimated_positions_new(:,2), 'b-', 'LineWidth', 1.2, 'DisplayName', legend1{4}); % 新增的轨迹

% 绘制起点和终点（或轨迹结束点）
plot(trajectory_coords(1,1), trajectory_coords(1,2), 'ko', 'MarkerSize', 7, 'LineWidth', 1.5, 'DisplayName', '轨迹起点');
plot(trajectory_coords(end,1), trajectory_coords(end,2), 'kx', 'MarkerSize', 7, 'LineWidth', 1.5, 'DisplayName', '轨迹终点');
axis([480,580,860,960])
% % 绘制信标
% if ~isempty(sensor_positions)
%     plot(sensor_positions(:,1), sensor_positions(:,2), 'ms', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'LBL信标');
%     % 为每个信标添加编号
%     for k = 1:size(sensor_positions, 1)
%         text(sensor_positions(k,1) + 20, sensor_positions(k,2) + 20, sprintf('T%d', k), 'FontSize', 10, 'Color', 'm');
%     end
% end

% 设置图表属性
grid on;
axis equal;
xlabel('X 坐标 (m)');
ylabel('Y 坐标 (m)');
title(sprintf('多轨迹对比(距离噪声标准差: %.2f m)', sigma_r));
% title('多轨迹对比');
legend('show', 'Location', 'best'); % 图例放在外部，避免遮挡
hold off;

% --- 子图 2: 误差随时间变化小图 ---
subplot(1, 2, 2); % 1行2列的子图，当前是第2个

% 计算每条解算轨迹的平面位置误差
error_ls = sqrt((estimated_positions(:,1) - trajectory_coords(:,1)).^2 + ...
                (estimated_positions(:,2) - trajectory_coords(:,2)).^2);
error_gn = sqrt((estimated_positions1(:,1) - trajectory_coords(:,1)).^2 + ...
                (estimated_positions1(:,2) - trajectory_coords(:,2)).^2);
error_gd = sqrt((estimated_positions_sd(:,1) - trajectory_coords(:,1)).^2 + ...
                (estimated_positions_sd(:,2) - trajectory_coords(:,2)).^2);
error_new = sqrt((estimated_positions_new(:,1) - trajectory_coords(:,1)).^2 + ...
                 (estimated_positions_new(:,2) - trajectory_coords(:,2)).^2); % 新增误差

% 绘制误差曲线
plot(time_vector_trj, error_ls, 'r--', 'LineWidth', 1, 'DisplayName', legend1{1});
hold on;
plot(time_vector_trj, error_gn, 'g-.', 'LineWidth', 1, 'DisplayName', legend1{2});
plot(time_vector_trj, error_gd, 'c:', 'LineWidth', 1, 'DisplayName', legend1{3});
plot(time_vector_trj, error_new, 'b-', 'LineWidth', 1, 'DisplayName', legend1{4}); % 新增误差曲线
xlim([0,3600])
% 设置误差图表属性
grid on;
xlabel('时间 (s)');
ylabel('平面位置误差 (m)');
title('平面位置误差随时间变化');
legend('show', 'Location', 'best');
hold off;

% --- 计算并显示误差参数 ---
fprintf('\n--- 轨迹误差参数对比 (距离噪声标准差: %.2f m) ---\n', sigma_r);

% 误差计算函数 (局部函数，方便复用)
function [rmse, me, std_err, max_err] = calculate_errors(true_traj, estimated_traj)
    % 计算每个点的2D位置误差
    pos_error_x = estimated_traj(:,1) - true_traj(:,1);
    pos_error_y = estimated_traj(:,2) - true_traj(:,2);
    
    % 合成平面位置误差（欧几里得距离）
    total_pos_error = sqrt(pos_error_x.^2 + pos_error_y.^2);
    
    % 均方根误差 (RMSE)
    rmse = sqrt(mean(total_pos_error.^2));
    
    % 平均误差 (ME)
    me = mean(total_pos_error);
    
    % 误差标准差 (STD)
    std_err = std(total_pos_error);
    
    % 最大绝对误差 (Max Error)
    max_err = max(total_pos_error);
end

% 计算并显示第一条解算轨迹 (LS) 的误差
fprintf(['\n---' ,legend1{1}, '误差','---\n']);
[rmse_ls, me_ls, std_err_ls, max_err_ls] = calculate_errors(trajectory_coords, estimated_positions);
fprintf('  RMSE: %.4f m\n', rmse_ls);
fprintf('  ME:   %.4f m\n', me_ls);
fprintf('  STD:  %.4f m\n', std_err_ls);
fprintf('  Max Error: %.4f m\n', max_err_ls);

% 计算并显示第二条解算轨迹 (GN) 的误差
fprintf(['\n---' ,legend1{2}, '误差','---\n']);
[rmse_gn, me_gn, std_err_gn, max_err_gn] = calculate_errors(trajectory_coords, estimated_positions1);
fprintf('  RMSE: %.4f m\n', rmse_gn);
fprintf('  ME:   %.4f m\n', me_gn);
fprintf('  STD:  %.4f m\n', std_err_gn);
fprintf('  Max Error: %.4f m\n', max_err_gn);

% 计算并显示第三条解算轨迹 (GD) 的误差
fprintf(['\n---' ,legend1{3}, '误差','---\n']);
[rmse_gd, me_gd, std_err_gd, max_err_gd] = calculate_errors(trajectory_coords, estimated_positions_sd);
fprintf('  RMSE: %.4f m\n', rmse_gd);
fprintf('  ME:   %.4f m\n', me_gd);
fprintf('  STD:  %.4f m\n', std_err_gd);
fprintf('  Max Error: %.4f m\n', max_err_gd);

% 计算并显示第四条新解算轨迹的误差
fprintf(['\n---' ,legend1{4}, '误差','---\n']);
[rmse_new, me_new, std_err_new, max_err_new] = calculate_errors(trajectory_coords, estimated_positions_new);
fprintf('  RMSE: %.4f m\n', rmse_new);
fprintf('  ME:   %.4f m\n', me_new);
fprintf('  STD:  %.4f m\n', std_err_new);
fprintf('  Max Error: %.4f m\n', max_err_new);

end