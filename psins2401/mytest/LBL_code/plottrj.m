function plottrj(trajectory_coords,estimated_positions,sensor_positions,time_vector_trj,sigma_r)
%% 图1: 轨迹对比图 (主图带局部放大子图)
% myfigurestartup(10,10,'prese') % 此行是自定义函数，未提供定义，故注释掉
figure('Name', '轨迹对比图与局部放大','Position',[100,100,700,700]); % 设置图窗在屏幕上的位置和大小

% 创建主坐标轴 (占据图窗的大部分区域)
main_ax = axes('Position', [0.1, 0.1, 0.8, 0.8]); % [左边距, 下边距, 宽度, 高度] (相对于 Figure 的比例)
plot(main_ax, trajectory_coords(:,1), trajectory_coords(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName', '真实轨迹');
hold(main_ax, 'on'); % 保持当前坐标轴，以便在其上继续绘图
plot(main_ax, estimated_positions(:,1), estimated_positions(:,2), 'r--', 'LineWidth', 1.0, 'DisplayName', '估计轨迹');

% 绘制起点和终点
plot(main_ax, trajectory_coords(1,1), trajectory_coords(1,2), 'go', 'MarkerSize', 7, 'LineWidth', 2, 'DisplayName', '起点');
plot(main_ax, trajectory_coords(end,1), trajectory_coords(end,2), 'rx', 'MarkerSize', 7, 'LineWidth', 2, 'DisplayName', '终点');

% 绘制传感器位置
plot(main_ax, sensor_positions(:,1), sensor_positions(:,2), 'k^', 'MarkerSize', 8, 'LineWidth', 1.5, 'DisplayName', '传感器');
text(main_ax, sensor_positions(1,1)+50, sensor_positions(1,2)+50, 'S1', 'Color', 'k', 'FontSize', 9);
text(main_ax, sensor_positions(2,1)-150, sensor_positions(2,2)+50, 'S2', 'Color', 'k', 'FontSize', 9);
text(main_ax, sensor_positions(3,1)+50, sensor_positions(3,2)-50, 'S3', 'Color', 'k', 'FontSize', 9);
text(main_ax, sensor_positions(4,1)-150, sensor_positions(4,2)-50, 'S4', 'Color', 'k', 'FontSize', 9);

axis(main_ax, 'equal'); % 确保X和Y轴的比例尺一致
grid(main_ax, 'on');     % 显示网格线
xlabel(main_ax, 'X 坐标 (m)'); % X轴标签
ylabel(main_ax, 'Y 坐标 (m)'); % Y轴标签
title(main_ax, '真实轨迹与估计轨迹对比'); % 主图标题
legend(main_ax, 'show', 'Location', 'west'); % 显示图例
hold(main_ax, 'off');    % 释放当前坐标轴
axis([-3000,3000,-3000,3000])

% --- 创建局部放大坐标轴 (右下角的小图) ---
% 定义局部放大区域的坐标范围
zoom_x_min = 970; 
zoom_x_max = 995;
zoom_y_min = 1180;
zoom_y_max = 1190;

% 定义子图在主图中的位置和大小 (相对坐标 0到1)
zoom_ax_width = 0.2; % 子图相对宽度
zoom_ax_height = 0.2; % 子图相对高度
zoom_ax_left = 1 - zoom_ax_width - 0.1; % 子图左边缘距Figure右边缘0.1的距离
zoom_ax_bottom = 0.2; % 子图底边缘距Figure底边缘0.2的距离

zoom_ax = axes('Position', [zoom_ax_left, zoom_ax_bottom, zoom_ax_width, zoom_ax_height]);
plot(zoom_ax, trajectory_coords(:,1), trajectory_coords(:,2), 'b-', 'LineWidth', 1.0); % 小图中的线宽可以细一些
hold(zoom_ax, 'on');
plot(zoom_ax, estimated_positions(:,1), estimated_positions(:,2), 'r.-', 'LineWidth', 0.8);

% 绘制传感器位置 (在小图中也显示，如果落在范围内)
plot(zoom_ax, sensor_positions(:,1), sensor_positions(:,2), 'k^', 'MarkerSize', 6, 'LineWidth', 1.0);

% 设置小图的轴范围
xlim(zoom_ax, [zoom_x_min, zoom_x_max]);
ylim(zoom_ax, [zoom_y_min, zoom_y_max]);
% axis(zoom_ax, 'equal'); % 确保X和Y轴的比例尺一致
grid(zoom_ax, 'on');     % 显示网格线
title(zoom_ax, '局部放大 (X: 0-20m, Y: 0-20m)'); % 小图标题更明确
xlabel(zoom_ax, 'X (m)'); % 简化小图的X轴标签
ylabel(zoom_ax, 'Y (m)'); % 简化小图的Y轴标签
box(zoom_ax, 'on');      % 为小图添加边框
set(zoom_ax, 'FontSize', 8); % 调整小图的字体大小
hold(zoom_ax, 'off');

fprintf('轨迹对比图及局部放大图绘制完成。\n');

%% 图2: 定位误差对比图

% dt=time_vector_trj(2)-time_vector_trj(1);
horizontal_localization_errors=sqrt((estimated_positions(:,1)-trajectory_coords(:,1)).^2+...
    (estimated_positions(:,2)-trajectory_coords(:,2)).^2);


figure('Name', '定位误差对比图', 'Position', [950, 100, 600, 400]); % 新图窗的位置
plot(time_vector_trj, horizontal_localization_errors, '-', 'LineWidth', 1.2, 'DisplayName', '水平定位误差');
hold on;
% 绘制平均误差线
mean_error_pos = nanmean(horizontal_localization_errors); % 使用nanmean忽略NaN值计算平均值
plot([time_vector_trj(1), time_vector_trj(end)], [mean_error_pos, mean_error_pos], 'k--', 'LineWidth', 1.0, 'DisplayName', sprintf('平均误差: %.2f m', mean_error_pos));
grid on;
xlabel('时间 (s)');
ylabel('水平定位误差 (m)');
title(sprintf('轨迹定位水平误差 (测距标准差 = %.2f m)', sigma_r));
legend('show', 'Location', 'best');
hold off;
xlim([time_vector_trj(1), time_vector_trj(end)]); % 设置X轴范围与时间向量匹配

fprintf('位置定位误差图绘制完成。\n');
