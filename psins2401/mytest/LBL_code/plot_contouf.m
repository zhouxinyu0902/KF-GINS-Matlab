function plot_contouf(localization_error_map,sensor_positions,X_grid, Y_grid,sigma_r,L)
figure;
num_levels = 20; % 示例：生成 20 条等高线
contourf(X_grid, Y_grid, localization_error_map, num_levels, 'LineStyle', 'none'); 
% 'LineStyle', 'none' 使等高线之间没有明显的黑线

% 添加颜色条
colorbar;

% 设置坐标轴方向和比例
axis xy;       % 修正 Y 轴方向，使其与典型的笛卡尔坐标系图匹配
axis equal;    % 使 X 和 Y 轴的比例相同，避免图像变形

% 添加标签和标题
xlabel('东向坐标 (m)'); % East coordinate
ylabel('北向坐标 (m)'); % North coordinate
title({'定位精度分布图(蒙特卡洛仿真)',['测距误差 ',num2str(sigma_r),'米'],...
    ['阵元间距 ',num2str(L),'米']}); % Localization Accuracy Distribution Map

% 自定义颜色映射
colormap('jet'); % 使用 'jet' 颜色映射，与示例图相似

% 调整颜色条限制 (可选)
c_min = min(localization_error_map(:));
c_max = max(localization_error_map(:));
% clim([c_min, c_max]);
% clim([4, 5]);

% 在图上添加传感器位置 (作为小标记)
hold on;
plot(sensor_positions(:,1), sensor_positions(:,2), 'R*', 'MarkerSize', 8, 'LineWidth', 2); % 白色 'x' 标记
% 添加传感器标签
% text(sensor_positions(1,1)+100, sensor_positions(1,2)+50, 'S1', 'Color', 'w');
% text(sensor_positions(2,1)-500, sensor_positions(2,2)+50, 'S2', 'Color', 'k');
% text(sensor_positions(3,1)+100, sensor_positions(3,2)+50, 'S3', 'Color', 'k');
% text(sensor_positions(4,1)-500, sensor_positions(4,2)+50, 'S4', 'Color', 'k');
% hold off;
fprintf('绘图完成！\n');
