function plotTrajectoryAndTransponders(trajectory_coords, transponders_coords)
% plotTrajectoryAndTransponders: 绘制水下航行器轨迹和LBL信标位置的对比图
%
%   输入:
%     trajectory_coords  : N x 3 矩阵，表示航行器轨迹的 (X, Y, Z) 坐标。
%                          只使用X和Y进行2D绘图。
%     transponders_coords: M x 3 矩阵，表示M个LBL信标的 (X, Y, Z) 坐标。
%                          只使用X和Y进行2D绘图。
%     start_pos          : 1 x 3 向量，航行器起点坐标 [X0, Y0, Z0]。
%     end_pos            : 1 x 3 向量，航行器目标方向点/终点坐标 [Xf, Yf, Zf]。

% 创建一个新的图形窗口
figure('Position',[100,100,350,350]); % 调整图形窗口大小以容纳两个子图

% 绘制实际轨迹
plot(trajectory_coords(:,1), trajectory_coords(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName', '实际轨迹');
hold on; % 保持当前图形，以便在其上添加更多元素
start_pos=trajectory_coords(1,:);
end_pos=trajectory_coords(end,:);
% 绘制起点和目标方向点
plot(start_pos(1), start_pos(2), 'go', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', '起点');
plot(end_pos(1), end_pos(2), 'rx', 'MarkerSize', 8, 'LineWidth', 2, 'DisplayName', '目标方向点');

% 绘制理想直线轨迹作为参考
% 确保 start_pos 和 end_pos 至少有两个维度
if numel(start_pos) >= 2 && numel(end_pos) >= 2
    ideal_x = linspace(start_pos(1), end_pos(1), 100);
    ideal_y = linspace(start_pos(2), end_pos(2), 100);
    plot(ideal_x, ideal_y, 'k--', 'LineWidth', 1, 'DisplayName', '理想直线方向');
end

% 绘制信标
if ~isempty(transponders_coords)
    plot(transponders_coords(:,1), transponders_coords(:,2), 'ms', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'LBL浮标');
    % 为每个信标添加编号
    for k = 1:size(transponders_coords, 1)
        text(transponders_coords(k,1) + 100, transponders_coords(k,2) + 100, sprintf('T%d', k), 'FontSize', 10, 'Color', 'm');
    end
end

% 设置图表属性
grid on; % 显示网格
axis equal; % 确保X和Y轴比例一致，避免图形变形
xlabel('X 坐标 (m)'); % X轴标签
ylabel('Y 坐标 (m)'); % Y轴标签
title('拟匀速航行轨迹与浮标位置 (XY 平面)'); % 图表标题
legend('show', 'Location', 'best'); % 显示图例，并自动选择最佳位置
hold off; % 释放图形，不再添加新元素到当前图
axis([-3000,3000,-3000,3000])
end