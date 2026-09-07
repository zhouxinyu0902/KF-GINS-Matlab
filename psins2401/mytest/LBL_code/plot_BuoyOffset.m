function plotBuoyOffset2D_Zoomed(offset_magnitude)
% plotBuoyOffset2D_Zoomed: 绘制水听器浮标的真实位置和偏移位置的二维图，
%                         并用明显箭头表示偏差，调整视图范围以突出偏移。
%
%   输入:
%     offset_magnitude: 标量，表示每个浮标在特定方向上的偏移量。
%                       例如，对于题目中的 [-1,0,0]*5，此参数应为 5。

% 1. 定义水听器阵列的参数
L = 50; % 边长 L
H = L / 100; % 水听器的 z 坐标 (深度，二维图中不显示但保留定义)

% 2. 定义四个水听器的真实（已知）位置 (x, y) - 仅取XY平面
true_sensor_positions_2D = [
    L/2,  L/2;    % 水听器1
    -L/2,  L/2;   % 水听器2
    L/2, -L/2;    % 水听器3
    -L/2, -L/2    % 水听器4
];

% 3. 定义每个水听器的偏移向量 (仅XY平面分量)
% 根据您的要求，偏移是 [-1,0,0]*offset_magnitude，所以在XY平面是 [-offset_magnitude, 0]
offset_vector_2D = [-offset_magnitude, 0]; % 单个浮标的二维偏移向量
buoy_offsets_2D = repmat(offset_vector_2D, size(true_sensor_positions_2D, 1), 1);
% buoy_offsets_2D=[1,1;-1,1;1,-1;-1,-1]*offset_magnitude;
% 4. 计算偏移后的浮标位置 (XY平面)
shifted_sensor_positions_2D = true_sensor_positions_2D + buoy_offsets_2D;

% 5. 绘制图形
figure('Name', '浮标偏移示意图 (二维)', 'Position', [100, 100, 900, 700]); % 调整窗口大小

% 绘制真实位置（已知位置）
p_true = plot(true_sensor_positions_2D(:,1), true_sensor_positions_2D(:,2), ...
              'go', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', '浮标真实位置');
hold on;

% 绘制偏移后的位置
p_shifted = plot(shifted_sensor_positions_2D(:,1), shifted_sensor_positions_2D(:,2), ...
                 'rx', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', '浮标偏移位置');

% 绘制连接真实位置和偏移位置的箭头
% quiver(X,Y,U,V) 绘制从 (X,Y) 开始，向量为 (U,V) 的箭头
% 调整 AutoScaleFactor 使箭头更明显，MaxHeadSize 调整箭头尖端大小
% LineWidth 调整箭头线粗细
for i = 1:size(true_sensor_positions_2D, 1)
    quiver(true_sensor_positions_2D(i,1), true_sensor_positions_2D(i,2), ...
           buoy_offsets_2D(i,1), buoy_offsets_2D(i,2), ...
           0, ... % AutoScaleFactor: 设为0则禁用自动缩放，箭头长度由U,V决定
           'Color', [0 0.4470 0.7410], ... % 蓝色，更醒目
           'LineWidth', 2, ...
           'MaxHeadSize', 0.8); % 增大箭头尖端尺寸
    
    % 添加浮标编号，方便识别，调整位置避免遮挡
    text_offset_x = -200; % 调整文本X方向偏移
    text_offset_y = 100;  % 调整文本Y方向偏移
    text(true_sensor_positions_2D(i,1) + text_offset_x, true_sensor_positions_2D(i,2) + text_offset_y, ...
         sprintf('T%d (True)', i), 'HorizontalAlignment', 'right', 'Color', 'g', 'FontSize', 10);
    text(shifted_sensor_positions_2D(i,1) + text_offset_x, shifted_sensor_positions_2D(i,2) + text_offset_y, ...
         sprintf('T%d (Offset)', i), 'HorizontalAlignment', 'left', 'Color', 'r', 'FontSize', 10);
end

% 设置图表属性
grid on;
axis equal; % 确保X轴和Y轴的比例一致，避免变形

xlabel('X 坐标 (m)');
ylabel('Y 坐标 (m)');
title(sprintf('水听器浮标偏移示意图 (二维平面, 偏移量: %.1f m)', offset_magnitude));

% 调整坐标轴范围，以放大偏移效果
% 找到所有浮标位置的最小和最大XY值
all_x = [true_sensor_positions_2D(:,1); shifted_sensor_positions_2D(:,1)];
all_y = [true_sensor_positions_2D(:,2); shifted_sensor_positions_2D(:,2)];

min_x = min(all_x); max_x = max(all_x);
min_y = min(all_y); max_y = max(all_y);

% 计算合适的填充边距，使箭头有足够的空间显示
padding_x = (max_x - min_x) * 0.2 + offset_magnitude * 1.5; % 根据偏移量调整填充
padding_y = (max_y - min_y) * 0.2 + offset_magnitude * 1.5;


xlim([min_x - padding_x, max_x + padding_x]);
ylim([min_y - padding_y, max_y + padding_y]);


% 创建一个空的 plot 来为箭头的图例条目
h_arrow = plot(NaN, NaN, 'Color', [0 0.4470 0.7410], 'LineWidth', 2, 'DisplayName', '浮标偏移');
legend([p_true, p_shifted, h_arrow], {'浮标标定位置', '浮标真实位置', '浮标偏移'}, 'Location', 'best');

hold off;
axis([-40,40,-40,40])
end