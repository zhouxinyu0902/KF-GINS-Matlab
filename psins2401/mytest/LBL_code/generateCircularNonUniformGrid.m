[grid_points_x, grid_points_y, total_points_count] =generateAndPlotSparseCircularGrid(20,0,1,1);
[grid_points_x, grid_points_y, total_points_count] =generateAndPlotSparseCircularGrid(6000,50,50,50);
function [grid_points_x, grid_points_y, total_points_count] = ...
    generateAndPlotSparseCircularGrid(max_radius,radial_start_radius,expected_circumference_spacing,radial_step)
% generateAndPlotSparseCircularGrid()
% 生成一个稀疏的圆形非均匀网格点坐标，并进行绘图。
% 网格参数：半径间隔50m，圆周上间隔50m。
%
% 输出:
%   grid_points_x    : 包含所有网格点X坐标的行向量。
%   grid_points_y    : 包含所有网格点Y坐标的行向量。
%   total_points_count : 生成的总网格点数量。

% --- 网格参数定义 (稀疏化) ---
center_x = 0;
center_y = 0;
% max_radius = 6000; % 最大半径
% radial_start_radius = 50; % 径向距离从 50m 开始
% radial_step = 50; % 径向步长为 50m
% expected_circumference_spacing = 50; % 期望的圆周点间距为 50m

fprintf('正在生成稀疏非均匀网格点坐标...\n');

% --- 径向网格点 (半径) ---
radii = radial_start_radius:radial_step:max_radius;
num_radial_layers = length(radii);

% --- 估算总点数以进行预分配 (提高效率) ---
estimated_total_points = 0;
for r_est = radii
    num_angles_est = ceil((2 * pi * r_est) / expected_circumference_spacing);
    if num_angles_est < 4 % 确保每个圆至少有4个点
        num_angles_est = 4;
    end
    estimated_total_points = estimated_total_points + num_angles_est;
end

grid_points_x = zeros(1, estimated_total_points);
grid_points_y = zeros(1, estimated_total_points);
current_idx = 1; % 用于跟踪当前填充到哪个位置

total_points_count = 0;

% --- 遍历每个半径层，生成点 ---
for k_idx = 1:num_radial_layers
    current_radius = radii(k_idx);
    
    % 计算当前半径下的角度点数
    num_angles = ceil((2 * pi * current_radius) / expected_circumference_spacing);
    
    % 确保至少有足够点来形成一个形状（至少4个点）
    if num_angles < 4
        num_angles = 4;
    end
    
    % 生成角度点 (从0到2pi，不包含2pi以避免重复点)
    angles = linspace(0, 2*pi, num_angles + 1);
    angles = angles(1:end-1); 

    % 将极坐标点转换为笛卡尔坐标
    current_x_points = center_x + current_radius * cos(angles);
    current_y_points = center_y + current_radius * sin(angles);
    
    % 将当前层的所有点添加到预分配的数组中
    grid_points_x(current_idx : current_idx + num_angles - 1) = current_x_points;
    grid_points_y(current_idx : current_idx + num_angles - 1) = current_y_points;
    
    current_idx = current_idx + num_angles;
    total_points_count = total_points_count + num_angles;

    % 打印进度 (可选)
    if mod(k_idx, 10) == 0 || k_idx == num_radial_layers
        fprintf('  生成点：已处理 %d/%d 个径向层...\n', k_idx, num_radial_layers);
    end
end
% 如果实际点数小于预估，截断数组
grid_points_x = grid_points_x(1:total_points_count);
grid_points_y = grid_points_y(1:total_points_count);

fprintf('网格点坐标生成完成。总点数: %d\n', total_points_count);


% --- 绘图部分 ---
fprintf('正在绘制网格示意图...\n');
figure;
hold on; % 允许在同一图中绘制多个对象
axis equal; % 保持轴比例一致，使圆形看起来是圆的
xlabel('X 坐标 (m)');
ylabel('Y 坐标 (m)');
title({'稀疏非均匀网格示意图',...
    ['最大半径',num2str(max_radius),'m'],...
    ['径向间隔',num2str(radial_step),'m',',切向间隔',num2str(expected_circumference_spacing),'m']
    });
grid on;
box on; % 绘制边框

% 绘制圆的边界
theta_circle = linspace(0, 2*pi, 360); % 绘制一个平滑的圆
plot(center_x + max_radius * cos(theta_circle), center_y + max_radius * sin(theta_circle), 'r-', 'LineWidth', 1.5, 'DisplayName', '最大边界');

% 绘制所有生成的网格点
plot(grid_points_x, grid_points_y, 'b.', 'MarkerSize', 3,'DisplayName','网格点'); % 蓝色点

% 可选：绘制径向和同心圆网格线，使其更像一个网格图
% 绘制同心圆线
% 我们使用 radii 数组直接绘制，而不是像之前那样跳过层。
for r = radii
    num_angles_line = ceil((2 * pi * r) / expected_circumference_spacing);
    if num_angles_line < 4
        num_angles_line = 4;
    end
    angles_line = linspace(0, 2*pi, num_angles_line + 1);
    plot(center_x + r * cos(angles_line), center_y + r * sin(angles_line), 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5, 'HandleVisibility', 'off'); % 灰色，细线
end

% 绘制径向线（从中心到最外圈的某些角度）
% 为了不至于太密集，只绘制部分角度的径向线
outermost_num_angles = ceil((2 * pi * max_radius) / expected_circumference_spacing);
if outermost_num_angles < 4
    outermost_num_angles = 4;
end
outermost_angles_for_lines = linspace(0, 2*pi, outermost_num_angles + 1);
outermost_angles_for_lines = outermost_angles_for_lines(1:end-1);

skip_radial_lines = 10; % 每隔 10 条径向线绘制一条 (这里可能仍然会密集，可以调整)
if length(outermost_angles_for_lines) < skip_radial_lines % 如果角度点数不够，则全部绘制
    skip_radial_lines = 1;
end
for a_idx = 1:skip_radial_lines:length(outermost_angles_for_lines)
    current_angle = outermost_angles_for_lines(a_idx);
    plot([center_x, center_x + max_radius * cos(current_angle)], ...
         [center_y, center_y + max_radius * sin(current_angle)], ...
         'Color', [0.7 0.7 0.7], 'LineWidth', 0.5, 'HandleVisibility', 'off'); % 灰色，细线
end

hold off;
legend('show', 'Location', 'southeast'); % 显示图例
fprintf('网格示意图绘制完成！\n');

end