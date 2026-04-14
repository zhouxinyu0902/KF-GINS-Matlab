%% 深海潜标多维度漂移观察 (3D, 俯瞰, 横截面)
clear; clc; close all;

% --- 1. 参数设置 ---
H_surface = 0;                
H_emitter = -1000;       
true_theta_deg = 21.8;           % 俯仰角
true_phi_deg = 45;            % 方位角 (正北为0, 顺时针)
gnss_pos_xy = [0, 0];         

deg2rad = pi/180;
theta_rad = true_theta_deg * deg2rad;
phi_rad = true_phi_deg * deg2rad;

% 计算偏移量
d_offset = abs(H_emitter) * tan(theta_rad);
dx = d_offset * sin(phi_rad); % 东向
dy = d_offset * cos(phi_rad); % 北向

% 关键点坐标
P_gnss = [0, 0, 0];
P_emitter = [dx, dy, H_emitter];

% --- 2. 绘图初始化 ---
fig=myfigurestartup(4,4,'paper');
% t = tiledlayout(1, 3, 'TileSpacing', 'Compact', 'Padding', 'Compact');

% ==========================================================
% 子图 1: 3D 全貌 (Perspective View)
% ==========================================================
subplot(2,2,[1,2]); hold on; grid on;
draw_transponder(P_gnss, P_emitter, d_offset, true_phi_deg, true_theta_deg);
view(3); % 标准3D视角
title('3D 全貌视图');
legend('垂直投影','垂直投影和发声点连线','GNSS点和发声点连线','GNSS点','发声点','北向箭头')
legend('Location','eastoutside','FontSize', 6)
zlim([-1200, 100]); 

% ==========================================================
% 子图 2: 俯瞰图 (Top-down XY View)
% ==========================================================
subplot(2,2,3); hold on; grid on;
draw_transponder(P_gnss, P_emitter, d_offset, true_phi_deg, true_theta_deg);
view(0, 90); % 俯拍视角
title('俯瞰图 (XY 平面偏移)');
xlabel('东向 (m)'); ylabel('北向 (m)');
% 限制范围以突出偏移
axis_range = d_offset * 1.5;
axis([-axis_range axis_range -axis_range axis_range]);

% ==========================================================
% 子图 3: 横截面 (Side View - 沿偏移方向)
% ==========================================================
subplot(2,2,4); hold on; grid on;
draw_transponder(P_gnss, P_emitter, d_offset, true_phi_deg, true_theta_deg);
% 关键：调整视角使其垂直于偏移方向矢量 [dx, dy]
% 这样可以看到最真实的 theta 角，没有投影失真
view(true_phi_deg + 90, 0); 
title('横截面视图 (垂直剖面)');
zlabel('深度 (m)');
zlim([-1200, 100]);
exportgraphics(fig, fullfile('', ...
    'F1.png'), 'Resolution', 600);

% --- 统一绘图辅助函数 (增加了距离显示) ---
function draw_transponder(P_g, P_e, d, phi_d, theta_d)
    % 1. 绘制参考线
    plot3([P_g(1) P_g(1)], [P_g(2) P_g(2)], [P_g(3) P_e(3)], 'k:', 'LineWidth', 1); % 垂直参考线
    
    % 2. 绘制水平投影线 (即漂移距离的物理表现)
    plot3([P_g(1) P_e(1)], [P_g(2) P_e(2)], [P_e(3) P_e(3)], 'r:', 'LineWidth', 1.5); 
    
    % 3. 绘制漂移距离数值显示 (在投影线中点位置)
    text_pos = [(P_g(1)+P_e(1))/2, (P_g(2)+P_e(2))/2, P_e(3)];
    text(text_pos(1), text_pos(2), text_pos(3) - 50, ...
        sprintf('d = %.2f m', d), ...
        'Color', 'r', 'FontWeight', 'bold', 'FontSize', 9, ...
        'HorizontalAlignment', 'center', 'BackgroundColor', 'none');

    % 4. 绘制缆绳 (红色虚线)
    plot3([P_g(1) P_e(1)], [P_g(2) P_e(2)], [P_g(3) P_e(3)], 'r--', 'LineWidth', 2);
    
    % 5. 绘制关键点
    scatter3(P_g(1), P_g(2), P_g(3), 80, 'b', 'filled'); % GNSS
    scatter3(P_e(1), P_e(2), P_e(3), 80, 'r', 'filled'); % 发声点
    
    % 6. 绘制北向参考箭头
    quiver3(0, 0, 0, 0, d*1.2, 0, 'g', 'LineWidth', 1.5, 'MaxHeadSize', 0.5);
    
    % 7. 坐标轴标签
    xlabel('E (m)'); ylabel('N (m)'); zlabel('Z (m)');
end
