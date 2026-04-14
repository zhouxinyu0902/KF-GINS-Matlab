%% 深海 LBL 潜标阵列数据仿真生成
clear; clc; close all;

% --- 1. 基础物理参数 ---
side_length = 20000;          % 边长 20km
H_emitter = -1000;            % 发声点深度
Z_vehicle = -1200;            % 航行器深度

% 设定统一漂移 (待估计参数)
true_theta_deg = 8;           % 俯仰角
true_phi_deg = 45;            % 方位角 (正北为0, 顺时针)

deg2rad = pi/180;
theta_rad = true_theta_deg * deg2rad;
phi_rad = true_phi_deg * deg2rad;

% --- 2. 产生潜标 GNSS 坐标 (等边三角形顶点) ---
R_tri = side_length / sqrt(3); % 外接圆半径
% P1: 北, P2: 西南, P3: 东南
P_gnss = [
    0,              R_tri,       0;   
    -side_length/2, -R_tri/2,    0;   
    side_length/2,  -R_tri/2,    0    
];

% --- 3. 产生发声点坐标 (应用统一偏移) ---
d_offset = abs(H_emitter) * tan(theta_rad);
dx = d_offset * sin(phi_rad); % 东向分量
dy = d_offset * cos(phi_rad); % 北向分量
P_emitters = P_gnss + [dx, dy, H_emitter];

% --- 4. 产生航行器位置 (放置在第1部分区域) ---
% 定义：中心点[0,0], P1, 以及 P1-P2 的中点 围成的三角形重心
mid_12 = (P_gnss(1,:) + P_gnss(2,:)) / 2;
V_xy = ([0,0,0] + P_gnss(1,:) + mid_12) / 3; 
P_vehicle = [V_xy(1), V_xy(2), Z_vehicle];

% --- 5. 计算声学观测值 (用于后续估计) ---
% 航行器到三个发声点的真实距离 R
R_obs = sqrt(sum((P_emitters - P_vehicle).^2, 2));

pngname = 'F2.png';
plot_beacons_auv(P_gnss,P_emitters,H_emitter,P_vehicle,true_theta_deg,true_phi_deg,R_obs,pngname);
%%
% --- 6. 绘图展示 ---
fig=myfigurestartup(5,3,'paper');
hold on; grid on; axis equal;

% 绘制潜标系统
for i = 1:3
    % 垂直投影辅助线
    h1 = plot3([P_gnss(i,1) P_gnss(i,1)], [P_gnss(i,2) P_gnss(i,2)], [0 H_emitter], 'k:', 'LineWidth', 1);
    % 发声点与垂直投影连线
    h2 = plot3([P_gnss(i,1) P_emitters(i,1)], [P_gnss(i,2) P_emitters(i,2)], [H_emitter H_emitter], 'r:', 'LineWidth', 1);
    % 实际电缆连线 (GNSS到发声点)
    h3 = plot3([P_gnss(i,1) P_emitters(i,1)], [P_gnss(i,2) P_emitters(i,2)], [0 H_emitter], 'r--', 'LineWidth', 1.5);
    % 节点标志
    h4 = scatter3(P_gnss(i,1), P_gnss(i,2), P_gnss(i,3), 10, 'b', 'filled');
    h5 = scatter3(P_emitters(i,1), P_emitters(i,2), P_emitters(i,3), 10, 'r', 'filled');
end

% 航行器
h_v = scatter3(P_vehicle(1), P_vehicle(2), P_vehicle(3), 120, 'm', 'pentagram', 'filled');

% 北向参考
h6 = quiver3(0, 0, 0, 0, 3000, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.8);

% 修饰
view(3);
xlabel('东向 East (m)'); ylabel('北向 North (m)'); zlabel('深度 Depth (m)');

title('潜标阵列与航行器空间布局仿真');
% legend('垂直参考线', '水平偏移投影', '电缆(漂移向量)', 'GNSS点', '发声点', '北向参考', '航行器');
% legend('Location', 'eastoutside', 'FontSize', 8)
% --- 图例优化 ---
lgd = legend([h1, h2, h3, h4, h5, h6, h_v], ...
    {'垂直参考线', '水平偏移投影', '电缆(漂移向量)', 'GNSS点', '发声点', '北向参考', '航行器'}, ...
    'Location', 'eastoutside', 'FontSize', 8);
lgd.Box = 'on';

% --- 7. 打印输出数据 (方便拷贝使用) ---
fprintf('================ 仿真数据输出 =================\n');
fprintf('设定漂移: Theta = %.2f°, Phi = %.2f°\n', true_theta_deg, true_phi_deg);
fprintf('-----------------------------------------------\n');
for i = 1:3
    fprintf('潜标 %d GNSS坐标: [%10.2f, %10.2f, %7.2f]\n', i, P_gnss(i,1), P_gnss(i,2), P_gnss(i,3));
    fprintf('潜标 %d 发声坐标: [%10.2f, %10.2f, %7.2f]\n', i, P_emitters(i,1), P_emitters(i,2), P_emitters(i,3));
end
fprintf('-----------------------------------------------\n');
fprintf('航行器真实坐标: [%10.2f, %10.2f, %7.2f]\n', P_vehicle(1), P_vehicle(2), P_vehicle(3));
fprintf('声学测距观测 R1=%.2f, R2=%.2f, R3=%.2f\n', R_obs(1), R_obs(2), R_obs(3));
fprintf('===============================================\n');
exportgraphics(fig, fullfile('', ...
    'F2.png'), 'Resolution', 600);