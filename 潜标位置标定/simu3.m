%% 统一模型假设下，单个航行器位置对漂移的估计
clear; clc; close all;
rng(1); % 固定随机种子，方便复现结果

% --- 1. 基础物理参数 (真实值) ---
side_length_true = 20000;          % 边长 20km
H_emitter_true = -1000;            % 发声点真实深度
Z_vehicle_true = -1200;            % 航行器真实深度
true_theta_deg = 8;                % 真实俯仰角
true_phi_deg = 45;                 % 真实方位角

deg2rad = pi/180;
rad2deg = 180/pi;

% --- 2. 产生真实坐标 ---
R_tri = side_length_true / sqrt(3);
P_gnss_true = [
    0,              R_tri,       0;   
    -side_length_true/2, -R_tri/2,    0;   
    side_length_true/2,  -R_tri/2,    0    
];

% 应用统一偏移计算真实发声点 [cite: 3, 6]
d_offset_true = abs(H_emitter_true) * tan(true_theta_deg * deg2rad);
dx_true = d_offset_true * sin(true_phi_deg * deg2rad);
dy_true = d_offset_true * cos(true_phi_deg * deg2rad);
P_emitters_true = P_gnss_true + [dx_true, dy_true, H_emitter_true];

% 产生航行器真实位置
mid_12_true = (P_gnss_true(1,:) + P_gnss_true(2,:)) / 2;
V_xy_true = ([0,0,0] + P_gnss_true(1,:) + mid_12_true) / 3; 
P_vehicle_true = [V_xy_true(1), V_xy_true(2), Z_vehicle_true];

% --- 3. 添加传感器观测误差 (产生测量值) ---
% (1) GNSS 测量误差 (假设 0.5m 漂移)
P_gnss = P_gnss_true + [0.5*randn(3,2), zeros(3,1)]; 

% (2) 深度计测量误差 (假设 1m 标准差)
H_emitter = H_emitter_true + 1.0 * randn(); 
Z_vehicle = Z_vehicle_true + 1.0 * randn();

% (3) 声学测距误差 (假设 8m 标准差) 
R_obs_true = sqrt(sum((P_emitters_true - P_vehicle_true).^2, 2));
R_obs = R_obs_true + 8.0 * randn(3,1); 

% (4) 用于标定的航行器已知位置误差 (假设 5m 水平误差) 
P_vehicle = P_vehicle_true + [5.0*randn(1,2), (Z_vehicle - Z_vehicle_true)]; 

pngname = 'F3-1.png';
plot_beacons_auv(P_gnss,P_emitters_true, H_emitter,P_vehicle_true,true_theta_deg,true_phi_deg,R_obs,pngname);
%% --- 4. 基于测量值的非线性最小二乘估计 (NLS) ---
fprintf('\n--- 启动基于含有噪声数据的估计程序 ---\n');
% 目标函数：使用含有误差的测量值 P_gnss, H_emitter, Z_vehicle, R_obs 

% H_diff = Z_vehicle - H_emitter; 
% D_obs_horiz = sqrt(max(0, R_obs.^2 - H_diff^2));
% obj_fun = @(x) [
%     sqrt((P_vehicle(1) - (P_gnss(1,1) + x(1)))^2 + (P_vehicle(2) - (P_gnss(1,2) + x(2)))^2) - D_obs_horiz(1);
%     sqrt((P_vehicle(1) - (P_gnss(2,1) + x(1)))^2 + (P_vehicle(2) - (P_gnss(2,2) + x(2)))^2) - D_obs_horiz(2);
%     sqrt((P_vehicle(1) - (P_gnss(3,1) + x(1)))^2 + (P_vehicle(2) - (P_gnss(3,2) + x(2)))^2) - D_obs_horiz(3);
% ];

obj_fun = @(x) [
    sqrt((P_vehicle(1) - (P_gnss(1,1) + x(1)))^2 + (P_vehicle(2) - (P_gnss(1,2) + x(2)))^2 + (Z_vehicle - H_emitter)^2) - R_obs(1);
    sqrt((P_vehicle(1) - (P_gnss(2,1) + x(1)))^2 + (P_vehicle(2) - (P_gnss(2,2) + x(2)))^2 + (Z_vehicle - H_emitter)^2) - R_obs(2);
    sqrt((P_vehicle(1) - (P_gnss(3,1) + x(1)))^2 + (P_vehicle(2) - (P_gnss(3,2) + x(2)))^2 + (Z_vehicle - H_emitter)^2) - R_obs(3);
];

x0 = [0, 0]; 
options = optimoptions('lsqnonlin', 'Display', 'none');
[res_drift, resnorm] = lsqnonlin(obj_fun, x0, [], [], options);

est_dx = res_drift(1);
est_dy = res_drift(2);
% 3. 将水平位移 (dx, dy) 转换为角度 (theta, phi)
% 根据定义：dx = d*sin(phi), dy = d*cos(phi), d = |H|*tan(theta)
est_d_offset = sqrt(est_dx^2 + est_dy^2);
est_theta_rad = atan(est_d_offset / abs(H_emitter));
est_theta_deg_final = est_theta_rad * rad2deg;
% 使用 atan2 求方位角 (x在前, y在后 对应 sin/cos 定义)
% 注意：atan2(sin, cos) 直接给出从北向顺时针的角度
est_phi_rad = atan2(est_dx, est_dy);
est_phi_deg_final = mod(est_phi_rad * rad2deg, 360); % 确保在 0-360 范围内
% --- 3. 结果输出 ---
fprintf('================ 估计结果 (航行器位置已知) =================\n');
fprintf('真实漂移向量: [dx: %.2f, dy: %.2f] (m)\n', dx_true, dy_true);
fprintf('估计漂移向量: [dx: %.2f, dy: %.2f] (m)\n', est_dx, est_dy);
fprintf('估计漂移误差: [dx: %.2f, dy: %.2f] (m)\n', est_dx-dx_true, est_dy-dy_true);
fprintf('----------------------------------------------------------\n');
fprintf('真实角度: Theta = %.2f°, Phi = %.2f°\n', true_theta_deg, true_phi_deg);
fprintf('估计角度: Theta = %.2f°, Phi = %.2f°\n', est_theta_deg_final, est_phi_deg_final);
fprintf('残差平方和: %.4e\n', resnorm);
fprintf('==========================================================\n');
%% --- 4. 误差敏感度观察 (可选：网格搜索可视化) ---
theta_vec = 0:0.1:15;
phi_vec = 0:2:360;
[T_mesh, P_mesh] = meshgrid(theta_vec, phi_vec);
J_val = zeros(size(T_mesh));
for r = 1:size(T_mesh, 1)
    for c = 1:size(T_mesh, 2)
        % 遍历计算每一个测试角度对应的残差
        d_test = abs(H_emitter) * tan(T_mesh(r,c)*deg2rad);
        dx_test = d_test * sin(P_mesh(r,c)*deg2rad);
        dy_test = d_test * cos(P_mesh(r,c)*deg2rad);
        
        tmp_res = [
            sqrt((P_vehicle(1)-(P_gnss(1,1)+dx_test))^2 + (P_vehicle(2)-(P_gnss(1,2)+dy_test))^2 + (Z_vehicle-H_emitter)^2) - R_obs(1);
            sqrt((P_vehicle(1)-(P_gnss(2,1)+dx_test))^2 + (P_vehicle(2)-(P_gnss(2,2)+dy_test))^2 + (Z_vehicle-H_emitter)^2) - R_obs(2);
            sqrt((P_vehicle(1)-(P_gnss(3,1)+dx_test))^2 + (P_vehicle(2)-(P_gnss(3,2)+dy_test))^2 + (Z_vehicle-H_emitter)^2) - R_obs(3);
        ];
        J_val(r,c) = sum(tmp_res.^2);
    end
end
fig=myfigurestartup(4,3,'paper');
contourf(P_mesh, T_mesh, log10(J_val), 50, 'EdgeColor', 'none');
colorbar; colormap('jet'); hold on;
plot(est_phi_deg_final, est_theta_deg_final, 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
xlabel('方位角 \phi (deg)'); ylabel('俯仰角 \theta (deg)');
title('已知航行器位置下的角度估计残差面 (Log Scale)');
legend('残差强度', '估计最优解', 'Location', 'northeast', 'FontSize', 8);
exportgraphics(fig, fullfile('', ...
    'F3-1.png'), 'Resolution', 600);
%% --- 误差量化与残差分析 (测距 + 坐标漂移) ---
est_theta = est_theta_deg_final;
est_phi = est_phi_deg_final;

% 1. 首先计算补偿后的发声点估计位置 (必须先定义才能使用)
est_d = abs(H_emitter) * tand(est_theta);
est_dx = est_d * sind(est_phi);
est_dy = est_d * cosd(est_phi);
% 应用估计的漂移矢量到名义 GNSS 坐标上
P_est_emitters = P_gnss + [est_dx, est_dy, H_emitter]; 

% 2. 坐标修正对比：计算点位误差 (欧氏距离)
% 修正前：GNSS名义坐标(海面) vs 真实发声点
pos_err_before = sqrt(sum((P_gnss - P_emitters_true).^2, 2)); 
% 修正后：估计补偿位置 vs 真实发声点 (现在 P_est_emitters 已定义)
pos_err_after  = sqrt(sum((P_est_emitters - P_emitters_true).^2, 2));

% 3. 测距残差对比
% 定义“名义发声点” (已知深度，但假设没有水平偏移)
P_nominal = [P_gnss(:,1:2), repmat(H_emitter, 3, 1)];
dist_nominal = sqrt(sum((P_nominal - P_vehicle_true).^2, 2)); 
error_nominal = dist_nominal - R_obs; % 修正前的残差

% 估计后：使用补偿后的发声点坐标
dist_compensated = sqrt(sum((P_est_emitters - P_vehicle_true).^2, 2));
error_compensated = dist_compensated - R_obs_true; % 修正后的残差
% --- 数据打印报告 ---
fprintf('\n================ 综合误差量化报告 (单位: 米) =================\n');
fprintf('潜标编号 | 坐标误差(修正前) | 坐标误差(修正后) | 测距残差(修正前) | 测距残差(修正后)\n');
fprintf('----------------------------------------------------------------------------\n');
for i = 1:3
    fprintf('  #%d     |    %10.2f    |    %10.4f    |    %10.2f    |    %10.4f\n', ...
            i, pos_err_before(i), pos_err_after(i), error_nominal(i), error_compensated(i));
end
fprintf('----------------------------------------------------------------------------\n');
fprintf('坐标RMSE : 修正前 = %.2f m, 修正后 = %.4f m\n', rms(pos_err_before), rms(pos_err_after));
fprintf('测距RMSE : 修正前 = %.2f m, 修正后 = %.4f m\n', rms(error_nominal), rms(error_compensated));
fprintf('============================================================================\n');

% --- 可视化：创建多维度对比图 ---
fig = myfigurestartup(7, 3, 'paper');
t_layout = tiledlayout(1, 2, 'TileSpacing', 'Compact', 'Padding', 'Compact');

% 左侧图：测距残差对比 (对数坐标，以同时看清百米和厘米)
ax1 = nexttile;
b1 = bar([abs(error_nominal), abs(error_compensated)]); % 取绝对值以防对数报错
% set(ax1, 'YScale', 'log'); 
set(ax1, 'XTickLabel', {'潜标1', '潜标2', '潜标3'});
ylabel('测距残差绝对值 |m| (Log Scale)'); title('测距维度：补偿效果');
legend('修正前(Nominal)', '修正后(Compensated)', 'Location', 'northeast', 'FontSize', 7);
grid on;

% 右侧图：坐标漂移修正对比
ax2 = nexttile;
b2 = bar([pos_err_before, pos_err_after]);
% set(ax2, 'YScale', 'log');
set(ax2, 'XTickLabel', {'潜标1', '潜标2', '潜标3'});
ylabel('坐标点位误差 (m) (Log Scale)'); title('坐标维度：漂移修正');
grid on;

title(t_layout, sprintf('潜标位移补偿量化分析 (\\theta=%d^\\circ, \\phi=%d^\\circ)', true_theta_deg, true_phi_deg), ...
    'FontSize', 12, 'FontWeight', 'bold');

exportgraphics(fig, fullfile('', 'F3-2.png'), 'Resolution', 600);
%% --- 潜标局部坐标修正对比俯视图 (增加测距圆弧显示) ---
% 1. 预计算估计点 (确保在循环前计算)
est_d = abs(H_emitter) * tand(est_theta_deg_final);
est_dx = est_d * sind(est_phi_deg_final);
est_dy = est_d * cosd(est_phi_deg_final);
P_est_emitters = P_gnss + [est_dx, est_dy, H_emitter];
% 2. 绘图初始化
fig_zoom = myfigurestartup(7, 3, 'paper');
t_zoom = tiledlayout(1, 3, 'TileSpacing', 'Loose', 'Padding', 'Compact');
zoom_range = 250; 
for i = 1:3
    nexttile; hold on; grid on; axis equal;
    
    % --- [核心新增] 绘制以航行器为圆心的局部测距圆弧 ---
    % 计算航行器到三个关键点的水平半径
    r_gnss = sqrt(sum((P_vehicle_true(1:2) - P_gnss(i,1:2)).^2));
    r_true = sqrt(sum((P_vehicle_true(1:2) - P_emitters_true(i,1:2)).^2));
    r_est  = sqrt(sum((P_vehicle_true(1:2) - P_est_emitters(i,1:2)).^2));
    
    % 计算从航行器看潜标的中心方位角
    angle_center = atan2(P_gnss(i,2) - P_vehicle_true(2), P_gnss(i,1) - P_vehicle_true(1));
    % 定义弧线跨度 (对于20km半径，1度跨度约350m，足够覆盖zoom_range)
    angle_span = deg2rad(1.0); 
    theta_arc = linspace(angle_center - angle_span/2, angle_center + angle_span/2, 100);
    
    % 绘制圆弧 (蓝色-名义，红色-真实，黑色-补偿)
    % 使用 HandleVisibility, off 避免圆弧挤占图例
    plot(P_vehicle_true(1) + r_gnss * cos(theta_arc), P_vehicle_true(2) + r_gnss * sin(theta_arc), 'b:', 'LineWidth', 0.5, 'HandleVisibility', 'off');
    plot(P_vehicle_true(1) + r_true * cos(theta_arc), P_vehicle_true(2) + r_true * sin(theta_arc), 'r-', 'LineWidth', 0.5, 'HandleVisibility', 'off');
    plot(P_vehicle_true(1) + r_est * cos(theta_arc), P_vehicle_true(2) + r_est * sin(theta_arc), 'k--', 'LineWidth', 0.5, 'HandleVisibility', 'off');
    % --- 原有绘图元素 ---
    h_gnss = plot(P_gnss(i,1), P_gnss(i,2), 'bo', 'MarkerSize', 10, 'LineWidth', 1.5);
    h_true = plot(P_emitters_true(i,1), P_emitters_true(i,2), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
    h_est = plot(P_est_emitters(i,1), P_est_emitters(i,2), 'kx', 'MarkerSize', 12, 'LineWidth', 2);
    h_v = plot(P_vehicle_true(1), P_vehicle_true(2), 'mp', 'MarkerSize', 12, 'MarkerFaceColor', 'm');
    
    % 连线指示
    plot([P_gnss(i,1) P_vehicle_true(1)], [P_gnss(i,2) P_vehicle_true(2)], 'm--', 'LineWidth', 0.5);
    plot([P_gnss(i,1) P_emitters_true(i,1)], [P_gnss(i,2) P_emitters_true(i,2)], 'r:', 'LineWidth', 1);
    plot([P_emitters_true(i,1) P_est_emitters(i,1)], [P_emitters_true(i,2) P_est_emitters(i,2)], 'k-');
    
    % 视图控制
    xlim([P_gnss(i,1)-zoom_range/2, P_gnss(i,1)+zoom_range/2]);
    ylim([P_gnss(i,2)-zoom_range/2, P_gnss(i,2)+zoom_range/2]);
    xlabel('E (m)'); ylabel('N (m)');
    title(sprintf('潜标 #%d 测距几何', i));
    
    % 文字标注
    dist_v = sqrt(sum((P_gnss(i,1:2) - P_vehicle_true(1:2)).^2));
    text(P_gnss(i,1)-zoom_range*0.4, P_gnss(i,2)+zoom_range*0.4, ...
        sprintf('测距半径: %.0fm', dist_v), 'Color', 'm', 'FontSize', 7, 'FontWeight', 'bold');
end
% 3. 图例与导出
lgd = legend([h_gnss, h_true, h_est, h_v], ...
    {'名义 GNSS', '真实发声点', '算法修正位置', '航行器'}, ...
    'Orientation', 'horizontal');
lgd.Layout.Tile = 'north';
exportgraphics(fig_zoom, fullfile('', 'F3-3.png'), 'Resolution', 600);