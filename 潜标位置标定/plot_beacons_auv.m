function plot_beacons_auv(P_gnss, P_emitters_true, H_emitter, P_v_true, true_theta_deg, true_phi_deg, R_obs, pngname)
% --- 6. 绘图展示 ---
fig = myfigurestartup(5,3,'paper');
hold on; grid on; axis equal;

% 1. 绘制潜标系统 (保持不变)
for i = 1:size(P_gnss,1)
    % 垂直投影辅助线
    h1 = plot3([P_gnss(i,1) P_gnss(i,1)], [P_gnss(i,2) P_gnss(i,2)], [0 H_emitter], 'k:', 'LineWidth', 1);
    % 发声点与垂直投影连线
    h2 = plot3([P_gnss(i,1) P_emitters_true(i,1)], [P_gnss(i,2) P_emitters_true(i,2)], [H_emitter H_emitter], 'r:', 'LineWidth', 1);
    % 实际电缆连线 (GNSS到发声点)
    h3 = plot3([P_gnss(i,1) P_emitters_true(i,1)], [P_gnss(i,2) P_emitters_true(i,2)], [0 H_emitter], 'r--', 'LineWidth', 1.5);
    % 节点标志
    h4 = scatter3(P_gnss(i,1), P_gnss(i,2), P_gnss(i,3), 15, 'b', 'filled');
    h5 = scatter3(P_emitters_true(i,1), P_emitters_true(i,2), P_emitters_true(i,3), 15, 'r', 'filled');
end

% 2. 绘制航行器 (处理 N*3 矩阵)
% 绘制运动轨迹线
h_v_path = plot3(P_v_true(:,1), P_v_true(:,2), P_v_true(:,3), 'm-', 'LineWidth', 1);
% 绘制离散观测点 (五角星)
h_v_pts = scatter3(P_v_true(:,1), P_v_true(:,2), P_v_true(:,3), 80, 'm', 'pentagram', 'filled');

% 3. 北向参考
h6 = quiver3(0, 0, 0, 0, 3000, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.8);

% --- 图例与修饰 ---
view(3);
xlabel('东向 East (m)'); ylabel('北向 North (m)'); zlabel('深度 Depth (m)');
title('潜标阵列与航行器空间布局仿真');

lgd = legend([h1, h2, h3, h4, h5, h6, h_v_pts], ...
    {'垂直参考线', '水平偏移投影', '电缆(漂移向量)', 'GNSS点', '发声点', '北向参考', '航行器观测点'}, ...
    'Location', 'eastoutside', 'FontSize', 8);
lgd.Box = 'on';

% --- 7. 打印输出数据 ---
num_v = size(P_v_true, 1); % 获取航行器点数
fprintf('================ 仿真数据输出 =================\n');
fprintf('设定漂移: Theta = %.2f°, Phi = %.2f°\n', true_theta_deg, true_phi_deg);
fprintf('-----------------------------------------------\n');
for i = 1:size(P_gnss,1)
    fprintf('潜标 %d GNSS坐标: [%10.2f, %10.2f, %7.2f]\n', i, P_gnss(i,1), P_gnss(i,2), P_gnss(i,3));
    fprintf('潜标 %d 发声坐标: [%10.2f, %10.2f, %7.2f]\n', i, P_emitters_true(i,1), P_emitters_true(i,2), P_emitters_true(i,3));
end
fprintf('-----------------------------------------------\n');
fprintf('航行器观测点总数: %d\n', num_v);
for j = 1:num_v
    fprintf('航行器位置 %d: [%10.2f, %10.2f, %7.2f]\n', j, P_v_true(j,1), P_v_true(j,2), P_v_true(j,3));
end
% 如果 R_obs 也是 N*3，可以取消下面注释
if size(P_gnss,1)~=1
    for j = 1:num_v
        fprintf('观测点 %d 测距: R1=%.2f, R2=%.2f, R3=%.2f\n', j, R_obs(1,j), R_obs(2,j), R_obs(3,j));
    end
else
    fprintf('观测点 %d 测距: R1=%.2f, R2=%.2f, R3=%.2f\n', j, R_obs(1), R_obs(2), R_obs(3));
end
fprintf('===============================================\n');

exportgraphics(fig, pngname, 'Resolution', 600);
end