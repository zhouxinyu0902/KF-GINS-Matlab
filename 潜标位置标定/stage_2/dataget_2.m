%% 深海潜标阵列与 AUV 两阶段(标定+直行)高保真仿真数据发生器 (含完整代码画图)
clear; clc; 
% close all;
% 初始化 PSINS 全局变量工具箱
glvs;

%% ==================== [核心修改] 6种轨迹组合配置 ====================
% 1. 轨迹形状选项: 'Square' (方形), 'Line' (直线), 'Circle' (圆形)
trj_shape = 'Circle';

% 2. 轨迹中心点选项: 'ArrayCenter' (阵列几何中心), 'Beacon1' (1号潜标附近)
trj_center = 'Beacon1';

% 动态构建输出目录 (例如: data3/Square_Beacon1_Trj/)
outputfolder = ['D:\Github\KF-GINS-Matlab\潜标位置标定\', trj_shape, '_', trj_center, '_Trj\input\'];
if ~exist(outputfolder, 'dir')
    mkdir(outputfolder);
    disp(['>>> 已创建新工况文件夹：', outputfolder]);
else
    disp(['>>> 追加到现有文件夹：', outputfolder]);
end

%% ==================== 1. 环境与潜标多坐标系位置生成 ====================
L = 20000;              % 等边三角形边长 20km
H_sea = 6000;           % 海深 6000m
d = [1000; 1015; 985];  % 发声点深度 (m)
pos0_geo = [36.066667*glv.deg; 120.350000*glv.deg; 0];

R_tri = L / sqrt(3);
P_surface_xyz = [
    0,           R_tri,      0;  % 1号潜标初始海面点
    -L/2, -R_tri*0.5,      0;  % 2号潜标
    L/2, -R_tri*0.5,      0   % 3号潜标
    ];

% 潜标流场偏转真值
theta_true = 20 * glv.deg;
phi_true   = 45 * glv.deg;

S_true_xyz = zeros(3,3); S_true_geo = zeros(3,3);
S_gnss_xyz = zeros(3,3); S_gnss_geo = zeros(3,3);
for i = 1:3
    delta_x = d(i) * tan(theta_true) * sin(phi_true);
    delta_y = d(i) * tan(theta_true) * cos(phi_true);

    S_true_xyz(i, :) = [P_surface_xyz(i,1) + delta_x, P_surface_xyz(i,2) + delta_y, -d(i)];
    S_gnss_xyz(i, :) = [P_surface_xyz(i,1), P_surface_xyz(i,2), -d(i)];
    S_true_geo(i, :) = dxyz2pos(S_true_xyz(i, :), pos0_geo)';
    S_gnss_geo(i, :) = dxyz2pos(S_gnss_xyz(i, :), pos0_geo)';
end

%% ==================== 2. 严格控制在30分钟内的标定轨迹生成 ====================
v_auv = 1.5;
w_turn = 1;
Z_start = -1000;

% 确定航线规划的基准中心点 (Cx, Cy)
if strcmp(trj_center, 'ArrayCenter')
    Cx = 0; Cy = 0;
elseif strcmp(trj_center, 'Beacon1')
    Cx = P_surface_xyz(1,1); Cy = P_surface_xyz(1,2)+600;
else
    error('未知的中心点设置，请选择 ArrayCenter 或 Beacon1');
end

seg1 = trjsegment([], 'init', v_auv);

switch trj_shape
    case 'Square'
        % 方形轨迹 (边长500m，总耗时约28.2分钟)
        L_side = 500;
        t_side = L_side / v_auv;
        X_start = Cx + L_side/2; Y_start = Cy - L_side/2;
        att0 = [0; 0; pi/2]; vel0 = [-v_auv; 0; 0];

        for k = 1:4
            seg1 = trjsegment(seg1, 'uniform', t_side);
            seg1 = trjsegment(seg1, 'turnleft', 90/w_turn, w_turn);
        end

    case 'Line'
        % 直线来回 (单程1200m，总耗时约29.6分钟)
        L_line = 1200;
        t_line = L_line / v_auv;
        X_start = Cx; Y_start = Cy - L_line/2;
        att0 = [0; 0; -pi/2]; vel0 = [v_auv; 0; 0];

        seg1 = trjsegment(seg1, 'uniform', t_line);
        seg1 = trjsegment(seg1, 'turnleft', 180/w_turn, w_turn); % U型掉头
        seg1 = trjsegment(seg1, 'uniform', t_line);

    case 'Circle'
        % 圆形轨迹 (半径400m，总耗时约27.9分钟)
        R_circle = 400;
        w_circle = (v_auv / R_circle) * (180/pi);
        t_360 = 360 / w_circle;
        X_start = Cx + R_circle; Y_start = Cy;
        att0 = [0; 0; pi/2]; vel0 = [-v_auv; 0; 0];

        seg1 = trjsegment(seg1, 'turnleft', t_360, w_circle);

    otherwise
        error('未知的轨迹形状，请在 Square, Line, Circle 中选择');
end

% 转换起点并生成轨迹
auv_start_geo = dxyz2pos([X_start, Y_start, Z_start], pos0_geo);
pos0 = auv_start_geo(:);
avp0 = [att0; vel0; pos0];

ts = 0.01;
trj1 = trjsimu(avp0, seg1.wat, ts, 1);

% --- 拼接阶段二：长距离直行 ---
avp0_stage2 = trj1.avp(end, 1:9)';
seg2 = trjsegment([], 'init', v_auv);
seg2 = trjsegment(seg2, 'uniform', 5000);
seg2 = trjsegment(seg2, 'turnleft', 180/0.5, 0.5);
seg2 = trjsegment(seg2, 'uniform', 5000);
trj2 = trjsimu(avp0_stage2, seg2.wat, ts, 1);

% 保存轨迹真值
pva1 = avpENU2NED(trj1.avp); pva2 = avpENU2NED(trj2.avp);
save([outputfolder, 'truth.nav'], 'pva1', '-ascii', '-double');
save([outputfolder, 'truth_stage2.nav'], 'pva2', '-ascii', '-double');

%% ==================== 3. 闭环轨迹画图可视化 ====================
auv_geo_st1 = trj1.avp(:, 7:9);
auv_geo_st2 = trj2.avp(:, 7:9);
[auv_xyz_st1, ~, ~] = pos2dxyz(auv_geo_st1, pos0_geo);
[auv_xyz_st2, ~, ~] = pos2dxyz(auv_geo_st2, pos0_geo);

myfigurestartup(7,3,'zxy');

% 大洋全景图
subplot(1,2,1);
plot(P_surface_xyz(:,1)/1000, P_surface_xyz(:,2)/1000, 'b^', 'MarkerSize', 8, 'LineWidth', 2); hold on;
plot(S_true_xyz(:,1)/1000, S_true_xyz(:,2)/1000, 'ro', 'MarkerSize', 7, 'MarkerFaceColor', 'r');
plot(auv_xyz_st1(:,1)/1000, auv_xyz_st1(:,2)/1000, 'm-', 'LineWidth', 2);
plot(auv_xyz_st2(:,1)/1000, auv_xyz_st2(:,2)/1000, 'g--', 'LineWidth', 2);
grid on; axis equal;
xlabel('东向 E (km)'); ylabel('北向 N (km)');
title(['全景图: ', trj_shape, ' @ ', trj_center]);
legend('潜标海面点', '水下发声点', '标定轨迹', '巡航轨迹', 'Location', 'SouthWest');

% 局部放大图 (根据当前选定的中心点动态调节视野边界)
subplot(1,2,2);
plot(P_surface_xyz(1,1), P_surface_xyz(1,2), 'b^', 'MarkerSize', 11, 'LineWidth', 2.5); hold on;
plot(S_true_xyz(1,1), S_true_xyz(1,2), 'ro', 'MarkerSize', 9, 'MarkerFaceColor', 'r');
plot(auv_xyz_st1(:,1), auv_xyz_st1(:,2), 'm-', 'LineWidth', 2.5);
plot(auv_xyz_st2(:,1), auv_xyz_st2(:,2), 'g--', 'LineWidth', 2.5);
grid on; axis equal;
xlabel('东向 E (m)'); ylabel('北向 N (m)');
title('标定作业区局部放大图');
% 设置局部显示范围，避免被 5000m 的长直轨迹拉远视野
xlim([Cx - 2000, Cx + 2000]);
ylim([Cy - 2000, Cy + 2000]);

%% ==================== 4. 传感器误差注入 ====================
eb = 0.003; db = 10; web = 0.0002; wdb = 7;
sqrtR0G = 0.01; TauG = 300; sqrtR0A = 10; TauA = 300;
dKGii = 5; dKAii = 20; dKGij = 10; dKAij = 10; KA2 = 0; rxyz = 0; dtGA = 0;
imuerr = imuerrset(eb, db, web, wdb, sqrtR0G, TauG, sqrtR0A, TauA, dKGii, dKAii, dKGij, dKAij, KA2, rxyz, dtGA);

imu1_FRD = imuRFU2FRD(imuadderr(trj1.imu, imuerr));
imu2_FRD = imuRFU2FRD(imuadderr(trj2.imu, imuerr));
save([outputfolder, 'imu_data.txt'], 'imu1_FRD', '-ascii', '-double');
save([outputfolder, 'imu_data_stage2.txt'], 'imu2_FRD', '-ascii', '-double');

t_ping = 20; ping_interval = round(t_ping / ts); sigma_r = 10;
[range1_st1, range2_st1, range3_st1] = generate_acoustic_stream(trj1.avp, pos0_geo, S_true_xyz, S_gnss_geo, ping_interval, sigma_r);
[range1_st2, range2_st2, range3_st2] = generate_acoustic_stream(trj2.avp, pos0_geo, S_true_xyz, S_gnss_geo, ping_interval, sigma_r);

save([outputfolder, 'range1.txt'], 'range1_st1', '-ascii', '-double');
save([outputfolder, 'range2.txt'], 'range2_st1', '-ascii', '-double');
save([outputfolder, 'range3.txt'], 'range3_st1', '-ascii', '-double');
save([outputfolder, 'range1_stage2.txt'], 'range1_st2', '-ascii', '-double');
save([outputfolder, 'range2_stage2.txt'], 'range2_st2', '-ascii', '-double');
save([outputfolder, 'range3_stage2.txt'], 'range3_st2', '-ascii', '-double');
save([outputfolder, 'beacon_pos.mat'], 'S_true_geo', 'S_gnss_geo', 'S_true_xyz', 'S_gnss_xyz', 'pos0_geo', 'theta_true', 'phi_true');

disp(['>>> 仿真完成！数据已保存至：', outputfolder]);

%% ==================== 5. 子函数 ====================
function [r1, r2, r3] = generate_acoustic_stream(avp, pos0_geo, S_true_xyz, S_gnss_geo, ping_interval, sigma_r)
ping_indices = 1:ping_interval:length(avp);
N_pings = length(ping_indices);
[auv_pos_xyz, ~, ~] = pos2dxyz(avp(:, 7:9), pos0_geo);
r1 = zeros(N_pings, 6); r2 = zeros(N_pings, 6); r3 = zeros(N_pings, 6);
for k = 1:N_pings
    idx = ping_indices(k); t_curr = avp(idx, 10); xyz_true = auv_pos_xyz(idx, :);
    r_h_meas = zeros(3,1); r_slant_meas = zeros(3,1);
    for i = 1:3
        v_k = normrnd(0, sigma_r);
        r_h_meas(i)     = norm(xyz_true(1:2) - S_true_xyz(i, 1:2)) + v_k;
        r_slant_meas(i) = norm(xyz_true - S_true_xyz(i, :)) + v_k;
    end
    r1(k, :) = [t_curr, r_slant_meas(1), r_h_meas(1), S_gnss_geo(1,1), S_gnss_geo(1,2), S_gnss_geo(1,3)];
    r2(k, :) = [t_curr, r_slant_meas(2), r_h_meas(2), S_gnss_geo(2,1), S_gnss_geo(2,2), S_gnss_geo(2,3)];
    r3(k, :) = [t_curr, r_slant_meas(3), r_h_meas(3), S_gnss_geo(3,1), S_gnss_geo(3,2), S_gnss_geo(3,3)];
end
end