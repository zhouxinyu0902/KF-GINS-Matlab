%% 深海潜标阵列与 AUV 两阶段(标定+直行)高保真仿真数据发生器 (含完整代码画图)
clear; clc; close all;
% 初始化 PSINS 全局变量工具箱
glvs; 

%% ==================== 1. 环境与潜标多坐标系位置生成 ====================
L = 20000;              % 等边三角形边长 20km
H_sea = 6000;           % 海深 6000m
d = [1000; 1015; 985];   % 三个潜标的实际发声点深度 (单位: m)

% --- 地理坐标原点选择：中国青岛坐标 ---
pos0_geo = [36.066667*glv.deg; 120.350000*glv.deg; 0]; 

% 理想海面布放点 (相对于阵列中心，本地 ENU 东北天直角坐标系，单位: 米)
R_tri = L / sqrt(3); 
P_surface_xyz = [
    0,           R_tri,      0;  % 1号潜标初始海面点 (正北顶点)
   -L/2, -R_tri*0.5,      0;  % 2号潜标初始海面点 (西南顶点)
    L/2, -R_tri*0.5,      0   % 3号潜标初始海面点 (东南顶点)
];

% 【潜标流场偏转真值设定】
theta_true = 20 * glv.deg;  % 缆绳倾角
phi_true   = 45 * glv.deg;   % 水平位移方位角 45°（向东北方向漂移）

S_true_xyz = zeros(3,3); S_true_geo = zeros(3,3);
S_gnss_xyz = zeros(3,3); S_gnss_geo = zeros(3,3);
for i = 1:3
    % 计算海流在水平面上引起的实际 X/Y 偏移量
    delta_x = d(i) * tan(theta_true) * sin(phi_true);
    delta_y = d(i) * tan(theta_true) * cos(phi_true);
    
    % 本地直角坐标
    S_true_xyz(i, :) = [P_surface_xyz(i,1) + delta_x, P_surface_xyz(i,2) + delta_y, -d(i)];
    S_gnss_xyz(i, :) = [P_surface_xyz(i,1), P_surface_xyz(i,2), -d(i)];

    % 转换为地理坐标
    S_true_geo(i, :) = dxyz2pos(S_true_xyz(i, :), pos0_geo)';
    S_gnss_geo(i, :) = dxyz2pos(S_gnss_xyz(i, :), pos0_geo)';
end

%% ==================== 2. 两阶段 AUV 真实参考轨迹生成 ====================
v_auv = 1.5;             % AUV 标定作业航速 1.5 m/s 
w_turn = 1;            % 降低转弯速率至 0.5 deg/s，使惯导零偏过渡更平稳
t_90  = 90 / w_turn;     % 90度转弯时间 180s

% --- 2.1 优化后的约30分钟闭环长方形标定轨迹 (阶段一) ---
L_long = 250;           % 南北长边 1800m (顺着偏转切向拉长，极大改善 GDOP)
L_short = 800;           % 东西短边 600m
t_long = L_long / v_auv;   % 1200 秒
t_short = L_short / v_auv; % 400 秒

% 方形航线起点规划 (本地 ENU 坐标系)：定在 1 号潜标附近，航行深度 1000m
% X_start = P_surface_xyz(1,1) + 300; 
% Y_start = P_surface_xyz(1,2) - 1500; 
% Z_start = -1000;         
X_start = 300; 
Y_start = 1500; 
Z_start = -1000;   
auv_start_geo = dxyz2pos([X_start, Y_start, Z_start], pos0_geo);

% 初始化 AVP 状态
att0 = [0; 0; pi/2]; vel0 = [-v_auv; 0;  0]; pos0 = auv_start_geo(:); 
avp0 = [att0; vel0; pos0]; 

% 拼接阶段一标准闭环长方形轨迹
seg1 = trjsegment([], 'init', v_auv);
seg1 = trjsegment(seg1, 'uniform', t_long);
seg1 = trjsegment(seg1, 'turnleft', t_90, w_turn); 
seg1 = trjsegment(seg1, 'uniform', t_short);
seg1 = trjsegment(seg1, 'turnleft', t_90, w_turn); 
seg1 = trjsegment(seg1, 'uniform', t_long);
seg1 = trjsegment(seg1, 'turnleft', t_90, w_turn); 
seg1 = trjsegment(seg1, 'uniform', t_short);
seg1 = trjsegment(seg1, 'turnleft', t_90, w_turn); 

ts = 0.01; 
trj1 = trjsimu(avp0, seg1.wat, ts, 1);
t_end_stage1 = trj1.avp(end, 10); 

% --- 2.2 拼接阶段二：长距离直行巡航轨迹 ---
% 承接阶段一的终点 AVP 状态
avp0_stage2 = trj1.avp(end, 1:9)'; 
seg2 = trjsegment([], 'init', v_auv);
seg2 = trjsegment(seg2, 'uniform', 5000); 
seg2 = trjsegment(seg2, 'turnleft', 180/0.5, 0.5); 
seg2 = trjsegment(seg2, 'uniform', 5000); 

trj2 = trjsimu(avp0_stage2, seg2.wat, ts, 1);
trj2.avp(:, 10) = trj2.avp(:, 10) + t_end_stage1; % 时间轴无缝衔接

% --- 2.3 分开保存惯导理论真值文本 ---
outputfolder = 'D:\Github\KF-GINS-Matlab\潜标位置标定\data2\';
if ~exist(outputfolder, 'dir'), mkdir(outputfolder); end 
pva1 = avpENU2NED(trj1.avp); pva2 = avpENU2NED(trj2.avp);
save([outputfolder, 'truth.nav'], 'pva1', '-ascii', '-double');
save([outputfolder, 'truth_stage2.nav'], 'pva2', '-ascii', '-double');

%% ==================== 3. 经典 MATLAB 代码闭环轨迹画图可视化 ====================
% fprintf('\n==================== 3. 正在提取坐标执行动态画图 ====================\n');

% 提取两阶段 AUV 地理轨迹，并利用工具箱逆向映射回本地 ENU 直角坐标系 [X, Y, Z] 
auv_geo_st1 = trj1.avp(:, 7:9);
auv_geo_st2 = trj2.avp(:, 7:9);
[auv_xyz_st1, ~, ~] = pos2dxyz(auv_geo_st1, pos0_geo);
[auv_xyz_st2, ~, ~] = pos2dxyz(auv_geo_st2, pos0_geo);

% 创建高清白色背景大图
myfigurestartup(7,3,'zxy');

% ---------------- 子图1：大洋尺度全景拓扑图 (20km 阵列全貌) ----------------
subplot(1,2,1);
% 绘制海面锚定基准点
plot(P_surface_xyz(:,1)/1000, P_surface_xyz(:,2)/1000, 'b^', 'MarkerSize', 8, 'LineWidth', 2); hold on;
% 绘制实际漂移形变后的水下实际发声点位置
plot(S_true_xyz(:,1)/1000, S_true_xyz(:,2)/1000, 'ro', 'MarkerSize', 7, 'MarkerFaceColor', 'r');
% 动态绘制 AUV 阶段一与阶段二轨迹
plot(auv_xyz_st1(:,1)/1000, auv_xyz_st1(:,2)/1000, 'm-', 'LineWidth', 2);
plot(auv_xyz_st2(:,1)/1000, auv_xyz_st2(:,2)/1000, 'g--', 'LineWidth', 2);
grid on; axis equal;
xlabel('本地东向距离 E (km)', 'FontSize', 11); ylabel('本地北向距离 N (km)', 'FontSize', 11);
title('深海全景图 (20km 潜标阵列拓扑与 AUV 轨迹流)', 'FontSize', 12);
legend('潜标海面锚定点(GNSS基准)', '潜标水下实际发声点', '阶段一：长方形闭环标定轨迹', '阶段二：长途直行巡航轨迹', 'Location', 'SouthWest');

% ---------------- 子图2：1号潜标附近的局部作业几何放大图 ----------------
subplot(1,2,2);
% 绘制海面名义点与实际发声点真值
plot(P_surface_xyz(1,1), P_surface_xyz(1,2), 'b^', 'MarkerSize', 11, 'LineWidth', 2.5); hold on;
plot(S_true_xyz(1,1), S_true_xyz(1,2), 'ro', 'MarkerSize', 9, 'MarkerFaceColor', 'r');
% 绘制流场形变带来的偏移矢量线（虚线）
plot([P_surface_xyz(1,1), S_true_xyz(1,1)], [P_surface_xyz(1,2), S_true_xyz(1,2)], 'r:', 'LineWidth', 2);

% 详细描绘 AUV 两阶段运行轨迹
plot(auv_xyz_st1(:,1), auv_xyz_st1(:,2), 'm-', 'LineWidth', 2.5);
plot(auv_xyz_st2(:,1), auv_xyz_st2(:,2), 'g--', 'LineWidth', 2.5);
% 标记关键状态切换打点位置
plot(auv_xyz_st1(1,1), auv_xyz_st1(1,2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); % 标定起点
plot(auv_xyz_st1(end,1), auv_xyz_st1(end,2), 'cx', 'MarkerSize', 10, 'LineWidth', 2.5); % 衔接切换点

grid on; axis equal;
xlabel('本地东向距离 E (m)', 'FontSize', 11); ylabel('本地北向距离 N (m)', 'FontSize', 11);
title('1号潜标作业区：两阶段航迹与形变矢量放大图', 'FontSize', 12);
legend('1号潜标海面基准点', '1号实际水下发声点', '海流钟摆偏移形变矢量', ...
       '阶段一：闭环长方形轨迹(约30min)', '阶段二：无缝切分直行轨迹', ...
       '标定作业起点', '阶段状态切换点', 'Location', 'Best');

%% ==================== 4. 惯导误差注入与水声数据流分阶段拆分保存 ====================
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

save D:\Github\KF-GINS-Matlab\潜标位置标定\data2\beacon_pos.mat S_true_geo S_gnss_geo S_true_xyz S_gnss_xyz pos0_geo theta_true phi_true;


%% ==================== 5. 辅助水声流生成子函数 ====================
function [r1, r2, r3] = generate_acoustic_stream(avp, pos0_geo, S_true_xyz, S_gnss_geo, ping_interval, sigma_r)
    ping_indices = 1:ping_interval:length(avp);
    N_pings = length(ping_indices);
    [auv_pos_xyz, ~, ~] = pos2dxyz(avp(:, 7:9), pos0_geo);
    r1 = zeros(N_pings, 6); r2 = zeros(N_pings, 6); r3 = zeros(N_pings, 6);
    for k = 1:N_pings
        idx = ping_indices(k); t_curr = avp(idx, 10); xyz_true = auv_pos_xyz(idx, :); 
        r_h_meas = zeros(3,1); r_slant_meas = zeros(3,1);
        for i = 1:3
            r_h_true     = norm(xyz_true(1:2) - S_true_xyz(i, 1:2)); 
            r_slant_true = norm(xyz_true - S_true_xyz(i, :));        
            v_k = normrnd(0, sigma_r); 
            r_h_meas(i)     = r_h_true + v_k;
            r_slant_meas(i) = r_slant_true + v_k;
        end
        r1(k, :) = [t_curr, r_slant_meas(1), r_h_meas(1), S_gnss_geo(1,1), S_gnss_geo(1,2), S_gnss_geo(1,3)];
        r2(k, :) = [t_curr, r_slant_meas(2), r_h_meas(2), S_gnss_geo(2,1), S_gnss_geo(2,2), S_gnss_geo(2,3)];
        r3(k, :) = [t_curr, r_slant_meas(3), r_h_meas(3), S_gnss_geo(3,1), S_gnss_geo(3,2), S_gnss_geo(3,3)];
    end
end