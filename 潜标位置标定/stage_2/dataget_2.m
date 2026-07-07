
%% 深海潜标阵列与 AUV 两阶段（标定 + 直行）高保真仿真数据发生器
% 功能：
% 1. 生成潜标阵列真值坐标与 GNSS 名义坐标；
% 2. 生成带 USBL 固定定位误差的潜标坐标；
% 3. 生成 AUV 标定阶段轨迹；
% 4. 生成 AUV 第二阶段直行/巡航轨迹；
% 5. 分别保存两个阶段的 truth、IMU、声学测距和潜标坐标数据；
% 6. 额外生成第二阶段带 USBL 潜标位置误差的三个距离文件；
% 7. 绘制全局轨迹图与局部放大图。

clear; clc;
% close all;

% 初始化 PSINS 全局变量工具箱
glvs;

%% ==================== 0. 工况配置与输出目录 ====================

% 轨迹形状选项：
% 'Square'  方形轨迹
% 'Line'    往复直线轨迹
% 'Circle'  圆形轨迹
% trj_shape = 'Circle';
trj_shape = 'Square';
% trj_shape = 'Line';
% 轨迹中心点选项：
% 'ArrayCenter' 阵列几何中心
% 'Beacon1'     1号潜标附近
% trj_center = 'ArrayCenter';
trj_center = 'Beacon1';

% 工况根目录
basefolder = fullfile( ...
    'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data', ...
    [trj_shape, '_', trj_center, '_Trj'] ...
    );

% 标定阶段数据目录
outputfolder_stage1 = fullfile(basefolder, 'input_stage1');

% 第二阶段数据目录
outputfolder_stage2 = fullfile(basefolder, 'input_stage2');

% 创建标定阶段目录
if ~exist(outputfolder_stage1, 'dir')
    mkdir(outputfolder_stage1);
    disp(['>>> 已创建标定阶段文件夹：', outputfolder_stage1]);
else
    disp(['>>> 追加到现有标定阶段文件夹：', outputfolder_stage1]);
end

% 创建第二阶段目录
if ~exist(outputfolder_stage2, 'dir')
    mkdir(outputfolder_stage2);
    disp(['>>> 已创建第二阶段文件夹：', outputfolder_stage2]);
else
    disp(['>>> 追加到现有第二阶段文件夹：', outputfolder_stage2]);
end


%% ==================== 1. 环境与潜标多坐标系位置生成 ====================

L = 20000;              % 等边三角形边长 20 km
H_sea = 6000;           % 海深 6000 m
d = [1000; 1015; 985];  % 三个潜标发声点深度，单位：m

% 局部坐标系原点对应的大地坐标
pos0_geo = [36.066667 * glv.deg; ...
            120.350000 * glv.deg; ...
            0];

% 三个潜标海面 GNSS 点构成等边三角形
R_tri = L / sqrt(3);

P_surface_xyz = [
    0,        R_tri,        0;   % 1号潜标初始海面点
   -L/2,     -R_tri * 0.5,  0;   % 2号潜标初始海面点
    L/2,     -R_tri * 0.5,  0    % 3号潜标初始海面点
];

% 潜标缆绳受海流作用后的偏转真值
theta_true = 20 * glv.deg;   % 偏垂角真值
phi_true   = 45 * glv.deg;   % 偏转方位角真值

% 初始化潜标真值坐标与 GNSS 名义坐标
S_true_xyz = zeros(3, 3);
S_true_geo = zeros(3, 3);
S_gnss_xyz = zeros(3, 3);
S_gnss_geo = zeros(3, 3);

for i = 1:3
    % 水下发声点相对于海面 GNSS 点的水平偏移
    delta_x = d(i) * tan(theta_true) * sin(phi_true);
    delta_y = d(i) * tan(theta_true) * cos(phi_true);

    % 水下发声点真实坐标
    S_true_xyz(i, :) = [
        P_surface_xyz(i, 1) + delta_x, ...
        P_surface_xyz(i, 2) + delta_y, ...
       -d(i)
    ];

    % GNSS 名义坐标，水平位置不考虑缆绳偏移，仅保留深度
    S_gnss_xyz(i, :) = [
        P_surface_xyz(i, 1), ...
        P_surface_xyz(i, 2), ...
       -d(i)
    ];

    % 转换为大地坐标
    S_true_geo(i, :) = dxyz2pos(S_true_xyz(i, :), pos0_geo)';
    S_gnss_geo(i, :) = dxyz2pos(S_gnss_xyz(i, :), pos0_geo)';
end


%% ==================== 1.1 生成带 USBL 固定误差的潜标位置 ====================
% 说明：
% S_true_xyz / S_true_geo ：真实水下发声点位置，用于生成真实声学测距；
% S_gnss_xyz / S_gnss_geo ：原始 GNSS 名义潜标位置；
% S_usbl_xyz / S_usbl_geo ：加入 USBL 固定定位误差后的潜标位置；
% 阶段2的 range*_usbl.txt 文件第 4-6 列将填充 S_usbl_geo。

% 三个潜标分别加入 5 m 量级的固定水平定位误差
% x 为东向误差，y 为北向误差，z 不加误差
usbl_err_xy = [
     3.0,   4.0;    % 1号潜标：水平误差 sqrt(3^2+4^2)=5 m
    -4.0,   3.0;    % 2号潜标：水平误差 5 m
     4.0,  -3.0     % 3号潜标：水平误差 5 m
];

usbl_err_xyz = [usbl_err_xy, zeros(3, 1)];

% 在原始 GNSS 名义位置基础上加入 USBL 固定定位误差
S_usbl_xyz = S_true_xyz + usbl_err_xyz;

% 转换为大地坐标，用于填充阶段2 range*_usbl.txt 的第 4-6 列
S_usbl_geo = zeros(3, 3);
for i = 1:3
    S_usbl_geo(i, :) = dxyz2pos(S_usbl_xyz(i, :), pos0_geo)';
end


%% ==================== 2. 标定阶段轨迹生成 ====================

v_auv = 1.5;       % AUV 航速，单位：m/s
w_turn = 1;        % 转弯角速度，单位：deg/s
Z_start = -1000;   % AUV 初始深度，单位：m

% 确定标定航线规划的中心点
if strcmp(trj_center, 'ArrayCenter')
    Cx = 0;
    Cy = 0;
elseif strcmp(trj_center, 'Beacon1')
    Cx = P_surface_xyz(1, 1);
    Cy = P_surface_xyz(1, 2) + 600;
else
    error('未知的中心点设置，请选择 ArrayCenter 或 Beacon1');
end

% 初始化标定阶段轨迹段
seg1 = trjsegment([], 'init', v_auv);

switch trj_shape

    case 'Square'
        % 闭合方形轨迹
        % 边长 500 m，总耗时约 28.2 min
        L_side = 500;
        t_side = L_side / v_auv;

        X_start = Cx + L_side / 2;
        Y_start = Cy - L_side / 2;

        att0 = [0; 0; pi/2];
        vel0 = [-v_auv; 0; 0];

        for k = 1:4
            seg1 = trjsegment(seg1, 'uniform', t_side);
            seg1 = trjsegment(seg1, 'turnleft', 90 / w_turn, w_turn);
        end

    case 'Line'
        % 往复直线轨迹
        % 单程 1200 m，总耗时约 29.6 min
        L_line = 1200;
        t_line = L_line / v_auv;

        X_start = Cx;
        Y_start = Cy - L_line / 2;

        att0 = [0; 0; -pi/2];
        vel0 = [v_auv; 0; 0];

        seg1 = trjsegment(seg1, 'uniform', t_line);
        seg1 = trjsegment(seg1, 'turnleft', 180 / w_turn, w_turn);
        seg1 = trjsegment(seg1, 'uniform', t_line);

    case 'Circle'
        % 闭合圆形轨迹
        % 半径 400 m，总耗时约 27.9 min
        R_circle = 400;
        w_circle = (v_auv / R_circle) * (180 / pi);
        t_360 = 360 / w_circle;

        X_start = Cx + R_circle;
        Y_start = Cy;

        att0 = [0; 0; pi/2];
        vel0 = [-v_auv; 0; 0];

        seg1 = trjsegment(seg1, 'turnleft', t_360, w_circle);

    otherwise
        error('未知的轨迹形状，请在 Square、Line、Circle 中选择');
end

% 转换 AUV 初始位置并生成标定阶段轨迹
auv_start_geo = dxyz2pos([X_start, Y_start, Z_start], pos0_geo);
pos0 = auv_start_geo(:);
avp0 = [att0; vel0; pos0];

ts = 0.01;   % 仿真采样周期，单位：s

trj1 = trjsimu(avp0, seg1.wat, ts, 1);


%% ==================== 3. 第二阶段直行/巡航轨迹生成 ====================

% 第二阶段从标定阶段末端状态开始
avp0_stage2 = trj1.avp(end, 1:9)';

seg2 = trjsegment([], 'init', v_auv);

% 第二阶段：长距离直行 + 掉头 + 长距离返回
seg2 = trjsegment(seg2, 'uniform', 5000);
seg2 = trjsegment(seg2, 'turnleft', 180 / 0.5, 0.5);
seg2 = trjsegment(seg2, 'uniform', 5000);

trj2 = trjsimu(avp0_stage2, seg2.wat, ts, 1);


%% ==================== 4. 保存两阶段轨迹真值 ====================

% ENU 转 NED 格式
pva1 = avpENU2NED(trj1.avp);
pva2 = avpENU2NED(trj2.avp);

% 保存标定阶段真值
save(fullfile(outputfolder_stage1, 'truth.nav'), ...
    'pva1', '-ascii', '-double');

% 保存第二阶段真值
save(fullfile(outputfolder_stage2, 'truth.nav'), ...
    'pva2', '-ascii', '-double');


%% ==================== 5. 轨迹可视化 ====================

auv_geo_st1 = trj1.avp(:, 7:9);
auv_geo_st2 = trj2.avp(:, 7:9);

[auv_xyz_st1, ~, ~] = pos2dxyz(auv_geo_st1, pos0_geo);
[auv_xyz_st2, ~, ~] = pos2dxyz(auv_geo_st2, pos0_geo);

myfigurestartup(7, 3, 'zxy');

% ---------- 全局图 ----------
subplot(1, 2, 1);

plot(P_surface_xyz(:, 1) / 1000, ...
     P_surface_xyz(:, 2) / 1000, ...
     'b^', 'MarkerSize', 8, 'LineWidth', 2);
hold on;

plot(S_true_xyz(:, 1) / 1000, ...
     S_true_xyz(:, 2) / 1000, ...
     'ro', 'MarkerSize', 7, 'MarkerFaceColor', 'r');

plot(auv_xyz_st1(:, 1) / 1000, ...
     auv_xyz_st1(:, 2) / 1000, ...
     'm-', 'LineWidth', 2);

plot(auv_xyz_st2(:, 1) / 1000, ...
     auv_xyz_st2(:, 2) / 1000, ...
     'g--', 'LineWidth', 2);

grid on;
axis equal;

xlabel('东向 E (km)');
ylabel('北向 N (km)');
title(['全景图: ', trj_shape, ' @ ', trj_center]);

legend( ...
    '潜标海面点', ...
    '水下发声点', ...
    '标定轨迹', ...
    '巡航轨迹', ...
    'Location', 'SouthWest' ...
    );

% ---------- 局部放大图 ----------
subplot(1, 2, 2);

plot(P_surface_xyz(1, 1), ...
     P_surface_xyz(1, 2), ...
     'b^', 'MarkerSize', 11, 'LineWidth', 2.5);
hold on;

plot(S_true_xyz(1, 1), ...
     S_true_xyz(1, 2), ...
     'ro', 'MarkerSize', 9, 'MarkerFaceColor', 'r');

plot(auv_xyz_st1(:, 1), ...
     auv_xyz_st1(:, 2), ...
     'm-', 'LineWidth', 2.5);

plot(auv_xyz_st2(:, 1), ...
     auv_xyz_st2(:, 2), ...
     'g--', 'LineWidth', 2.5);

grid on;
axis equal;

xlabel('东向 E (m)');
ylabel('北向 N (m)');
title('标定作业区局部放大图');

% 局部显示范围，避免被第二阶段长直航线拉远视野
xlim([Cx - 2000, Cx + 2000]);
ylim([Cy - 2000, Cy + 2000]);


%% ==================== 6. 传感器误差注入 ====================

% IMU 误差参数
eb = 0.003;       % 陀螺零偏，单位通常为 deg/h
db = 10;          % 加速度计零偏，单位通常为 ug
web = 0.0002;
wdb = 7;

sqrtR0G = 0.01;
TauG = 300;
sqrtR0A = 10;
TauA = 300;

dKGii = 5;
dKAii = 20;
dKGij = 10;
dKAij = 10;

KA2 = 0;
rxyz = 0;
dtGA = 0;

imuerr = imuerrset( ...
    eb, db, web, wdb, ...
    sqrtR0G, TauG, ...
    sqrtR0A, TauA, ...
    dKGii, dKAii, ...
    dKGij, dKAij, ...
    KA2, rxyz, dtGA ...
    );

% 向两个阶段 IMU 真值中注入误差，并转换坐标系
imu1_FRD = imuRFU2FRD(imuadderr(trj1.imu, imuerr));
imu2_FRD = imuRFU2FRD(imuadderr(trj2.imu, imuerr));

% 保存标定阶段 IMU 数据
save(fullfile(outputfolder_stage1, 'imu_data.txt'), ...
    'imu1_FRD', '-ascii', '-double');

% 保存第二阶段 IMU 数据
save(fullfile(outputfolder_stage2, 'imu_data.txt'), ...
    'imu2_FRD', '-ascii', '-double');


%% ==================== 7. 声学测距数据生成与保存 ====================

t_ping = 20;                          % 测距周期，单位：s
ping_interval = round(t_ping / ts);   % 测距采样间隔
sigma_r = 10;                         % 测距噪声标准差，单位：m

% 标定阶段声学测距数据
[range1_st1, range2_st1, range3_st1] = generate_acoustic_stream( ...
    trj1.avp, pos0_geo, S_true_xyz, S_gnss_geo, ping_interval, sigma_r);

% 第二阶段声学测距数据
[range1_st2, range2_st2, range3_st2] = generate_acoustic_stream( ...
    trj2.avp, pos0_geo, S_true_xyz, S_gnss_geo, ping_interval, sigma_r);

% 保存标定阶段原始声学测距数据
save(fullfile(outputfolder_stage1, 'range1.txt'), ...
    'range1_st1', '-ascii', '-double');

save(fullfile(outputfolder_stage1, 'range2.txt'), ...
    'range2_st1', '-ascii', '-double');

save(fullfile(outputfolder_stage1, 'range3.txt'), ...
    'range3_st1', '-ascii', '-double');

% 保存第二阶段原始声学测距数据
save(fullfile(outputfolder_stage2, 'range1.txt'), ...
    'range1_st2', '-ascii', '-double');

save(fullfile(outputfolder_stage2, 'range2.txt'), ...
    'range2_st2', '-ascii', '-double');

save(fullfile(outputfolder_stage2, 'range3.txt'), ...
    'range3_st2', '-ascii', '-double');


%% ==================== 7.1 额外生成第二阶段带 USBL 潜标位置误差的三个距离文件 ====================
% 说明：
% 这里只额外生成第二阶段的 3 个 USBL 误差版本距离文件：
% range1_usbl.txt、range2_usbl.txt、range3_usbl.txt。
%
% 它们与第二阶段原始 range1.txt、range2.txt、range3.txt 的区别是：
% 第 1 列：时间，不变；
% 第 2 列：斜距测量值，不变；
% 第 3 列：水平距测量值，不变；
% 第 4-6 列：潜标位置，替换为带 USBL 固定定位误差的位置。

range1_st2_usbl = range1_st2;
range2_st2_usbl = range2_st2;
range3_st2_usbl = range3_st2;

% 将第 4-6 列替换为带 USBL 固定误差的潜标坐标
range1_st2_usbl(:, 4:6) = repmat(S_usbl_geo(1, :), size(range1_st2_usbl, 1), 1);
range2_st2_usbl(:, 4:6) = repmat(S_usbl_geo(2, :), size(range2_st2_usbl, 1), 1);
range3_st2_usbl(:, 4:6) = repmat(S_usbl_geo(3, :), size(range3_st2_usbl, 1), 1);

% 额外保存三个第二阶段 USBL 误差版本距离文件
save(fullfile(outputfolder_stage2, 'range1_usbl.txt'), ...
    'range1_st2_usbl', '-ascii', '-double');

save(fullfile(outputfolder_stage2, 'range2_usbl.txt'), ...
    'range2_st2_usbl', '-ascii', '-double');

save(fullfile(outputfolder_stage2, 'range3_usbl.txt'), ...
    'range3_st2_usbl', '-ascii', '-double');


%% ==================== 8. 保存潜标坐标参数 ====================

% 两个阶段各保存一份 beacon_pos.mat，方便后续程序独立读取
% 同时保存 S_usbl_xyz / S_usbl_geo / usbl_err_xyz，方便复现实验
save(fullfile(outputfolder_stage1, 'beacon_pos.mat'), ...
    'S_true_geo', ...
    'S_gnss_geo', ...
    'S_usbl_geo', ...
    'S_true_xyz', ...
    'S_gnss_xyz', ...
    'S_usbl_xyz', ...
    'usbl_err_xy', ...
    'usbl_err_xyz', ...
    'P_surface_xyz', ...
    'pos0_geo', ...
    'theta_true', ...
    'phi_true', ...
    'd', ...
    'H_sea' ...
    );

save(fullfile(outputfolder_stage2, 'beacon_pos.mat'), ...
    'S_true_geo', ...
    'S_gnss_geo', ...
    'S_usbl_geo', ...
    'S_true_xyz', ...
    'S_gnss_xyz', ...
    'S_usbl_xyz', ...
    'usbl_err_xy', ...
    'usbl_err_xyz', ...
    'P_surface_xyz', ...
    'pos0_geo', ...
    'theta_true', ...
    'phi_true', ...
    'd', ...
    'H_sea' ...
    );


%% ==================== 9. 完成提示 ====================

disp('>>> 仿真完成！');
disp(['>>> 标定阶段数据已保存至：', outputfolder_stage1]);
disp(['>>> 第二阶段数据已保存至：', outputfolder_stage2]);
disp('>>> 已额外生成第二阶段 USBL 误差版本距离文件：');
disp(['>>> ', fullfile(outputfolder_stage2, 'range1_usbl.txt')]);
disp(['>>> ', fullfile(outputfolder_stage2, 'range2_usbl.txt')]);
disp(['>>> ', fullfile(outputfolder_stage2, 'range3_usbl.txt')]);


%% ==================== 10. 子函数：声学测距流生成 ====================

function [r1, r2, r3] = generate_acoustic_stream( ...
    avp, pos0_geo, S_true_xyz, S_gnss_geo, ping_interval, sigma_r)

    ping_indices = 1:ping_interval:length(avp);
    N_pings = length(ping_indices);

    [auv_pos_xyz, ~, ~] = pos2dxyz(avp(:, 7:9), pos0_geo);

    r1 = zeros(N_pings, 6);
    r2 = zeros(N_pings, 6);
    r3 = zeros(N_pings, 6);

    for k = 1:N_pings

        idx = ping_indices(k);
        t_curr = avp(idx, 10);
        xyz_true = auv_pos_xyz(idx, :);

        r_h_meas = zeros(3, 1);
        r_slant_meas = zeros(3, 1);

        for i = 1:3

            % 声学测距噪声
            v_k = normrnd(0, sigma_r);

            % 水平距离测量值
            r_h_meas(i) = norm( ...
                xyz_true(1:2) - S_true_xyz(i, 1:2) ...
                ) + v_k;

            % 斜距测量值
            r_slant_meas(i) = norm( ...
                xyz_true - S_true_xyz(i, :) ...
                ) + v_k;
        end

        % 每个潜标对应一个 range 文件
        % 数据格式：
        % [时间, 斜距测量值, 水平距测量值, 潜标纬度, 潜标经度, 潜标高度]
        r1(k, :) = [
            t_curr, ...
            r_slant_meas(1), ...
            r_h_meas(1), ...
            S_gnss_geo(1, 1), ...
            S_gnss_geo(1, 2), ...
            S_gnss_geo(1, 3)
        ];

        r2(k, :) = [
            t_curr, ...
            r_slant_meas(2), ...
            r_h_meas(2), ...
            S_gnss_geo(2, 1), ...
            S_gnss_geo(2, 2), ...
            S_gnss_geo(2, 3)
        ];

        r3(k, :) = [
            t_curr, ...
            r_slant_meas(3), ...
            r_h_meas(3), ...
            S_gnss_geo(3, 1), ...
            S_gnss_geo(3, 2), ...
            S_gnss_geo(3, 3)
        ];
    end
end

