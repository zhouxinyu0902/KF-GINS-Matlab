%% 深海潜标阵列与 AUV 微型方形轨迹高保真仿真数据发生器 (青岛中心地理坐标系版)
% 阶段一：纯粹运行【潜标位置解算】与【AUV理想参考轨迹生成】
clear; clc; close all;
% 初始化 PSINS 全局变量工具箱
glvs; 

%% ==================== 1. 环境与潜标多坐标系位置生成 ====================
L = 20000;              % 等边三角形边长 20km
H_sea = 6000;           % 海深 6000m
d = [1000; 1015; 985];   % 三个潜标的实际发声点深度 (由内置压力计测得，已知量，单位: m)

% --- 地理坐标原点选择：中国青岛坐标 ---
pos0_geo = [36.066667*glv.deg; 120.350000*glv.deg; 0]; 

% 理想海面布放点 (相对于阵列中心，本地 ENU 东北天直角坐标系，单位: 米)
R_tri = L / sqrt(3); 
P_surface_xyz = [
    0,           R_tri,      0;  % 1号潜标初始海面点 (正北顶点)
   -L/2, -R_tri*0.5,      0;  % 2号潜标初始海面点 (西南顶点)
    L/2, -R_tri*0.5,      0   % 3号潜标初始海面点 (东南顶点)
];

% 【潜标流场偏转真值设定】假定该海域流场导致潜标产生以下偏转
theta_true = 20 * glv.deg;  % 缆绳倾角 3.5°
phi_true   = 45 * glv.deg;   % 水平位移方位角 45°（向东北方向漂移）

% 预分配潜标多维位置存储空间
S_true_xyz = zeros(3,3);
S_true_geo = zeros(3,3);
for i = 1:3
    % 计算海流在水平面上引起的实际 X/Y 偏移量
    delta_x = d(i) * tan(theta_true) * sin(phi_true);
    delta_y = d(i) * tan(theta_true) * cos(phi_true);
    
    % 本地直角坐标
    S_true_xyz(i, :) = [P_surface_xyz(i,1) + delta_x, P_surface_xyz(i,2) + delta_y, -d(i)];
    S_gnss_xyz(i, :) = [P_surface_xyz(i,1), P_surface_xyz(i,2), -d(i)];

    % 调用 PSINS 工具箱将本地直角坐标转换为青岛基准下的地理大圆弧坐标 [Lat; Lon; Hgt]
    S_true_geo(i, :) = dxyz2pos(S_true_xyz(i, :), pos0_geo)';
    S_gnss_geo(i, :) = dxyz2pos(S_gnss_xyz(i, :), pos0_geo)';
end

% 打印核对潜标绝对地理坐标
fprintf('==================== 1. 青岛基准：潜标地理坐标生成结果 ====================\n');
for i = 1:3
    fprintf('潜标%d 真实位置: Lat = %.6f°, Lon = %.6f°, Hgt = %.1fm\n', ...
        i, S_true_geo(i,1)/glv.deg, S_true_geo(i,2)/glv.deg, S_true_geo(i,3));
end

%% ==================== 2. AUV 真实参考轨迹生成 (基于 PSINS trjsimu) ====================
v_auv = 1.5;             % AUV 标定作业航速 1.5 m/s 
L_side = 600;            % 局部微型方形轨迹边长 600m
t_side = L_side / v_auv; % 单边直线航行时间 400 秒
w_turn = 1.0;            % AUV 转弯速率 1.0 deg/s
t_90  = 90 / w_turn;     % 90度定时转弯时间 90 秒

% 方形航线起点规划 (本地 ENU 坐标系)：定在 1 号潜标原点西南方，航行深度 1000m
X_start = P_surface_xyz(1,1) + 300; 
Y_start = P_surface_xyz(1,2) - 300; 
Z_start = -1000;         

% 将本地 AUV 直角起点转换为绝对青岛地理坐标
auv_start_geo = dxyz2pos([X_start, Y_start, Z_start], pos0_geo);

% 初始化 AUV 姿态、速度、位置 (AVP) 状态向量
att0 = [0; 0; 0];         % 理想初始姿态：设定初始航向正北 (0°)
vel0 = [0; v_auv; 0];    % 理想初始速度：沿北向航行 (Vn = 1.5 m/s)
pos0 = auv_start_geo(:); % AUV 绝对地理坐标起点 [rad; rad; m]
avp0 = [att0; vel0; pos0]; 

% 拼接标准闭环正方形轨迹段
seg = trjsegment([], 'init', v_auv);
for edge = 1:4
    seg = trjsegment(seg, 'uniform', t_side);
    seg = trjsegment(seg, 'turnleft', t_90, w_turn); 
end

% 调用 PSINS 轨迹发生器生成高精度理论真实轨迹
ts = 0.01; 
trj = trjsimu(avp0, seg.wat, ts, 1);

% 【核心导出】：将 AUV 真实参考轨迹转换为标准 NED(北东地) 格式并保存到文件 truth.nav
outputfolder = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_1\data1\';
if ~exist(outputfolder, 'dir'), mkdir(outputfolder); end % 若路径不存在则自动创建
file_ref_100 = [outputfolder, 'truth.nav'];
pva = avpENU2NED(trj.avp);
save(file_ref_100, 'pva', '-ascii', '-double');

fprintf('\n==================== 2. AUV 轨迹生成结果 ====================\n');
fprintf('AUV 青岛外海真实参考轨迹已成功保存至: truth.nav\n');
fprintf('总仿真时长: %.2f 秒，总数据点数: %d 点\n', trj.avp(end,10), length(trj.avp));

%% ==================== 3. 纯理想参考轨迹可视化 ====================
auv_pos_geo_true = trj.avp(:, 7:9);
[auv_pos_xyz_true, ~, ~] = pos2dxyz(auv_pos_geo_true, pos0_geo);

% figure('Color',[1 1 1], 'Position', [150, 150, 1100, 500]);
myfigurestartup(7, 3, 'zxy');
% 子图1：大洋尺度全景图
subplot(1,2,1);
plot(P_surface_xyz(:,1)/1000, P_surface_xyz(:,2)/1000, 'kv'); hold on;
plot(S_true_xyz(:,1)/1000, S_true_xyz(:,2)/1000, 'ro');
plot(auv_pos_xyz_true(:,1)/1000, auv_pos_xyz_true(:,2)/1000, 'b-', 'LineWidth', 2);
grid on; axis equal;
xlabel('本地东向距离 X (km)'); ylabel('本地北向距离 Y (km)');
title('20km 潜标阵列全景');
legend('潜标海面锚定点', '潜标发声点(真值)', 'AUV真实方形参考轨迹', 'Location', 'SouthWest');

% 子图2：1号潜标附近的局部高保真轨迹放大图
subplot(1,2,2);
plot(P_surface_xyz(1,1), P_surface_xyz(1,2), 'kv', 'MarkerSize', 10, 'LineWidth', 2); hold on;
plot(S_true_xyz(1,1), S_true_xyz(1,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(auv_pos_xyz_true(:,1), auv_pos_xyz_true(:,2), 'b-', 'LineWidth', 2);
plot(auv_pos_xyz_true(1,1), auv_pos_xyz_true(1,2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
grid on; axis equal;
xlabel('本地东向距离 X (m)'); ylabel('本地北向距离 Y (m)');
title('1号潜标附近 AUV 方形标定轨迹放大图');
legend('1号潜标海面点', '1号实际发声点', 'AUV真实方形轨迹', '方形起点', 'Location', 'Best');

%% ==================== 3. FN-120 级光纤惯导数据仿真与机理更新 ====================
% 根据 FN-120 物理手册性能指标配置传感器级误差结构体
eb      = 0.003;    % 陀螺常值零偏 (deg/h)
db      = 10;      % 加速度计常值零偏 (ug)
web     = 0.0002;   % 陀螺角度随机游走系数 (deg/sqrt(h))
wdb     = 7;       % 加速度计速度随机游走系数 (ug/sqrt(Hz))

% 一阶马尔可夫相关噪声 (模拟环境变温引起的慢变漂移)
sqrtR0G = 0.01;    % 陀螺相关零偏 (deg/h)
TauG    = 300;     % 陀螺相关时间 (s)
sqrtR0A = 10;      % 加速度计相关零偏 (ug)
TauA    = 300;     % 加速度计相关时间 (s)

% 标度因数与安装误差
dKGii   = 5;       % 陀螺刻度系数误差 (ppm)
dKAii   = 20;      % 加速度计刻度系数误差 (ppm)
dKGij   = 10;      % 陀螺安装角误差 (arcsec)
dKAij   = 10;      % 加速度计安装角误差 (arcsec)

% 高阶非线性与异步项配置 (置零)
KA2     = 0;       rxyz    = 0;       dtGA    = 0;       

% 构造 PSINS 误差结构体并向理想 IMU 数据注入物理误差
imuerr = imuerrset(eb, db, web, wdb, sqrtR0G, TauG, sqrtR0A, TauA, ...
                   dKGii, dKAii, dKGij, dKAij, KA2, rxyz, dtGA);
imu = imuadderr(trj.imu, imuerr);

% 核心物理对齐：将 IMU 数据轴向由前向物理系的 RFU(右前上) 转换为解算器要求的 FRD(前右下)
imuFRD = imuRFU2FRD(imu);

file_imu = [outputfolder, 'imu_data.txt'];
save(file_imu, 'imuFRD', '-ascii', '-double');

%% 执行纯惯导算法机理更新解算 (Pure INS Mechanization)
% cfg = cfginit(pva);
% pure_ins_mechanization(cfg, imuFRD, pva); 
% 
% % 读入纯惯导算法解算出的、包含舒勒振荡和位置发散的名义导航结果
% pva_ins = importdata('NavResult-pureins-height.nav');
% avp_ins = pvaNED2ENU(pva_ins);
% 
% auv_pos_geo_true = trj.avp(:, 7:9);
% auv_pos_geo_ins  = avp_ins(:, 7:9);
% 
% [auv_pos_xyz_true, ~, ~] = pos2dxyz(auv_pos_geo_true, pos0_geo);
% [auv_pos_xyz_ins, ~, ~]  = pos2dxyz(auv_pos_geo_ins,  pos0_geo);

%% ==================== 4. 坐标系逆向映射与水声测距数据流仿真 ====================
fprintf('\n==================== 4. 正在执行多基站水声测距仿真 ====================\n');

% 提取 AUV 真实地理轨迹并映射回本地 ENU 直角坐标系 [X, Y, Z] (米)
auv_pos_geo_true = trj.avp(:, 7:9);
[auv_pos_xyz_true, ~, ~] = pos2dxyz(auv_pos_geo_true, pos0_geo);

% --- 水声互测时序采样仿真配置 ---
t_ping = 20;                             % 设定声学响应打点周期为 20 秒
ts_sim = trj.avp(2,10) - trj.avp(1,10);  % 提取仿真步长 (0.01s)
ping_interval = round(t_ping / ts_sim);   % 计算对应的数据索引步长
ping_indices = 1:ping_interval:length(trj.avp);
N_pings = length(ping_indices);

% 设定水声物理测距的高斯白噪声标准差（严格对齐你设定的 5.0m 大噪声工况）
sigma_r = 5;                           

% 初始化纯水平测距大盘数据集矩阵
range_data = zeros(N_pings, 7);

% --- 【4.5 专属多基站时序打包格式预分配】 ---
% 格式严格满足：[时间, 斜距, 水平距离, 潜标Lat(rad), 潜标Lon(rad), 潜标Hgt(m)]
range1_mat = zeros(N_pings, 6);
range2_mat = zeros(N_pings, 6);
range3_mat = zeros(N_pings, 6);

for k = 1:N_pings
    idx = ping_indices(k);
    t_curr = trj.avp(idx, 10);
    
    % 提取 AUV 当前时刻对应的真实物理三维直角坐标 [X, Y, Z] 与天向深度
    xyz_true   = auv_pos_xyz_true(idx, :); 
    depth_meas = -trj.avp(idx, 9);         % 深度计名义测量正高程
    
    r_h_meas = zeros(3,1);
    r_slant_meas = zeros(3,1);
    
    % 循环处理 3 个潜标
    for i = 1:3
        % 1. 计算当前潜标对应的物理距离真值
        r_h_true     = norm(xyz_true(1:2) - S_true_xyz(i, 1:2)); % 2D平面水平距离真值
        r_slant_true = norm(xyz_true - S_true_xyz(i, :));        % 3D空间斜距真值
        
        % 2. 【物理对齐完善】单次打点使用同一个水声信道突发噪声，避免两次独立随机产生冲突
        v_k = normrnd(0, sigma_r); 
        
        % 3. 混入同源噪声，直接得到水平与斜距观测值
        r_h_meas(i)     = r_h_true + v_k;
        r_slant_meas(i) = r_slant_true + v_k;
    end
    
    % 4. 打包总观测快照大盘 (因目前处于纯理想轨迹阶段，前两维暂用理想 X, Y 占位)
    range_data(k, :) = [t_curr, xyz_true(1), xyz_true(2), depth_meas, r_h_meas(1), r_h_meas(2), r_h_meas(3)];
    
    % 5. 【严格按格式要求】分装 3 个潜标的专属时序数据，4-6列直接灌入绝对青岛基准地理坐标真值
    range1_mat(k, :) = [t_curr, r_slant_meas(1), r_h_meas(1), S_gnss_geo(1, 1), S_gnss_geo(1, 2), S_gnss_geo(1, 3)];
    range2_mat(k, :) = [t_curr, r_slant_meas(2), r_h_meas(2), S_gnss_geo(2, 1), S_gnss_geo(2, 2), S_gnss_geo(2, 3)];
    range3_mat(k, :) = [t_curr, r_slant_meas(3), r_h_meas(3), S_gnss_geo(3, 1), S_gnss_geo(3, 2), S_gnss_geo(3, 3)];
end

% ==================== 4.5 专属多基站时序打包与地理坐标 TXT 导出 ====================
fprintf('==================== 正在生成独立潜标时序文本数据 ====================\n');

file_range1 = [outputfolder, 'range1.txt'];
file_range2 = [outputfolder, 'range2.txt'];
file_range3 = [outputfolder, 'range3.txt'];

% --- 执行强物理文本导出 (-ascii -double 保证高精度大圆弧弧度不丢失数位) ---
save(file_range1, 'range1_mat', '-ascii', '-double');
save(file_range2, 'range2_mat', '-ascii', '-double');
save(file_range3, 'range3_mat', '-ascii', '-double');

fprintf('1号潜标时序文本成功导出至: %s\n', file_range1);
fprintf('2号潜标时序文本成功导出至: %s\n', file_range2);
fprintf('3号潜标时序文本成功导出至: %s\n', file_range3);

true_bea = [outputfolder, 'true_beacon.mat'];
save D:\Github\KF-GINS-Matlab\潜标位置标定\stage_1\data1\beacon_pos.mat S_true_geo S_gnss_geo S_true_xyz  S_gnss_xyz pos0_geo theta_true phi_true



