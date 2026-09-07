function [LBL_out, USBL_out] = process_acoustic_data(DistData, tt_lbl, USBL_sync, compass, depther, cfg, isfig)
% PROCESS_ACOUSTIC_DATA 处理声学定位数据 (LBL平滑, USBL多级清洗, 杆臂补偿与UTM投影)
%
% 输入:
%   DistData   : 舱内综合数据矩阵
%   tt_lbl     : LBL 相对时间轴
%   USBL_sync  : USBL 时间同步结构体 (含 PTSAG, PTSAX, PIXOG 等)
%   compass    : 罗盘姿态 (用于构建 LBL AVP 矩阵)
%   depther    : 修正后的深度计数据
%   cfg        : 配置参数结构体 (需包含 tt_usbl, TT_USBL)
%   isfig      : 是否绘图校验 (1/0)
%
% 输出:
%   LBL_out    : 包含信标坐标、平滑测距、AVP定位真值的结构体
%   USBL_out   : 包含清洗后的 USBL 各项数据及换能器(Transducer)坐标的结构体

if nargin < 7, isfig = 0; end
fprintf('开始处理声学数据 (LBL平滑与USBL级联清洗)...\n');

% 提取常用时间轴
tt_usbl = cfg.tt_usbl;
TT_USBL = cfg.TT_USBL;
path = 'D:\WPS云盘\469639050\WPS云盘\成果\1_DR_RANGE\fig\';
%% ================= 1. 长基线 (LBL) 数据处理 =================
% LBL positioning results from DistData
% DistData(19,:) : latitude  [deg]
% DistData(18,:) : longitude [deg]

% Sampling index for LBL positioning results
ID = 1:16:min(16 * length(tt_usbl), length(tt_lbl));

% Raw LBL latitude and longitude
lat_lbl_raw = d2r(DistData(19, ID));
lon_lbl_raw = d2r(DistData(18, ID));
t_lbl_raw   = tt_lbl(ID);

% Remove invalid samples
valid_pos = isfinite(t_lbl_raw) & isfinite(lat_lbl_raw) & isfinite(lon_lbl_raw);
t_lbl_raw   = t_lbl_raw(valid_pos);
lat_lbl_raw = lat_lbl_raw(valid_pos);
lon_lbl_raw = lon_lbl_raw(valid_pos);

% Remove duplicate time stamps
[t_lbl_raw, unique_id] = unique(t_lbl_raw, 'stable');
lat_lbl_raw = lat_lbl_raw(unique_id);
lon_lbl_raw = lon_lbl_raw(unique_id);

% Smooth LBL positioning results
lat_lbl_smooth = smooth(t_lbl_raw, lat_lbl_raw, 0.02, 'rloess');
lon_lbl_smooth = smooth(t_lbl_raw, lon_lbl_raw, 0.035, 'rloess');

% Interpolate smoothed LBL positions to the full LBL time axis
lat_lbl = interp1(t_lbl_raw, lat_lbl_smooth, tt_lbl, 'linear', 'extrap')';
lon_lbl = interp1(t_lbl_raw, lon_lbl_smooth, tt_lbl, 'linear', 'extrap')';

% Keep the final original endpoint if needed
lat_lbl(end) = d2r(DistData(19, end));
lon_lbl(end) = d2r(DistData(18, end));

% Save results
LBL_out.lat_raw = lat_lbl_raw;
LBL_out.lon_raw = lon_lbl_raw;
LBL_out.t_raw   = t_lbl_raw;

LBL_out.lat_smooth_sparse = lat_lbl_smooth;
LBL_out.lon_smooth_sparse = lon_lbl_smooth;

LBL_out.lat = lat_lbl;
LBL_out.lon = lon_lbl;
LBL_out.t   = tt_lbl;
colors=[
    0.051, 0.251, 0.502; 
    0.651, 0.102, 0.153; 
    0.000, 0.600, 0.498;
    1.000, 0.498, 0.055;
    0.337, 0.706, 0.914; 
    0.902, 0.624, 0.000; 
    0.800, 0.475, 0.655; 
    0.000, 0.447, 0.698  
    ];

%%= Plot 1: Latitude and longitude versus time

fig = myfigurestartup(7, 3, 'zxy');
set(fig, 'Name', 'LBL Position Smoothing');

subplot(1,2,1);
plot(t_lbl_raw, r2d(lat_lbl_raw), "Color",colors(1,:),...
    'DisplayName', 'Raw LBL latitude');
hold on;
plot(t_lbl_raw, r2d(lat_lbl_smooth),  "Color",colors(2,:),...
    'DisplayName', 'Smoothed LBL latitude');
plot(tt_lbl, r2d(lat_lbl), '--',  "Color",colors(3,:),...
    'DisplayName', 'Interpolated LBL latitude');
grid on;
xlabel('Time (s)');
ylabel('Latitude (deg)');
legend('Location', 'best');
% exportgraphics(gca, [path,'LBL process lat.png'], 'Resolution', 600);

subplot(1,2,2);
plot(t_lbl_raw, r2d(lon_lbl_raw), "Color",colors(1,:),...
    'DisplayName', 'Raw LBL longitude');
hold on;
plot(t_lbl_raw, r2d(lon_lbl_smooth),  "Color",colors(2,:),...
    'DisplayName', 'Smoothed LBL longitude');
plot(tt_lbl, r2d(lon_lbl), '--',  "Color",colors(3,:),...
    'DisplayName', 'Interpolated LBL longitude');
grid on;
xlabel('Time (s)');
ylabel('Longitude (deg)');
legend('Location', 'best');
% exportgraphics(gca, [path,'LBL process lon.png'], 'Resolution', 600);
exportpngandpdf(fig, [path,'LBL process'])
%%= Plot 2: Local East-North trajectory

% Convert latitude and longitude to a local tangent-plane approximation
lat0 = lat_lbl(1);
lon0 = lon_lbl(1);
R0 = 6378137;   % Earth radius approximation [m]

E_raw = (lon_lbl_raw - lon0) .* R0 .* cos(lat0);
N_raw = (lat_lbl_raw - lat0) .* R0;

E_smooth = (lon_lbl_smooth - lon0) .* R0 .* cos(lat0);
N_smooth = (lat_lbl_smooth - lat0) .* R0;

E_interp = (lon_lbl - lon0) .* R0 .* cos(lat0);
N_interp = (lat_lbl - lat0) .* R0;

fig = myfigurestartup(3, 3, 'zxy');
set(fig, 'Name', 'LBL Local Trajectory');

plot(E_raw, N_raw, '.',"Color",colors(1,:),...
    'DisplayName', 'Raw LBL position');
hold on;

plot(E_smooth, N_smooth, "Color",colors(2,:),...
    'DisplayName', 'Smoothed LBL position');

plot(E_interp, N_interp, '--', "Color",colors(3,:),...
    'DisplayName', 'Interpolated LBL trajectory');

grid on;
axis equal;
xlabel('East (m)');
ylabel('North (m)');

ylim([-1000,400])
legend('Location', 'best');
exportpngandpdf(fig, [path,'LBL trj'])

%%
% 使用四个信标计算
beacon.pos{1}=[deg2rad([17+34/60+53.2675/3600,117+48/60+13.9232/3600]),-3856.6];
beacon.pos{2}=[deg2rad([17+35/60+54.6676/3600,117+47/60+33.9740/3600]),-3848.4];
beacon.pos{3}=[deg2rad([17+35/60+5.121/3600,117+46/60+27.3108/3600]),-3806.06];
beacon.pos{4}=[deg2rad([17+34/60+1.3920/3600,117+47/60+27.03/3600]),-3856.3];
beacon.range_raw=cell(1,4);
for i=1:4
    beacon.range_raw{i} = DistData(10+i,:)';
    [beacon.range{i}, ~, ~] ...
        = clean_outliers_dynamic_knn(beacon.range_raw{i}, tt_lbl, 46, 8, 1);
    if i==4
        exportpngandpdf(gca, [path,'range4'])
    end
end
[hov_llh, ~] = solve_lbl_with_depth(beacon,-DistData(17,:), tt_lbl');
lat_lbl = hov_llh(:,1);
lon_lbl = hov_llh(:,2);

LBL_out.BCN = beacon.pos;
LBL_out.RNG = beacon.range;



% 构建 LBL 参考真值 AVP 矩阵
LBL_out.avp_m = [compass, zeros(length(compass),3), lat_lbl, lon_lbl, -depther, tt_lbl'];
LBL_out.lld_raw = [DistData(19, :)', DistData(18, :)', -depther];

for i=1:4
    LBL_out.RNG1{i} = RCompu(LBL_out.avp_m(:,7:9),beacon.pos{i})';
end


LBL_out.LatLonShipCabin = [d2r(DistData(10,:));d2r(DistData(9,:));tt_lbl];
%% ================= 2. 超短基线 (USBL) 数据处理 =================

% --- 2.1 深度计与 USBL 定位深度清洗 ---
DepthHovPTSAG = USBL_sync.PTSAGHov(6, :);
idx_err = DepthHovPTSAG > 3940 | DepthHovPTSAG < 3860;
DepthHovPTSAG(idx_err) = [];
USBL_out.DepthHov = interp1(USBL_sync.ttPTSAGHov(~idx_err), DepthHovPTSAG, tt_usbl, 'linear');
USBL_out.DepthHovPTSAX = USBL_out.DepthHov; % PTSAX 深度共用

% --- 2.2 PTSAG 经纬度级联清洗 (调用底部子函数) ---
% 纬度: 窗口 [100, 50, 50], 阈值 [0.0025, 0.0016, 0.0012]
[T_lat, Lat_clean] = cascade_clean(USBL_sync.TimePTSAGHov, USBL_sync.PTSAGHov(3,:), ...
                                   [100, 50, 50], [0.0025, 0.0016, 0.0012], 'lat/deg');
LatHov = interp1(T_lat, Lat_clean, TT_USBL, 'linear');
USBL_out.LatHov = smooth(tt_usbl, LatHov, 0.02, 'rloess')';

% % ---------论文绘图--------
usbl_north_raw = USBL_sync.PTSAGHov(3,:)    ;
usbl_east_raw  = USBL_sync.PTSAGHov(4, :);
t_usbl         = USBL_sync.TimePTSAGHov;


% % 使用设计的 KNN 函数进行两轮清洗
[usbl_north, ~, ~] = clean_outliers_dynamic_knn(usbl_north_raw, t_usbl, 0.0008, 20, 1,'USBL Lat/deg');
exportpngandpdf(gca, [path,'USBL Lat'])
[usbl_north, ~, ~] = clean_outliers_dynamic_knn(usbl_north, t_usbl, 0.0003, 5, 1,'USBL Lat/deg');
[usbl_east, ~, ~]  = clean_outliers_dynamic_knn(usbl_east_raw, t_usbl, 0.0008, 20, 1,'USBL Lon/deg');
exportpngandpdf(gca, [path,'USBL Lon'])
[usbl_east, ~, ~]  = clean_outliers_dynamic_knn(usbl_east, t_usbl, 0.0003, 5, 1,'USBL Lon/deg'); % 最后一轮开启绘图检查
% % ---------论文绘图--------

% 经度: 窗口 [100, 50], 阈值 [0.0020, 0.0012]
[T_lon, Lon_clean] = cascade_clean(USBL_sync.TimePTSAGHov, USBL_sync.PTSAGHov(4,:), ...
                                   [100, 50], [0.0020, 0.0012], 'lon/deg');
LonHov = interp1(T_lon, Lon_clean, TT_USBL, 'linear');
USBL_out.LonHov = smooth(tt_usbl, LonHov, 0.02, 'rloess')';

% --- 2.3 PTSAX 前向与右向位移级联清洗 ---
[T_xf, XF_clean] = cascade_clean(USBL_sync.TimePTSAX, USBL_sync.PTSAX(9,:), ...
                                 50, 450, 'XForward/m');
XForward = interp1(T_xf, XF_clean, TT_USBL, 'linear');
USBL_out.XForward = smooth(tt_usbl, XForward, 0.01, 'rloess')';

[T_ys, YS_clean] = cascade_clean(USBL_sync.TimePTSAX, USBL_sync.PTSAX(10,:), ...
                                 [30, 35, 30], [500, 430, 400], 'YStarboard/m');
YStarboard = interp1(T_ys, YS_clean, TT_USBL, 'linear');
USBL_out.YStarboard = smooth(tt_usbl, YStarboard, 0.01, 'rloess')';

% % ---------论文绘图--------
[YStarboard11, ~, ~]  = clean_outliers_dynamic_knn(USBL_sync.PTSAX(10,:), USBL_sync.TimePTSAX,300, 9, 1,'USBL YStarboard/m');
ylim([min(USBL_sync.PTSAX(10,:)) max(YStarboard11)*1.5])
exportpngandpdf(gca, [path,'USBL YStarboard'])
[XForward11, ~, ~]  = clean_outliers_dynamic_knn(USBL_sync.PTSAX(9,:),USBL_sync.TimePTSAX, 250, 10, 1,'USBL XForward/m');
ylim([min(USBL_sync.PTSAX(9,:)) max(XForward11)*1.5])
exportpngandpdf(gca, [path,'USBL XForward'])
% % ---------论文绘图--------

% --- 2.4 PIXOG 传播时间级联清洗 ---
TimeH1_raw = (USBL_sync.PIXOG(17,:) - 20e3) * 1e-6; % 减去 20ms 周转时间
[T_h1, TimeH1_clean] = cascade_clean(USBL_sync.TimePIXOG, TimeH1_raw, ...
                                     20, 0.28, 'PropaTime/(s)');
USBL_out.TimeH1 = interp1(T_h1, TimeH1_clean, TT_USBL, 'spline');

% % ---------论文绘图--------
% % 使用设计的 KNN 函数进行两轮清洗
[TimePIXOG, ~, ~]  = clean_outliers_dynamic_knn(TimeH1_raw,USBL_sync.TimePIXOG, 0.2, 10, 1,'USBL PropaTime/s');
ylim([min(TimeH1_raw) max(TimePIXOG)*1.5])
exportpngandpdf(gca, [path,'USBL TimePIXOG'])
% % ---------论文绘图--------


% --- 2.5 母船状态插值 ---
USBL_out.Ship.Lat   = interp1(USBL_sync.TimePIXOG, r2d(USBL_sync.PIXOG(8,:)), TT_USBL, 'linear');
USBL_out.Ship.Lon   = interp1(USBL_sync.TimePIXOG, r2d(USBL_sync.PIXOG(9,:)), TT_USBL, 'linear');
USBL_out.Ship.Head  = interp1(USBL_sync.TimePIXOG, r2d(USBL_sync.PIXOG(11,:)), TT_USBL, 'linear');
USBL_out.Ship.Depth = interp1(USBL_sync.TimePIXOG, USBL_sync.PIXOG(10,:), TT_USBL, 'linear');

%% ================= 3. 杆臂补偿与 UTM 投影 =================
% 转换为 UTM 坐标 (WGS84)
[XUTMShip, YUTMShip, f_utm] = ll2utm(USBL_out.Ship.Lat, USBL_out.Ship.Lon);

% 向阳红九号杆臂参数
lv1 = 3.1; lv2 = -0.66; % lv3 = 27.3;
H1range = sqrt((-lv1+0.29)^2 + (lv2+0)^2);
H1angle = rad2deg(atan2(-lv1+0.29, lv2+0));

% 计算换能器在 UTM 下的偏移量
xdelta = H1range * cos(deg2rad(H1angle + USBL_out.Ship.Head - 90));
ydelta = H1range * sin(deg2rad(H1angle + USBL_out.Ship.Head - 90));

USBL_out.Transducer.XUTM = XUTMShip + xdelta;
USBL_out.Transducer.YUTM = YUTMShip - ydelta;

% UTM 转回经纬度
[LatTran, LonTran] = utm2ll(USBL_out.Transducer.XUTM, USBL_out.Transducer.YUTM, f_utm);
USBL_out.Transducer.Lat = LatTran';
USBL_out.Transducer.Lon = LonTran';

fprintf('声学数据处理与坐标投影完毕！\n\n');

%% 4. 可视化校验 (如果 isfig == 1)
if isfig
    myfigurestartup(7,4,'paper');
    % LBL 轨迹平滑对比
    subplot(2, 2, 1);
    plot(tt_lbl, d2r(DistData(19, :)), '.', 'MarkerSize', 2); hold on;
    plot(tt_lbl, lat_lbl, 'LineWidth', 1.5);
    xygo('t/s', 'lat/rad'); title('LBL Latitude Smoothing'); legend('Raw', 'Smoothed');

    % USBL 深度补偿对比
    subplot(2, 2, 2);
    plot(USBL_sync.ttPTSAGHov, -USBL_sync.PTSAGHov(6,:), '.', 'MarkerSize', 4); hold on;
    plot(tt_usbl, -USBL_out.DepthHov, '.', 'MarkerSize', 4);
    ylim([-4100, -3500]); xygo('t/s', 'Depth (m)'); title('USBL Depth Cleaning');

    % 杆臂补偿投影验证
    subplot(2, 2, [3, 4]);
    plot(USBL_out.Ship.Lon, USBL_out.Ship.Lat, 'b-', 'DisplayName', 'Mother Ship'); hold on;
    plot(USBL_out.Transducer.Lon, USBL_out.Transducer.Lat, 'r-', 'DisplayName', 'Transducer');
    xygo('Longitude', 'Latitude'); title('Ship vs Transducer Trajectory'); legend;
end
end

%% ================== 局部辅助函数 ==================
function [T_out, D_out] = cascade_clean(T_in, D_in, wins, ths, label)
% CASCADE_CLEAN 级联执行 KNN 异常值剔除 (indentify_error)
% 通过循环应用不同的窗口和阈值，替代冗长的嵌套代码
T_out = T_in;
D_out = D_in;
for i = 1:length(wins)
    % 假设外部作用域已经加载了 indentify_error 函数
    [D_out, T_out] = indentify_error(T_out, D_out, wins(i), ths(i), label);
end
end
function [hov_llh, stats] = solve_lbl_with_depth(beacon, hov_depth_seq, LBL_time)
% SOLVE_LBL_WITH_DEPTH 使用深度计约束解算 LBL 定位轨迹
%
% 输入:
%   beacon.pos      : 1x4 cell, 包含 [lat(rad), lon(rad), depth(m)]
%   beacon.range    : 1x4 cell, 每个是与 LBL_time 等长的测距序列
%   hov_depth_seq   : N*1 向量, HOV 的压力深度序列 (需与 beacon 深度符号一致)
%   LBL_time        : N*1 时间序列

% 1. 数据预处理：中值滤波清洗测距毛刺
fprintf('正在预清洗测距数据...\n');
clean_ranges = zeros(length(LBL_time), 4);
for i = 1:4
    % 使用窗口大小为 11 的中值滤波剔除瞬时野值
    clean_ranges(:, i) = medfilt1(beacon.range{i}, 11);
end

% 2. 坐标系转换：LLH -> 局部 ENU (米)
Re = 6378137.0;
num_beacons = 4;
beacons_llh = zeros(num_beacons, 3);
for i = 1:num_beacons
    beacons_llh(i, :) = beacon.pos{i};
end

% 以 1 号信标为局部坐标系原点
lat0 = beacons_llh(1, 1);
lon0 = beacons_llh(1, 2);

beacons_enu = zeros(num_beacons, 3);
for i = 1:num_beacons
    beacons_enu(i, 2) = (beacons_llh(i, 1) - lat0) * Re; % North
    beacons_enu(i, 1) = (beacons_llh(i, 2) - lon0) * Re * cos(lat0); % East
    beacons_enu(i, 3) = beacons_llh(i, 3); % 深度 (假设为负值)
end

% 3. 核心解算：深度约束下的最小二乘
N = length(LBL_time);
hov_enu = zeros(N, 3);

% 初始猜测点：设在信标阵列中心
x_guess = [mean(beacons_enu(:,1)), mean(beacons_enu(:,2))];

% 优化参数设置
opts = optimoptions('lsqnonlin', 'Display', 'off', 'StepTolerance', 1e-6);

fprintf('开始执行深度约束解算 (总计 %d 帧)...\n', N);
for k = 1:N
    rk = clean_ranges(k, :)';
    zk = hov_depth_seq(k); % 当前时刻深度

    % 筛选有效信标 (剔除 0 或异常跳变点)
    valid = rk > 500 & rk < 8000; % 根据实际海域调整范围

    if sum(valid) >= 2 % 有深度约束，最少只需 2 个信标即可解算
        % 定义 2D 残差函数: p = [x, y]
        % 方程: (x_i - x)^2 + (y_i - y)^2 + (z_i - z_known)^2 = r_i^2
        res_fun = @(p) sqrt( (beacons_enu(valid, 1) - p(1)).^2 + ...
            (beacons_enu(valid, 2) - p(2)).^2 + ...
            (beacons_enu(valid, 3) - zk).^2 ) - rk(valid);

        % 求解水平位置 [E, N]
        [p_sol, ~] = lsqnonlin(res_fun, x_guess, [], [], opts);

        hov_enu(k, :) = [p_sol, zk];
        x_guess = p_sol; % 将当前解作为下一时刻初值，加速收敛
    else
        % 如果信号丢失，保持上一时刻位置或记为 NaN
        if k > 1
            hov_enu(k, :) = hov_enu(k-1, :);
        else
            hov_enu(k, :) = [NaN, NaN, zk];
        end
    end
end

% 4. 逆转换：ENU -> LLH
hov_llh = zeros(N, 3);
hov_llh(:, 1) = lat0 + hov_enu(:, 2) / Re; % Latitude (rad)
hov_llh(:, 2) = lon0 + hov_enu(:, 1) / (Re * cos(lat0)); % Longitude (rad)
hov_llh(:, 3) = hov_enu(:, 3); % Depth

% 拼入时间戳作为最后一列 (符合你之前的要求)
hov_llh = [hov_llh, LBL_time];

% 5. 绘图对比
figure('Name', 'LBL 深度辅助解算结果');
% 绘制轨迹
subplot(2,1,1);
plot(rad2deg(hov_llh(:,2)), rad2deg(hov_llh(:,1)), 'b-', 'LineWidth', 1);
hold on;
% 绘制信标位置
plot(rad2deg(beacons_llh(:,2)), rad2deg(beacons_llh(:,1)), 'rp', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
grid on; axis equal;
title('LBL 水平轨迹 (深度约束解算)');
xlabel('Longitude (deg)'); ylabel('Latitude (deg)');

% 绘制深度对比
subplot(2,1,2);
plot(LBL_time, hov_depth_seq, 'k', 'LineWidth', 1.2);
grid on; set(gca, 'YDir', 'reverse'); % 深度图通常向下展示
title('HOV 深度剖面 (用于约束的已知量)');
ylabel('Depth (m)'); xlabel('Time (s)');

stats = '解算完成';
end