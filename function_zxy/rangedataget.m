function rangedataget(path, align_mode,isfig1,isfig2en,isfiggeo,isfigrange)
% =========================================================================
% 函数名称：rangedataget
% 功能描述：基于GNSS基准轨迹，反向仿真生成3个伪信标的测距解算数据
% 输入参数:
%   path       - 数据及配置文件存放的文件夹路径 (String)
%   align_mode - 阵列基准对齐模式 (String)
%                'right'  : 以等边三角形的【右侧斜边】中点作为车辆轨迹起点
%                'bottom' : 以等边三角形的【平坦底边】中点作为车辆轨迹起点
% =========================================================================

% 缺省参数容错处理
if nargin < 2
    align_mode = 'right';
    isfig1 = 1;
    isfig2en = 0;
    isfiggeo = 0;
    isfigrange = 0;
end

% 1. 安全读取标准参考轨迹 (Truth Nav)
truth_file = fullfile(path, 'truth.nav');
if ~exist(truth_file, 'file')
    error('错误：未在路径 %s 下找到参考轨迹 truth.nav 文件！', path);
end
truth = importdata(truth_file);
GNSS_1s = truth(1:100:end, 2:5); % 降采样至 1Hz 提取 [时间, 纬度, 经度, 高程]

%% 2. 阵列几何拓扑定义与空间变换
glvs;
orgin0 = d2r(GNSS_1s(1, 2:4)); % 提取轨迹首帧作为站心局部导航坐标系(ENU)原点

% --- 【修正定义】标准等边三角形阵列坐标（单位：米，基线长度 20km） ---
% 点1: 上方顶点; 点2: 左下角顶点; 点3: 右下角顶点
% 此时 点2-点3 构成了绝对意义上的水平【底边】，点1-点3 构成了【右侧边】
dxyz_original = [  0,  5*sqrt(3), 0;
    -10, -5*sqrt(3), 0;
    10, -5*sqrt(3), 0] * 1000;

% 配置阵列整体旋转角度 (当前设定 0 deg)
theta_deg = 0;
theta_rad = deg2rad(theta_deg);

% 构造绕 Z 轴（天向轴）自转的旋转矩阵
R = [cos(theta_rad), -sin(theta_rad), 0;
    sin(theta_rad),  cos(theta_rad), 0;
    0,               0, 1];

% 步骤 I：阵列绕自身几何中心进行自转
center_xyz = mean(dxyz_original, 1);
dxyz_centered = dxyz_original - center_xyz;
dxyz_rotated = (R * dxyz_centered')' + center_xyz;

% 步骤 II：根据所选方案，精确定位对准图钉的“边中点”
if strcmpi(align_mode, 'right')
    % 方案 A：右侧斜边中点 (点1与点3的连线中点)
    target_midpoint = (dxyz_rotated(1, :) + dxyz_rotated(3, :)) / 2;
    fprintf('当前配置: 以等边三角形【右侧斜边】中点对齐车辆起点。\n');

elseif strcmpi(align_mode, 'bottom')
    % 方案 B：平坦底边中点 (点2与点3的连线中点)
    target_midpoint = (dxyz_rotated(2, :) + dxyz_rotated(3, :)) / 2;
    fprintf('当前配置: 以等边三角形【水平底边】中点对齐车辆起点。\n');

else
    error('错误：未知的对齐模式！请输入 ''right'' 或 ''bottom''。');
end

% 步骤 III：阵列整体刚性平移，将选定的边中点重合至 ENU 坐标系原点 (0,0,0)
dxyz_final = dxyz_rotated - target_midpoint;

%% 3. 导航系坐标系重构转换
rrm = dxyz2pos(dxyz_final, orgin0'); % 局部局部米级坐标转大椭球大地球经纬高
beaconxyz = dxyz_final(1:3, :);
beaconrrm = rrm(1:3, :);

% 转化动态轨迹至本地 ENU 坐标系
trj = GNSS_1s(:, 2:4);
trj(:, 1:2) = d2r(trj(:, 1:2));
trajectory_xyz = pos2dxyz(trj, orgin0');

% 绘图模块 1：拓扑展板（动态观察轨迹是否完美从设定的边中点出发）
if isfig1
    plot_trajectory_and_beacons_m(trajectory_xyz, beaconxyz);
end
if isfig2en
    plot_beacons_and_trajectory_en(trajectory_xyz, beaconxyz, path, align_mode);
end
% ================= 新增：在线地图绘图模块 =================
% 1. 提取轨迹的经纬度 (GNSS_1s 的 2,3 列本来就是角度制)
traj_latlon_deg = GNSS_1s(:, 2:3);

% 2. 提取信标的经纬度 (beaconrrm 是弧度制，需要转换为角度制)
% PSINS 中的 r2d 函数或者直接乘以 180/pi
beacon_latlon_deg = beaconrrm(1:3, :)' * (180 / pi);
if isfiggeo
    % 3. 调用在线地图绘图函数
    plot_beacons_and_trajectory_geomap(traj_latlon_deg, beacon_latlon_deg,path);
end
%% 4. 高精度测距仿真解算
trajectory_x = trajectory_xyz(:, 1);
trajectory_y = trajectory_xyz(:, 2);
time_s = GNSS_1s(:, 1);
num_points = length(time_s);

distances = zeros(num_points, 3);

% 绘图模块 2：动态测距收敛观测曲线
if isfigrange
    figure('Name', sprintf('信标解算距离趋势 (%s)', align_mode), 'Color', 'w');
    hold on; grid on;
    plot_colors = {'#D95319', '#77AC30', '#0072BD'}; % 规范工程配色：橙、绿、蓝
end
for i = 1:3
    % 计算运载体到各个地面静态信标的空间 2D 基线平距
    distances(:, i) = sqrt((trajectory_x - beaconxyz(i, 1)).^2 + ...
        (trajectory_y - beaconxyz(i, 2)).^2);
    if isfigrange
        plot(time_s, distances(:, i)/1000, 'LineWidth', 1.5, 'Color', plot_colors{i}, ...
            'DisplayName', sprintf('至地面信标 Beacon %d', i));
    end
end
if isfigrange
    xlabel('GPS时间 (s)', 'FontSize', 11);
    ylabel('空间几何距离 (km)', 'FontSize', 11);
    title(sprintf('运载体至静态信标 2D 测距时历曲线 (%s 模式)', align_mode), 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'best');
end
% 仿真传感器高斯白噪声（如需更贴近半实物仿真环境，可取消下行注释）
% distances = distances + normrnd(0, 15, size(distances));

%% 5. 格式化数据矩阵批量写盘
for i = 1:3
    % 平铺信标的大地坐标 [Lat, Lon, Alt] 为 N 行 3 列矩阵
    beacon_ext = repmat(beaconrrm(i, :), num_points, 1);

    % 按照紧凑型量测更新矩阵组装：[时间, 距离, 距离(双向测距占位), 信标Lat, 信标Lon, 信标Alt]
    range_matrix = [time_s, distances(:, i), distances(:, i), beacon_ext];

    output_file = fullfile(path, sprintf('range%d.txt', i));

    try
        writematrix(range_matrix, output_file, 'Delimiter', ' ');
        fprintf('  [导出成功] 成功生成测距量测文件: range%d.txt\n', i);
    catch ME
        warning('  [导出失败] 无法写入文件 range%d.txt. 错误描述: %s', i, ME.message);
    end
end
end
%%
function [figPath, pngPath, figHandle] = plot_beacons_and_trajectory_en(traj_xyz, beaconxyz, outdir, align_mode, fnameBase)
% plot_beacons_and_trajectory_en 绘制 Beacon 与 Vehicle trajectory（英文）
%
% Usage:
% [figPath, pngPath, figHandle] = plot_beacons_and_trajectory_en(traj_xyz, beaconxyz)
% [figPath, pngPath, figHandle] = plot_beacons_and_trajectory_en(traj_xyz, beaconxyz, outdir, align_mode, fnameBase)
%
% Inputs:
% traj_xyz - N x 3 (or N x 2) trajectory coordinates in local ENU (meters)
% beaconxyz - M x 3 (M typically 3) beacon coordinates in same ENU (meters)
% outdir - output folder (optional, default: pwd)
% align_mode - string for title/filename (optional)
% fnameBase - base name for saved files (optional, default: 'topology_<align_mode>_EN')
%
% Outputs:
% figPath - full path to saved .fig
% pngPath - full path to saved .png
% figHandle - figure handle

% ----------------- 参数与输入校验 -----------------
if nargin < 3 || isempty(outdir)
    outdir = pwd;
end
if nargin < 4
    align_mode = 'mode';
end
if nargin < 5 || isempty(fnameBase)
    fnameBase = sprintf('topology_%s_EN', align_mode);
end
if ~exist(outdir, 'dir')
    mkdir(outdir);
end

% 确保 traj_xyz 是 N x 3 或 N x 2
if size(traj_xyz,2) == 2
    traj_xyz = [traj_xyz, zeros(size(traj_xyz,1),1)];
end
if size(beaconxyz,2) == 2
    beaconxyz = [beaconxyz, zeros(size(beaconxyz,1),1)];
end

x_tr = traj_xyz(:,1);
y_tr = traj_xyz(:,2);
bx = beaconxyz(:,1);
by = beaconxyz(:,2);

% 颜色与绘图样式
colors = lines(max(3,size(beaconxyz,1)));

% ----------------- 创建图形 -----------------
figHandle = myfigurestartup(4,4,'zxy');
hold on; grid on; axis equal;

% trajectory
plot(x_tr, y_tr, '-', 'LineWidth', 1.4, 'Color', [0 0.4470 0.7410], 'DisplayName', 'Vehicle trajectory');

% start point
scatter(x_tr(1), y_tr(1), 70, 'k', 'filled', 'DisplayName', 'Start point');
% 生成 legend（自动包含所有带 DisplayName 的对象）
lg = legend;
% 若后续还会添加对象并不想让 legend 自动更新，可关闭自动更新：
lg.AutoUpdate = 'off';
% beacons
hBeacons = gobjects(size(beaconxyz,1),1);
for i = 1:size(beaconxyz,1)
    hBeacons(i) = scatter(bx(i), by(i), 100, ...
        'MarkerEdgeColor', 'k', ...
        'MarkerFaceColor', colors(i,:), ...
        'LineWidth', 1.0);   % <-- 添加 DisplayName
    ht = text(bx(i)+200, by(i)+200, sprintf('Beacon %d', i), 'FontSize', 10, 'FontWeight', 'bold');
    % 不把 text 放入 legend 中（可选）
    if isprop(ht, 'IconDisplayStyle')
        ht.IconDisplayStyle = 'off';
    end
end

xlabel('East (m)');
ylabel('North (m)');
if align_mode == "right"
    axis([-20000 10000 -10000 15000])
else
    axis([-15000 15000 -5000 25000])
end
drawnow;
ax = gca;
if isprop(ax, 'YAxis') && isprop(ax.YAxis, 'Exponent')
    ax.YAxis.Exponent = 0;   % 禁用科学计数法缩放
    ax.XAxis.Exponent = 0;   % 禁用科学计数法缩放
end

% ----------------- 保存文件（.fig 和 .png） -----------------
figPath = fullfile(outdir, [fnameBase, '.fig']);
pngPath = fullfile(outdir, [fnameBase, '.png']);

savefig(figHandle, figPath);
exportgraphics(figHandle, pngPath, 'Resolution', 600);
fprintf('Saved EN figure: %s\nSaved EN png: %s\n', figPath, pngPath);
end
%%
function [figHandle] = plot_beacons_and_trajectory_geomap(traj_lla, beacon_lla, outdir)
% =========================================================================
% 函数名称：plot_beacons_and_trajectory_geomap
% 功能描述：在在线卫星/街道地图上投影轨迹与信标
% 输入参数:
%   traj_lla   - N x 2 或 N x 3 轨迹地理坐标 [Lat(deg), Lon(deg), (Alt)]
%   beacon_lla - M x 2 或 M x 3 信标地理坐标 [Lat(deg), Lon(deg), (Alt)]
% =========================================================================
figHandle = figure('Name', '在线卫星地图拓扑投影', 'Color', 'w');
% 提取经度和纬度坐标 (geoplot的输入顺序为 纬度, 经度)
traj_lat = traj_lla(:, 1);
traj_lon = traj_lla(:, 2);

beacon_lat = beacon_lla(1, :);
beacon_lon = beacon_lla(2, :);

% 绘制动态轨迹
geoplot(traj_lat, traj_lon, '-', 'LineWidth', 1.5, 'Color', [0 0.4470 0.7410], 'DisplayName', 'Vehicle trajectory');
hold on;

% 绘制轨迹起点
geoscatter(traj_lat(1), traj_lon(1), 80, 'k', 'filled', 'DisplayName', 'Start point');
lg = legend('Location', 'best');
% 若后续还会添加对象并不想让 legend 自动更新，可关闭自动更新：
lg.AutoUpdate = 'off';
% 绘制地面信标
colors = lines(size(beacon_lla, 1));
for i = 1:size(beacon_lla, 1)
    geoscatter(beacon_lat(i), beacon_lon(i), 120, colors(i,:), 'filled', ...
        'MarkerEdgeColor', 'k');

    % 添加信标文本标签（使用 text 功能，需配合地理坐标区）
    text(beacon_lat(i), beacon_lon(i), sprintf('  Beacon %d', i), ...
        'FontSize', 10, 'FontWeight', 'bold'); % 白色字体在卫星图上更清晰
end

% 设置在线底图样式
% 可选底图包括: 'satellite'(卫星图), 'streets'(街道图), 'topographic'(地形图)
% geobasemap('satellite');
geobasemap('streets');
% geobasemap('topographic');
% title('基准轨迹与信标阵列空间拓扑 (真实地理底图)', 'FontSize', 12, 'FontWeight', 'bold');

% 获取当前系统自动适配的经纬度范围
[latlim, lonlim] = geolimits;

% 计算当前画面的纬度和经度跨度
lat_span = latlim(2) - latlim(1);
lon_span = lonlim(2) - lonlim(1);

% 设置向外扩充的留白比例（0.2 表示上下左右各扩大 20% 的视野）
margin = 0.2;

% 计算并应用新的、更宽裕的坐标系上下限
new_latlim = [latlim(1) - lat_span * margin, latlim(2) + lat_span * margin];
new_lonlim = [lonlim(1) - lon_span * margin, lonlim(2) + lon_span * margin];
geolimits(new_latlim, new_lonlim);
% 获取当前的地理坐标区对象
gx = gca;

% 清空默认的“经纬度”汉字标签
gx.LatitudeLabel.String = '';
gx.LongitudeLabel.String = '';
% 可选：开启经纬度网格
% geogrid on;

% ----------------- 保存文件（.fig 和 .png） -----------------

pngPath = fullfile(outdir, 'GeoTopology_EN.png');
exportgraphics(figHandle, pngPath, 'Resolution', 600);
fprintf('Saved EN png: %s\n', pngPath);
end