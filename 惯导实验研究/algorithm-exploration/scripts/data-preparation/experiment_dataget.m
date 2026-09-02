clear; clc;
% 获取高度、距离数据
%% ===================== 路径配置 =====================
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
setup_inertial_experiment();
cfg = load_algorithm_exploration_config("experiment", "rad", []);
inputDir = cfg.inputfolder;
outputDir = cfg.referencefolder;

%% ===================== 数据读取 =====================
std_430 = importdata(fullfile(inputDir, 'std_430.txt'));
std_830 = importdata(fullfile(inputDir, 'std_830.txt'));
pva_truth = importdata(fullfile(outputDir, 'truth.nav'));

%% ===================== 误差统计分析 =====================
data_830 = std_830(:, 2:4);
data_430 = std_430(:, 2:4);

rmse_830 = sqrt(mean(data_830.^2, 1));
rmse_430 = sqrt(mean(data_430.^2, 1));

std_830_val = std(data_830, 0, 1);
std_430_val = std(data_430, 0, 1);

max_err_830 = max(abs(data_830), [], 1);
max_err_430 = max(abs(data_430), [], 1);

printErrorStatistics(rmse_830, rmse_430, std_830_val, std_430_val, max_err_830, max_err_430);

%% ===================== 生成高度数据 =====================
height = pva_truth(:, [2, 5]);
writeMatrixSafe(height, fullfile(inputDir, 'height.txt'), 'height');

%% ===================== 构造静止信标数据 =====================
GNSS_1s = pva_truth(1:100:end, 2:5);

glvs;
pos0 = d2r([36.40042003, 120.68981831, 0]);

% 原始等边三角形信标坐标，单位：m
dxyz_original = [
     0,  -5 * sqrt(3), 0;
   -10,   5 * sqrt(3), 0;
   -20,  -5 * sqrt(3), 0
] * 1000;

% 绕 Z 轴旋转角度
theta_deg = 15;
theta_rad = deg2rad(theta_deg);

Rz = [
    cos(theta_rad), -sin(theta_rad), 0;
    sin(theta_rad),  cos(theta_rad), 0;
    0,               0,              1
];

beacon_xyz = (Rz * dxyz_original')';

% 信标坐标转换
beacon_rrm = dxyz2pos(beacon_xyz, pos0');

% 轨迹坐标转换
trj = GNSS_1s(:, 2:4);
trj(:, 1:2) = d2r(trj(:, 1:2));

trajectory_xyz = pos2dxyz(trj, pos0');

%% ===================== 计算轨迹到各信标距离 =====================
trajectory_x = trajectory_xyz(:, 1);
trajectory_y = trajectory_xyz(:, 2);

numEpochs = size(trajectory_xyz, 1);
numBeacons = size(beacon_xyz, 1);

distances = zeros(numEpochs, numBeacons);

for i = 1:numBeacons
    dx = trajectory_x - beacon_xyz(i, 1);
    dy = trajectory_y - beacon_xyz(i, 2);
    
    distances(:, i) = sqrt(dx.^2 + dy.^2);
    
end
%% ===================== 在线地图显示轨迹与信标 =====================

% 统一转换为 [纬度(deg), 经度(deg), 高度(m)]
trajectory_pos_deg = GNSS_1s(:, 2:4);

beacon_pos_deg = beacon_rrm;
beacon_pos_deg(:, 1:2) = r2d(beacon_pos_deg(:, 1:2));

plotTrajectoryBeaconMap(trajectory_pos_deg, beacon_pos_deg, 'satellite');
%% ===================== 生成 range 数据 =====================
rangeData = cell(numBeacons, 1);

for i = 1:numBeacons
    beaconPos = repmat(beacon_rrm(i, :), numEpochs, 1);
    
    rangeData{i} = [
        GNSS_1s(:, 1), ...
        distances(:, i), ...
        distances(:, i), ...
        beaconPos
    ];
    
    outputFile = fullfile(inputDir, sprintf('range%d.txt', i));
    writeMatrixSafe(rangeData{i}, outputFile, sprintf('range%d', i));
end

%% ===================== 局部函数 =====================

function writeMatrixSafe(data, outputFile, dataName)
    try
        writematrix(data, outputFile, 'Delimiter', ' ');
        fprintf('%s 信息已成功写入到：%s\n', dataName, outputFile);
    catch ME
        error('错误：%s 写入失败。错误信息：%s', dataName, ME.message);
    end
end

function printErrorStatistics(rmse_830, rmse_430, std_830, std_430, max_830, max_430)
    fprintf('\n================ 导航误差量化对比，单位：m ================\n');
    fprintf('%-12s | %-15s | %-15s | %-15s\n', '指标', '纬度 Lat', '经度 Lon', '高度 Alt');
    fprintf('------------------------------------------------------------\n');
    fprintf('830 RMSE    | %-15.4f | %-15.4f | %-15.4f\n', rmse_830);
    fprintf('430 RMSE    | %-15.4f | %-15.4f | %-15.4f\n', rmse_430);
    fprintf('------------------------------------------------------------\n');
    fprintf('830 STD     | %-15.4f | %-15.4f | %-15.4f\n', std_830);
    fprintf('430 STD     | %-15.4f | %-15.4f | %-15.4f\n', std_430);
    fprintf('------------------------------------------------------------\n');
    fprintf('830 MAX     | %-15.4f | %-15.4f | %-15.4f\n', max_830);
    fprintf('430 MAX     | %-15.4f | %-15.4f | %-15.4f\n', max_430);
    fprintf('============================================================\n');
end
function fig = plotTrajectoryBeaconMap(trajectoryPos, beaconPos, baseMap)
%PLOTTRAJECTORYBEACONMAP 在线地图显示轨迹和信标位置
%
% 输入：
%   trajectoryPos : N×3 或 N×2
%                   [纬度(deg), 经度(deg), 高度(m)]
%
%   beaconPos     : M×3 或 M×2
%                   [纬度(deg), 经度(deg), 高度(m)]
%
%   baseMap       : 在线地图类型，可选：
%                   'satellite'  卫星地图
%                   'streets'    街道地图
%                   'topographic' 地形地图
%                   'grayterrain'
%                   'darkwater'
%
% 输出：
%   fig           : Figure 句柄
%
% 示例：
%   plotTrajectoryBeaconMap(trj, beacon, 'satellite');

    %% -------------------- 参数检查 --------------------
    if nargin < 3 || isempty(baseMap)
        baseMap = 'satellite';
    end

    if size(trajectoryPos, 2) < 2
        error('trajectoryPos 至少需要两列：[纬度, 经度]');
    end

    if size(beaconPos, 2) < 2
        error('beaconPos 至少需要两列：[纬度, 经度]');
    end

    trajectoryLat = trajectoryPos(:, 1);
    trajectoryLon = trajectoryPos(:, 2);

    beaconLat = beaconPos(:, 1);
    beaconLon = beaconPos(:, 2);

    %% -------------------- 去除无效数据 --------------------
    validTrajectory = ...
        isfinite(trajectoryLat) & ...
        isfinite(trajectoryLon);

    trajectoryLat = trajectoryLat(validTrajectory);
    trajectoryLon = trajectoryLon(validTrajectory);

    validBeacon = ...
        isfinite(beaconLat) & ...
        isfinite(beaconLon);

    beaconLat = beaconLat(validBeacon);
    beaconLon = beaconLon(validBeacon);

    if isempty(trajectoryLat)
        error('轨迹数据中不存在有效经纬度。');
    end

    if isempty(beaconLat)
        error('信标数据中不存在有效经纬度。');
    end

    %% -------------------- 创建地图 --------------------
    fig = figure( ...
        'Name', 'Trajectory and Beacon Map', ...
        'Color', 'w');

    gx = geoaxes(fig);
    hold(gx, 'on');

    %% -------------------- 绘制轨迹 --------------------
    geoplot( ...
        gx, ...
        trajectoryLat, ...
        trajectoryLon, ...
        '-', ...
        'LineWidth', 2.0, ...
        'DisplayName', 'Trajectory');

    %% -------------------- 起点 --------------------
    geoscatter( ...
        gx, ...
        trajectoryLat(1), ...
        trajectoryLon(1), ...
        80, ...
        '^', ...
        'filled', ...
        'DisplayName', 'Start');

    %% -------------------- 终点 --------------------
    geoscatter( ...
        gx, ...
        trajectoryLat(end), ...
        trajectoryLon(end), ...
        80, ...
        'v', ...
        'filled', ...
        'DisplayName', 'End');

    %% -------------------- 绘制信标 --------------------
    geoscatter( ...
        gx, ...
        beaconLat, ...
        beaconLon, ...
        120, ...
        'p', ...
        'filled', ...
        'MarkerFaceColor', 'w', ...
        'MarkerEdgeColor', 'w', ...
        'DisplayName', 'Beacon');

    %% -------------------- 标注信标编号 --------------------
    for i = 1:length(beaconLat)

        text( ...
            gx, ...
            beaconLat(i), ...
            beaconLon(i), ...
            sprintf('  B%d', i), ...
            'Color', 'w', ...
            'FontSize', 11, ...
            'FontWeight', 'bold', ...
            'VerticalAlignment', 'bottom' ...
            );

    end

    %% -------------------- 在线底图 --------------------
    try
        geobasemap(gx, baseMap);
    catch
        warning('无法加载底图 "%s"，改用 streets。', baseMap);
        geobasemap(gx, 'streets');
    end

    %% -------------------- 显示范围 --------------------
    allLat = [trajectoryLat; beaconLat];
    allLon = [trajectoryLon; beaconLon];

    latRange = max(allLat) - min(allLat);
    lonRange = max(allLon) - min(allLon);

    % 防止范围太小时 margin 为 0
    latMargin = max(0.10 * latRange, 0.002);
    lonMargin = max(0.10 * lonRange, 0.002);

    geolimits( ...
        gx, ...
        [min(allLat) - latMargin, max(allLat) + latMargin], ...
        [min(allLon) - lonMargin, max(allLon) + lonMargin]);

    %% -------------------- 图形属性 --------------------
    title(gx, 'Trajectory and Beacon Distribution');

    legend( ...
        gx, ...
        'Location', 'best');

    gx.FontSize = 11;

    hold(gx, 'off');

end
