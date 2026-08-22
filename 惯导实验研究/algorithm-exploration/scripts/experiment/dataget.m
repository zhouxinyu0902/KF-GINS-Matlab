clear; clc;
% 获取高度、距离数据
%% ===================== 路径配置 =====================
cfg = ProcessConfig_exper();
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
