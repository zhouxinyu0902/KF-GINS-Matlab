%% KF RANGE/INS 组合导航
% 高度量测：100 Hz，每个 IMU 历元更新一次垂向。
% 距离量测：约 360 s 一次，只更新水平距离。

clear; clc; close all;

%% 1. 路径与参数
this_dir = fileparts(mfilename('fullpath'));
project_root = fullfile(this_dir, '..', '..');
input_dir = fullfile(this_dir, 'input');
output_dir = fullfile(this_dir, 'output');

addpath(genpath(project_root));
addpath(input_dir);

param = Param();
cfg = config_simu(input_dir);
cfg.outputfolder = output_dir;
if ~exist(cfg.outputfolder, 'dir')
    mkdir(cfg.outputfolder);
end

rngstd = 6;        % 水平距离噪声，m
heightstd = 0.4;   % 高度噪声，m
rng(1);
tic;

%% 2. 导入 IMU、高度和 360 s 距离量测
imudata = importdata(cfg.imufilepath);
truth = importdata(cfg.truthpath);
heightdata = truth(:, [2, 5]);

range_streams = { ...
    importdata(cfg.rangefile1path), ...
    importdata(cfg.rangefile2path), ...
    importdata(cfg.rangefile3path)};
rangedata = buildRange360(range_streams);

% 不要把起始时刻推到第一条距离量测。距离是稀疏量测，如果从 359 s
% 开始但仍使用 config_simu 中 0.01 s 的初始 PVA，会造成明显初值错位。
cfg.starttime = max([cfg.starttime, imudata(1, 1), heightdata(1, 1)]);
cfg.endtime = min([cfg.endtime, imudata(end, 1), heightdata(end, 1), rangedata(end, 1)]);

imudata = imudata(imudata(:, 1) >= cfg.starttime & imudata(:, 1) <= cfg.endtime, :);
heightdata = heightdata(heightdata(:, 1) >= cfg.starttime & heightdata(:, 1) <= cfg.endtime, :);
rangedata = rangedata(rangedata(:, 1) >= cfg.starttime & rangedata(:, 1) <= cfg.endtime, :);

heightdata(:, 2) = heightdata(:, 2) + heightstd * randn(size(heightdata, 1), 1);
rangedata(:, 3) = rangedata(:, 3) + rngstd * randn(size(rangedata, 1), 1);

fprintf('\n========== KF RANGE/INS Processing ==========\n');
fprintf('处理时间: %.2f -> %.2f s\n', cfg.starttime, cfg.endtime);
fprintf('IMU/高度点数: %d, 距离点数: %d\n', size(imudata, 1), size(rangedata, 1));

%% 3. 初始化和输出
[kf, navstate] = myInitialize_15state(cfg);
kf.rangstd = rngstd;
kf.depthstd = heightstd;

navpath = fullfile(cfg.outputfolder, 'KF-RANGE-INS.nav');
navfp = fopen(navpath, 'wt');
if navfp < 0
    error('无法打开输出文件: %s', navpath);
end
cleanup = onCleanup(@() fclose(navfp));

lastimu = imudata(1, :)';
thisimu = imudata(1, :)';
rangeindex = 1;
range_update_count = 0;
lastpercent = 0;

%% 4. 主循环：100 Hz 高度更新 + 360 s 距离更新
for imuindex = 2:size(imudata, 1)
    lastimu = thisimu;
    thisimu = imudata(imuindex, :)';
    imudt = thisimu(1) - lastimu(1);
    if imudt <= 0
        error('IMU 时间戳必须严格递增。');
    end

    navstate = InsMech(navstate, lastimu, thisimu);

    % 高度是 100 Hz：每个 IMU 历元先做垂向量测更新，并立即反馈垂向。
    kf = heightOnlyUpdate(navstate, heightdata(imuindex, :), kf);
    navstate.pos(3) = navstate.pos(3) - kf.x(3);
    navstate.vel(3) = navstate.vel(3) - kf.x(6);
    kf.x(3) = 0;
    kf.x(6) = 0;

    % 距离是 360 s 周期：到达距离时刻时只做水平距离更新。
    while rangeindex <= size(rangedata, 1) && rangedata(rangeindex, 1) <= thisimu(1) + 5e-4
        kf = rangeOnlyUpdate(navstate, rangedata(rangeindex, :), kf);
        [kf, navstate] = myErrorFeedback_range(kf, navstate);
        rangeindex = rangeindex + 1;
        range_update_count = range_update_count + 1;
    end

    kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);

    nav = [imuindex - 1; navstate.time; ...
        navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3); ...
        navstate.vel; navstate.att * param.R2D];
    fprintf(navfp, ['%6d %12.6f %14.9f %14.9f %10.4f ', ...
        '%10.5f %10.5f %10.5f %10.6f %10.6f %10.6f\n'], nav);

    if imuindex / size(imudata, 1) - lastpercent > 0.20
        fprintf('processing %d %%\n', floor(imuindex * 100 / size(imudata, 1)));
        lastpercent = imuindex / size(imudata, 1);
    end
end

fprintf('距离更新次数: %d\n', range_update_count);
fprintf('结果文件: %s\n', navpath);
toc;

%% 局部函数
function rangedata = buildRange360(range_streams)
    sample_step = 360;
    for i = 1:numel(range_streams)
        range_streams{i} = range_streams{i}(sample_step:sample_step:end, :);
    end
    n = min(cellfun(@(x) size(x, 1), range_streams));
    rangedata = zeros(n, size(range_streams{1}, 2));
    seq = [1, 2, 3];
    for i = 1:3
        rows = i:3:n;
        rangedata(rows, :) = range_streams{seq(i)}(rows, :);
    end
    rangedata = rangedata(any(rangedata, 2), :);
end

function kf = heightOnlyUpdate(navstate, heightrow, kf)
    z = navstate.pos(3) - heightrow(2);
    H = zeros(1, kf.RANK);
    H(3) = 1;
    R = kf.depthstd ^ 2;
    K = kf.P * H' / (H * kf.P * H' + R);

    % 高度量测只反馈到天向位置和天向速度，避免 100 Hz 高度牵动水平状态。
    mask = zeros(kf.RANK, 1);
    mask(3) = 1;
    mask(6) = 1;
    K = K .* mask;

    kf.x = kf.x + K * (z - H * kf.x);
    I = eye(kf.RANK);
    kf.P = (I - K * H) * kf.P * (I - K * H)' + K * R * K';
end

function kf = rangeOnlyUpdate(navstate, rangerow, kf)
    param = Param();
    beacon = rangerow(4:6)';
    [rm, rn] = getRmRn(beacon(1), param);
    DR = diag([rm + beacon(3), (rn + beacon(3)) * cos(beacon(1)), -1]);
    delta = DR * (navstate.pos - beacon);
    predicted = max(norm(delta(1:2)), 1e-6);

    z = predicted - rangerow(3);
    H = zeros(1, kf.RANK);
    derivative = (navstate.pos' - beacon') * (DR ^ 2) / predicted;
    H(1, 1:2) = derivative(1:2);
    R = kf.rangstd ^ 2;

    K = kf.P * H' / (H * kf.P * H' + R);
    kf.x = kf.x + K * (z - H * kf.x);
    I = eye(kf.RANK);
    kf.P = (I - K * H) * kf.P * (I - K * H)' + K * R * K';
end
