%% GNSS/INS 组合导航：1 Hz GNSS + 100 Hz INS
% 本脚本保留原 GNSS_INS.m 的顺序：配置 -> 导入数据 -> 初始化 -> 主循环 -> 保存。
% 原来的距离/高度量测更新已改为 simulated_gnss.txt 的 GNSS 位置更新。

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

gnss_path = fullfile(cfg.outputfolder, 'simulated_gnss.txt');
if ~exist(gnss_path, 'file')
    gnss_path = fullfile(this_dir, 'simulated_gnss.txt');
end
if ~exist(gnss_path, 'file')
    error('找不到 simulated_gnss.txt，请先生成或放到 output/ 目录下。');
end

feedback = 1;                         % GNSS/INS 组合导航需要反馈修正名义状态
time_tolerance = 5e-4;                % simulated_gnss 与 IMU 时间应严格对齐，这里留一点容差
lastpercent = 0;
tic;

%% 2. 导入 IMU 与 GNSS 数据
imudata = importdata(cfg.imufilepath);
gnssdata = importdata(gnss_path);
if size(gnssdata, 2) < 4
    error('simulated_gnss.txt 至少需要 4 列：[time, lat_deg, lon_deg, height_m]。');
end

% simulated_gnss.txt 保存的是角度制，经纬度先转为弧度，供 myGNSSUpdate_15state 使用。
gnssdata = gnssdata(:, 1:4);
gnssdata(:, 2:3) = gnssdata(:, 2:3) * param.D2R;

imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);
cfg.starttime = max([cfg.starttime, imustarttime, gnssdata(1, 1)]);
cfg.endtime = min([cfg.endtime, imuendtime, gnssdata(end, 1)]);

imudata = imudata(imudata(:, 1) >= cfg.starttime & imudata(:, 1) <= cfg.endtime, :);
gnssdata = gnssdata(gnssdata(:, 1) >= cfg.starttime & gnssdata(:, 1) <= cfg.endtime, :);
if isempty(imudata) || isempty(gnssdata)
    error('IMU 与 GNSS 数据在处理时间段内没有重叠。');
end

fprintf('\n========== KF GNSS/INS Processing ==========\n');
fprintf('GNSS 文件: %s\n', gnss_path);
fprintf('处理时间: %.2f -> %.2f s\n', cfg.starttime, cfg.endtime);
fprintf('IMU 点数: %d, GNSS 点数: %d\n', size(imudata, 1), size(gnssdata, 1));

%% 3. 输出文件
navpath = fullfile(cfg.outputfolder, 'KF-GNSS-INS.nav');
navfp = fopen(navpath, 'wt');
if navfp < 0
    error('无法打开输出文件: %s', navpath);
end
cleanup = onCleanup(@() fclose(navfp));

%% 4. 初始化
[kf, navstate] = myInitialize_15state(cfg);
kf.gnssstd = [2; 2; 3];               % 记录使用的 GNSS 位置噪声，单位 m

lastimu = imudata(1, :)';
thisimu = imudata(1, :)';
gnssindex = 1;
gnss_update_count = 0;

while gnssindex <= size(gnssdata, 1) && gnssdata(gnssindex, 1) < thisimu(1) - time_tolerance
    gnssindex = gnssindex + 1;
end

disp('Start KF GNSS/INS Processing!');

%% 5. 主循环：INS 连续递推，GNSS 1 s 一次更新
for imuindex = 2:size(imudata, 1)
    lastimu = thisimu;
    thisimu = imudata(imuindex, :)';
    imudt = thisimu(1) - lastimu(1);
    if imudt <= 0
        error('IMU 时间戳必须严格递增。');
    end

    % INS 机械编排和误差状态一步预测，对应 myInsPropagate_15state。
    navstate = InsMech(navstate, lastimu, thisimu);
    kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);

    % GNSS 量测时间为 1 Hz。若当前 IMU 已经到达 GNSS 历元，则执行位置更新。
    while gnssindex <= size(gnssdata, 1) && gnssdata(gnssindex, 1) <= thisimu(1) + time_tolerance
        if abs(gnssdata(gnssindex, 1) - thisimu(1)) <= 0.5 * max(imudt, time_tolerance)
            kf = myGNSSUpdate_15state(navstate, gnssdata(gnssindex, :)', kf);
            gnss_update_count = gnss_update_count + 1;

            if feedback == 1
                [kf, navstate] = myErrorFeedback_15state(kf, navstate);
            end
        end
        gnssindex = gnssindex + 1;
    end

    %% 6. 保存导航结果
    nav = zeros(11, 1);
    nav(1, 1) = imuindex - 1;
    nav(2, 1) = navstate.time;
    nav(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
    nav(6:8, 1) = navstate.vel;
    nav(9:11, 1) = navstate.att * param.R2D;
    fprintf(navfp, '%6d %12.6f %14.9f %14.9f %10.4f %10.5f %10.5f %10.5f %10.6f %10.6f %10.6f\n', nav);

    if (imuindex / size(imudata, 1) - lastpercent > 0.20)
        fprintf('processing %d %%\n', floor(imuindex * 100 / size(imudata, 1)));
        lastpercent = imuindex / size(imudata, 1);
    end
end

fprintf('GNSS 更新次数: %d\n', gnss_update_count);
fprintf('结果文件: %s\n', navpath);
toc;
