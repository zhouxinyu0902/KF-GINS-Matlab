%% PSINS轨迹与IMU数据仿真
clear;
clc;
glvs;
%% 1. 仿真参数
ts = 0.01;          % IMU采样周期，单位：s
gpsWeek = 2300;     % GPS周，请根据实验设置修改
startSow = 0;       % 起始周内秒，单位：s

% outputPath = "D:\Github\KF-GINS-Matlab\new_惯导试验\simu\input";
outputPath = "D:\Github\KF-GINS-Matlab\new_惯导试验\simu\input1";
if ~isfolder(outputPath)
    mkdir(outputPath);
end

%% 2. 初始状态
% PSINS AVP排列：[pitch; roll; yaw; vE; vN; vU; lat; lon; height]
avp0 = [0; 0; d2r(135); ...
        0; 0; 0; ...
        glv.pos0];

%% 3. 生成运动轨迹
seg = trjsegment([], 'init', 0);
seg = trjsegment(seg, 'accelerate', 10, [], 0.20577);
seg = trjsegment(seg, 'uniform', 5000);

trj = trjsimu(avp0, seg.wat, ts, 1);

fprintf("轨迹总时长：%.2f s\n", sum(trj.wat(:, 1)));

% 绘制轨迹
insplot(trj.avp);

%% 4. 添加IMU误差
gyroBias = 0.01;
accBias = 7;
gyroNoise = 0.0005;
accNoise = 10e-6 * 1e5 / 3600;

imuError = imuerrset(gyroBias, accBias, gyroNoise, accNoise);
imuWithError = imuadderr(trj.imu, imuError);

%% 5. 坐标系转换
% AVP：ENU转换为NED
refPvaSimu = avpENU2NED(trj.avp);

% IMU：RFU转换为FRD
imuSimu = imuRFU2FRD(imuWithError);

%% 6. 写入truth.nav
% KF-GINS真值文件格式：
% 1    GPS周
% 2    周内秒，s
% 3~5  纬度、经度、高程，deg、deg、m
% 6~8  北、东、地速度，m/s
% 9~11 横滚角、俯仰角、航向角，deg

numNavEpoch = size(refPvaSimu, 1);

truthFile = fullfile(outputPath, "truth.nav");
fidNav = fopen(truthFile, "w");

if fidNav == -1
    error("无法创建真值文件：%s", truthFile);
end

try
    fprintf(fidNav, ...
        "%d %.4f %.10f %.10f %.4f " + ...
        "%.8f %.8f %.8f %.8f %.8f %.8f\n", ...
        refPvaSimu');

    fclose(fidNav);
    fprintf("真值文件已写入：%s\n", truthFile);
catch ME
    fclose(fidNav);
    error("写入truth.nav失败：%s", ME.message);
end

%% 7. 写入IMU_120.txt
% KF-GINS IMU文件格式：
% 1    周内秒，s
% 2~4  X、Y、Z轴角增量，rad
% 5~7  X、Y、Z轴速度增量，m/s

numImuEpoch = size(imuSimu, 1);

imuFile = fullfile(outputPath, "IMU_120.txt");
fidImu = fopen(imuFile, "w");

try
    fprintf(fidImu, ...
        "%.4f %.12e %.12e %.12e %.12e %.12e %.12e\n", ...
        imuSimu');

    fclose(fidImu);
    fprintf("IMU文件已写入：%s\n", imuFile);
catch ME
    fclose(fidImu);
    error("写入IMU_120.txt失败：%s", ME.message);
end

%% 8. 生成测距数据
rangedataget(outputPath,'right',0,1,0,0);

fprintf("全部仿真数据生成完成。\n");