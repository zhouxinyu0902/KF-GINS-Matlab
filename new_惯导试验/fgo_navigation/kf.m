clear;
% 参数初始化
param = Param();
clc
in_dir = "D:\Github\KF-GINS-Matlab\new_惯导试验\fgo_navigation\input\";
cfg = config_simu(in_dir);
cfg.outputfolder ="D:\Github\KF-GINS-Matlab\new_惯导试验\fgo_navigation\output\";
% mkdir(cfg.outputfolder);
%% 定义标准差和其他设置
rngstd = 6;
depthstd = 0.4;
rng(1);
feedback = 0;
tic; % 启动计时器
%% 导入数据
% 导入IMU数据
imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);
% 导入范围数据
cfg.userange = 1;
rangedata1 = importdata(cfg.rangefile1path);
rangedata2 = importdata(cfg.rangefile2path);
rangedata3 = importdata(cfg.rangefile3path);
range = {rangedata1, rangedata2, rangedata3};

% 构造范围数据
id = 360; % 420秒 = 7分钟数据周期
for i = 1:3
    range{i} = range{i}(id:id:end, :);
end

% 初始化合并后的范围数据
rangedata = zeros(size(range{1}));
seq = [1, 2, 3];

for i = 1:3
    rangedata(i:3:end, :) = range{seq(i)}(i:3:end, :);
end

% 获取时间范围
rangestarttime = rangedata(1, 1);
rangeendtime = rangedata(end, 1);

% 导入高度数据
truth = importdata(cfg.truthpath);
height = truth(:, [2,5]);


%% 设置文件保存路径
if feedback==0
    navpath = fullfile(cfg.outputfolder, 'PureIns.nav');
else
    navpath = fullfile(cfg.outputfolder, 'ES-EKF.nav');
end
navfp = fopen(navpath, 'wt');
%% 时间调整

if cfg.starttime < imustarttime
    cfg.starttime = imustarttime;
end
if cfg.endtime > imuendtime
    cfg.endtime = imuendtime;
end

% 筛选在处理时间范围内的数据
imudata = imudata(imudata(:, 1) >= cfg.starttime & imudata(:, 1) <= cfg.endtime, :);
rangedata = rangedata(rangedata(:, 1) >= cfg.starttime & rangedata(:, 1) <= cfg.endtime, :);
rangedata_true = rangedata(:, 3);

% 添加随机噪声到范围数据
rangedata(:, 3) = rangedata(:, 3) + normrnd(0, rngstd, size(rangedata(:, 3)));

% 筛选高度数据并添加噪声
height = height(height(:, 1) >= cfg.starttime & height(:, 1) <= cfg.endtime, :);
height(:, 2) = height(:, 2) + normrnd(0, depthstd, size(height(:, 2)));

% writematrix(rangedata, '惯导实验数据\input_pre\rangedata_noised.txt', 'Delimiter', 'space'); % 空格
% writematrix(height, '惯导实验数据\input_pre\height_noised.txt', 'Delimiter', 'space'); % 空格

%% for debug
disp("Start INS/RANGE Processing!");
lastprecent = 0;
%% initialization
[kf, navstate] = myInitialize_15state(cfg);
laststate = navstate;
kf.rangstd = rngstd;
kf.depthstd = depthstd;
% data index preprocess
lastimu = imudata(1, :)';
thisimu = imudata(1, :)';
imudt = thisimu(1, 1) - lastimu(1, 1);

rangeindex = 1;
while rangedata(rangeindex, 1) < thisimu(1, 1)
    rangeindex = rangeindex + 1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MAIN PROCEDD PROCEDURE!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for imuindex = 2:size(imudata, 1)
    %% 1、set value of last state
    lastimu = thisimu;
    laststate = navstate;
    old_state = navstate;
    thisimu = imudata(imuindex, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);

    %% 2、compensate IMU error
    % thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * cfg.initgyrbiasstd);
    % thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * cfg.initaccbiasstd);
    %% 3、adjust range index
    while (rangeindex <= size(rangedata, 1) && rangedata(rangeindex, 1) < lastimu(1, 1))
        rangeindex = rangeindex + 1;
    end
    if (rangeindex > size(rangedata, 1))
        % rangeindex = rangeindex - 1;
        disp('range file END!');
        break;
    end
    %% 4、determine whether gnss update is required
    if lastimu(1, 1) == rangedata(rangeindex, 1) && cfg.userange==1
        % 测量更新
        kf = myRangeUpdate(navstate, rangedata(rangeindex,:), height(imuindex,:), kf);
        rangeindex = rangeindex + 1;
        if feedback==1
            [kf, navstate] = myErrorFeedback_range(kf, navstate);
            % [kf, navstate] = myErrorFeedback_15state(kf, navstate);
        end
        laststate = navstate;
        % 惯导推算
        imudt = thisimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, thisimu);
        kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
    elseif (lastimu(1, 1) < rangedata(rangeindex, 1) && thisimu(1, 1) > rangedata(rangeindex, 1))&& cfg.userange==1
        kf = myRangeUpdate(navstate, rangedata(rangeindex,:), height(imuindex,:), kf);
        if feedback == 1
            [kf, navstate] = myErrorFeedback_range(kf, navstate);
            % [kf, navstate] = myErrorFeedback_15state(kf, navstate);
        end
        rangeindex = rangeindex + 1;
        laststate = navstate;
        lastimu = firstimu;
        % do propagation for second imu
        imudt = secondimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, secondimu);
        kf = myInsPropagate_15state(navstate, secondimu, imudt, kf);
        pos0 = navstate.pos;
        vel0 = navstate.vel;
    else
        %% only do propagation
        % INS mechanization
        navstate = InsMech(laststate, lastimu, thisimu);
        % navstate.pos(3) = heightdata(imuindex,2);
        % 调用高度卡尔曼量测更新
        % kf = myHeightUpdate(navstate, height(imuindex, :), kf);
        % % 反馈修正惯导状态 (仅修正天向)
        % navstate.pos(3) = navstate.pos(3) - kf.x(3);
        % navstate.vel(3) = navstate.vel(3) - kf.x(6);
        % % 💡 反馈后，将整个误差状态量清零（代表误差已融入主状态）
        % % kf.x = zeros(size(kf.x));
        % kf.x(3) = 0;kf.x(6) = 0;
        % error propagation
        kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);

    end

    %% save data
    % xkk(imuindex-1,:)=[navstate.time;kf.x];
    % write navresult to file
    nav = zeros(11, 1);
    nav(2, 1) = navstate.time;
    nav(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
    nav(6:8, 1) = navstate.vel;
    nav(9:11, 1) = navstate.att * param.R2D;
    fprintf(navfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);
    % print processing information
    if (imuindex / size(imudata, 1) - lastprecent > 0.20)
        disp("processing " + num2str(floor(imuindex * 100 / size(imudata, 1))) + " %!");
        lastprecent = imuindex / size(imudata, 1);
    end
end
%%
fclose all;
toc
% [fig,finalExcelData] = calc_radial_error_gjb(cfg.truthpath,navpath);


