clear;
% 支持纯惯导+1s gnss
%% 定义全局参数
rng(1)
feedback = 0; % 是否反馈，不反馈则可以观察参数
imuerrrecord = 1;
stdrecord = 1;
glvs
%% 定义参数+加载过程配置
param = Param();
cfg = ProcessConfig_exper();
%% 加载数据

imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);


gnss = importdata(cfg.gnssfilepath);
stdd = importdata(cfg.stdfilepath);
gnssdata = [gnss(:,2:5),stdd(:,2:4),gnss(:,6:8),stdd(:,5:7)];
gnssdata(:, 2:3) = gnssdata(1:100:end, 2:3) * param.D2R;

gnssstarttime = gnssdata(1, 1);
gnssendtime = gnssdata(end, 1);

heightdata = importdata(cfg.heightfilepath);
%% 获取处理时间，调整时间
starttime = imustarttime;
endtime = imuendtime;
if cfg.starttime < starttime
    cfg.starttime = starttime;
end
if cfg.endtime > endtime
    cfg.endtime = endtime;
end

imudata = imudata(imudata(:,1) >= cfg.starttime, :);
imudata = imudata(imudata(:,1) <= cfg.endtime, :);
gnssdata = gnssdata(gnssdata(:, 1) >= cfg.starttime, :);
gnssdata = gnssdata(gnssdata(:, 1) <= cfg.endtime, :);
heightdata = heightdata(heightdata(:, 1) >= cfg.starttime, :);
heightdata = heightdata(heightdata(:, 1) <= cfg.endtime, :);
%% 设置文件保存路径
navpath = [cfg.outputfolder, '/NavResult-gnss.nav'];
navfp = fopen(navpath, 'wt');
if imuerrrecord == 1
    imuerrpath = [cfg.outputfolder, '/NavResult-gnss-ImuError.txt'];
    imuerrfp = fopen(imuerrpath, 'wt');
end
if stdrecord == 1
    stdpath = [cfg.outputfolder, '/NavSTD.txt'];
    stdfp = fopen(stdpath, 'wt');
end
if feedback == 0
    xkpath = [cfg.outputfolder, '/xk-gnss.txt'];
    xkfp = fopen(xkpath, 'wt');
end
%% 调试
disp("Start GNSS/INS Processing!");
lastprecent = 0;
%% 初始化
[kf, navstate] = myInitialize_15state(cfg);
laststate = navstate;

lastimu = imudata(1, :)';
thisimu = imudata(1, :)';
imudt = thisimu(1, 1) - lastimu(1, 1);

gnssindex = 1;
while gnssdata(gnssindex, 1) < thisimu(1, 1)
    gnssindex = gnssindex + 1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MAIN PROCEDD PROCEDURE!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for imuindex = 2:size(imudata, 1)

    %% set value of last state
    lastimu = thisimu;
    laststate = navstate;
    thisimu = imudata(imuindex, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);

    %% compensate IMU error
    % thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * navstate.gyrbias)./(ones(3, 1) + navstate.gyrscale);
    % thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * navstate.accbias)./(ones(3, 1) + navstate.accscale);

    %% adjust GNSS index
    while (gnssindex <= size(gnssdata, 1) && gnssdata(gnssindex, 1) < lastimu(1, 1))
        gnssindex = gnssindex + 1;
    end
    % check whether gnss data is valid
    if (gnssindex > size(gnssdata, 1))
        disp('GNSS file END!');
        break;
    end

    %% determine whether gnss update is required
    if lastimu(1, 1) == gnssdata(gnssindex, 1)
        % do gnss update for the current state
        thisgnss = gnssdata(gnssindex, :)';
        kf = myGNSSUpdate_15state(navstate, thisgnss, kf);
        % kf = myGNSSUpdate(navstate, thisgnss, kf, cfg.antlever);
        if feedback==1
            [kf, navstate] = myErrorFeedback_noatt(kf, navstate);
            % [kf, navstate] = myErrorFeedback_15state(kf, navstate);
        end
        gnssindex = gnssindex + 1;
        laststate = navstate;

        % do propagation for current imu data
        imudt = thisimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, thisimu);
        % navstate.pos(3) = heightdata(imuindex,2);
        kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
    elseif (lastimu(1, 1) < gnssdata(gnssindex, 1) && thisimu(1, 1) > gnssdata(gnssindex, 1))
        % ineterpolate imu to gnss time
        [firstimu, secondimu] = interpolate(lastimu, thisimu, gnssdata(gnssindex, 1));

        % do propagation for first imu
        imudt = firstimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, firstimu);
        navstate.pos(3) = heightdata(imuindex,2);
        kf = myInsPropagate_15state(navstate, firstimu, imudt, kf);

        % do gnss update
        thisgnss = gnssdata(gnssindex, :)';
        kf = myGNSSUpdate_15state(navstate, thisgnss, kf);
        % kf = myGNSSUpdate(navstate, thisgnss, kf, cfg.antlever);
        if feedback==1
            [kf, navstate] = myErrorFeedback_noatt(kf, navstate);
            % [kf, navstate] = myErrorFeedback_15state(kf, navstate);
        end
        gnssindex = gnssindex + 1;
        laststate = navstate;
        lastimu = firstimu;

        % do propagation for second imu
        imudt = secondimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, secondimu);
        % navstate.pos(3) = heightdata(imuindex,2);
        kf = myInsPropagate_15state(navstate, secondimu, imudt, kf);
    else
        %% only do propagation
        navstate = InsMech(laststate, lastimu, thisimu);
        % navstate.pos(3) = heightdata(imuindex,2);
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
    % % 保存估计的状态值
    
    % xk = zeros(16, 1);
    % xk(1) = navstate.time;
    % xk(2:16) = kf.x(1:15);
    % fprintf(xkfp, '%12.6f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f\n', xk);
    if imuerrrecord == 1
        imuerror = zeros(13, 1);
        imuerror(1, 1) = navstate.time;
        imuerror(2:4, 1) = navstate.gyrbias * param.R2D * 3600;
        imuerror(5:7, 1) = navstate.accbias * 1e5;
        imuerror(8:10, 1) = navstate.gyrscale * 1e6;
        imuerror(11:13, 1) = navstate.accscale * 1e6;
        fprintf(imuerrfp, '%12.6f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', imuerror);
    end
    if stdrecord == 1
        std = zeros(1, length(kf.P)+1);
        std(1) = navstate.time;
        for idx=1:length(kf.P)
            std(idx + 1) = sqrt(kf.P(idx, idx));
        end
        std(8:10) = std(8:10) * param.R2D;
        std(11:13) = std(11:13) * param.R2D *3600;
        std(14:16) = std(14:16) * 1e5;
        fprintf(stdfp, '%12.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f \n', std);
    end

    %% print processing information
    if (imuindex / size(imudata, 1) - lastprecent > 0.20) 
        disp("processing " + num2str(floor(imuindex * 100 / size(imudata, 1))) + " %!");
        lastprecent = imuindex / size(imudata, 1);
    end
end
%%
fclose all;
disp("gnss/INS Integration Processing Finished!");
%%
fig = calc_error(navpath, cfg.pureinsfilepath);

exportgraphics(fig, fullfile('D:\GitHub\KF-GINS-Matlab\惯导实验数据\fig\', ...
    'Radial-error.png'), 'Resolution', 600);
