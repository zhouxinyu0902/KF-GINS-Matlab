clear;
% clc;
%% 定义参数和加载配置
param = Param();
cfg = ProcessConfig1_zxy();
% filepath='dataset-simu\line';
% cfg = ProcessConfigsimu(filepath);
global rangstd
rangstd = 5;
global depstd
depstd = 0.5;
%% 导入数据
% imudata
imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);

% range data

cfg.userange=1;

rangedata1 = importdata(cfg.rangefile1path);
rangedata2 = importdata(cfg.rangefile2path);
rangedata3 = importdata(cfg.rangefile3path);
rangedata4 = importdata(cfg.rangefile4path);
rangedata5 = importdata(cfg.rangefile5path);

id=1;
rangedata1 = rangedata1(id:id:end,:);
% rangedata2 = rangedata2(id:id:end,:);
% rangedata3 = rangedata3(id:id:end,:);
% rangedata = zeros(size(rangedata1));
% 
rangedata = rangedata1;
% rangedata(1:3:end,:)=rangedata1(1:3:end,:);
% rangedata(2:3:end,:)=rangedata2(2:3:end,:);
% rangedata(3:3:end,:)=rangedata3(3:3:end,:);
rangestarttime = rangedata(1, 1);
rangeendtime = rangedata(end, 1);
% height data

heightdata = importdata(cfg.depthfilepath);
heightdata = heightdata(id:id:end,:);
heightstarttime = heightdata(1, 1);
heightendtime = heightdata(end, 1);
%% 设置文件保存路径
navpath = [cfg.outputfolder, '/NavResult'];
if cfg.userange
    navpath = [navpath, '_RANGE'];
    disp("use RANGE data!");
end

% navpath = [navpath, '_pureINS'];
navpath = [navpath, '.nav'];
navfp = fopen(navpath, 'wt');

imuerrpath = [cfg.outputfolder, '/ImuError.txt'];
imuerrfp = fopen(imuerrpath, 'wt');

stdpath = [cfg.outputfolder, '/NavSTD.txt'];
stdfp = fopen(stdpath, 'wt');

xkpath = [cfg.outputfolder, '/xk_range.txt'];
xkfp = fopen(xkpath, 'wt');
%% 获取处理时间，调整时间
starttime = imustarttime;
if imuendtime > rangeendtime
    endtime = rangeendtime;
else
    endtime = imuendtime;
end
if cfg.starttime < starttime
    cfg.starttime = starttime;
end
if cfg.endtime > endtime
    cfg.endtime = endtime;
end

imudata = imudata(imudata(:,1) >= cfg.starttime, :);
imudata = imudata(imudata(:,1) <= cfg.endtime, :);

rangedata = rangedata(rangedata(:, 1) >= cfg.starttime, :);
rangedata = rangedata(rangedata(:, 1) <= cfg.endtime, :);
heightdata = heightdata(heightdata(:, 1) >= cfg.starttime, :);
heightdata = heightdata(heightdata(:, 1) <= cfg.endtime, :);


%% 调试
disp("Start RANGE/INS Processing!");
lastprecent = 0;

%% 初始化 
[kf, navstate] = myInitialize_15state(cfg);
% for i=1:15
% disp(diag(kf.P(i,i)))
% end
laststate = navstate;

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
kk=1;
for imuindex = 2:size(imudata, 1)-1

    % 设置上一个状态的值
    lastimu = thisimu;
    laststate = navstate;
    thisimu = imudata(imuindex, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);
    % 补偿IMU误差
    % thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * navstate.gyrbias)./(ones(3, 1) + navstate.gyrscale);
    % thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * navstate.accbias)./(ones(3, 1) + navstate.accscale);

    % 调整索引
    while (rangeindex <= size(rangedata, 1) && rangedata(rangeindex, 1) < lastimu(1, 1))
        rangeindex = rangeindex + 1;
    end    
    if (rangeindex > size(rangedata, 1))
        disp('Range file END!');
        break;
    end
 
    % 区分是否需要距离更新
    if lastimu(1, 1) == rangedata(rangeindex, 1)
        % 先对当前状态进行测量更新
        thisRange = rangedata(rangeindex, :);
        depthdata = heightdata(rangeindex, :);
        kf = myRangeUpdate(navstate, thisRange, depthdata, kf);
        [kf, navstate] = myErrorFeedback_15state(kf, navstate);
        rangeindex = rangeindex + 1;
        laststate = navstate;

        % 惯导解算
        imudt = thisimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, thisimu);
        kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
    elseif (lastimu(1, 1) < rangedata(rangeindex, 1) && thisimu(1, 1) > rangedata(rangeindex, 1))
        % 将imu插值到测量值的时间
        [firstimu, secondimu] = interpolate(lastimu, thisimu, rangedata(rangeindex, 1));

        % 前一个imu到测量值时间的惯导更新
        imudt = firstimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, firstimu);
        kf = myInsPropagate_15state(navstate, firstimu, imudt, kf);

        % 测量更新
        thisRange = rangedata(rangeindex, :);
        depthdata = heightdata(rangeindex, :);
        kf = myRangeUpdate(navstate, thisRange, depthdata, kf);
        [kf, navstate] = myErrorFeedback_15state(kf, navstate);
        rangeindex = rangeindex + 1;
        laststate = navstate;
        lastimu = firstimu;

        % 测量时间到下一个惯导值的更新
        imudt = secondimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, secondimu);
        kf = myInsPropagate_15state(navstate, secondimu, imudt, kf);
    else
        % 仅做惯导结算
        % INS 编排
        navstate = InsMech(laststate, lastimu, thisimu);
        % 尝试做天向速度和位置约束
        % navstate.vel(3)=(-heightdata(imuindex,1)-(-heightdata(imuindex-1,1)))/imudt;
        % navstate.pos(3)=-heightdata(imuindex,1);
        % error propagation
        % if mod(imuindex,20)==0
        %     navstate.pos(3)=heightdata(floor(imuindex/20),1);
        % end
        % 状态传播
        kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
        % disp('predict\')
        % for i=1:15
        %     disp(diag(kf.P(i,i)))
        % end
    end

    %% save data
    % write navresult to file
    nav = zeros(11, 1);
    nav(2, 1) = navstate.time;
    nav(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
    nav(6:8, 1) = navstate.vel;
    nav(9:11, 1) = navstate.att * param.R2D;
    fprintf(navfp, '%2d %12.9f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);
    % % 保存估计的状态值
    % xk = zeros(16, 1);
    % xk(1) = navstate.time;
    % xk(2:16) = kf.x(1:15);
    % fprintf(xkfp, '%12.6f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f\n', xk);
    % % 
    % % write imu error, convert to common unit
    % imuerror = zeros(13, 1);
    % imuerror(1, 1) = navstate.time;
    % imuerror(2:4, 1) = navstate.gyrbias * param.R2D * 3600;
    % imuerror(5:7, 1) = navstate.accbias * 1e5;
    % imuerror(8:10, 1) = navstate.gyrscale * 1e6;
    % imuerror(11:13, 1) = navstate.accscale * 1e6;
    % fprintf(imuerrfp, '%12.6f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', imuerror);
    % 
    % % write state std, convert to common unit
    % std = zeros(1, 22);
    % std(1) = navstate.time;
    % for idx=1:21
    %     std(idx + 1) = sqrt(kf.P(idx, idx));
    % end
    % std(8:10) = std(8:10) * param.R2D;
    % std(11:13) = std(11:13) * param.R2D *3600;
    % std(14:16) = std(14:16) * 1e5;
    % std(17:22) = std(17:22) * 1e6;
    % fprintf(stdfp, '%12.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f \n', std);
    % 

    % print processing information
    % if (imuindex / size(imudata, 1) - lastprecent > 0.05) 
    %     disp("processing " + num2str(floor(imuindex * 100 / size(imudata, 1))) + " %!");
    %     lastprecent = imuindex / size(imudata, 1);
    % end
end
% 关闭文件
% fclose(imuerrfp);
fclose(navfp);
% fclose(stdfp);
fclose(xkfp);
disp("range/INS Integration Processing Finished!");
%%
navc="D:\Github\KF-GINS-main\dataset\KF_GINS_Navresult.nav";
truthpath=cfg.truthpath;
%%
% plot_xk(xkpath,navpath,truthpath)
% 
%%
% plot_imuerror
%%
calc_error(navpath,cfg.truthpath)
%%
calc_error(navc,cfg.truthpath)