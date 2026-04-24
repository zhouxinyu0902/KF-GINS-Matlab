% 支持纯惯导
%% 定义全局参数
clear;
glvs
%% 定义参数+加载过程配置
param = Param();
cfg = ProcessConfig_exper();
%% 加载数据
% imudata
imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);
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
% data in process interval
imudata = imudata(imudata(:,1) >= cfg.starttime, :);
imudata = imudata(imudata(:,1) <= cfg.endtime, :);
heightdata = heightdata(heightdata(:,1) >= cfg.starttime, :);
heightdata = heightdata(heightdata(:,1) <= cfg.endtime, :);
%% 设置文件保存路径
navpath = [cfg.outputfolder, '/NavResult-pureins-height.nav'];
navfp = fopen(navpath, 'wt');
%% 调试
disp("Start GNSS/INS Processing!");
lastprecent = 0;
%% 初始化
[kf, navstate] = myInitialize_15state(cfg);
laststate = navstate;

lastimu = imudata(1, :)';
thisimu = imudata(1, :)';
imudt = thisimu(1, 1) - lastimu(1, 1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MAIN PROCEDD PROCEDURE!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for imuindex = 2:size(imudata, 1)
    lastimu = thisimu;
    laststate = navstate;
    thisimu = imudata(imuindex, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);
    %% only do propagation
    % INS mechanization
    navstate = InsMech(laststate, lastimu, thisimu);
    navstate.pos(3) = heightdata(imuindex,2);
    % error propagation
    kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
    %% save data
    nav = zeros(11, 1);
    nav(2, 1) = navstate.time;
    nav(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
    nav(6:8, 1) = navstate.vel;
    nav(9:11, 1) = navstate.att * param.R2D;
    fprintf(navfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);
    
    %% print processing information
    if (imuindex / size(imudata, 1) - lastprecent > 0.20) 
        disp("processing " + num2str(floor(imuindex * 100 / size(imudata, 1))) + " %!");
        lastprecent = imuindex / size(imudata, 1);
    end
end
%%
fclose(navfp);
disp("PureIns Integration Processing Finished!");
%%
plot_result(navpath)  

