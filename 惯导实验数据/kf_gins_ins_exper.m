% 纯惯导
%% 定义参数+加载过程配置
param = Param();
cfg = ProcessConfig_exper();
%% 加载数据
% imudata
imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);
% height data
truth = importdata(cfg.truthpath);
height = truth(:,[2,5]);
heightdata = height(id*100:id*100:end,:);
heightstarttime = heightdata(1, 1);
heightendtime = heightdata(end, 1);
%% 设置文件保存路径
navpath = [cfg.outputfolder, '/NavResult-pureins'];
navpath = [navpath, '.nav'];
navfp = fopen(navpath, 'wt');

%% 获取处理时间，调整时间
if cfg.starttime < imustarttime
    cfg.starttime = imustarttime;
end
if cfg.endtime > imuendtime
    cfg.endtime = imuendtime;
end
% data in process interval
imudata = imudata(imudata(:,1) >= cfg.starttime, :);
imudata = imudata(imudata(:,1) <= cfg.endtime, :);
rangedata = rangedata(rangedata(:, 1) >= cfg.starttime, :);
rangedata = rangedata(rangedata(:, 1) <= cfg.endtime, :);
heightdata = heightdata(heightdata(:, 1) >= cfg.starttime, :);
heightdata = heightdata(heightdata(:, 1) <= cfg.endtime, :);
%% 调试
disp("Start pure-ins Processing!");
lastprecent = 0;
% initialization
[kf, navstate] = myInitialize_15state(cfg);
laststate = navstate;
% data index preprocess
lastimu = imudata(1, :)';
thisimu = imudata(1, :)';
imudt = thisimu(1, 1) - lastimu(1, 1);
%% 正式纯惯导
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MAIN PROCEDD PROCEDURE!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for imuindex = 2:size(imudata, 1)-1

    % 1、set value of last state
    lastimu = thisimu;
    laststate = navstate;
    thisimu = imudata(imuindex, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);
    % 2、only do propagation
    % INS mechanization
    navstate = InsMech(laststate, lastimu, thisimu);
    navstate.pos(3) = height(imuindex,2) + randn*1; % 天向位置约束
    % navstate.vel(3) = -(navstate.pos(3) - laststate.pos(3))/imudt; % 天向速度约束
    % error propagation
    kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
    % 3、save data
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
% close file
fclose(navfp);
disp("Processing Finished!");
%%
calc_error(navpath,cfg.truthpath)