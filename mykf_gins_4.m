% 纯惯导解算
clear;
clc;
%% 定义参数/加载配置
param = Param();
cfg = ProcessConfig4_zxy();
%% 导入imu数据
% imudata
imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);
%% 保存结果
pvapath="D:/GitHub/KF-GINS-Matlab/dataset4/pure_ins_i300.txt";
pvafp=fopen(pvapath,"wt");

%% 统一处理时间
if cfg.starttime < imustarttime
    cfg.starttime = imustarttime;
end
if cfg.endtime > imuendtime
    cfg.endtime = imuendtime;
end
imudata = imudata(imudata(:,1) >= cfg.starttime, :);
imudata = imudata(imudata(:,1) <= cfg.endtime, :);
%% for debug
disp("Start GNSS/INS Processing!");
lastprecent = 0;
% initialization
tic;
navstate = InitializeCfg(cfg);
laststate = navstate;
ll=length(imudata);
avp=prealloc(ll-1,10);
avp(1,:)=[navstate.att;navstate.vel;navstate.pos;0]';
thisimu = imudata(1,:)';
starttime = thisimu(1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MAIN PROCEDD PROCEDURE!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for imuindex = 2:ll
    % set value of last state
    
    lastimu = thisimu;
    laststate = navstate;
    thisimu = imudata(imuindex, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);

    % % compensate IMU error
    % thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * navstate.gyrbias)./(ones(3, 1) + navstate.gyrscale);
    % thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * navstate.accbias)./(ones(3, 1) + navstate.accscale);

    % INS mechanization
    navstate = InsMech(laststate, lastimu, thisimu);
    % write navresult to file
    nav = zeros(11, 1);
    nav(2, 1) = navstate.time;
    nav(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
    nav(6:8, 1) = navstate.vel;
    nav(9:11, 1) = navstate.att * param.R2D;
    fprintf(pvafp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);
    % print processing information
    if (imuindex / (ll - 1) - lastprecent> 0.01)
        disp("processing " + num2str(floor(imuindex * 100 / (ll-1))) + " %!");
        lastprecent = imuindex / (ll - 1);
    end
end
tt=toc;
disp(['处理完毕，数据从',num2str(starttime),' s，到',num2str(thisimu(1)),' s，总共',...
    num2str(navstate.time-starttime),' s'])
disp(['代码运行时间为: ', num2str(tt), ' 秒']);

% close file
fclose(pvafp);
disp("GNSS/INS Integration Processing Finished!");
%%
truthpath = 'dataset4/truth-i300.nav';
calc_error(pvapath,truthpath)
plot_result(pvapath)
plot_result(truthpath)