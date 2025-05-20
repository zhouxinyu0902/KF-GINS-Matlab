% 纯惯导解算
clear;
% clc;
%% 定义参数/加载配置
param = Param();
% cfg = ProcessConfig1();
% cfg = ProcessConfig2();
% cfg = ProcessConfig3();
cfg = ProcessConfigsimu();
% cfg = ProcessConfig4_zxy();
%% 导入数据
% imudata
imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);
mode={'ins/gnss','ins/range','ins/compass','ins/2range'};
% 选择模式 %
chosenmode=mode{2};
%%%%%%%%%%%
switch(chosenmode)
    case 'ins/2range'
        depthdata = importdata(cfg.depthfilepath);
        depthstarttime = depthdata(1, 1);
        depthendtime = depthdata(end, 1);
        Rangedata1 = importdata(cfg.rangefilepath);
        rangestarttime = Rangedata1(1, 1);
        rangeendtime = Rangedata1(end, 1);
        Rangedata2 = importdata(cfg.rangefilepath2);
        rangestarttime2 = Rangedata2(1, 1);
        rangeendtime2 = Rangedata2(end, 1);
        pvapath=[cfg.outputfolder,'/ins','_range.txt'];
        pvafp = fopen(pvapath,"wt");
        xkpath = [cfg.outputfolder, '/xk_2range.txt'];
        xkfp = fopen(xkpath, 'wt');
    case 'ins/range'
        depthdata = importdata(cfg.depthfilepath);
        depthstarttime = depthdata(1, 1);
        depthendtime = depthdata(end, 1);
        Rangedata = importdata(cfg.rangefilepath);
        rangestarttime = Rangedata(1, 1);
        rangeendtime = Rangedata(end, 1);
        pvapath=[cfg.outputfolder,'/ins','_range.txt'];
        pvafp = fopen(pvapath,"wt");
        xkpath = [cfg.outputfolder, '/xk_range-5.txt'];
        xkfp = fopen(xkpath, 'wt');
    case 'ins/gnss'
        gnssdata = importdata(cfg.gnssfilepath);
        gnssdata(:, 2:3) = gnssdata(:, 2:3) * param.D2R;
        gnssstarttime = gnssdata(1, 1);
        gnssendtime = gnssdata(end, 1);
        pvapath=[cfg.outputfolder,'/ins','_gnss.txt'];
        pvafp=fopen(pvapath,"wt");
        xkpath = [cfg.outputfolder, '/xk_gnss.txt'];
        xkfp = fopen(xkpath, 'wt');
    case 'ins/compass'
        compassdata = importdata(cfg.compassfilepath);
        compassstarttime = compassdata(1, 2);
        compassendtime = compassdata(end, 2);
        pvapath=[cfg.outputfolder,'/ins','_compass.txt'];
        pvafp=fopen(pvapath, "wt");
        xkpath = [cfg.outputfolder, '/xk_compass.txt'];
        xkfp = fopen(xkpath, 'wt');
    case 'pure_ins'
        pvapath=[cfg.outputfolder,'/pure_ins.txt'];
        pvafp=fopen(pvapath,"wt");
end
%% 保存结果
truthpath = 'dataset-simu/truth.nav';

% imuerrpath = [cfg.outputfolder, '/ImuError.txt'];
% imuerrfp = fopen(imuerrpath, 'wt');
% 
% stdpath = [cfg.outputfolder, '/NavSTD.txt'];
% stdfp = fopen(stdpath, 'wt');


%% 统一处理时间
if cfg.starttime < imustarttime
    cfg.starttime = imustarttime;
    disp('处理开始时间小于IMU的开始时间')
end
if cfg.endtime > imuendtime
    cfg.endtime = imuendtime;
    disp('处理结束时间大于IMU的结束时间')
end
imudata = imudata(imudata(:,1) >= cfg.starttime, :);
imudata = imudata(imudata(:,1) <= cfg.endtime, :);
switch(chosenmode)
    case 'ins/2range'
        depthdata = depthdata(depthdata(:,1) >= cfg.starttime, :);
        depthdata = depthdata(depthdata(:,1) <= cfg.endtime, :);
        Rangedata1 = Rangedata1(Rangedata1(:,1) >= cfg.starttime, :);
        Rangedata1 = Rangedata1(Rangedata1(:,1) <= cfg.endtime, :);
        Rangedata2 = Rangedata2(Rangedata2(:,1) >= cfg.starttime, :);
        Rangedata2 = Rangedata2(Rangedata2(:,1) <= cfg.endtime, :);
        id=1;
        Range1=Rangedata1(id*200,:);
        Range2=Rangedata2(id*200,:);
        depth=depthdata(id*200,:);
        disp("Start 2RANGE/INS Processing!");
    case 'ins/range'
        depthdata = depthdata(depthdata(:,1) >= cfg.starttime, :);
        depthdata = depthdata(depthdata(:,1) <= cfg.endtime, :);
        Rangedata = Rangedata(Rangedata(:,1) >= cfg.starttime, :);
        Rangedata = Rangedata(Rangedata(:,1) <= cfg.endtime, :);
        id=1;
        Range=Rangedata(id*20000,:);
        depth=depthdata(id*20000,:);
        disp("Start RANGE/INS Processing!");
    case 'ins/gnss'
        gnssdata = gnssdata(gnssdata(:,1) >= cfg.starttime, :);
        gnssdata = gnssdata(gnssdata(:,1) <= cfg.endtime, :);
        id=1;
        gnss=gnssdata(id,:);
        disp("Start GNSS/INS Processing!");
    case 'ins/compass'
        compassdata = compassdata(compassdata(:,1) >= cfg.starttime, :);
        compassdata = compassdata(compassdata(:,1) <= cfg.endtime, :);
        id=1;
        compass=compassdata(id*200,:);
    disp("Start compass/INS Processing!");
end
%% for debug
lastprecent = 0;
% initialization
tic;
% navstate = InitializeCfg(cfg);
[kf, navstate] = myInitialize(cfg);
laststate = navstate;
ll=length(imudata);

thisimu = imudata(1,:)';
starttime = thisimu(1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MAIN PROCEDD PROCEDURE!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for imuindex = 2:ll-1
    lastimu = thisimu;
    laststate = navstate;
    thisimu = imudata(imuindex, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);
    % 惯导解算
    navstate = InsMech(laststate, lastimu, thisimu);
    % navstate.pos(3)=-1200;
    % 惯导传播，计算状态转移矩阵，一步预测
    kf = myInsPropagate(navstate, thisimu, imudt, kf);
    % phi(imuindex,:) = poscalyaw(laststate.pos,navstate.pos);
    % figure,plot(1:length(phi),phi,1:length(trueyaw),trueyaw)

    % %gnss位置进行约束
    % if gnss(1)==thisimu(1)
    %     % 测量值更新
    %     kf = myGNSSUpdate(navstate, gnss, kf);
    % 
    %     % 估计状态值反馈
    %     % [kf, navstate] = myErrorFeedback(kf, navstate);
    %     % 取下一个测量值
    %     id=id+1;
    %     gnss=gnssdata(id,:);
    % end

    % % % 两个距离值进行约束
    % if Range1(1)==thisimu(1)
    %     % 测量值更新
    %     kf = my2RangeUpdate(navstate, Range1, Range2, depth, kf);
    %     % 估计状态值反馈
    %     % [kf, navstate] = myErrorFeedback(kf, navstate);
    %     % 取下一个测量值
    %     id=id+1;
    %     Range1=Rangedata1(id*200,:);
    %     Range2=Rangedata2(id*200,:);
    %     depth=depthdata(id*200,:);
    % end

    % % 一个距离值进行约束
    if Range(1)==thisimu(1)
        % 测量值更新
        kf = myRangeUpdate(navstate, Range, depth, kf);
        % 估计状态值反馈
        [kf, navstate] = myErrorFeedback(kf, navstate);
        % 取下一个测量值
        id=id+1;
        Range=Rangedata(id*20000,:);
        depth=depthdata(id*20000,:);
    end
    
    % % 罗盘信息进行约束
    % if compass(2)==thisimu(1)
    %     % 测量值更新
    %     kf = myCompassUpdate(navstate, laststate, compass, kf);
    %     % 估计状态值反馈
    %     % [kf, navstate] = myErrorFeedback(kf, navstate);
    %     % 取下一个测量值
    %     id=id+1;
    %     compass=compassdata(id*200,:);
    % end

    % compensate IMU error
    % thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * navstate.gyrbias)./(ones(3, 1) + navstate.gyrscale);
    % thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * navstate.accbias)./(ones(3, 1) + navstate.accscale);
    
    % 保存导航结果
    nav = zeros(11, 1);
    nav(2, 1) = navstate.time;
    nav(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
    nav(6:8, 1) = navstate.vel;
    nav(9:11, 1) = navstate.att * param.R2D;
    fprintf(pvafp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);
    
    % 保存估计的状态值
    xk = zeros(16, 1);
    xk(1) = navstate.time;
    xk(2:16) = kf.x;
    fprintf(xkfp, '%12.6f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f\n', xk);

    % % write state std, convert to common unit
    % std = zeros(1, 22);
    % std(1) = navstate.time;
    % for idx=1:15
    %     std(idx + 1) = sqrt(kf.P(idx, idx));
    % end
    % std(8:10) = std(8:10) * param.R2D;
    % std(11:13) = std(11:13) * param.R2D *3600;
    % std(14:16) = std(14:16) * 1e5;
    % fprintf(stdfp, '%12.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f \n', std);
    % 
    % % write imu error, convert to common unit
    % imuerror = zeros(13, 1);
    % imuerror(1, 1) = navstate.time;
    % imuerror(2:4, 1) = navstate.gyrbias * param.R2D * 3600;
    % imuerror(5:7, 1) = navstate.accbias * 1e5;
    % imuerror(8:10, 1) = navstate.gyrscale * 1e6;
    % imuerror(11:13, 1) = navstate.accscale * 1e6;
    % fprintf(imuerrfp, '%12.6f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', imuerror);

    
    % 打印处理信息
    if (imuindex / (ll - 1) - lastprecent> 0.05)
        disp("processing " + num2str(floor(imuindex * 100 / (ll-1))) + " %!");
        lastprecent = imuindex / (ll - 1);
    end
end
tt=toc;
disp(['处理完毕，数据从',num2str(starttime),' s，到',num2str(thisimu(1)),' s，总共',...
    num2str(navstate.time-starttime),' s'])
disp(['代码运行时间为: ', num2str(tt), ' 秒']);

%% close file
fclose(pvafp);
fclose(xkfp);
%%
% fclose(stdfp);
% fclose(imuerrfp);
% disp("GNSS/INS Integration Processing Finished!");
%%
% calc_error(pvapath,truthpath)
%%
plot_xk(xkpath,pvapath,truthpath)
close all
plot_xk('xk_range-4.txt',pvapath,truthpath)
plot_xk('xk_range-3.txt',pvapath,truthpath)
plot_xk('xk_range-2.txt',pvapath,truthpath)
plot_xk('xk_range-1.txt',pvapath,truthpath)
plot_xk('xk_range.txt',pvapath,truthpath)
plot_xk('xk_gnss.txt',pvapath,truthpath)

%%
% plot_result(pvapath,'single')
% plot_result(truthpath)
plot_cmp(pvapath,truthpath)
legend('start','ref','start','ins')
