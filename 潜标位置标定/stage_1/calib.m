% close all
clear
glvs
%% 定义参数+加载过程配置
param = Param();
cfg = Config_10state('D:\Github\KF-GINS-Matlab\潜标位置标定\stage_1\data1\','align');
%% 加载数据
% imudata
imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);
truth = importdata(cfg.truthpath);
feedback = 1;
id = 2;
%% 获取距离
% 构造距离信息
rangpath1 = "D:\Github\KF-GINS-Matlab\潜标位置标定\stage_1\data1\input\range1.txt";
rangpath2 = "D:\Github\KF-GINS-Matlab\潜标位置标定\stage_1\data1\input\range2.txt";
rangpath3 = "D:\Github\KF-GINS-Matlab\潜标位置标定\stage_1\data1\input\range3.txt";
if id==1
    rangedata1 = importdata(rangpath1);
    rangedata2 = importdata(rangpath2);
    rangedata3 = importdata(rangpath3);
else
    rangpath11 = "D:\Github\KF-GINS-Matlab\潜标位置标定\stage_1\data1\input\range1_calib_1_20s.txt";
    rangpath22 = "D:\Github\KF-GINS-Matlab\潜标位置标定\stage_1\data1\input\range2_calib_1_20s.txt";
    rangpath33 = "D:\Github\KF-GINS-Matlab\潜标位置标定\stage_1\data1\input\range3_calib_1_20s.txt";
    rangedata1 = importdata(rangpath11);
    rangedata2 = importdata(rangpath22);
    rangedata3 = importdata(rangpath33);
end
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
heightdata = truth(:,[2,5]);
heightdata = heightdata(heightdata(:,1) >= cfg.starttime, :);
heightdata = heightdata(heightdata(:,1) <= cfg.endtime, :);
heightdata(:,2) = heightdata(:,2) + normrnd(0,0.2,size(heightdata(:,2)));
rangedata1 = rangedata1(rangedata1(:, 1) >= cfg.starttime, :);
rangedata1 = rangedata1(rangedata1(:, 1) <= cfg.endtime, :);
rangedata2 = rangedata2(rangedata2(:, 1) >= cfg.starttime, :);
rangedata2 = rangedata2(rangedata2(:, 1) <= cfg.endtime, :);
rangedata3 = rangedata3(rangedata3(:, 1) >= cfg.starttime, :);
rangedata3 = rangedata3(rangedata3(:, 1) <= cfg.endtime, :);
%% 设置文件保存路径
navpath = [cfg.outputfolder, '/NavResult.nav'];
navfp = fopen(navpath, 'wt');

xkpath = [cfg.outputfolder, '/xk.nav'];
xkfp = fopen(xkpath, 'wt');
%% 调试
disp("Start Processing!");
lastprecent = 0;
%% 初始化
[kf, navstate] = myInitialize_10state(cfg);
if id==2
    % --- 【手写扩展核心】：并入潜标静态流场不确定度 ---
    kf.P(9, 9)     = power(5.0 * glv.deg, 2);                  % 9:   潜标共同流场倾角（放宽到3度）
    kf.P(10, 10)   = power(5.0 * glv.deg, 2);                % 10:  潜标方位角不确定度（全向360度）
    kf.P0 = kf.P;
    navstate.theta_calib = d2r(2.8); % 弧度
    navstate.phi_calib   = d2r(68); % 弧度
end

kf.depthstd = 0.2;
laststate = navstate;
kf.rangstd = 6;
lastimu = imudata(1, :)';
thisimu = imudata(1, :)';
imudt = thisimu(1, 1) - lastimu(1, 1);

rangeindex = 2;
rangedata = [rangedata1(rangeindex,:);
    rangedata2(rangeindex,:);
    rangedata3(rangeindex,:)];
while rangedata(rangeindex, 1) < thisimu(1, 1)
    rangeindex = rangeindex + 1;

end
cfg.userange = 1;
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
    
    while (rangeindex <= size(rangedata1, 1) && rangedata1(rangeindex, 1) < lastimu(1, 1))
        rangeindex = rangeindex + 1;
    end

    if (rangeindex > 98)
        % rangeindex = rangeindex - 1;
        disp('range file END!');
        break;
    end
    rangedata = [rangedata1(rangeindex,:);
        rangedata2(rangeindex,:);
        rangedata3(rangeindex,:)];
    %% 4、determine whether gnss update is required
    if lastimu(1, 1) == rangedata1(rangeindex, 1) && cfg.userange==1
        % 测量更新
        d_sub = [1000+randn*0.2; 1015+randn*0.2; 985+randn*0.2];
        kf = myRangeUpdate_10state_m(navstate, rangedata, d_sub, kf);
        rangeindex = rangeindex + 1;
        xk = zeros(11, 1);
        xk(1) = navstate.time;
        xk(2:11) = kf.x;
        xk(10) = navstate.theta_calib;
        xk(11) = navstate.phi_calib;
        fprintf(xkfp, '%12.6f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f \n', xk);
        if feedback==1
            [kf, navstate] = myErrorFeedback_range_m_10(kf, navstate);
            % [kf, navstate] = myErrorFeedback_15state(kf, navstate);
        end
        laststate = navstate;      

        % 惯导推算
        imudt = thisimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, thisimu);
        kf = myInsPropagate_10state(navstate, thisimu, imudt, kf);
    else
        %% only do propagation
        % INS mechanization
        navstate = InsMech(laststate, lastimu, thisimu);
        navstate.pos(3) = heightdata(imuindex,2);

        
        % error propagation
        kf = myInsPropagate_10state(navstate, thisimu, imudt, kf);

    end

    % 6、save data
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
disp("PureIns Integration Processing Finished!");
%%
% plot_result(navpath)
%%
% calc_radial_error(cfg.truthpath,navpath)
calc_error(navpath,cfg.truthpath);
xk = importdata(xkpath);
%%
myfigurestartup(7,3,'zxy');
subplot(1,2,1)
plot(r2d(xk(:,10)), 'LineWidth', 1.5)
hold on
plot(20*ones(size(xk(:,10))), '--r', 'LineWidth', 1.5)
grid on
xlabel('迭代步')
ylabel('倾角 \theta (deg)')
legend('估计值', '真值 20°', 'Location', 'best')

subplot(1,2,2)
plot(r2d(xk(:,11)), 'LineWidth', 1.5)
hold on
plot(45*ones(size(xk(:,11))), '--r', 'LineWidth', 1.5)
grid on
xlabel('迭代步')
ylabel('方位角 \phi (deg)')
legend('估计值', '真值 45°', 'Location', 'best')
%%
load('D:\Github\KF-GINS-Matlab\潜标位置标定\stage_1\data1\beacon_pos.mat')
theta_est =  xk(end,10);
phi_est = xk(end,11);
if id ==1
    outputExcelName ='D:\Github\KF-GINS-Matlab\潜标位置标定\stage_1\data1\标定结果_1.xlsx' ;
    [S_est_xyz,theta_next,phi_next] = show_result1(1, S_gnss_geo, S_true_geo, pos0_geo, theta_est, phi_est, 20, 45,outputExcelName);
else
    outputExcelName ='D:\Github\KF-GINS-Matlab\潜标位置标定\stage_1\data1\标定结果_2.xlsx' ;
    [S_est_xyz,theta_next,phi_next] = show_result1(2, S_gnss_geo, S_true_geo, pos0_geo, theta_est, phi_est, 2.82, 68,outputExcelName);
end
%%
pathpos='D:\Github\KF-GINS-Matlab\潜标位置标定\stage_1\data1\beacon_pos.mat';
outputfolder = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_1\data1\input\';
path_range = {rangpath1,rangpath2,rangpath3};
outputFiles = range_reconstruct1( ...
    path_range, ...
    pathpos, ...
    outputfolder, ...
    id, ...
    S_est_xyz, ...
    '20s');
