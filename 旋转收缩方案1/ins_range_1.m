clear
%% 定义全局参数
nnn=1;
seq=[1,2,3];
global rangstd;rangstd = 5;
global depstd;depstd = 0.5;
global posstd;posstd = 10;
rng(1)
glvs
%% 定义参数+加载过程配置
param = Param();
path='D:\GitHub\KF-GINS-Matlab\旋转收缩方案1/input/input6';
cfg = ProcessConfigforSemiPhy_all(path);
type = {"single","moving","3","2"};
beacontype = type{3};

type = {"Range","Range+azi","Range+azi+pos","pos"};
meas = type{1};

isoptim = 0; expand = 20; % 是否优化

backwardIsOpen_1s = 1;
IsEKFRotate = 1 ;
feedback = 1; % 是否反馈，不反馈则可以观察参数
IsHardConstrain = 0; MaxLimit = 100;
%% 获取距离数据
% get3beacons(path)
% get1beacon(path,[-10000,4000,0])
% getmovingbeacon(path,'follow')
% close all
%% 加载数据
% imudata
imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);

% 构造距离信息
id = 420;
rangedata1 = importdata(cfg.rangefile1path);
rangedata2 = importdata(cfg.rangefile2path);
rangedata3 = importdata(cfg.rangefile3path);
rangedatasingle = importdata(cfg.singlerangefilepath);
rangedatamoving = importdata(cfg.rangefilemovingpath);
range = {rangedata1,rangedata2,rangedata3,rangedatasingle,rangedatamoving};

for i=1:length(range)
    range{i} = range{i}(id:id:end,:);
end

if beacontype=="3"||beacontype=="2"
    numnum = str2double(beacontype);
    rangedata = zeros(size(range{1}));
    for i=1:numnum
        rangedata(i:numnum:end,:)=range{seq(nnn,i)}(i:numnum:end,:);
    end
elseif beacontype == "single"
    rangedata = range{4};
elseif beacontype == "moving"
    rangedata = range{5};
end

range_old = rangedata(:,3) ;
rangedata(:,3) = rangedata(:,3) + normrnd(0,rangstd,size(rangedata(:,3)));
range_new = rangedata(:,3) ;
para = struct('sigma_min', 0.1, 'sigma_max', 0.2, 'power_fac', 2, 'jitter', 0.1);
azi_old = rangedata(:, 7);
rangedata(:, 7) = add_azimuth_noise_irregular(rangedata(:, 7), para);
azi_new = rangedata(:, 7);

fig = plot_trajectory_and_beacons(cfg.truthpath, rangedata(1:3,4:6));
% rangedata(4,:)=[];
%% 方位角和距离分析
% myfigurestartup(10,10,'prese')
% % --- 1. 方位角分析 ---
% subplot 221
% plot(azi_old, 'LineWidth', 1); hold on; plot(azi_new, '--');
% legend('参考方位角','观测方位角'); grid on; ylabel('角度/deg');
% title('方位角序列对比');
% 
% subplot 222
% err_azi = mod(azi_new - azi_old + 180, 360) - 180;
% plot(err_azi,'.'); hold on; yline(rms(err_azi),'r', 'LineWidth', 1.5);
% title(sprintf('方位角残差 (RMSE: %.2f°)', rms(err_azi)));
% grid on; ylabel('偏差/deg');
% 
% % --- 2. 距离分析 ---
% subplot 223
% plot(range_old, 'LineWidth', 1); hold on; plot(range_new, '--');
% legend('参考距离','观测距离'); grid on; ylabel('距离/m');
% title('距离序列对比');
% 
% subplot 224
% err_range = range_new - range_old;
% plot(err_range,'.'); hold on; yline(rms(err_range),'r', 'LineWidth', 1.5);
% title(sprintf('距离残差 (RMSE: %.2f m)', rms(err_range)));
% grid on; ylabel('偏差/m');

%%
rangestarttime = rangedata(1, 1);
rangeendtime = rangedata(end, 1);
% 构造高度数据
truth = importdata(cfg.truthpath);
height = truth(:,[2,5]);
height(:,2) = height(:,2) + normrnd(0,depstd,size(height(:,2)));
heistarttime = height(1, 1);
heitendtime = height(end, 1);
%% 获取处理时间，调整时间
if imustarttime > heistarttime
    starttime = imustarttime;
else
    starttime = heistarttime;
end
if imuendtime > heitendtime
    endtime = heitendtime;
else
    endtime = imuendtime;
end
if cfg.starttime < starttime
    cfg.starttime = starttime;
end
if cfg.endtime > endtime
    cfg.endtime = endtime;
end
% data in process interval

imudata = imudata(imudata(:,1) >= cfg.starttime, :);
imudata = imudata(imudata(:,1) <= cfg.endtime, :);

rangedata = rangedata(rangedata(:, 1) >= cfg.starttime, :);
rangedata = rangedata(rangedata(:, 1) <= cfg.endtime, :);

height = height(height(:, 1) >= cfg.starttime, :);
height = height(height(:, 1) <= cfg.endtime, :);
%%
if backwardIsOpen_1s == 1
    ki2 = 1;
    indexrecord2(1) = 1;
    navdt1s = [];
    NAV = [];
end


if IsHardConstrain == 0
    
    dxx = zeros(1,length(rangedata));
    dyy = zeros(1,length(rangedata));
end
%%
% for type = ["PureIns","EKF","AEKF"]
for type = "EKF"
    if strcmp(type,'PureIns')
        cfg.userange=0;
        cfg.adap=0;
    elseif strcmp(type,'EKF')
        cfg.userange=1;
        cfg.adap=0;
    elseif strcmp(type,'AEKF')
        cfg.userange=1;
        cfg.adap=1;
    end
    %% 设置文件保存路径
    if cfg.userange
        navpath = [cfg.outputfolder, '/FOG-',sprintf('%s.nav',type)];
        % disp("use RANGE data!");
        % disp("Start GNSS/RANGE Processing!");
    else
        navpath =[cfg.outputfolder,'/FOG-PureIns.nav'];
        % disp("PURE INS!");
        % disp("Start INS Processing!");
    end
    % max_interv = 42;
    navfp = fopen(navpath, 'wt');

    if backwardIsOpen_1s == 1
        navdt1spath = [cfg.outputfolder,'/FOG-',sprintf('%s',type),'dt.nav']; %% 后向滤波
        navdt1sfp = fopen(navdt1spath,'wt');
        navdt1srotatepath = [cfg.outputfolder,'/FOG-',sprintf('%s',type),'dtrotate.nav']; %% 后向滤波
        navdt1srotatefp = fopen(navdt1srotatepath,'wt');
    end

    navdtmeapath = [cfg.outputfolder,'/FOG-mea.nav'];
    nav_measfp = fopen(navdtmeapath,'wt');

    if IsEKFRotate == 1
        navEKFRotatepath = [cfg.outputfolder,'/FOG-',sprintf('%s',type),'rotate.nav']; 
        navEKFRotatefp = fopen(navEKFRotatepath,'wt');

    end

    if feedback==0
        xkpath = [cfg.outputfolder, '/xk_range.txt'];
        xkfp = fopen(xkpath, 'wt');
    end

    %% 调试
    lastprecent = 0;
    %% initialization
    [kf, navstate] = myInitialize_15state(cfg);
    kf.rangstd = rangstd;
    kf.depthstd = depstd;
    Pk = zeros(length(imudata), 16 );
    % kf.Qc = kf.Qc * 5;
    % kf.P0 = 10 * kf.P;

    % kf.Qc = kf.Qc * 100;
    % kf.P0 = kf.P * 100;
    laststate = navstate;
    pos0 = navstate.pos;
    vel0 = navstate.vel;
    navstate0 = navstate;
    % data index preprocess
    lastimu = imudata(1, :)';
    thisimu = imudata(1, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);

    dtheta = zeros(3,1);
    dv=zeros(3,1);

    rangeindex = 1;
    while rangedata(rangeindex, 1) < thisimu(1, 1)
        rangeindex = rangeindex + 1;
    end
    
    nav_record = zeros(length(imudata),11);
    %% 正式滤波
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
            Rangedata = rangedata(rangeindex,:);
            depthdata = height(imuindex,1:2);

            if isoptim == 1
                optimize_imu;
            end
            
            if cfg.adap==1
                kf = myRangeUpdate_adap(navstate, Rangedata, depthdata, kf);
            else
                if meas == "Range"
                    kf = myRangeUpdate(navstate, Rangedata, depthdata, kf);
                elseif meas == "Range+azi"
                    kf = myRange_aziUpdate(navstate, Rangedata, depthdata, kf);
                elseif meas == "Range+azi+pos"
                    kf = myRange_azi_pos_Update(navstate, Rangedata, depthdata, kf);
                    % kf = myRange_azi_pos_Update_Adaptive(navstate, Rangedata, depthdata, kf);
                    nav = zeros(11, 1);
                    nav(2, 1) = navstate.time - 0.01;
                    nav(3:5, 1) = [kf.pos_rad1(1) * param.R2D; kf.pos_rad1(2) * param.R2D; navstate.pos(3)];
                    nav(6:8, 1) = navstate.vel;
                    nav(9:11, 1) = navstate.att * param.R2D;
                    fprintf(nav_measfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);
                elseif meas == "pos"
                    kf = myAziRangePosUpdate(navstate, Rangedata, depthdata, kf);
                    nav = zeros(11, 1);
                    nav(2, 1) = navstate.time - 0.01;
                    nav(3:5, 1) = [kf.pos_rad1(1) * param.R2D; kf.pos_rad1(2) * param.R2D; navstate.pos(3)];
                    nav(6:8, 1) = navstate.vel;
                    nav(9:11, 1) = navstate.att * param.R2D;
                    fprintf(nav_measfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);
                end
            end
            rangeindex = rangeindex + 1;

            if IsHardConstrain == 1
                kf = hard_constrain(navstate,pos0,imudt,kf,MaxLimit);
                % drange(rangeindex) = norm([kf.x(1)*glv.Re,kf.x(1)*glv.Re*cos(navstate.pos(1))]);
                % if drange(rangeindex) > 20
                % kf.x = 0.5*kf.x;       
            end

            if feedback==1
                % [kf, navstate] = myErrorFeedback_range(kf, navstate);
                [kf, navstate] = myErrorFeedback_15state(kf, navstate);
            end
            laststate = navstate;
            
            dxx(rangeindex) = (navstate.pos(1)- pos0(1)) * glv.Re;
            dyy(rangeindex) = (navstate.pos(2)- pos0(2)) * glv.Re *cos(pos0(1));
            % 惯导推算
            imudt = thisimu(1, 1) - lastimu(1, 1);
            navstate = InsMech(laststate, lastimu, thisimu);
            kf = myInsPropagate_15state_NED(navstate, thisimu, imudt, kf);

            if backwardIsOpen_1s == 1 
                runArgs.imudata      = imudata;
                runArgs.imuindex     = imuindex;
                runArgs.ki2          = ki2;
                runArgs.kf           = kf;
                runArgs.navstate     = navstate;
                runArgs.indexrecord2 = indexrecord2;
                runArgs.rangedata    = rangedata;
                runArgs.navdt1s      = navdt1s;
                runArgs.NAV          = NAV;
                runArgs.height       = height;
                runArgs.rangeindex   = rangeindex;
                runArgs.navstate0    = navstate0;
                runArgs.meas         = meas;
                [NAV,navdt1s,indexrecord2,ki2] = backward_1s(runArgs);
                % dd = 50;
                % if  ki2 > dd && mod(ki2,dd+1)==0
                %     dN = navdt1s (indexrecord2(ki2-dd):indexrecord2(ki2)-1,3);
                %     dE = navdt1s (indexrecord2(ki2-dd):indexrecord2(ki2)-1,4);
                %     navdt1s (indexrecord2(ki2-dd):indexrecord2(ki2)-1,3) = smooth(dN, 0.4, 'loess');
                %     navdt1s (indexrecord2(ki2-dd):indexrecord2(ki2)-1,4) = smooth(dE, 0.4, 'loess');
                % 
                %     dN = NAV (indexrecord2(ki2-dd):indexrecord2(ki2)-1,3);
                %     dE = NAV (indexrecord2(ki2-dd):indexrecord2(ki2)-1,4);
                %     NAV (indexrecord2(ki2-dd):indexrecord2(ki2)-1,3) = smooth(dN, 0.4, 'loess');
                %     NAV (indexrecord2(ki2-dd):indexrecord2(ki2)-1,4) = smooth(dE, 0.4, 'loess');
                % end
            end
            pos0 = navstate.pos;
            vel0 = navstate.vel;
            IsRangeUpdate = 1;
        elseif (lastimu(1, 1) < rangedata(rangeindex, 1) && thisimu(1, 1) > rangedata(rangeindex, 1))&& cfg.userange==1
            Rangedata = rangedata(rangeindex,:);
            depthdata = [height(imuindex,1),height(imuindex,2)];

            if isoptim == 1
                optimize_imu;
            end

            % 插值imu
            [firstimu, secondimu] = interpolate(lastimu, thisimu, rangedata(rangeindex, 1));
            % 惯导推算
            imudt = firstimu(1, 1) - lastimu(1, 1);
            navstate = InsMech(laststate, lastimu, firstimu);
            kf = myInsPropagate_15state_NED(navstate, firstimu, imudt, kf);

            % 测量更新
            if cfg.adap==1
                kf = myRangeUpdate_adap(navstate, Rangedata, depthdata, kf);
            else
                if meas == "Range"
                    kf = myRangeUpdate(navstate, Rangedata, depthdata, kf);
                elseif meas == "Range+azi"
                    kf = myRange_aziUpdate(navstate, Rangedata, depthdata, kf);
                elseif meas == "Range+azi+pos"
                    kf = myRange_azi_pos_Update(navstate, Rangedata, depthdata, kf);
                    % kf = myRange_azi_pos_Update_Adaptive(navstate, Rangedata, depthdata, kf);
                    nav = zeros(11, 1);
                    nav(2, 1) = navstate.time - 0.01;
                    nav(3:5, 1) = [kf.pos_rad1(1) * param.R2D; kf.pos_rad1(2) * param.R2D; navstate.pos(3)];
                    nav(6:8, 1) = navstate.vel;
                    nav(9:11, 1) = navstate.att * param.R2D;
                    fprintf(nav_measfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);
                elseif meas == "pos"
                    kf = myAziRangePosUpdate(navstate, Rangedata, depthdata, kf);
                    nav = zeros(11, 1);
                    nav(2, 1) = navstate.time - 0.01;
                    nav(3:5, 1) = [kf.pos_rad1(1) * param.R2D; kf.pos_rad1(2) * param.R2D; navstate.pos(3)];
                    nav(6:8, 1) = navstate.vel;
                    nav(9:11, 1) = navstate.att * param.R2D;
                    fprintf(nav_measfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);
                end

            end
            if IsHardConstrain == 1
                kf = hard_constrain(navstate,pos0,imudt,kf, MaxLimit);
                
            end

            if feedback == 1
                % [kf, navstate] = myErrorFeedback_range(kf, navstate);
                [kf, navstate] = myErrorFeedback_15state(kf, navstate);
            end

            dxx(rangeindex) = (navstate.pos(1)- pos0(1)) * glv.Re;
            dyy(rangeindex) = (navstate.pos(2)- pos0(2)) * glv.Re *cos(pos0(1));

            rangeindex = rangeindex + 1;
            laststate = navstate;
            lastimu = firstimu;

            % do propagation for second imu
            imudt = secondimu(1, 1) - lastimu(1, 1);
            navstate = InsMech(laststate, lastimu, secondimu);
            kf = myInsPropagate_15state(navstate, secondimu, imudt, kf);

            if backwardIsOpen_1s == 1
                runArgs.imudata      = imudata;
                runArgs.imuindex     = imuindex;
                runArgs.ki2          = ki2;
                runArgs.kf           = kf;
                runArgs.navstate     = navstate;
                runArgs.indexrecord2 = indexrecord2;
                runArgs.rangedata    = rangedata;
                runArgs.navdt1s      = navdt1s;
                runArgs.NAV          = NAV;
                runArgs.height       = height;
                runArgs.rangeindex   = rangeindex;
                runArgs.navstate0    = navstate0;
                runArgs.meas         = meas;
                [NAV,navdt1s,indexrecord2,ki2] = backward_1s(runArgs);
            end

            pos0 = navstate.pos;
            vel0 = navstate.vel;
            
            IsRangeUpdate = 1;
        else
            % 5、only do propagation
            thisimu(2:4, 1) = (thisimu(2:4, 1) + dtheta );
            thisimu(5:7, 1) = (thisimu(5:7, 1) + dv );

            navstate = InsMech(laststate, lastimu, thisimu);
            navstate.pos(3) = height(imuindex,2) ; % 天向位置约束
            % error propagation
            kf = myInsPropagate_15state_NED(navstate, thisimu, imudt, kf);
            IsRangeUpdate = 0;
        end
        
        Pk(imuindex-1,:)= [navstate.time;diag(kf.P)];

        
        % 6、save data
        nav = zeros(11, 1);
        nav(2, 1) = navstate.time;
        nav(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
        nav(6:8, 1) = navstate.vel;
        nav(9:11, 1) = navstate.att * param.R2D;
        fprintf(navfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);
        nav_record(imuindex-1,:) = nav;


        if IsEKFRotate == 1 && IsRangeUpdate == 1
            rotatepoint = [navstate.pos(1:2);0];% 旋转点
            index = indexrecord2(ki2-1):imuindex-2;
            
            trajectory =[d2r(nav_record(index,3:4)), zeros(length(index),1)]'; % 待旋转轨迹
            [rotatedTrajectory11, ~, ~] = rotateAndScaleTrajectory(trajectory, rotatepoint);
            rotatedTrajectory = [rotatedTrajectory11,rotatepoint];
            nav_record([index,imuindex-1],3:4) = r2d(rotatedTrajectory(1:2,:)');
            fprintf(navEKFRotatefp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_record([index,imuindex-1],:)');
        end

        % 保存估计的状态值
        if feedback == 0
            xk = zeros(16, 1);
            xk(1) = navstate.time;
            xk(2:16) = kf.x(1:15);
            fprintf(xkfp, '%12.6f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f\n', xk);
        end
        % print processing information
        if (imuindex / size(imudata, 1) - lastprecent > 0.20)
            disp("processing " + num2str(floor(imuindex * 100 / size(imudata, 1))) + " %!");
            lastprecent = imuindex / size(imudata, 1);
        end
        % check whether gnss data is valid
    end
end
% close file

fclose(navfp);
if feedback==0
    fclose(xkfp);
end
% fclose(imuerrfp);
% fclose(stdfp);
disp("range/INS Integration Processing Finished!");

Pk(imuindex-1:end,:)=[];
calc_radial_error(cfg.truthpath,navpath)

%% ------每个测距周期反向推算----
if backwardIsOpen_1s == 1
    fprintf(navdt1sfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', navdt1s');
    fprintf(navdt1srotatefp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', NAV');
    fclose all;
    calc_error(navdt1spath,cfg.truthpath);
    calc_error(navdt1srotatepath,cfg.truthpath);
    calc_radial_error(cfg.truthpath,navpath,navdt1spath);
    calc_radial_error(cfg.truthpath,navpath,navdt1spath,navdt1srotatepath);
    plot_trj(cfg.truthpath,navdt1spath,navdt1srotatepath,navpath);
end

%% -----RTS 平滑--------
if IsEKFRotate == 1 
    navdt1spath = [cfg.outputfolder,'/FOG-',sprintf('%s',type),'dt.nav']; %% 后向滤波
    navdt1srotatepath = [cfg.outputfolder,'/FOG-',sprintf('%s',type),'dtrotate.nav']; %% 后向滤波
    navRTS = [cfg.outputfolder,'/FOG-LinearDistribute.nav'];
    % calc_radial_error(cfg.truthpath,navpath,navdt1spath,navdt1srotatepath,navEKFRotatepath,navdtmeapath)
    calc_radial_error(cfg.truthpath,navpath,navdt1spath,navdt1srotatepath,navEKFRotatepath,navRTS);
    % calc_radial_error(cfg.truthpath,navdtmeapath)
end

%%
if RTSIsOpen==1
    calc_error(navRTSpath,cfg.truthpath)
end
%%

calc_error(navpath,cfg.truthpath)
marker=[">","hexagram","pentagram"];
plot_trj(cfg.truthpath,navpath)
hold on
pos00 = [d2r(truth(1,3:4)),truth(1,5)]';
if beacontype=="3"||beacontype=="2"
    dxyz=pos2dxyz(rangedata(1:numnum,4:6),pos00);
    for i=1:numnum
        plot(dxyz(i,1),dxyz(i,2),marker(i),'DisplayName',sprintf('信标%d',i))
    end
elseif beacontype=="single" % 静止信标
    dxyz=pos2dxyz(rangedata(1,4:6),pos00);
    plot(dxyz(1),dxyz(2),marker(1),'DisplayName','信标')
elseif beacontype=="moving"
    dxyz=pos2dxyz(rangedata(:,4:6),pos00);
    plot(dxyz(:,1),dxyz(:,2),'DisplayName','信标')
    plot(dxyz(1,1),dxyz(1,2),'>','DisplayName','信标起点')
    plot(dxyz(end,1),dxyz(end,2),'<','DisplayName','信标终点')
end
%% 观察协方差矩阵
myfigurestartup(10,5,'prese')
subplot 121
plot(Pk(:,2))
subplot 122
plot(Pk(:,3))

%%
figure
plot(drange,'.')
figure;
plot(dxx,'.');
hold on
plot(dyy,'.')
%% 数据准备与预处理
% nn = importdata(nav60path);
nn = importdata(navpath);
t_start = max([rangedata(1,1), truth(1,2), nn(1,2)]);
t_end = min([rangedata(end,1), truth(end,2), nn(end,2)]);
time_common = (t_start:1:t_end)';
truth_sync = interp1(truth(:,2), truth(:,3:5), time_common);
dr_sync = interp1(nn(:,2), nn(:,3:5), time_common);
beacon_sync_deg = interp1(rangedata(:,1), [r2d(rangedata(:,4:5)), rangedata(:,6)], time_common);
period = 60;
plot_trajectory_analysis(time_common, truth_sync, dr_sync, beacon_sync_deg, period);

