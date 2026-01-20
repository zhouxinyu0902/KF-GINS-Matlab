clear
%% 定义全局参数
nnn=1;
seq=[1,2,3];
global rangstd
rangstd = 2;
global depstd
depstd = 0.5;
rng(1)
feedback = 1; % 是否反馈，不反馈则可以观察参数
glvs

%% 定义参数+加载过程配置
param = Param();
path='D:\Github\KF-GINS-Matlab\MEMS_RANGE\data_830_430_2';
cfg = ProcessConfig_MEMS_5(path);
type = {"single","moving","3","2"};
beacontype=type{1};

%% 三信标
% get3beacons(path)
%% 单信标
% poss=[5000,1000,0;
%     5000,4000,0;
%     5000,7000,0;
%     5000,9000,0;
%     -2000,1000,0;
%     -2000,4000,0;
%     -2000,7000,0;
%     -2000,9000,0;];
get1beacon(path,[-500,-500,0])
%% 移动信标
getmovingbeacon(path,'circle')
% close all
%% 加载数据
% imudata
imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);

% range data
id = 1;
rangedata1 = importdata(cfg.rangefile1path);
rangedata2 = importdata(cfg.rangefile2path);
rangedata3 = importdata(cfg.rangefile3path);
rangedatasingle = importdata(cfg.singlerangefilepath);
rangedatamoving = importdata(cfg.rangefilemovingpath);
range = {rangedata1,rangedata2,rangedata3,rangedatasingle,rangedatamoving};
% 构造距离信息

for i=1:length(range)
    range{i} = range{i}(id:id:end,:);
end

if beacontype=="3"||beacontype=="2"
    numnum=str2num(beacontype);
    rangedata = zeros(size(range{1}));
    for i=1:numnum
        rangedata(i:numnum:end,:)=range{seq(nnn,i)}(i:numnum:end,:);
    end
elseif beacontype=="single"
    rangedata = range{4};
elseif beacontype=="moving"
    rangedata = range{5};
end

% rangedata(1,:)=[];
rangedata(:,3) = rangedata(:,3) + normrnd(0,rangstd,size(rangedata(:,3)));
rangestarttime = rangedata(1, 1);
rangeendtime = rangedata(end, 1);

% height data
truth = importdata(cfg.truthpath);
height = truth(:,[2,5]);

height(:,2) = height(:,2) + normrnd(0,depstd,size(height(:,2)));
heistarttime = height(1, 1);
heitendtime = height(end, 1);
heightdata = height(id*100:id*100:end,:);

heightstarttime = heightdata(1, 1);
heightendtime = heightdata(end, 1);
%%
% output_file  = ['D:\Github\KF-GINS-main\dataset_exper','\rangedata', '.txt'];
% try
%     writematrix(rangedata, output_file, 'Delimiter', ' ');
%     fprintf('信息已成功写入到 %s\n', output_file);
% catch ME
%     error('错误：写入文件失败。错误信息：%s', ME.message);
% end
% output_file  = ['D:\Github\KF-GINS-main\dataset_exper','\heightdata', '.txt'];
% try
%     writematrix(heightdata, output_file, 'Delimiter', ' ');
%     fprintf('信息已成功写入到 %s\n', output_file);
% catch ME
%     error('错误：写入文件失败。错误信息：%s', ME.message);
% end
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

% imudata = imudata(1:2:end,:);
% heightdata = heightdata(1:2:end,:);

imudata = imudata(imudata(:,1) >= cfg.starttime, :);
imudata = imudata(imudata(:,1) <= cfg.endtime, :);
rangedata = rangedata(rangedata(:, 1) >= cfg.starttime, :);
rangedata = rangedata(rangedata(:, 1) <= cfg.endtime, :);
% rangedata(length(rangedata)+1,:)=zeros(1,6);

heightdata = heightdata(heightdata(:, 1) >= cfg.starttime, :);
heightdata = heightdata(heightdata(:, 1) <= cfg.endtime, :);
%% 绘制真实pva
% plot_result(cfg.truthpath)
% output_file  = [path,'\truth_600s', '.txt'];
% writematrix(truth(1:length(imudata),:), output_file, 'Delimiter', ' ');
% fprintf('信息已成功写入到 %s\n', output_file);
% plot_result(output_file)
%% 画信标图
% myfigurestartup(5,5,'prese')
% plot(truth(:,4),truth(:,3),'DisplayName','轨迹')
% hold on
% plot(truth(1,4),truth(1,3),'*','DisplayName','起点')
% marker=[">","hexagram","pentagram"];
% if beacontype=="3"
%     for i=1:3
%         plot(rangedata(i,5)/pi*180,rangedata(i,4)/pi*180,marker(i),'DisplayName',sprintf('信标%d',i))
%     end
% elseif beacontype=="single" % 静止信标
%     plot(rangedata(1,5)/pi*180,rangedata(1,4)/pi*180,marker(1),'DisplayName','信标')
% elseif beacontype=="moving"
%     plot(r2d(rangedata(:,5)),r2d(rangedata(:,4)),'DisplayName','信标')
% end
% xygo('经度/°','纬度/°')
% title('信标及轨迹')
% legend
%%
% for type = ["PureIns","EKF","AEKF"]
for type ="EKF"
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
        navpath = [cfg.outputfolder, '/MEMS-',sprintf('%s.nav',type)];
        disp("use RANGE data!");
        disp("Start GNSS/RANGE Processing!");
    else
        navpath =[cfg.outputfolder,'/MEMS-PureIns.nav'];
        disp("PURE INS!");
        disp("Start INS Processing!");
    end
    % max_interv = 42;
    navfp = fopen(navpath, 'wt');

    %
    % imuerrpath = [cfg.outputfolder, '/ImuError.txt'];
    % imuerrfp = fopen(imuerrpath, 'wt');
    %
    % stdpath = [cfg.outputfolder, '/NavSTD.txt'];
    % stdfp = fopen(stdpath, 'wt');
    %
    if feedback==0
        xkpath = [cfg.outputfolder, '/xk_range.txt'];
        xkfp = fopen(xkpath, 'wt');
    end
    %% 调试

    lastprecent = 0;
    %% 用于对比几类改进方法的参数
    ki=1;
    ki2=1;
    indexrecord=zeros(1,12);
    indexrecord2=zeros(1,12);
    z=zeros(11,6);
    indexrecord(1) = 1;
    indexrecord2(1) = 1;

    % === 前向滤波结果 (FFR) ===
    % 存储协方差矩阵 (3D 矩阵)

    P_F_store = zeros(2, 2, 463555);

    % === 后向滤波结果 (BFR) ===
    % 存储状态估计
    nav11 = zeros(3, 463555);
    nav112 = zeros(3, 463555);
    % 存储协方差矩阵 (3D 矩阵)
    P_B_store = zeros(2, 2, 463555);
    P_B_store2 = zeros(2, 2, 463555);
    %% initialization
    [kf, navstate] = myInitialize_15state(cfg);
    kf.Qc = kf.Qc * 5;
    kf.P0 = 10 * kf.P;
    laststate = navstate;
    pos0 = navstate.pos;
    vel0 = navstate.vel;
    % data index preprocess
    lastimu = imudata(1, :)';
    thisimu = imudata(1, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);
    dtheta=zeros(3,1);
    dv=zeros(3,1);
    rangeindex = 1;
    while rangedata(rangeindex, 1) < thisimu(1, 1)
        rangeindex = rangeindex + 1;
    end
    %% 正式滤波
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% MAIN PROCEDD PROCEDURE!
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for imuindex = 2:size(imudata, 1)
        % 1、set value of last state
        lastimu = thisimu;
        laststate = navstate;
        thisimu = imudata(imuindex, :)';

        imudt = thisimu(1, 1) - lastimu(1, 1);

        % thisimu(2:4) = thisimu(2:4) + 0.00001;
        % thisimu(5:7) = thisimu(5:7) + 0.0001;

        % 2、compensate IMU error
        % thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * navstate.gyrbias)./(ones(3, 1) + navstate.gyrscale);
        % thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * navstate.accbias)./(ones(3, 1) + navstate.accscale);

        % eb=0.2;
        % db=100;
        % cfg.initgyrbiasstd = [eb; eb; eb]; % [deg/h]
        % cfg.initaccbiasstd = [db; db; db]; % [mGal]
        % navstate.gyrbias = cfg.initgyrbiasstd * param.D2R / 3600;
        % navstate.accbias = cfg.initaccbiasstd * 1e-5;
        % thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * navstate.gyrbias);
        % thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * navstate.accbias);

        % 3、adjust range index
        while (rangeindex <= size(rangedata, 1) && rangedata(rangeindex, 1) < lastimu(1, 1))
            rangeindex = rangeindex + 1;
        end
        if (rangeindex > size(rangedata, 1))
            % rangeindex = rangeindex - 1;
            disp('range file END!');
            break;
        end
        % 4、determine whether gnss update is required
        if lastimu(1, 1) == rangedata(rangeindex, 1) && cfg.userange==1
            % 测量更新
            Rangedata = rangedata(rangeindex,:);
            depthdata = height(imuindex,1:2);

            % thisimu(5:7) = thisimu(5:7) + 0.1;

            thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * cfg.initgyrbiasstd);
            thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * cfg.initaccbiasstd);
            % 优化惯导数据
            b_g =  cfg.gyrbiasstd ;
            b_a =  cfg.accbiasstd ;
            % b_g =  navstate.gyrbias;
            % b_a =  navstate.accbias;
            % b_g = 0;
            % b_a = 0;
            param.sigma_g = cfg.gyrarw*40;
            param.sigma_a = cfg.accvrw*40;
            param.sigma_R = rangstd;

            state.Cb0_n = navstate.cbn;
            state.wnin = navstate.wnin;
            beacon_pos = Rangedata(4:6)';
            R_meas = Rangedata(3);


            % state.p0 = navstate.pos;
            % state.v0 = navstate.vel;
            % Hz = 1/imudt;
            % dtheta_tilde = thisimu(2:4);
            % dv_tilde = thisimu(5:7);
            % [dtheta_opt, dv_opt] = optimize_imu_incremental(dtheta_tilde, dv_tilde,state, beacon_pos, R_meas, param, Hz);
            % dtheta = dtheta_opt-dtheta_tilde;
            % dv = dv_opt-dv_tilde;

            state.v0 = vel0;
            state.p0 = pos0;
            Hz = 1;
            % dtheta_tilde = sum(imudata(imuindex-99:imuindex,2:4),1)';
            % dv_tilde = sum(imudata(imuindex-99:imuindex,5:7),1)';
            % [dtheta_opt, dv_opt] = optimize_imu_incremental_1s(dtheta_tilde, dv_tilde, state, beacon_pos, R_meas, param, Hz);
            % dtheta = (dtheta_opt-dtheta_tilde)/100;
            % dv = (dv_opt-dv_tilde)/100;

            

            thisimu(2:4, 1) = (thisimu(2:4, 1) + dtheta );
            thisimu(5:7, 1) = (thisimu(5:7, 1) + dv );
            
            

            % state.v0 = vel0;
            % state.p0 = pos0;
            % Hz = 1;
            % w_tilde=sum(imudata(imuindex-99:imuindex,2:4),1)/10000';
            % f_tilde=sum(imudata(imuindex-99:imuindex,5:7),1)/10000';
            % w_tilde=thisimu(2:4)/0.01;
            % f_tilde=thisimu(5:7)/0.01;
            % [w_opt, f_opt] = optimize_imu_rates(w_tilde, f_tilde, b_g, b_a, state, beacon_pos, R_meas, param, 1);
            % dtheta = (w_opt - w_tilde + cfg.initgyrbiasstd)*0.01;
            % dv = (f_opt - f_tilde + cfg.initgyrbiasstd)*0.01;

            dtheta_1(rangeindex,:) = dtheta;
            dv_1(rangeindex,:) = dv;
            
            % thisimu(2:4, 1) = (thisimu(2:4, 1) + dtheta );
            % thisimu(5:7, 1) = (thisimu(5:7, 1) + dv );

            if cfg.adap==1
                kf = myRangeUpdate_adap(navstate, Rangedata, depthdata, kf);
            else
                kf = myRangeUpdate(navstate, Rangedata, depthdata, kf);
            end
            if feedback==1
                [kf, navstate] = myErrorFeedback_range(kf, navstate);
                % [kf, navstate] = myErrorFeedback_range_posonly(kf, navstate);
            end
            rangeindex = rangeindex + 1;
            laststate = navstate;
            % 惯导推算
            imudt = thisimu(1, 1) - lastimu(1, 1);
            navstate = InsMech(laststate, lastimu, thisimu);
            kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
            pos0 = navstate.pos;
            vel0 = navstate.vel;

            % %% 反向推算
            % thisimu1 = imudata(imuindex-1, :)';
            % lastimu1 = imudata(imuindex, :)';
            % navstate_1 = navstate;
            % laststate_1 = navstate;
            %
            % kf1 = kf;
            % ki = ki+1;
            %
            % nav11(:,imuindex) = laststate_1.pos;
            % P_B_store(:,:,imuindex)=kf1.P(1:2,1:2);
            % indexrecord(ki) = imuindex;
            %
            % for ii=indexrecord(ki)-1:-1:indexrecord(ki-1)
            %     laststate_1 = InsMechBackward(navstate_1,lastimu1,thisimu1);
            %     laststate_1.pos(3) = height(ii,2);
            %     kf1 = myInsPropagate_15state(laststate_1, thisimu1, 0.01, kf1);
            %     nav11(:,ii) = laststate_1.pos;
            %     P_B_store(:,:,ii)=kf1.P(1:2,1:2);
            %     lastimu1 = thisimu1;
            %     thisimu1 = imudata(ii, :)';
            %     navstate_1 = laststate_1;
            % end
            %
            % %% 反向推算+滤波
            % thisimu2 = imudata(imuindex-1, :)';
            % lastimu2 = imudata(imuindex, :)';
            % navstate_2 = navstate;
            % laststate_2 = navstate;
            %
            % kf2 = kf;
            % ki2 = ki2+1;
            %
            % nav112(:,imuindex) = laststate_2.pos;
            % P_B_store2(:,:,imuindex)=kf2.P(1:2,1:2);
            % indexrecord2(ki2) = imuindex;
            % for ii = indexrecord2(ki2)-1:-1:indexrecord2(ki2-1)
            %     if ii==indexrecord2(ki2-1)
            %         laststate_2 = InsMechBackward(navstate_2,lastimu2,thisimu2);
            %         laststate_2.pos(3) = height(ii,2);
            %         % 反向滤波
            %         if rangeindex == 2
            %             Rangedata = zeros(1,6);
            %             Rangedata(4:6) = rangedata3(1,4:6);
            %             Rangedata(3) = caldot2dot(Rangedata(4:6),pos0');
            %         else
            %             Rangedata = rangedata(rangeindex-2,:);
            %         end
            %         depthdata = height(ii,1:2);
            %         kf2 = myRangeUpdate_adap(laststate_2, Rangedata, depthdata, kf2);
            %         [kf2, laststate_2] = myErrorFeedback_range(kf2, laststate_2);
            %
            %         % if rangeindex>2 & abs(kf2.Z(1))-abs(z(rangeindex-2,1))>40
            %         %     break;
            %         % else
            %         %     nav11(:,ii-1) = laststate_1.pos;
            %         % end
            %         z_back(ki2-1,1) = kf2.Z(1);% 残差记录
            %         z_back(ki2-1,2) = kf2.Znew;
            %         z_back(ki2-1,3) = kf2.alpha;
            %         z_back(ki2-1,4) = kf2.d_squared ;
            %         z_back(ki2-1,5) = kf2.chi2_threshold ;
            %         z_back(ki2-1,6) = kf2.is_anomaly ;
            %     else
            %         laststate_2 = InsMechBackward(navstate_2,lastimu2,thisimu2);
            %         laststate_2.pos(3) = height(ii,2);
            %         kf2 = myInsPropagate_15state(laststate_2, thisimu2, 0.01, kf2);
            %     end
            %     lastimu2 = thisimu2;
            %     thisimu2 = imudata(ii, :)';
            %     navstate_2 = laststate_2;
            %     nav112(:,ii) = laststate_2.pos;
            %     P_B_store2(:,:,ii) = kf2.P(1:2,1:2);
            % end

            % elseif (lastimu(1, 1) < rangedata(rangeindex, 1) && thisimu(1, 1) > rangedata(rangeindex, 1))&& cfg.userange==1
            %
            %     Rangedata = rangedata(rangeindex,:);
            %     depthdata = [height(imuindex,1),height(imuindex,2)];
            %
            %     % 插值imu
            %     [firstimu, secondimu] = interpolate(lastimu, thisimu, rangedata(rangeindex, 1));
            %     % 惯导推算
            %     imudt = firstimu(1, 1) - lastimu(1, 1);
            %     navstate = InsMech(laststate, lastimu, firstimu);
            %     kf = myInsPropagate_15state(navstate, firstimu, imudt, kf);
            %     % 测量更新
            %
            %     % if abs(navstate.time-122235-max_interv*60)<20||abs(navstate.time-122235-max_interv*2*60)<20||abs(navstate.time-122235-max_interv*3*60)<20
            %     %     tt = Rangedata(1);
            %     %     Rangedata1 = rangedata1(rangedata1(:,1)==tt,:) ;
            %     %     Rangedata2 = rangedata2(rangedata2(:,1)==tt,:) ;
            %     %     Rangedata3 = rangedata3(rangedata3(:,1)==tt,:) ;
            %     %     Rangedata1(3) = Rangedata1(3) + randn * rangstd ;
            %     %     Rangedata2(3) = Rangedata2(3) + randn * rangstd ;
            %     %     Rangedata3(3) = Rangedata3(3) + randn * rangstd ;
            %     %     kf = my3RangeUpdate(navstate,Rangedata1 ,Rangedata2, Rangedata3, depthdata, kf);
            %     %     fprintf('   3距离\n')
            %     % else
            %     if cfg.adap==1
            %         kf = myRangeUpdate_adap(navstate, Rangedata, depthdata, kf);
            %     else
            %         kf = myRangeUpdate(navstate, Rangedata, depthdata, kf);
            %     end
            %     % end
            %
            %     if feedback==1
            %         [kf, navstate] = myErrorFeedback_range(kf, navstate);
            %         % [kf, navstate] = myErrorFeedback_range_posonly(kf, navstate);
            %     end
            %
            %     % 优化惯导数据
            %     dtheta_tilde = thisimu(2:4);
            %     dv_tilde = thisimu(5:7);
            %
            %     params.sigma_g = cfg.gyrarw;
            %     params.sigma_a = cfg.accvrw;
            %     params.sigma_R = rangstd;
            %
            %     state.p0 = pos2dxyz(laststate.pos',pos0)';
            %     state.v0 = laststate.vel;
            %     state.Cb0_n = laststate.cbn;
            %
            %     beacon_pos = pos2dxyz(Rangedata(4:6),pos0)';
            %
            %     R_meas = Rangedata(3);
            %
            %     Hz = 1/imudt;
            %     [thisimu(2:4), thisimu(5:7)] = optimize_imu_incremental(dtheta_tilde, dv_tilde, b_g, b_a, state, beacon_pos, R_meas, params, Hz);
            %
            %     z(rangeindex,1) =kf.Z(1);% 残差记录
            %     z(rangeindex,2) =kf.Znew;
            %     z(rangeindex,3) =kf.alpha;
            %     z(rangeindex,4) =kf.d_squared ;
            %     z(rangeindex,5) =kf.chi2_threshold ;
            %     z(rangeindex,6) =kf.is_anomaly ;
            %     rangeindex = rangeindex + 1;
            %     laststate = navstate;
            %     lastimu = firstimu;
            %     % do propagation for second imu
            %     imudt = secondimu(1, 1) - lastimu(1, 1);
            %     navstate = InsMech(laststate, lastimu, secondimu);
            %     kf = myInsPropagate_15state(navstate, secondimu, imudt, kf);

            % %% 反向推算
            % thisimu1 = imudata(imuindex-1, :)';
            % lastimu1 = imudata(imuindex, :)';
            % navstate_1 = navstate;
            % laststate_1 = navstate;
            %
            % kf1 = kf;
            % ki = ki+1;
            %
            % nav11(:,imuindex) = laststate_1.pos;
            % P_B_store(:,:,imuindex)=kf1.P(1:2,1:2);
            % indexrecord(ki) = imuindex;
            %
            % for ii=indexrecord(ki)-1:-1:indexrecord(ki-1)
            %     laststate_1 = InsMechBackward(navstate_1,lastimu1,thisimu1);
            %     laststate_1.pos(3) = height(ii,2);
            %     kf1 = myInsPropagate_15state(laststate_1, thisimu1, 0.01, kf1);
            %     nav11(:,ii) = laststate_1.pos;
            %     P_B_store(:,:,ii)=kf1.P(1:2,1:2);
            %     lastimu1 = thisimu1;
            %     thisimu1 = imudata(ii, :)';
            %     navstate_1 = laststate_1;
            % end
            % %% 反向推算+滤波
            % thisimu2 = imudata(imuindex-1, :)';
            % lastimu2 = imudata(imuindex, :)';
            % navstate_2 = navstate;
            % laststate_2 = navstate;
            %
            % kf2 = kf;
            % ki2 = ki2+1;
            %
            % nav112(:,imuindex) = laststate_2.pos;
            % P_B_store2(:,:,imuindex)=kf2.P(1:2,1:2);
            % indexrecord2(ki2) = imuindex;
            % for ii = indexrecord2(ki2)-1:-1:indexrecord2(ki2-1)
            %     if ii==indexrecord2(ki2-1)
            %         laststate_2 = InsMechBackward(navstate_2,lastimu2,thisimu2);
            %         laststate_2.pos(3) = height(ii,2);
            %         % 反向滤波
            %         if rangeindex == 2
            %             Rangedata = zeros(1,6);
            %             Rangedata(4:6) = rangedata3(1,4:6);
            %             Rangedata(3) = caldot2dot(Rangedata(4:6),pos0');
            %         else
            %             Rangedata = rangedata(rangeindex-2,:);
            %         end
            %         depthdata = height(ii,1:2);
            %         kf2 = myRangeUpdate_adap(laststate_2, Rangedata, depthdata, kf2);
            %         [kf2, laststate_2] = myErrorFeedback_range(kf2, laststate_2);
            %
            %         % if rangeindex>2 & abs(kf2.Z(1))-abs(z(rangeindex-2,1))>40
            %         %     break;
            %         % else
            %         %     nav11(:,ii-1) = laststate_1.pos;
            %         % end
            %         z_back(ki2-1,1) = kf2.Z(1);% 残差记录
            %         z_back(ki2-1,2) = kf2.Znew;
            %         z_back(ki2-1,3) = kf2.alpha;
            %         z_back(ki2-1,4) = kf2.d_squared ;
            %         z_back(ki2-1,5) = kf2.chi2_threshold ;
            %         z_back(ki2-1,6) = kf2.is_anomaly ;
            %     else
            %         laststate_2 = InsMechBackward(navstate_2,lastimu2,thisimu2);
            %         laststate_2.pos(3) = height(ii,2);
            %         kf2 = myInsPropagate_15state(laststate_2, thisimu2, 0.01, kf2);
            %     end
            %     lastimu2 = thisimu2;
            %     thisimu2 = imudata(ii, :)';
            %     navstate_2 = laststate_2;
            %     nav112(:,ii) = laststate_2.pos;
            %     P_B_store2(:,:,ii) = kf2.P(1:2,1:2);
            % end
        else
            % 5、only do propagation
            % INS mechanization
            thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * cfg.initgyrbiasstd);
            thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * cfg.initaccbiasstd);

            thisimu(2:4, 1) = (thisimu(2:4, 1) + dtheta );
            thisimu(5:7, 1) = (thisimu(5:7, 1) + dv );

            navstate = InsMech(laststate, lastimu, thisimu);
            navstate.pos(3) = height(imuindex,2) ; % 天向位置约束
            % error propagation
            kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);


        end

        % 6、save data
        % xkk(imuindex-1,:)=[navstate.time;kf.x];
        % write navresult to file
        P_F_store(:,:,imuindex-1)=kf.P(1:2,1:2);
        nav = zeros(11, 1);
        nav(2, 1) = navstate.time;
        nav(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
        nav(6:8, 1) = navstate.vel;
        nav(9:11, 1) = navstate.att * param.R2D;
        fprintf(navfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);
        % 保存估计的状态值
        if feedback==0
            xk = zeros(16, 1);
            xk(1) = navstate.time;
            xk(2:16) = kf.x(1:15);
            fprintf(xkfp, '%12.6f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f\n', xk);
        end

        % write imu error, convert to common unit
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
        % if (imuindex / size(imudata, 1) - lastprecent > 0.20)
        %     disp("processing " + num2str(floor(imuindex * 100 / size(imudata, 1))) + " %!");
        %     lastprecent = imuindex / size(imudata, 1);
        % end
        % check whether gnss data is valid

    end
    % close file
    fclose(navfp);
    if feedback==0
        fclose(xkfp);
    end
    % fclose(imuerrfp);
    % fclose(stdfp);
    disp("range/INS Integration Processing Finished!");

    %% 误差绘图
    % calc_error(navpath,cfg.truthpath)
    % %%

    % if cfg.adap==1
    %     nav11(:,imuindex:end)=[];
    %     nav112(:,imuindex:end)=[];
    %     forward = importdata(navpath);%% 参考的位置
    %     nav000 = [d2r(truth(1:imuindex-1,3:4)),truth(1:imuindex-1,5)]';
    %     nav00 = [nav000(:,1),[d2r(forward(:,3:4)),forward(:,5)]'];
    %     nav11(:,1) = nav000(:,1);
    %     nav112(:,1) = nav000(:,1);
    %     nav_rotatesum1(:,1) = nav000(:,1);
    %     myfigurestartup(14,7,'paper')
    %     for ki=2: size(rangedata,1)+1
    %         % nav_rotate = rotateTrajectory(nav00(:,indexrecord(ki-1)+1:indexrecord(ki)-1),...
    %         %     nav00(:,indexrecord(ki)));
    %         trajectory = nav112(:,indexrecord(ki-1)+1:indexrecord(ki)-1);
    %         if ki==5
    %             newstartPoint = nav00(:,indexrecord(ki-1));
    %         else
    %             newstartPoint = nav112(:,indexrecord(ki-1));
    %         end
    %         if ki==4
    %             newEndPoint = nav00(:,indexrecord(ki));
    %         else
    %             newEndPoint = nav112(:,indexrecord(ki));
    %         end
    %         % if ki==2||ki==3
    %         %     nav_rotate = nav00(:,indexrecord(ki-1)+1:indexrecord(ki)-1);
    %         % else
    %         trajectory = flip(trajectory,2);
    %         [nav_rotate, ~, ~] = rotateAndScaleTrajectory(trajectory, newstartPoint);
    %         trajectory = flip(trajectory,2);
    %         nav_rotate = flip(nav_rotate,2);
    %         [nav_rotate, ~, ~] = rotateAndScaleTrajectory(nav_rotate, newEndPoint);
    %         % end
    %         nav_rotatesum1(:,indexrecord(ki-1):indexrecord(ki)) = [newstartPoint,nav_rotate,newEndPoint];
    %         subplot(4,4,ki-1)
    %         plot(trajectory(2,:), ...
    %             trajectory(1,:))
    %         hold on
    %         plot(newEndPoint(2),newEndPoint(1),'*')
    %         plot(nav_rotate(2,1:end),nav_rotate(1,1:end))
    %         plot(nav000(2,indexrecord(ki-1)+1:indexrecord(ki)-1), ...
    %             nav000(1,indexrecord(ki-1)+1:indexrecord(ki)-1))
    %         if ki==2
    %             legend('backforward','dot','rotate','truth')
    %         end
    %     end
    %     subplot(4,4,ki)
    %     plot(nav_rotatesum1(2,:),nav_rotatesum1(1,:))
    %     % 不加滤波
    %
    %     myfigurestartup(14,7,'paper')
    %     for ki=2:size(rangedata,1)+1
    %         % nav_rotate = rotateTrajectory(nav00(:,indexrecord(ki-1)+1:indexrecord(ki)-1),...
    %         %     nav00(:,indexrecord(ki)));
    %         trajectory = nav11(:,indexrecord(ki-1)+1:indexrecord(ki)-1);
    %         if ki==5
    %             newstartPoint = nav00(:,indexrecord(ki-1));
    %         else
    %             newstartPoint = nav11(:,indexrecord(ki-1));
    %         end
    %         if ki==4
    %             newEndPoint = nav00(:,indexrecord(ki));
    %         else
    %             newEndPoint = nav11(:,indexrecord(ki));
    %         end
    %         % if ki==2||ki==3
    %         %     nav_rotate = nav00(:,indexrecord(ki-1)+1:indexrecord(ki)-1);
    %         % else
    %         trajectory = flip(trajectory,2);
    %         [nav_rotate, ~, ~] = rotateAndScaleTrajectory(trajectory, newstartPoint);
    %         trajectory = flip(trajectory,2);
    %         nav_rotate = flip(nav_rotate,2);
    %         [nav_rotate, ~, ~] = rotateAndScaleTrajectory(nav_rotate, newEndPoint);
    %         % end
    %         nav_rotatesum2(:,indexrecord(ki-1):indexrecord(ki)) = [newstartPoint,nav_rotate,newEndPoint];
    %         subplot(4,4,ki-1)
    %         plot(trajectory(2,:), ...
    %             trajectory(1,:))
    %         hold on
    %         plot(newEndPoint(2),newEndPoint(1),'*')
    %         plot(nav_rotate(2,1:end),nav_rotate(1,1:end))
    %         plot(nav000(2,indexrecord(ki-1)+1:indexrecord(ki)-1), ...
    %             nav000(1,indexrecord(ki-1)+1:indexrecord(ki)-1))
    %         if ki==2
    %             legend('backforward','dot','rotate','truth')
    %         end
    %     end
    %     subplot(4,4,ki)
    %     plot(nav_rotatesum1(2,:),nav_rotatesum1(1,:))
    %
    %     % 旋转缩放后向结果
    %     nav_rotscale1=importdata(navpath);
    %     nav_rotscale1(:,3:4)=r2d(nav_rotatesum1(1:2,2:end)');
    %     output_file_rotateback  = [cfg.outputfolder,'/GTS-BRC-AEKF', '.txt'];
    %     try
    %         writematrix(nav_rotscale1, output_file_rotateback, 'Delimiter', ' ');
    %         fprintf('nav_rotscale信息已成功写入到 %s\n', output_file_rotateback);
    %     catch ME
    %         error('错误：写入文件失败。错误信息：%s', ME.message);
    %     end
    %     %
    %     % 旋转缩放后向结果（不加后向滤波-效果不好）
    %     nav_rotscale1=importdata(navpath);
    %     nav_rotscale1(:,3:4)=r2d(nav_rotatesum2(1:2,2:end)');
    %     output_file_rotateback  = [cfg.outputfolder,'/GTS-BRC-1-AEKF', '.txt'];
    %     try
    %         writematrix(nav_rotscale1, output_file_rotateback, 'Delimiter', ' ');
    %         fprintf('nav_rotscale信息已成功写入到 %s\n', output_file_rotateback);
    %     catch ME
    %         error('错误：写入文件失败。错误信息：%s', ME.message);
    %     end
    %     % 旋转缩放后向结果
    %     nav_back=importdata(navpath);
    %     nav_back(:,3:4)=r2d(nav112(1:2,2:end)');
    %     output_file_back  = [cfg.outputfolder,'/BRC-AEKF', '.txt'];
    %     try
    %         writematrix(nav_back, output_file_back, 'Delimiter', ' ');
    %         fprintf('nav_rotscale信息已成功写入到 %s\n', output_file_back);
    %     catch ME
    %         error('错误：写入文件失败。错误信息：%s', ME.message);
    %     end
    % end
    % close all
end
%%
calc_error(navpath,cfg.truthpath)
marker=[">","hexagram","pentagram"];
myfigurestartup(12,5,'prese')
plot_trj(cfg.truthpath,navpath)
hold on
pos0 = [d2r(truth(1,3:4)),truth(1,5)]';
if beacontype=="3"||beacontype=="2"
    dxyz=pos2dxyz(rangedata(1:numnum,4:6),pos0);
    for i=1:numnum
        plot(dxyz(i,1),dxyz(i,2),marker(i),'DisplayName',sprintf('信标%d',i))
    end
elseif beacontype=="single" % 静止信标
    dxyz=pos2dxyz(rangedata(1,4:6),pos0);
    plot(dxyz(1),dxyz(2),marker(1),'DisplayName','信标')
elseif beacontype=="moving"
    dxyz=pos2dxyz(rangedata(:,4:6),pos0);
    plot(dxyz(:,1),dxyz(:,2),'DisplayName','信标')
    plot(dxyz(1,1),dxyz(1,2),'>','DisplayName','信标起点')
    plot(dxyz(end,1),dxyz(end,2),'<','DisplayName','信标终点')
end
%%

%% 残差及自适应因子
% adap_factor_visualize(z,z_back)
% adap_factor_visualize_gui(z, z_back)
%% 几个固定信标位置对比
% chosen=[1,5,6,7,8];
% myfigurestartup(12,5,'prese')
% dxyz_truth = pos2dxyz([d2r(truth(:,3:4)),truth(:,5)],[d2r(truth(1,3:4)),truth(1,5)]');
% plot(dxyz_truth(:,1),dxyz_truth(:,2),'DisplayName','轨迹')
% hold on
% plot(dxyz_truth(1,1),dxyz_truth(1,2),'.','DisplayName','起点')
% plot(dxyz_truth(end,1),dxyz_truth(end,2),'.','DisplayName','终点')
% for i=1:5
%     plot(poss(chosen(i),1),poss(chosen(i),2),"hexagram",'DisplayName',sprintf('信标%d',chosen(i)))
% end
% axis equal
% legend()
%% 几个固定信标的误差对比
% for i=1:5
%     navv{i} = [cfg.outputfolder, '/MEMS-',num2str(chosen(i)),sprintf('%s.nav',type)];
% end
% myfigurestartup(12,5,'prese')
% calc_radial_error(cfg.truthpath,navv{1},navv{2},navv{3},navv{4},navv{5})
% plot_trj(cfg.truthpath,navv{1},navv{2},navv{3},navv{4},navv{5})
%%
path=[cfg.outputfolder,'\MEMS-EKF-none.nav'];
path1=[cfg.outputfolder,'\MEMS-EKF-opti-40.nav'];
path2=[cfg.outputfolder,'\MEMS-EKF-opti-70.nav'];
path3=[cfg.outputfolder,'\MEMS-EKF-opti-75.nav'];
path4=[cfg.outputfolder,'\MEMS-EKF-opti-20.nav'];
myfigurestartup(12,5,'prese')
% calc_radial_error(cfg.truthpath,path,path4,path1,path2,path3)
plot_trj(cfg.truthpath,path,path4,path1,path2,path3)
%% 针对600s的时候
path=[cfg.outputfolder,'\MEMS-EKF-600s-none.nav'];
% path1=[cfg.outputfolder,'\MEMS-EKF-600s-20.nav'];
% path2=[cfg.outputfolder,'\MEMS-EKF-600s-40.nav'];
path80=[cfg.outputfolder,'\MEMS-EKF-600s-80.nav'];
% path4=[cfg.outputfolder,'\MEMS-EKF-600s-100.nav'];
path120=[cfg.outputfolder,'\MEMS-EKF-600s-120.nav'];
path150=[cfg.outputfolder,'\MEMS-EKF-600s-150.nav'];
path160=[cfg.outputfolder,'\MEMS-EKF-600s-160.nav'];
path170=[cfg.outputfolder,'\MEMS-EKF-600s-170.nav'];
path180=[cfg.outputfolder,'\MEMS-EKF-600s-180.nav'];
myfigurestartup(12,5,'prese')
calc_radial_error(cfg.truthpath,path,path80,path160,path180)
% plot_trj(cfg.truthpath,path,path1,path2,path3)
%%
path=[cfg.outputfolder,'\MEMS-EKF-moving-none.nav'];
path1=[cfg.outputfolder,'\MEMS-EKF-moving-1.nav'];
path5=[cfg.outputfolder,'\MEMS-EKF-moving-5.nav'];
path10=[cfg.outputfolder,'\MEMS-EKF-moving-10.nav'];
path20=[cfg.outputfolder,'\MEMS-EKF-moving-20.nav'];
path30=[cfg.outputfolder,'\MEMS-EKF-moving-30.nav'];
path40=[cfg.outputfolder,'\MEMS-EKF-moving-40.nav'];
myfigurestartup(12,5,'prese')
calc_radial_error(cfg.truthpath,path,path20,path30,path40)
%% 估计误差对比
if feedback==0
    plot_xk(xkpath,navpath,cfg.truthpath)
end

%% 辅助函数
function get3beacons(path)
glvs
truth = importdata([path,'/truth.nav']);
num=floor(1/(truth(2,2)-truth(1,2)));
GNSS_1s = truth(num:num:end,2:5);% 时间间隔
orgin0 = d2r(GNSS_1s(1,2:4));
% 原始等边三角形坐标（单位：米）
dxyz_original = [0, -5*sqrt(3), 0;
    -10, 5*sqrt(3), 0;
    10, 5*sqrt(3), 0;] * 1000;
% 定义旋转角度（15度）
theta_deg = 90;
theta_rad = deg2rad(theta_deg);
% 创建绕Z轴的旋转矩阵
R = [cos(theta_rad), -sin(theta_rad), 0;
    sin(theta_rad), cos(theta_rad),  0;
    0,              0,              1];
% 对每个点应用旋转
dxyz_rotated = (R * dxyz_original')'; % 转置以便矩阵乘法
% 使用旋转后的坐标
dxyz = dxyz_rotated;
% 分开获取信标和轨迹原点
rrm = dxyz2pos(dxyz, orgin0');
ddm = r2d(rrm(:, 1:2));
beaconxyz = dxyz(1:3, :);
beaconrrm = rrm(1:3, :);
beaconddm = ddm(1:3, :);

trj=GNSS_1s(:, 2:4);
trj(:,1:2)=d2r(trj(:,1:2));
trajectory_xyz = pos2dxyz(trj, orgin0');
trajectory_ddm=GNSS_1s(:, 2:4);

% 绘图
plot_trajectory_and_beacons(trajectory_xyz/1000, beaconxyz, beaconddm, trajectory_ddm)
trajectory_x = trajectory_xyz(:, 1);
trajectory_y = trajectory_xyz(:, 2);
for i=1:3
    % 获取信标的坐标 (东向，北向，天向)
    beacon1_x = beaconxyz(i, 1);
    beacon1_y = beaconxyz(i, 2);
    % 计算每个轨迹点到第一个信标的2D距离
    distances(:,i) = sqrt((trajectory_x - beacon1_x).^2 + ...
        (trajectory_y - beacon1_y).^2);
    % % 创建新的图窗来绘制距离曲线
    % figure;
    % plot(GNSS_1s(:,1), distances(:,i), 'LineWidth', 1.5); % 洋红色实线
    % xlabel('时间 (s) ');
    % ylabel('距离 (km)');
    % title('轨迹点到信标的距离');
    % grid on;
end
%
beacon1=ones(length(distances),3)*diag(beaconrrm(1,:));
beacon2=ones(length(distances),3)*diag(beaconrrm(2,:));
beacon3=ones(length(distances),3)*diag(beaconrrm(3,:));
range1=[GNSS_1s(:,1),distances(:,1),distances(:,1),beacon1];
range2=[GNSS_1s(:,1),distances(:,2),distances(:,2),beacon2];
range3=[GNSS_1s(:,1),distances(:,3),distances(:,3),beacon3];

output_file=[path,'/range1.txt'];
try
    writematrix(range1, output_file, 'Delimiter', ' ');
    fprintf('信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
output_file=[path,'/range2.txt'];
try
    writematrix(range2, output_file, 'Delimiter', ' ');
    fprintf('信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
output_file=[path,'/range3.txt'];
try
    writematrix(range3, output_file, 'Delimiter', ' ');
    fprintf('信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end

end

function get1beacon(path,dxyz)
glvs
truth = importdata([path,'/truth.nav']);
num=floor(1/(truth(2,2)-truth(1,2)));
GNSS_1s = truth(num:num:end,2:5);% 时间间隔
orgin0 = d2r(GNSS_1s(1,2:4));

% 分开获取信标和轨迹原点
rrm = dxyz2pos(dxyz, orgin0');
ddm = r2d(rrm(1:2));
beaconxyz = dxyz;
beaconrrm = rrm;
beaconddm = ddm;
trj = GNSS_1s(:, 2:4);
trj(:,1:2) = d2r(trj(:,1:2));
trajectory_xyz = pos2dxyz(trj, orgin0');
trajectory_ddm=GNSS_1s(:, 2:4);
% 绘图
% plot_trajectory_and_beacons(trajectory_xyz/1000, beaconxyz, beaconddm, trajectory_ddm)
trajectory_x = trajectory_xyz(:, 1);
trajectory_y = trajectory_xyz(:, 2);
% 获取信标的坐标 (东向，北向，天向)
beacon1_x = beaconxyz(1);
beacon1_y = beaconxyz(2);
% 计算每个轨迹点到第一个信标的2D距离
distances(:) = sqrt((trajectory_x - beacon1_x).^2 + ...
    (trajectory_y - beacon1_y).^2);
% % 创建新的图窗来绘制距离曲线
% figure;
% plot(GNSS_1s(:,1), distances(:,i), 'LineWidth', 1.5); % 洋红色实线
% xlabel('时间 (s) ');
% ylabel('距离 (km)');
% title('轨迹点到信标的距离');
% grid on;

%
beacon1=ones(length(distances),3)*diag(beaconrrm);
range1=[GNSS_1s(:,1),distances',distances',beacon1];

output_file=[path,'/single_range.txt'];
try
    writematrix(range1, output_file, 'Delimiter', ' ');
    fprintf('信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end

end

% function getmovingbeacon(path)
% glvs
% truth = importdata([path,'/truth.nav']);
% num=floor(1/(truth(2,2)-truth(1,2)));
% GNSS_1s = truth(num:num:end,2:5);% 时间间隔
%
% orgin0 = d2r(GNSS_1s(1,2:4));
% dxyz_orgin0= [-5, 1, 0] * 1000;
% orgin0_bea = dxyz2pos(dxyz_orgin0, orgin0');
%
% refavp=pvaNED2ENU(truth);
% ts = 1;
% avp0 = [[0;0;d2r(140)]; [0;0;0]; orgin0_bea'];
% xxx = [];
% seg = trjsegment(xxx, 'init',         0);
% seg = trjsegment(seg, 'accelerate',   10, xxx, 0.3);
% seg = trjsegment(seg, 'uniform',    length(refavp)/100/4);
% seg = trjsegment(seg, 'turnright', 90, 1);
% seg = trjsegment(seg, 'uniform',      length(refavp)/100/4);
% seg = trjsegment(seg, 'turnright', 90, 1);
% seg = trjsegment(seg, 'uniform',      length(refavp)/100/4);
% seg = trjsegment(seg, 'turnright', 90, 1);
% seg = trjsegment(seg, 'uniform',      length(refavp)/100/4);
% trjbea = trjsimu(avp0, seg.wat, ts, 1); % 只需要位置和姿态信息就可以
%
% % avp0 = [[0;0;d2r(180)]; [0;0;0]; orgin0_bea'];
% % xxx = [];
% % seg = trjsegment(xxx, 'init',         0);
% % seg = trjsegment(seg, 'accelerate',   10, xxx, 0.3);
% % seg = trjsegment(seg, 'uniform',    length(refavp)/100);
% % trjbea = trjsimu(avp0, seg.wat, ts, 1); % 只需要位置和姿态信息就可以
%
%
% rrm = trjbea.avp(:,7:9);
% ddm = r2d(rrm(:,1:2));
% beaconxyz = pos2dxyz(rrm,orgin0');
% beaconrrm = rrm;
% beaconddm = ddm;
%
% trj=GNSS_1s(:, 2:4);
% trj(:,1:2)=d2r(trj(:,1:2));
% trajectory_xyz = pos2dxyz(trj, orgin0');
% trajectory_ddm=GNSS_1s(:, 2:4);
% % 绘图
% plot_trajectory_and_movingbeacons(trajectory_xyz/1000, beaconxyz, beaconddm, trajectory_ddm)
% trajectory_x = trajectory_xyz(:, 1);
% trajectory_y = trajectory_xyz(:, 2);
% % 获取信标的坐标 (东向，北向，天向)
% beacon_x = beaconxyz(1:length(trajectory_xyz),1);
% beacon_y = beaconxyz(1:length(trajectory_xyz),2);
% % 计算每个轨迹点到第一个信标的2D距离
% distances(:) = sqrt((trajectory_x - beacon_x).^2 + ...
%     (trajectory_y - beacon_y).^2);
% % % 创建新的图窗来绘制距离曲线
% % figure;
% % plot(GNSS_1s(:,1), distances(:,i), 'LineWidth', 1.5); % 洋红色实线
% % xlabel('时间 (s) ');
% % ylabel('距离 (km)');
% % title('轨迹点到信标的距离');
% % grid on;
%
% range1=[GNSS_1s(:,1),distances',distances',beaconrrm(1:length(trajectory_xyz),:)];
%
% output_file=[path,'/range_moving.txt'];
% try
%     writematrix(range1, output_file, 'Delimiter', ' ');
%     fprintf('信息已成功写入到 %s\n', output_file);
% catch ME
%     error('错误：写入文件失败。错误信息：%s', ME.message);
% end
%
% end

function getmovingbeacon(path,type)
glvs
truth = importdata([path,'/truth.nav']);
num=floor(1/(truth(2,2)-truth(1,2)));
GNSS_1s = truth(num:num:end,2:5);% 时间间隔


orgin0 =[d2r(GNSS_1s(1,2:3)),GNSS_1s(1,4) ] ;
dxyz_orgin0= [-500, -500, 0];
orgin0_bea = dxyz2pos(dxyz_orgin0, orgin0');
ts = 1;

if type=="circle"
    % 圆形轨迹
    R=250;
    V=0.5;
    N=1;
    trjbea.avp = circle(orgin0_bea',R,V,N,pi/2);
elseif type=="follow"
    N = length(GNSS_1s);
    mid = floor(N/2);
    base_offset = [-500, -500, 0];
    k = 0.8; % 轨迹缩放比例：0.8 表示 trjbea 的运动范围只有 GNSS_1s 的 80%
    start_pos_rad = [d2r(GNSS_1s(1,2:3)), GNSS_1s(1,4)]';
    for ii = 1:N
        % 1. 计算当前 GNSS 点相对于起点的位移 (Delta NEU)
        delta_neu = pos2dxyz([d2r(GNSS_1s(ii,2:3)), GNSS_1s(ii,4)], start_pos_rad);
        % 2. 缩放位移 (实现“范围小一点，速度慢一点”)
        scaled_move = delta_neu * k;
        % 3. 计算“掉队”逻辑 (Offset)
        if ii <= mid
            current_drift = (ii-1) * [0.5, 0.5, 0];
        else
            max_drift = (mid-1) * [0.5, 0.5, 0];
            current_drift = max_drift + (ii-mid) * [-0.5, -0.5, 0];
        end
        % current_drift=[0,-500,0];
        % 4. 最终位置 = 起点 + 缩放后的位移 + 基础偏差 + 掉队偏差
        final_offset = scaled_move + base_offset + current_drift;

        trjbea.avp(ii,7:9) = dxyz2pos(final_offset, start_pos_rad);
    end

    % % 准备原始数据
    % raw_data = [d2r(GNSS_1s(500:1000, 2:3)), GNSS_1s(500:1000, 4)];
    %
    % % 定义原始索引 (1 到 501)
    % old_index = 1:size(raw_data, 1);
    %
    % % 定义新的查询索引 (从 1 到 501 均匀取 N 个点)
    % % 这一步完成了“时间”上的重采样
    % new_index = linspace(1, size(raw_data, 1), N);
    %
    % % 执行插值
    % trjbea.avp(1:N, 7:9) = interp1(old_index, raw_data, new_index, 'linear');
    myfigurestartup(5,5,'prese')
    plot(d2r(GNSS_1s(:,3)),d2r(GNSS_1s(:,2)))
    hold on
    plot(trjbea.avp(:,8),trjbea.avp(:,7))

else
    if type=="trjsquare"
        % 方形轨迹
        avp0 = [[0;0;d2r(90)]; [0;0;0]; orgin0_bea'];
        xxx = [];
        seg = trjsegment(xxx, 'init',         0);
        seg = trjsegment(seg, 'accelerate',   10, xxx, 0.1);
        seg = trjsegment(seg, 'uniform',    length(truth)/num/8);
        seg = trjsegment(seg, 'turnleft', 90, 1);
        seg = trjsegment(seg, 'uniform',    length(truth)/num/5);
        seg = trjsegment(seg, 'turnright', 90, 1);
        seg = trjsegment(seg, 'uniform',      length(truth)/num/5);
        seg = trjsegment(seg, 'turnright', 90, 1);
        seg = trjsegment(seg, 'uniform',      length(truth)/num/5);
        seg = trjsegment(seg, 'turnright', 90, 1);
        seg = trjsegment(seg, 'uniform',      length(truth)/num/5);
    elseif type=="trjlike"
        avp0 = [[0;0;d2r(-45)]; [0;0;0]; orgin0_bea'];
        n=3;
        xxx = [];
        seg = trjsegment(xxx, 'init',         0);
        seg = trjsegment(seg, 'uniform',      20); % 保持原来的状态不变
        seg = trjsegment(seg, 'accelerate',   10, xxx, 0.1); % 加速
        seg = trjsegment(seg, 'uniform',      500);
        seg = trjsegment(seg, 'turnright', 27, 5);
        seg = trjsegment(seg, 'uniform',      1000);
        seg = trjsegment(seg, 'turnleft', 27, 5);
        seg = trjsegment(seg, 'uniform',      1000);
        seg = trjsegment(seg, 'turnright', 27, 5);
        seg = trjsegment(seg, 'uniform',      500);
        seg = trjsegment(seg, 'turnleft', 27, 5);
        seg = trjsegment(seg, 'uniform',     500);

    elseif type=="square"
        % 方形轨迹
        avp0 = [[0;0;d2r(140)]; [0;0;0]; orgin0_bea'];
        xxx = [];
        seg = trjsegment(xxx, 'init',         0);
        seg = trjsegment(seg, 'accelerate',   10, xxx, 0.1);
        seg = trjsegment(seg, 'uniform',    length(truth)/num/4);
        seg = trjsegment(seg, 'turnright', 90, 1);
        seg = trjsegment(seg, 'uniform',      length(truth)/num/4);
        seg = trjsegment(seg, 'turnright', 90, 1);
        seg = trjsegment(seg, 'uniform',      length(truth)/num/4);
        seg = trjsegment(seg, 'turnright', 90, 1);
        seg = trjsegment(seg, 'uniform',      length(truth)/num/4);
    elseif type=="line"
        % 直线轨迹
        avp0 = [[0;0;d2r(180)]; [0;0;0]; orgin0_bea'];
        xxx = [];
        seg = trjsegment(xxx, 'init',         0);
        seg = trjsegment(seg, 'accelerate',   10, xxx, 0.3);
        seg = trjsegment(seg, 'uniform',    length(truth)/num);
    end
    trjbea= trjsimu(avp0, seg.wat, ts, 1);
end
% insplot(trjbea.avp)
rrm = trjbea.avp(:,7:9);
ddm = r2d(rrm(:,1:2));
beaconxyz = pos2dxyz(rrm,orgin0');
beaconrrm = rrm;
beaconddm = ddm;

trj=GNSS_1s(:, 2:4);
trj(:,1:2)=d2r(trj(:,1:2));
trajectory_xyz = pos2dxyz(trj, orgin0');
trajectory_ddm=GNSS_1s(:, 2:4);
% 绘图
% figure
% plot_trajectory_and_movingbeacons(trajectory_xyz/1000, beaconxyz, beaconddm, trajectory_ddm)
trajectory_x = trajectory_xyz(:, 1);
trajectory_y = trajectory_xyz(:, 2);
% 获取信标的坐标 (东向，北向，天向)
beacon_x = beaconxyz(1:length(trajectory_xyz),1);
beacon_y = beaconxyz(1:length(trajectory_xyz),2);
% 计算每个轨迹点到第一个信标的2D距离
distances(:) = sqrt((trajectory_x - beacon_x).^2 + ...
    (trajectory_y - beacon_y).^2);
% % 创建新的图窗来绘制距离曲线
% figure;
% plot(GNSS_1s(:,1), distances(:,i), 'LineWidth', 1.5); % 洋红色实线
% xlabel('时间 (s) ');
% ylabel('距离 (km)');
% title('轨迹点到信标的距离');
% grid on;

range1=[GNSS_1s(:,1),distances',distances',beaconrrm(1:length(trajectory_xyz),:)];

output_file=sprintf('%s/range_moving.txt',path);
try
    writematrix(range1, output_file, 'Delimiter', ' ');
    fprintf('信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end

end
function avp_circle = circle(pos0,R,V,N,deg)
%CIRCLE_SEG 根据输入的半径R和速度V，以及圈数N，以及参考初始纬经度pos0，产生圆周轨迹
T = 2*pi*R/V;omega = V/R;
t = 0:1:N*T;l = length(t);
theta = deg-omega * t;
x = R * cos(theta);
y = R * sin(theta);
vx = -V * sin(theta);
vy = V * cos(theta);
yaw = mod(theta,2*pi);
yaw(yaw>pi) = yaw(yaw>pi)-2*pi;
phi=[zeros(l,2),yaw'];
pos = dxyz2pos([x',y',zeros(l,1)], pos0);
avp_circle=[phi,vx',vy',zeros(l,1),pos,t'];
end