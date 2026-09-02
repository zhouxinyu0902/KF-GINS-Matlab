clear
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
paths = setup_mems_range_navigation();
%% 定义全局参数
nnn=1;
seq=[1,2,3];
global rangstd 
rangstd = 2;
global depstd
depstd = 0.5;
global azimustd
azimustd = 0.2;
rng(1)
glvs
%% 定义参数+加载过程配置
param = Param();
path = paths.dataset_830_430_2;
cfg = ProcessConfig_MEMS_exper(path);
type = {"single","moving","3","2"};
beacontype = type{2};

isoptim = 1; % 是否
expand = 1;

backwardIsOpen_1s = 0;

feedback = 1; % 是否反馈，不反馈则可以观察参数

smoothIsOpen = 0; % 是否平滑
dot = 6000;

RTSIsOpen = 0; % 是否RTS平滑
%% 单信标
% get1beacon(path,[-2000,4000,0])
%% 移动信标
getmovingbeacon(path,'plan')
%% 加载数据
imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);
% 构造距离信息
id = 1;
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
elseif beacontype=="single"
    rangedata = range{4};
elseif beacontype=="moving"
    rangedata = range{5};
end

rangedata(:,3) = rangedata(:,3) + normrnd(0,rangstd,size(rangedata(:,3)));

para.sigma_min = 0.1;
para.sigma_max = 0.5;
para.power_fac = 2;
para.jitter = 0.2;
rangedata(:,7) = add_azimuth_noise_irregular(rangedata(:,7), para);

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

imudata = imudata(imudata(:,1) >= cfg.starttime, :);
imudata = imudata(imudata(:,1) <= cfg.endtime, :);
rangedata = rangedata(rangedata(:, 1) >= cfg.starttime, :);
rangedata = rangedata(rangedata(:, 1) <= cfg.endtime, :);
height = height(height(:, 1) >= cfg.starttime, :);
height = height(height(:, 1) <= cfg.endtime, :);

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
    else
        navpath =[cfg.outputfolder,'/MEMS-PureIns.nav'];
    end
    navfp = fopen(navpath, 'wt');

    if backwardIsOpen == 1
        navdtpath = [cfg.outputfolder,'/MEMS-EKFdt.nav']; %% 后向推算
        navbfpath = [cfg.outputfolder,'/MEMS-EKFdt-bf.nav']; %% 前后向滤波按权重分配
        navbbpath = [cfg.outputfolder,'/MEMS-EKFdt-bb.nav']; %% 后向滤波

        navdtfp = fopen(navdtpath,'wt');
        navbffp = fopen(navbfpath,'wt');
        navbbfp = fopen(navbbpath,'wt');
    end
    if backwardIsOpen_1s == 1
        navdt1spath = [cfg.outputfolder,'/MEMS-EKFdt1s.nav']; %% 后向滤波
        navdt1sfp = fopen(navdt1spath,'wt');
        navdt1srotatepath = [cfg.outputfolder,'/MEMS-EKFdtrotate1s.nav']; %% 后向滤波
        navdt1srotatefp = fopen(navdt1srotatepath,'wt');
    end
    navRTSpath = [cfg.outputfolder,'/MEMS-EKFRTS.nav'];
    navRTSfp = fopen(navRTSpath,'wt');
    % 其余数据保存
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

    %% initialization
    [kf, navstate] = myInitialize_15state(cfg);
    % kf.Qc = kf.Qc * 5;
    % kf.P0 = 10 * kf.P;

    kf.Qc = kf.Qc * 100;
    kf.P0 = kf.P * 100;
    % kf.P0(1,1)=1e-12;
    % kf.P0(2,2)=1e-12;
    laststate = navstate;
    pos0 = navstate.pos;
    vel0 = navstate.vel;
    % data index preprocess
    lastimu = imudata(1, :)';
    thisimu = imudata(1, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);
    dtheta = zeros(3,1);
    dv = zeros(3,1);
    rangeindex = 1;
    while rangedata(rangeindex, 1) < thisimu(1, 1)
        rangeindex = rangeindex + 1;
    end
    %% 正式滤波
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% MAIN PROCEDD PROCEDURE!
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for imuindex = 2:size(imudata, 1)
        %% 1、set value of last state
        lastimu = thisimu;
        laststate = navstate;
        thisimu = imudata(imuindex, :)';
        imudt = thisimu(1, 1) - lastimu(1, 1);

        thisimu(2:4, 1) = thisimu(2:4, 1) - imudt * cfg.initgyrbiasstd;
        thisimu(5:7, 1) = thisimu(5:7, 1) - imudt * cfg.initaccbiasstd;
        %% 2、adjust range index
        while (rangeindex <= size(rangedata, 1) && rangedata(rangeindex, 1) < lastimu(1, 1))
            rangeindex = rangeindex + 1;
        end
        if (rangeindex > size(rangedata, 1))
            % rangeindex = rangeindex - 1;
            disp('range file END!');
            break;
        end
        %% 3、determine whether gnss update is required
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
                kf = myRangeUpdate(navstate, Rangedata, depthdata, kf);
                % kf = myAziRangePosUpdate(navstate, Rangedata, depthdata, kf);
                % kf = myRange_aziUpdate(navstate, Rangedata, depthdata, kf);
            end
            rangeindex = rangeindex + 1;
            
            if feedback == 1
                [kf, navstate] = myErrorFeedback_range(kf, navstate);
                % [kf, navstate] = myErrorFeedback_range_posonly(kf, navstate);
            end
            laststate = navstate;

            % 惯导推算
            imudt = thisimu(1, 1) - lastimu(1, 1);
            navstate = InsMech(laststate, lastimu, thisimu);
            kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);

            pos0 = navstate.pos;
            vel0 = navstate.vel;

            IsRangeUpdate = 1;

            if backwardIsOpen_1s == 1
                backward_1s;
            end


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
            kf = myInsPropagate_15state(navstate, firstimu, imudt, kf);

            % 测量更新
            if cfg.adap==1
                kf = myRangeUpdate_adap(navstate, Rangedata, depthdata, kf);
            else
                kf = myRangeUpdate(navstate, Rangedata, depthdata, kf);
                % kf = myAziRangePosUpdate(navstate, Rangedata, depthdata, kf);
                % kf = myRange_aziUpdate(navstate, Rangedata, depthdata, kf);
            end
            %%
            if feedback==1
                [kf, navstate] = myErrorFeedback_range(kf, navstate);
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

            IsRangeUpdate=1;

            if backwardIsOpen_1s == 1
                backward_1s;
            end


            % % 残差记录
            % z(rangeindex,1) =kf.Z(1);
            % z(rangeindex,2) =kf.Znew;
            % z(rangeindex,3) =kf.alpha;
            % z(rangeindex,4) =kf.d_squared ;
            % z(rangeindex,5) =kf.chi2_threshold ;
            % z(rangeindex,6) =kf.is_anomaly ;
        else
            % 5、only do propagation
            % INS mechanization
            
            thisimu(2:4, 1) = (thisimu(2:4, 1) + dtheta );
            thisimu(5:7, 1) = (thisimu(5:7, 1) + dv );

            navstate = InsMech(laststate, lastimu, thisimu);
            navstate.pos(3) = height(imuindex,2) ; % 天向位置约束

            % error propagation
            kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
            IsRangeUpdate = 0;
        end

        %% 6、save data
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
        if (imuindex / size(imudata, 1) - lastprecent > 0.20)
            disp("processing " + num2str(floor(imuindex * 100 / size(imudata, 1))) + " %!");
            lastprecent = imuindex / size(imudata, 1);
        end
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

end
%% -----smooth 平滑------
if smoothIsOpen==1
    smoothpath=[cfg.outputfolder,'/smooth.txt'];
    writematrix(navforwardsmooth,smoothpath, 'Delimiter', ' ');
    % calc_error(smoothpath,cfg.truthpath)
    calc_radial_error(cfg.truthpath,smoothpath,navpath)
end
% myfigurestartup(12,5,'prese')
% plot_trj(cfg.truthpath,navpath,smoothpath)
%% -----RTS 平滑--------
if RTSIsOpen==1
    calc_error(navRTSpath,cfg.truthpath)
end
%% ------每个测距周期反向推算----
if backwardIsOpen_1s == 1
    % calc_error(navdt1spath,cfg.truthpath)
    calc_radial_error(cfg.truthpath,navdt1spath,navpath)
    % plot_trj(cfg.truthpath,navdt1spath,navdt1srotatepath,navpath)
end
%% -----反向推算--------
if backwardIsOpen == 1
    calc_radial_error(cfg.truthpath,navdtpath,navbfpath,navpath)
    plot_trj(cfg.truthpath,navdtpath,navbfpath,navpath)
end
%%
% azi_position_cmp;
% AUAX = [path,'/input','/pva_file.txt'];
% calc_error(navpath,AUAX)
% 
% calc_error(cfg.truthpath,AUAX)
%%
% figure
% plot(drange)
% figure
% plot(xk(1,2:end)*glv.Re)
% hold on
% plot(xk(1,2:end)*glv.Re*cos(36/180*pi))
calc_radial_error(cfg.truthpath,navpath)
ylim([0 60])
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
%%
P_F_store(:,:,imuindex-1:end)=[];
myfigurestartup(12,5,'prese')
subplot 121
plot(squeeze(P_F_store(1,1,:)))
subplot 122
plot(squeeze(P_F_store(2,2,:)))
%% 数据准备与预处理
% nn = importdata(nav60path);
nn = importdata(navpath);
% 假设各原始数据的时间列：
% rangedata: 第一列为时间
% truth: 第二列为时间
% nn: 第二列为时间

% 1. 确定共同的时间范围（交集）
t_start = max([rangedata(1,1), truth(1,2), nn(1,2)]);
t_end = min([rangedata(end,1), truth(end,2), nn(end,2)]);

% 2. 创建统一的时间轴 (以1s为步长)
time_common = (t_start:1:t_end)';

% 3. 将所有轨迹插值/同步到统一时间轴上
% 注意：truth 和 nn 的位置在 3:5 列
truth_sync = interp1(truth(:,2), truth(:,3:5), time_common);
dr_sync = interp1(nn(:,2), nn(:,3:5), time_common);
% beacon 的位置在 rangedata 的 4:6 列
beacon_sync_deg = interp1(rangedata(:,1), [r2d(rangedata(:,4:5)), rangedata(:,6)], time_common);

% 4. 调用绘图函数
period = 60;
plot_trajectory_analysis(time_common, truth_sync, dr_sync, beacon_sync_deg, period);
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
%% 优化参数
% path=[cfg.outputfolder,'\MEMS-EKF-none.nav'];
% path1=[cfg.outputfolder,'\MEMS-EKF-opti-40.nav'];
% path2=[cfg.outputfolder,'\MEMS-EKF-opti-70.nav'];
% path3=[cfg.outputfolder,'\MEMS-EKF-opti-75.nav'];
% path4=[cfg.outputfolder,'\MEMS-EKF-opti-20.nav'];
% myfigurestartup(12,5,'prese')
% % calc_radial_error(cfg.truthpath,path,path4,path1,path2,path3)
% plot_trj(cfg.truthpath,path,path4,path1,path2,path3)
%% 针对600s的时候
% path=[cfg.outputfolder,'\MEMS-EKF-600s-none.nav'];
% % path1=[cfg.outputfolder,'\MEMS-EKF-600s-20.nav'];
% % path2=[cfg.outputfolder,'\MEMS-EKF-600s-40.nav'];
% path80=[cfg.outputfolder,'\MEMS-EKF-600s-80.nav'];
% % path4=[cfg.outputfolder,'\MEMS-EKF-600s-100.nav'];
% path120=[cfg.outputfolder,'\MEMS-EKF-600s-120.nav'];
% path150=[cfg.outputfolder,'\MEMS-EKF-600s-150.nav'];
% path160=[cfg.outputfolder,'\MEMS-EKF-600s-160.nav'];
% path170=[cfg.outputfolder,'\MEMS-EKF-600s-170.nav'];
% path180=[cfg.outputfolder,'\MEMS-EKF-600s-180.nav'];
% myfigurestartup(12,5,'prese')
% calc_radial_error(cfg.truthpath,path,path80,path160,path180)
% % plot_trj(cfg.truthpath,path,path1,path2,path3)
%% 移动信标对比
% path=[cfg.outputfolder,'\MEMS-EKF-moving-none.nav'];
% path1=[cfg.outputfolder,'\MEMS-EKF-moving-1.nav'];
% path5=[cfg.outputfolder,'\MEMS-EKF-moving-5.nav'];
% path10=[cfg.outputfolder,'\MEMS-EKF-moving-10.nav'];
% path20=[cfg.outputfolder,'\MEMS-EKF-moving-20.nav'];
% path30=[cfg.outputfolder,'\MEMS-EKF-moving-30.nav'];
% path40=[cfg.outputfolder,'\MEMS-EKF-moving-40.nav'];
% myfigurestartup(12,5,'prese')
% calc_radial_error(cfg.truthpath,path,path20,path30,path40)
%% 估计误差对比
if feedback==0
    plot_xk(xkpath,navpath,cfg.truthpath)
end

