%% 环境清理与全局参数设置
clear; clc;
global rangstd depstd azimustd;

% 传感器标准差
rangstd  = 2;    % 距离标准差 (m)
depstd   = 0.5;  % 深度标准差 (m)
azimustd = 0.2;  % 方位角标准差 (rad)

rng(1);          % 固定随机种子
glvs;            % 加载常用物理常数 (由惯导工具箱提供)

%% 1. 配置参数与路径
param = Param();
path = 'D:\Github\KF-GINS-Matlab\MEMS_RANGE\data_830_430_2';
cfg = ProcessConfig_MEMS_exper(path);

% 滤波与模式开关
type_list      = "EKF";   % 可选 "PureIns", "EKF", "AEKF"
beacontype     = "moving"; % "single", "moving", "3", "2"
meas_mode      = "range";  % "range", "range+azi"
feedback    = 1;        % 是否状态反馈
is_hard_constrain = 0;     % 是否开启硬约束
backwardIsOpen_1s = 0;        % 是否开启1秒后向推算
RTS_on         = 0;        % 是否开启RTS平滑
isoptim = 0; expand = 100;% 是否

%% 2. 模拟数据生成 (信标)
switch beacontype
    case {"3", "2"}
        get3beacons(path);
    case "single"
        get1beacon(path, [-2000, 4000, 0]);
    case "moving"
        getmovingbeacon(path, 'plan');
end

%% 3. 数据加载与预处理
% 加载 IMU 数据
imudata = importdata(cfg.imufilepath);

% 加载并根据信标类型筛选距离数据
range = {importdata(cfg.rangefile1path), importdata(cfg.rangefile2path),...
    importdata(cfg.rangefile3path), importdata(cfg.singlerangefilepath),...
    importdata(cfg.rangefilemovingpath)};
id = 1; 
for imuindex=1:length(range)
    range{imuindex} = range{imuindex}(id:id:end,:);
end

if beacontype=="3"||beacontype=="2"
    numnum = str2double(beacontype);
    rangedata = zeros(size(range{1}));
    for imuindex=1:numnum
        rangedata(imuindex:numnum:end,:)=range{seq(nnn,imuindex)}(imuindex:numnum:end,:);
    end
elseif beacontype=="single"
    rangedata = range{4};
elseif beacontype=="moving"
    rangedata = range{5};
end
% 添加测量噪声
rangedata(:, 3) = rangedata(:, 3) + normrnd(0, rangstd, size(rangedata(:, 3)));

para = struct('sigma_min', 0.1, 'sigma_max', 0.5, 'power_fac', 2, 'jitter', 0.2);
rangedata(:, 7) = add_azimuth_noise_irregular(rangedata(:, 7), para);

% 加载高度/深度数据并加噪
truth = importdata(cfg.truthpath);
height = truth(:, [2, 5]);
height(:, 2) = height(:, 2) + normrnd(0, depstd, size(height(:, 2)));

% 时间对齐与截取
cfg.starttime = max([imudata(1,1), height(1,1), cfg.starttime]);
cfg.endtime   = min([imudata(end,1), height(end,1), cfg.endtime]);

imudata    = imudata(imudata(:,1) >= cfg.starttime & imudata(:,1) <= cfg.endtime, :);
rangedata  = rangedata(rangedata(:,1) >= cfg.starttime & rangedata(:,1) <= cfg.endtime, :);
height = height(height(:,1) >= cfg.starttime & height(:,1) <= cfg.endtime, :);

%% 4. 滤波处理循环
for filter_type = type_list
    % 设置算法属性
    cfg.userange = ~strcmp(filter_type, 'PureIns');
    cfg.adap     = strcmp(filter_type, 'AEKF');
    
    % 打开文件句柄 (Nav 结果保存)
    navpath = fullfile(cfg.outputfolder, sprintf('MEMS-%s.nav', filter_type));
    navfp = fopen(navpath, 'wt');
    
    % 初始化 KF 状态
    [kf, navstate] = myInitialize_15state(cfg);
    kf.Qc = kf.Qc * 100; % 放大过程噪声
    kf.P0 = kf.P * 100;
    
    rangeindex = 1;
    last_percent = 0;
    laststate = navstate;
    % 惯性数据优化
    dtheta = zeros(3,1);
    dv = zeros(3,1);
    %% --- 主循环 ---
    for imuindex = 2:size(imudata, 1)
        % 提取 IMU 数据并计算步长
        imudt = imudata(imuindex, 1) - imudata(imuindex-1, 1);
        thisimu = imudata(imuindex, :)';
        lastimu = imudata(imuindex-1, :)';
        laststate = navstate;
        % 匹配当前时刻的距离观测
        while (rangeindex <= size(rangedata, 1) && rangedata(rangeindex, 1) < lastimu(1, 1))
            rangeindex = rangeindex + 1;
        end
        if (rangeindex > size(rangedata, 1))
            % rangeindex = rangeindex - 1;
            disp('range file END!');
            break;
        end

        % A. 测量更新时刻
        if lastimu(1, 1) == rangedata(rangeindex, 1) && cfg.userange==1
            % 测量更新
            Rangedata = rangedata(rangeindex,:);
            depthdata = height(imuindex,1:2);

            thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * cfg.initgyrbiasstd);
            thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * cfg.initaccbiasstd);

            if isoptim == 1
                optimize_imu;
            end

            if cfg.adap==1
                kf = myRangeUpdate_adap(navstate, Rangedata, depthdata, kf);
            else
                kf = myRangeUpdate(navstate, Rangedata, depthdata, kf);
            end
            rangeindex = rangeindex + 1;

            if feedback==1
                [kf, navstate] = myErrorFeedback_range(kf, navstate);
                % [kf, navstate] = myErrorFeedback_range_posonly(kf, navstate);
            end
            laststate = navstate;

            % 惯导推算
            imudt = thisimu(1, 1) - lastimu(1, 1);
            navstate = InsMech(laststate, lastimu, thisimu);
            kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
            
            % 惯性数据优化存储的内容
            pos0 = navstate.pos;
            vel0 = navstate.vel;

            IsRangeUpdate = 1;

            if backwardIsOpen_1s == 1
                backward_1s;
            end


        elseif (lastimu(1, 1) < rangedata(rangeindex, 1) && thisimu(1, 1) > rangedata(rangeindex, 1))&& cfg.userange==1

            Rangedata = rangedata(rangeindex,:);
            depthdata = [height(imuindex,1),height(imuindex,2)];

            thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * cfg.initgyrbiasstd);
            thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * cfg.initaccbiasstd);

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
            end
            % RTS平滑存储的内容
            P_seq(:, :, imuindex-1) = kf.P;
            P_pred_seq(:, :, imuindex-1) = kf.Pk_k1;

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

            % 惯性数据优化存储的内容
            pos0 = navstate.pos;
            vel0 = navstate.vel;

            IsRangeUpdate=1;

            if backwardIsOpen_1s == 1
                backward_1s;
            end

        % B. 纯惯导时刻
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

            P_seq(:, :, imuindex-1) = kf.P;
            P_pred_seq(:, :, imuindex-1) = kf.Pk_k1;
            IsRangeUpdate=0;
        end
        % C. 结果存储
        nav_res = [0; navstate.time; navstate.pos(1:2)*param.R2D; navstate.pos(3); ...
                   navstate.vel; navstate.att*param.R2D];
        fprintf(navfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n', nav_res);
        
        % 进度显示
        if (imuindex / size(imudata, 1) - last_percent > 0.2)
            fprintf('处理进度: %d%%\n', floor(imuindex * 100 / size(imudata, 1)));
            last_percent = imuindex / size(imudata, 1);
        end
    end
    
    fclose(navfp);
end

%% 5. 后处理与绘图分析
fprintf('计算径向误差...\n');
calc_radial_error(cfg.truthpath, navpath);
calc_error(navpath, cfg.truthpath);

% 轨迹绘制
plot_trj(cfg.truthpath, navpath);
% (此处可补充信标位置的绘制逻辑)