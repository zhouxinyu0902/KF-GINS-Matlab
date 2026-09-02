clear;
clc;

%% 1. 环境初始化
% =========================================================================
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);

paths = setup_mems_range_navigation();
glvs;
rng(1);

%% 2. 仿真实验参数
% =========================================================================
% 测量噪声
global rangstd
rangstd = 2;              % Range测量噪声标准差，m

global depstd
depstd = 0.5;             % 深度测量噪声标准差，m

% 功能开关
isoptim = 0;              % 是否进行Range约束下的IMU增量优化
expand = 1;               % optimize_imu中使用的参数
backwardIsOpen_1s = 0;    % 是否进行测距周期后向推算
feedback = 1;             % 是否进行EKF误差反馈
RTSIsOpen = 0;            % 是否进行RTS平滑
IsHardConstrain = 0;      % 是否使用硬约束
MaxLimit = 50;            % Hard Constraint阈值

%% 3. 数据路径与配置
% =========================================================================
param = Param();
dataset_root = paths.simulation;
path = fullfile(dataset_root, 'input');
cfg = ProcessConfig_imusimu(dataset_root);

%% 4. 构造单固定信标Range数据
% =========================================================================
% 信标在轨迹初始点局部ENU坐标系中的位置
% [East, North, Up]，单位：m
beacon_dxyz = [0, 400, 0];

get1beacon(path, beacon_dxyz);
beacontype = "single";

%% 5. 后向处理初始化
% =========================================================================
if backwardIsOpen_1s == 1
    ki2 = 1;
    indexrecord2 = zeros(1,1);
    indexrecord2(1) = 1;
    navdt1s = [];
    NAV = [];
end

%% 6. 加载IMU数据
% =========================================================================
imudata = importdata(cfg.imufilepath);
imu_start_time = imudata(1,1);
imu_end_time = imudata(end,1);

%% 7. 加载单固定信标Range数据
% =========================================================================
rangedata = importdata(cfg.singlerangefilepath);

% Range降采样
% id = 1：保持原频率
% id = 2：每2个取1个
id = 1;
rangedata = rangedata(id:id:end,:);

% 给Range观测添加测量噪声
% 第2列保留理想距离，第3列作为EKF输入测距量
rangedata(:,3) = rangedata(:,3) + ...
    rangstd * randn(size(rangedata,1),1);

range_start_time = rangedata(1,1);
range_end_time = rangedata(end,1);

%% 8. 构造深度观测
% =========================================================================
truth = importdata(cfg.truthpath);

% truth:
% col 2：time
% col 3：latitude
% col 4：longitude
% col 5：height
height = truth(:,[2,5]);

% 添加深度测量噪声
height(:,2) = height(:,2) + ...
    depstd * randn(size(height,1),1);

height_start_time = height(1,1);
height_end_time = height(end,1);

%% 9. 确定公共处理时间
% =========================================================================
starttime = max(imu_start_time, height_start_time);
endtime = min(imu_end_time, height_end_time);

cfg.starttime = max(cfg.starttime, starttime);
cfg.endtime = min(cfg.endtime, endtime);

fprintf('处理时间范围：%.3f s -> %.3f s\n', ...
    cfg.starttime, cfg.endtime);

%% 10. 截取处理区间
% =========================================================================
imudata = imudata( ...
    imudata(:,1) >= cfg.starttime & ...
    imudata(:,1) <= cfg.endtime, :);

rangedata = rangedata( ...
    rangedata(:,1) >= cfg.starttime & ...
    rangedata(:,1) <= cfg.endtime, :);

height = height( ...
    height(:,1) >= cfg.starttime & ...
    height(:,1) <= cfg.endtime, :);

%% 11. 深度插值到IMU时刻
% =========================================================================
% 避免直接使用height(imuindex,:)时IMU和高度索引发生错位
depth_imu = interp1( ...
    height(:,1), ...
    height(:,2), ...
    imudata(:,1), ...
    'linear', ...
    'extrap');

%% 12. 设置滤波模式
% =========================================================================
filter_type = "EKF";

switch filter_type
    case "PureIns"
        cfg.userange = 0;
        cfg.adap = 0;

    case "EKF"
        cfg.userange = 1;
        cfg.adap = 0;

    case "AEKF"
        cfg.userange = 1;
        cfg.adap = 1;

    otherwise
        error('未知滤波类型：%s', filter_type);
end

%% 13. 设置结果保存路径
% =========================================================================
if cfg.userange == 1
    navpath = fullfile( ...
        cfg.outputfolder, ...
        sprintf('MEMS-single-%s.nav', filter_type));
else
    navpath = fullfile( ...
        cfg.outputfolder, ...
        'MEMS-PureIns.nav');
end

navfp = fopen(navpath, 'wt');

if navfp < 0
    error('无法打开导航结果文件：%s', navpath);
end

%% 14. 后向滤波结果文件
% =========================================================================
if backwardIsOpen_1s == 1
    navdt1spath = fullfile( ...
        cfg.outputfolder, ...
        'MEMS-EKFdt1s.nav');
    navdt1sfp = fopen(navdt1spath, 'wt');

    navdt1srotatepath = fullfile( ...
        cfg.outputfolder, ...
        'MEMS-EKFdtrotate1s.nav');
    navdt1srotatefp = fopen(navdt1srotatepath, 'wt');
end

%% 15. RTS输出路径
% =========================================================================
navRTSpath = fullfile( ...
    cfg.outputfolder, ...
    'MEMS-EKFRTS.nav');

if RTSIsOpen == 1
    navRTSfp = fopen(navRTSpath, 'wt');
end

%% 16. 不反馈时保存状态估计
% =========================================================================
if feedback == 0
    xkpath = fullfile( ...
        cfg.outputfolder, ...
        'xk_range.txt');
    xkfp = fopen(xkpath, 'wt');
end

%% 17. EKF初始化
% =========================================================================
[kf, navstate] = myInitialize_15state(cfg);

% EKF观测噪声
kf.rangstd = rangstd;
kf.depthstd = depstd;

% 当前实验设置：放大过程噪声
kf.Qc = kf.Qc * 100;

% 如需修改初始协方差：
% kf.P = kf.P * 100;

%% 18. 导航状态初始化
% =========================================================================
laststate = navstate;

% pos0 / vel0用于Range约束下IMU增量优化
pos0 = navstate.pos;
vel0 = navstate.vel;

%% 19. IMU索引初始化
% =========================================================================
lastimu = imudata(1,:)';
thisimu = imudata(1,:)';
imudt = thisimu(1) - lastimu(1);

% IMU增量优化结果
dtheta = zeros(3,1);
dv = zeros(3,1);

%% 20. Range索引初始化
% =========================================================================
rangeindex = 1;

while rangeindex <= size(rangedata,1) && ...
        rangedata(rangeindex,1) < thisimu(1)
    rangeindex = rangeindex + 1;
end

%% 21. 数据记录初始化
% =========================================================================
lastprecent = 0;

% 相邻状态位置变化，用于调试
dxx = zeros(size(imudata,1),1);
dyy = zeros(size(imudata,1),1);

% 导航结果
nav_record = zeros(size(imudata,1)-1, 11);

%% 22. 正式INS / Range滤波
% =========================================================================
for imuindex = 2:size(imudata,1)

    % -------------------------------------------------------------------------
    % 22.1 保存上一历元状态
    lastimu = thisimu;
    laststate = navstate;
    old_state = navstate;

    thisimu = imudata(imuindex,:)';
    imudt = thisimu(1) - lastimu(1);

    % -------------------------------------------------------------------------
    % 22.2 IMU固定误差补偿
    thisimu(2:4) = thisimu(2:4) - ...
        imudt * cfg.initgyrbiasstd;

    thisimu(5:7) = thisimu(5:7) - ...
        imudt * cfg.initaccbiasstd;

    % 如果后续改成使用滤波估计bias反馈：
    % thisimu(2:4) = thisimu(2:4) - imudt * navstate.gyrbias;
    % thisimu(5:7) = thisimu(5:7) - imudt * navstate.accbias;

    % -------------------------------------------------------------------------
    % 22.3 调整Range索引
    while rangeindex <= size(rangedata,1) && ...
            rangedata(rangeindex,1) < lastimu(1)
        rangeindex = rangeindex + 1;
    end

    % Range结束后仍继续INS推算
    has_range = rangeindex <= size(rangedata,1);

    % -------------------------------------------------------------------------
    % 22.4 情况A：Range恰好落在IMU历元
    if cfg.userange == 1 && ...
            has_range && ...
            lastimu(1) == rangedata(rangeindex,1)

        % Range测量
        Rangedata = rangedata(rangeindex,:);
        depthdata = [lastimu(1), depth_imu(imuindex-1)];

        % Range约束下IMU增量优化
        if isoptim == 1
            optimize_imu;
        end

        % Range EKF量测更新
        if cfg.adap == 1
            kf = myRangeUpdate_adap( ...
                navstate, ...
                Rangedata, ...
                depthdata, ...
                kf);
        else
            kf = myRangeUpdate( ...
                navstate, ...
                Rangedata, ...
                depthdata, ...
                kf);
        end

        % Range索引后移
        rangeindex = rangeindex + 1;

        % Hard Constraint
        if IsHardConstrain == 1
            kf = hard_constrain( ...
                navstate, ...
                imudt, ...
                kf, ...
                MaxLimit);
        end

        % EKF误差反馈
        if feedback == 1
            [kf, navstate] = ...
                myErrorFeedback_range(kf, navstate);
        end

        % Range更新后的INS推算
        laststate = navstate;

        navstate = InsMech( ...
            laststate, ...
            lastimu, ...
            thisimu);

        kf = myInsPropagate_15state_NED( ...
            navstate, ...
            thisimu, ...
            imudt, ...
            kf);

        % 后向处理
        if backwardIsOpen_1s == 1
            [NAV, navdt1s, indexrecord2, ki2] = backward_1s( ...
                imudata, ...
                imuindex, ...
                ki2, ...
                kf, ...
                navstate, ...
                indexrecord2, ...
                rangedata, ...
                navdt1s, ...
                NAV, ...
                height, ...
                rangeindex, ...
                pos0);
        end

        % 保存下一段优化起始状态
        pos0 = navstate.pos;
        vel0 = navstate.vel;

    % -------------------------------------------------------------------------
    % 22.5 情况B：Range位于两个IMU历元之间
    elseif cfg.userange == 1 && ...
            has_range && ...
            lastimu(1) < rangedata(rangeindex,1) && ...
            thisimu(1) > rangedata(rangeindex,1)

        Rangedata = rangedata(rangeindex,:);
        range_time = rangedata(rangeindex,1);

        % Range时刻深度插值
        depth_at_range = interp1( ...
            height(:,1), ...
            height(:,2), ...
            range_time, ...
            'linear', ...
            'extrap');

        depthdata = [range_time, depth_at_range];

        % IMU增量优化
        if isoptim == 1
            optimize_imu;
        end

        % 将IMU增量插值到Range时刻
        [firstimu, secondimu] = interpolate( ...
            lastimu, ...
            thisimu, ...
            range_time);

        % 推算到Range时刻
        imudt_first = firstimu(1) - lastimu(1);

        navstate = InsMech( ...
            laststate, ...
            lastimu, ...
            firstimu);

        kf = myInsPropagate_15state_NED( ...
            navstate, ...
            firstimu, ...
            imudt_first, ...
            kf);

        % Range量测更新
        if cfg.adap == 1
            kf = myRangeUpdate_adap( ...
                navstate, ...
                Rangedata, ...
                depthdata, ...
                kf);
        else
            kf = myRangeUpdate( ...
                navstate, ...
                Rangedata, ...
                depthdata, ...
                kf);
        end

        % Hard Constraint
        if IsHardConstrain == 1
            kf = hard_constrain( ...
                navstate, ...
                imudt_first, ...
                kf, ...
                MaxLimit);
        end

        % EKF误差反馈
        if feedback == 1
            [kf, navstate] = ...
                myErrorFeedback_range(kf, navstate);
        end

        rangeindex = rangeindex + 1;

        % 从Range时刻继续推算到当前IMU时刻
        laststate = navstate;
        lastimu_interpolated = firstimu;

        imudt_second = ...
            secondimu(1) - lastimu_interpolated(1);

        navstate = InsMech( ...
            laststate, ...
            lastimu_interpolated, ...
            secondimu);

        kf = myInsPropagate_15state_NED( ...
            navstate, ...
            secondimu, ...
            imudt_second, ...
            kf);

        % 后向处理
        if backwardIsOpen_1s == 1
            [NAV, navdt1s, indexrecord2, ki2] = backward_1s( ...
                imudata, ...
                imuindex, ...
                ki2, ...
                kf, ...
                navstate, ...
                indexrecord2, ...
                rangedata, ...
                navdt1s, ...
                NAV, ...
                height, ...
                rangeindex, ...
                pos0);
        end

        % 保存下一段优化起点
        pos0 = navstate.pos;
        vel0 = navstate.vel;

    % -------------------------------------------------------------------------
    % 22.6 无Range：纯INS推算
    else

        % 加入IMU增量优化修正量
        if isoptim == 1
            thisimu(2:4) = thisimu(2:4) + dtheta;
            thisimu(5:7) = thisimu(5:7) + dv;
        end

        % INS机械编排
        navstate = InsMech( ...
            laststate, ...
            lastimu, ...
            thisimu);

        % 深度约束
        navstate.pos(3) = depth_imu(imuindex);

        % EKF状态传播
        kf = myInsPropagate_15state_NED( ...
            navstate, ...
            thisimu, ...
            imudt, ...
            kf);
    end

    % -------------------------------------------------------------------------
    % 22.7 记录相邻历元水平位置变化
    dxx(imuindex) = ...
        (old_state.pos(1) - navstate.pos(1)) * glv.Re;

    dyy(imuindex) = ...
        (old_state.pos(2) - navstate.pos(2)) * glv.Re;

    % -------------------------------------------------------------------------
    % 22.8 保存导航结果
    nav = zeros(11,1);
    nav(2) = navstate.time;

    nav(3:5) = [ ...
        navstate.pos(1) * param.R2D; ...
        navstate.pos(2) * param.R2D; ...
        navstate.pos(3)];

    nav(6:8) = navstate.vel;
    nav(9:11) = navstate.att * param.R2D;

    nav_record(imuindex-1,:) = nav';

    fprintf( ...
        navfp, ...
        ['%2d %12.6f %12.8f %12.8f ', ...
         '%8.4f %8.4f %8.4f %8.4f ', ...
         '%8.4f %8.4f %8.4f\n'], ...
        nav);

    % -------------------------------------------------------------------------
    % 22.9 不反馈时保存状态
    if feedback == 0
        xk = zeros(16,1);
        xk(1) = navstate.time;
        xk(2:16) = kf.x(1:15);

        fprintf( ...
            xkfp, ...
            ['%12.6f ', repmat('%12.8f ',1,15), '\n'], ...
            xk);
    end

    % -------------------------------------------------------------------------
    % 22.10 输出处理进度
    if imuindex / size(imudata,1) - lastprecent > 0.20
        fprintf( ...
            'processing %d %%\n', ...
            floor(imuindex * 100 / size(imudata,1)));

        lastprecent = imuindex / size(imudata,1);
    end
end

%% 23. 关闭文件
% =========================================================================
fclose(navfp);

if feedback == 0
    fclose(xkfp);
end

if RTSIsOpen == 1
    fclose(navRTSfp);
end

disp('Range/INS Integration Processing Finished!');

%% 24. RTS平滑结果评价
% =========================================================================
if RTSIsOpen == 1
    calc_error(navRTSpath, cfg.truthpath);
end

%% 25. 测距周期后向推算结果
% =========================================================================
if backwardIsOpen_1s == 1
    fprintf( ...
        navdt1sfp, ...
        ['%2d %12.6f %12.8f %12.8f ', ...
         '%8.4f %8.4f %8.4f %8.4f ', ...
         '%8.4f %8.4f %8.4f\n'], ...
        navdt1s');

    fprintf( ...
        navdt1srotatefp, ...
        ['%2d %12.6f %12.8f %12.8f ', ...
         '%8.4f %8.4f %8.4f %8.4f ', ...
         '%8.4f %8.4f %8.4f\n'], ...
        NAV');

    fclose(navdt1sfp);
    fclose(navdt1srotatefp);

    % 对后向结果进行平滑
    nav11 = NAV(indexrecord2(1:end-1),:);
    east_series = nav11(:,3);
    north_series = nav11(:,4);

    nav11(:,3) = smooth(east_series, 0.005, 'rloess');
    nav11(:,4) = smooth(north_series, 0.005, 'rloess');

    fpath = fullfile(cfg.outputfolder, 'smooth.txt');
    fp = fopen(fpath, 'wt');

    fprintf( ...
        fp, ...
        ['%2d %12.6f %12.8f %12.8f ', ...
         '%8.4f %8.4f %8.4f %8.4f ', ...
         '%8.4f %8.4f %8.4f\n'], ...
        nav11');

    fclose(fp);

    calc_radial_error( ...
        cfg.truthpath, ...
        navpath, ...
        fpath);

    calc_error(fpath, cfg.truthpath);
end

%% 26. 基础导航误差评价
% =========================================================================
calc_radial_error(cfg.truthpath, navpath);
calc_error(navpath, cfg.truthpath);

%% 27. 轨迹与单固定信标绘图
% =========================================================================
myfigurestartup(12,5,'prese');
plot_trj(cfg.truthpath, navpath);
hold on;

% Truth起始位置作为局部坐标原点
pos00 = [ ...
    d2r(truth(1,3:4)), ...
    truth(1,5)]';

% 单固定信标位置
beacon_xyz = pos2dxyz( ...
    rangedata(1,4:6), ...
    pos00);

plot( ...
    beacon_xyz(1), ...
    beacon_xyz(2), ...
    'hexagram', ...
    'MarkerSize', 10, ...
    'DisplayName', '固定信标');

legend('Location','best');

%% 28. 相邻导航历元位置变化
% =========================================================================
figure;
plot(dxx,'.');
hold on;
plot(dyy,'--');

xlabel('IMU历元');
ylabel('位置变化 / m');
legend('\Delta x','\Delta y');
grid on;

%% 29. 轨迹与信标几何关系分析
% =========================================================================
nn = importdata(navpath);

t_start = max([ ...
    rangedata(1,1), ...
    truth(1,2), ...
    nn(1,2)]);

t_end = min([ ...
    rangedata(end,1), ...
    truth(end,2), ...
    nn(end,2)]);

% 统一为1 s采样
time_common = (t_start:1:t_end)';

truth_sync = interp1( ...
    truth(:,2), ...
    truth(:,3:5), ...
    time_common);

nav_sync = interp1( ...
    nn(:,2), ...
    nn(:,3:5), ...
    time_common);

beacon_sync_deg = interp1( ...
    rangedata(:,1), ...
    [r2d(rangedata(:,4:5)), rangedata(:,6)], ...
    time_common);

period = 60;

plot_trajectory_analysis( ...
    time_common, ...
    truth_sync, ...
    nav_sync, ...
    beacon_sync_deg, ...
    period);

%% 30. 不反馈时查看状态估计误差
% =========================================================================
if feedback == 0
    plot_xk( ...
        xkpath, ...
        navpath, ...
        cfg.truthpath);
end

%% 31. 局部函数
% =========================================================================
function get1beacon(path, dxyz)
%GET1BEACON 根据仿真Truth构造单固定信标1 Hz测距数据
%
% 输入：
% path  ：仿真input目录
% dxyz ：信标相对轨迹初始位置的ENU坐标 [East, North, Up]，m
%
% 输出：
% single_range.txt
%
% 数据格式：
% col 1：time
% col 2：ideal range
% col 3：measured range
% col 4：beacon latitude [rad]
% col 5：beacon longitude [rad]
% col 6：beacon height [m]
%
% 第2、3列在此函数中相同，测距噪声在主程序中加入第3列。

glvs;

% -------------------------------------------------------------------------
% 加载Truth
truth_file = fullfile(path, 'truth.nav');
truth = importdata(truth_file);

% -------------------------------------------------------------------------
% Truth降采样到1 Hz
truth_dt = truth(2,2) - truth(1,2);
sample_interval = round(1 / truth_dt);
truth_1hz = truth(sample_interval:sample_interval:end, 2:5);

% -------------------------------------------------------------------------
% 局部坐标参考原点
origin = [ ...
    d2r(truth_1hz(1,2)), ...
    d2r(truth_1hz(1,3)), ...
    truth_1hz(1,4)]';

% -------------------------------------------------------------------------
% 信标ENU -> 经纬高
beacon_pos = dxyz2pos(dxyz, origin);

% -------------------------------------------------------------------------
% Truth经纬高 -> ENU
trajectory_pos = truth_1hz(:,2:4);
trajectory_pos(:,1:2) = d2r(trajectory_pos(:,1:2));

trajectory_xyz = pos2dxyz( ...
    trajectory_pos, ...
    origin);

% -------------------------------------------------------------------------
% 计算水平距离
delta_e = trajectory_xyz(:,1) - dxyz(1);
delta_n = trajectory_xyz(:,2) - dxyz(2);
distance = hypot(delta_e, delta_n);

% -------------------------------------------------------------------------
% 信标位置复制到所有历元
beacon_all = repmat( ...
    beacon_pos, ...
    length(distance), ...
    1);

% -------------------------------------------------------------------------
% 构造Range文件
range_data = [ ...
    truth_1hz(:,1), ...
    distance, ...
    distance, ...
    beacon_all];

% -------------------------------------------------------------------------
% 保存
output_file = fullfile(path, 'single_range.txt');

writematrix( ...
    range_data, ...
    output_file, ...
    'Delimiter', ...
    ' ');

fprintf('单固定信标Range数据已生成：%s\n', output_file);
fprintf( ...
    '信标ENU位置：[%.2f, %.2f, %.2f] m\n', ...
    dxyz(1), dxyz(2), dxyz(3));
end