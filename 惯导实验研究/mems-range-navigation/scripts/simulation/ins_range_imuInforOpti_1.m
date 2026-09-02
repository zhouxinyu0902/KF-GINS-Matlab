clear;
clc;

%%  1. 环境初始化
% =========================================================================
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
paths = setup_mems_range_navigation();
glvs;
rng(1);
%% % 2. 仿真与滤波参数
% =========================================================================

% 是否进行滤波误差反馈
feedback = true;

% 测量噪声
range_noise_std = 2;      % m
depth_noise_std = 0.2;    % m

% 数据抽取间隔
% 1：保留全部1 Hz Range
% 2：Range降为0.5 Hz
sample_step = 1;

% 多信标轮换顺序
beacon_sequence = [1, 2, 3];

% 当前采用第几组轮换顺序
sequence_id = 1;

% 当前信标类型：
% "single"
% "moving"
% "2"
% "3"
beacon_type = "single";

% 移动信标轨迹类型
moving_type = "trjlike";

% 当前滤波类型
% "PureIns"
% "EKF"
% "AEKF"
filter_type = "EKF";
%%  3. 参数与路径配置
% =========================================================================
param = Param();
dataset_root = paths.simulation;
input_path = fullfile(dataset_root, 'input');
cfg = ProcessConfig_imusimu(dataset_root);
%%  4. 构造信标测距数据
% =========================================================================
switch beacon_type
    case "single"
        % 单固定信标：
        % 相对于轨迹初始点 ENU = [0, 400, 0] m
        get1beacon( ...
            input_path, ...
            [0, 400, 0]);
    case {"2", "3"}
        get3beacons(input_path);
    case "moving"
        getmovingbeacon( ...
            input_path, ...
            moving_type);
    otherwise
        error( ...
            '未知 beacon_type：%s', ...
            beacon_type);
end
%%  5. 加载 IMU
% =========================================================================
imudata = importdata(cfg.imufilepath);

imu_start_time = imudata(1,1);
imu_end_time   = imudata(end,1);
%%  6. 加载并构造 Range 数据
% =========================================================================
switch beacon_type
    case "single"
        rangedata = importdata( ...
            cfg.singlerangefilepath);
    case "moving"

        rangedata = importdata( ...
            cfg.rangefilemovingpath);
    case {"2", "3"}
        range1 = importdata(cfg.rangefile1path);
        range2 = importdata(cfg.rangefile2path);
        range3 = importdata(cfg.rangefile3path);
        range_all = {
            range1, ...
            range2, ...
            range3
        };
        beacon_num = str2double(beacon_type);
        % Range降采样
        for i = 1:numel(range_all)

            range_all{i} = ...
                range_all{i}( ...
                    sample_step:sample_step:end, :);
        end

        % ---------------------------------------------------------------
        % 构造轮换信标测距
        %
        % 例如2信标：
        % B1 B2 B1 B2 ...
        %
        % 例如3信标：
        % B1 B2 B3 B1 B2 B3 ...
        % ---------------------------------------------------------------
        rangedata = zeros(size(range_all{1}));
        for i = 1:beacon_num
            beacon_id = ...
                beacon_sequence(i);

            rangedata( ...
                i:beacon_num:end, :) = ...
                range_all{beacon_id}( ...
                    i:beacon_num:end, :);
        end
end


%% 单/移动信标降采样
if beacon_type == "single" || beacon_type == "moving"
    rangedata = ...
        rangedata( ...
            sample_step:sample_step:end, :);
end

%% -------------------------------------------------------------------------
% 给实际送入滤波器的 Range 加噪声
%
% 当前程序使用第3列作为测距量：
%
% R_meas = Rangedata(3)
% -------------------------------------------------------------------------
rangedata(:,3) = rangedata(:,3) + ...
    range_noise_std * randn(size(rangedata,1),1);
range_start_time = rangedata(1,1);
range_end_time   = rangedata(end,1);

%%  7. 加载 Truth 并生成深度数据
% =========================================================================
truth = importdata(cfg.truthpath);
height = truth(:, [2,5]);
height(:,2) = height(:,2) + ...
    depth_noise_std * randn(size(height,1),1);
height_start_time = height(1,1);
height_end_time   = height(end,1);

%% 8. 确定实际处理时间
% =========================================================================

data_start_time = max( ...
    imu_start_time, ...
    height_start_time);

data_end_time = min( ...
    imu_end_time, ...
    height_end_time);


cfg.starttime = max( ...
    cfg.starttime, ...
    data_start_time);

cfg.endtime = min( ...
    cfg.endtime, ...
    data_end_time);


fprintf( ...
    '处理时间：%.3f s -> %.3f s\n', ...
    cfg.starttime, ...
    cfg.endtime);


%%  9. 截取处理区间
% =========================================================================

imudata = imudata( ...
    imudata(:,1) >= cfg.starttime & ...
    imudata(:,1) <= cfg.endtime, :);


rangedata = rangedata( ...
    rangedata(:,1) >= cfg.starttime & ...
    rangedata(:,1) <= cfg.endtime, :);


%% -------------------------------------------------------------------------
% 将深度严格插值到 IMU 时刻
% -------------------------------------------------------------------------
depth_imu = interp1( ...
    height(:,1), ...
    height(:,2), ...
    imudata(:,1), ...
    'linear');


if any(~isfinite(depth_imu))
    error('深度数据无法完整插值到IMU时刻。');
end


%% =========================================================================
% 10. 设置滤波模式
% =========================================================================
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

        error( ...
            '未知滤波类型：%s', ...
            filter_type);

end


%% =========================================================================
% 11. 输出文件
% =========================================================================
if cfg.userange

    if beacon_type == "moving"

        result_name = ...
            beacon_type + "-" + moving_type;

    else

        result_name = beacon_type;

    end


    navpath = fullfile( ...
        cfg.outputfolder, ...
        sprintf( ...
            'MEMS-%s-%s.nav', ...
            char(result_name), ...
            char(filter_type)));


    fprintf( ...
        '使用 Range 数据：%s\n', ...
        result_name);

else

    navpath = fullfile( ...
        cfg.outputfolder, ...
        'MEMS-PureIns.nav');

    disp('PURE INS');

end


navfp = fopen(navpath, 'wt');

if navfp < 0
    error('无法创建导航结果文件：%s', navpath);
end


%% 非反馈模式：保存状态估计
if ~feedback

    xkpath = fullfile( ...
        cfg.outputfolder, ...
        'xk_range.txt');

    xkfp = fopen(xkpath, 'wt');

end


%%  12. 滤波初始化
% =========================================================================
[kf, navstate] = ...
    myInitialize_15state(cfg);
kf.rangstd = 2;
kf.depthstd = 0.2;

% 当前实验人为放大过程噪声
kf.Qc = 5 * kf.Qc;


% 注意：
% 如果你的目的是放大当前初始协方差，
% 应确认 myInsPropagate_15state 使用的是 kf.P 还是 kf.P0
kf.P0 = 10 * kf.P;

laststate = navstate;

% 保存1 s IMU优化使用的起始状态
pos0 = navstate.pos;
vel0 = navstate.vel;


%%  13. IMU 初始化
% =========================================================================
lastimu = imudata(1,:)';
thisimu = imudata(1,:)';

dtheta = zeros(3,1);
dv     = zeros(3,1);


%%  14. Range 索引初始化
% =========================================================================
rangeindex = find( ...
    rangedata(:,1) >= thisimu(1), ...
    1, ...
    'first');


if isempty(rangeindex)

    rangeindex = size(rangedata,1) + 1;

end


%% 时间匹配容差
time_tolerance = 1e-6;


%%  15. 正式 INS / Range 滤波
% =========================================================================

for imuindex = 2:size(imudata,1)

    %% --------------------------------------------------------------------
    % 15.1 保存上一时刻数据
    % ---------------------------------------------------------------------

    lastimu = thisimu;
    laststate = navstate;

    thisimu = imudata(imuindex,:)';

    imudt = ...
        thisimu(1) - lastimu(1);


    %% --------------------------------------------------------------------
    % 15.2 去除仿真时设置的固定IMU偏差
    % ---------------------------------------------------------------------

    thisimu(2:4) = ...
        thisimu(2:4) ...
        - imudt * cfg.initgyrbiasstd;


    thisimu(5:7) = ...
        thisimu(5:7) ...
        - imudt * cfg.initaccbiasstd;


    %% --------------------------------------------------------------------
    % 15.3 调整 Range 索引
    % ---------------------------------------------------------------------

    while rangeindex <= size(rangedata,1) && ...
            rangedata(rangeindex,1) < lastimu(1) - time_tolerance

        rangeindex = rangeindex + 1;

    end


    has_range = ...
        rangeindex <= size(rangedata,1);


    range_update = false;


    if cfg.userange && has_range

        range_update = ...
            abs( ...
                rangedata(rangeindex,1) ...
                - lastimu(1)) ...
            <= time_tolerance;

    end


    %% --------------------------------------------------------------------
    % 15.4 Range 更新
    % ---------------------------------------------------------------------

    if range_update

        Rangedata = ...
            rangedata(rangeindex,:);


        % 当前量测时刻实际上对应 lastimu
        depthdata = [
            lastimu(1), ...
            depth_imu(imuindex-1)
        ];


        %% ---------------------------------------------------------------
        % 利用 Range 对最近1 s IMU增量进行优化
        % ---------------------------------------------------------------

        param.sigma_g = ...
            cfg.gyrarw * 300;

        param.sigma_a = ...
            cfg.accvrw * 300;

        param.sigma_R = ...
            range_noise_std;


        state.Cb0_n = ...
            navstate.cbn;

        state.wnin = ...
            navstate.wnin;


        beacon_pos = ...
            Rangedata(4:6)';


        R_meas = ...
            Rangedata(3);


        state.v0 = vel0;
        state.p0 = pos0;


        % IMU 100 Hz，取最近100个采样
        imu_window_length = 100;


        if imuindex >= imu_window_length

            Hz = 1;

            dtheta_tilde = sum( ...
                imudata( ...
                    imuindex-imu_window_length+1:imuindex, ...
                    2:4), ...
                1)';


            dv_tilde = sum( ...
                imudata( ...
                    imuindex-imu_window_length+1:imuindex, ...
                    5:7), ...
                1)';


            [dtheta_opt, dv_opt] = ...
                optimize_imu_incremental_1s( ...
                    dtheta_tilde, ...
                    dv_tilde, ...
                    state, ...
                    beacon_pos, ...
                    R_meas, ...
                    param, ...
                    Hz);


            % 将1 s整体修正量均摊到100 Hz
            dtheta = ...
                (dtheta_opt - dtheta_tilde) ...
                / imu_window_length;


            dv = ...
                (dv_opt - dv_tilde) ...
                / imu_window_length;

        end


        %% 将IMU优化量施加到当前增量
        thisimu(2:4) = ...
            thisimu(2:4) + dtheta;

        thisimu(5:7) = ...
            thisimu(5:7) + dv;


        %% ---------------------------------------------------------------
        % EKF量测更新
        % ---------------------------------------------------------------

        if cfg.adap

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


        %% ---------------------------------------------------------------
        % 误差反馈
        % ---------------------------------------------------------------

        if feedback

            [kf, navstate] = ...
                myErrorFeedback_range( ...
                    kf, ...
                    navstate);

        end


        %% Range指针前移
        rangeindex = rangeindex + 1;


        %% ---------------------------------------------------------------
        % Range更新完成后，再从 lastimu 推算到 thisimu
        % ---------------------------------------------------------------

        laststate = navstate;


        navstate = InsMech( ...
            laststate, ...
            lastimu, ...
            thisimu);


        kf = myInsPropagate_15state( ...
            navstate, ...
            thisimu, ...
            imudt, ...
            kf);


        %% 保存下一段1 s优化的起始状态
        pos0 = navstate.pos;
        vel0 = navstate.vel;


    else

        %% ---------------------------------------------------------------
        % 15.5 无 Range 时仅惯导推算
        % ---------------------------------------------------------------

        thisimu(2:4) = ...
            thisimu(2:4) + dtheta;


        thisimu(5:7) = ...
            thisimu(5:7) + dv;


        navstate = InsMech( ...
            laststate, ...
            lastimu, ...
            thisimu);


        %% 深度约束
        navstate.pos(3) = ...
            depth_imu(imuindex);


        %% 状态协方差传播
        kf = myInsPropagate_15state( ...
            navstate, ...
            thisimu, ...
            imudt, ...
            kf);

    end


    %% --------------------------------------------------------------------
    % 15.6 保存导航结果
    % ---------------------------------------------------------------------

    nav = zeros(11,1);

    nav(2) = navstate.time;

    nav(3:5) = [
        navstate.pos(1) * param.R2D;
        navstate.pos(2) * param.R2D;
        navstate.pos(3)
    ];

    nav(6:8) = ...
        navstate.vel;

    nav(9:11) = ...
        navstate.att * param.R2D;


    fprintf( ...
        navfp, ...
        ['%2d %12.6f %12.8f %12.8f ', ...
         '%8.4f %8.4f %8.4f %8.4f ', ...
         '%8.4f %8.4f %8.4f\n'], ...
        nav);


    %% --------------------------------------------------------------------
    % 保存未反馈状态
    % ---------------------------------------------------------------------

    if ~feedback

        xk = [
            navstate.time;
            kf.x(1:15)
        ];


        fprintf( ...
            xkfp, ...
            repmat('%12.8f ',1,16), ...
            xk);

        fprintf(xkfp, '\n');

    end

end


%% =========================================================================
% 16. 关闭文件
% =========================================================================
fclose(navfp);

if ~feedback
    fclose(xkfp);
end

disp('Range/INS Integration Processing Finished!');


%% =========================================================================
% 17. 误差分析
% =========================================================================
calc_error( ...
    navpath, ...
    cfg.truthpath);


%% =========================================================================
% 18. 轨迹及信标绘图
% =========================================================================
marker = [
    ">";
    "hexagram";
    "pentagram"
];


myfigurestartup(12,5,'prese');

plot_trj( ...
    cfg.truthpath, ...
    navpath);

hold on;


pos_ref = [
    d2r(truth(1,3:4)), ...
    truth(1,5)
]';


switch beacon_type

    case {"2", "3"}

        beacon_num = str2double(beacon_type);

        beacon_xyz = pos2dxyz( ...
            rangedata(1:beacon_num,4:6), ...
            pos_ref);


        for i = 1:beacon_num

            plot( ...
                beacon_xyz(i,1), ...
                beacon_xyz(i,2), ...
                marker(i), ...
                'DisplayName', ...
                sprintf('信标%d',i));

        end


    case "single"

        beacon_xyz = pos2dxyz( ...
            rangedata(1,4:6), ...
            pos_ref);


        plot( ...
            beacon_xyz(1), ...
            beacon_xyz(2), ...
            marker(1), ...
            'DisplayName', ...
            '信标');


    case "moving"

        beacon_xyz = pos2dxyz( ...
            rangedata(:,4:6), ...
            pos_ref);


        plot( ...
            beacon_xyz(:,1), ...
            beacon_xyz(:,2), ...
            'DisplayName', ...
            '移动信标');


        plot( ...
            beacon_xyz(1,1), ...
            beacon_xyz(1,2), ...
            '>', ...
            'DisplayName', ...
            '信标起点');


        plot( ...
            beacon_xyz(end,1), ...
            beacon_xyz(end,2), ...
            '<', ...
            'DisplayName', ...
            '信标终点');

end


legend('Location','best');


%% =========================================================================
% 19. 未反馈时查看状态估计
% =========================================================================
if ~feedback

    plot_xk( ...
        xkpath, ...
        navpath, ...
        cfg.truthpath);

end


%% =========================================================================
% 辅助函数：构造单固定信标
% =========================================================================
function get1beacon(path, dxyz)

    glvs;

    truth_file = ...
        fullfile(path, 'truth.nav');

    truth = importdata(truth_file);


    %% --------------------------------------------------------------------
    % 将Truth降采样到1 Hz
    % ---------------------------------------------------------------------

    truth_dt = ...
        truth(2,2) - truth(1,2);


    sample_num = ...
        round(1 / truth_dt);


    truth_1hz = ...
        truth( ...
            sample_num:sample_num:end, ...
            2:5);


    %% --------------------------------------------------------------------
    % 局部ENU坐标原点
    % ---------------------------------------------------------------------

    origin = [
        d2r(truth_1hz(1,2)), ...
        d2r(truth_1hz(1,3)), ...
        truth_1hz(1,4)
    ]';


    %% --------------------------------------------------------------------
    % 信标ENU -> LLH
    % ---------------------------------------------------------------------

    beacon_pos = ...
        dxyz2pos( ...
            dxyz, ...
            origin);


    %% --------------------------------------------------------------------
    % Truth -> ENU
    % ---------------------------------------------------------------------

    trajectory_pos = ...
        truth_1hz(:,2:4);


    trajectory_pos(:,1:2) = ...
        d2r(trajectory_pos(:,1:2));


    trajectory_xyz = ...
        pos2dxyz( ...
            trajectory_pos, ...
            origin);


    %% --------------------------------------------------------------------
    % 计算二维水平距离
    % ---------------------------------------------------------------------

    delta_x = ...
        trajectory_xyz(:,1) - dxyz(1);

    delta_y = ...
        trajectory_xyz(:,2) - dxyz(2);


    distance = hypot( ...
        delta_x, ...
        delta_y);


    %% --------------------------------------------------------------------
    % 信标位置扩展
    % ---------------------------------------------------------------------

    beacon_all = repmat( ...
        beacon_pos, ...
        length(distance), ...
        1);


    %% --------------------------------------------------------------------
    % Range文件
    %
    % col 1：time
    % col 2：参考距离
    % col 3：测量距离
    % col 4~6：beacon LLH
    %
    % 此处2、3列先相同，主程序中再向第3列加入测距噪声。
    % ---------------------------------------------------------------------

    range_data = [
        truth_1hz(:,1), ...
        distance, ...
        distance, ...
        beacon_all
    ];


    %% --------------------------------------------------------------------
    % 保存
    % ---------------------------------------------------------------------

    output_file = ...
        fullfile( ...
            path, ...
            'single_range.txt');


    writematrix( ...
        range_data, ...
        output_file, ...
        'Delimiter', ...
        ' ');


    fprintf( ...
        '单信标测距数据已写入：%s\n', ...
        output_file);

end