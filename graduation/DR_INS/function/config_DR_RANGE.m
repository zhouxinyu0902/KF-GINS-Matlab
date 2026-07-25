function cfg = config_DR_RANGE(in_dir)
%CONFIG_DR_RANGE  DR/DVL/Compass/Range组合导航配置
%
% 输入文件默认来自前面生成的数据目录：
%   dvl.txt       : t vb_x vb_y vb_z
%   compass.txt   : t pitch roll yaw            rad
%   depth.txt     : t depth                     m, D向下为正
%   beacon.txt    : t slant horizontal lat lon h, rad rad m
%   reference.txt : t N E D pitch roll yaw vN vE vD，可选
%
% 内部导航状态采用PSINS常用形式：
%   pos = [lat; lon; h]，rad rad m，高度h向上为正
%   vn  = [VE; VN; VU]，m/s
%   att = [pitch; roll; yaw]，rad
    glvs
    if nargin < 1 || isempty(in_dir)
        in_dir = 'D:\Github\KF-GINS-Matlab\graduation\DR_INS\input\data_lawnmower_single_side';
    end

    cfg = struct();
    cfg.inputfolder  = char(in_dir);
    cfg.outputfolder = fullfile(cfg.inputfolder, 'output_DR_RANGE');

    cfg.dvlpath     = fullfile(cfg.inputfolder, 'dvl.txt');
    cfg.compasspath = fullfile(cfg.inputfolder, 'compass.txt');
    cfg.depthpath   = fullfile(cfg.inputfolder, 'depth.txt');
    cfg.beaconpath  = fullfile(cfg.inputfolder, 'beacon.txt');
    for i=1:4
        cfg.beaconpath1{i}  = fullfile(cfg.inputfolder, sprintf('beacon_%d.txt',i));
    end
    cfg.truthpath   = fullfile(cfg.inputfolder, 'reference.txt');

    % 与数据生成脚本保持一致。若你改过生成脚本中的初始经纬度/深度，这里也要同步修改。
    deg = pi/180;
    cfg.lat0 = 30 * deg + 1/glv.Re;
    cfg.lon0 = 120 * deg + 1/glv.Re;
    cfg.depth0 = 3893.066 + 1;        % m, D向下为正
    cfg.pos0 = [cfg.lat0; cfg.lon0; -cfg.depth0];

    % NED输出所用局部原点。这里与数据生成脚本的 cfg.ned_origin = [lat0; lon0; 0] 保持一致。
    cfg.ned_origin = [cfg.lat0; cfg.lon0; 0];

    % 处理时间。inf表示自动使用数据重叠区间。
    cfg.starttime = -inf;
    cfg.endtime   = inf;

    % 量测时间对齐容差。距离观测时刻落在当前DR历元附近时触发更新。
    cfg.range_time_tolerance = 0.25;   % s，若DVL采样0.5 s，取0.25较合适

    % 量测噪声
    cfg.range_std = 5.0;               % m，对应beacon.txt第二列斜距量测

    % 滤波器状态维数。第一版建议4维：[dK; dYaw; dLat; dLon]
    cfg.kf.state_dim = 4;

    % 初始状态标准差
    cfg.kf.init_dk_std       = 0.01;        % DVL刻度因子误差，无量纲
    cfg.kf.init_yaw_std      = 1 * deg;     % 航向误差，rad
    cfg.kf.init_pos_std_m    = 1;          % 水平位置误差，m

    % 连续过程噪声标准差。这里是比较温和的默认值，后续可根据仿真调参。
    % cfg.kf.q_dk      = 1e-5;          % 1/sqrt(s)
    % cfg.kf.q_yaw     = 0.0005 * deg;   % rad/sqrt(s)
    % cfg.kf.q_pos_m   = 0.02;          % m/sqrt(s)

    cfg.kf.q_dk      = 1e-5;          % 1/sqrt(s)
    cfg.kf.q_yaw     = 0.05 * deg;   % rad/sqrt(s)
    cfg.kf.q_pos_m   = 0;          % m/sqrt(s)
    % 是否使用新息门限。第一版默认关闭，便于调试。
    cfg.kf.use_gate = false;
    cfg.kf.gate_sigma = 5;

    % DR主状态初始DVL刻度因子。DVL量测速度会除以 dr.kod。
    cfg.init_kod = 1.0;

    % 反馈设置。
    % 第一版建议只反馈位置；dK和dYaw作为滤波状态保留，用于观察可观测性。
    cfg.feedback.position = true;
    cfg.feedback.dvl_scale = false;
    cfg.feedback.yaw = false;
    cfg.feedback.coef = 1.0;

    % 输出格式：导航结果按deg输出经纬度和姿态，速度保持m/s。
    cfg.output_in_degree = true;
end
