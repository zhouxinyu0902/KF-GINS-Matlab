%% 路线 A：INS 主推算，DVL 与深度计作为 Kalman 量测
% 不使用距离信息。沿用 InsMech、15 状态传播和闭环反馈结构。
clear;
close all;
clc;

study_dir = fileparts(mfilename('fullpath'));
repo_root = fileparts(fileparts(study_dir));
addpath(study_dir);
addpath(fullfile(study_dir, 'function'));
addpath(fullfile(repo_root, 'function'));
addpath(genpath(fullfile(repo_root, 'function_zxy')));
addpath(fullfile(repo_root, 'GINS-KF'));
addpath(genpath(fullfile(repo_root, 'psins2401', 'base')));
addpath(fullfile(repo_root, '惯导实验研究', 'algorithm-exploration', ...
    'functions', 'experiment'));
glvs;

data_file = fullfile(study_dir, 'data_no_range', 'simulation_data.mat');
if ~isfile(data_file)
    error('缺少仿真数据，请先运行 simulate_dr_ins_data.m。');
end
data = load(data_file, 'cfg', 'truth', 'imu', 'dvl', 'depth');
cfg = data.cfg;
truth = data.truth;
imu = data.imu;
dvl = data.dvl;
depth = data.depth;

nav_cfg = build_nav_config(cfg, truth);
[kf, navstate] = myInitialize_15state(nav_cfg);
kf.depthstd = cfg.depth_filter_std_m;

sample_count = numel(dvl.time_s);
position_lla = zeros(sample_count, 3);
position_ned_m = zeros(sample_count, 3);
velocity_ned_mps = zeros(sample_count, 3);
attitude_rph_rad = zeros(sample_count, 3);
state_std = zeros(sample_count, 15);

[position_lla(1, :), position_ned_m(1, :)] = ...
    export_ins_position(navstate, truth.origin_lla);
velocity_ned_mps(1, :) = navstate.vel';
attitude_rph_rad(1, :) = navstate.att';
state_std(1, :) = sqrt(max(diag(kf.P), 0))';

dvl_index = 2;
last_imu_corrected = compensate_imu(imu(1, :)', navstate, cfg.imu_ts_s);

for imu_index = 2:size(imu, 1)
    laststate = navstate;
    this_imu_raw = imu(imu_index, :)';
    dt = this_imu_raw(1) - imu(imu_index - 1, 1);
    this_imu_corrected = compensate_imu(this_imu_raw, navstate, dt);

    navstate = InsMech(laststate, last_imu_corrected, this_imu_corrected);
    kf = myInsPropagate_15state(navstate, this_imu_corrected, dt, kf);

    % 深度计 100 Hz：沿用原高度更新接口，但显式执行 D=-h 的符号转换。
    height_data = [depth.time_s(imu_index), -depth.meas_m(imu_index)];
    kf = myHeightUpdate(navstate, height_data, kf);
    navstate.pos(3) = navstate.pos(3) - kf.x(3);
    navstate.vel(3) = navstate.vel(3) - kf.x(6);
    kf.x(3) = 0;
    kf.x(6) = 0;

    if dvl_index <= sample_count && imu_index == dvl.imu_index(dvl_index)
        kf = dvl_velocity_update(navstate, ...
            dvl.velocity_d_meas_mps(dvl_index, :)', ...
            cfg.dvl_filter_std_mps, kf);

        [kf, navstate] = myErrorFeedback_15state(kf, navstate);

        [position_lla(dvl_index, :), position_ned_m(dvl_index, :)] = ...
            export_ins_position(navstate, truth.origin_lla);
        velocity_ned_mps(dvl_index, :) = navstate.vel';
        attitude_rph_rad(dvl_index, :) = navstate.att';
        state_std(dvl_index, :) = sqrt(max(diag(kf.P), 0))';
        dvl_index = dvl_index + 1;
    end

    last_imu_corrected = this_imu_corrected;
end

if dvl_index ~= sample_count + 1
    error('路线 A 未处理完全部 DVL 历元。');
end

route_a = struct();
route_a.name = 'Route A: INS/DVL/depth 15-state EKF';
route_a.time_s = dvl.time_s;
route_a.position_lla = position_lla;
route_a.position_ned_m = position_ned_m;
route_a.velocity_ned_mps = velocity_ned_mps;
route_a.attitude_rph_rad = attitude_rph_rad;
route_a.state_std = state_std;

output_dir = fullfile(study_dir, 'output_no_range');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
save(fullfile(output_dir, 'route_a_result.mat'), 'route_a', 'cfg', '-v7.3');
writematrix([route_a.time_s, route_a.position_ned_m, ...
    route_a.velocity_ned_mps, route_a.attitude_rph_rad], ...
    fullfile(output_dir, 'route_a_result.txt'), 'Delimiter', 'tab');
fprintf('路线 A 推算完成：%s\n', fullfile(output_dir, 'route_a_result.mat'));


function kf = dvl_velocity_update(navstate, velocity_d_rfu, velocity_std, kf)
% 第一阶段假设安装矩阵已知为单位阵，将 DVL 系数据按名义安装关系
% 转为 FRD 载体系，再用当前 INS 姿态投影到 NED 导航系。
    velocity_body_frd = [velocity_d_rfu(2); velocity_d_rfu(1); -velocity_d_rfu(3)];
    velocity_dvl_ned = navstate.cbn * velocity_body_frd;
    innovation = navstate.vel - velocity_dvl_ned;

    H = zeros(3, kf.RANK);
    H(:, 4:6) = eye(3);
    R = velocity_std^2 * eye(3);
    K = kf.P * H' / (H * kf.P * H' + R);
    kf.x = kf.x + K * (innovation - H * kf.x);
    I = eye(kf.RANK);
    kf.P = (I - K * H) * kf.P * (I - K * H)' + K * R * K';
end


function imu_corrected = compensate_imu(imu_raw, navstate, dt)
    imu_corrected = imu_raw;
    imu_corrected(2:4) = (imu_raw(2:4) - dt * navstate.gyrbias) ./ ...
        (ones(3, 1) + navstate.gyrscale);
    imu_corrected(5:7) = (imu_raw(5:7) - dt * navstate.accbias) ./ ...
        (ones(3, 1) + navstate.accscale);
end


function nav_cfg = build_nav_config(cfg, truth)
    param = Param();
    nav_cfg.starttime = truth.time_s(1);
    nav_cfg.initpos = truth.position_lla(1, :)';
    [rm, rn] = getRmRn(nav_cfg.initpos(1), param);
    DR = diag([rm + nav_cfg.initpos(3), ...
        (rn + nav_cfg.initpos(3)) * cos(nav_cfg.initpos(1)), -1]);
    nav_cfg.initpos = nav_cfg.initpos + DR \ cfg.initial_position_error_ned_m;
    nav_cfg.initvel = truth.velocity_ned_mps(1, :)' + ...
        cfg.initial_velocity_error_ned_mps;
    nav_cfg.initatt = truth.attitude_rph_rad(1, :)' + ...
        cfg.initial_attitude_error_rph_deg * param.D2R;
    nav_cfg.initgyrbias = zeros(3, 1);
    nav_cfg.initaccbias = zeros(3, 1);
    nav_cfg.initgyrscale = zeros(3, 1);
    nav_cfg.initaccscale = zeros(3, 1);
    nav_cfg.initposstd = DR \ [1; 1; 1];
    nav_cfg.initvelstd = [0.05; 0.05; 0.05];
    nav_cfg.initattstd = [0.05; 0.05; 0.20] * param.D2R;
    nav_cfg.initgyrbiasstd = ones(3, 1) * cfg.gyro_bias_dph * param.D2R / 3600;
    nav_cfg.initaccbiasstd = ones(3, 1) * cfg.acc_bias_ug * 1e-5;
    nav_cfg.gyrarw = cfg.gyro_arw_dpsh * param.D2R / 60;
    nav_cfg.accvrw = cfg.acc_vrw_ugpsHz * 1e-5;
    nav_cfg.gyrbiasstd = cfg.gyro_bias_dph * param.D2R / 3600;
    nav_cfg.accbiasstd = cfg.acc_bias_ug * 1e-5;
    nav_cfg.corrtime = 3600;
end


function [position_lla, position_ned] = export_ins_position(navstate, origin_lla)
    enu = pos2dxyz(navstate.pos', origin_lla);
    position_lla = navstate.pos';
    position_ned = [enu(2), enu(1), -enu(3)];
end
