%% 路线 B：DVL 为主、纯 INS 姿态为参考的航位推算
% 不使用距离信息；位置由 DVL 速度经 INS 姿态投影后积分得到。
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

% 沿用 15 状态初始化函数取得与路线 A 相同的初始 INS 导航状态；
% 路线 B 不使用该滤波器，只使用纯 INS 输出的姿态。
nav_cfg = build_nav_config(cfg, truth);
[~, ins_state] = myInitialize_15state(nav_cfg);

compass0 = ned_attitude_to_psins_row(dvl.time_s(1), ins_state.att);
dr_cfg.pos0 = ins_state.pos;
dr_cfg.init_kod = 1.0;
dr_cfg.range_time_tolerance = cfg.dvl_interval_s / 2;
dvl0 = [dvl.time_s(1), dvl.velocity_d_meas_mps(1, :)];
depth0 = [depth.time_s(1), depth.meas_m(1)];
dr = DRInitialize(dr_cfg, dvl0, compass0, depth0);

sample_count = numel(dvl.time_s);
position_lla = zeros(sample_count, 3);
position_ned_m = zeros(sample_count, 3);
velocity_ned_mps = zeros(sample_count, 3);
ins_attitude_rph_rad = zeros(sample_count, 3);

[position_lla(1, :), position_ned_m(1, :), velocity_ned_mps(1, :)] = ...
    export_dr_state(dr, truth.origin_lla);
ins_attitude_rph_rad(1, :) = ins_state.att';

dvl_index = 2;
last_imu = imu(1, :)';
for imu_index = 2:size(imu, 1)
    this_imu = imu(imu_index, :)';
    ins_state = InsMech(ins_state, last_imu, this_imu);
    last_imu = this_imu;

    % 深度计为 100 Hz，逐历元更新 DR 高程；水平位置仍由 2 Hz DVL 推进。
    dr.pos(3) = -depth.meas_m(imu_index);
    dr.avp = [dr.att; dr.vn; dr.pos];

    if dvl_index <= sample_count && imu_index == dvl.imu_index(dvl_index)
        compass_k = ned_attitude_to_psins_row(dvl.time_s(dvl_index), ins_state.att);
        dvl_k = [dvl.time_s(dvl_index), dvl.velocity_d_meas_mps(dvl_index, :)];
        depth_k = [depth.time_s(imu_index), depth.meas_m(imu_index)];
        dr = DRmechanization(dr, dvl_k, compass_k, depth_k);

        [position_lla(dvl_index, :), position_ned_m(dvl_index, :), ...
            velocity_ned_mps(dvl_index, :)] = export_dr_state(dr, truth.origin_lla);
        ins_attitude_rph_rad(dvl_index, :) = ins_state.att';
        dvl_index = dvl_index + 1;
    end
end

if dvl_index ~= sample_count + 1
    error('路线 B 未处理完全部 DVL 历元。');
end

route_b = struct();
route_b.name = 'Route B: DVL DR with pure-INS attitude';
route_b.time_s = dvl.time_s;
route_b.position_lla = position_lla;
route_b.position_ned_m = position_ned_m;
route_b.velocity_ned_mps = velocity_ned_mps;
route_b.attitude_rph_rad = ins_attitude_rph_rad;

output_dir = fullfile(study_dir, 'output_no_range');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
save(fullfile(output_dir, 'route_b_result.mat'), 'route_b', 'cfg', '-v7.3');
writematrix([route_b.time_s, route_b.position_ned_m, ...
    route_b.velocity_ned_mps, route_b.attitude_rph_rad], ...
    fullfile(output_dir, 'route_b_result.txt'), 'Delimiter', 'tab');
fprintf('路线 B 推算完成：%s\n', fullfile(output_dir, 'route_b_result.mat'));


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


function compass_row = ned_attitude_to_psins_row(time_s, attitude_rph)
% KF-GINS 为 [roll pitch clockwise-heading]；PSINS DR 为
% [pitch roll counter-clockwise-yaw]。
    yaw_psins = yawcvt(attitude_rph(3), 'c360cc180');
    compass_row = [time_s, attitude_rph(2), attitude_rph(1), yaw_psins];
end


function [position_lla, position_ned, velocity_ned] = ...
        export_dr_state(dr, origin_lla)
    enu = pos2dxyz(dr.pos', origin_lla);
    position_lla = dr.pos';
    position_ned = [enu(2), enu(1), -enu(3)];
    velocity_ned = [dr.vn(2), dr.vn(1), -dr.vn(3)];
end
