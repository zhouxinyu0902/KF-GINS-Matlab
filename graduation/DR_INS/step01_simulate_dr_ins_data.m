%% 生成无距离辅助的 DR/INS 公共仿真数据
% 输出同一套真值、IMU、DVL 和深度计数据，供路线 A、路线 B 共用。
clear;
close all;
clc;

study_dir = fileparts(mfilename('fullpath'));
repo_root = fileparts(fileparts(study_dir));
addpath(fullfile(repo_root, 'function'));
addpath(genpath(fullfile(repo_root, 'function_zxy')));
addpath(fullfile(repo_root, 'GINS-KF'));
addpath(genpath(fullfile(repo_root, 'psins2401', 'base')));
glvs;

%% 仿真配置
cfg.random_seed = 20260910;
cfg.imu_ts_s = 0.01;          % IMU 100 Hz
cfg.dvl_interval_s = 0.5;     % DVL 2 Hz
cfg.depth_interval_s = 0.01;  % 深度计 100 Hz
cfg.total_duration_s = 3600;
cfg.cruise_speed_mps = 1.0;
cfg.initial_depth_m = 100;
cfg.initial_yaw_deg = 0;

% IMU：imuerrset 的单位依次为 deg/h、ug、deg/sqrt(h)、ug/sqrt(Hz)。
cfg.gyro_bias_dph = 0.01;
cfg.acc_bias_ug = 7;
cfg.gyro_arw_dpsh = 0.0005;
cfg.acc_vrw_ugpsHz = 10;

% DVL：真值先由载体系 RFU 转到 DVL 系，再加入误差。
% cfg.dvl_scale_error = 0.004;
% cfg.dvl_install_pry_deg = [0.0; 0.0; 0.5];
% cfg.dvl_bias_rfu_mps = [0.001; 0.001; 0.000];
% cfg.dvl_noise_std_mps = 0.002;
% cfg.dvl_filter_std_mps = 0.01;
cfg.dvl_scale_error = 0.002;              % 0.2%
cfg.dvl_install_pry_deg = [0; 0; 0.5];   % 0.5°
cfg.dvl_bias_rfu_mps = [0.0005; 0.0005; 0.0005];
cfg.dvl_noise_std_mps = 0.002;
cfg.dvl_filter_std_mps = 0.005;

% 深度计输出为向下为正的 depth。
cfg.depth_bias_m = 0.20;
cfg.depth_noise_std_m = 0.10;
cfg.depth_filter_std_m = 0.15;

% 两条路线使用完全相同的初始导航误差。
cfg.initial_position_error_ned_m = [1; 1; 1];
cfg.initial_velocity_error_ned_mps = [0; 0; 0];
% cfg.initial_attitude_error_rph_deg = [0.01; -0.01; 0.10];
cfg.initial_attitude_error_rph_deg = [0.003; 0.003; 0.023];

rng(cfg.random_seed, 'twister');

%% 生成一小时典型航迹：静止、加速、直航、左右转弯和再次直航
pos0 = glv.pos0;
pos0(3) = -cfg.initial_depth_m;
avp0 = [[0; 0; cfg.initial_yaw_deg * glv.deg]; [0; 0; 0]; pos0];

acceleration_time_s = 20;
acceleration_mps2 = cfg.cruise_speed_mps / acceleration_time_s;
seg = trjsegment([], 'init', 0);
seg = trjsegment(seg, 'uniform', 60);
seg = trjsegment(seg, 'accelerate', acceleration_time_s, [], acceleration_mps2);
% seg = trjsegment(seg, 'uniform', 1000);
% seg = trjsegment(seg, 'turnleft', 90, 1);
% seg = trjsegment(seg, 'uniform', 1000);
% seg = trjsegment(seg, 'turnright', 90, 1);
% seg = trjsegment(seg, 'uniform', 1340);
seg = trjsegment(seg, 'uniform', 3600*4);
trj = trjsimu(avp0, seg.wat, cfg.imu_ts_s, 1);

%% 真值：同时保存原始 PSINS ENU 表达和 KF-GINS NED 表达
truth_pva_ned = avpENU2NED(trj.avp);
truth = struct();
truth.time_s = truth_pva_ned(:, 2);
truth.position_lla = [truth_pva_ned(:, 3:4) * glv.deg, truth_pva_ned(:, 5)];
truth.velocity_ned_mps = truth_pva_ned(:, 6:8);
truth.attitude_rph_rad = truth_pva_ned(:, 9:11) * glv.deg;
truth.avp_psins = trj.avp;
truth.pva_ned = truth_pva_ned;
truth.origin_lla = truth.position_lla(1, :)';

truth_enu_m = pos2dxyz(trj.avp(:, 7:9), trj.avp(1, 7:9)');
truth.position_ned_m = [truth_enu_m(:, 2), truth_enu_m(:, 1), -truth_enu_m(:, 3)];

%% IMU：沿用 PSINS 的 imuerrset、imuadderr 和原有 RFU->FRD 转换
imu_error = imuerrset(cfg.gyro_bias_dph, cfg.acc_bias_ug, ...
    cfg.gyro_arw_dpsh, cfg.acc_vrw_ugpsHz);
imu_noisy_rfu = imuadderr(trj.imu, imu_error);
imu = imuRFU2FRD(imu_noisy_rfu);

%% DVL：1 Hz，输出 DVL 坐标系 RFU 速度
dvl_stride = round(cfg.dvl_interval_s / cfg.imu_ts_s);
if abs(dvl_stride * cfg.imu_ts_s - cfg.dvl_interval_s) > 1e-12
    error('DVL 周期必须是 IMU 周期的整数倍。');
end
dvl_imu_index = (1:dvl_stride:size(imu, 1))';
dvl_count = numel(dvl_imu_index);
dvl_velocity_body_rfu_true = zeros(dvl_count, 3);
dvl_velocity_d_true = zeros(dvl_count, 3);
dvl_velocity_d_meas = zeros(dvl_count, 3);
Cbd_true = a2mat(cfg.dvl_install_pry_deg * glv.deg); % DVL系到RFU载体系

for k = 1:dvl_count
    imu_index = dvl_imu_index(k);
    Ceb = a2mat(trj.avp(imu_index, 1:3));
    velocity_body_rfu = Ceb' * trj.avp(imu_index, 4:6)';
    velocity_d_true = Cbd_true' * velocity_body_rfu;
    velocity_d_meas = (1 + cfg.dvl_scale_error) * velocity_d_true + ...
        cfg.dvl_bias_rfu_mps + cfg.dvl_noise_std_mps * randn(3, 1);

    dvl_velocity_body_rfu_true(k, :) = velocity_body_rfu';
    dvl_velocity_d_true(k, :) = velocity_d_true';
    dvl_velocity_d_meas(k, :) = velocity_d_meas';
end

dvl = struct();
dvl.time_s = imu(dvl_imu_index, 1);
dvl.imu_index = dvl_imu_index;
dvl.velocity_body_rfu_true_mps = dvl_velocity_body_rfu_true;
dvl.velocity_d_true_mps = dvl_velocity_d_true;
dvl.velocity_d_meas_mps = dvl_velocity_d_meas;
dvl.Cbd_true = Cbd_true;
dvl.Cbd_assumed = eye(3);

%% 深度计：100 Hz，与 IMU 同步，向下为正
depth_stride = round(cfg.depth_interval_s / cfg.imu_ts_s);
if abs(depth_stride * cfg.imu_ts_s - cfg.depth_interval_s) > 1e-12
    error('深度计周期必须是 IMU 周期的整数倍。');
end
depth_imu_index = (1:depth_stride:size(imu, 1))';
depth = struct();
depth.time_s = imu(depth_imu_index, 1);
depth.imu_index = depth_imu_index;
depth.true_m = -truth.position_lla(depth_imu_index, 3);
depth.meas_m = depth.true_m + cfg.depth_bias_m + ...
    cfg.depth_noise_std_m * randn(numel(depth_imu_index), 1);

%% 保存 MAT 和沿用原工程格式的 TXT 文件
data_dir = fullfile(study_dir, 'data_no_range');
if ~exist(data_dir, 'dir')
    mkdir(data_dir);
end
save(fullfile(data_dir, 'simulation_data.mat'), ...
    'cfg', 'truth', 'imu', 'dvl', 'depth', 'imu_error', '-v7.3');

writematrix(truth_pva_ned, fullfile(data_dir, 'reference.txt'), 'Delimiter', 'tab');
writematrix(imu, fullfile(data_dir, 'imu.txt'), 'Delimiter', 'tab');
writematrix([dvl.time_s, dvl.velocity_d_meas_mps], ...
    fullfile(data_dir, 'dvl.txt'), 'Delimiter', 'tab');
writematrix([depth.time_s, depth.meas_m], ...
    fullfile(data_dir, 'depth.txt'), 'Delimiter', 'tab');

fprintf('公共仿真数据生成完成：%s\n', fullfile(data_dir, 'simulation_data.mat'));
fprintf(['IMU 历元：%d，DVL 历元：%d，深度计历元：%d，', ...
    '总时长：%.1f s。\n'], size(imu, 1), dvl_count, ...
    numel(depth.time_s), truth.time_s(end) - truth.time_s(1));
