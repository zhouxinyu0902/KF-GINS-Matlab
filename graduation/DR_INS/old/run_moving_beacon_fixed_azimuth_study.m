%% 同步移动信标固定方位角辅助 INS/DVL 实验
% 信标相对载体真值轨迹保持固定水平距离和固定方位角，并与导航轨迹
% 时间同步。比较无距离基线及 50/95/160/275 deg 四个典型方位。
%
% 更新方式与 run_route_a_joint_dvl_depth_range.m 一致：
%   普通历元：INS 推算 + 解耦深度更新；
%   DVL 历元：深度 + DVL 四维联合更新；
%   DVL 与距离同时到达：深度 + 水平距离 + DVL 五维联合更新。

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

%% 公共数据与实验配置
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

if numel(depth.time_s) ~= size(imu, 1) || ...
        any(depth.imu_index(:) ~= (1:size(imu, 1))')
    error('本脚本要求深度计与 100 Hz IMU 逐历元同步。');
end

study_cfg.beacon_azimuth_deg = [50, 95, 160, 275];
study_cfg.range_interval_s = 8.0;
study_cfg.range_noise_std_m = 5.0;
study_cfg.range_filter_std_m = 5.0;
study_cfg.range_noise_seed = cfg.random_seed + 2800;
study_cfg.radial_margin_m = 500.0;
study_cfg.beacon_depth_offset_m = 50.0;

truth_output_ned_m = truth.position_ned_m(dvl.imu_index, :);
north_limits_m = [min(truth.position_ned_m(:, 1)), ...
    max(truth.position_ned_m(:, 1))];
east_limits_m = [min(truth.position_ned_m(:, 2)), ...
    max(truth.position_ned_m(:, 2))];
scan_center_ne_m = [mean(north_limits_m), mean(east_limits_m)];
distance_from_center_m = vecnorm( ...
    truth.position_ned_m(:, 1:2) - scan_center_ne_m, 2, 2);
study_cfg.horizontal_range_m = max(distance_from_center_m) + ...
    study_cfg.radial_margin_m;

range_stride = round(study_cfg.range_interval_s / cfg.imu_ts_s);
if abs(range_stride * cfg.imu_ts_s - study_cfg.range_interval_s) > 1e-12
    error('水平距离更新间隔必须是 IMU 周期的整数倍。');
end
range_imu_index = (1 + range_stride:range_stride:size(imu, 1))';
[is_matched, range_dvl_index] = ismember(range_imu_index, dvl.imu_index);
if ~all(is_matched)
    error('存在没有同步 DVL 的水平距离历元。');
end
range_time_s = imu(range_imu_index, 1);
range_count = numel(range_time_s);

%% 生成与载体同步移动的四条信标轨迹
azimuth_deg = study_cfg.beacon_azimuth_deg(:);
azimuth_count = numel(azimuth_deg);
beacon_trajectory_ned_m = zeros(output_count_from_dvl(dvl), 3, azimuth_count);
beacon_range_lla = zeros(range_count, 3, azimuth_count);
for azimuth_index = 1:azimuth_count
    horizontal_offset_ne_m = study_cfg.horizontal_range_m * ...
        [cosd(azimuth_deg(azimuth_index)), sind(azimuth_deg(azimuth_index))];
    beacon_trajectory_ned_m(:, :, azimuth_index) = ...
        truth_output_ned_m + [horizontal_offset_ne_m, ...
        study_cfg.beacon_depth_offset_m];

    beacon_range_ned_m = beacon_trajectory_ned_m( ...
        range_dvl_index, :, azimuth_index);
    beacon_range_enu_m = [beacon_range_ned_m(:, 2), ...
        beacon_range_ned_m(:, 1), -beacon_range_ned_m(:, 3)];
    beacon_range_lla(:, :, azimuth_index) = ...
        dxyz2pos(beacon_range_enu_m, truth.origin_lla);
end

range_true_m = repmat(study_cfg.horizontal_range_m, range_count, 1);
rng(study_cfg.range_noise_seed, 'twister');
common_range_noise_m = study_cfg.range_noise_std_m * randn(range_count, 1);
range_meas_m = range_true_m + common_range_noise_m;

%% 初始化无距离基线和四个固定方位角方案
case_names = ["INS-DVL-Depth-Joint"; ...
    compose("INS-DVL-HRange-%03ddeg", azimuth_deg)];
case_count = numel(case_names);
case_azimuth_deg = [nan; azimuth_deg];
use_range = [false; true(azimuth_count, 1)];

nav_cfg = build_nav_config(cfg, truth);
kfs = cell(1, case_count);
navstates = cell(1, case_count);
last_imu_corrected = cell(1, case_count);

output_count = numel(dvl.time_s);
position_lla = cell(1, case_count);
position_ned_m = cell(1, case_count);
velocity_ned_mps = cell(1, case_count);
attitude_rph_rad = cell(1, case_count);
state_std = cell(1, case_count);
range_innovation_m = nan(range_count, azimuth_count);

% 首个 DVL/深度历元采用四维联合更新；首次距离量测从约 8 s 开始。
for case_index = 1:case_count
    [kfs{case_index}, navstates{case_index}] = myInitialize_15state(nav_cfg);
    kfs{case_index}.depthstd = cfg.depth_filter_std_m;
    [kfs{case_index}, ~] = joint_aiding_update( ...
        navstates{case_index}, depth.meas_m(1), ...
        dvl.velocity_d_meas_mps(1, :)', cfg.depth_filter_std_m, ...
        cfg.dvl_filter_std_mps, false, nan(3, 1), nan, ...
        study_cfg.range_filter_std_m, kfs{case_index});
    [kfs{case_index}, navstates{case_index}] = ...
        myErrorFeedback_15state(kfs{case_index}, navstates{case_index});

    position_lla{case_index} = zeros(output_count, 3);
    position_ned_m{case_index} = zeros(output_count, 3);
    velocity_ned_mps{case_index} = zeros(output_count, 3);
    attitude_rph_rad{case_index} = zeros(output_count, 3);
    state_std{case_index} = zeros(output_count, 15);
    [position_lla{case_index}(1, :), position_ned_m{case_index}(1, :)] = ...
        export_ins_position(navstates{case_index}, truth.origin_lla);
    velocity_ned_mps{case_index}(1, :) = navstates{case_index}.vel';
    attitude_rph_rad{case_index}(1, :) = navstates{case_index}.att';
    state_std{case_index}(1, :) = sqrt(max(diag(kfs{case_index}.P), 0))';
    last_imu_corrected{case_index} = compensate_imu( ...
        imu(1, :)', navstates{case_index}, cfg.imu_ts_s);
end

%% 五套滤波器并行推算
dvl_index = 2;
range_index = 1;
fprintf(['开始同步移动信标固定方位角实验：%d 个方位，', ...
    '水平距离 %.1f m，更新间隔 %.1f s。\n'], ...
    azimuth_count, study_cfg.horizontal_range_m, ...
    study_cfg.range_interval_s);

for imu_index = 2:size(imu, 1)
    is_dvl_epoch = dvl_index <= output_count && ...
        imu_index == dvl.imu_index(dvl_index);
    is_range_epoch = range_index <= range_count && ...
        imu_index == range_imu_index(range_index);
    this_imu_raw = imu(imu_index, :)';
    dt = this_imu_raw(1) - imu(imu_index - 1, 1);

    for case_index = 1:case_count
        laststate = navstates{case_index};
        this_imu_corrected = compensate_imu( ...
            this_imu_raw, navstates{case_index}, dt);
        navstates{case_index} = InsMech(laststate, ...
            last_imu_corrected{case_index}, this_imu_corrected);
        kfs{case_index} = myInsPropagate_15state( ...
            navstates{case_index}, this_imu_corrected, dt, kfs{case_index});

        if is_dvl_epoch
            include_range = use_range(case_index) && is_range_epoch;
            if include_range
                azimuth_index = case_index - 1;
                beacon_lla = beacon_range_lla( ...
                    range_index, :, azimuth_index)';
                measured_range_m = range_meas_m(range_index);
            else
                beacon_lla = nan(3, 1);
                measured_range_m = nan;
            end

            [kfs{case_index}, innovation] = joint_aiding_update( ...
                navstates{case_index}, depth.meas_m(imu_index), ...
                dvl.velocity_d_meas_mps(dvl_index, :)', ...
                cfg.depth_filter_std_m, cfg.dvl_filter_std_mps, ...
                include_range, beacon_lla, measured_range_m, ...
                study_cfg.range_filter_std_m, kfs{case_index});
            [kfs{case_index}, navstates{case_index}] = ...
                myErrorFeedback_15state(kfs{case_index}, ...
                navstates{case_index});

            if include_range
                range_innovation_m(range_index, azimuth_index) = ...
                    innovation.range_m;
            end
        else
            height_data = [depth.time_s(imu_index), -depth.meas_m(imu_index)];
            kfs{case_index} = myHeightUpdate( ...
                navstates{case_index}, height_data, kfs{case_index});
            navstates{case_index}.pos(3) = navstates{case_index}.pos(3) - ...
                kfs{case_index}.x(3);
            navstates{case_index}.vel(3) = navstates{case_index}.vel(3) - ...
                kfs{case_index}.x(6);
            kfs{case_index}.x(3) = 0;
            kfs{case_index}.x(6) = 0;
        end
        last_imu_corrected{case_index} = this_imu_corrected;
    end

    if is_dvl_epoch
        for case_index = 1:case_count
            [position_lla{case_index}(dvl_index, :), ...
                position_ned_m{case_index}(dvl_index, :)] = ...
                export_ins_position(navstates{case_index}, truth.origin_lla);
            velocity_ned_mps{case_index}(dvl_index, :) = ...
                navstates{case_index}.vel';
            attitude_rph_rad{case_index}(dvl_index, :) = ...
                navstates{case_index}.att';
            state_std{case_index}(dvl_index, :) = ...
                sqrt(max(diag(kfs{case_index}.P), 0))';
        end
        dvl_index = dvl_index + 1;
    end
    if is_range_epoch
        range_index = range_index + 1;
    end
end

if dvl_index ~= output_count + 1 || range_index ~= range_count + 1
    error('存在未处理完的 DVL 或水平距离历元。');
end

%% 保存 .nav、信标轨迹和完整状态结果
output_dir = fullfile(study_dir, 'output_moving_beacon_fixed_azimuth');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

result_template = struct('name', '', 'azimuth_deg', nan, 'time_s', [], ...
    'position_lla', [], 'position_ned_m', [], 'velocity_ned_mps', [], ...
    'attitude_rph_rad', [], 'state_std', []);
results = repmat(result_template, 1, case_count);
nav_paths = cell(1, case_count);
for case_index = 1:case_count
    results(case_index).name = char(case_names(case_index));
    results(case_index).azimuth_deg = case_azimuth_deg(case_index);
    results(case_index).time_s = dvl.time_s;
    results(case_index).position_lla = position_lla{case_index};
    results(case_index).position_ned_m = position_ned_m{case_index};
    results(case_index).velocity_ned_mps = velocity_ned_mps{case_index};
    results(case_index).attitude_rph_rad = attitude_rph_rad{case_index};
    results(case_index).state_std = state_std{case_index};

    nav_matrix = make_nav_matrix((0:output_count-1)', dvl.time_s, ...
        position_lla{case_index}, velocity_ned_mps{case_index}, ...
        attitude_rph_rad{case_index});
    nav_paths{case_index} = fullfile(output_dir, ...
        [char(case_names(case_index)), '.nav']);
    writematrix(nav_matrix, nav_paths{case_index}, ...
        'FileType', 'text', 'Delimiter', 'tab');
end

beacon_log = make_beacon_log(range_time_s, azimuth_deg, ...
    beacon_trajectory_ned_m(range_dvl_index, :, :), ...
    range_true_m, range_meas_m);
writetable(beacon_log, fullfile(output_dir, ...
    'moving_beacon_trajectories.csv'));

innovation_table = array2table([range_time_s, range_innovation_m], ...
    'VariableNames', [{'Time_s'}, cellstr(compose( ...
    'Innovation_%03ddeg_m', azimuth_deg))']);
writetable(innovation_table, fullfile(output_dir, ...
    'range_innovations.csv'));

%% 误差评估
truth_output_velocity_ned_mps = truth.velocity_ned_mps(dvl.imu_index, :);
route_distance_m = sum(vecnorm(diff( ...
    truth.position_ned_m(:, 1:2), 1, 1), 2, 2));

system_name = case_names;
final_horizontal_m = zeros(case_count, 1);
max_horizontal_m = zeros(case_count, 1);
rms_horizontal_m = zeros(case_count, 1);
mean_horizontal_m = zeros(case_count, 1);
p95_horizontal_m = zeros(case_count, 1);
route_error_percent = zeros(case_count, 1);
rms_vertical_m = zeros(case_count, 1);
rms_velocity_mps = zeros(case_count, 1);

for case_index = 1:case_count
    position_error_ned_m = position_ned_m{case_index} - truth_output_ned_m;
    horizontal_error_m = vecnorm(position_error_ned_m(:, 1:2), 2, 2);
    velocity_error_mps = velocity_ned_mps{case_index} - ...
        truth_output_velocity_ned_mps;
    final_horizontal_m(case_index) = horizontal_error_m(end);
    max_horizontal_m(case_index) = max(horizontal_error_m);
    rms_horizontal_m(case_index) = sqrt(mean(horizontal_error_m.^2));
    mean_horizontal_m(case_index) = mean(horizontal_error_m);
    p95_horizontal_m(case_index) = prctile(horizontal_error_m, 95);
    route_error_percent(case_index) = 100 * horizontal_error_m(end) / ...
        route_distance_m;
    rms_vertical_m(case_index) = sqrt(mean(position_error_ned_m(:, 3).^2));
    rms_velocity_mps(case_index) = sqrt(mean(sum(velocity_error_mps.^2, 2)));
end

rms_improvement_percent = 100 * (rms_horizontal_m(1) - ...
    rms_horizontal_m) / rms_horizontal_m(1);
final_improvement_percent = 100 * (final_horizontal_m(1) - ...
    final_horizontal_m) / final_horizontal_m(1);

error_summary = table(system_name, case_azimuth_deg, ...
    repmat(route_distance_m, case_count, 1), final_horizontal_m, ...
    max_horizontal_m, rms_horizontal_m, mean_horizontal_m, ...
    p95_horizontal_m, route_error_percent, rms_vertical_m, ...
    rms_velocity_mps, rms_improvement_percent, final_improvement_percent, ...
    'VariableNames', {'System', 'BeaconAzimuth_deg', 'RouteDistance_m', ...
    'FinalHorizontalError_m', 'MaxHorizontalError_m', ...
    'RMSHorizontalError_m', 'MeanHorizontalError_m', ...
    'P95HorizontalError_m', 'RouteError_percent', ...
    'RMSVerticalError_m', 'RMSVelocityError_mps', ...
    'RMSImprovement_percent', 'FinalImprovement_percent'});
writetable(error_summary, fullfile(output_dir, 'error_summary.csv'));
writetable(error_summary, fullfile(output_dir, 'error_summary.xlsx'));

save(fullfile(output_dir, 'moving_beacon_fixed_azimuth_result.mat'), ...
    'results', 'study_cfg', 'azimuth_deg', 'range_time_s', ...
    'range_true_m', 'range_meas_m', 'range_innovation_m', ...
    'beacon_trajectory_ned_m', 'beacon_range_lla', ...
    'error_summary', '-v7.3');

%% 绘图
truth_path = fullfile(study_dir, 'data_no_range', 'reference.txt');
[radial_fig, radial_statistics] = calc_radial_error_gjb( ...
    truth_path, nav_paths{:}, false);
exportgraphics(radial_fig, fullfile(output_dir, ...
    '01_radial_error_comparison.png'), 'Resolution', 200);
writecell(radial_statistics, fullfile(output_dir, ...
    'radial_error_statistics.xlsx'));

effect_fig = figure('Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact');
nexttile;
plot(azimuth_deg, rms_horizontal_m(2:end), 'o-', 'LineWidth', 1.3);
yline(rms_horizontal_m(1), 'k--', 'No-range baseline');
grid on;
ylabel('Horizontal RMS (m)');
nexttile;
plot(azimuth_deg, final_horizontal_m(2:end), 'o-', 'LineWidth', 1.3);
yline(final_horizontal_m(1), 'k--', 'No-range baseline');
grid on;
ylabel('Final error (m)');
nexttile;
plot(azimuth_deg, route_error_percent(2:end), 'o-', 'LineWidth', 1.3);
yline(route_error_percent(1), 'k--', 'No-range baseline');
grid on;
xlabel('Fixed beacon azimuth (deg)');
ylabel('Route error (%)');
exportgraphics(effect_fig, fullfile(output_dir, ...
    '02_aiding_effect_vs_azimuth.png'), 'Resolution', 200);

geometry_fig = figure('Color', 'w');
plot(truth_output_ned_m(:, 2), truth_output_ned_m(:, 1), ...
    'k-', 'LineWidth', 1.6, 'DisplayName', 'Vehicle truth');
hold on;
colors = lines(azimuth_count);
for azimuth_index = 1:azimuth_count
    beacon_track = beacon_trajectory_ned_m(:, :, azimuth_index);
    plot(beacon_track(:, 2), beacon_track(:, 1), '--', ...
        'LineWidth', 1.1, 'Color', colors(azimuth_index, :), ...
        'DisplayName', sprintf('Beacon %.0f deg', ...
        azimuth_deg(azimuth_index)));
end
axis equal;
grid on;
box on;
xlabel('East (m)');
ylabel('North (m)');
title(sprintf('Synchronized moving beacons, fixed range %.1f m', ...
    study_cfg.horizontal_range_m));
legend('Location', 'best');
exportgraphics(geometry_fig, fullfile(output_dir, ...
    '03_synchronized_beacon_trajectories.png'), 'Resolution', 200);

innovation_fig = figure('Color', 'w');
for azimuth_index = 1:azimuth_count
    plot(range_time_s / 60, range_innovation_m(:, azimuth_index), ...
        'LineWidth', 1.0, 'DisplayName', sprintf('%.0f deg', ...
        azimuth_deg(azimuth_index)));
    hold on;
end
yline(study_cfg.range_noise_std_m, 'k--', '5 m', ...
    'HandleVisibility', 'off');
yline(-study_cfg.range_noise_std_m, 'k--', '-5 m', ...
    'HandleVisibility', 'off');
grid on;
box on;
xlabel('Time (min)');
ylabel('Prefit horizontal-range innovation (m)');
legend('Location', 'best');
exportgraphics(innovation_fig, fullfile(output_dir, ...
    '04_range_innovations.png'), 'Resolution', 200);

fprintf('\n同步移动信标固定方位角实验完成：%s\n', output_dir);
fprintf('水平距离 %.1f m，距离更新 %d 次，公共噪声标准差 %.1f m。\n', ...
    study_cfg.horizontal_range_m, range_count, ...
    study_cfg.range_noise_std_m);
disp(error_summary(:, {'System', 'BeaconAzimuth_deg', ...
    'FinalHorizontalError_m', 'RMSHorizontalError_m', ...
    'RouteError_percent', 'RMSImprovement_percent'}));


function count = output_count_from_dvl(dvl)
    count = numel(dvl.time_s);
end


function [kf, innovation] = joint_aiding_update(navstate, depth_meas_m, ...
    velocity_d_rfu, depth_std_m, dvl_std_mps, include_range, ...
    beacon_lla, measured_range_m, range_std_m, kf)
% 将同一历元的深度、可选水平距离和 DVL 堆叠后只更新一次。
    velocity_body_frd = [velocity_d_rfu(2); velocity_d_rfu(1); ...
        -velocity_d_rfu(3)];
    velocity_dvl_ned = navstate.cbn * velocity_body_frd;
    depth_residual_m = navstate.pos(3) - (-depth_meas_m);
    dvl_residual_mps = navstate.vel - velocity_dvl_ned;

    if include_range
        [range_residual_m, range_H_position] = ...
            horizontal_range_residual(navstate, beacon_lla, measured_range_m);
        Z = [depth_residual_m; range_residual_m; dvl_residual_mps];
        H = zeros(5, kf.RANK);
        H(1, 3) = 1;
        H(2, 1:2) = range_H_position;
        H(3:5, 4:6) = eye(3);
        R = diag([depth_std_m^2; range_std_m^2; ...
            repmat(dvl_std_mps^2, 3, 1)]);
    else
        range_residual_m = nan;
        Z = [depth_residual_m; dvl_residual_mps];
        H = zeros(4, kf.RANK);
        H(1, 3) = 1;
        H(2:4, 4:6) = eye(3);
        R = diag([depth_std_m^2; repmat(dvl_std_mps^2, 3, 1)]);
    end

    K = kf.P * H' / (H * kf.P * H' + R);
    kf.x = kf.x + K * (Z - H * kf.x);
    I = eye(kf.RANK);
    kf.P = (I - K * H) * kf.P * (I - K * H)' + K * R * K';

    innovation.depth_m = depth_residual_m;
    innovation.range_m = range_residual_m;
    innovation.dvl_mps = dvl_residual_mps;
end


function [residual_m, H_position] = horizontal_range_residual( ...
    navstate, beacon_lla, measured_range_m)
    param = Param();
    [rm, rn] = getRmRn(beacon_lla(1), param);
    DR_horizontal = diag([rm + beacon_lla(3), ...
        (rn + beacon_lla(3)) * cos(beacon_lla(1))]);
    delta_horizontal_m = DR_horizontal * ...
        (navstate.pos(1:2) - beacon_lla(1:2));
    predicted_range_m = norm(delta_horizontal_m);
    if predicted_range_m < 1e-6
        error('载体与信标水平距离过小，无法构造距离量测雅可比。');
    end
    residual_m = predicted_range_m - measured_range_m;
    H_position = (delta_horizontal_m' / predicted_range_m) * ...
        DR_horizontal;
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
    nav_cfg.initpos = nav_cfg.initpos + DR \ ...
        cfg.initial_position_error_ned_m;
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
    nav_cfg.initgyrbiasstd = ones(3, 1) * cfg.gyro_bias_dph * ...
        param.D2R / 3600;
    nav_cfg.initaccbiasstd = ones(3, 1) * cfg.acc_bias_ug * 1e-5;
    nav_cfg.gyrarw = cfg.gyro_arw_dpsh * param.D2R / 60;
    nav_cfg.accvrw = cfg.acc_vrw_ugpsHz * 1e-5;
    nav_cfg.gyrbiasstd = cfg.gyro_bias_dph * param.D2R / 3600;
    nav_cfg.accbiasstd = cfg.acc_bias_ug * 1e-5;
    nav_cfg.corrtime = 3600;
end


function [position_lla, position_ned] = export_ins_position( ...
    navstate, origin_lla)
    enu = pos2dxyz(navstate.pos', origin_lla);
    position_lla = navstate.pos';
    position_ned = [enu(2), enu(1), -enu(3)];
end


function nav_matrix = make_nav_matrix(id, time_s, position_lla, ...
    velocity_ned_mps, attitude_rph_rad)
    radians_to_degrees = 180 / pi;
    nav_matrix = [id, time_s, ...
        position_lla(:, 1:2) * radians_to_degrees, position_lla(:, 3), ...
        velocity_ned_mps, attitude_rph_rad * radians_to_degrees];
end


function beacon_log = make_beacon_log(time_s, azimuth_deg, ...
    beacon_range_ned_m, true_range_m, measured_range_m)
    range_count = numel(time_s);
    azimuth_count = numel(azimuth_deg);
    log_time_s = repmat(time_s, azimuth_count, 1);
    log_azimuth_deg = repelem(azimuth_deg, range_count, 1);
    log_beacon_ned_m = zeros(range_count * azimuth_count, 3);
    for azimuth_index = 1:azimuth_count
        rows = (azimuth_index - 1) * range_count + (1:range_count);
        log_beacon_ned_m(rows, :) = beacon_range_ned_m(:, :, azimuth_index);
    end
    beacon_log = table(log_time_s, log_azimuth_deg, ...
        log_beacon_ned_m(:, 1), log_beacon_ned_m(:, 2), ...
        log_beacon_ned_m(:, 3), repmat(true_range_m, azimuth_count, 1), ...
        repmat(measured_range_m, azimuth_count, 1), ...
        'VariableNames', {'Time_s', 'Azimuth_deg', 'BeaconNorth_m', ...
        'BeaconEast_m', 'BeaconDown_m', 'TrueHorizontalRange_m', ...
        'MeasuredHorizontalRange_m'});
end
