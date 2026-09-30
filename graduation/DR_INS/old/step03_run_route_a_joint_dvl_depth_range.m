%% 路线 A 专项：DVL、深度与水平距离的同历元联合量测更新
% 本脚本只研究以下两种 INS 主导航方案：
%   1) INS + DVL + Depth（DVL 历元采用 4 维联合更新）
%   2) INS + DVL + Horizontal Range + Depth
%      （DVL 与距离同时到达时采用 5 维联合更新）
%
% 更新规则：
%   - 普通 IMU 历元：INS 推算 + 深度更新；
%   - DVL 历元：深度和 DVL 组成同一个量测向量，一次 Kalman 更新；
%   - DVL+距离历元：深度、水平距离和 DVL 组成同一个量测向量，
%     一次 Kalman 更新；同一深度量测不会重复使用。

clear;
% close all;
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

%% 加载公共仿真数据
data_file = fullfile(study_dir, 'data_no_range', 'simulation_data.mat');
if ~isfile(data_file)
    error('缺少仿真数据，请先运行 simulate_dr_ins_data.m。');
end
data = load(data_file, 'cfg', 'truth', 'imu', 'dvl', 'depth');
cfg = data.cfg;
cfg.dvl_filter_std_mps = 0.01;
truth = data.truth;
imu = data.imu;
dvl = data.dvl;
depth = data.depth;

if numel(depth.time_s) ~= size(imu, 1) || ...
        any(depth.imu_index(:) ~= (1:size(imu, 1))')
    error('本脚本要求深度计与 100 Hz IMU 逐历元同步。');
end

%% 生成 8 s、5 m 标准差的单信标水平距离
range_cfg.interval_s = 8.0;
range_cfg.noise_std_m = 4.0; 
range_cfg.filter_std_m = 5.0;
range_cfg.random_seed = cfg.random_seed + 800;
range_cfg.beacon_east_margin_m = 500.0;
range_cfg.beacon_depth_margin_m = 50.0;

range_stride = round(range_cfg.interval_s / cfg.imu_ts_s);
if abs(range_stride * cfg.imu_ts_s - range_cfg.interval_s) > 1e-12
    error('水平距离更新周期必须是 IMU 周期的整数倍。');
end

truth_ned_m = truth.position_ned_m;
beacon_ned_m = [ truth_ned_m(1, 2) + 2000; ...
    truth_ned_m(1, 2) - 1000; ...
    range_cfg.beacon_depth_margin_m];
% beacon_ned_m = [mean([min(truth_ned_m(:, 1)), max(truth_ned_m(:, 1))]); ...
%     max(truth_ned_m(:, 2)) + range_cfg.beacon_east_margin_m; ...
%     range_cfg.beacon_depth_margin_m];
beacon_enu_m = [beacon_ned_m(2), beacon_ned_m(1), -beacon_ned_m(3)];
beacon_lla = dxyz2pos(beacon_enu_m, truth.origin_lla);
beacon_lla = beacon_lla(1, 1:3)';

% 第一次距离量测位于 t = 8 s，并且所有距离历元必须同时存在 DVL。
range_imu_index = (1 + range_stride:range_stride:size(imu, 1))';
if ~all(ismember(range_imu_index, dvl.imu_index))
    error('存在没有同步 DVL 的水平距离历元，无法执行指定的联合更新。');
end
range_time_s = imu(range_imu_index, 1);
delta_to_beacon_ned_m = truth_ned_m(range_imu_index, :) - beacon_ned_m';
range_true_m = sqrt(sum(delta_to_beacon_ned_m(:, 1:2).^2, 2));
rng(range_cfg.random_seed, 'twister');
range_meas_m = range_true_m + range_cfg.noise_std_m * ...
    randn(size(range_true_m));

%% 初始化两套相同的 15 状态 INS
case_names = {'INS-DVL-Depth-Joint', 'INS-DVL-HRange-Depth-Joint'};
case_count = numel(case_names);
use_range = [false, true];

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
depth_innovation_m = nan(output_count, case_count);
dvl_innovation_norm_mps = nan(output_count, case_count);
range_innovation_m = nan(output_count, case_count);
joint_measurement_dimension = zeros(output_count, case_count);

% t = 0.01 s 的首个 DVL 和深度量测也进行 4 维联合更新。
for case_index = 1:case_count
    [kfs{case_index}, navstates{case_index}] = myInitialize_15state(nav_cfg);
    % 普通 IMU 历元仍复用本地 myHeightUpdate 接口。
    kfs{case_index}.depthstd = cfg.depth_filter_std_m;
    [kfs{case_index}, innovation] = joint_aiding_update(navstates{case_index}, depth.meas_m(1), ...
        dvl.velocity_d_meas_mps(1, :)', cfg.depth_filter_std_m, ...
        cfg.dvl_filter_std_mps, false, beacon_lla, nan, ...
        range_cfg.filter_std_m, kfs{case_index});
    [kfs{case_index}, navstates{case_index}] = myErrorFeedback_15state(kfs{case_index}, navstates{case_index});

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
    depth_innovation_m(1, case_index) = innovation.depth_m;
    dvl_innovation_norm_mps(1, case_index) = norm(innovation.dvl_mps);
    joint_measurement_dimension(1, case_index) = innovation.dimension;
    last_imu_corrected{case_index} = compensate_imu( ...
        imu(1, :)', navstates{case_index}, cfg.imu_ts_s);
end

%% 100 Hz 惯性推算与按历元选择的联合更新
dvl_index = 2;
range_index = 1;
fprintf(['开始联合量测专项推算：IMU %.0f Hz，DVL %.0f Hz，', ...
    '深度计 %.0f Hz，水平距离间隔 %.1f s。\n'], ...
    1 / cfg.imu_ts_s, 1 / cfg.dvl_interval_s, ...
    1 / cfg.depth_interval_s, range_cfg.interval_s);

for imu_index = 2:size(imu, 1)
    is_dvl_epoch = dvl_index <= output_count && ...
        imu_index == dvl.imu_index(dvl_index);
    is_range_epoch = range_index <= numel(range_imu_index) && ...
        imu_index == range_imu_index(range_index);
    this_imu_raw = imu(imu_index, :)';
    dt = this_imu_raw(1) - imu(imu_index - 1, 1);

    for case_index = 1:case_count
        laststate = navstates{case_index};
        % this_imu_corrected = compensate_imu( ...
        %     this_imu_raw, navstates{case_index}, dt);
        this_imu_corrected = this_imu_raw;
        navstates{case_index} = InsMech(laststate, ...
            last_imu_corrected{case_index}, this_imu_corrected);
        kfs{case_index} = myInsPropagate_15state( ...
            navstates{case_index}, this_imu_corrected, dt, kfs{case_index});

        if is_dvl_epoch
            include_range = use_range(case_index) && is_range_epoch;
            if include_range
                measured_range_m = range_meas_m(range_index);
            else
                measured_range_m = nan;
            end

            % DVL 历元不先做单独深度更新；深度只在此联合更新中使用一次。
            [kfs{case_index}, innovation] = joint_aiding_update( ...
                navstates{case_index}, depth.meas_m(imu_index), ...
                dvl.velocity_d_meas_mps(dvl_index, :)', ...
                cfg.depth_filter_std_m, cfg.dvl_filter_std_mps, ...
                include_range, beacon_lla, measured_range_m, ...
                range_cfg.filter_std_m, kfs{case_index});
            [kfs{case_index}, navstates{case_index}] = ...
                myErrorFeedback_15state(kfs{case_index}, ...
                navstates{case_index});

            depth_innovation_m(dvl_index, case_index) = innovation.depth_m;
            dvl_innovation_norm_mps(dvl_index, case_index) = ...
                norm(innovation.dvl_mps);
            range_innovation_m(dvl_index, case_index) = innovation.range_m;
            joint_measurement_dimension(dvl_index, case_index) = ...
                innovation.dimension;
        else
            % 无 DVL 时沿用原来的解耦深度更新，仅反馈垂向位置和速度。
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

if dvl_index ~= output_count + 1 || range_index ~= numel(range_imu_index) + 1
    error('存在未处理完的 DVL 或水平距离历元。');
end

%% 保存导航结果、量测记录和 .nav 文件
output_dir = fullfile(study_dir, 'output_route_a_joint_update');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

result_template = struct('name', '', 'time_s', [], 'position_lla', [], ...
    'position_ned_m', [], 'velocity_ned_mps', [], ...
    'attitude_rph_rad', [], 'state_std', []);
results = repmat(result_template, 1, case_count);
nav_paths = cell(1, case_count);
for case_index = 1:case_count
    results(case_index).name = case_names{case_index};
    results(case_index).time_s = dvl.time_s;
    results(case_index).position_lla = position_lla{case_index};
    results(case_index).position_ned_m = position_ned_m{case_index};
    results(case_index).velocity_ned_mps = velocity_ned_mps{case_index};
    results(case_index).attitude_rph_rad = attitude_rph_rad{case_index};
    results(case_index).state_std = state_std{case_index};

    nav_matrix = make_nav_matrix((0:output_count-1)', dvl.time_s, ...
        position_lla{case_index}, velocity_ned_mps{case_index}, ...
        attitude_rph_rad{case_index});
    nav_paths{case_index} = fullfile(output_dir, [case_names{case_index}, '.nav']);
    writematrix(nav_matrix, nav_paths{case_index}, ...
        'FileType', 'text', 'Delimiter', 'tab');
end

range_data = table((1:numel(range_time_s))', range_time_s, range_true_m, ...
    range_meas_m, 'VariableNames', {'ID', 'Time_s', ...
    'TrueHorizontalRange_m', 'MeasuredHorizontalRange_m'});
writetable(range_data, fullfile(output_dir, ...
    'horizontal_range_measurements.csv'));

joint_update_log = table(dvl.time_s, ...
    depth_innovation_m(:, 1), dvl_innovation_norm_mps(:, 1), ...
    joint_measurement_dimension(:, 1), depth_innovation_m(:, 2), ...
    dvl_innovation_norm_mps(:, 2), range_innovation_m(:, 2), ...
    joint_measurement_dimension(:, 2), ...
    'VariableNames', {'Time_s', 'DepthInnovation_NoRange_m', ...
    'DVLInnovationNorm_NoRange_mps', 'Dimension_NoRange', ...
    'DepthInnovation_WithRange_m', 'DVLInnovationNorm_WithRange_mps', ...
    'HorizontalRangeInnovation_m', 'Dimension_WithRange'});
writetable(joint_update_log, fullfile(output_dir, 'joint_update_log.csv'));

save(fullfile(output_dir, 'route_a_joint_update_result.mat'), ...
    'results', 'cfg', 'range_cfg', 'beacon_ned_m', 'beacon_lla', ...
    'range_data', 'joint_update_log', '-v7.3');

%% 误差评估：水平径向误差和 %航程
truth_output_ned_m = truth.position_ned_m(dvl.imu_index, :);
truth_output_velocity_ned_mps = truth.velocity_ned_mps(dvl.imu_index, :);
route_distance_m = sum(sqrt(sum(diff(truth.position_ned_m(:, 1:2), ...
    1, 1).^2, 2)));

system_name = strings(case_count, 1);
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
    horizontal_error_m = sqrt(sum(position_error_ned_m(:, 1:2).^2, 2));
    velocity_error_mps = velocity_ned_mps{case_index} - ...
        truth_output_velocity_ned_mps;

    system_name(case_index) = string(case_names{case_index});
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

error_summary = table(system_name, repmat(route_distance_m, case_count, 1), ...
    final_horizontal_m, max_horizontal_m, rms_horizontal_m, ...
    mean_horizontal_m, p95_horizontal_m, route_error_percent, ...
    rms_vertical_m, rms_velocity_mps, ...
    'VariableNames', {'System', 'RouteDistance_m', ...
    'FinalHorizontalError_m', 'MaxHorizontalError_m', ...
    'RMSHorizontalError_m', 'MeanHorizontalError_m', ...
    'P95HorizontalError_m', 'RouteError_percent', ...
    'RMSVerticalError_m', 'RMSVelocityError_mps'});
writetable(error_summary, fullfile(output_dir, 'error_summary.csv'));
writetable(error_summary, fullfile(output_dir, 'error_summary.xlsx'));

truth_path = fullfile(study_dir, 'data_no_range', 'reference.txt');
[radial_fig, radial_statistics] = calc_radial_error_gjb( ...
    truth_path, nav_paths{:}, false);
exportgraphics(radial_fig, fullfile(output_dir, ...
    'radial_error_comparison.png'), 'Resolution', 200);
writecell(radial_statistics, fullfile(output_dir, ...
    'radial_error_statistics.xlsx'));

trajectory_fig = figure('Color', 'w');
plot(truth.position_ned_m(:, 2), truth.position_ned_m(:, 1), ...
    'k-', 'LineWidth', 1.5, 'DisplayName', 'Truth');
hold on;
colors = lines(case_count);
for case_index = 1:case_count
    plot(position_ned_m{case_index}(:, 2), ...
        position_ned_m{case_index}(:, 1), 'LineWidth', 1.0, ...
        'Color', colors(case_index, :), ...
        'DisplayName', case_names{case_index});
end
plot(beacon_ned_m(2), beacon_ned_m(1), 'p', ...
    'MarkerSize', 12, 'MarkerFaceColor', [0.95, 0.65, 0.10], ...
    'MarkerEdgeColor', 'k', 'DisplayName', 'Acoustic beacon');
axis equal;
grid on;
box on;
xlabel('East (m)');
ylabel('North (m)');
legend('Location', 'best');
exportgraphics(trajectory_fig, fullfile(output_dir, ...
    'trajectory_comparison.png'), 'Resolution', 200);

fprintf('\n联合量测专项推算完成，结果目录：%s\n', output_dir);
fprintf('无距离方案：4 维深度+DVL 联合更新 %d 次。\n', output_count);
fprintf('有距离方案：4 维联合更新 %d 次，5 维联合更新 %d 次。\n', ...
    output_count - numel(range_time_s), numel(range_time_s));
disp(error_summary);


function [kf, innovation] = joint_aiding_update(navstate, depth_meas_m, ...
    velocity_d_rfu, depth_std_m, dvl_std_mps, include_range, ...
    beacon_lla, measured_range_m, range_std_m, kf)
% 将当前历元所有有效量测堆叠为一个 Z、H、R，并只执行一次 KF 更新。
% 误差状态顺序沿用 myInitialize_15state：位置、速度、姿态、陀螺零偏、
% 加速度计零偏。DVL 部分沿用原脚本的速度误差量测模型。

    velocity_body_frd = [velocity_d_rfu(2); velocity_d_rfu(1); -velocity_d_rfu(3)];
    velocity_dvl_ned = navstate.cbn * velocity_body_frd;

    depth_residual_m = navstate.pos(3) - (-depth_meas_m);
    dvl_residual_mps = navstate.vel - velocity_dvl_ned;

    if include_range
        [range_residual_m, range_H_position] = horizontal_range_residual(navstate, beacon_lla, measured_range_m);
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
    innovation.dimension = size(H, 1);
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
