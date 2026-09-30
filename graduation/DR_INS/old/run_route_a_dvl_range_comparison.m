%% 路线 A：深度 / 水平距离 / DVL 不同辅助组合的对比实验
% 四套 15 状态 INS 使用相同的 IMU、初始误差和滤波参数，仅量测组合不同：
%   1) INS + Depth
%   2) INS + Horizontal Range + Depth
%   3) INS + DVL + Depth
%   4) INS + DVL + Horizontal Range + Depth
%
% 声学量测采用单个固定海底信标到载体的水平距离，8 s 更新一次，
% 仿真噪声和滤波量测标准差均为 5 m。脚本直接生成 .nav、误差表和图。

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

%% 数据与声学距离配置
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

range_cfg.interval_s = 8.0;
range_cfg.noise_std_m = 5.0;
range_cfg.filter_std_m = 5.0;
range_cfg.random_seed = cfg.random_seed + 800;
range_cfg.beacon_east_margin_m = 500.0;
range_cfg.beacon_depth_margin_m = 50.0;

range_stride = round(range_cfg.interval_s / cfg.imu_ts_s);
if abs(range_stride * cfg.imu_ts_s - range_cfg.interval_s) > 1e-12
    error('声学距离更新周期必须是 IMU 周期的整数倍。');
end

% 固定海底信标：北向取航迹中点，东向位于航迹东侧 500 m。
% 信标深度仍被保存用于几何展示，但不参与本次水平距离量测。
truth_ned = truth.position_ned_m;
beacon_ned_m = [mean([min(truth_ned(:, 1)), max(truth_ned(:, 1))]); ...
    max(truth_ned(:, 2)) + range_cfg.beacon_east_margin_m; ...
    range_cfg.beacon_depth_margin_m];
beacon_enu_m = [beacon_ned_m(2), beacon_ned_m(1), -beacon_ned_m(3)];
beacon_lla = dxyz2pos(beacon_enu_m, truth.origin_lla);
beacon_lla = beacon_lla(1, 1:3)';

% 第一次测距安排在 t=8 s，使四种方案在 t=0 保持完全相同的初值。
range_imu_index = (1 + range_stride:range_stride:size(imu, 1))';
range_time_s = imu(range_imu_index, 1);
delta_to_beacon_ned_m = truth_ned(range_imu_index, :) - beacon_ned_m';
range_true_m = sqrt(sum(delta_to_beacon_ned_m(:, 1:2).^2, 2));
rng(range_cfg.random_seed, 'twister');
range_meas_m = range_true_m + range_cfg.noise_std_m * ...
    randn(size(range_true_m));

%% 四套相同初值的 15 状态 INS
case_names = {'INS-Depth', 'INS-Range-Depth', ...
    'INS-DVL-Depth', 'INS-DVL-Range-Depth'};
use_dvl = [false, false, true, true];
use_range = [false, true, false, true];
case_count = numel(case_names);

nav_cfg = build_nav_config(cfg, truth);
kfs = cell(1, case_count);
navstates = cell(1, case_count);
last_imu_corrected = cell(1, case_count);
for case_index = 1:case_count
    [kfs{case_index}, navstates{case_index}] = myInitialize_15state(nav_cfg);
    kfs{case_index}.depthstd = cfg.depth_filter_std_m;
    kfs{case_index}.rangstd = range_cfg.filter_std_m;
    last_imu_corrected{case_index} = compensate_imu( ...
        imu(1, :)', navstates{case_index}, cfg.imu_ts_s);
end

output_count = numel(dvl.time_s);
position_lla = cell(1, case_count);
position_ned_m = cell(1, case_count);
velocity_ned_mps = cell(1, case_count);
attitude_rph_rad = cell(1, case_count);
state_std = cell(1, case_count);
for case_index = 1:case_count
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
end

range_innovation_m = nan(numel(range_time_s), case_count);
dvl_index = 2;
range_index = 1;

fprintf('开始路线 A 四组并行推算：IMU %.0f Hz，DVL %.0f Hz，深度计 %.0f Hz，测距间隔 %.1f s。\n', ...
    1 / cfg.imu_ts_s, 1 / cfg.dvl_interval_s, ...
    1 / cfg.depth_interval_s, range_cfg.interval_s);

for imu_index = 2:size(imu, 1)
    is_dvl_epoch = dvl_index <= output_count && ...
        imu_index == dvl.imu_index(dvl_index);
    is_range_epoch = range_index <= numel(range_imu_index) && ...
        imu_index == range_imu_index(range_index);

    for case_index = 1:case_count
        laststate = navstates{case_index};
        this_imu_raw = imu(imu_index, :)';
        dt = this_imu_raw(1) - imu(imu_index - 1, 1);
        this_imu_corrected = compensate_imu( ...
            this_imu_raw, navstates{case_index}, dt);

        navstates{case_index} = InsMech(laststate, ...
            last_imu_corrected{case_index}, this_imu_corrected);
        kfs{case_index} = myInsPropagate_15state( ...
            navstates{case_index}, this_imu_corrected, dt, kfs{case_index});

        % 深度计输出向下为正的 depth；原高度更新接口输入 h=-depth。
        height_data = [depth.time_s(imu_index), -depth.meas_m(imu_index)];
        kfs{case_index} = myHeightUpdate( ...
            navstates{case_index}, height_data, kfs{case_index});
        navstates{case_index}.pos(3) = navstates{case_index}.pos(3) - ...
            kfs{case_index}.x(3);
        navstates{case_index}.vel(3) = navstates{case_index}.vel(3) - ...
            kfs{case_index}.x(6);
        kfs{case_index}.x(3) = 0;
        kfs{case_index}.x(6) = 0;

        horizontal_update_applied = false;
        if is_dvl_epoch && use_dvl(case_index)
            kfs{case_index} = dvl_velocity_update( ...
                navstates{case_index}, ...
                dvl.velocity_d_meas_mps(dvl_index, :)', ...
                cfg.dvl_filter_std_mps, kfs{case_index});
            horizontal_update_applied = true;
        end

        if is_range_epoch && use_range(case_index)
            [kfs{case_index}, innovation_m] = acoustic_horizontal_range_update( ...
                navstates{case_index}, beacon_lla, ...
                range_meas_m(range_index), range_cfg.filter_std_m, ...
                kfs{case_index});
            range_innovation_m(range_index, case_index) = innovation_m;
            horizontal_update_applied = true;
        end

        % 同一历元的 DVL 与距离先顺序更新，再统一进行一次闭环反馈。
        if horizontal_update_applied
            [kfs{case_index}, navstates{case_index}] = ...
                myErrorFeedback_15state(kfs{case_index}, navstates{case_index});
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
    error('存在未处理完的 DVL 或声学距离历元。');
end

%% 保存四组结果和标准 .nav 文件
output_dir = fullfile(study_dir, 'output_route_a_range');
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
    range_meas_m, range_innovation_m(:, 2), range_innovation_m(:, 4), ...
    'VariableNames', {'ID', 'Time_s', 'TrueHorizontalRange_m', ...
    'MeasuredHorizontalRange_m', 'INSRangeDepthInnovation_m', ...
    'INSDVLRangeDepthInnovation_m'});
writetable(range_data, fullfile(output_dir, 'acoustic_range_measurements.csv'));

save(fullfile(output_dir, 'route_a_aiding_comparison.mat'), ...
    'results', 'cfg', 'range_cfg', 'beacon_ned_m', 'beacon_lla', ...
    'range_data', '-v7.3');

%% 误差评估：径向误差、垂向误差、速度误差和 %航程
truth_output_ned_m = truth.position_ned_m(dvl.imu_index, :);
truth_output_velocity_ned_mps = truth.velocity_ned_mps(dvl.imu_index, :);
route_distance_m = sum(sqrt(sum(diff(truth.position_ned_m(:, 1:2), 1, 1).^2, 2)));

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
%% 沿用本地 GJB 径向误差函数，输出其统计表和对比图。
truth_path = fullfile(study_dir, 'data_no_range', 'reference.txt');
[radial_fig, radial_statistics] = calc_radial_error_gjb( ...
    truth_path, nav_paths{2:4}, false);
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

fprintf('\n路线 A 辅助方式对比完成，结果目录：%s\n', output_dir);
fprintf('固定信标 NED = [%.1f, %.1f, %.1f] m，测距数量 = %d。\n', ...
    beacon_ned_m(1), beacon_ned_m(2), beacon_ned_m(3), ...
    numel(range_time_s));
disp(error_summary);


function kf = dvl_velocity_update(navstate, velocity_d_rfu, velocity_std, kf)
% DVL 名义安装矩阵取单位阵；数据中的安装角、比例因子和零偏作为误差保留。
    velocity_body_frd = [velocity_d_rfu(2); velocity_d_rfu(1); ...
        -velocity_d_rfu(3)];
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


function [kf, innovation_m] = acoustic_horizontal_range_update( ...
    navstate, beacon_lla, measured_range_m, range_std_m, kf)
% 水平距离仅约束纬度、经度误差；深度由独立深度计量测约束。
    param = Param();
    [rm, rn] = getRmRn(beacon_lla(1), param);
    DR = diag([rm + beacon_lla(3), ...
        (rn + beacon_lla(3)) * cos(beacon_lla(1)), -1]);
    delta_ned_m = DR * (navstate.pos - beacon_lla);
    predicted_range_m = norm(delta_ned_m(1:2));
    if predicted_range_m < 1e-6
        error('载体与信标距离过小，无法构造距离量测雅可比。');
    end

    innovation_m = predicted_range_m - measured_range_m;
    H = zeros(1, kf.RANK);
    H(1, 1:2) = (delta_ned_m(1:2)' / predicted_range_m) * ...
        DR(1:2, 1:2);
    R = range_std_m^2;
    K = kf.P * H' / (H * kf.P * H' + R);
    kf.x = kf.x + K * (innovation_m - H * kf.x);
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
% 与既有 FGO/INS 脚本一致：ID, time, lat/lon(deg), h, vN/vE/vD,
% roll/pitch/heading(deg)。
    radians_to_degrees = 180 / pi;
    nav_matrix = [id, time_s, ...
        position_lla(:, 1:2) * radians_to_degrees, position_lla(:, 3), ...
        velocity_ned_mps, attitude_rph_rad * radians_to_degrees];
end
