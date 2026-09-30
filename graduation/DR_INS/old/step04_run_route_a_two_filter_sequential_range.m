%% 路线 A Step04：INS/DVL/深度主滤波器 + 独立水平距离位置滤波器
% 本脚本在 step03 的基础上研究“顺序更新、分离反馈”结构：
%   1) 主滤波器：15 状态 INS 误差状态滤波器；
%   2) 辅助滤波器：2 状态水平位置误差滤波器 [dN; dE]（单位 m）。
%
% 每个 DVL 历元的严格处理顺序：
%   A. 主滤波器先用深度 + DVL 做 4 维联合更新并完成 15 状态反馈；
%   B. 若同历元存在水平距离，辅助滤波器再单独更新；
%   C. 辅助滤波器只把北、东位置误差反馈给导航位置，不修改主滤波器
%      的速度、姿态、IMU 零偏及协方差。
%
% 对比方案：
%   - INS-DVL-Depth：无距离基线；
%   - INS-DVL-Depth-Then-HRange-PositionOnly：两滤波器顺序更新。

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

%% 加载与 step03 相同的公共仿真数据
data_file = fullfile(study_dir, 'data_no_range', 'simulation_data.mat');
if ~isfile(data_file)
    error('缺少仿真数据，请先运行 step01_simulate_dr_ins_data.m。');
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

%% 沿用 step03 的单信标水平距离设置
range_cfg.interval_s = 8.0;
range_cfg.noise_std_m = 4.0;
range_cfg.filter_std_m = 5.0;
range_cfg.random_seed = cfg.random_seed + 800;
range_cfg.beacon_depth_margin_m = 50.0;

% 独立二维位置滤波器参数。位置误差采用米制 N/E 坐标。
range_cfg.position_filter_init_std_m = 5.0;
% 两次距离更新之间未建模水平速度误差导致的位置增长率。
% 每个距离间隔的离散过程噪声为 (rate * DeltaT)^2 I。
range_cfg.position_error_growth_std_mps = 0.01;

range_stride = round(range_cfg.interval_s / cfg.imu_ts_s);
if abs(range_stride * cfg.imu_ts_s - range_cfg.interval_s) > 1e-12
    error('水平距离更新周期必须是 IMU 周期的整数倍。');
end

truth_ned_m = truth.position_ned_m;
beacon_ned_m = [truth_ned_m(1, 2) + 1600; ...
    truth_ned_m(1, 2) - 1500; ...
    range_cfg.beacon_depth_margin_m];
beacon_enu_m = [beacon_ned_m(2), beacon_ned_m(1), -beacon_ned_m(3)];
beacon_lla = dxyz2pos(beacon_enu_m, truth.origin_lla);
beacon_lla = beacon_lla(1, 1:3)';

range_imu_index = (1 + range_stride:range_stride:size(imu, 1))';
[range_has_dvl, range_dvl_index] = ismember(range_imu_index, dvl.imu_index);
if ~all(range_has_dvl)
    error('存在没有同步 DVL 的水平距离历元，无法执行指定的顺序更新。');
end
range_time_s = imu(range_imu_index, 1);
range_count = numel(range_time_s);
delta_to_beacon_ned_m = truth_ned_m(range_imu_index, :) - beacon_ned_m';
range_true_m = vecnorm(delta_to_beacon_ned_m(:, 1:2), 2, 2);
rng(range_cfg.random_seed, 'twister');
range_meas_m = range_true_m + range_cfg.noise_std_m * ...
    randn(size(range_true_m));

%% 初始化两套主滤波器以及距离方案的独立二维位置滤波器
case_names = {'INS-DVL-Depth', ...
    'INS-DVL-Depth-Then-HRange-PositionOnly'};
case_count = numel(case_names);
use_range = [false, true];

nav_cfg = build_nav_config(cfg, truth);
main_kfs = cell(1, case_count);
navstates = cell(1, case_count);
last_imu_corrected = cell(1, case_count);
range_position_kfs = cell(1, case_count);

output_count = numel(dvl.time_s);
position_lla = cell(1, case_count);
position_ned_m = cell(1, case_count);
velocity_ned_mps = cell(1, case_count);
attitude_rph_rad = cell(1, case_count);
main_state_std = cell(1, case_count);
depth_innovation_m = nan(output_count, case_count);
dvl_innovation_norm_mps = nan(output_count, case_count);

range_prefit_innovation_m = nan(range_count, 1);
range_postfit_residual_m = nan(range_count, 1);
range_position_feedback_ne_m = nan(range_count, 2);
range_kalman_gain_ne = nan(range_count, 2);
range_position_std_ne_m = nan(range_count, 2);

for case_index = 1:case_count
    [main_kfs{case_index}, navstates{case_index}] = ...
        myInitialize_15state(nav_cfg);
    main_kfs{case_index}.depthstd = cfg.depth_filter_std_m;

    % 首个 DVL 历元同样只做深度 + DVL 四维联合更新。
    [main_kfs{case_index}, innovation] = depth_dvl_update( ...
        navstates{case_index}, depth.meas_m(1), ...
        dvl.velocity_d_meas_mps(1, :)', cfg.depth_filter_std_m, ...
        cfg.dvl_filter_std_mps, main_kfs{case_index});
    [main_kfs{case_index}, navstates{case_index}] = ...
        myErrorFeedback_15state(main_kfs{case_index}, ...
        navstates{case_index});

    if use_range(case_index)
        range_position_kfs{case_index} = initialize_range_position_filter( ...
            range_cfg.position_filter_init_std_m, ...
            range_cfg.filter_std_m);
    else
        range_position_kfs{case_index} = [];
    end

    position_lla{case_index} = zeros(output_count, 3);
    position_ned_m{case_index} = zeros(output_count, 3);
    velocity_ned_mps{case_index} = zeros(output_count, 3);
    attitude_rph_rad{case_index} = zeros(output_count, 3);
    main_state_std{case_index} = zeros(output_count, 15);
    [position_lla{case_index}(1, :), position_ned_m{case_index}(1, :)] = ...
        export_ins_position(navstates{case_index}, truth.origin_lla);
    velocity_ned_mps{case_index}(1, :) = navstates{case_index}.vel';
    attitude_rph_rad{case_index}(1, :) = navstates{case_index}.att';
    main_state_std{case_index}(1, :) = ...
        sqrt(max(diag(main_kfs{case_index}.P), 0))';
    depth_innovation_m(1, case_index) = innovation.depth_m;
    dvl_innovation_norm_mps(1, case_index) = norm(innovation.dvl_mps);
    last_imu_corrected{case_index} = compensate_imu( ...
        imu(1, :)', navstates{case_index}, cfg.imu_ts_s);
end

%% 100 Hz 惯性推算、2 Hz DVL更新、8 s距离位置更新
dvl_index = 2;
range_index = 1;
fprintf(['开始 Step04 两滤波器顺序更新：IMU %.0f Hz，DVL %.0f Hz，', ...
    '深度计 %.0f Hz，水平距离间隔 %.1f s。\n'], ...
    1 / cfg.imu_ts_s, 1 / cfg.dvl_interval_s, ...
    1 / cfg.depth_interval_s, range_cfg.interval_s);

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
        main_kfs{case_index} = myInsPropagate_15state( ...
            navstates{case_index}, this_imu_corrected, dt, ...
            main_kfs{case_index});

        if is_dvl_epoch
            % 第一级：无论有没有距离，都先执行深度 + DVL 更新和反馈。
            [main_kfs{case_index}, innovation] = depth_dvl_update( ...
                navstates{case_index}, depth.meas_m(imu_index), ...
                dvl.velocity_d_meas_mps(dvl_index, :)', ...
                cfg.depth_filter_std_m, cfg.dvl_filter_std_mps, ...
                main_kfs{case_index});
            [main_kfs{case_index}, navstates{case_index}] = ...
                myErrorFeedback_15state(main_kfs{case_index}, ...
                navstates{case_index});

            depth_innovation_m(dvl_index, case_index) = ...
                innovation.depth_m;
            dvl_innovation_norm_mps(dvl_index, case_index) = ...
                norm(innovation.dvl_mps);

            % 第二级：距离单独估计水平位置误差，并且只反馈位置。
            if use_range(case_index) && is_range_epoch
                if range_index == 1
                    range_dt_s = range_time_s(1) - imu(1, 1);
                else
                    range_dt_s = range_time_s(range_index) - ...
                        range_time_s(range_index - 1);
                end
                range_position_kfs{case_index} = ...
                    propagate_range_position_filter( ...
                    range_position_kfs{case_index}, range_dt_s, ...
                    range_cfg.position_error_growth_std_mps);

                [range_position_kfs{case_index}, range_update] = ...
                    update_range_position_filter( ...
                    range_position_kfs{case_index}, ...
                    navstates{case_index}, truth.origin_lla, ...
                    beacon_ned_m, range_meas_m(range_index));
                navstates{case_index} = ...
                    feedback_horizontal_position_only( ...
                    navstates{case_index}, range_update.feedback_ne_m);

                range_prefit_innovation_m(range_index) = ...
                    range_update.prefit_innovation_m;
                range_postfit_residual_m(range_index) = ...
                    horizontal_range_residual_ned( ...
                    navstates{case_index}, truth.origin_lla, ...
                    beacon_ned_m, range_meas_m(range_index));
                range_position_feedback_ne_m(range_index, :) = ...
                    range_update.feedback_ne_m';
                range_kalman_gain_ne(range_index, :) = ...
                    range_update.kalman_gain_ne';
                range_position_std_ne_m(range_index, :) = ...
                    sqrt(max(diag(range_position_kfs{case_index}.P), 0))';

                % 误差状态已反馈到名义位置，辅助滤波器状态清零。
                range_position_kfs{case_index}.x(:) = 0;
            end
        else
            % 非 DVL 历元沿用 step03 的解耦深度更新。
            height_data = [depth.time_s(imu_index), -depth.meas_m(imu_index)];
            main_kfs{case_index} = myHeightUpdate( ...
                navstates{case_index}, height_data, main_kfs{case_index});
            navstates{case_index}.pos(3) = navstates{case_index}.pos(3) - ...
                main_kfs{case_index}.x(3);
            navstates{case_index}.vel(3) = navstates{case_index}.vel(3) - ...
                main_kfs{case_index}.x(6);
            main_kfs{case_index}.x(3) = 0;
            main_kfs{case_index}.x(6) = 0;
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
            main_state_std{case_index}(dvl_index, :) = ...
                sqrt(max(diag(main_kfs{case_index}.P), 0))';
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
if any(~isfinite(range_prefit_innovation_m))
    error('存在未执行的距离位置滤波更新。');
end

%% 保存导航结果、两级滤波日志和 .nav 文件
output_dir = fullfile(study_dir, 'output_route_a_two_filter_sequential');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

result_template = struct('name', '', 'time_s', [], ...
    'position_lla', [], 'position_ned_m', [], ...
    'velocity_ned_mps', [], 'attitude_rph_rad', [], ...
    'main_state_std', []);
results = repmat(result_template, 1, case_count);
nav_paths = cell(1, case_count);
for case_index = 1:case_count
    results(case_index).name = case_names{case_index};
    results(case_index).time_s = dvl.time_s;
    results(case_index).position_lla = position_lla{case_index};
    results(case_index).position_ned_m = position_ned_m{case_index};
    results(case_index).velocity_ned_mps = velocity_ned_mps{case_index};
    results(case_index).attitude_rph_rad = attitude_rph_rad{case_index};
    results(case_index).main_state_std = main_state_std{case_index};

    nav_matrix = make_nav_matrix((0:output_count-1)', dvl.time_s, ...
        position_lla{case_index}, velocity_ned_mps{case_index}, ...
        attitude_rph_rad{case_index});
    nav_paths{case_index} = fullfile(output_dir, ...
        [case_names{case_index}, '.nav']);
    writematrix(nav_matrix, nav_paths{case_index}, ...
        'FileType', 'text', 'Delimiter', 'tab');
end

range_data = table((1:range_count)', range_time_s, range_true_m, ...
    range_meas_m, range_prefit_innovation_m, ...
    range_postfit_residual_m, ...
    range_position_feedback_ne_m(:, 1), ...
    range_position_feedback_ne_m(:, 2), ...
    range_kalman_gain_ne(:, 1), range_kalman_gain_ne(:, 2), ...
    range_position_std_ne_m(:, 1), range_position_std_ne_m(:, 2), ...
    'VariableNames', {'ID', 'Time_s', 'TrueHorizontalRange_m', ...
    'MeasuredHorizontalRange_m', 'PrefitInnovation_m', ...
    'PostfitResidual_m', 'FeedbackNorth_m', 'FeedbackEast_m', ...
    'KalmanGainNorth', 'KalmanGainEast', ...
    'PositionStdNorth_m', 'PositionStdEast_m'});
writetable(range_data, fullfile(output_dir, ...
    'separate_range_position_filter_log.csv'));

main_filter_log = table(dvl.time_s, ...
    depth_innovation_m(:, 1), dvl_innovation_norm_mps(:, 1), ...
    depth_innovation_m(:, 2), dvl_innovation_norm_mps(:, 2), ...
    'VariableNames', {'Time_s', 'DepthInnovation_NoRange_m', ...
    'DVLInnovationNorm_NoRange_mps', ...
    'DepthInnovation_TwoFilter_m', ...
    'DVLInnovationNorm_TwoFilter_mps'});
writetable(main_filter_log, fullfile(output_dir, ...
    'main_depth_dvl_filter_log.csv'));

%% 误差评估
truth_output_ned_m = truth.position_ned_m(dvl.imu_index, :);
truth_output_velocity_ned_mps = truth.velocity_ned_mps(dvl.imu_index, :);
route_distance_m = sum(vecnorm(diff( ...
    truth.position_ned_m(:, 1:2), 1, 1), 2, 2));

system_name = string(case_names(:));
final_horizontal_m = zeros(case_count, 1);
max_horizontal_m = zeros(case_count, 1);
rms_horizontal_m = zeros(case_count, 1);
mean_horizontal_m = zeros(case_count, 1);
p95_horizontal_m = zeros(case_count, 1);
route_error_percent = zeros(case_count, 1);
rms_vertical_m = zeros(case_count, 1);
rms_velocity_mps = zeros(case_count, 1);

for case_index = 1:case_count
    position_error_ned_m = position_ned_m{case_index} - ...
        truth_output_ned_m;
    horizontal_error_m = vecnorm(position_error_ned_m(:, 1:2), 2, 2);
    velocity_error_mps = velocity_ned_mps{case_index} - ...
        truth_output_velocity_ned_mps;
    final_horizontal_m(case_index) = horizontal_error_m(end);
    max_horizontal_m(case_index) = max(horizontal_error_m);
    rms_horizontal_m(case_index) = sqrt(mean(horizontal_error_m.^2));
    mean_horizontal_m(case_index) = mean(horizontal_error_m);
    p95_horizontal_m(case_index) = prctile(horizontal_error_m, 95);
    route_error_percent(case_index) = 100 * ...
        final_horizontal_m(case_index) / route_distance_m;
    rms_vertical_m(case_index) = ...
        sqrt(mean(position_error_ned_m(:, 3).^2));
    rms_velocity_mps(case_index) = ...
        sqrt(mean(sum(velocity_error_mps.^2, 2)));
end

rms_improvement_percent = 100 * (rms_horizontal_m(1) - ...
    rms_horizontal_m) / rms_horizontal_m(1);
final_improvement_percent = 100 * (final_horizontal_m(1) - ...
    final_horizontal_m) / final_horizontal_m(1);

error_summary = table(system_name, ...
    repmat(route_distance_m, case_count, 1), final_horizontal_m, ...
    max_horizontal_m, rms_horizontal_m, mean_horizontal_m, ...
    p95_horizontal_m, route_error_percent, rms_vertical_m, ...
    rms_velocity_mps, rms_improvement_percent, ...
    final_improvement_percent, ...
    'VariableNames', {'System', 'RouteDistance_m', ...
    'FinalHorizontalError_m', 'MaxHorizontalError_m', ...
    'RMSHorizontalError_m', 'MeanHorizontalError_m', ...
    'P95HorizontalError_m', 'RouteError_percent', ...
    'RMSVerticalError_m', 'RMSVelocityError_mps', ...
    'RMSImprovement_percent', 'FinalImprovement_percent'});
writetable(error_summary, fullfile(output_dir, 'error_summary.csv'));
writetable(error_summary, fullfile(output_dir, 'error_summary.xlsx'));

save(fullfile(output_dir, 'route_a_two_filter_sequential_result.mat'), ...
    'results', 'cfg', 'range_cfg', 'beacon_ned_m', 'beacon_lla', ...
    'range_data', 'main_filter_log', 'error_summary', '-v7.3');

%% 绘图
truth_path = fullfile(study_dir, 'data_no_range', 'reference.txt');
[radial_fig, radial_statistics] = calc_radial_error_gjb( ...
    truth_path, nav_paths{:}, false);
exportgraphics(radial_fig, fullfile(output_dir, ...
    '01_radial_error_comparison.png'), 'Resolution', 200);
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
    '02_trajectory_comparison.png'), 'Resolution', 200);

range_fig = figure('Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact');
nexttile;
plot(range_time_s / 60, range_prefit_innovation_m, ...
    'LineWidth', 1.0, 'DisplayName', 'Prefit');
hold on;
plot(range_time_s / 60, range_postfit_residual_m, ...
    'LineWidth', 1.0, 'DisplayName', 'Postfit');
grid on;
ylabel('Range residual (m)');
legend('Location', 'best');
nexttile;
plot(range_time_s / 60, range_position_feedback_ne_m(:, 1), ...
    'LineWidth', 1.0, 'DisplayName', 'North');
hold on;
plot(range_time_s / 60, range_position_feedback_ne_m(:, 2), ...
    'LineWidth', 1.0, 'DisplayName', 'East');
grid on;
ylabel('Position feedback (m)');
legend('Location', 'best');
nexttile;
plot(range_time_s / 60, range_position_std_ne_m(:, 1), ...
    'LineWidth', 1.0, 'DisplayName', 'North std');
hold on;
plot(range_time_s / 60, range_position_std_ne_m(:, 2), ...
    'LineWidth', 1.0, 'DisplayName', 'East std');
grid on;
xlabel('Time (min)');
ylabel('Auxiliary KF std (m)');
legend('Location', 'best');
exportgraphics(range_fig, fullfile(output_dir, ...
    '03_separate_range_filter_diagnostics.png'), 'Resolution', 200);

fprintf('\nStep04 两滤波器顺序更新完成：%s\n', output_dir);
fprintf('主滤波器深度+DVL更新 %d 次；独立距离位置更新 %d 次。\n', ...
    output_count, range_count);
fprintf(['距离只反馈 N/E 位置；不反馈主滤波器速度、姿态、', ...
    'IMU 零偏和协方差。\n']);
disp(error_summary(:, {'System', 'FinalHorizontalError_m', ...
    'RMSHorizontalError_m', 'RouteError_percent', ...
    'RMSImprovement_percent'}));


function [kf, innovation] = depth_dvl_update(navstate, depth_meas_m, ...
    velocity_d_rfu, depth_std_m, dvl_std_mps, kf)
% 主滤波器只使用深度和 DVL，不包含距离量测。
    velocity_body_frd = [velocity_d_rfu(2); velocity_d_rfu(1); ...
        -velocity_d_rfu(3)];
    velocity_dvl_ned = navstate.cbn * velocity_body_frd;
    depth_residual_m = navstate.pos(3) - (-depth_meas_m);
    dvl_residual_mps = navstate.vel - velocity_dvl_ned;

    Z = [depth_residual_m; dvl_residual_mps];
    H = zeros(4, kf.RANK);
    H(1, 3) = 1;
    H(2:4, 4:6) = eye(3);
    R = diag([depth_std_m^2; repmat(dvl_std_mps^2, 3, 1)]);
    K = kf.P * H' / (H * kf.P * H' + R);
    kf.x = kf.x + K * (Z - H * kf.x);
    I = eye(kf.RANK);
    kf.P = (I - K * H) * kf.P * (I - K * H)' + K * R * K';

    innovation.depth_m = depth_residual_m;
    innovation.dvl_mps = dvl_residual_mps;
end


function range_kf = initialize_range_position_filter(init_std_m, range_std_m)
% 二维误差状态定义为估计位置减真实位置：[dN; dE]，单位 m。
    range_kf.x = zeros(2, 1);
    range_kf.P = eye(2) * init_std_m^2;
    range_kf.R = range_std_m^2;
end


function range_kf = propagate_range_position_filter( ...
    range_kf, delta_t_s, position_error_growth_std_mps)
% 将未建模的水平速度误差折算为相邻距离历元间的位置过程噪声。
    interval_position_std_m = position_error_growth_std_mps * delta_t_s;
    range_kf.P = range_kf.P + ...
        eye(2) * interval_position_std_m^2;
end


function [range_kf, update] = update_range_position_filter( ...
    range_kf, navstate, origin_lla, beacon_ned_m, measured_range_m)
% 以米制水平位置误差为状态，用一维水平距离残差进行更新。
    vehicle_ned_m = position_lla_to_ned(navstate.pos, origin_lla);
    delta_ne_m = vehicle_ned_m(1:2) - beacon_ned_m(1:2);
    predicted_range_m = norm(delta_ne_m);
    if predicted_range_m < 1e-6
        error('载体与信标水平距离过小，无法构造距离雅可比。');
    end

    H = delta_ne_m' / predicted_range_m;
    innovation_m = predicted_range_m - measured_range_m;
    K = range_kf.P * H' / (H * range_kf.P * H' + range_kf.R);
    range_kf.x = range_kf.x + K * (innovation_m - H * range_kf.x);
    I = eye(2);
    range_kf.P = (I - K * H) * range_kf.P * (I - K * H)' + ...
        K * range_kf.R * K';

    update.prefit_innovation_m = innovation_m;
    update.feedback_ne_m = range_kf.x;
    update.kalman_gain_ne = K;
end


function navstate = feedback_horizontal_position_only(navstate, feedback_ne_m)
% 只改正纬度、经度；高度、速度、姿态和所有传感器误差状态均不变。
    param = Param();
    [rm, rn] = getRmRn(navstate.pos(1), param);
    DR_horizontal = diag([rm + navstate.pos(3), ...
        (rn + navstate.pos(3)) * cos(navstate.pos(1))]);
    navstate.pos(1:2) = navstate.pos(1:2) - ...
        DR_horizontal \ feedback_ne_m;
    [navstate.Rm, navstate.Rn] = getRmRn(navstate.pos(1), param);
    navstate.gravity = getGravity(navstate.pos);
end


function residual_m = horizontal_range_residual_ned( ...
    navstate, origin_lla, beacon_ned_m, measured_range_m)
    vehicle_ned_m = position_lla_to_ned(navstate.pos, origin_lla);
    residual_m = norm(vehicle_ned_m(1:2) - beacon_ned_m(1:2)) - ...
        measured_range_m;
end


function position_ned_m = position_lla_to_ned(position_lla, origin_lla)
    enu_m = pos2dxyz(position_lla', origin_lla);
    position_ned_m = [enu_m(2); enu_m(1); -enu_m(3)];
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
