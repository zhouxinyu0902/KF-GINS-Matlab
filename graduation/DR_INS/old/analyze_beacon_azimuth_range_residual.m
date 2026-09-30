%% 单信标方位角扫描：同一 INS/DVL 轨迹的水平距离残差表现
% 本脚本不重新运行导航滤波，而是固定无距离辅助的
% INS-DVL-Depth-Joint 轨迹，仅改变单信标相对航迹中心的方位角。
%
% 方位角约定：0 deg 为北，90 deg 为东，顺时针为正。
% 主分析量：
%   exact residual = |p_est-b| - |p_true-b|
%   radial error   = u' * (p_est-p_true)
% 其中 u 为信标指向载体真值位置的水平单位视线向量。

clear;
close all;
clc;

study_dir = fileparts(mfilename('fullpath'));
data_file = fullfile(study_dir, 'data_no_range', 'simulation_data.mat');
result_file = fullfile(study_dir, 'output_route_a_joint_update', ...
    'route_a_joint_update_result.mat');
if ~isfile(data_file)
    error('缺少 simulation_data.mat，请先运行 simulate_dr_ins_data.m。');
end
if ~isfile(result_file)
    error(['缺少无距离 INS/DVL 结果，请先运行 ', ...
        'run_route_a_joint_dvl_depth_range.m。']);
end

data = load(data_file, 'cfg', 'truth', 'dvl', 'imu');
joint = load(result_file, 'results', 'beacon_ned_m');
cfg = data.cfg;
truth = data.truth;
dvl = data.dvl;
imu = data.imu;

result_names = string({joint.results.name});
no_range_result_index = find(result_names == "INS-DVL-Depth-Joint", 1);
if isempty(no_range_result_index)
    error('结果文件中没有找到 INS-DVL-Depth-Joint。');
end
no_range_result = joint.results(no_range_result_index);

%% 扫描设置
scan_cfg.azimuth_deg = (0:5:355)';
scan_cfg.range_interval_s = 8.0;
scan_cfg.range_noise_std_m = 5.0;
scan_cfg.range_noise_seed = cfg.random_seed + 1800;
scan_cfg.radial_margin_m = 500.0;
scan_cfg.blind_cos_threshold = 0.2;

range_stride = round(scan_cfg.range_interval_s / cfg.imu_ts_s);
if abs(range_stride * cfg.imu_ts_s - scan_cfg.range_interval_s) > 1e-12
    error('距离更新间隔必须是 IMU 周期的整数倍。');
end
range_imu_index = (1 + range_stride:range_stride:size(imu, 1))';
[is_matched, range_dvl_index] = ismember(range_imu_index, dvl.imu_index);
if ~all(is_matched)
    error('存在没有同步 DVL 输出的距离历元。');
end
range_time_s = imu(range_imu_index, 1);

truth_xy_m = truth.position_ned_m(range_imu_index, 1:2);
estimate_xy_m = no_range_result.position_ned_m(range_dvl_index, 1:2);
position_error_m = estimate_xy_m - truth_xy_m;
horizontal_error_m = vecnorm(position_error_m, 2, 2);

north_limits_m = [min(truth.position_ned_m(:, 1)), ...
    max(truth.position_ned_m(:, 1))];
east_limits_m = [min(truth.position_ned_m(:, 2)), ...
    max(truth.position_ned_m(:, 2))];
scan_center_ne_m = [mean(north_limits_m), mean(east_limits_m)];
distance_from_center_m = vecnorm( ...
    truth.position_ned_m(:, 1:2) - scan_center_ne_m, 2, 2);
scan_radius_m = max(distance_from_center_m) + scan_cfg.radial_margin_m;

azimuth_deg = scan_cfg.azimuth_deg;
azimuth_count = numel(azimuth_deg);
beacon_ne_m = scan_center_ne_m + scan_radius_m * ...
    [cosd(azimuth_deg), sind(azimuth_deg)];

rng(scan_cfg.range_noise_seed, 'twister');
common_range_noise_m = scan_cfg.range_noise_std_m * ...
    randn(numel(range_time_s), 1);

%% 对每个候选方位计算精确距离残差和一阶径向投影
sample_count = numel(range_time_s);
exact_residual_m = zeros(sample_count, azimuth_count);
noisy_innovation_m = zeros(sample_count, azimuth_count);
radial_error_m = zeros(sample_count, azimuth_count);
tangential_error_m = zeros(sample_count, azimuth_count);
abs_cos_theta = zeros(sample_count, azimuth_count);

rms_geometric_residual_m = zeros(azimuth_count, 1);
rms_noisy_innovation_m = zeros(azimuth_count, 1);
mean_abs_residual_m = zeros(azimuth_count, 1);
max_abs_residual_m = zeros(azimuth_count, 1);
rms_radial_error_m = zeros(azimuth_count, 1);
rms_tangential_error_m = zeros(azimuth_count, 1);
mean_abs_cos_theta = zeros(azimuth_count, 1);
blind_angle_fraction = zeros(azimuth_count, 1);
below_one_sigma_fraction = zeros(azimuth_count, 1);
linearization_gap_rms_m = zeros(azimuth_count, 1);
information_lambda_min = zeros(azimuth_count, 1);
information_condition_number = zeros(azimuth_count, 1);

for azimuth_index = 1:azimuth_count
    beacon = beacon_ne_m(azimuth_index, :);
    truth_los_m = truth_xy_m - beacon;
    truth_range_m = vecnorm(truth_los_m, 2, 2);
    estimated_range_m = vecnorm(estimate_xy_m - beacon, 2, 2);
    unit_los = truth_los_m ./ truth_range_m;

    exact_residual_m(:, azimuth_index) = ...
        estimated_range_m - truth_range_m;
    noisy_innovation_m(:, azimuth_index) = ...
        exact_residual_m(:, azimuth_index) - common_range_noise_m;
    radial_error_m(:, azimuth_index) = ...
        sum(unit_los .* position_error_m, 2);
    tangential_error_m(:, azimuth_index) = ...
        -unit_los(:, 2) .* position_error_m(:, 1) + ...
         unit_los(:, 1) .* position_error_m(:, 2);
    abs_cos_theta(:, azimuth_index) = abs( ...
        radial_error_m(:, azimuth_index) ./ max(horizontal_error_m, eps));

    residual = exact_residual_m(:, azimuth_index);
    rms_geometric_residual_m(azimuth_index) = sqrt(mean(residual.^2));
    rms_noisy_innovation_m(azimuth_index) = sqrt(mean( ...
        noisy_innovation_m(:, azimuth_index).^2));
    mean_abs_residual_m(azimuth_index) = mean(abs(residual));
    max_abs_residual_m(azimuth_index) = max(abs(residual));
    rms_radial_error_m(azimuth_index) = sqrt(mean( ...
        radial_error_m(:, azimuth_index).^2));
    rms_tangential_error_m(azimuth_index) = sqrt(mean( ...
        tangential_error_m(:, azimuth_index).^2));
    mean_abs_cos_theta(azimuth_index) = mean( ...
        abs_cos_theta(:, azimuth_index));
    blind_angle_fraction(azimuth_index) = mean( ...
        abs_cos_theta(:, azimuth_index) < scan_cfg.blind_cos_threshold);
    below_one_sigma_fraction(azimuth_index) = mean( ...
        abs(residual) < scan_cfg.range_noise_std_m);
    linearization_gap_rms_m(azimuth_index) = sqrt(mean( ...
        (residual - radial_error_m(:, azimuth_index)).^2));

    information_matrix = (unit_los' * unit_los) / ...
        scan_cfg.range_noise_std_m^2;
    information_eigenvalues = eig(information_matrix);
    information_lambda_min(azimuth_index) = min(information_eigenvalues);
    information_condition_number(azimuth_index) = cond(information_matrix);
end

%% 汇总最强、最弱及当前信标方向
[~, strongest_index] = max(rms_geometric_residual_m);
[~, weakest_index] = min(rms_geometric_residual_m);
[~, best_geometry_index] = max(information_lambda_min);

if isfield(joint, 'beacon_ned_m')
    current_beacon_ne_m = joint.beacon_ned_m(1:2)';
    current_offset_ne_m = current_beacon_ne_m - scan_center_ne_m;
    current_beacon_azimuth_deg = mod(atan2d( ...
        current_offset_ne_m(2), current_offset_ne_m(1)), 360);
    [~, current_azimuth_index] = min(abs(wrap_to_180( ...
        azimuth_deg - current_beacon_azimuth_deg)));
else
    current_beacon_ne_m = [nan, nan];
    current_beacon_azimuth_deg = nan;
    current_azimuth_index = [];
end

summary_table = table(azimuth_deg, beacon_ne_m(:, 1), beacon_ne_m(:, 2), ...
    repmat(scan_radius_m, azimuth_count, 1), ...
    rms_geometric_residual_m, rms_noisy_innovation_m, ...
    mean_abs_residual_m, max_abs_residual_m, ...
    rms_radial_error_m, rms_tangential_error_m, ...
    mean_abs_cos_theta, 100 * blind_angle_fraction, ...
    100 * below_one_sigma_fraction, linearization_gap_rms_m, ...
    information_lambda_min, information_condition_number, ...
    'VariableNames', {'Azimuth_deg', 'BeaconNorth_m', 'BeaconEast_m', ...
    'Radius_m', 'RMSGeometricResidual_m', 'RMSNoisyInnovation_m', ...
    'MeanAbsResidual_m', 'MaxAbsResidual_m', 'RMSRadialError_m', ...
    'RMSTangentialError_m', 'MeanAbsCosTheta', ...
    'BlindAngleFraction_percent', 'BelowOneSigmaFraction_percent', ...
    'LinearizationGapRMS_m', 'InformationLambdaMin', ...
    'InformationConditionNumber'});

%% 保存结果
output_dir = fullfile(study_dir, 'output_beacon_azimuth_scan');
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end
writetable(summary_table, fullfile(output_dir, ...
    'beacon_azimuth_residual_summary.csv'));
writetable(summary_table, fullfile(output_dir, ...
    'beacon_azimuth_residual_summary.xlsx'));

representative_table = table(range_time_s, horizontal_error_m, ...
    exact_residual_m(:, strongest_index), ...
    exact_residual_m(:, weakest_index), ...
    exact_residual_m(:, current_azimuth_index), ...
    'VariableNames', {'Time_s', 'HorizontalPositionError_m', ...
    'StrongestAzimuthResidual_m', 'WeakestAzimuthResidual_m', ...
    'CurrentNearestAzimuthResidual_m'});
writetable(representative_table, fullfile(output_dir, ...
    'representative_residual_timeseries.csv'));

save(fullfile(output_dir, 'beacon_azimuth_residual_scan.mat'), ...
    'scan_cfg', 'scan_center_ne_m', 'scan_radius_m', 'azimuth_deg', ...
    'beacon_ne_m', 'range_time_s', 'position_error_m', ...
    'horizontal_error_m', 'exact_residual_m', 'noisy_innovation_m', ...
    'radial_error_m', 'tangential_error_m', 'abs_cos_theta', ...
    'summary_table', 'strongest_index', 'weakest_index', ...
    'best_geometry_index', 'current_beacon_ne_m', ...
    'current_beacon_azimuth_deg', 'current_azimuth_index', '-v7.3');

%% 图 1：方位角—距离残差 RMS 极坐标图
polar_fig = figure('Color', 'w');
polar_ax = polaraxes(polar_fig);
polarplot(polar_ax, deg2rad([azimuth_deg; azimuth_deg(1)]), ...
    [rms_geometric_residual_m; rms_geometric_residual_m(1)], ...
    'LineWidth', 1.6, 'Color', [0.10, 0.35, 0.70], ...
    'DisplayName', 'Residual RMS');
hold(polar_ax, 'on');
polarplot(polar_ax, deg2rad(azimuth_deg(strongest_index)), ...
    rms_geometric_residual_m(strongest_index), 'ro', ...
    'MarkerFaceColor', 'r', 'DisplayName', 'Strongest residual');
polarplot(polar_ax, deg2rad(azimuth_deg(weakest_index)), ...
    rms_geometric_residual_m(weakest_index), 'ko', ...
    'MarkerFaceColor', 'k', 'DisplayName', 'Weakest residual');
polar_ax.ThetaZeroLocation = 'top';
polar_ax.ThetaDir = 'clockwise';
title(polar_ax, sprintf('Geometric range residual RMS, radius %.1f m', ...
    scan_radius_m));
legend(polar_ax, 'Location', 'southoutside');
exportgraphics(polar_fig, fullfile(output_dir, ...
    '01_residual_rms_polar.png'), 'Resolution', 200);

%% 图 2：方位角—时间距离残差热图
heatmap_fig = figure('Color', 'w');
imagesc(range_time_s / 60, azimuth_deg, exact_residual_m');
axis xy;
xlabel('Time (min)');
ylabel('Beacon azimuth (deg)');
title('Signed geometric horizontal-range residual (m)');
colorbar;
colormap(red_blue_colormap(256));
color_limit = max(abs(exact_residual_m), [], 'all');
if color_limit > 0
    clim([-color_limit, color_limit]);
end
exportgraphics(heatmap_fig, fullfile(output_dir, ...
    '02_residual_azimuth_time_heatmap.png'), 'Resolution', 200);

%% 图 3：代表方位的残差时序
timeseries_fig = figure('Color', 'w');
plot(range_time_s / 60, exact_residual_m(:, strongest_index), ...
    '-', 'LineWidth', 1.3, ...
    'DisplayName', sprintf('Strongest: %.0f deg', ...
    azimuth_deg(strongest_index)));
hold on;
plot(range_time_s / 60, exact_residual_m(:, weakest_index), ...
    '--', 'LineWidth', 1.3, ...
    'DisplayName', sprintf('Weakest: %.0f deg', ...
    azimuth_deg(weakest_index)));
plot(range_time_s / 60, exact_residual_m(:, current_azimuth_index), ...
    ':', 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Nearest to current: %.0f deg', ...
    azimuth_deg(current_azimuth_index)));
yline(scan_cfg.range_noise_std_m, 'k-.', '5 m noise std', ...
    'HandleVisibility', 'off');
yline(-scan_cfg.range_noise_std_m, 'k-.', ...
    'HandleVisibility', 'off');
grid on;
box on;
xlabel('Time (min)');
ylabel('Geometric range residual (m)');
legend('Location', 'best');
exportgraphics(timeseries_fig, fullfile(output_dir, ...
    '03_representative_residual_timeseries.png'), 'Resolution', 200);

%% 图 4：候选信标在轨迹周围的分布
geometry_fig = figure('Color', 'w');
plot(truth.position_ned_m(:, 2), truth.position_ned_m(:, 1), ...
    'k-', 'LineWidth', 1.4, 'DisplayName', 'Truth trajectory');
hold on;
scatter(beacon_ne_m(:, 2), beacon_ne_m(:, 1), 45, ...
    rms_geometric_residual_m, 'filled', 'DisplayName', 'Candidate beacons');
plot(scan_center_ne_m(2), scan_center_ne_m(1), 'kx', ...
    'MarkerSize', 10, 'LineWidth', 1.5, 'DisplayName', 'Scan center');
if all(isfinite(current_beacon_ne_m))
    plot(current_beacon_ne_m(2), current_beacon_ne_m(1), 'p', ...
        'MarkerSize', 12, 'MarkerFaceColor', [0.95, 0.65, 0.10], ...
        'MarkerEdgeColor', 'k', 'DisplayName', 'Current beacon');
end
axis equal;
grid on;
box on;
xlabel('East (m)');
ylabel('North (m)');
title('Beacon azimuth scan; color = residual RMS (m)');
colorbar;
legend('Location', 'best');
exportgraphics(geometry_fig, fullfile(output_dir, ...
    '04_beacon_scan_geometry.png'), 'Resolution', 200);

%% 图 5：可观测性辅助指标
metric_fig = figure('Color', 'w');
tiledlayout(2, 1, 'TileSpacing', 'compact');
nexttile;
plot(azimuth_deg, mean_abs_cos_theta, 'LineWidth', 1.4);
hold on;
yline(scan_cfg.blind_cos_threshold, 'r--', '|cos(theta)|=0.2');
grid on;
xlim([0, 355]);
ylabel('Mean |cos(theta)|');
nexttile;
plot(azimuth_deg, 100 * below_one_sigma_fraction, ...
    'LineWidth', 1.4, 'DisplayName', '|residual| < 5 m');
hold on;
plot(azimuth_deg, 100 * blind_angle_fraction, '--', ...
    'LineWidth', 1.4, 'DisplayName', '|cos(theta)| < 0.2');
grid on;
xlim([0, 355]);
xlabel('Beacon azimuth (deg)');
ylabel('Epoch fraction (%)');
legend('Location', 'best');
exportgraphics(metric_fig, fullfile(output_dir, ...
    '05_observability_metrics.png'), 'Resolution', 200);

fprintf('\n信标方位角扫描完成，结果目录：%s\n', output_dir);
fprintf('扫描圆心 [N,E] = [%.1f, %.1f] m，固定半径 %.1f m。\n', ...
    scan_center_ne_m(1), scan_center_ne_m(2), scan_radius_m);
fprintf('距离残差 RMS 最大：%.0f deg，%.3f m。\n', ...
    azimuth_deg(strongest_index), ...
    rms_geometric_residual_m(strongest_index));
fprintf('距离残差 RMS 最小：%.0f deg，%.3f m。\n', ...
    azimuth_deg(weakest_index), ...
    rms_geometric_residual_m(weakest_index));
fprintf('信息矩阵最小特征值最大：%.0f deg，%.6f。\n', ...
    azimuth_deg(best_geometry_index), ...
    information_lambda_min(best_geometry_index));
fprintf('当前信标相对扫描中心方位：%.2f deg；最近扫描角：%.0f deg。\n', ...
    current_beacon_azimuth_deg, azimuth_deg(current_azimuth_index));

ranked_table = sortrows(summary_table, 'RMSGeometricResidual_m', 'descend');
disp(ranked_table(1:min(5, height(ranked_table)), ...
    {'Azimuth_deg', 'RMSGeometricResidual_m', ...
    'MeanAbsCosTheta', 'BelowOneSigmaFraction_percent', ...
    'InformationLambdaMin'}));


function angle_deg = wrap_to_180(angle_deg)
    angle_deg = mod(angle_deg + 180, 360) - 180;
end


function colors = red_blue_colormap(color_count)
    half_count = floor(color_count / 2);
    lower = [linspace(0.10, 1.00, half_count)', ...
        linspace(0.35, 1.00, half_count)', ...
        linspace(0.75, 1.00, half_count)'];
    upper_count = color_count - half_count;
    upper = [linspace(1.00, 0.80, upper_count)', ...
        linspace(1.00, 0.10, upper_count)', ...
        linspace(1.00, 0.10, upper_count)'];
    colors = [lower; upper];
end
