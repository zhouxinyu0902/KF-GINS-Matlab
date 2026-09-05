clear; clc; close all;
%% 1. 数据集与文件配置
project_root = 'D:\Github\KF-GINS-Matlab';
dataset_mode = 'simulation';              % 'simulation' 或 'experiment'
case_name = 'case-00';                    % 实测示例可改为 'case-06'
% dataset_mode = 'experiment';              % 'simulation' 或 'experiment'
% case_name = 'case-06';                    % 实测示例可改为 'case-06'
beacon_count = 3;
switch lower(dataset_mode)
    case 'simulation'
        case_dir = fullfile(project_root, 'data', 'inertial-experiment', 'algorithm-exploration', 'input', 'simulation', case_name);
        truth_file = fullfile(case_dir, 'truth.txt');
    case 'experiment'
        case_dir = fullfile(project_root, 'data', 'inertial-experiment', 'algorithm-exploration', 'input', 'experiment', case_name);
        truth_file = fullfile(case_dir, 'truth.nav');
    otherwise
        error('dataset_mode 仅支持 simulation 或 experiment。');
end
if ~isfile(truth_file), error('找不到真值文件：%s', truth_file); end

%% 2. 数据列与双水听器参数
truth_col.time = 2; truth_col.lat = 3; truth_col.lon = 4; truth_col.yaw = 11;
range_col.time = 1; range_col.beacon_lat = 4; range_col.beacon_lon = 5; range_col.beacon_height = 6;
baseline_m = 3.0;
phase_config.carrier_hz = 2e3;
phase_config.sound_speed_mps = 1500;
phase_config.phase_std_deg = 5.0;
phase_config.phase_bias_deg = 0.0;
phase_config.channel_delay_us = 0.0;
measurement_interval_s = 120;             % 到达角分析周期；<=0 表示使用全部相位历元
show_figures = true;
rng(10);

%% 3. 读取并整理真值
truth = readmatrix(truth_file, 'FileType', 'text');
required_truth_col = max([truth_col.time, truth_col.lat, truth_col.lon, truth_col.yaw]);
if size(truth, 2) < required_truth_col, error('真值文件列数不足，至少需要 %d 列。', required_truth_col); end
valid_truth = all(isfinite(truth(:, [truth_col.time, truth_col.lat, truth_col.lon, truth_col.yaw])), 2);
truth = truth(valid_truth, :);
[truth_time, unique_idx] = unique(truth(:, truth_col.time));
truth = truth(unique_idx, :);
yaw_unwrapped_rad = unwrap(deg2rad(truth(:, truth_col.yaw)));

%% 4. 相位生成 + 正确候选角误差分析
lambda = phase_config.sound_speed_mps / phase_config.carrier_hz;
k_limit = ceil(baseline_m / lambda) + 1;
k_search = -k_limit:k_limit;
phase_results = cell(beacon_count, 1);
angle_results = cell(beacon_count, 1);
candidate_results = cell(beacon_count, 1);
statistics = nan(beacon_count, 6);
fprintf('数据集：%s/%s，lambda = %.3f m，d/lambda = %.2f，k = [%d, %d]\n', dataset_mode, case_name, lambda, baseline_m/lambda, k_search(1), k_search(end));

for beacon_index = 1:beacon_count
    range_file = fullfile(case_dir, sprintf('range%d.txt', beacon_index));
    if ~isfile(range_file), error('找不到距离文件：%s', range_file); end
    range_data = readmatrix(range_file, 'FileType', 'text');
    required_range_col = max([range_col.time, range_col.beacon_lat, range_col.beacon_lon, range_col.beacon_height]);
    if size(range_data, 2) < required_range_col, error('range%d.txt 列数不足，至少需要 %d 列。', beacon_index, required_range_col); end
    valid_range = all(isfinite(range_data(:, [range_col.time, range_col.beacon_lat, range_col.beacon_lon, range_col.beacon_height])), 2);
    range_data = range_data(valid_range, :);
    inside = range_data(:, range_col.time) >= truth_time(1) & range_data(:, range_col.time) <= truth_time(end);
    if any(~inside), warning('Beacon %d 有 %d 个 range 历元超出真值时间范围，已忽略。', beacon_index, nnz(~inside)); end
    range_data = range_data(inside, :);
    if isempty(range_data), error('Beacon %d 没有可用的 range 数据。', beacon_index); end

    phase_output = generate_phase_output(range_data, truth, truth_time, yaw_unwrapped_rad, truth_col, range_col, baseline_m, phase_config);
    phase_file = fullfile(case_dir, sprintf('phase%d.txt', beacon_index));
    writematrix(phase_output, phase_file, 'Delimiter', ' ');
    phase_results{beacon_index} = phase_output;

    [angle_output, candidate_angles, stat] = analyze_correct_candidate(phase_output, measurement_interval_s, baseline_m, lambda, k_search);
    angle_file = fullfile(case_dir, sprintf('arrival_angle%d.txt', beacon_index));
    writematrix(angle_output, angle_file, 'Delimiter', ' ');
    angle_results{beacon_index} = angle_output;
    candidate_results{beacon_index} = candidate_angles;
    statistics(beacon_index, :) = stat;

    fprintf('Beacon %d：phase=%d，angle=%d，相位STD=%.3f deg，角度STD=%.4f deg，RMSE=%.4f deg，Max=%.4f deg\n', ...
        beacon_index, size(phase_output, 1), size(angle_output, 1), stat(5), stat(2), stat(3), stat(4));
end

%% 5. 统计结果
statistics_table = array2table(statistics, 'VariableNames', {'AngleMean_deg','AngleSTD_deg','AngleRMSE_deg','AngleMax_deg','PhaseSTD_deg','MeanCandidateCount'}, 'RowNames', compose('Beacon%d', 1:beacon_count));
disp(statistics_table);

%% 6. 可视化
if show_figures
    figure('Color', 'w', 'Position', [100, 100, 1100, 750]);
    tiledlayout(beacon_count, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for beacon_index = 1:beacon_count
        data = phase_results{beacon_index};
        nexttile; hold on; grid on; box on;
        plot(data(:, 1), data(:, 3), 'LineWidth', 1.1);
        plot(data(:, 1), data(:, 2), '.', 'MarkerSize', 6);
        xlim([data(1, 1), data(end, 1)]);
        ylabel('\Delta\phi / deg');
        title(sprintf('Beacon %d', beacon_index));
        if beacon_index == 1, legend('真实包裹相位', '加噪包裹相位', 'Location', 'best'); end
        if beacon_index == beacon_count, xlabel('Time / s'); end
    end
    sgtitle(sprintf('%s/%s：双水听器相位数据，\\sigma_\\phi = %.1f^\\circ', dataset_mode, case_name, phase_config.phase_std_deg));

    % 相位差误差
    figure('Color', 'w', 'Position', [120, 120, 1100, 750]);
    tiledlayout(beacon_count, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    for beacon_index = 1:beacon_count
        data = phase_results{beacon_index};
        time = data(:, 1);
        phase_error_deg = wrap180(data(:, 2) - data(:, 3));

        nexttile; hold on; grid on; box on;
        plot(time, phase_error_deg, 'LineWidth', 1.0);
        yline(0, 'k--');
        xlim([time(1), time(end)]);

        mean_error = mean(phase_error_deg, 'omitnan');
        std_error = std(phase_error_deg, 'omitnan');
        rmse_error = sqrt(mean(phase_error_deg.^2, 'omitnan'));
        text(0.02, 0.95, sprintf('Mean = %.3f^\\circ\nSTD = %.3f^\\circ\nRMSE = %.3f^\\circ', mean_error, std_error, rmse_error), ...
            'Units', 'normalized', 'VerticalAlignment', 'top');

        ylabel('\delta\phi / deg');
        title(sprintf('Beacon %d：相位差误差', beacon_index));
        if beacon_index == beacon_count, xlabel('Time / s'); end
    end
    sgtitle(sprintf('%s/%s：双水听器相位差误差，设定 \\sigma_\\phi = %.1f^\\circ', dataset_mode, case_name, phase_config.phase_std_deg));
    figure('Color', 'w', 'Position', [120, 120, 1100, 750]);
    tiledlayout(beacon_count, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for beacon_index = 1:beacon_count
        data = angle_results{beacon_index};
        candidates = candidate_results{beacon_index};
        time = data(:, 1); theta_true = data(:, 3); theta_est = data(:, 4); angle_error = data(:, 5); phase_error = data(:, 8);

        nexttile; hold on; grid on; box on;
        for i = 1:numel(time)
            theta_candidate = candidates{i};
            plot(time(i)*ones(size(theta_candidate)), theta_candidate, '.', 'MarkerSize', 5, 'HandleVisibility', 'off');
        end
        h_candidate = plot(NaN, NaN, '.', 'MarkerSize', 5, 'DisplayName', '候选周期点');
        h_true = plot(time, theta_true, 'k-', 'LineWidth', 1.4, 'DisplayName', '真实到达角');
        h_est = plot(time, theta_est, 'r--', 'LineWidth', 1.0, 'DisplayName', '正确候选角');
        xlim([time(1), time(end)]); ylabel('\theta / deg'); title(sprintf('Beacon %d：候选角', beacon_index));
        if beacon_index == 1, legend([h_candidate, h_true, h_est], 'Location', 'best'); end
        if beacon_index == beacon_count, xlabel('Time / s'); end

        nexttile; hold on; grid on; box on;
        plot(time, angle_error, '.-', 'LineWidth', 0.8); yline(0, 'k--'); xlim([time(1), time(end)]);
        mean_error = mean(angle_error, 'omitnan'); std_error = std(angle_error, 'omitnan'); rmse_error = sqrt(mean(angle_error.^2, 'omitnan'));
        text(0.02, 0.95, sprintf('Mean = %.4f^\\circ\nSTD = %.4f^\\circ\nRMSE = %.4f^\\circ', mean_error, std_error, rmse_error), 'Units', 'normalized', 'VerticalAlignment', 'top');
        ylabel('\delta\theta / deg'); title(sprintf('Beacon %d：选对候选后的误差', beacon_index));
        if beacon_index == beacon_count, xlabel('Time / s'); end

        nexttile; hold on; grid on; box on;
        scatter(phase_error, angle_error, 12, 'filled');
        xlabel('\delta\phi / deg'); ylabel('\delta\theta / deg'); title(sprintf('Beacon %d：相位误差 → 角度误差', beacon_index));
    end
    sgtitle(sprintf('%s/%s：正确候选假设下的到达角误差，分析周期 = %.1f s', dataset_mode, case_name, measurement_interval_s));
end

%% 局部函数
function phase_output = generate_phase_output(range_data, truth, truth_time, yaw_unwrapped_rad, truth_col, range_col, baseline_m, cfg)
range_time = range_data(:, range_col.time);
center_lat_deg = interp1(truth_time, truth(:, truth_col.lat), range_time, 'linear');
center_lon_deg = interp1(truth_time, truth(:, truth_col.lon), range_time, 'linear');
heading_deg = mod(rad2deg(interp1(truth_time, yaw_unwrapped_rad, range_time, 'linear')), 360);
beacon_lat_rad = range_data(:, range_col.beacon_lat);
beacon_lon_rad = range_data(:, range_col.beacon_lon);
beacon_height_m = range_data(:, range_col.beacon_height);
beacon_lat_deg = rad2deg(beacon_lat_rad);
beacon_lon_deg = rad2deg(beacon_lon_rad);

source_bearing_deg = initial_bearing_deg(center_lat_deg, center_lon_deg, beacon_lat_deg, beacon_lon_deg);
angle_to_normal_deg = wrap180(source_bearing_deg - (heading_deg + 90));
theta_principal_deg = asind(sind(angle_to_normal_deg));
path_difference_m = baseline_m .* sind(theta_principal_deg);
lambda = cfg.sound_speed_mps / cfg.carrier_hz;
true_phase_unwrapped_deg = 360 .* path_difference_m ./ lambda;
true_phase_wrapped_deg = wrap180(true_phase_unwrapped_deg);
random_phase_error_deg = cfg.phase_std_deg .* randn(size(range_time));
delay_phase_deg = 360 * cfg.carrier_hz * cfg.channel_delay_us * 1e-6;
injected_phase_error_deg = random_phase_error_deg + cfg.phase_bias_deg + delay_phase_deg;
measured_phase_wrapped_deg = wrap180(true_phase_unwrapped_deg + injected_phase_error_deg);

phase_output = [range_time, measured_phase_wrapped_deg, true_phase_wrapped_deg, true_phase_unwrapped_deg, injected_phase_error_deg, path_difference_m, heading_deg, source_bearing_deg, angle_to_normal_deg, beacon_lat_rad, beacon_lon_rad, beacon_height_m];
end
function [angle_output, candidate_angles, stat] = analyze_correct_candidate(phase_output, measurement_interval_s, baseline_m, lambda, k_search)
time_all = phase_output(:, 1);
if measurement_interval_s > 0
    target_time = (time_all(1):measurement_interval_s:time_all(end)).';
    sample_index = interp1(time_all, (1:numel(time_all)).', target_time, 'nearest');
    sample_index = unique(round(sample_index), 'stable');
else
    sample_index = (1:numel(time_all)).';
end

data = phase_output(sample_index, :);
time = data(:, 1);
phase_meas_deg = data(:, 2);
true_phase_unwrapped_deg = data(:, 4);
phase_error_deg = data(:, 5);
theta_true_deg = asind(sind(data(:, 9)));

sample_count = numel(time);
theta_est_deg = nan(sample_count, 1);
selected_k = nan(sample_count, 1);
candidate_count = zeros(sample_count, 1);
candidate_angles = cell(sample_count, 1);

for i = 1:sample_count
    % 全部周期候选
    phase_candidates_deg = phase_meas_deg(i) + 360 .* k_search;
    sin_theta = lambda .* phase_candidates_deg ./ (360 * baseline_m);
    valid = abs(sin_theta) <= 1;
    candidate_angles{i} = asind(sin_theta(valid));
    candidate_count(i) = nnz(valid);

    % 利用真实未包裹相位确定“正确周期”
    correct_k = round((true_phase_unwrapped_deg(i) - phase_meas_deg(i)) / 360);
    selected_k(i) = correct_k;

    % 正确周期对应的含噪未包裹相位
    correct_phase_deg = phase_meas_deg(i) + 360 * correct_k;
    sin_theta_correct = lambda * correct_phase_deg / (360 * baseline_m);

    % 即使周期正确，相位噪声也可能使测角方程无实数解
    if abs(sin_theta_correct) <= 1
        theta_est_deg(i) = asind(sin_theta_correct);
    end
end

angle_error_deg = theta_est_deg - theta_true_deg;

angle_output = [time, phase_meas_deg, theta_true_deg, theta_est_deg, ...
    angle_error_deg, selected_k, candidate_count, phase_error_deg];

mean_error = mean(angle_error_deg, 'omitnan');
std_error = std(angle_error_deg, 'omitnan');
rmse_error = sqrt(mean(angle_error_deg.^2, 'omitnan'));
max_error = max(abs(angle_error_deg), [], 'omitnan');
phase_std = std(phase_error_deg, 'omitnan');
mean_candidate_num = mean(candidate_count, 'omitnan');

stat = [mean_error, std_error, rmse_error, max_error, phase_std, mean_candidate_num];
end
function bearing_deg = initial_bearing_deg(lat1_deg, lon1_deg, lat2_deg, lon2_deg)
lat1 = deg2rad(lat1_deg); lat2 = deg2rad(lat2_deg); dlon = deg2rad(lon2_deg - lon1_deg);
y = sin(dlon) .* cos(lat2);
x = cos(lat1) .* sin(lat2) - sin(lat1) .* cos(lat2) .* cos(dlon);
bearing_deg = mod(rad2deg(atan2(y, x)), 360);
end

function angle_deg = wrap180(angle_deg)
angle_deg = mod(angle_deg + 180, 360) - 180;
end
