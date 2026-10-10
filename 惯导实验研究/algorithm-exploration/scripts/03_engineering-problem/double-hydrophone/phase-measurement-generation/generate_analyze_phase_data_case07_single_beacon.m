clear; clc; close all;

%% 1. case-07 单信标相位差数据配置
% 本脚本只处理仿真 case-07：truth.txt + range1.txt。
% 默认一次生成 5/10/15 deg 三组结果；如只需一组，可改成标量，例如 5。
case_id = 7;
phase_std_values_deg = [5, 10, 15];
baseline_m = 3.0;
carrier_hz = 2e3;
sound_speed_mps = 1500;
phase_bias_deg = 0.0;
channel_delay_us = 0.0;
analysis_interval_s = 20;       % 候选角诊断间隔；<= 0 表示分析所有相位历元
random_seed = 10;               % 各相位误差工况共用同一组标准正态样本
show_figures = true;

truth_col.time = 2;
truth_col.lat = 3;
truth_col.lon = 4;
truth_col.yaw = 11;
range_col.time = 1;
range_col.beacon_lat = 4;
range_col.beacon_lon = 5;
range_col.beacon_height = 6;

%% 2. 定位工程与 case-07 输入目录
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(fileparts(script_dir))));
addpath(topic_dir);
paths = setup_inertial_experiment();

input_dir = paths.simulation_input(case_id);
truth_file = fullfile(input_dir, 'truth.txt');
range_file = fullfile(input_dir, 'range1.txt');
if ~isfile(truth_file)
    error('找不到 case-07 真值文件：%s', truth_file);
end
if ~isfile(range_file)
    error('找不到 case-07 单信标测距文件：%s', range_file);
end

%% 3. 读取并检查真值与单信标数据
truth = readmatrix(truth_file, 'FileType', 'text');
required_truth_col = max([truth_col.time, truth_col.lat, truth_col.lon, truth_col.yaw]);
if size(truth, 2) < required_truth_col
    error('truth.txt 列数不足，至少需要 %d 列。', required_truth_col);
end
valid_truth = all(isfinite(truth(:, ...
    [truth_col.time, truth_col.lat, truth_col.lon, truth_col.yaw])), 2);
truth = truth(valid_truth, :);
[truth_time, unique_truth_idx] = unique(truth(:, truth_col.time), 'stable');
truth = truth(unique_truth_idx, :);
if numel(truth_time) < 2
    error('truth.txt 中有效且不重复的历元不足。');
end
yaw_unwrapped_rad = unwrap(deg2rad(truth(:, truth_col.yaw)));

range_data = readmatrix(range_file, 'FileType', 'text');
required_range_col = max([range_col.time, range_col.beacon_lat, ...
    range_col.beacon_lon, range_col.beacon_height]);
if size(range_data, 2) < required_range_col
    error('range1.txt 列数不足，至少需要 %d 列。', required_range_col);
end
valid_range = all(isfinite(range_data(:, ...
    [range_col.time, range_col.beacon_lat, range_col.beacon_lon, ...
    range_col.beacon_height])), 2);
range_data = range_data(valid_range, :);
[~, unique_range_idx] = unique(range_data(:, range_col.time), 'stable');
range_data = range_data(unique_range_idx, :);
inside = range_data(:, range_col.time) >= truth_time(1) & ...
    range_data(:, range_col.time) <= truth_time(end);
if any(~inside)
    warning('range1.txt 有 %d 个历元超出真值时间范围，已忽略。', nnz(~inside));
end
range_data = range_data(inside, :);
if isempty(range_data)
    error('range1.txt 中没有位于真值时间范围内的有效数据。');
end

lambda_m = sound_speed_mps / carrier_hz;
k_limit = ceil(baseline_m / lambda_m) + 1;
k_search = -k_limit:k_limit;
fprintf(['case-07 单信标：%d 个相位历元，lambda = %.3f m，', ...
    'd/lambda = %.2f，候选周数 k = [%d, %d]\n'], ...
    size(range_data, 1), lambda_m, baseline_m / lambda_m, ...
    k_search(1), k_search(end));

summary_values = nan(numel(phase_std_values_deg), 9);

%% 4. 分别生成各相位误差工况
for std_index = 1:numel(phase_std_values_deg)
    phase_std_deg = phase_std_values_deg(std_index);
    if ~isscalar(phase_std_deg) || ~isfinite(phase_std_deg) || phase_std_deg < 0
        error('phase_std_values_deg 必须由非负有限标量组成。');
    end

    phase_config.carrier_hz = carrier_hz;
    phase_config.sound_speed_mps = sound_speed_mps;
    phase_config.phase_std_deg = phase_std_deg;
    phase_config.phase_bias_deg = phase_bias_deg;
    phase_config.channel_delay_us = channel_delay_us;

    % 每个工况重置随机数，使 5/10/15 deg 使用同一噪声序列的缩放版本。
    rng(random_seed, 'twister');
    phase_result = generate_single_beacon_phase(range_data, truth, ...
        truth_time, yaw_unwrapped_rad, truth_col, range_col, ...
        baseline_m, phase_config);

    [angle_result, candidate_matrix] = analyze_correct_candidate( ...
        phase_result, baseline_m, lambda_m, k_search, analysis_interval_s);

    output_dir = fullfile(input_dir, sprintf('phase-%gdeg', phase_std_deg));
    if ~isfolder(output_dir)
        mkdir(output_dir);
    end
    writematrix(phase_result, fullfile(output_dir, 'phase1.txt'), ...
        'Delimiter', ' ', 'FileType', 'text');
    writematrix(angle_result, fullfile(output_dir, 'arrival_angle1.txt'), ...
        'Delimiter', ' ', 'FileType', 'text');

    phase_error_deg = phase_result(:, 5);
    angle_error_deg = angle_result(:, 5);
    valid_angle_error = isfinite(angle_error_deg);
    summary_values(std_index, :) = [phase_std_deg, size(phase_result, 1), ...
        size(angle_result, 1), sqrt(mean(phase_error_deg .^ 2)), ...
        mean(angle_error_deg(valid_angle_error)), ...
        std(angle_error_deg(valid_angle_error)), ...
        sqrt(mean(angle_error_deg(valid_angle_error) .^ 2)), ...
        max(abs(angle_error_deg(valid_angle_error))), ...
        mean(angle_result(valid_angle_error, 7))];

    diagnostics = table(angle_result(:, 1), angle_result(:, 2), ...
        angle_result(:, 3), angle_result(:, 4), angle_result(:, 5), ...
        angle_result(:, 6), angle_result(:, 7), angle_result(:, 8), ...
        'VariableNames', {'Time_s', 'MeasuredPhase_deg', ...
        'TruePrincipalAngle_deg', 'SelectedCandidateAngle_deg', ...
        'SelectedAngleError_deg', 'SelectedCycleNumber', ...
        'CandidateCount', 'InjectedPhaseError_deg'});
    writetable(diagnostics, fullfile(output_dir, ...
        'single_beacon_candidate_diagnostics.txt'), ...
        'Delimiter', '\t', 'FileType', 'text');

    draw_diagnostics(phase_result, angle_result, candidate_matrix, ...
        phase_std_deg, output_dir, show_figures);

    fprintf(['Phase STD = %g deg：phase RMSE = %.3f deg，', ...
        '候选角误差 mean/std/RMSE/max = %.4f/%.4f/%.4f/%.4f deg\n'], ...
        phase_std_deg, summary_values(std_index, 4), ...
        summary_values(std_index, 5), summary_values(std_index, 6), ...
        summary_values(std_index, 7), summary_values(std_index, 8));
    fprintf('  已写入：%s\n', output_dir);
end

%% 5. 汇总文件
summary_table = array2table(summary_values, 'VariableNames', ...
    {'PhaseSTD_deg', 'PhaseEpochCount', 'AnalyzedEpochCount', ...
    'PhaseErrorRMSE_deg', 'AngleErrorMean_deg', 'AngleErrorSTD_deg', ...
    'AngleErrorRMSE_deg', 'AngleErrorMaxAbs_deg', 'MeanCandidateCount'});
summary_file = fullfile(input_dir, 'single_beacon_phase_generation_summary.txt');
writetable(summary_table, summary_file, 'Delimiter', '\t', 'FileType', 'text');
fprintf('case-07 单信标相位差数据生成完成。汇总：%s\n', summary_file);

%% 局部函数
function phase_output = generate_single_beacon_phase(range_data, truth, ...
        truth_time, yaw_unwrapped_rad, truth_col, range_col, baseline_m, config)
    time_s = range_data(:, range_col.time);
    vehicle_lat_deg = interp1(truth_time, truth(:, truth_col.lat), ...
        time_s, 'linear');
    vehicle_lon_deg = interp1(truth_time, truth(:, truth_col.lon), ...
        time_s, 'linear');
    heading_deg = rad2deg(interp1(truth_time, yaw_unwrapped_rad, ...
        time_s, 'linear'));

    beacon_lat_deg = rad2deg(range_data(:, range_col.beacon_lat));
    beacon_lon_deg = rad2deg(range_data(:, range_col.beacon_lon));
    source_bearing_deg = initial_bearing_deg(vehicle_lat_deg, ...
        vehicle_lon_deg, beacon_lat_deg, beacon_lon_deg);

    % 与原三信标脚本保持相同定义：阵列法线为 heading + 90 deg。
    angle_to_normal_deg = wrap180(source_bearing_deg - (heading_deg + 90));
    principal_true_deg = asind(sind(angle_to_normal_deg));
    path_difference_m = baseline_m .* sind(principal_true_deg);

    lambda_m = config.sound_speed_mps / config.carrier_hz;
    phase_true_unwrapped_deg = 360 .* path_difference_m ./ lambda_m;
    phase_true_wrapped_deg = wrap180(phase_true_unwrapped_deg);
    delay_phase_deg = 360 .* config.carrier_hz .* ...
        config.channel_delay_us .* 1e-6;
    injected_phase_error_deg = config.phase_bias_deg + delay_phase_deg + ...
        config.phase_std_deg .* randn(size(time_s));
    phase_measured_wrapped_deg = wrap180(phase_true_wrapped_deg + ...
        injected_phase_error_deg);

    % 列定义与 generate_analyze_phase_data.m 完全一致。
    phase_output = [time_s, phase_measured_wrapped_deg, ...
        phase_true_wrapped_deg, phase_true_unwrapped_deg, ...
        injected_phase_error_deg, path_difference_m, heading_deg, ...
        source_bearing_deg, angle_to_normal_deg, ...
        range_data(:, range_col.beacon_lat), ...
        range_data(:, range_col.beacon_lon), ...
        range_data(:, range_col.beacon_height)];
end

function [result, candidate_matrix] = analyze_correct_candidate( ...
        phase_data, baseline_m, lambda_m, k_search, interval_s)
    time_all = phase_data(:, 1);
    if interval_s <= 0
        selected_idx = (1:numel(time_all)).';
    else
        target_time = (time_all(1):interval_s:time_all(end)).';
        selected_idx = interp1(time_all, (1:numel(time_all)).', ...
            target_time, 'nearest', 'extrap');
        selected_idx = unique(selected_idx, 'stable');
    end

    selected_phase = phase_data(selected_idx, 2);
    true_unwrapped_phase = phase_data(selected_idx, 4);
    true_principal_angle = asind(sind(phase_data(selected_idx, 9)));
    n_epoch = numel(selected_idx);
    max_candidate_count = numel(k_search);
    candidate_matrix = nan(n_epoch, max_candidate_count);
    result = nan(n_epoch, 8);

    for ii = 1:n_epoch
        phase_unwrapped_candidates = selected_phase(ii) + 360 .* k_search;
        sin_candidates = phase_unwrapped_candidates .* lambda_m ./ ...
            (360 .* baseline_m);
        valid = abs(sin_candidates) <= 1 + 1e-12;
        valid_k = k_search(valid);
        candidate_angles = asind(max(-1, min(1, sin_candidates(valid))));

        [~, best_idx] = min(abs(phase_unwrapped_candidates(valid) - ...
            true_unwrapped_phase(ii)));
        selected_angle = candidate_angles(best_idx);
        angle_error = wrap180(selected_angle - true_principal_angle(ii));
        candidate_matrix(ii, 1:numel(candidate_angles)) = candidate_angles;
        result(ii, :) = [time_all(selected_idx(ii)), selected_phase(ii), ...
            true_principal_angle(ii), selected_angle, angle_error, ...
            valid_k(best_idx), numel(candidate_angles), phase_data(selected_idx(ii), 5)];
    end
end

function draw_diagnostics(phase_result, angle_result, candidate_matrix, ...
        phase_std_deg, output_dir, show_figures)
    if show_figures
        visibility = 'on';
    else
        visibility = 'off';
    end
    fig = figure('Color', 'w', 'Visible', visibility, ...
        'Name', sprintf('case-07 single beacon phase STD %g deg', phase_std_deg));
    tiledlayout(fig, 3, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    h_phase_true = plot(phase_result(:, 1), phase_result(:, 3), ...
        'k-', 'LineWidth', 1.0); hold on;
    h_phase_measured = plot(phase_result(:, 1), phase_result(:, 2), ...
        'b-', 'LineWidth', 0.7);
    grid on; xlabel('Time / s'); ylabel('Wrapped phase / deg');
    legend([h_phase_true, h_phase_measured], {'True', 'Measured'}, ...
        'Location', 'northeast');

    nexttile;
    plot(phase_result(:, 1), phase_result(:, 5), 'Color', [0.85, 0.20, 0.15]);
    yline(0, 'k:'); grid on; xlabel('Time / s'); ylabel('Phase error / deg');

    nexttile;
    candidate_time = repmat(angle_result(:, 1), 1, size(candidate_matrix, 2));
    h_candidates = plot(candidate_time, candidate_matrix, '.', ...
        'Color', [0.72, 0.72, 0.72], ...
        'MarkerSize', 5); hold on;
    h_true_angle = plot(angle_result(:, 1), angle_result(:, 3), ...
        'k-', 'LineWidth', 1.2);
    h_selected_angle = plot(angle_result(:, 1), angle_result(:, 4), ...
        'r.', 'MarkerSize', 8);
    grid on; xlabel('Time / s'); ylabel('Principal angle / deg');
    legend([h_candidates(1), h_true_angle, h_selected_angle], ...
        {'Candidates', 'True', 'Selected'}, 'Location', 'northeast');

    nexttile;
    plot(angle_result(:, 1), angle_result(:, 5), 'm.-');
    yline(0, 'k:'); grid on; xlabel('Time / s'); ylabel('Selected angle error / deg');

    nexttile;
    plot(angle_result(:, 3), angle_result(:, 4), 'b.'); hold on;
    lim = [-90, 90]; plot(lim, lim, 'k--');
    xlim(lim); ylim(lim); axis square; grid on;
    xlabel('True principal angle / deg'); ylabel('Selected angle / deg');

    nexttile;
    yyaxis left;
    plot(phase_result(:, 1), phase_result(:, 9), 'Color', [0.00, 0.45, 0.74]);
    ylabel('Angle to array normal / deg');
    yyaxis right;
    stairs(angle_result(:, 1), angle_result(:, 7), 'Color', [0.85, 0.33, 0.10]);
    ylabel('Candidate count'); grid on; xlabel('Time / s');

    sgtitle(sprintf('case-07 single beacon, phase STD = %g deg', phase_std_deg));
    exportgraphics(fig, fullfile(output_dir, 'single_beacon_phase_analysis.png'), ...
        'Resolution', 180);
    if ~show_figures
        close(fig);
    end
end

function bearing_deg = initial_bearing_deg(lat1_deg, lon1_deg, lat2_deg, lon2_deg)
    lat1 = deg2rad(lat1_deg);
    lon1 = deg2rad(lon1_deg);
    lat2 = deg2rad(lat2_deg);
    lon2 = deg2rad(lon2_deg);
    delta_lon = lon2 - lon1;
    y = sin(delta_lon) .* cos(lat2);
    x = cos(lat1) .* sin(lat2) - ...
        sin(lat1) .* cos(lat2) .* cos(delta_lon);
    bearing_deg = mod(rad2deg(atan2(y, x)), 360);
end

function angle_deg = wrap180(angle_deg)
    angle_deg = mod(angle_deg + 180, 360) - 180;
end
