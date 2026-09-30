%% Step 5：相同方位角、不同移动信标起点的对比
% 本脚本研究：保持信标相对航向方位角不变时，改变信标轨迹起点
% （等价于改变信标与载体的恒定距离），距离残差和辅助效果是否变化。
%
% 只保留两个最有代表性的方位：
%   1）DVL 与罗盘正误差的一阶残差抵消方向；
%   2）DVL 与罗盘正误差的最大绝对残差方向。
%
% 每个方向设置 1000 m、3000 m、5000 m 三种起始距离。
% 信标随后与真实载体平行移动，因此相对方位和距离全程保持不变。
%
% 一阶理论中，距离残差 delta_rho = u' * delta_p，仅与视线方向有关，
% 与信标距离无关；不同距离之间的差别主要来自精确距离的非线性项。

clear;
close all;
clc;

study_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(study_dir, 'generated_data');
step2_file = fullfile(data_dir, 'step2_error_propagation_results.mat');
step3_file = fullfile(data_dir, 'step3_beacon_bearing_sweep_results.mat');

if ~exist(step2_file, 'file') || ~exist(step3_file, 'file')
    error(['未找到 Step2 或 Step3 结果，请先依次运行 ', ...
        'Step1、Step2 和 Step3。']);
end

step2_data = load(step2_file, 't', 'p_true', 'position_error_exact', ...
    'dvl_scale_error', 'compass_error_deg');
step3_data = load(step3_file, 'relative_bearing_deg', ...
    'total_plus_theory', 'blind_angle_plus');

t = step2_data.t;
p_true = step2_data.p_true;
dvl_scale_error = step2_data.dvl_scale_error;
compass_error_deg = step2_data.compass_error_deg;

% Step2 的第 4 个工况为 DVL 正刻度误差与罗盘正航向误差的组合。
p_dr = p_true + step2_data.position_error_exact(:, :, 4);
error_dr_m = sqrt(sum((p_dr - p_true).^2, 2));

%% 从 Step3 自动读取抵消方向和最大绝对残差方向
blind_angle = step3_data.blind_angle_plus;
blind_angle = blind_angle(blind_angle >= 0 & blind_angle < 180);
if isempty(blind_angle)
    error('Step3 结果中未找到 0°至180°范围内的一阶残差抵消方向。');
end
cancellation_bearing_deg = blind_angle(1);

[~, maximum_index] = max(abs(step3_data.total_plus_theory));
maximum_residual_bearing_deg = ...
    step3_data.relative_bearing_deg(maximum_index);

bearing_deg = [cancellation_bearing_deg, maximum_residual_bearing_deg];
bearing_name = { ...
    sprintf('残差抵消方向 %.2f°', cancellation_bearing_deg), ...
    sprintf('最大绝对残差方向 %.2f°', maximum_residual_bearing_deg)};
bearing_number = numel(bearing_deg);

%% 设置不同的信标轨迹起点
% 在相同方位角下，改变初始距离就会得到彼此平行、起点不同的信标轨迹。
beacon_range_m = [1000, 3000, 5000];
range_case_number = numel(beacon_range_m);

measurement_interval_s = 10;
range_noise_std_m = 1;
position_process_noise_m_sqrt_s = 0.08;
sample_time_s = median(diff(t));
sample_count = size(p_true, 1);
measurement_stride = round(measurement_interval_s / sample_time_s);
if abs(measurement_stride * sample_time_s - measurement_interval_s) > 1e-10
    error('测距周期必须是轨迹采样周期的整数倍。');
end

track_direction = p_true(end, :) - p_true(1, :);
track_direction = track_direction / norm(track_direction);
cross_direction = [track_direction(2), -track_direction(1)];

% 所有工况使用同一组测距噪声，只比较方位和信标起点的影响。
rng(2013, 'twister');
common_range_noise_m = range_noise_std_m * randn(sample_count, 1);

beacon_xy = zeros(sample_count, 2, bearing_number, range_case_number);
p_aided = zeros(sample_count, 2, bearing_number, range_case_number);
raw_range_residual_m = zeros(sample_count, bearing_number, range_case_number);
aided_range_residual_m = zeros(sample_count, bearing_number, range_case_number);
error_aided_m = zeros(sample_count, bearing_number, range_case_number);
maximum_bearing_deviation_deg = zeros(bearing_number, range_case_number);

%% 生成信标轨迹并进行二维位置 EKF
for bearing_index = 1:bearing_number
    line_of_sight_direction = ...
        cosd(bearing_deg(bearing_index)) * track_direction + ...
        sind(bearing_deg(bearing_index)) * cross_direction;

    for range_index = 1:range_case_number
        current_range_m = beacon_range_m(range_index);
        current_beacon_xy = p_true - ...
            current_range_m * line_of_sight_direction;
        beacon_xy(:, :, bearing_index, range_index) = current_beacon_xy;

        true_range_vector = p_true - current_beacon_xy;
        true_range_m = sqrt(sum(true_range_vector.^2, 2));
        line_of_sight = true_range_vector ./ true_range_m;

        actual_bearing_deg = mod(atan2d( ...
            line_of_sight * cross_direction', ...
            line_of_sight * track_direction'), 360);
        near_full_circle = actual_bearing_deg > 359.999999;
        actual_bearing_deg(near_full_circle) = 0;
        maximum_bearing_deviation_deg(bearing_index, range_index) = ...
            max(abs(actual_bearing_deg - bearing_deg(bearing_index)));

        raw_range = sqrt(sum((p_dr - current_beacon_xy).^2, 2));
        raw_range_residual_m(:, bearing_index, range_index) = ...
            raw_range - true_range_m;

        [current_p_aided, current_aided_residual] = position_range_ekf( ...
            p_true, p_dr, current_beacon_xy, true_range_m, ...
            common_range_noise_m, sample_time_s, measurement_stride, ...
            range_noise_std_m, position_process_noise_m_sqrt_s);

        p_aided(:, :, bearing_index, range_index) = current_p_aided;
        aided_range_residual_m(:, bearing_index, range_index) = ...
            current_aided_residual;
        error_aided_m(:, bearing_index, range_index) = sqrt(sum(( ...
            current_p_aided - p_true).^2, 2));
    end
end

%% 生成汇总表
row_number = bearing_number * range_case_number;
direction_column = cell(row_number, 1);
bearing_column_deg = zeros(row_number, 1);
range_column_m = zeros(row_number, 1);
beacon_start_e_m = zeros(row_number, 1);
beacon_start_n_m = zeros(row_number, 1);
bearing_deviation_column_deg = zeros(row_number, 1);
raw_residual_rmse_m = zeros(row_number, 1);
raw_final_residual_m = zeros(row_number, 1);
dr_rmse_m = sqrt(mean(error_dr_m.^2)) * ones(row_number, 1);
aided_rmse_m = zeros(row_number, 1);
aided_final_error_m = zeros(row_number, 1);
rmse_improvement_percent = zeros(row_number, 1);

row = 0;
for bearing_index = 1:bearing_number
    for range_index = 1:range_case_number
        row = row + 1;
        current_residual = raw_range_residual_m(:, bearing_index, range_index);
        current_error = error_aided_m(:, bearing_index, range_index);

        direction_column{row} = bearing_name{bearing_index};
        bearing_column_deg(row) = bearing_deg(bearing_index);
        range_column_m(row) = beacon_range_m(range_index);
        beacon_start_e_m(row) = beacon_xy(1, 1, bearing_index, range_index);
        beacon_start_n_m(row) = beacon_xy(1, 2, bearing_index, range_index);
        bearing_deviation_column_deg(row) = ...
            maximum_bearing_deviation_deg(bearing_index, range_index);
        raw_residual_rmse_m(row) = sqrt(mean(current_residual.^2));
        raw_final_residual_m(row) = current_residual(end);
        aided_rmse_m(row) = sqrt(mean(current_error.^2));
        aided_final_error_m(row) = current_error(end);
        rmse_improvement_percent(row) = 100 * ...
            (dr_rmse_m(row) - aided_rmse_m(row)) / dr_rmse_m(row);
    end
end

summary_table = table(direction_column, bearing_column_deg, range_column_m, ...
    beacon_start_e_m, beacon_start_n_m, bearing_deviation_column_deg, ...
    raw_residual_rmse_m, raw_final_residual_m, dr_rmse_m, aided_rmse_m, ...
    aided_final_error_m, rmse_improvement_percent, ...
    'VariableNames', {'方位类型', '恒定方位角_deg', '恒定距离_m', ...
    '信标起点东向_m', '信标起点北向_m', '最大方位偏差_deg', ...
    '纯DR距离残差RMSE_m', '纯DR终点距离残差_m', ...
    '纯DR位置RMSE_m', '距离辅助位置RMSE_m', ...
    '距离辅助终点误差_m', 'RMSE改善率_percent'});
disp(summary_table);

%% 图1：真实载体、纯 DR 与不同起点的信标轨迹
colors = lines(range_case_number);
figure('Name', '不同信标起点的轨迹对比', ...
    'Color', 'w', 'Position', [80, 100, 1180, 500]);

for bearing_index = 1:bearing_number
    subplot(1, bearing_number, bearing_index);
    plot(p_true(:, 1), p_true(:, 2), 'k-', 'LineWidth', 1.8, ...
        'DisplayName', '真实载体轨迹');
    hold on;
    plot(p_dr(:, 1), p_dr(:, 2), 'k--', 'LineWidth', 1.2, ...
        'DisplayName', '纯 DR 轨迹');

    for range_index = 1:range_case_number
        current_beacon_xy = beacon_xy(:, :, bearing_index, range_index);
        plot(current_beacon_xy(:, 1), current_beacon_xy(:, 2), ...
            'Color', colors(range_index, :), 'LineWidth', 1.4, ...
            'DisplayName', sprintf('信标轨迹：%d m', ...
            beacon_range_m(range_index)));
        plot(current_beacon_xy(1, 1), current_beacon_xy(1, 2), 'o', ...
            'Color', colors(range_index, :), 'MarkerFaceColor', 'w', ...
            'HandleVisibility', 'off');
        plot(current_beacon_xy(end, 1), current_beacon_xy(end, 2), 'p', ...
            'Color', colors(range_index, :), ...
            'MarkerFaceColor', colors(range_index, :), ...
            'MarkerSize', 9, 'HandleVisibility', 'off');
    end

    grid on;
    axis equal;
    axis padded;
    xlabel('东向 / m');
    ylabel('北向 / m');
    title(bearing_name{bearing_index});
    legend('Location', 'best');
end

%% 图2：不同信标起点下的距离残差和辅助效果
figure('Name', '不同信标起点的残差与辅助效果', ...
    'Color', 'w', 'Position', [80, 60, 1150, 800]);

for bearing_index = 1:bearing_number
    subplot(bearing_number, 2, 2 * bearing_index - 1);
    hold on;
    for range_index = 1:range_case_number
        plot(t, raw_range_residual_m(:, bearing_index, range_index), ...
            'Color', colors(range_index, :), 'LineWidth', 1.3, ...
            'DisplayName', sprintf('信标距离 %d m', ...
            beacon_range_m(range_index)));
    end
    yline(0, 'k:', 'HandleVisibility', 'off');
    grid on;
    xlabel('时间 / s');
    ylabel('纯 DR 距离残差 / m');
    title([bearing_name{bearing_index}, '：距离残差']);
    legend('Location', 'best');

    subplot(bearing_number, 2, 2 * bearing_index);
    plot(t, error_dr_m, 'k--', 'LineWidth', 1.5, ...
        'DisplayName', '纯 DR');
    hold on;
    for range_index = 1:range_case_number
        plot(t, error_aided_m(:, bearing_index, range_index), ...
            'Color', colors(range_index, :), 'LineWidth', 1.3, ...
            'DisplayName', sprintf('距离辅助：%d m', ...
            beacon_range_m(range_index)));
    end
    grid on;
    xlabel('时间 / s');
    ylabel('水平位置误差 / m');
    title([bearing_name{bearing_index}, '：位置误差']);
    legend('Location', 'best');
end

%% 保存结果
save(fullfile(data_dir, 'step5_beacon_start_comparison_results.mat'), ...
    't', 'p_true', 'p_dr', 'p_aided', 'error_dr_m', 'error_aided_m', ...
    'bearing_deg', 'bearing_name', 'cancellation_bearing_deg', ...
    'maximum_residual_bearing_deg', 'beacon_range_m', 'beacon_xy', ...
    'raw_range_residual_m', 'aided_range_residual_m', ...
    'maximum_bearing_deviation_deg', 'dvl_scale_error', ...
    'compass_error_deg', 'measurement_interval_s', ...
    'range_noise_std_m', 'summary_table');
writetable(summary_table, fullfile(data_dir, ...
    'step5_beacon_start_comparison_summary.csv'));

figure(1);
exportgraphics(gcf, fullfile(data_dir, ...
    'step5_beacon_start_trajectories.png'), 'Resolution', 180);
figure(2);
exportgraphics(gcf, fullfile(data_dir, ...
    'step5_beacon_start_effect.png'), 'Resolution', 180);

fprintf('\nStep5 比较的两个恒定方位角为 %.2f°和 %.2f°。\n', ...
    cancellation_bearing_deg, maximum_residual_bearing_deg);
fprintf('每个方向的移动信标距离为 1000 m、3000 m 和 5000 m。\n');


function [p_aided, aided_range_residual_m] = position_range_ekf( ...
        p_true, p_dr, beacon_xy, true_range_m, range_noise_m, ...
        sample_time_s, measurement_stride, range_noise_std_m, ...
        position_process_noise_m_sqrt_s)
% 使用原始 DR 增量进行二维位置预测，再使用单距离量测修正位置。
    sample_count = size(p_true, 1);
    p_aided = zeros(sample_count, 2);
    p_aided(1, :) = p_dr(1, :);

    covariance = 5^2 * eye(2);
    process_covariance = position_process_noise_m_sqrt_s^2 * ...
        sample_time_s * eye(2);
    measurement_variance = range_noise_std_m^2;

    for sample_index = 2:sample_count
        dr_increment = p_dr(sample_index, :) - p_dr(sample_index - 1, :);
        predicted_position = p_aided(sample_index - 1, :) + dr_increment;
        predicted_covariance = covariance + process_covariance;

        if mod(sample_index - 1, measurement_stride) == 0
            predicted_vector = predicted_position - beacon_xy(sample_index, :);
            predicted_range = norm(predicted_vector);
            measurement_matrix = predicted_vector / predicted_range;
            measured_range = true_range_m(sample_index) + ...
                range_noise_m(sample_index);
            innovation = measured_range - predicted_range;
            innovation_covariance = measurement_matrix * ...
                predicted_covariance * measurement_matrix' + ...
                measurement_variance;
            kalman_gain = predicted_covariance * measurement_matrix' / ...
                innovation_covariance;

            p_aided(sample_index, :) = predicted_position + ...
                (kalman_gain * innovation)';
            identity_matrix = eye(2);
            covariance = (identity_matrix - kalman_gain * measurement_matrix) * ...
                predicted_covariance * ...
                (identity_matrix - kalman_gain * measurement_matrix)' + ...
                kalman_gain * measurement_variance * kalman_gain';
        else
            p_aided(sample_index, :) = predicted_position;
            covariance = predicted_covariance;
        end
    end

    aided_range = sqrt(sum((p_aided - beacon_xy).^2, 2));
    aided_range_residual_m = aided_range - true_range_m;
end
