%% Step 4：比较多种移动信标几何下的单距离辅助效果
% 本脚本使用 Step2 中的 1800 s 直线轨迹和误差平衡后的
% DVL、罗盘航位推算结果，重点比较三种移动信标：
%   1）相对方位角始终为 0°：DVL 误差敏感方向；
%   2）相对方位角始终为 90°：罗盘误差敏感方向；
%   3）相对方位角始终为自动计算值：一阶距离残差抵消方向。
% 此外保留 45°和 80°，作为用户设置的补充对照工况。
%
% 信标与真实载体保持 3000 m 距离，并以相同速度平行移动，因此
% 信标视线相对航向的角度在整个 1800 s 内保持不变。
%
% 相对方位角定义与 Step3 相同：视线单位向量由信标指向载体，
% 0°与航行方向相同，90°与航行方向垂直。
%
% 为突出几何关系，这里使用最简单的二维位置 EKF：
%   状态        = [东向位置；北向位置]
%   状态预测    = 叠加原始 DR 的相邻位置增量
%   距离量测    = 载体到移动信标的距离
% 暂不估计 DVL 刻度因子和罗盘误差，后续再扩展状态。

clear;
close all;
clc;

study_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(study_dir, 'generated_data');
data_file = fullfile(data_dir, 'step2_error_propagation_results.mat');
if ~exist(data_file, 'file')
    error(['未找到 Step2 结果，请依次运行 Step1_Generate_Typical_Data.m ', ...
        '和 Step2_Error_Propagation_And_Range_Residual.m。']);
end

load(data_file, 't', 'p_true', 'position_error_exact', ...
    'dvl_scale_error', 'compass_error_deg');

%% 取 DVL 正刻度误差、罗盘正航向误差组合的原始 DR 结果
% Step2 的第 4 个工况是两类正误差的组合。
p_dr = p_true + position_error_exact(:, :, 4);
sample_count = size(p_true, 1);
sample_time_s = median(diff(t));

track_direction = p_true(end, :) - p_true(1, :);
track_direction = track_direction / norm(track_direction);
cross_direction = [track_direction(2), -track_direction(1)];

%% 设置恒定相对方位角和距离量测参数
% 一阶抵消条件：delta_k*cos(alpha)-delta_psi*sin(alpha)=0。
cancellation_bearing_deg = atand(dvl_scale_error / ...
    (compass_error_deg * pi / 180));
target_bearing_deg = [0, 90, cancellation_bearing_deg, 135, 120];
case_name = {'恒定 0°', '恒定 90°', ...
    sprintf('一阶抵消 %.2f°', cancellation_bearing_deg), ...
    '恒定 135°', '恒定 120°'};
case_number = numel(target_bearing_deg);

beacon_radius_m = 3000;
measurement_interval_s = 10;
range_noise_std_m = 1;
position_process_noise_m_sqrt_s = 0.08;
measurement_stride = round(measurement_interval_s / sample_time_s);
if abs(measurement_stride * sample_time_s - measurement_interval_s) > 1e-10
    error('测距周期必须是轨迹采样周期的整数倍。');
end

% 所有工况使用同一组测距噪声，保证差异主要来自信标几何。
rng(2013, 'twister');
common_range_noise_m = range_noise_std_m * randn(sample_count, 1);

beacon_xy = zeros(sample_count, 2, case_number);
relative_bearing_deg = zeros(sample_count, case_number);
true_range_m = zeros(sample_count, case_number);
raw_range_residual_m = zeros(sample_count, case_number);
aided_range_residual_m = zeros(sample_count, case_number);
p_aided = zeros(sample_count, 2, case_number);
error_aided_m = zeros(sample_count, case_number);
innovation_m = nan(sample_count, case_number);

%% 生成移动信标轨迹并分别进行距离辅助
for case_index = 1:case_number
    % 信标始终位于真实载体的固定相对位置。
    % 因而信标与载体具有相同的位置增量，相对距离和方位均保持不变。
    line_of_sight_direction = ...
        cosd(target_bearing_deg(case_index)) * track_direction + ...
        sind(target_bearing_deg(case_index)) * cross_direction;
    beacon_xy(:, :, case_index) = p_true - ...
        beacon_radius_m * line_of_sight_direction;

    true_range_vector = p_true - beacon_xy(:, :, case_index);
    true_range_m(:, case_index) = sqrt(sum(true_range_vector.^2, 2));
    line_of_sight = true_range_vector ./ true_range_m(:, case_index);

    % 根据实际位置重新计算相对方位角，用于检查信标构造是否正确。
    along_component = line_of_sight * track_direction';
    cross_component = line_of_sight * cross_direction';
    relative_bearing_deg(:, case_index) = mod( ...
        atan2d(cross_component, along_component), 360);
    near_full_circle = relative_bearing_deg(:, case_index) > 359.999999;
    relative_bearing_deg(near_full_circle, case_index) = 0;

    raw_range_vector = p_dr - beacon_xy(:, :, case_index);
    raw_range = sqrt(sum(raw_range_vector.^2, 2));
    raw_range_residual_m(:, case_index) = raw_range - ...
        true_range_m(:, case_index);

    % 二维位置 EKF。初始位置与原始 DR 相同，随后使用 DR 增量递推。
    p_aided(1, :, case_index) = p_dr(1, :);
    covariance = 5^2 * eye(2);
    process_covariance = position_process_noise_m_sqrt_s^2 * ...
        sample_time_s * eye(2);
    measurement_variance = range_noise_std_m^2;

    for sample_index = 2:sample_count
        dr_increment = p_dr(sample_index, :) - p_dr(sample_index - 1, :);
        predicted_position = p_aided(sample_index - 1, :, case_index) + ...
            dr_increment;
        predicted_covariance = covariance + process_covariance;

        if mod(sample_index - 1, measurement_stride) == 0
            current_beacon = beacon_xy(sample_index, :, case_index);
            measured_range = true_range_m(sample_index, case_index) + ...
                common_range_noise_m(sample_index);
            predicted_vector = predicted_position - current_beacon;
            predicted_range = norm(predicted_vector);
            measurement_matrix = predicted_vector / predicted_range;

            innovation_m(sample_index, case_index) = ...
                measured_range - predicted_range;
            innovation_covariance = measurement_matrix * ...
                predicted_covariance * measurement_matrix' + ...
                measurement_variance;
            kalman_gain = predicted_covariance * measurement_matrix' / ...
                innovation_covariance;

            corrected_position = predicted_position + ...
                (kalman_gain * innovation_m(sample_index, case_index))';

            % Joseph 形式可以减小数值计算造成的协方差非对称。
            identity_matrix = eye(2);
            covariance = (identity_matrix - kalman_gain * measurement_matrix) * ...
                predicted_covariance * ...
                (identity_matrix - kalman_gain * measurement_matrix)' + ...
                kalman_gain * measurement_variance * kalman_gain';
            p_aided(sample_index, :, case_index) = corrected_position;
        else
            covariance = predicted_covariance;
            p_aided(sample_index, :, case_index) = predicted_position;
        end
    end

    aided_range_vector = p_aided(:, :, case_index) - ...
        beacon_xy(:, :, case_index);
    aided_range = sqrt(sum(aided_range_vector.^2, 2));
    aided_range_residual_m(:, case_index) = aided_range - ...
        true_range_m(:, case_index);
    error_aided_m(:, case_index) = sqrt(sum(( ...
        p_aided(:, :, case_index) - p_true).^2, 2));
end

error_dr_m = sqrt(sum((p_dr - p_true).^2, 2));

%% 汇总不同移动信标几何的结果
initial_bearing_deg = relative_bearing_deg(1, :)';
final_bearing_deg = relative_bearing_deg(end, :)';
maximum_bearing_deviation_deg = max(abs( ...
    relative_bearing_deg - target_bearing_deg), [], 1)';
mean_true_range_m = mean(true_range_m, 1)';
dr_rmse_m = sqrt(mean(error_dr_m.^2)) * ones(case_number, 1);
aided_rmse_m = sqrt(mean(error_aided_m.^2, 1))';
rmse_improvement_percent = 100 * (dr_rmse_m - aided_rmse_m) ./ dr_rmse_m;
final_dr_error_m = error_dr_m(end) * ones(case_number, 1);
final_aided_error_m = error_aided_m(end, :)';
final_raw_range_residual_m = raw_range_residual_m(end, :)';

summary_table = table(case_name', target_bearing_deg', ...
    initial_bearing_deg, final_bearing_deg, maximum_bearing_deviation_deg, ...
    mean_true_range_m, dr_rmse_m, aided_rmse_m, ...
    rmse_improvement_percent, final_dr_error_m, final_aided_error_m, ...
    final_raw_range_residual_m, ...
    'VariableNames', {'工况', '设定方位角_deg', ...
    '实际起点方位角_deg', '实际终点方位角_deg', '最大方位偏差_deg', ...
    '平均真实距离_m', '纯DR_RMSE_m', '距离辅助_RMSE_m', ...
    'RMSE改善率_percent', '纯DR终点误差_m', ...
    '距离辅助终点误差_m', '纯DR终点距离残差_m'});
disp(summary_table);

%% 绘图：载体与移动信标轨迹、方位角、距离残差和位置误差
colors = lines(case_number);
figure('Name', '移动信标几何对比', ...
    'Color', 'w', 'Position', [80, 60, 1100, 820]);

subplot(2, 2, 1);
plot(p_true(:, 1), p_true(:, 2), 'k-', 'LineWidth', 1.8, ...
    'DisplayName', '真实载体轨迹');
hold on;
plot(p_dr(:, 1), p_dr(:, 2), 'k--', 'LineWidth', 1.2, ...
    'DisplayName', '纯 DR 轨迹');
for case_index = 1:case_number
    plot(beacon_xy(:, 1, case_index), beacon_xy(:, 2, case_index), ...
        'Color', colors(case_index, :), 'LineWidth', 1.4, ...
        'DisplayName', [case_name{case_index}, '信标轨迹']);
    plot(beacon_xy(1, 1, case_index), beacon_xy(1, 2, case_index), ...
        'o', 'Color', colors(case_index, :), 'MarkerFaceColor', 'w', ...
        'HandleVisibility', 'off');
    plot(beacon_xy(end, 1, case_index), beacon_xy(end, 2, case_index), ...
        'p', 'Color', colors(case_index, :), ...
        'MarkerFaceColor', colors(case_index, :), 'MarkerSize', 9, ...
        'HandleVisibility', 'off');
end
grid on;
axis equal;
axis padded;
xlabel('东向 / m');
ylabel('北向 / m');
title('载体轨迹与移动信标轨迹（圆点为起点，五角星为终点）');
legend('Location', 'best');

subplot(2, 2, 2);
hold on;
for case_index = 1:case_number
    plot(t, relative_bearing_deg(:, case_index), ...
        'Color', colors(case_index, :), 'LineWidth', 1.3, ...
        'DisplayName', case_name{case_index});
end
grid on;
xlabel('时间 / s');
ylabel('实际相对方位角 / (°)');
title('移动信标的相对方位角保持不变');
legend('Location', 'best');

subplot(2, 2, 3);
hold on;
for case_index = 1:case_number
    plot(t, raw_range_residual_m(:, case_index), ...
        'Color', colors(case_index, :), 'LineWidth', 1.3, ...
        'DisplayName', case_name{case_index});
end
yline(0, 'k:', 'HandleVisibility', 'off');
grid on;
xlabel('时间 / s');
ylabel('纯 DR 距离残差 / m');
title('同一位置误差在恒定视线方向上的投影');
legend('Location', 'best');

subplot(2, 2, 4);
plot(t, error_dr_m, 'k--', 'LineWidth', 1.5, ...
    'DisplayName', '纯 DR');
hold on;
for case_index = 1:case_number
    plot(t, error_aided_m(:, case_index), ...
        'Color', colors(case_index, :), 'LineWidth', 1.3, ...
        'DisplayName', ['距离辅助：', case_name{case_index}]);
end
grid on;
xlabel('时间 / s');
ylabel('水平位置误差 / m');
title('单距离辅助位置误差对比');
legend('Location', 'best');

save(fullfile(data_dir, 'step4_three_beacon_geometry_results.mat'), ...
    't', 'p_true', 'p_dr', 'p_aided', 'error_dr_m', 'error_aided_m', ...
    'target_bearing_deg', 'case_name', 'beacon_radius_m', 'beacon_xy', ...
    'cancellation_bearing_deg', 'dvl_scale_error', 'compass_error_deg', ...
    'relative_bearing_deg', 'true_range_m', ...
    'raw_range_residual_m', 'aided_range_residual_m', 'innovation_m', ...
    'measurement_interval_s', 'range_noise_std_m', 'summary_table');
writetable(summary_table, fullfile(data_dir, ...
    'step4_three_beacon_geometry_summary.csv'));
exportgraphics(gcf, fullfile(data_dir, ...
    'step4_three_beacon_geometry.png'), 'Resolution', 180);

fprintf('\n所有信标均随真实载体平行移动，实际距离保持 %.1f m。\n', ...
    beacon_radius_m);
fprintf('DVL +%.1f%%、罗盘 +%.2f°对应的一阶抵消角为 %.2f°。\n', ...
    100 * dvl_scale_error, compass_error_deg, cancellation_bearing_deg);
