%% 评估无距离辅助时路线 A 与路线 B 的累积误差
% 只做统一误差计算、制表和绘图，不进行误差机理分析。
clear;
close all;
clc;

study_dir = fileparts(mfilename('fullpath'));
data_file = fullfile(study_dir, 'data_no_range', 'simulation_data.mat');
route_a_file = fullfile(study_dir, 'output_no_range', 'route_a_result.mat');
route_b_file = fullfile(study_dir, 'output_no_range', 'route_b_result.mat');
required_files = {data_file, route_a_file, route_b_file};
if ~all(cellfun(@isfile, required_files))
    error(['缺少输入结果。请依次运行 simulate_dr_ins_data.m、', ...
        'run_route_b_dvl_dr.m 和 run_route_a_ins_dvl.m。']);
end

data = load(data_file, 'truth', 'dvl', 'cfg');
a = load(route_a_file, 'route_a');
b = load(route_b_file, 'route_b');
truth = data.truth;
dvl = data.dvl;
route_a = a.route_a;
route_b = b.route_b;

truth_position_ned = truth.position_ned_m(dvl.imu_index, :);
truth_velocity_ned = truth.velocity_ned_mps(dvl.imu_index, :);
truth_attitude = truth.attitude_rph_rad(dvl.imu_index, :);
segment_distance_m = sqrt(sum(diff(truth_position_ned(:, 1:2), 1, 1).^2, 2));
cumulative_distance_m = [0; cumsum(segment_distance_m)];
total_distance_m = cumulative_distance_m(end);

metrics_a = calculate_metrics(route_a, truth_position_ned, ...
    truth_velocity_ned, truth_attitude, cumulative_distance_m);
metrics_b = calculate_metrics(route_b, truth_position_ned, ...
    truth_velocity_ned, truth_attitude, cumulative_distance_m);

summary = table( ...
    ["路线A_INS_DVL_EKF"; "路线B_DVL_DR"], ...
    [total_distance_m; total_distance_m], ...
    [metrics_a.horizontal_rmse_m; metrics_b.horizontal_rmse_m], ...
    [metrics_a.horizontal_max_m; metrics_b.horizontal_max_m], ...
    [metrics_a.horizontal_final_m; metrics_b.horizontal_final_m], ...
    [metrics_a.final_error_percent_distance; metrics_b.final_error_percent_distance], ...
    [metrics_a.position_3d_rmse_m; metrics_b.position_3d_rmse_m], ...
    [metrics_a.velocity_rmse_mps; metrics_b.velocity_rmse_mps], ...
    [metrics_a.heading_rmse_deg; metrics_b.heading_rmse_deg], ...
    'VariableNames', {'Method', 'TotalDistance_m', 'HorizontalRMSE_m', ...
    'HorizontalMax_m', 'HorizontalFinal_m', 'FinalErrorPercentDistance_pct', ...
    'Position3DRMSE_m', 'VelocityRMSE_mps', 'HeadingRMSE_deg'});

output_dir = fullfile(study_dir, 'output_no_range');
writetable(summary, fullfile(output_dir, 'error_summary.csv'));
save(fullfile(output_dir, 'error_evaluation.mat'), ...
    'summary', 'metrics_a', 'metrics_b');
disp(summary);

time_min = (dvl.time_s - dvl.time_s(1)) / 60;
fig = figure('Name', 'DR/INS 无距离辅助累积误差', 'Color', 'w', ...
    'Position', [100, 100, 1200, 760]);
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(truth_position_ned(:, 2), truth_position_ned(:, 1), 'k-', 'LineWidth', 1.4);
hold on;
plot(route_a.position_ned_m(:, 2), route_a.position_ned_m(:, 1), 'b-', 'LineWidth', 1.0);
plot(route_b.position_ned_m(:, 2), route_b.position_ned_m(:, 1), 'r--', 'LineWidth', 1.0);
axis equal;
grid on;
xlabel('东向 / m'); ylabel('北向 / m'); title('水平轨迹');
legend('真值', '路线 A', '路线 B', 'Location', 'best');

nexttile;
plot(time_min, metrics_a.horizontal_error_m, 'b-', 'LineWidth', 1.0);
hold on;
plot(time_min, metrics_b.horizontal_error_m, 'r--', 'LineWidth', 1.0);
grid on;
xlabel('时间 / min'); ylabel('水平位置误差 / m'); title('累积水平误差');
legend('路线 A', '路线 B', 'Location', 'best');

nexttile;
plot(time_min, metrics_a.position_error_ned_m(:, 1:2), 'LineWidth', 1.0);
hold on;
plot(time_min, metrics_b.position_error_ned_m(:, 1), 'r--', 'LineWidth', 1.0);
plot(time_min, metrics_b.position_error_ned_m(:, 2), 'm--', 'LineWidth', 1.0);
grid on;
xlabel('时间 / min'); ylabel('位置误差 / m'); title('北向、东向误差');
legend('A-N', 'A-E', 'B-N', 'B-E', 'Location', 'best');  

nexttile;
plot(time_min, metrics_a.heading_error_deg, 'b-', 'LineWidth', 1.0);
hold on;
plot(time_min, metrics_b.heading_error_deg, 'r--', 'LineWidth', 1.0);
grid on;
xlabel('时间 / min'); ylabel('航向误差 / deg'); title('INS 航向误差');
legend('路线 A', '路线 B', 'Location', 'best');

figure_file = fullfile(output_dir, 'route_error_comparison.png');
exportgraphics(fig, figure_file, 'Resolution', 200);
fprintf('误差评估完成：%s\n', fullfile(output_dir, 'error_summary.csv'));
fprintf('对比图已保存：%s\n', figure_file);
fprintf('总航程：%.3f m。路线 A/B 的终点误差航程比分别为 %.4f%%、%.4f%%。\n', ...
    total_distance_m, metrics_a.final_error_percent_distance, ...
    metrics_b.final_error_percent_distance);


function metrics = calculate_metrics(result, truth_position, truth_velocity, ...
        truth_attitude, cumulative_distance_m)
    metrics.position_error_ned_m = result.position_ned_m - truth_position;
    metrics.velocity_error_ned_mps = result.velocity_ned_mps - truth_velocity;
    attitude_error = wrap_to_pi_local(result.attitude_rph_rad - truth_attitude);
    metrics.attitude_error_rph_rad = attitude_error;
    metrics.horizontal_error_m = hypot(metrics.position_error_ned_m(:, 1), ...
        metrics.position_error_ned_m(:, 2));
    metrics.position_3d_error_m = sqrt(sum(metrics.position_error_ned_m.^2, 2));
    metrics.velocity_error_norm_mps = sqrt(sum(metrics.velocity_error_ned_mps.^2, 2));
    metrics.heading_error_deg = rad2deg(attitude_error(:, 3));
    metrics.horizontal_error_percent_distance = nan(size(metrics.horizontal_error_m));
    valid_distance = cumulative_distance_m > 0;
    metrics.horizontal_error_percent_distance(valid_distance) = 100 * ...
        metrics.horizontal_error_m(valid_distance) ./ cumulative_distance_m(valid_distance);

    metrics.horizontal_rmse_m = sqrt(mean(metrics.horizontal_error_m.^2));
    metrics.horizontal_max_m = max(metrics.horizontal_error_m);
    metrics.horizontal_final_m = metrics.horizontal_error_m(end);
    metrics.position_3d_rmse_m = sqrt(mean(metrics.position_3d_error_m.^2));
    metrics.velocity_rmse_mps = sqrt(mean(metrics.velocity_error_norm_mps.^2));
    metrics.heading_rmse_deg = sqrt(mean(metrics.heading_error_deg.^2));
    metrics.total_distance_m = cumulative_distance_m(end);
    metrics.final_error_percent_distance = 100 * ...
        metrics.horizontal_final_m / metrics.total_distance_m;
end


function angle = wrap_to_pi_local(angle)
    angle = mod(angle + pi, 2 * pi) - pi;
end
