%% Step 6：评估典型轨迹下的单距离辅助 DVL + 罗盘航位推算
% 运行本脚本前，请先运行 Step1_Generate_Typical_Data。
%
% 处理架构与现有实测实验保持一致：
%   DVL + 罗盘 + 深度 -> mydr 航位递推
%   四状态误差模型    -> myekf 时间更新
%   单固定信标水平距离 -> myekf 距离量测更新
%   估计的水平位置误差 -> 闭环反馈修正 DR
%
% myekf 状态顺序：
%   [DVL 刻度因子误差；航向误差；纬度误差；经度误差]
%
% generated_data 中的输出：
%   single_range_aided_dr_results.mat - 完整轨迹与滤波记录
%   single_range_summary.csv          - 单距离辅助与纯 DR 的定量对比

clear;
close all;
clc;

study_dir = fileparts(mfilename('fullpath'));
psins_root = fileparts(fileparts(fileparts(study_dir)));
addpath(genpath(psins_root));

glvs;

data_dir = fullfile(study_dir, 'generated_data');
catalog_file = fullfile(data_dir, 'trajectory_catalog.mat');
if ~exist(catalog_file, 'file')
    error(['未找到轨迹索引文件，请先运行 ', ...
        'Step1_Generate_Typical_Data.m。']);
end

catalog_data = load(catalog_file, 'scenario_ids', 'scenario_names_zh', ...
    'scenario_descriptions', 'file_names');
scenario_ids = catalog_data.scenario_ids;
scenario_names_zh = catalog_data.scenario_names_zh;
scenario_descriptions = catalog_data.scenario_descriptions;
file_names = catalog_data.file_names;

config.random_seed = 2013;
config.measurement_interval_s = 8;
config.range_noise_std_m = 3.0;
config.initial_dr_error_enu_m = [5; -3; 0.5];
config.beacon_margin_m = 300;

rng(config.random_seed, 'twister');

num_scenarios = numel(file_names);
results = repmat(struct(), num_scenarios, 1);
summary_rows = repmat(empty_summary_row(), 2 * num_scenarios, 1);

figure('Name', '单距离辅助航位推算评估', 'Color', 'w');

for scenario_index = 1:num_scenarios
    input_data = load(fullfile(data_dir, file_names{scenario_index}), ...
        'trj', 'dvl_plus', 'yaw_plus', 'depth_true');
    trj = input_data.trj;
    avp_ref = trj.avp;
    sample_time_s = trj.ts;
    sample_count = size(avp_ref, 1);
    time_s = avp_ref(:, end);

    measurement_stride = round(config.measurement_interval_s / sample_time_s);
    if abs(measurement_stride * sample_time_s - config.measurement_interval_s) > 1e-10
        error('测距周期必须是轨迹采样周期的整数倍。');
    end

    % 在轨迹包围盒外布设一个水面固定信标，使每个场景都具有
    % 可重复且非对称的观测几何。
    local_ref_xyz = pos2dxyz(avp_ref(:, 7:9), avp_ref(1, 7:9)');
    span_xy = max(local_ref_xyz(:, 1:2), [], 1) - ...
        min(local_ref_xyz(:, 1:2), [], 1);
    margin_m = max(config.beacon_margin_m, 0.25 * max(span_xy));
    beacon_xyz = [max(local_ref_xyz(:, 1)) + margin_m, ...
        min(local_ref_xyz(:, 2)) - margin_m, -avp_ref(1, 9)];
    beacon_pos = dxyz2pos(beacon_xyz, avp_ref(1, 7:9)');

    % 直接使用 Step1 生成的 DVL +0.4% 和罗盘 +4°数据。
    % 这一阶段只在距离量测中加入噪声，便于先观察主要系统误差。
    compass_yaw = input_data.yaw_plus;
    dvl_body = input_data.dvl_plus;
    height_meas = input_data.depth_true;

    dr_base = mydr('init', avp_ref(1, 7:9)', ...
        config.initial_dr_error_enu_m, sample_time_s);
    dr_aided = mydr('init', avp_ref(1, 7:9)', ...
        config.initial_dr_error_enu_m, sample_time_s);

    x0 = zeros(4, 1);
    dx0 = [0.01; 5 * glv.deg; 5 / glv.Re; 5 / glv.Re];
    process_noise = [0; 0.02 * glv.deg; 0; 0];
    kf = myekf('init', sample_time_s, x0, dx0, ...
        process_noise, config.range_noise_std_m);

    avp_dr = zeros(sample_count, 10);
    avp_aided = zeros(sample_count, 10);
    max_measurements = floor(sample_count / measurement_stride);
    filter_log = zeros(max_measurements, 9);
    range_log = zeros(max_measurements, 6);
    measurement_count = 0;

    for sample_index = 1:sample_count
        dr_base = mydr('update', dr_base, height_meas(sample_index), ...
            compass_yaw(sample_index), dvl_body(sample_index, :));
        dr_aided = mydr('update', dr_aided, height_meas(sample_index), ...
            compass_yaw(sample_index), dvl_body(sample_index, :));

        kf = myekf('fk', kf, dr_aided);
        kf = myekf('algo', kf, 'T');

        if mod(sample_index, measurement_stride) == 0
            measurement_count = measurement_count + 1;
            dr_aided.beacon = beacon_pos;

            true_slant_m = RCompu(avp_ref(sample_index, 7:9), beacon_pos);
            measured_slant_m = true_slant_m + ...
                config.range_noise_std_m * randn;
            vertical_separation_m = avp_ref(sample_index, 9) - beacon_pos(3);
            measured_horizontal_m = sqrt(max( ...
                measured_slant_m^2 - vertical_separation_m^2, eps));

            predicted_slant_m = RCompu(dr_aided.pos', beacon_pos);
            predicted_vertical_m = dr_aided.pos(3) - beacon_pos(3);
            predicted_horizontal_m = sqrt(max( ...
                predicted_slant_m^2 - predicted_vertical_m^2, eps));

            % 残差为正表示 DR 预测的距离大于实际量测距离。
            kf.r_dr = predicted_horizontal_m;
            kf.yk = predicted_horizontal_m - measured_horizontal_m;
            kf = myekf('hk', kf, dr_aided, 'range');
            kf = myekf('algo', kf, 'M');

            estimated_error = kf.xk;
            dr_aided.pos(1:2) = dr_aided.pos(1:2) - estimated_error(3:4);
            kf.xk(3:4) = 0;
            dr_aided.avp = [dr_aided.att; dr_aided.vn; dr_aided.pos];

            filter_log(measurement_count, :) = [estimated_error', ...
                diag(kf.Pxk)', time_s(sample_index)];
            range_log(measurement_count, :) = [time_s(sample_index), ...
                true_slant_m, measured_slant_m, measured_horizontal_m, ...
                predicted_horizontal_m, kf.yk];
        end

        avp_dr(sample_index, :) = [dr_base.avp', time_s(sample_index)];
        avp_aided(sample_index, :) = [dr_aided.avp', time_s(sample_index)];
    end

    filter_log = filter_log(1:measurement_count, :);
    range_log = range_log(1:measurement_count, :);

    error_dr_m = RCompu(avp_ref(:, 7:9), avp_dr(:, 7:9));
    error_aided_m = RCompu(avp_ref(:, 7:9), avp_aided(:, 7:9));
    stats_dr = error_statistics(error_dr_m);
    stats_aided = error_statistics(error_aided_m);
    improvement_percent = 100 * (stats_dr.rmse_m - stats_aided.rmse_m) / ...
        stats_dr.rmse_m;

    row = 2 * scenario_index - 1;
    summary_rows(row) = make_summary_row(scenario_ids{scenario_index}, ...
        '纯 DR', stats_dr, 0);
    summary_rows(row + 1) = make_summary_row(scenario_ids{scenario_index}, ...
        'DR + 单距离', stats_aided, improvement_percent);

    results(scenario_index).scenario = scenario_ids{scenario_index};
    results(scenario_index).description = scenario_descriptions{scenario_index};
    results(scenario_index).beacon_pos = beacon_pos;
    results(scenario_index).beacon_xyz_enu_m = beacon_xyz;
    results(scenario_index).avp_ref = avp_ref;
    results(scenario_index).avp_dr = avp_dr;
    results(scenario_index).avp_aided = avp_aided;
    results(scenario_index).error_dr_m = error_dr_m;
    results(scenario_index).error_aided_m = error_aided_m;
    results(scenario_index).filter_log = filter_log;
    results(scenario_index).range_log = range_log;
    results(scenario_index).stats_dr = stats_dr;
    results(scenario_index).stats_aided = stats_aided;
    results(scenario_index).rmse_improvement_percent = improvement_percent;

    subplot(num_scenarios, 2, 2 * scenario_index - 1);
    local_dr_xyz = pos2dxyz(avp_dr(:, 7:9), avp_ref(1, 7:9)');
    local_aided_xyz = pos2dxyz(avp_aided(:, 7:9), avp_ref(1, 7:9)');
    plot(local_ref_xyz(:, 1), local_ref_xyz(:, 2), 'k-', 'LineWidth', 1.2);
    hold on;
    plot(local_dr_xyz(:, 1), local_dr_xyz(:, 2), '--', 'LineWidth', 1.0);
    plot(local_aided_xyz(:, 1), local_aided_xyz(:, 2), '-', 'LineWidth', 1.0);
    plot(beacon_xyz(1), beacon_xyz(2), 'rp', 'MarkerFaceColor', 'r');
    grid on;
    axis equal;
    xlabel('东向 / m');
    ylabel('北向 / m');
    title(scenario_names_zh{scenario_index});
    if scenario_index == 1
        legend('参考轨迹', '纯 DR', 'DR + 单距离', '信标', 'Location', 'best');
    end

    subplot(num_scenarios, 2, 2 * scenario_index);
    plot(time_s, error_dr_m, '--', 'LineWidth', 1.0);
    hold on;
    plot(time_s, error_aided_m, '-', 'LineWidth', 1.0);
    grid on;
    xlabel('时间 / s');
    ylabel('水平位置误差 / m');
    title(sprintf('RMSE 改善率：%.1f%%', improvement_percent));
    if scenario_index == 1
        legend('纯 DR', 'DR + 单距离', 'Location', 'best');
    end
end

summary_table = struct2table(summary_rows);
disp(summary_table);

save(fullfile(data_dir, 'single_range_aided_dr_results.mat'), ...
    'results', 'summary_table', 'config');
writetable(summary_table, fullfile(data_dir, 'single_range_summary.csv'));

fprintf('\n完整结果和 CSV 汇总已保存至：\n%s\n', data_dir);


function stats = error_statistics(error_m)
% 使用 MATLAB 基础函数计算统计量，无需统计工具箱。
    valid_error = error_m(isfinite(error_m));
    sorted_error = sort(valid_error);
    p95_index = max(1, ceil(0.95 * numel(sorted_error)));

    stats.mean_m = mean(valid_error);
    stats.rmse_m = sqrt(mean(valid_error.^2));
    stats.p95_m = sorted_error(p95_index);
    stats.max_m = max(valid_error);
    stats.final_m = valid_error(end);
end


function row = empty_summary_row()
    row = struct( ...
        'Scenario', '', ...
        'Method', '', ...
        'MeanError_m', 0, ...
        'RMSE_m', 0, ...
        'P95Error_m', 0, ...
        'MaxError_m', 0, ...
        'FinalError_m', 0, ...
        'RMSEImprovement_percent', 0);
end


function row = make_summary_row(scenario, method, stats, improvement_percent)
    row = empty_summary_row();
    row.Scenario = scenario;
    row.Method = method;
    row.MeanError_m = stats.mean_m;
    row.RMSE_m = stats.rmse_m;
    row.P95Error_m = stats.p95_m;
    row.MaxError_m = stats.max_m;
    row.FinalError_m = stats.final_m;
    row.RMSEImprovement_percent = improvement_percent;
end
