%% Step 2：初步比较 DVL/罗盘误差传播和距离残差
% 本脚本读取 Step1 生成的直线航行数据，只研究三个问题：
%   1）DVL 刻度因子误差如何形成沿程位置误差；
%   2）正负 0.23°罗盘误差如何形成横向位置误差；
%   3）位置误差投影到信标视线后，距离残差为何会叠加或抵消。
%
% 距离残差定义：
%   距离残差 = 带误差 DR 到信标的距离 - 真实轨迹到信标的距离
%
% 一阶传播模型：
%   delta_v = delta_k * v + delta_psi * [-v_N, v_E]
%   delta_p = integral(delta_v dt)
%   delta_rho = u' * delta_p
%
% 当前只做确定性对比，不加入随机噪声，不使用 EKF。

clear;
close all;
clc;

study_dir = fileparts(mfilename('fullpath'));
psins_root = fileparts(fileparts(fileparts(study_dir)));
addpath(genpath(psins_root));

data_dir = fullfile(study_dir, 'generated_data');
data_file = fullfile(data_dir, 'trajectory_01_straight.mat');
if ~exist(data_file, 'file')
    error('未找到直线轨迹数据，请先运行 Step1_Generate_Typical_Data.m。');
end

load(data_file, 'avp_ref', 'time_s', 'local_xy', ...
    'dvl_true', 'dvl_plus', 'yaw_true', 'yaw_plus', 'yaw_minus', ...
    'ts', 'dvl_scale_error', 'compass_error_deg');

%% 截取 1800 s 匀速航行段
speed_mps = sqrt(sum(avp_ref(:, 4:5).^2, 2));
first_index = find(speed_mps >= 0.999, 1, 'first');
analysis_time_s = 1800;
sample_number = round(analysis_time_s / ts) + 1;
last_index = first_index + sample_number - 1;
if isempty(first_index) || last_index > size(avp_ref, 1)
    error('轨迹中不存在完整的 1800 s 匀速航行段。');
end

id = first_index:last_index;
t = time_s(id) - time_s(first_index);
p_true = local_xy(id, :) - local_xy(first_index, :);

%% 设置五组最基本的误差组合
% 每行依次为：[DVL 刻度因子误差，罗盘航向误差/deg]
case_parameter = [ ...
    dvl_scale_error, 0; ...
    0, compass_error_deg; ...
    0, -compass_error_deg; ...
    dvl_scale_error, compass_error_deg; ...
    dvl_scale_error, -compass_error_deg];

case_name = { ...
    sprintf('仅 DVL +%.1f%%', 100 * dvl_scale_error); ...
    sprintf('仅罗盘 +%.2f°', compass_error_deg); ...
    sprintf('仅罗盘 -%.2f°', compass_error_deg); ...
    sprintf('DVL +%.1f%%、罗盘 +%.2f°', ...
        100 * dvl_scale_error, compass_error_deg); ...
    sprintf('DVL +%.1f%%、罗盘 -%.2f°', ...
        100 * dvl_scale_error, compass_error_deg)};

case_number = size(case_parameter, 1);
position_error_exact = zeros(sample_number, 2, case_number);
position_error_theory = zeros(sample_number, 2, case_number);
range_residual_exact = zeros(sample_number, case_number);
range_residual_theory = zeros(sample_number, case_number);

%% 根据真实航迹自动放置一个非对称固定信标
track_direction = p_true(end, :) - p_true(1, :);
track_direction = track_direction / norm(track_direction);
cross_direction = [track_direction(2), -track_direction(1)];
beacon_xy = mean(p_true, 1) - 2500 * track_direction + ...
    1500 * cross_direction;

true_range = sqrt(sum((p_true - beacon_xy).^2, 2));
line_of_sight = (p_true - beacon_xy) ./ true_range;
v_true_en = body_to_en(dvl_true(id, :), yaw_true(id));

%% 分别计算精确非线性结果和一阶理论结果
for k = 1:case_number
    delta_k = case_parameter(k, 1);
    delta_psi_deg = case_parameter(k, 2);

    if delta_k == 0
        dvl_used = dvl_true(id, :);
    else
        dvl_used = dvl_plus(id, :);
    end

    if delta_psi_deg > 0
        yaw_used = yaw_plus(id);
    elseif delta_psi_deg < 0
        yaw_used = yaw_minus(id);
    else
        yaw_used = yaw_true(id);
    end

    % 精确结果：直接使用 Step1 生成的带误差 DVL 和罗盘数据。
    v_measured_en = body_to_en(dvl_used, yaw_used);
    delta_v_exact = v_measured_en - v_true_en;
    position_error_exact(:, :, k) = cumtrapz(t, delta_v_exact);

    % 理论结果：保留刻度因子误差和航向误差的一阶项。
    delta_psi = delta_psi_deg * pi / 180;
    % PSINS 水平速度顺序为 [东, 北]。对 a2mat 的航向旋转矩阵
    % 求一阶导数后，航向敏感方向为 [-v_N, v_E]。
    heading_sensitivity = [-v_true_en(:, 2), v_true_en(:, 1)];
    delta_v_theory = delta_k * v_true_en + ...
        delta_psi * heading_sensitivity;
    position_error_theory(:, :, k) = cumtrapz(t, delta_v_theory);

    p_dr = p_true + position_error_exact(:, :, k);
    predicted_range = sqrt(sum((p_dr - beacon_xy).^2, 2));
    range_residual_exact(:, k) = predicted_range - true_range;
    range_residual_theory(:, k) = sum(line_of_sight .* ...
        position_error_theory(:, :, k), 2);
end

%% 汇总 1800 s 终点结果
final_position_error_m = zeros(case_number, 1);
final_along_error_m = zeros(case_number, 1);
final_cross_error_m = zeros(case_number, 1);
final_range_residual_m = zeros(case_number, 1);
position_linearization_error_m = zeros(case_number, 1);
range_linearization_rmse_m = zeros(case_number, 1);

for k = 1:case_number
    final_position_error_m(k) = norm(position_error_exact(end, :, k));
    final_along_error_m(k) = position_error_exact(end, :, k) * ...
        track_direction';
    final_cross_error_m(k) = position_error_exact(end, :, k) * ...
        cross_direction';
    final_range_residual_m(k) = range_residual_exact(end, k);
    position_linearization_error_m(k) = max(sqrt(sum(( ...
        position_error_exact(:, :, k) - ...
        position_error_theory(:, :, k)).^2, 2)));
    range_linearization_rmse_m(k) = sqrt(mean(( ...
        range_residual_exact(:, k) - ...
        range_residual_theory(:, k)).^2));
end

summary_table = table(case_name, case_parameter(:, 1), case_parameter(:, 2), ...
    final_position_error_m, final_along_error_m, final_cross_error_m, ...
    final_range_residual_m, ...
    position_linearization_error_m, range_linearization_rmse_m, ...
    'VariableNames', {'工况', 'DVL刻度因子误差', '罗盘误差_deg', ...
    '终点位置误差_m', '终点沿程误差_m', '终点横向误差_m', ...
    '终点距离残差_m', ...
    '位置一阶近似最大误差_m', '距离一阶近似RMSE_m'});
disp(summary_table);

%% 绘图：分别观察沿程、横向和距离方向的误差
colors = lines(case_number);
% figure('Name', '误差传播与距离残差初步对比', ...
%     'Color', 'w', 'Position', [100, 50, 950, 900]);
myfigurestartup(9,2.5,'paper');
subplot(1, 3, 1);
hold on;
for k = 1:case_number
    along_error = position_error_exact(:, :, k) * track_direction';
    plot(t, along_error, 'Color', colors(k, :), 'DisplayName', case_name{k});
end
grid on;
xlabel('时间 / s');
ylabel('沿程误差 / m');
title('DVL 刻度因子误差主要形成沿程漂移');
legend('Location', 'northwest');
xlim([t(1),t(end)])
subplot(1, 3, 2);
hold on;
for k = 1:case_number
    cross_error = position_error_exact(:, :, k) * cross_direction';
    plot(t, cross_error, 'Color', colors(k, :), 'DisplayName', case_name{k});
end
grid on;
xlabel('时间 / s');
ylabel('横向误差 / m');
title('正负罗盘误差形成方向相反的横向漂移');
% legend('Location', 'best');
xlim([t(1),t(end)])
subplot(1, 3, 3);
hold on;
for k = 1:case_number
    plot(t, range_residual_exact(:, k), '-', 'Color', colors(k, :), ...
        'LineWidth', 1.2, 'DisplayName', case_name{k});
    plot(t, range_residual_theory(:, k), '--', ...
        'Color', colors(k, :), 'HandleVisibility', 'off');
end
grid on;
xlabel('时间 / s');
ylabel('距离残差 / m');
title('距离残差：实线为精确值，虚线为一阶近似');
% legend('Location', 'best');
xlim([t(1),t(end)])
save(fullfile(data_dir, 'step2_error_propagation_results.mat'), ...
    't', 'p_true', 'beacon_xy', 'case_name', 'case_parameter', ...
    'dvl_scale_error', 'compass_error_deg', ...
    'position_error_exact', 'position_error_theory', ...
    'range_residual_exact', 'range_residual_theory', 'summary_table');
writetable(summary_table, ...
    fullfile(data_dir, 'step2_error_propagation_summary.csv'));
exportgraphics(gcf, fullfile(data_dir, ...
    'step2_error_propagation.png'), 'Resolution', 180);

fprintf(['\n1800 s 理论量级：DVL +%.1f%% 约 %.2f m，', ...
    '罗盘 %.2f°约 %.2f m。\n'], ...
    100 * dvl_scale_error, ...
    analysis_time_s * dvl_scale_error, ...
    compass_error_deg, ...
    analysis_time_s * compass_error_deg * pi / 180);
fprintf('注意：距离残差只是位置误差在信标视线方向上的投影。\n');


function velocity_en = body_to_en(dvl_body, yaw)
% 使用航向角把 DVL 水平速度从载体系转换到东、北坐标系。
    velocity_en = zeros(size(dvl_body));
    for i = 1:size(dvl_body, 1)
        cnb = a2mat([0; 0; yaw(i)]);
        velocity_n = cnb * [dvl_body(i, :)'; 0];
        velocity_en(i, :) = velocity_n(1:2)';
    end
end
