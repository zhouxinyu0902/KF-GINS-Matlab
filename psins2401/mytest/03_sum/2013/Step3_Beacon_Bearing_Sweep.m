%% Step 3：扫描信标方位，研究距离残差的几何敏感性
% 本脚本固定 Step2 的 1800 s 直线航行终点，把一个信标布置在
% 航行器周围半径 3000 m 的圆周上，并将相对方位角从 0°扫描到 360°。
%
% 相对方位角定义：
%   0°   信标视线方向与航行方向一致；
%   90°  信标视线方向与航行方向垂直；
%   180° 信标视线方向与航行方向相反。
%
% 研究量：
%   DVL 项       = u' * delta_p_dvl
%   罗盘项       = u' * delta_p_compass
%   一阶总残差   = DVL 项 + 罗盘项
%   精确总残差   = 直接计算带误差位置到信标的距离变化
%
% 目标是找出：
%   1）DVL 刻度因子误差的最大敏感方向；
%   2）罗盘误差的最大敏感方向；
%   3）两项相互抵消、总距离残差接近零的信标方向。

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

load(data_file, 'p_true', 'position_error_exact', ...
    'position_error_theory', 'dvl_scale_error', 'compass_error_deg');

%% 提取 1800 s 终点的误差分量
% Step2 工况顺序：1=仅DVL，2=罗盘正误差，3=罗盘负误差，
%               4=DVL与罗盘正误差组合，5=DVL与罗盘负误差组合。
p_final = p_true(end, :);
delta_p_dvl = position_error_theory(end, :, 1);
delta_p_compass_plus = position_error_theory(end, :, 2);
delta_p_compass_minus = position_error_theory(end, :, 3);
delta_p_combined_plus = position_error_exact(end, :, 4);
delta_p_combined_minus = position_error_exact(end, :, 5);

track_direction = p_true(end, :) - p_true(1, :);
track_direction = track_direction / norm(track_direction);
cross_direction = [track_direction(2), -track_direction(1)];

%% 将信标相对方位角从 0°扫描到 360°
relative_bearing_deg = (0:0.5:360)';
line_of_sight = cosd(relative_bearing_deg) * track_direction + ...
    sind(relative_bearing_deg) * cross_direction;

dvl_term = line_of_sight * delta_p_dvl';
compass_plus_term = line_of_sight * delta_p_compass_plus';
compass_minus_term = line_of_sight * delta_p_compass_minus';
total_plus_theory = dvl_term + compass_plus_term;
total_minus_theory = dvl_term + compass_minus_term;

%% 直接计算组合误差对应的精确距离残差
beacon_radius_m = 3000;
beacon_xy = p_final - beacon_radius_m * line_of_sight;
true_range_m = beacon_radius_m * ones(size(relative_bearing_deg));

range_vector_plus = p_final + delta_p_combined_plus - beacon_xy;
range_vector_minus = p_final + delta_p_combined_minus - beacon_xy;
total_plus_exact = sqrt(sum(range_vector_plus.^2, 2)) - true_range_m;
total_minus_exact = sqrt(sum(range_vector_minus.^2, 2)) - true_range_m;

%% 查找最大敏感方向和一阶残差抵消方向
[dvl_max_value, dvl_max_index] = max(abs(dvl_term));
[compass_max_value, compass_max_index] = max(abs(compass_plus_term));
[total_plus_max_value, total_plus_max_index] = max(abs(total_plus_theory));
[total_minus_max_value, total_minus_max_index] = max(abs(total_minus_theory));

blind_angle_plus = find_zero_crossings(relative_bearing_deg, total_plus_theory);
blind_angle_minus = find_zero_crossings(relative_bearing_deg, total_minus_theory);

error_item = { ...
    sprintf('DVL +%.1f%%', 100 * dvl_scale_error); ...
    sprintf('罗盘 +%.2f°', compass_error_deg); ...
    sprintf('罗盘 -%.2f°', compass_error_deg); ...
    sprintf('DVL +%.1f%% 与罗盘 +%.2f°', ...
        100 * dvl_scale_error, compass_error_deg); ...
    sprintf('DVL +%.1f%% 与罗盘 -%.2f°', ...
        100 * dvl_scale_error, compass_error_deg)};

summary_table = table( ...
    error_item, ...
    [dvl_max_value; compass_max_value; compass_max_value; ...
     total_plus_max_value; total_minus_max_value], ...
    [relative_bearing_deg(dvl_max_index); ...
     relative_bearing_deg(compass_max_index); ...
     mod(relative_bearing_deg(compass_max_index) + 180, 360); ...
     relative_bearing_deg(total_plus_max_index); ...
     relative_bearing_deg(total_minus_max_index)], ...
    'VariableNames', {'误差项', '最大绝对距离残差_m', '对应相对方位角_deg'});
disp(summary_table);

fprintf('\nDVL 项最大敏感方向：%.1f°，最大投影 %.2f m。\n', ...
    relative_bearing_deg(dvl_max_index), dvl_max_value);
fprintf('罗盘项最大敏感方向：%.1f°，最大投影 %.2f m。\n', ...
    relative_bearing_deg(compass_max_index), compass_max_value);
fprintf('DVL 与罗盘 +%.2f°的一阶残差抵消方向：%s。\n', ...
    compass_error_deg, ...
    angle_list_text(blind_angle_plus));
fprintf('DVL 与罗盘 -%.2f°的一阶残差抵消方向：%s。\n', ...
    compass_error_deg, ...
    angle_list_text(blind_angle_minus));

%% 绘制“距离残差—信标相对方位角”曲线
% figure('Name', '信标方位角与距离残差', ...
%     'Color', 'w', 'Position', [100, 80, 1000, 760]);
myfigurestartup(7,3,'paper');
subplot(1, 2, 1);
plot(relative_bearing_deg, dvl_term, 'b-', 'LineWidth', 1.3);
hold on;
plot(relative_bearing_deg, compass_plus_term, 'r-', 'LineWidth', 1.3);
plot(relative_bearing_deg, total_plus_theory, 'k-', 'LineWidth', 1.5);
plot(relative_bearing_deg, total_plus_exact, 'g--', 'LineWidth', 1.2);
plot(blind_angle_plus, zeros(size(blind_angle_plus)), ...
    'ko', 'MarkerFaceColor', 'y');
grid on;
xlim([0, 360]);
xticks(0:45:360);
xlabel('信标相对航向方位角 / (°)');
ylabel('距离残差 / m');
title(sprintf('DVL +%.1f%% 与罗盘 +%.2f°', ...
    100 * dvl_scale_error, compass_error_deg));
legend('DVL 项', '罗盘项', '一阶总残差', '精确总残差', ...
    '一阶抵消方向', 'Location', 'best');

subplot(1, 2, 2);
plot(relative_bearing_deg, dvl_term, 'b-', 'LineWidth', 1.3);
hold on;
plot(relative_bearing_deg, compass_minus_term, 'r-', 'LineWidth', 1.3);
plot(relative_bearing_deg, total_minus_theory, 'k-', 'LineWidth', 1.5);
plot(relative_bearing_deg, total_minus_exact, 'g--', 'LineWidth', 1.2);
plot(blind_angle_minus, zeros(size(blind_angle_minus)), ...
    'ko', 'MarkerFaceColor', 'y');
grid on;
xlim([0, 360]);
xticks(0:45:360);
xlabel('信标相对航向方位角 / (°)');
ylabel('距离残差 / m');
title(sprintf('DVL +%.1f%% 与罗盘 -%.2f°', ...
    100 * dvl_scale_error, compass_error_deg));
legend('DVL 项', '罗盘项', '一阶总残差', '精确总残差', ...
    '一阶抵消方向', 'Location', 'best');

save(fullfile(data_dir, 'step3_beacon_bearing_sweep_results.mat'), ...
    'relative_bearing_deg', 'beacon_radius_m', 'beacon_xy', ...
    'dvl_term', 'compass_plus_term', 'compass_minus_term', ...
    'total_plus_theory', 'total_minus_theory', ...
    'total_plus_exact', 'total_minus_exact', ...
    'blind_angle_plus', 'blind_angle_minus', ...
    'dvl_scale_error', 'compass_error_deg', 'summary_table');
writetable(summary_table, ...
    fullfile(data_dir, 'step3_beacon_bearing_sweep_summary.csv'));
exportgraphics(gcf, fullfile(data_dir, ...
    'step3_beacon_bearing_sweep.png'), 'Resolution', 180);


function zero_angle = find_zero_crossings(angle_deg, value)
% 对相邻扫描点之间的过零位置进行线性插值。
    zero_angle = [];
    for i = 1:numel(value) - 1
        if value(i) == 0
            zero_angle(end + 1, 1) = angle_deg(i); %#ok<AGROW>
        elseif value(i) * value(i + 1) < 0
            ratio = -value(i) / (value(i + 1) - value(i));
            zero_angle(end + 1, 1) = angle_deg(i) + ...
                ratio * (angle_deg(i + 1) - angle_deg(i)); %#ok<AGROW>
        end
    end
    zero_angle = unique(round(zero_angle, 6));
end


function text_out = angle_list_text(angle_deg)
% 将角度数组转换为便于在命令行阅读的文字。
    if isempty(angle_deg)
        text_out = '未找到';
        return;
    end
    text_out = strjoin(compose('%.2f°', angle_deg), '、');
end
