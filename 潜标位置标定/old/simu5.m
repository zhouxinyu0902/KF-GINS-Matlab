%% 深海潜标：单航行器对单潜标(Beacon #3) 1/2/3点估计性能研究 (含误差修正版)
clear; clc; close all;
rng(1); % 固定随机种子

% --- 1. 基础物理参数 (真实值 _true) ---
side_length_true = 20000;          
H_emitter_true = -1000;            
Z_vehicle_true = -1200;            
deg2rad = pi/180; rad2deg = 180/pi;

% 设定漂移真值
true_theta_deg = 8; 
true_phi_deg = 45;
d_offset_true = abs(H_emitter_true) * tan(true_theta_deg * deg2rad);
dx_true = d_offset_true * sin(true_phi_deg * deg2rad); 
dy_true = d_offset_true * cos(true_phi_deg * deg2rad); 
true_vec = [dx_true, dy_true]; % 标定目标真值矢量

% --- 2. 产生潜标坐标 (真实值) ---
R_tri_true = side_length_true / sqrt(3); 
P_gnss_true = [0, R_tri_true, 0; 
               -side_length_true/2, -R_tri_true/2, 0; 
               side_length_true/2, -R_tri_true/2, 0];

% 选择目标潜标 (ID = 3)
id = 3; 
P_emitters_true  = P_gnss_true + repmat([dx_true, dy_true, H_emitter_true],3,1);
P_emit1_true = P_gnss_true(id,:) + [dx_true, dy_true, H_emitter_true];

% --- 3. 产生 3 个航行器观测位置 (真实值，分布在潜标周围以优化几何) ---
% 点1: 阵列重心附近
P_v_true(1,:) = [0, 0, Z_vehicle_true];
% 点2: 偏向东北方向
P_v_true(2,:) = [5000, 5000, Z_vehicle_true]; 
% 点3: 偏向西北方向 (形成大跨度三角形)
P_v_true(3,:) = [100, 100, Z_vehicle_true]; 

% --- 4. 模拟传感器观测值 (测量值，算法实际可获取的数据) ---
% A. GNSS 测量误差 (假设 0.5m 漂移)
P_gnss_meas = P_gnss_true + [0.5 * randn(3,2), zeros(3,1)];
P_gnss1_meas = P_gnss_meas(id,1:2); % 目标潜标的名义水平坐标

% B. 深度计测量误差 (假设 1.0m 标准差)
H_emitter = H_emitter_true + 1.0 * randn(); 
Z_vehicle = Z_vehicle_true + 1.0 * randn();

% C. 航行器位置参考误差
% 模拟实际中定位系统的系统性偏差
P_v_used = P_v_true + [5 * randn(3,2), repmat(Z_vehicle - Z_vehicle_true, 3, 1)];

% D. 声学测距观测值 (含 8m 噪声)
noise_std = 8; 
R_obs_1 = zeros(3,1);
for j = 1:3
    dist_real = norm(P_emit1_true - P_v_true(j,:));
    R_obs_1(j) = dist_real + noise_std * randn();
end
% pngname = 'F5-1.png';
% plot_beacons_auv(P_gnss_meas,P_emitters_true, H_emitter,P_v_used,true_theta_deg,true_phi_deg,R_obs,pngname);
%% --- 5. 逐级估计 (仅使用测量值) ---
% % 使用 LM 算法以增强收敛稳定性
options = optimoptions('lsqnonlin', 'Display', 'none', 'Algorithm', 'levenberg-marquardt');
H_diff_est_sq = (Z_vehicle - H_emitter)^2;

% 情况 A: 1点观测
obj1 = @(x) sqrt((P_v_used(1,1)-(P_gnss1_meas(1)+x(1)))^2 + (P_v_used(1,2)-(P_gnss1_meas(2)+x(2)))^2 + H_diff_est_sq) - R_obs_1(1);
res1 = lsqnonlin(obj1, [0, 0], [], [], options);

% 情况 B: 2点观测
obj2 = @(x) [
    sqrt((P_v_used(1,1)-(P_gnss1_meas(1)+x(1)))^2 + (P_v_used(1,2)-(P_gnss1_meas(2)+x(2)))^2 + H_diff_est_sq) - R_obs_1(1);
    sqrt((P_v_used(2,1)-(P_gnss1_meas(1)+x(1)))^2 + (P_v_used(2,2)-(P_gnss1_meas(2)+x(2)))^2 + H_diff_est_sq) - R_obs_1(2)
];
res2 = lsqnonlin(obj2, [0, 0], [], [], options);

% 情况 C: 3点观测
obj3 = @(x) [
    sqrt((P_v_used(1,1)-(P_gnss1_meas(1)+x(1)))^2 + (P_v_used(1,2)-(P_gnss1_meas(2)+x(2)))^2 + H_diff_est_sq) - R_obs_1(1);
    sqrt((P_v_used(2,1)-(P_gnss1_meas(1)+x(1)))^2 + (P_v_used(2,2)-(P_gnss1_meas(2)+x(2)))^2 + H_diff_est_sq) - R_obs_1(2);
    sqrt((P_v_used(3,1)-(P_gnss1_meas(1)+x(1)))^2 + (P_v_used(3,2)-(P_gnss1_meas(2)+x(2)))^2 + H_diff_est_sq) - R_obs_1(3)
];
res3 = lsqnonlin(obj3, [0, 0], [], [], options);


% % 1. 预处理：计算观测到的水平投影距离 D_obs 
% % H_diff_est_sq 为由深度计测量值计算出的估计深度差平方 (Z_vehicle - H_emitter)^2 [cite: 9]
% % 使用 max(0, ...) 确保在噪声较大时不会出现负值导致虚数结果
% D_obs_1_horiz = sqrt(max(0, R_obs_1.^2 - H_diff_est_sq)); 
% 
% % 2. 定义水平距离目标函数 (去掉了根号内的深度项 H_diff_est_sq) [cite: 31]
% % 情况 A: 1点观测 (1个水平距离方程)
% obj1 = @(x) sqrt((P_v_used(1,1)-(P_gnss1_meas(1)+x(1)))^2 + (P_v_used(1,2)-(P_gnss1_meas(2)+x(2)))^2) - D_obs_1_horiz(1);
% res1 = lsqnonlin(obj1, [0, 0], [], [], options);
% 
% % 情况 B: 2点观测 (2个水平距离方程)
% obj2 = @(x) [
%     sqrt((P_v_used(1,1)-(P_gnss1_meas(1)+x(1)))^2 + (P_v_used(1,2)-(P_gnss1_meas(2)+x(2)))^2) - D_obs_1_horiz(1);
%     sqrt((P_v_used(2,1)-(P_gnss1_meas(1)+x(1)))^2 + (P_v_used(2,2)-(P_gnss1_meas(2)+x(2)))^2) - D_obs_1_horiz(2)
% ];
% res2 = lsqnonlin(obj2, [0, 0], [], [], options);
% 
% % 情况 C: 3点观测 (3个水平距离方程)
% obj3 = @(x) [
%     sqrt((P_v_used(1,1)-(P_gnss1_meas(1)+x(1)))^2 + (P_v_used(1,2)-(P_gnss1_meas(2)+x(2)))^2) - D_obs_1_horiz(1);
%     sqrt((P_v_used(2,1)-(P_gnss1_meas(1)+x(1)))^2 + (P_v_used(2,2)-(P_gnss1_meas(2)+x(2)))^2) - D_obs_1_horiz(2);
%     sqrt((P_v_used(3,1)-(P_gnss1_meas(1)+x(1)))^2 + (P_v_used(3,2)-(P_gnss1_meas(2)+x(2)))^2) - D_obs_1_horiz(3)
% ];
% res3 = lsqnonlin(obj3, [0, 0], [], [], options);
%% --- 6. 精度与残差分析 ---
% A. 坐标维度误差计算 (与真值对比)
vec_errs = [norm(res1 - true_vec), norm(res2 - true_vec), norm(res3 - true_vec)];

% B. 测距维度残差计算 (基于测量值 P_v_used 进行闭环验证)
indiv_residuals = nan(3, 3); 
P_e_list = {res1, res2, res3};
for s = 1:3 % 场景 (1, 2, 3点估计)
    P_est = P_gnss1_meas + P_e_list{s};
    for j = 1:s % 计算该场景下参与标定的每一个观测点的残差
        indiv_residuals(j, s) = abs(sqrt(sum((P_v_used(j,1:2) - P_est).^2) + H_diff_est_sq) - R_obs_1(j));
    end
end

% --- 7. 报告输出 ---
fprintf('\n================ 模块 A: 坐标维度精度报告 (Beacon #%d) =================\n', id);
fprintf('真实漂移真值 : dx = %.2f m, dy = %.2f m\n', dx_true, dy_true);
fprintf('1点估计结果   : dx = %.2f m, dy = %.2f m -> 误差: %.2f m\n', res1(1), res1(2), vec_errs(1));
fprintf('2点联合结果   : dx = %.2f m, dy = %.2f m -> 误差: %.2f m\n', res2(1), res2(2), vec_errs(2));
fprintf('3点联合结果   : dx = %.2f m, dy = %.2f m -> 误差: %.2f m\n', res3(1), res3(2), vec_errs(3));
fprintf('----------------------------------------------------------------------\n');

% --- 8. 绘图对比 ---
% 这里省略绘图细节，逻辑与之前一致，关键在于数据源已切换为上述修正后的变量

% 绘图：坐标误差演进

fig = myfigurestartup(3,3,'paper');
% 1. 确保只绘制前三个数据，并明确指定 X 轴位置为 1, 2, 3
h_bar = bar(1:3, vec_errs(1:3), 0.5, 'FaceColor', [0.2 0.4 0.6]);
grid on;
set(gca, 'XTick', 1:3); % 强制只在 1, 2, 3 设刻度
set(gca, 'XTickLabel', {'1点估计', '2点联合', '3点联合'}); % 一一对应
set(gca, 'XLim', [0.5, 3.5]); % 锁定 X 轴显示范围，不给多余刻度留空间
ylabel('位移矢量误差 (m)'); 
title('坐标维度：定位精度改善');
% 3. 标注数值 (确保 text 的 X 坐标也是 1, 2, 3)
xtips = 1:3;
ytips = vec_errs(1:3);
text(xtips, ytips, string(round(ytips,1))+"m", ...
    'HorizontalAlignment','center', ...
    'VerticalAlignment','bottom', ...
    'FontWeight', 'bold', ...
    'FontSize', 9);
% 适当抬高 Y 轴上限，防止文字贴顶
ylim([0, max(vec_errs(1:3))*1.2]);
exportgraphics(fig, 'F5-2.png', 'Resolution', 600);
%% --- 6B. 测距维度：各观测位置残差分解 ---
% 这里的残差计算公式为：r = |dist(P_v_used, P_est) - R_obs| [cite: 12, 19]
% indiv_residuals(观测点j, 场景s): 记录第 s 种估计场景下，第 j 个观测点的适配残差
indiv_residuals = nan(3, 3); 

% 预存储估计结果列表
P_e_list = {res1, res2, res3};

for s = 1:3 % 遍历 1点、2点联合、3点联合三种场景
    % 算法得到的估计发声点位置 (基于测量名义坐标 + 估计出的漂移矢量) [cite: 13]
    P_est = [P_gnss1_meas(1) + P_e_list{s}(1), P_gnss1_meas(2) + P_e_list{s}(2), H_emitter];
    
    for j = 1:s % 仅计算参与了该场景建模的观测点
        % 计算该观测点到估计位置的推算距离
        dist_calc = sqrt(sum((P_v_used(j,1:2) - P_est(1:2)).^2) + (Z_vehicle - H_emitter)^2);
        
        % 计算残差：推算距离与实际声学观测值 R_obs_1 的差值 [cite: 19]
        indiv_residuals(j, s) = abs(dist_calc - R_obs_1(j));
    end
end

fprintf('\n================ 模块 B: 测距维度残差报告 (拟合优度) =================\n');
fprintf('说明：1点估计通常因过拟合导致残差极小；多点联合则展示了模型无法吸收的测距噪声 。\n');
fprintf('观测位置 | 1点场景残差 | 2点场景残差 | 3点场景残差\n');
for j = 1:3
    fprintf(' 位置 #%d |   %8.4f   |   %8.4f   |   %8.4f\n', ...
        j, indiv_residuals(j,1), indiv_residuals(j,2), indiv_residuals(j,3));
end
fprintf('======================================================================\n');

% --- 绘图：测距残差分布 ---
fig = myfigurestartup(3, 3, 'paper');
% 转置矩阵以使 X 轴代表不同的“联合估计场景”
b = bar(indiv_residuals', 'grouped'); 
grid on; 

set(gca, 'XTickLabel', {'1点估计场景', '2点联合场景', '3点联合场景'});
ylabel('测距绝对残差 (m)'); 
title('测距维度：各观测点对标定模型的适配残差');
legend({'位置 #1 (中心)', '位置 #2 (偏东)', '位置 #3 (偏西)'}, 'Location', 'northeast', 'FontSize', 7);

% 标注数值
for i = 1:3
    xtips = b(i).XEndPoints; 
    ytips = b(i).YEndPoints;
    labels = string(round(ytips, 2));
    mask = ~isnan(ytips); % 仅标注非 NaN 的数据点
    text(xtips(mask), ytips(mask), labels(mask), ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', ...
        'FontSize', 8, 'FontWeight', 'bold');
end

ylim([0, 15]); % 测距噪声 std=8，残差范围设为 0-15 较为合理
exportgraphics(fig, 'F5-3.png', 'Resolution', 600);