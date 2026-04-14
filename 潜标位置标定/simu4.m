%% 深海潜标：统一漂移模型下 1/2/3 观测点性能对比
clear; clc; close all;
rng(1); % 固定随机种子

% --- 1. 基础物理参数 (真实值 _true) ---
side_length_true = 20000;          
H_emitter_true = -1000;            
Z_vehicle_true = -1200;            
deg2rad = pi/180; rad2deg = 180/pi;

% 设定真实统一漂移
true_theta_deg = 8;           
true_phi_deg = 45;            
theta_rad_true = true_theta_deg * deg2rad;
phi_rad_true = true_phi_deg * deg2rad;

% --- 2. 产生潜标真实坐标 ---
R_tri_true = side_length_true / sqrt(3); 
P_gnss_true = [0, R_tri_true, 0;
    -side_length_true/2, -R_tri_true/2, 0; 
    side_length_true/2, -R_tri_true/2, 0];

% 产生发声点真实坐标
d_offset_true = abs(H_emitter_true) * tan(theta_rad_true);
dx_true = d_offset_true * sin(phi_rad_true); 
dy_true = d_offset_true * cos(phi_rad_true); 
P_emitters_true = P_gnss_true + [dx_true, dy_true, H_emitter_true];

% --- 3. 产生 3 个航行器观测位置 (真实值 _true) ---
mid_12_true = (P_gnss_true(1,:) + P_gnss_true(2,:)) / 2;
V1_xy_true = ([0,0,0] + P_gnss_true(1,:) + mid_12_true) / 3; 
P_vehicle_true(1,:) = [V1_xy_true(1), V1_xy_true(2), Z_vehicle_true];
P_vehicle_true(2,:) = P_vehicle_true(1,:) + [5000, 5000, 0];
P_vehicle_true(3,:) = P_vehicle_true(2,:) + [100, 100, 0];

% --- 4. 模拟传感器观测值 (含噪声测量值) ---
% A. GNSS 测量误差 (假设 0.5m)
P_gnss = P_gnss_true + [0.5 * randn(3,2), zeros(3,1)]; 

% B. 深度计误差 (假设 1.0m)
H_emitter = H_emitter_true + 1.0 * randn(); 
Z_vehicle = Z_vehicle_true + 1.0 * randn();

% C. 航行器自身定位参考误差 (用于算法输入，假设 5m)
P_vehicle = P_vehicle_true + [5 * randn(3,2), repmat(Z_vehicle - Z_vehicle_true, 3, 1)];

% D. 声学测距观测值
noise_std = 8; 
R_obs = zeros(3, 3); 
for j = 1:3 % 观测点
    for i = 1:3 % 潜标
        dist_real = norm(P_emitters_true(i,:) - P_vehicle_true(j,:));
        R_obs(i,j) = dist_real + noise_std * randn();
    end
end

pngname = 'F4-1.png';
plot_beacons_auv(P_gnss,P_emitters_true, H_emitter,P_vehicle_true,true_theta_deg,true_phi_deg,R_obs,pngname);
%% --- 5. 逐级联合估计 (仅使用测量值) ---
options = optimoptions('lsqnonlin', 'Display', 'none');
H_diff_est = Z_vehicle - H_emitter; % 估计用的深度差

% 情况 A: 1个观测点 (利用 P_vehicle(1,:) 对3个潜标的观测)
obj1 = @(x) sqrt(sum((P_vehicle(1,1:2) - (P_gnss(:,1:2) + x)).^2, 2) + H_diff_est^2) - R_obs(:,1);
res1 = lsqnonlin(obj1, [0, 0], [], [], options);

% 情况 B: 2个观测点联合
obj2 = @(x) [
    sqrt(sum((P_vehicle(1,1:2) - (P_gnss(:,1:2) + x)).^2, 2) + H_diff_est^2) - R_obs(:,1);
    sqrt(sum((P_vehicle(2,1:2) - (P_gnss(:,1:2) + x)).^2, 2) + H_diff_est^2) - R_obs(:,2)
];
res2 = lsqnonlin(obj2, [0, 0], [], [], options);

% 情况 C: 3个观测点联合
obj3 = @(x) [
    sqrt(sum((P_vehicle(1,1:2) - (P_gnss(:,1:2) + x)).^2, 2) + H_diff_est^2) - R_obs(:,1);
    sqrt(sum((P_vehicle(2,1:2) - (P_gnss(:,1:2) + x)).^2, 2) + H_diff_est^2) - R_obs(:,2);
    sqrt(sum((P_vehicle(3,1:2) - (P_gnss(:,1:2) + x)).^2, 2) + H_diff_est^2) - R_obs(:,3)
];
res3 = lsqnonlin(obj3, [0, 0], [], [], options);



% % 预处理：将 3 个观测点对 3 个潜标的斜距全部转换为水平距离观测值
% % H_diff_est 为由深度计测量值计算出的估计深度差 [cite: 3]
% D_obs = sqrt(max(0, R_obs.^2 - H_diff_est^2)); 
% 
% % 情况 A: 1个观测点 (3个水平距离方程)
% % 目标函数减去了垂直分量，变为纯二维平面距离逼近 
% obj1 = @(x) sqrt(sum((P_vehicle(1,1:2) - (P_gnss(:,1:2) + x)).^2, 2)) - D_obs(:,1);
% res1 = lsqnonlin(obj1, [0, 0], [], [], options);
% 
% % 情况 B: 2个观测点联合 (6个水平距离方程)
% obj2 = @(x) [
%     sqrt(sum((P_vehicle(1,1:2) - (P_gnss(:,1:2) + x)).^2, 2)) - D_obs(:,1);
%     sqrt(sum((P_vehicle(2,1:2) - (P_gnss(:,1:2) + x)).^2, 2)) - D_obs(:,2)
% ];
% res2 = lsqnonlin(obj2, [0, 0], [], [], options);
% 
% % 情况 C: 3个观测点联合 (9个水平距离方程)
% obj3 = @(x) [
%     sqrt(sum((P_vehicle(1,1:2) - (P_gnss(:,1:2) + x)).^2, 2)) - D_obs(:,1);
%     sqrt(sum((P_vehicle(2,1:2) - (P_gnss(:,1:2) + x)).^2, 2)) - D_obs(:,2);
%     sqrt(sum((P_vehicle(3,1:2) - (P_gnss(:,1:2) + x)).^2, 2)) - D_obs(:,3)
% ];
% res3 = lsqnonlin(obj3, [0, 0], [], [], options);
%% --- 6. 综合分析：测距维度 vs 坐标维度 ---
% 1. 产生补偿后的估计坐标
P_est_list = {
    P_gnss + [repmat(res1, 3, 1), repmat(H_emitter, 3, 1)], ... 
    P_gnss + [repmat(res2, 3, 1), repmat(H_emitter, 3, 1)], ... 
    P_gnss + [repmat(res3, 3, 1), repmat(H_emitter, 3, 1)]      
};

range_res = zeros(3, 4); % 测距残差: Naive, 1pt, 2pt, 3pt
pos_errs = zeros(3, 3);  % 坐标误差: 1pt, 2pt, 3pt
for i = 1:3 
    % --- A. 计算测距拟合残差 (基于测量值对比) ---
    % 标定前 (Naive)
    range_res(i, 1) = abs(norm(P_vehicle(1,:) - [P_gnss(i,1:2), H_emitter]) - R_obs(i,1));
    % 标定后 1/2/3 点
    for s = 1:3
        P_curr_est = P_est_list{s}(i,:);
        temp_res = zeros(s, 1);
        for pt = 1:s 
            temp_res(pt) = abs(norm(P_vehicle(pt,:) - P_curr_est) - R_obs(i,pt));
        end
        range_res(i, s+1) = mean(temp_res);
    end
    
    % --- B. 计算坐标真值误差 (与 P_emitters_true 对比) ---
    for s = 1:3
        pos_errs(i, s) = norm(P_est_list{s}(i,:) - P_emitters_true(i,:));
    end
end

%% --- 6A. 模块 A: 坐标维度 (真值估计精度) ---
% 1. 计算位移矢量误差 (m)
vec_errs = [norm(res1 - [dx_true, dy_true]), ...
            norm(res2 - [dx_true, dy_true]), ...
            norm(res3 - [dx_true, dy_true])];

% 2. 汇总各场景下 3 个潜标的坐标 RMSE
pos_rmse_scenarios = [rms(pos_errs(:,1)), rms(pos_errs(:,2)), rms(pos_errs(:,3))];

% 3. 命令行报告
fprintf('\n================ 模块 A: 坐标维度精度报告 (单位: m) =================\n');
fprintf('真实漂移矢量 : dx = %.2f, dy = %.2f\n', dx_true, dy_true);
fprintf('1点估计结果   : dx = %.2f, dy = %.2f -> 矢量偏差: %.2f\n', res1(1), res1(2), vec_errs(1));
fprintf('2点联合结果   : dx = %.2f, dy = %.2f -> 矢量偏差: %.2f\n', res2(1), res2(2), vec_errs(2));
fprintf('3点联合结果   : dx = %.2f, dy = %.2f -> 矢量偏差: %.2f\n', res3(1), res3(2), vec_errs(3));
fprintf('--------------------------------------------------------------------\n');
fprintf('坐标总精度(RMSE): 1点标定 = %.3f | 2点联合 = %.3f | 3点联合 = %.3f\n', ...
        pos_rmse_scenarios(1), pos_rmse_scenarios(2), pos_rmse_scenarios(3));
fprintf('====================================================================\n');

% 4. 可视化
fig = myfigurestartup(3, 3, 'paper');
bar(pos_errs, 'grouped'); grid on;
set(gca, 'XTick', 1:size(pos_errs, 1)); 
set(gca, 'XTickLabel', {'潜标 #1', '潜标 #2', '潜标 #3'});
ylabel('坐标点位误差 (m)');
title('坐标维度：真值估计误差 (越小说明标定越准)');
legend('1点估计', '2点联合', '3点联合');
exportgraphics(fig, fullfile('', 'F4-2.png'), 'Resolution', 600);
%% --- 6B. 模块 B: 测距维度 (模型拟合残差) ---
% 1. 计算测距维度的汇总数据 (基于之前 loop 已计算的 range_res)
range_rmse_scenarios = [rms(range_res(:,2)), rms(range_res(:,3)), rms(range_res(:,4))];

% 2. 命令行报告
fprintf('\n================ 模块 B: 测距维度残差报告 (单位: m) =================\n');
fprintf('说明：残差代表估计位置与实际测距值的匹配度。1点标定通常因过拟合导致残差极小。\n');
fprintf('潜标编号 | 1点补偿残差 | 2点联合残差 | 3点联合残差\n');
fprintf('--------------------------------------------------------------------\n');
for i = 1:3
    fprintf('  #%d     |   %8.4f   |   %8.4f   |   %8.4f\n', ...
            i, range_res(i,2), range_res(i,3), range_res(i,4));
end
fprintf('--------------------------------------------------------------------\n');
fprintf('测距拟合RMSE: 1点标定 = %.3f | 2点联合 = %.3f | 3点联合 = %.3f\n', ...
        range_rmse_scenarios(1), range_rmse_scenarios(2), range_rmse_scenarios(3));
fprintf('====================================================================\n');

% 3. 可视化
fig = myfigurestartup(3, 3, 'paper');
bar(range_res(:, 2:4), 'grouped'); grid on;
set(gca, 'XTick', 1:size(pos_errs, 1)); 
set(gca, 'XTickLabel', {'潜标 #1', '潜标 #2', '潜标 #3'});
ylabel('测距绝对残差 (m)');
title('测距维度：模型适配残差 (反映数据拟合优度)');
legend('1点补偿', '2点联合', '3点联合');
exportgraphics(fig, fullfile('', 'F4-3.png'), 'Resolution', 600);