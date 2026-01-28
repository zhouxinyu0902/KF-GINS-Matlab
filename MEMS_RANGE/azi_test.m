%% 方位角误差模型对比脚本：线光滑 vs 非线性不规律
clc; clear; close all;

% --- 参数设置 ---
num_samples = 3000;
true_azi = linspace(-180, 180, num_samples);

% 基础参数
p.sigma_min = 0.5;  % 最佳精度
p.sigma_max = 5.0;  % 最差精度

% --- 模型 1: 旧版 (完全线性，光滑) ---
% 相当于 power_fac = 1, jitter = 0
p_old = p;
p_old.power_fac = 1.0;
p_old.jitter    = 0.0; 
meas_old = add_azimuth_noise_irregular(true_azi, p_old);
err_old = mod(meas_old - true_azi + 180, 360) - 180;

% --- 模型 2: 新版 (非线性，不规律) ---
% power_fac = 2 (使用 sin平方，这是许多物理场的特性，形状更陡峭)
% jitter = 0.25 (标准差本身有 +/- 25% 的随机波动，增加不规律感)
p_new = p;
p_new.power_fac = 2.0; 
p_new.jitter    = 0.25; 
meas_new = add_azimuth_noise_irregular(true_azi, p_new);
err_new = mod(meas_new - true_azi + 180, 360) - 180;

%% --- 绘图对比 ---
figure('Color', 'w', 'Position', [100, 100, 1200, 500]);

% 子图1: 旧版线性光滑模型
subplot(1, 2, 1);
scatter(true_azi, err_old, 10, 'b', 'filled', 'MarkerFaceAlpha', 0.2); hold on;
% 计算旧版理论包络以便对比
title_str = sprintf('旧版: 线性光滑 (|sin(\\theta)|)\n规律性太强');
ylabel('测量误差 (deg)'); xlabel('真实方位角 (deg)');
grid on; xlim([-180, 180]); ylim([-15, 15]);
title(title_str);

% 子图2: 新版非线性不规律模型
subplot(1, 2, 2);
% 绘制散点
scatter(true_azi, err_new, 10, 'r', 'filled', 'MarkerFaceAlpha', 0.2); hold on;

% 绘制基础理论曲线(不带抖动)作为参考
base_sigma_curve = p.sigma_min + (p.sigma_max - p.sigma_min) * abs(sin(deg2rad(true_azi))).^p_new.power_fac;
plot(true_azi, 3*base_sigma_curve, 'k--', 'LineWidth', 1.5, 'DisplayName', '基础3\sigma曲线(无抖动)');
plot(true_azi, -3*base_sigma_curve, 'k--', 'LineWidth', 1.5, 'HandleVisibility','off');

title_str = sprintf('新版: 非线性+不规律 (sin^2(\\theta) + Jitter)\n更接近真实情况');
legend('单次测量误差', '基础理论包络参考');
xlabel('真实方位角 (deg)');
grid on; xlim([-180, 180]); ylim([-15, 15]);
title(title_str);

%% 结果分析打印
fprintf('--- 模型参数对比 ---\n');
fprintf('旧版: 形状指数=%.1f (线性), 抖动强度=%.1f (无)\n', p_old.power_fac, p_old.jitter);
fprintf('新版: 形状指数=%.1f (非线性凹陷), 抖动强度=%.1f (有随机波动)\n', p_new.power_fac, p_new.jitter);
fprintf('观察右图：\n 1. 黑色虚线展示了 sin^2 的形状，比左图更陡峭。\n 2. 红色散点并没有完美贴合黑色虚线，而是由于 Jitter 的存在，在包络线附近随机跳动，看起来更“杂乱”。\n');