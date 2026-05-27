% =========================================================================
% 导航系统径向误差 (RMS) 综合统计图绘制程序
% 包含两类标定方案：1. 几何阵列中心标定  2. 潜标1附近标定
% 每类方案分别绘制：正常距离辅助 (Origin) 与 算法优化后 (Smooth)
% =========================================================================
clear; clc; close all;

% 统一通用标签设置
traj_labels = {'Square (方形)', 'Line (直线)', 'Circle (圆形)'};
state_labels = {'未修正', '一次标定', '二次标定', '真实位置'};

%% =========================================================================
%  第一部分：几何阵列中心标定数据与绘图
% =========================================================================

% 1.1 阵列中心 - 正常距离辅助 (Origin) 数据
center_origin = [
    651.25, 163.60, 148.59, 147.01;  % Square
    621.81, 197.59, 200.10, 196.32;  % Line
    625.40, 229.86, 214.46, 204.63   % Circle
];

% 1.2 阵列中心 - 算法优化后 (Smooth) 数据
center_smooth = [
    500.67,  94.89,  80.03,  78.42;  % Square
    480.02,  89.10,  91.50,  88.32;  % Line
    471.44, 111.92,  99.04,  92.75   % Circle
];

% ---- 绘图 1A: 阵列中心 - Origin ----
fig1 = figure('Name', '阵列中心 - Origin', 'Position', [100, 100, 720, 520]);
b1 = bar(center_origin, 'grouped'); grid on;
title('均方根误差 (RMS) - 几何阵列中心标定 (Origin)', 'FontSize', 13, 'FontWeight', 'bold');
set(gca, 'XTickLabel', traj_labels, 'FontSize', 11);
ylabel('均方根误差 RMS (m)', 'FontSize', 12);
xlabel('标定轨迹类型', 'FontSize', 12);
legend(state_labels, 'Location', 'northeast', 'FontSize', 10);
for i = 1:length(b1)
    text(b1(i).XEndPoints, b1(i).YEndPoints, string(round(b1(i).YData, 1)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 9);
end
exportgraphics(fig1, '阵列中心标定_正常距离辅助.png', 'Resolution', 600);

% ---- 绘图 1B: 阵列中心 - Smooth ----
fig2 = figure('Name', '阵列中心 - Smooth', 'Position', [150, 150, 720, 520]);
b2 = bar(center_smooth, 'grouped'); grid on;
title('均方根误差 (RMS) - 几何阵列中心标定 (Smooth)', 'FontSize', 13, 'FontWeight', 'bold');
set(gca, 'XTickLabel', traj_labels, 'FontSize', 11);
ylabel('均方根误差 RMS (m)', 'FontSize', 12);
xlabel('标定轨迹类型', 'FontSize', 12);
legend(state_labels, 'Location', 'northeast', 'FontSize', 10);
for i = 1:length(b2)
    text(b2(i).XEndPoints, b2(i).YEndPoints, string(round(b2(i).YData, 1)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 9);
end
exportgraphics(fig2, '阵列中心标定_优化平滑算法.png', 'Resolution', 600);


%% =========================================================================
%  第二部分：潜标1附近标定数据与绘图
% =========================================================================

% 2.1 潜标1附近 - 正常距离辅助 (Origin) 数据
mooring1_origin = [
    666.49, 237.22, 223.98, 220.01;  % Square
    638.19, 161.50, 156.01, 157.72;  % Line
    483.76, 183.36, 180.61, 144.88   % Circle
];

% 2.2 潜标1附近 - 算法优化后 (Smooth) 数据
mooring1_smooth = [
    514.22, 133.41, 115.87, 109.23;  % Square
    509.84,  89.72,  80.94,  80.64;  % Line
    406.95, 111.82, 108.86,  74.17   % Circle
];

% ---- 绘图 2A: 潜标1附近 - Origin ----
fig3 = figure('Name', '潜标1附近 - Origin', 'Position', [200, 200, 720, 520]);
b3 = bar(mooring1_origin, 'grouped'); grid on;
title('均方根误差 (RMS) - 潜标1附近标定 (Origin)', 'FontSize', 13, 'FontWeight', 'bold');
set(gca, 'XTickLabel', traj_labels, 'FontSize', 11);
ylabel('均方根误差 RMS (m)', 'FontSize', 12);
xlabel('标定轨迹类型', 'FontSize', 12);
legend(state_labels, 'Location', 'northeast', 'FontSize', 10);
for i = 1:length(b3)
    text(b3(i).XEndPoints, b3(i).YEndPoints, string(round(b3(i).YData, 1)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 9);
end
exportgraphics(fig3, '潜标1附近标定_正常距离辅助.png', 'Resolution', 600);

% ---- 绘图 2B: 潜标1附近 - Smooth ----
fig4 = figure('Name', '潜标1附近 - Smooth', 'Position', [250, 250, 720, 520]);
b4 = bar(mooring1_smooth, 'grouped'); grid on;
title('均方根误差 (RMS) - 潜标1附近标定 (Smooth)', 'FontSize', 13, 'FontWeight', 'bold');
set(gca, 'XTickLabel', traj_labels, 'FontSize', 11);
ylabel('均方根误差 RMS (m)', 'FontSize', 12);
xlabel('标定轨迹类型', 'FontSize', 12);
legend(state_labels, 'Location', 'northeast', 'FontSize', 10);
for i = 1:length(b4)
    text(b4(i).XEndPoints, b4(i).YEndPoints, string(round(b4(i).YData, 1)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 9);
end
exportgraphics(fig4, '潜标1附近标定_优化平滑算法.png', 'Resolution', 600);