function plot_beacon_data(beaconddm, beaconxyz)
%PLOT_BEACON_DATA 绘制信标的绝对经纬度位置和相对本地坐标系位置。
%
%   PLOT_BEACON_DATA(beaconddm, beaconxyz) 接受以下输入：
%   - beaconddm: Nx3 矩阵，表示N个信标的绝对位置 [纬度(度), 经度(度), 高度(米)]。
%                这里的 N 是信标数量（例如 3 个）。
%   - beaconxyz: Nx3 矩阵，表示N个信标相对于本地坐标系原点的XYZ位移，单位为米。
%                通常第一行为 [0,0,0] 表示本地原点，后续行为其他信标的相对位移。
%
%   该函数将绘制两个图：
%   1. 信标的绝对位置（经纬度）。
%   2. 信标在本地坐标系中的相对位置（千米）。
%
%   示例用法（假设 beaconddm 和 beaconxyz 已在工作空间中定义）：
%   plot_beacon_data(beaconddm, beaconxyz);

% --- 绘图 1: 绝对经纬度图 ---
myfigurestartup(5, 5, 'prese'); % 设置图窗大小和预设样式
plot_markers = {'ro', 'g^', 'bs', 'mD', 'cv', 'y>'}; % 定义一组标记样式和颜色

hold on; % 保持当前图窗，以便绘制多个点

% 循环绘制每个信标的绝对经纬度位置
num_beacons_abs = size(beaconddm, 1);
for i = 1:num_beacons_abs
    % 绘制经度（X轴）对纬度（Y轴）
    plot(beaconddm(i, 2), beaconddm(i, 1), ...
         plot_markers{mod(i-1, length(plot_markers)) + 1}, ... % 循环使用标记样式
         'MarkerSize', 8, ... % 设置标记大小
         'DisplayName', sprintf('绝对信标 %d', i)); % 添加图例名称
end

xlabel('经度 (度)'); % 设置X轴标签
ylabel('纬度 (度)'); % 设置Y轴标签
title('信标绝对位置（经纬度）'); % 设置图表标题
grid on; % 显示网格线
axis equal; % 确保地理坐标的横纵轴比例一致，避免图形变形
legend('show', 'Location', 'best'); % 显示图例
hold off; % 释放 hold 状态

% --- 绘图 2: 相对本地坐标系图（千米） ---
% 将 beaconxyz 转换为千米单位，用于本地坐标系绘图
local_beacons_km = beaconxyz / 1000;

myfigurestartup(5, 5, 'prese'); % 设置新图窗
hold on; % 保持当前图窗，以便绘制多个点

% 循环绘制每个信标在本地坐标系中的相对位置
num_beacons_local = size(local_beacons_km, 1);
for i = 1:num_beacons_local
    % 绘制东向（X轴）对北向（Y轴）
    plot(local_beacons_km(i, 1), local_beacons_km(i, 2), ...
         plot_markers{mod(i-1, length(plot_markers)) + 1}, ... % 循环使用标记样式
         'MarkerSize', 8, ... % 设置标记大小
         'DisplayName', sprintf('本地信标 %d', i)); % 添加图例名称
    text(local_beacons_km(i, 1), local_beacons_km(i, 2), ...
        ['(',num2str(local_beacons_km(i, 1)),',',num2str(local_beacons_km(i, 2)),')'])
end

xlabel('东向 (km)'); % 设置X轴标签
ylabel('北向 (km)'); % 设置Y轴标签
title('信标相对位置（本地坐标系）'); % 设置图表标题
grid on; % 显示网格线
axis equal; % 确保横纵轴比例一致
legend('show', 'Location', 'best'); % 显示图例
hold off; % 释放 hold 状态

end