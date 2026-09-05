%% =========================================================================
% 导出数据 TXT 可视化检查器 (Data Viewer)
% 功能：读取 input 文件夹中的 TXT，同框对比多设备的轨迹、姿态、速度
% =========================================================================
clear; clc; close all;

% ==================== 1. 配置要检查的组别 ====================
target_round = 1; % 【修改这里】你想检查第几组的数据？(如 1, 2, 7, 8)

% 设定基础路径
data_dir = sprintf('D:\\Github\\KF-GINS-Matlab\\data\\experiment-data\\experiment-01\\case-0%d\\input', target_round);
fprintf('正在加载 Round %d 的数据进行检查...\n路径: %s\n', target_round, data_dir);

% ==================== 2. 安全加载 TXT 数据 ====================
% 预定义空矩阵，防止某些文件不存在时报错
pva_120 = []; pva_430 = []; pva_830 = [];
gps_430 = []; gps_830 = [];

% 加载 PVA 导航解算数据 (格式: [week, sec, lat, lon, alt, vn, ve, vu, roll, pitch, yaw])
if exist(fullfile(data_dir, 'pva_file.txt'), 'file'), pva_120 = readmatrix(fullfile(data_dir, 'pva_file.txt')); end
if exist(fullfile(data_dir, 'pva_430.txt'), 'file'),  pva_430 = readmatrix(fullfile(data_dir, 'pva_430.txt')); end
if exist(fullfile(data_dir, 'pva_830.txt'), 'file'),  pva_830 = readmatrix(fullfile(data_dir, 'pva_830.txt')); end

% 加载 GPS 观测数据 (格式: [gnss_seconds, time, lat, lon, alt, gps_status])
if exist(fullfile(data_dir, 'gps_430.txt'), 'file'),  gps_430 = readmatrix(fullfile(data_dir, 'gps_430.txt')); end
if exist(fullfile(data_dir, 'gps_830.txt'), 'file'),  gps_830 = readmatrix(fullfile(data_dir, 'gps_830.txt')); end

if isempty(pva_120) && isempty(pva_430) && isempty(pva_830)
    error('未在该路径下找到任何 pva 文件，请检查路径是否正确！');
end

% ==================== 3. 可视化检查绘图 ====================
set(0, 'DefaultLineLineWidth', 1.5); % 全局线宽变粗，方便观察

%% 🎯 图 1：2D 地理轨迹同框对比 (检查轨迹重合度)
% figure('Name', sprintf('Round %d - 2D 轨迹对比', target_round), 'Position', [100, 100, 800, 600]);
myfigurestartup(5,5,'prese');
hold on; grid on; axis equal;

% 以其中一个有数据的作为相对原点 (减小数值，防止坐标轴失真)
ref_lat = NaN; ref_lon = NaN;
if ~isempty(pva_120), ref_lat = pva_120(1,3); ref_lon = pva_120(1,4);
elseif ~isempty(pva_430), ref_lat = pva_430(1,3); ref_lon = pva_430(1,4); end

% 绘制 120 轨迹
if ~isempty(pva_120)
    plot((pva_120(:,4)-ref_lon)*111319.9.*cosd(ref_lat), (pva_120(:,3)-ref_lat)*111319.9, 'k', 'DisplayName', '120 导航结果 (基准)');
end
% 绘制 430 轨迹与 GPS
if ~isempty(pva_430)
    plot((pva_430(:,4)-ref_lon)*111319.9.*cosd(ref_lat), (pva_430(:,3)-ref_lat)*111319.9, 'r--', 'DisplayName', '430 组合导航');
end
if ~isempty(gps_430)
    plot((gps_430(:,4)-ref_lon)*111319.9.*cosd(ref_lat), (gps_430(:,3)-ref_lat)*111319.9, 'r.', 'MarkerSize', 5, 'DisplayName', '430 原始GPS');
end
% 绘制 830 轨迹与 GPS
bad_idx = find(pva_830(:,3) > 37 | pva_830(:,3) < 36);
pva_830(bad_idx, :) = [];
if ~isempty(pva_830)
    plot((pva_830(:,4)-ref_lon)*111319.9.*cosd(ref_lat), (pva_830(:,3)-ref_lat)*111319.9, 'b--', 'DisplayName', '830 组合导航');
end
if ~isempty(gps_830)
    plot((gps_830(:,4)-ref_lon)*111319.9.*cosd(ref_lat), (gps_830(:,3)-ref_lat)*111319.9, 'b.', 'MarkerSize', 5, 'DisplayName', '830 原始GPS');
end

xlabel('East 相对距离 (m)'); ylabel('North 相对距离 (m)');
title(sprintf('Round %d - 2D 轨迹同框对比', target_round));
legend('Location', 'best');

%% 🎯 图 2：姿态角对比 (检查坐标系与对齐)
% figure('Name', sprintf('Round %d - 姿态角对比', target_round), 'Position', [150, 150, 900, 700]);
myfigurestartup(5,5,'zxy');
titles = {'Roll (横滚角)', 'Pitch (俯仰角)', 'Yaw (航向角)'};
cols = [9, 10, 11]; % PVA矩阵中姿态所在的列

for i = 1:3
    subplot(3, 1, i); hold on; grid on;
    if ~isempty(pva_120), plot(pva_120(:,2), pva_120(:,cols(i)), 'k', 'DisplayName', '120'); end
    if ~isempty(pva_430), plot(pva_430(:,2), pva_430(:,cols(i)), 'r', 'DisplayName', '430'); end
    if ~isempty(pva_830), plot(pva_830(:,2), pva_830(:,cols(i)), 'b', 'DisplayName', '830'); end
    ylabel('Angle (deg)'); title(titles{i});
    if i == 1, legend('Location', 'best'); end
    if i == 3, xlabel('GPS Time (s)'); end
end

%% 🎯 图 3：速度与高度对比 (检查动态性能)
% figure('Name', sprintf('Round %d - 速度与高度对比', target_round), 'Position', [200, 200, 900, 700]);
myfigurestartup(5,5,'zxy');
% 子图1：北向速度
subplot(3, 1, 1); hold on; grid on;
if ~isempty(pva_120), plot(pva_120(:,2), pva_120(:,6), 'k', 'DisplayName', '120 Vn'); end
if ~isempty(pva_430), plot(pva_430(:,2), pva_430(:,6), 'r', 'DisplayName', '430 Vn'); end
if ~isempty(pva_830), plot(pva_830(:,2), pva_830(:,6), 'b', 'DisplayName', '830 Vn'); end
ylabel('Vn (m/s)'); title('北向速度'); legend('Location', 'best');

% 子图2：东向速度
subplot(3, 1, 2); hold on; grid on;
if ~isempty(pva_120), plot(pva_120(:,2), pva_120(:,7), 'k'); end
if ~isempty(pva_430), plot(pva_430(:,2), pva_430(:,7), 'r'); end
if ~isempty(pva_830), plot(pva_830(:,2), pva_830(:,7), 'b'); end
ylabel('Ve (m/s)'); title('东向速度');

% 子图3：高度对比 (含 GPS)
subplot(3, 1, 3); hold on; grid on;
if ~isempty(pva_120), plot(pva_120(:,2), pva_120(:,5), 'k', 'DisplayName', '120 Alt'); end
if ~isempty(pva_430), plot(pva_430(:,2), pva_430(:,5), 'r', 'DisplayName', '430 Alt'); end
if ~isempty(gps_430), plot(gps_430(:,1), gps_430(:,5), 'r.', 'DisplayName', '430 GPS Alt'); end
if ~isempty(pva_830), plot(pva_830(:,2), pva_830(:,5), 'b', 'DisplayName', '830 Alt'); end
if ~isempty(gps_830), plot(gps_830(:,1), gps_830(:,5), 'b.', 'DisplayName', '830 GPS Alt'); end
ylabel('Altitude (m)'); title('高度对比'); xlabel('GPS Time (s)');
legend('Location', 'best');
%% ==================== 4. 数据时长与对齐状态统计面板 ====================
fprintf('\n======================================================\n');
fprintf(' 📊 各设备数据时间统计面板 (Round %d)\n', target_round);
fprintf('======================================================\n');

% 打印 PVA 时长 (时间列在第 2 列)
if ~isempty(pva_120)
    t_span = pva_120(end,2) - pva_120(1,2);
    fprintf('[120 PVA]  时长: %7.2f 秒 (约 %4.1f 分钟) | 起点: %.3f | 终点: %.3f\n', t_span, t_span/60, pva_120(1,2), pva_120(end,2));
else, fprintf('[120 PVA]  未找到数据\n'); end

if ~isempty(pva_430)
    t_span = pva_430(end,2) - pva_430(1,2);
    fprintf('[430 PVA]  时长: %7.2f 秒 (约 %4.1f 分钟) | 起点: %.3f | 终点: %.3f\n', t_span, t_span/60, pva_430(1,2), pva_430(end,2));
else, fprintf('[430 PVA]  未找到数据\n'); end

if ~isempty(pva_830)
    t_span = pva_830(end,2) - pva_830(1,2);
    fprintf('[830 PVA]  时长: %7.2f 秒 (约 %4.1f 分钟) | 起点: %.3f | 终点: %.3f\n', t_span, t_span/60, pva_830(1,2), pva_830(end,2));
else, fprintf('[830 PVA]  未找到数据\n'); end

fprintf('------------------------------------------------------\n');

% 打印 GPS 时长 (时间列在第 1 列)
if ~isempty(gps_430)
    t_span = gps_430(end,1) - gps_430(1,1);
    fprintf('[430 GPS]  时长: %7.2f 秒 (约 %4.1f 分钟) | 起点: %.3f | 终点: %.3f\n', t_span, t_span/60, gps_430(1,1), gps_430(end,1));
else, fprintf('[430 GPS]  未找到数据\n'); end

if ~isempty(gps_830)
    t_span = gps_830(end,1) - gps_830(1,1);
    fprintf('[830 GPS]  时长: %7.2f 秒 (约 %4.1f 分钟) | 起点: %.3f | 终点: %.3f\n', t_span, t_span/60, gps_830(1,1), gps_830(end,1));
else, fprintf('[830 GPS]  未找到数据\n'); end

fprintf('======================================================\n\n');
fprintf('检查完毕！请查看弹出的图表。\n');