%% --- 0. 参数设置与预分配 ---
% 确定循环长度：取量测数据和惯导数据(降采样后)的最小值，防止越界
len_data = min(size(rangedata, 1), floor(size(navforward, 1) / 100));

% 预分配内存 (显著提高循环速度)
pos_xy = zeros(len_data, 2);      % 相对位置 [East, North]
pos_est_rad = zeros(len_data, 3); % 估计位置 [Lat, Lon, H] (弧度)

% 提取信标位置 (假设信标位置固定或随 rangedata 变化)
beacon_pos_lla = rangedata(:, 4:6); 

%% --- 1. 使用数据进行定位解算 ---
for i = 1:len_data
    % 提取当前时刻的量测值
    curr_range = rangedata(i, 3);
    curr_azi   = rangedata(i, 7);
    
    % 提取对应时刻的惯导航向 (假设 navforward 是 100Hz，取第 i*100 个点)
    % 注意：navforward 最后一列通常是航向
    curr_heading = navforward(i * 100, end); 
    
    % 1. 计算相对位置 (相对信标的 EN 坐标)
    pos_xy(i, :) = calc_position_from_beacon([0,0], curr_range, curr_azi, curr_heading);
    
    % 2. 转换绝对位置 (叠加信标经纬度)
    % dxyz2pos 输入: [dE, dN, dU], 参考点LLA
    % 输出: [Lat, Lon, H] (通常为弧度，取决于工具箱定义，此处假设为弧度)
    pos_est_rad(i, :) = dxyz2pos([pos_xy(i, :), 0], beacon_pos_lla(i, :)');
end

%% --- 2. 误差计算 ---
% 提取对应的真值 (降采样以匹配估计值)
idx_slice = (1:len_data) * 100; 
truth_lat_deg = truth(idx_slice, 3); % 真值 Lat (度)
truth_lon_deg = truth(idx_slice, 4); % 真值 Lon (度)

% 提取估计值 (弧度)
est_lat_rad = pos_est_rad(:, 1);
est_lon_rad = pos_est_rad(:, 2);

% 计算时间轴 (假设 rangedata 第1列是时间)
time_axis = rangedata(1:len_data, 1);

% --- 核心误差公式 ---
% 北向误差 (m)
err_N = (d2r(truth_lat_deg) - est_lat_rad) * glv.Re;
% 东向误差 (m) - 必须乘以 cos(Lat)
err_E = (d2r(truth_lon_deg) - est_lon_rad) * glv.Re .* cos(d2r(truth_lat_deg));
% 水平定位误差 (m)
err_Pos = sqrt(err_E.^2 + err_N.^2);

% 统计 RMS
rms_N = rms(err_N);
rms_E = rms(err_E);
rms_Pos = rms(err_Pos);

%% --- 3. 绘图结果分析 ---
myfigurestartup(12, 8, 'prese'); % 调整窗口大小以容纳子图

% === 子图 1: 轨迹对比 (Lat/Lon) ===
subplot(2, 2, 1);
plot(truth_lon_deg, truth_lat_deg, 'k-', 'LineWidth', 1.5, 'DisplayName', '真值'); 
hold on; grid on;
% 将估计值转为度进行绘图
plot(r2d(est_lon_rad), r2d(est_lat_rad), 'r--', 'LineWidth', 1.2, 'DisplayName', '双水听器定位结果');
legend('Location', 'best');
xlabel('经度 (deg)'); ylabel('纬度 (deg)');
title('定位轨迹对比');
axis equal; % 保持比例，防止地图变形

% === 子图 2: 水平位置总误差 (Scalar Distance Error) ===
subplot(2, 2, 2);
plot(time_axis, err_Pos, 'm-', 'LineWidth', 1);
grid on;
xlabel('时间 (s)'); ylabel('误差 (m)');
title(['水平定位总误差 (RMS: ' sprintf('%.2f', rms_Pos) 'm)']);
xlim([time_axis(1), time_axis(end)]);

% === 子图 3: 东向误差 ===
subplot(2, 2, 3);
plot(time_axis, err_E, 'b-');
grid on;
xlabel('时间 (s)'); ylabel('东向误差 (m)');
title(['东向误差 (RMS: ' sprintf('%.2f', rms_E) 'm)']);
xlim([time_axis(1), time_axis(end)]);

% === 子图 4: 北向误差 ===
subplot(2, 2, 4);
plot(time_axis, err_N, 'b-');
grid on;
xlabel('时间 (s)'); ylabel('北向误差 (m)');
title(['北向误差 (RMS: ' sprintf('%.2f', rms_N) 'm)']);
xlim([time_axis(1), time_axis(end)]);

% % 联动 x 轴缩放 (方便查看同一时间段的误差)
% linkaxes(findall(gcf, 'type', 'axes'), 'x'); 
% % 注意：子图1是经纬度，不应联动x轴，上面这行可能会影响子图1，建议改用:
% linkaxes([subplot(2,2,2), subplot(2,2,3), subplot(2,2,4)], 'x');

%% --- 4. 角度对比图 ---
myfigurestartup(12, 4, 'prese'); 

% 提取惯导航向 (降采样) 和 观测方位角
heading_ds = navforward(idx_slice, end); 
azi_meas = rangedata(1:len_data, 7);
time_ds = rangedata(1:len_data, 1);

plot(time_ds, heading_ds, 'k.', 'LineWidth', 1.5, 'DisplayName', '惯导航向角 (Heading)');
hold on; 
plot(time_ds, azi_meas, 'r.', 'MarkerSize', 8, 'DisplayName', '观测方位角 (Azimuth)');
grid on;
legend('Location', 'best');
xlabel('时间 (s)'); ylabel('角度 (deg)');
title('航向角 vs 相对方位角');
xlim([time_ds(1), time_ds(end)]);