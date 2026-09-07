function [f_trajectory, f_errors] = plotTrajectoriesComparison_Three(avp_ref, avp_ins_full, avp_pureins_full)
% plotTrajectoriesComparison_Three: 绘制并比较三条轨迹的2D平面投影，
%                                  并计算绘制它们相对于参考轨迹的位置误差。
%
%   输入:
%     avp_ref      : 参考AVP数据。最后一列必须是时间序列(秒)。
%                    其他列格式应为: [..., Lat(deg), Lon(deg), Alt(m), Time(s)]
%                    请确保第N-3, N-2, N-1 列是纬度、经度、高度，N列是时间。
%     avp_ins_full : 完整惯导解算得到的AVP数据。格式同 avp_ref。
%     avp_pureins_full : 纯惯导解算得到的AVP数据。格式同 avp_ref。
%
%   输出:
%     f_trajectory : 轨迹对比图的句柄。
%     f_errors     : 位置误差图的句柄。
%
%   示例用法:
%     % 假设 avp_ref, avp_ins_full, avp_pureins_full 已经定义，且最后一列是时间
%     % 例如:
%     % ref_avp = generate_ref_avp(); % 假设此函数返回带时间列的AVP
%     % ins_avp = generate_ins_avp();
%     % pureins_avp = generate_pureins_avp();
%     % [traj_fig, err_fig] = plotTrajectoriesComparison_Three(ref_avp, ins_avp, pureins_avp);

% --- 输入参数检查 ---
if nargin < 3
    error('函数需要三个输入参数: avp_ref, avp_ins_full 和 avp_pureins_full。');
end

% 假设 AVPs 的最后一列是时间，倒数第四、三、二列是姿态，倒数三、二、一列是经纬高
% 格式: [roll, pitch, yaw, vx, vy, vz, Lat(deg), Lon(deg), Alt(m), Time(s)]
% 假设 Lat, Lon, Alt 总是最后三列，Time 是最后一列
if size(avp_ref, 2) < 4 || size(avp_ins_full, 2) < 4 || size(avp_pureins_full, 2) < 4
    error('输入AVP数据至少需要包含经纬高和时间信息 (至少4列)。');
end

% 提取时间和位置列
time_ref = avp_ref(:, end);
pos_ref_rad = avp_ref(:, end-3:end-1); % 假设 Lat, Lon, Alt 是倒数第4到第2列

time_ins = avp_ins_full(:, end);
pos_ins_rad = avp_ins_full(:, end-3:end-1);

time_pureins = avp_pureins_full(:, end);
pos_pureins_rad = avp_pureins_full(:, end-3:end-1);

% --- 时间轴统一化及数据插值 ---
% 确定所有轨迹的共同时间范围
min_time = max([time_ref(1), time_ins(1), time_pureins(1)]);
max_time = min([time_ref(end), time_ins(end), time_pureins(end)]);

% 创建统一的时间向量，使用参考轨迹的采样间隔作为插值步长
% 假设 ts_ref 是参考轨迹的平均采样间隔
if length(time_ref) > 1
    ts_ref = mean(diff(time_ref));
else
    ts_ref = 1; % 默认值，如果只有一个点
end

unified_time_s = (min_time : ts_ref : max_time)';

% 对所有轨迹的位置数据进行插值，使其对齐到统一时间轴
% 假设 avp 中的经纬高是度数 (deg)
% 注意：这里插值的是经纬度，而不是XYZ，因为XYZ是基于经纬度的。
ref_pos_interp_rad = interp1(time_ref, pos_ref_rad, unified_time_s, 'linear', 'extrap');
ins_pos_interp_rad = interp1(time_ins, pos_ins_rad, unified_time_s, 'linear', 'extrap');
pureins_pos_interp_rad = interp1(time_pureins, pos_pureins_rad, unified_time_s, 'linear', 'extrap');


% --- 确定本地坐标系原点 ---
% 以参考轨迹插值后的第一个点作为本地坐标系的原点 (pos0)。
% pos0 是一个行向量 [Lat, Lon, Alt]，单位为度
pos0_rad = ref_pos_interp_rad(1, :);

% --- 将轨迹经纬高转换为本地XYZ坐标 (东-北-天) ---
% pos2dxyz 函数通常期望输入位置是 [N x 3] 矩阵 (度数)，参考点是列向量 (弧度)。
% 假设 pos2dxyz 输出单位是米 (m)
ref_xyz_m = pos2dxyz(ref_pos_interp_rad, pos0_rad');
ins_xyz_m = pos2dxyz(ins_pos_interp_rad, pos0_rad');
pureins_xyz_m = pos2dxyz(pureins_pos_interp_rad, pos0_rad');

% --- 绘图部分 1: 轨迹对比图 (本地XYZ坐标) ---
f_trajectory = figure; % 创建一个新的图窗
set(f_trajectory, 'Name', '三轨迹对比'); % 设置图窗名称

% 绘制参考轨迹的XY平面投影
plot(ref_xyz_m(:, 1), ref_xyz_m(:, 2), 'b-', 'LineWidth', 1.5, 'DisplayName', '参考轨迹'); % 蓝色实线
hold on; % 保持当前图

% 绘制 INS 轨迹的XY平面投影
plot(ins_xyz_m(:, 1), ins_xyz_m(:, 2), 'g-', 'LineWidth', 1.5, 'DisplayName', '距离约束轨迹'); % 绿色虚线

% 绘制纯惯导轨迹的XY平面投影
plot(pureins_xyz_m(:, 1), pureins_xyz_m(:, 2), 'r:', 'LineWidth', 1.5, 'DisplayName', '纯惯导轨迹'); % 红色点线

% 增加图表元素
xlabel('东向 (m)');
ylabel('北向 (m)');
title('轨迹对比 (本地坐标系)');
legend('show', 'Location', 'best');
grid on;
axis equal; % 确保X轴和Y轴的比例相等
hold off;

% --- 误差计算 ---
% 误差 = 某轨迹 - 参考轨迹 (单位：米)
error_ins_xyz = ins_xyz_m - ref_xyz_m;
error_pureins_xyz = pureins_xyz_m - ref_xyz_m;

% 计算径向误差 (水平面上的总误差)
radial_error_ins = sqrt(error_ins_xyz(:, 1).^2 + error_ins_xyz(:, 2).^2);
radial_error_pureins = sqrt(error_pureins_xyz(:, 1).^2 + error_pureins_xyz(:, 2).^2);

% --- 绘图部分 2: 位置误差图 ---
f_errors = figure; % 创建一个新的图窗用于误差曲线
set(f_errors, 'Name', '位置误差曲线'); % 设置图窗名称

% 绘制东向误差
subplot(3, 1, 1); % 3行1列的子图中的第1个
plot(unified_time_s, error_ins_xyz(:, 1), 'g-', 'LineWidth', 1, 'DisplayName', '距离约束 东向误差');
hold on;
plot(unified_time_s, error_pureins_xyz(:, 1), 'r-', 'LineWidth', 1, 'DisplayName', '纯惯导 东向误差');
hold off;
ylabel('东向误差 (m)');
title('各类惯导与参考轨迹位置误差');
grid on;
legend('show', 'Location', 'best');
set(gca, 'Xticklabel', []); % 隐藏X轴刻度标签

% 绘制北向误差
subplot(3, 1, 2); % 3行1列的子图中的第2个
plot(unified_time_s, error_ins_xyz(:, 2), 'g-', 'LineWidth', 1, 'DisplayName', '距离约束 北向误差');
hold on;
plot(unified_time_s, error_pureins_xyz(:, 2), 'r-', 'LineWidth', 1, 'DisplayName', '纯惯导 北向误差');
hold off;
ylabel('北向误差 (m)');
grid on;
legend('show', 'Location', 'best');
set(gca, 'Xticklabel', []); % 隐藏X轴刻度标签

% 绘制径向误差
subplot(3, 1, 3); % 3行1列的子图中的第3个
plot(unified_time_s, radial_error_ins, 'g-', 'LineWidth', 1, 'DisplayName', '距离约束 径向误差');
hold on;
plot(unified_time_s, radial_error_pureins, 'r-', 'LineWidth', 1, 'DisplayName', '纯惯导 径向误差');
hold off;
xlabel('时间 (秒)');
ylabel('径向误差 (m)');
grid on;
legend('show', 'Location', 'best');

end