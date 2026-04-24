clear
pva_830=importdata('惯导实验数据/input/pva_830.txt');
pva_430=importdata('惯导实验数据/input/pva_430.txt');
std_430=importdata('惯导实验数据/input/std_430.txt');
std_830=importdata('惯导实验数据/input/std_830.txt');
pva_RS=importdata('惯导实验数据/input/pva_RS.txt');

pva_truth = importdata('惯导实验数据/output/truth.nav');
%% 误差对比
ax1 = myfigurestartup(4,5,'paper');

% 纬度误差对比
subplot(3,1,1);
plot(std_830 (:,1), std_830 (:,2), 'r.'); hold on;
plot(std_430 (:,1), std_430 (:,2), 'b.');
ylabel('纬度误差 (m)');
title('各维度定位误差对比');
legend('pva\_830', 'pva\_430');
ylim([0 0.08])
grid on;

% 经度误差对比
subplot(3,1,2);
plot(std_830 (:,1), std_830 (:,3), 'r.'); hold on;
plot(std_430 (:,1), std_430 (:,3), 'b.');
ylim([0 0.08])
ylabel('经度误差 (m)');
grid on;

% 高度误差对比
subplot(3,1,3);
plot(std_830 (:,1), std_830 (:,4), 'r.'); hold on;
plot(std_430 (:,1), std_430 (:,4), 'b.');
ylim([0 0.08])
xlabel('时间 (s)');
ylabel('高度误差 (m)');
grid on;


% 假设 pva_830/430 的第 6, 7, 8 列分别是 纬度、经度、高度的误差 (单位: m)
% 时间列为第 2 列

% 定义数据矩阵
data_830 = std_830(:, 2:4); 
data_430 = std_430(:, 2:4);

% 计算统计指标
rmse_830 = sqrt(mean(data_830.^2));
rmse_430 = sqrt(mean(data_430.^2));

std830 = std(data_830);
std430 = std(data_430);

max_err_830 = max(abs(data_830));
max_err_430 = max(abs(data_430));

% 打印量化结果表格
fprintf('\n================ 导航误差量化对比 (单位: m) ================\n');
fprintf('%-10s | %-15s | %-15s | %-15s\n', '指标', '纬度 (Lat)', '经度 (Lon)', '高度 (Alt)');
fprintf('------------------------------------------------------------\n');
fprintf('830 RMSE   | %-15.4f | %-15.4f | %-15.4f\n', rmse_830);
fprintf('430 RMSE   | %-15.4f | %-15.4f | %-15.4f\n', rmse_430);
fprintf('------------------------------------------------------------\n');
fprintf('830 MAX    | %-15.4f | %-15.4f | %-15.4f\n', max_err_830);
fprintf('430 MAX    | %-15.4f | %-15.4f | %-15.4f\n', max_err_430);
fprintf('============================================================\n');

% exportgraphics(ax1, fullfile('D:\GitHub\KF-GINS-Matlab\fig\', ...
%     'Preview_Figure_1.png'), 'Resolution', 600);
%%
myfigurestartup(5,5,'paper');
plot(pva_830(:,3))
myfigurestartup(7,5,'paper');
subplot 121
plot(diff(pva_830(:,5)),'.')
subplot 122
plot(pva_830(:,8))
%% 生成高度数据
height = pva_truth(:,[2,5]);
% 数据保存：高度数据
output_file="D:\Github\KF-GINS-Matlab\惯导实验数据\input\height.txt";
try
    writematrix(height, output_file, 'Delimiter', ' ');
    fprintf('height信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
%% 构造静止信标数据
% 构造GNSS数据：直接选择gps数据/选择参考数据
GNSS_1s = pva_truth(1:100:end,2:5);
glvs
pos0 = d2r([36.40042003, 120.68981831, 0]);
% 原始等边三角形坐标（单位：米）
dxyz_original = [0, -5*sqrt(3), 0;
                -10, 5*sqrt(3), 0;
                -20, -5*sqrt(3), 0;] * 1000;
% 定义旋转角度（15度）
theta_deg = 15;
theta_rad = deg2rad(theta_deg);
% 创建绕Z轴的旋转矩阵
R = [cos(theta_rad), -sin(theta_rad), 0;
     sin(theta_rad), cos(theta_rad),  0;
     0,              0,              1];
% 对每个点应用旋转
beacon_xyz = (R * dxyz_original')'; % 转置以便矩阵乘法
% 分开获取信标和轨迹原点
beacon_rrm = dxyz2pos(beacon_xyz, pos0');
beacon_ddm = [r2d(beacon_rrm(:, 1:2)),beacon_rrm(:, 3)];

trj = GNSS_1s(:, 2:4);
trj(:,1:2) = d2r(trj(:,1:2));
trajectory_xyz = pos2dxyz(trj, pos0');
trajectory_ddm = GNSS_1s(:, 2:4);
% 绘图
plot_navigation_scene(trajectory_xyz, 'static', beacon_xyz, 'type', 'xyz')
exportgraphics(gca, fullfile('D:\GitHub\KF-GINS-Matlab\fig\', ...
    'plot_navigation_scene.png'), 'Resolution', 600);
plot_navigation_scene(trajectory_ddm(:,[2,1,3]), 'static', beacon_ddm(:,[2,1,3]), 'type', 'lla')
%%
trajectory_x = trajectory_xyz(:, 1);
trajectory_y = trajectory_xyz(:, 2);
myfigurestartup(12,5,'prese');
for i=1:3
    % 获取信标的坐标 (东向，北向，天向)
    beacon1_x = beacon_xyz(i, 1);
    beacon1_y = beacon_xyz(i, 2);
    % 计算每个轨迹点到第一个信标的2D距离
    distances(:,i) = sqrt((trajectory_x - beacon1_x).^2 + ...
        (trajectory_y - beacon1_y).^2);
    % 创建新的图窗来绘制距离曲线
    subplot(1,3,i)
    plot(GNSS_1s(:,1), distances(:,i), 'LineWidth', 1.5); % 洋红色实线
    xlabel('时间 (s) ');
    ylabel('距离 (km)');
    grid on;
end
distances_N_by_1_max = max(distances, [], 2);
beacon1 = ones(length(distances),3)*diag(beacon_rrm(1,:));
beacon2 = ones(length(distances),3)*diag(beacon_rrm(2,:));
beacon3 = ones(length(distances),3)*diag(beacon_rrm(3,:));
range1 = [GNSS_1s(:,1),distances(:,1),distances(:,1),beacon1];
range2 = [GNSS_1s(:,1),distances(:,2),distances(:,2),beacon2];
range3 = [GNSS_1s(:,1),distances(:,3),distances(:,3),beacon3];
%%
output_file="D:\Github\KF-GINS-Matlab\惯导实验数据\input\range1.txt";
try
    writematrix(range1, output_file, 'Delimiter', ' ');
    fprintf('height信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
output_file="D:\Github\KF-GINS-Matlab\惯导实验数据\input\range2.txt";
try
    writematrix(range2, output_file, 'Delimiter', ' ');
    fprintf('height信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
output_file="D:\Github\KF-GINS-Matlab\惯导实验数据\input\range3.txt";
try
    writematrix(range3, output_file, 'Delimiter', ' ');
    fprintf('height信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end