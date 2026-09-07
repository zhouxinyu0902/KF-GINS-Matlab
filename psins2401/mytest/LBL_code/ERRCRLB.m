% MATLAB Code to plot CRLB space distribution for Long Baseline Navigation
% 计算CRLB下界
clear;          % 清除工作区所有变量
% close all;      % 关闭所有图形窗口
clc;            % 清空命令行窗口

%% 1. 定义系统参数
% 应答器 (AP) 位置 (x, y, z) - 假设使用三个应答器形成一个三角形布局
% 为了获得与示例图相似的图案，应答器应分散布置
% 让我们将它们对称地放置在原点周围。
% ap_positions = [
%     5000,     0, 100;    % AP1
%     2500, 2500*sqrt(3),  100;    % AP2 (大约位于半径1000m，角度210度的位置)
%     0, 0,  100     % AP3 (大约位于半径1000m，角度330度的位置)
% ];
d=5000;
ap_positions = [
    d,     0, 100;   
    0, d,  100;
    d, d,  100; 
    0, 0,  100;
    d/2,d/2,100
];
num_aps = size(ap_positions, 1); % 获取应答器的数量

% 测量噪声标准差
sigma_r = 0.4; % 米 (例如，1米的测距误差)

% 测距值的协方差矩阵 (C)
% 假设每个测距值的噪声是独立且同方差的
C = eye(num_aps) * sigma_r^2; % C = sigma_r^2 * I (单位矩阵)

%% 2. 定义潜水器位置的网格 (x0, y0)
x_min = -2500; x_max = 8000; % X轴范围
y_min = -2500; y_max = 8000; % Y轴范围
grid_resolution = 50; % 米 (分辨率越小，网格越细，计算时间越长)

x_grid = x_min : grid_resolution : x_max; % X轴网格点
y_grid = y_min : grid_resolution : y_max; % Y轴网格点

[X, Y] = meshgrid(x_grid, y_grid); % 生成XY网格点坐标
Z_sub = 0; % 假设潜水器始终位于Z=0平面，用于绘制2D图

% 初始化矩阵，用于存储CRLB值
CRLB_rms_error = zeros(size(X)); % 存储位置估计的均方根误差 (RMSE)

%% 3. 计算每个网格点的CRLB
fprintf('正在网格上计算CRLB... 这可能需要一些时间。\n');
for i = 1:size(X, 1) % 遍历Y轴网格点
    for j = 1:size(X, 2) % 遍历X轴网格点
        % 当前潜水器估计位置 (x0, y0, z0)
        x0 = X(i, j);
        y0 = Y(i, j);
        sub_pos_est = [x0, y0, Z_sub];

        % 初始化设计矩阵 M
        M = zeros(num_aps, 3); % N x 3 矩阵，用于 (x, y, z) 估计

        % 计算当前潜水器位置对应的 M 矩阵
        for k = 1:num_aps
            ap_k = ap_positions(k, :); % 第 k 个应答器的位置

            % 预测距离 r0k (潜水器估计位置到AP_k的距离)
            r0k = norm(sub_pos_est - ap_k);

            % 避免当潜水器估计位置恰好在某个应答器上时出现除以零的情况
            if r0k == 0
                M(k, :) = [inf, inf, inf]; % 标记为无穷大不确定性
            else
                % 偏导数 (M的元素)
                m_i1 = (sub_pos_est(1) - ap_k(1)) / r0k; % d(r_k)/dx
                m_i2 = (sub_pos_est(2) - ap_k(2)) / r0k; % d(r_k)/dy
                m_i3 = (sub_pos_est(3) - ap_k(3)) / r0k; % d(r_k)/dz (如果 Z_sub 和 ap_k(3) 都为0且恒定，则此项可能为0)

                M(k, :) = [m_i1, m_i2, m_i3];
            end
        end

        % 计算费舍尔信息矩阵 (FIM)
        FIM = M' * (C \ M); % C \ M 等价于 inv(C) * M，更稳定高效

        % 计算 CRLB
        % 处理潜在的奇异性 (例如，如果 FIM 不可逆)
        if det(FIM) == 0 || any(isinf(M(:))) || any(isnan(M(:)))
            CRLB_mat = inf(3,3); % 标记为无穷大不确定性
        else
            CRLB_mat = inv(FIM); % 计算CRLB矩阵
        end
        
        % 存储位置的均方根误差 (RMSE) (来自 CRLB)
        % 这是 sqrt(trace(CRLB_mat))，即误差协方差矩阵对角线元素之和的平方根
        CRLB_rms_error(i, j) = sqrt(trace(CRLB_mat));
    end
end
fprintf('CRLB计算完成。\n');

%% 4. 绘制结果
figure; % 创建新的图形窗口
% 使用 contourf 绘制填充的等高线，'LineStyle', 'none' 表示不显示等高线边框
% 等高线级别设定为 5 到 25，步长 2.5，与示例图相似
[C_levels, h] = contourf(X, Y, CRLB_rms_error, 0:2.5:25, 'LineStyle', 'none'); 

% 添加等高线以便更好地观察边界
hold on; % 保持当前图形，以便添加更多绘图元素
contour(X, Y, CRLB_rms_error, 5:2.5:25, 'k'); % 绘制黑色等高线

% 绘制应答器位置
plot(ap_positions(:,1), ap_positions(:,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Transponders');

axis equal; % 确保坐标轴比例相等，避免图形变形
colormap(jet); % 设置颜色映射，例如 'jet'
colorbar; % 显示颜色条
clim([0 25]); % 设置颜色条的显示范围，使其与等高线级别和示例图一致

title({'球面试交汇长基线定位导航', 'CRLB 下界空间分布 (RMSE in meters)'}); % 设置图表标题
xlabel('x (m)'); % 设置X轴标签
ylabel('y (m)'); % 设置Y轴标签
grid on; % 显示网格

% 调整字体大小以提高可读性
set(gca, 'FontSize', 12); % 设置坐标轴字体大小
set(findobj(gca, 'Type', 'Title'), 'FontSize', 14); % 设置标题字体大小
set(findobj(gca, 'Type', 'Colorbar'), 'FontSize', 12); % 设置颜色条字体大小

% 如果需要，可以添加文本标签来标识应答器
% text(ap_positions(1,1), ap_positions(1,2)+100, 'AP1', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
% text(ap_positions(2,1)-100, ap_positions(2,2)-100, 'AP2', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top');
% text(ap_positions(3,1)+100, ap_positions(3,2)-100, 'AP3', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');

fprintf('图表已成功生成。\n');