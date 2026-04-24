function stations = generate_equilateral_stations(L, center)
% GENERATE_EQUILATERAL_STATIONS 生成等边三角形布局的潜标坐标
% 输入:
%   L: 边长 (例如 1000 米)
%   center: 1x2 向量，中心点坐标 [x0, y0] (默认 [0,0])
% 输出:
%   stations: 3x2 矩阵，每行是一个潜标的 [x, y] 坐标

if nargin < 2
    center = [0, 0];
end

% 计算外接圆半径
R = L / sqrt(3);

% 定义三个顶点的相对坐标 (旋转 90, 210, 330 度)
rel_pos = [
    0,          R;                  % 顶点
    -L/2,       -R/2;               % 左下
    L/2,        -R/2                % 右下
    ];

% 加上中心点偏移
stations = rel_pos + center;

% --- 可视化验证 ---
figure;
plot([stations(:,1); stations(1,1)], [stations(:,2); stations(1,2)], '-o', 'LineWidth', 2);
grid on; axis equal;
xlabel('X 坐标 (m)'); ylabel('Y 坐标 (m)');
title(sprintf('等边三角形潜标布局 (边长: %.0fm)', L));
for i = 1:3
    text(stations(i,1), stations(i,2), sprintf('  基站 %d', i), 'FontSize', 12);
end
end