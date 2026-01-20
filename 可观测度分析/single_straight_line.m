% 清空环境
% clear; clc; close all;

%% 1. 参数设置
v = 2;          % AUV速度 (m/s)
T = 30 * 60;     % 任务时长 (s)
dt = 1;          % 采样间隔 (s)，假设1秒1次
t = 0:dt:T;      % 时间向量
N = length(t);   % 采样点数

% 起点 (度)
lat0_deg = 36;
lon0_deg = 118;

% 转换为弧度
lat0 = deg2rad(lat0_deg);
lon0 = deg2rad(lon0_deg);

Re = 6371000; % 地球半径 (m)

% 距离测量噪声标准差 (m)
sigma_r = 5.0;

% 信标部署区域: 以起点为圆心，半径2000m的圆
R_region = 2000;

%% 2. 生成AUV轨迹 (在平面坐标系下，原点为起点)
% 假设在起点切平面上，正北为y轴，正东为x轴
distance = v * t; % 行驶距离
x_auv = distance;  % 向东运动，x坐标变化
y_auv = zeros(size(x_auv)); % 向北运动为0，直线向东

% % 改为曲线或折线轨迹
% theta = linspace(0, 4*pi, N);
% x_auv = 2000 * sin(theta);
% y_auv = 1000 * cos(2*theta);  % 更复杂的运动模式

P_auv = [y_auv',x_auv'];
% 轨迹点坐标矩阵: 每一行是一个点 [x, y]
% P_auv = [y_auv',x_auv' ;
%     distance',x_auv(end)*ones(size(x_auv))'];

%%
%% 3. 定义目标函数 J(p_b)

% 注意：优化变量是信标的直角坐标 [x_b, y_b]
objective_function = @(pb) -calculate_log_det_FIM(pb, P_auv, sigma_r);

%% 4. 设置约束并求解优化问题
% 决策变量初值 (在区域内随机初始化或选择起点)


% 线性约束 (无，用非线性约束表示圆形区域)
A = [];
b = [];
Aeq = [];
beq = [];
R_region=4000;
% 变量上下界 (提供一个宽松的边界)
lb = [-R_region, -R_region];
ub = [R_region, R_region];

% 非线性约束：信标必须在半径R_region的圆内
nonlcon = @circle_constraint;

% 优化选项
options = optimoptions('fmincon', ...
    'Display', 'iter', ... % 显示迭代过程
    'Algorithm', 'sqp', ... % 序列二次规划算法
    'MaxFunctionEvaluations', 10000);

% 调用 fmincon 进行优化
% 求解：在圆形区域内最小化 -log(det(FIM))，即最大化 log(det(FIM))
x0 = [10, 10]; % 以起点作为初始猜测
[pb_opt, fval, exitflag, output] = fmincon(objective_function, x0, A, b, Aeq, beq, lb, ub, nonlcon, options);

obs_results = calculate_instantaneous_observability(pb_opt(1:2), P_auv(:,1:2), 5);
%%
fprintf('优化结果:\n');
fprintf('退出标志: %d\n', exitflag);
fprintf('最优信标位置 (相对于起点): x = %.2f m, y = %.2f m\n', pb_opt(1), pb_opt(2));
fprintf('最优信标到起点的距离: %.2f m\n', norm(pb_opt));
fprintf('最大 log(det(FIM)) 值: %.4f\n', -fval); % 注意取负号
% 
fixed_fim = calculate_log_det_FIM(pb_opt, P_auv, sigma_r)
%% 5. 结果可视化
figure('Position', [100, 100, 1200, 500]);
% 
% 子图1：轨迹与信标布局
subplot(1,2,1);
hold on; grid on; axis equal;

% 绘制AUV轨迹
plot(P_auv(:,1), P_auv(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'AUV轨迹');
plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', '起点');

% 绘制信标部署区域
theta_circle = linspace(0, 2*pi, 100);
x_circle = R_region * cos(theta_circle);
y_circle = R_region * sin(theta_circle);
plot(x_circle, y_circle, 'k--', 'LineWidth', 1, 'DisplayName', '部署区域边界');

% 绘制最优信标位置
% plot(pb_opt(1), pb_opt(2), 'ms', 'MarkerSize', 10, 'MarkerFaceColor', 'm', 'DisplayName', '最优信标');

% 绘制从最优信标到轨迹起止点的连线，示意几何关系
% plot([pb_opt(1), P_auv(1,1)], [pb_opt(2), P_auv(1,2)], 'g:');
% plot([pb_opt(1), P_auv(end,1)], [pb_opt(2), P_auv(end,2)], 'g:');

xlabel('东向距离 (m)');
ylabel('北向距离 (m)');
% title('AUV轨迹与最优信标布局');
legend('Location', 'best');

% 子图2：目标函数空间（可选，计算量大但很直观）
subplot(1,2,2);
hold on; grid on;

% 在部署区域内采样，计算目标函数值
[x_grid, y_grid] = meshgrid(linspace(-R_region, R_region, 50));
J_grid = zeros(size(x_grid));

for i = 1:size(x_grid, 1)
    for j = 1:size(x_grid, 2)
        pb_test = [x_grid(i,j), y_grid(i,j)];
        if norm(pb_test) <= R_region % 只在区域内计算
            J_grid(i,j) = -calculate_log_det_FIM(pb_test, P_auv, sigma_r);
        else
            J_grid(i,j) = NaN;
        end
    end
end

% 绘制目标函数等值线图
contourf(x_grid, y_grid, -J_grid, 20); % 注意取负号，使最大值对应最亮区域
colorbar;
xlabel('东向距离 (m)');
ylabel('北向距离 (m)');
title('目标函数 log(det(FIM)) 等值线图');

% 标记最优解
plot(pb_opt(1), pb_opt(2), 'm*', 'MarkerSize', 15, 'LineWidth', 2);
plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% 绘制部署区域边界
plot(x_circle, y_circle, 'k--', 'LineWidth', 1.5);