% 清空环境
clear; 
clc; 
% close all;

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
R_region = 4000;

%% 2. 生成AUV轨迹 (在平面坐标系下，原点为起点)
distance = v * t; % 行驶距离
x_auv = distance;  % 向东运动，x坐标变化
y_auv = zeros(size(x_auv)); % 向北运动为0，直线向东
P_auv = [y_auv',x_auv'];

%% 3. 定义固定信标位置
% 固定第一个信标在起点位置
fixed_beacon = [600, 800];

%% 4. 定义两个信标的目标函数
objective_function = @(pb2) -calculate_log_det_FIM_two_beacons(fixed_beacon, pb2, P_auv, sigma_r);

%% 5. 设置约束并求解优化问题
% 决策变量初值 (第二个信标的初始位置)
x0 = [-50, -50]; % 在区域内随机初始化

% 线性约束
A = [];
b = [];
Aeq = [];
beq = [];

% 变量上下界
lb = [-R_region, -R_region];
ub = [R_region, R_region];

% 非线性约束：第二个信标必须在半径R_region的圆内
nonlcon = @circle_constraint;

% 优化选项
options = optimoptions('fmincon', ...
    'Display', 'iter', ... % 显示迭代过程
    'Algorithm', 'sqp', ... % 序列二次规划算法
    'MaxFunctionEvaluations', 10000);

% 调用 fmincon 进行优化
[pb2_opt, fval, exitflag, output] = fmincon(objective_function, x0, A, b, Aeq, beq, lb, ub, nonlcon, options);

%% 6. 输出结果
fprintf('优化结果:\n');
fprintf('退出标志: %d\n', exitflag);
fprintf('固定信标1位置: x = %.2f m, y = %.2f m\n', fixed_beacon(1), fixed_beacon(2));
fprintf('最优信标2位置: x = %.2f m, y = %.2f m\n', pb2_opt(1), pb2_opt(2));
fprintf('信标2到起点的距离: %.2f m\n', norm(pb2_opt));
fprintf('两个信标之间的距离: %.2f m\n', norm(pb2_opt - fixed_beacon));
fprintf('最大 log(det(FIM)) 值: %.4f\n', -fval);

%% 7. 结果可视化
figure('Position', [100, 100, 1200, 500]);

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

% 绘制固定信标1
plot(fixed_beacon(1), fixed_beacon(2), 'gs', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', '固定信标1');

% 绘制最优信标2
plot(pb2_opt(1), pb2_opt(2), 'ms', 'MarkerSize', 10, 'MarkerFaceColor', 'm', 'DisplayName', '最优信标2');

% 绘制从信标到轨迹起止点的连线
plot([fixed_beacon(1), P_auv(1,1)], [fixed_beacon(2), P_auv(1,2)], 'g:');
plot([fixed_beacon(1), P_auv(end,1)], [fixed_beacon(2), P_auv(end,2)], 'g:');
plot([pb2_opt(1), P_auv(1,1)], [pb2_opt(2), P_auv(1,2)], 'm:');
plot([pb2_opt(1), P_auv(end,1)], [pb2_opt(2), P_auv(end,2)], 'm:');

xlabel('东向距离 (m)');
ylabel('北向距离 (m)');
title('AUV轨迹与双信标布局');
legend('Location', 'best');

% 子图2：目标函数空间
subplot(1,2,2);
hold on; grid on;

% 在部署区域内采样，计算目标函数值
[x_grid, y_grid] = meshgrid(linspace(-R_region, R_region, 50));
J_grid = zeros(size(x_grid));

for i = 1:size(x_grid, 1)
    for j = 1:size(x_grid, 2)
        pb2_test = [x_grid(i,j), y_grid(i,j)];
        if norm(pb2_test) <= R_region % 只在区域内计算
            J_grid(i,j) = -calculate_log_det_FIM_two_beacons(fixed_beacon, pb2_test, P_auv, sigma_r);
        else
            J_grid(i,j) = NaN;
        end
    end
end

% 绘制目标函数等值线图
contourf(x_grid, y_grid, -J_grid, 20);
colorbar;
xlabel('东向距离 (m)');
ylabel('北向距离 (m)');
title('目标函数 log(det(FIM)) 等值线图（信标2位置）');

% 标记固定信标1和最优信标2
plot(fixed_beacon(1), fixed_beacon(2), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(pb2_opt(1), pb2_opt(2), 'm*', 'MarkerSize', 15, 'LineWidth', 2);

% 绘制部署区域边界
plot(x_circle, y_circle, 'k--', 'LineWidth', 1.5);

%% 8. 性能分析
% 计算单个信标和双信标的性能对比
log_det_single = calculate_log_det_FIM(fixed_beacon, P_auv, sigma_r);
log_det_double = calculate_log_det_FIM_two_beacons(fixed_beacon, pb2_opt, P_auv, sigma_r);

fprintf('\n性能对比:\n');
fprintf('单个信标 (位置[%.1f, %.1f]) log(det(FIM)): %.4f\n', fixed_beacon(1), fixed_beacon(2), log_det_single);
fprintf('双信标系统 log(det(FIM)): %.4f\n', log_det_double);
fprintf('性能提升: %.2f倍\n', exp(log_det_double - log_det_single));


%% 5. 双信标优化（修正版）- 同时优化两个信标
fprintf('\n=== 开始双信标优化（同时优化两个信标） ===\n');

% 定义目标函数，同时优化两个信标的位置
objective_function2_joint = @(x) -calculate_log_det_FIM_two_beacons(...
    x(1:2), x(3:4), P_auv, sigma_r);

% 设置初始猜测（两个信标的初始位置）
x0_joint = [50, 50 , -50, -50]; % [信标1_x, 信标1_y, 信标2_x, 信标2_y]

% 设置联合优化的约束
lb_joint = [-R_region, -R_region, -R_region, -R_region];
ub_joint = [R_region, R_region, R_region, R_region];

% 非线性约束：两个信标都必须在部署区域内
nonlcon_joint_2beacons = @(x) circle_constraint_joint_2beacons(x, R_region);

% 多起点优化策略
x0_options_2beacons = [
    [600, 800, -50, -50];    % 当前配置
    [1000, 0, -1000, 0];     % 东西对称
    [0, 1000, 0, -1000];     % 南北对称  
    [500, 500, -500, -500];   % 对角线
    [800, 300, -800, -300];   % 不同角度
    [300, 800, -300, -800];   % 另一角度
];

best_x0_2beacons = x0_options_2beacons(1, :);
best_value_2beacons = -inf;

fprintf('测试不同初始点...\n');
for i = 1:size(x0_options_2beacons, 1)
    test_x0 = x0_options_2beacons(i, :);
    % 检查两个信标是否在部署区域内
    valid = true;
    for j = 1:2
        if norm(test_x0((j-1)*2+1:j*2)) > R_region
            valid = false;
            break;
        end
    end
    
    if valid
        test_value = calculate_log_det_FIM_two_beacons(...
            test_x0(1:2), test_x0(3:4), P_auv, sigma_r);
        fprintf('初始点配置 %d: log(det(FIM)) = %.4f\n', i, test_value);
        if test_value > best_value_2beacons && ~isinf(test_value)
            best_value_2beacons = test_value;
            best_x0_2beacons = test_x0;
        end
    end
end

fprintf('选择最佳初始配置\n');

% 执行联合优化
[x_opt_2beacons, fval2_joint, exitflag2_joint, output2_joint] = fmincon(...
    objective_function2_joint, best_x0_2beacons, [], [], [], [], ...
    lb_joint, ub_joint, nonlcon_joint_2beacons, options);


% 提取优化后的两个信标位置
pb1_opt_joint = x_opt_2beacons(1:2);
pb2_opt_joint = x_opt_2beacons(3:4);

double_beacons_joint = [pb1_opt_joint; pb2_opt_joint];
log_det_double_joint = -fval2_joint;

log_det_double = calculate_log_det_FIM_two_beacons(pb1_opt_joint, pb2_opt_joint, P_auv, sigma_r)*10000000
log_det_double2 = calculate_log_det_FIM_two_beacons([600,800], pb2_opt, P_auv, sigma_r)*10000000
fprintf('双信标联合优化完成: log(det(FIM)) = %.4f\n', log_det_double_joint);
fprintf('优化后的信标位置:\n');
fprintf('信标1: [%.1f, %.1f] m\n', pb1_opt_joint);
fprintf('信标2: [%.1f, %.1f] m\n', pb2_opt_joint);

% 比较两种方法的结果
fprintf('\n=== 双信标优化方法对比 ===\n');
fprintf('分步优化结果: %.4f\n', log_det_double);
fprintf('联合优化结果: %.4f\n', log_det_double_joint);
fprintf('性能提升: %.4f (%.2f%%)\n', ...
    log_det_double_joint - log_det_double, ...
    (log_det_double_joint - log_det_double) / log_det_double * 100);

% 使用联合优化的结果
double_beacons = double_beacons_joint;
log_det_double = log_det_double_joint;

%% 添加双信标联合约束函数
function [c, ceq] = circle_constraint_joint_2beacons(x, R_region)
    % 非线性不等式约束：两个信标都必须在部署区域内
    c = [];
    for i = 1:2
        % 每个信标的约束：到起点的距离平方 - R_region^2 <= 0
        beacon_pos = x((i-1)*2+1:i*2);
        c = [c; beacon_pos(1)^2 + beacon_pos(2)^2 - R_region^2];
    end
    % 非线性等式约束 ceq(x) = 0
    ceq = [];
end