% 清空环境
clear; 
clc; 
% close all;

%% 1. 参数设置
v = 2;          % AUV速度 (m/s)
T = 30 * 60;     % 任务时长 (s)
dt = 1;          % 采样间隔 (s)
t = 0:dt:T;      % 时间向量
N = length(t);   % 采样点数

% 起点
lat0_deg = 36;
lon0_deg = 118;
lat0 = deg2rad(lat0_deg);
lon0 = deg2rad(lon0_deg);

Re = 6371000; % 地球半径 (m)
sigma_r = 5.0; % 距离测量噪声标准差 (m)
R_region = 4000; % 信标部署区域半径

%% 2. 生成AUV轨迹
distance = v * t; % 行驶距离
x_auv = distance;  % 向东运动
y_auv = zeros(size(x_auv)); % 直线向东
P_auv = [y_auv', x_auv'];

%% 3. 定义固定信标位置（两个固定信标）
% 固定第一个信标在起点位置
fixed_beacon1 = [600, 800];
% 固定第二个信标在起点正北方向
fixed_beacon2 = [-1188.96, 1107.22];

%  x = 600.00 m, y = 800.00 m
% x = -1188.96 m, y = 1107.22 m
%% 4. 定义三个信标的目标函数
objective_function = @(pb3) -calculate_log_det_FIM_three_beacons(...
    fixed_beacon1, fixed_beacon2, pb3, P_auv, sigma_r);

%% 5. 设置约束并求解优化问题
% 尝试不同的初始点
x0_options = [1000, 0; -1000, 0; 0, 1000; 0, -1000; 800, 800; -800, -800];
best_x0 = x0_options(1, :);
best_value = -inf;

fprintf('测试不同初始点...\n');
for i = 1:size(x0_options, 1)
    test_x0 = x0_options(i, :);
    if norm(test_x0) <= R_region % 确保在部署区域内
        test_value = calculate_log_det_FIM_three_beacons(...
            fixed_beacon1, fixed_beacon2, test_x0, P_auv, sigma_r);
        fprintf('初始点 [%.0f, %.0f]: log(det(FIM)) = %.4f\n', ...
            test_x0(1), test_x0(2), test_value);
        if test_value > best_value && ~isinf(test_value)
            best_value = test_value;
            best_x0 = test_x0;
        end
    end
end

x0 = best_x0;
fprintf('选择最佳初始点: [%.1f, %.1f]\n', x0(1), x0(2));

% 优化选项
options = optimoptions('fmincon', ...
    'Display', 'iter', ...
    'Algorithm', 'interior-point', ...
    'MaxFunctionEvaluations', 10000, ...
    'FunctionTolerance', 1e-6);

% 约束设置
A = []; b = []; Aeq = []; beq = [];
lb = [-R_region, -R_region];
ub = [R_region, R_region];
nonlcon = @circle_constraint;

% 优化第三个信标位置
[pb3_opt, fval, exitflag, output] = fmincon(objective_function, x0, A, b, Aeq, beq, lb, ub, nonlcon, options);

%% 6. 输出结果
fprintf('\n=== 三信标优化结果 ===\n');
fprintf('退出标志: %d\n', exitflag);
fprintf('固定信标1位置: [%.1f, %.1f] m\n', fixed_beacon1);
fprintf('固定信标2位置: [%.1f, %.1f] m\n', fixed_beacon2);
fprintf('最优信标3位置: [%.1f, %.1f] m\n', pb3_opt);
fprintf('信标3到起点距离: %.1f m\n', norm(pb3_opt));
fprintf('最大 log(det(FIM)): %.4f\n', -fval);

% 计算信标间距离
d12 = norm(fixed_beacon2 - fixed_beacon1);
d13 = norm(pb3_opt - fixed_beacon1);
d23 = norm(pb3_opt - fixed_beacon2);
fprintf('信标间距: 1-2: %.1f m, 1-3: %.1f m, 2-3: %.1f m\n', d12, d13, d23);

%% 7. 性能对比分析
log_det_single = calculate_log_det_FIM(fixed_beacon2, P_auv, sigma_r);
log_det_double = calculate_log_det_FIM_two_beacons(fixed_beacon1, fixed_beacon2, P_auv, sigma_r);
log_det_triple = calculate_log_det_FIM_three_beacons(fixed_beacon1, fixed_beacon2, pb3_opt, P_auv, sigma_r);

fprintf('\n=== 性能对比 ===\n');
fprintf('单信标系统:   log(det(FIM)) = %.4f\n', log_det_single);
fprintf('双信标系统:   log(det(FIM)) = %.4f (提升%.2f倍)\n', log_det_double, exp(log_det_double - log_det_single));
fprintf('三信标系统:   log(det(FIM)) = %.4f (提升%.2f倍)\n', log_det_triple, exp(log_det_triple - log_det_single));
fprintf('三信标 vs 双信标: 提升%.2f倍\n', exp(log_det_triple - log_det_double));

%% 8. 结果可视化
figure('Position', [100, 100, 1400, 600]);

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
plot(x_circle, y_circle, 'k--', 'LineWidth', 1, 'DisplayName', '部署区域');

% 绘制三个信标
plot(fixed_beacon1(1), fixed_beacon1(2), 'gs', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', '固定信标1');
plot(fixed_beacon2(1), fixed_beacon2(2), 'bs', 'MarkerSize', 12, 'MarkerFaceColor', 'b', 'DisplayName', '固定信标2');
plot(pb3_opt(1), pb3_opt(2), 'ms', 'MarkerSize', 12, 'MarkerFaceColor', 'm', 'DisplayName', '最优信标3');

% 绘制信标连线
plot([fixed_beacon1(1), fixed_beacon2(1)], [fixed_beacon1(2), fixed_beacon2(2)], 'g-', 'LineWidth', 1.5);
plot([fixed_beacon1(1), pb3_opt(1)], [fixed_beacon1(2), pb3_opt(2)], 'm-', 'LineWidth', 1.5);
plot([fixed_beacon2(1), pb3_opt(1)], [fixed_beacon2(2), pb3_opt(2)], 'b-', 'LineWidth', 1.5);

xlabel('东向距离 (m)'); ylabel('北向距离 (m)');
title('三信标布局与AUV轨迹');
legend('Location', 'best');

% 子图2：目标函数等值线
subplot(1,2,2);
hold on; grid on;

% 采样计算目标函数
[x_grid, y_grid] = meshgrid(linspace(-R_region, R_region, 40));
J_grid = zeros(size(x_grid));

for i = 1:size(x_grid, 1)
    for j = 1:size(x_grid, 2)
        pb3_test = [x_grid(i,j), y_grid(i,j)];
        if norm(pb3_test) <= R_region
            J_grid(i,j) = -calculate_log_det_FIM_three_beacons(...
                fixed_beacon1, fixed_beacon2, pb3_test, P_auv, sigma_r);
        else
            J_grid(i,j) = NaN;
        end
    end
end

contourf(x_grid, y_grid, -J_grid, 30);
colorbar;
plot(fixed_beacon1(1), fixed_beacon1(2), 'go', 'MarkerSize', 10, 'LineWidth', 3);
plot(fixed_beacon2(1), fixed_beacon2(2), 'bo', 'MarkerSize', 10, 'LineWidth', 3);
plot(pb3_opt(1), pb3_opt(2), 'm*', 'MarkerSize', 15, 'LineWidth', 2);
plot(x_circle, y_circle, 'k--', 'LineWidth', 1.5);

xlabel('东向距离 (m)'); ylabel('北向距离 (m)');
title('目标函数等值线图');

% % 子图3：性能对比柱状图
% subplot(1,3,3);
% performance = [log_det_single, log_det_double, log_det_triple];
% improvement = exp(performance - performance(1)); % 相对于单信标的提升倍数
% 
% bar(improvement, 'FaceColor', [0.2 0.6 0.8]);
% set(gca, 'XTickLabel', {'单信标', '双信标', '三信标'});
% ylabel('性能提升倍数');
% title('不同信标数量的性能对比');
% grid on;
% 
% for i = 1:3
%     text(i, improvement(i)+0.1, sprintf('%.1fx', improvement(i)), ...
%         'HorizontalAlignment', 'center', 'FontWeight', 'bold');
% end