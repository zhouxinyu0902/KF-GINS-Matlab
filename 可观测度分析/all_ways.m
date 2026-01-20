clc; clear; close all;

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

%% 3. 定义目标函数 J(p_b)
objective_function = @(pb) -calculate_log_det_FIM(pb, P_auv, sigma_r);

%% 4. 单信标优化
fprintf('=== 开始单信标优化 ===\n');
A = []; b = []; Aeq = []; beq = [];
lb = [-R_region, -R_region];
ub = [R_region, R_region];
nonlcon = @circle_constraint;

options = optimoptions('fmincon', ...
    'Display', 'iter', ...
    'Algorithm', 'sqp', ...
    'MaxFunctionEvaluations', 10000);

x0 = [10, 10];
[pb_opt, fval, exitflag, output] = fmincon(objective_function, x0, A, b, Aeq, beq, lb, ub, nonlcon, options);

single_beacon = pb_opt;
log_det_single = -fval;

fprintf('单信标优化完成: log(det(FIM)) = %.4f\n', log_det_single);

%% 5. 双信标优化
fprintf('\n=== 开始双信标优化 ===\n');
single_beacon=[600,800];
objective_function2 = @(pb2) -calculate_log_det_FIM_two_beacons(single_beacon, pb2, P_auv, sigma_r);
x0 = [-50, -50]; % 在区域内随机初始化
[pb2_opt, fval2, exitflag2, output2] = fmincon(objective_function2, x0, A, b, Aeq, beq, lb, ub, nonlcon, options);

double_beacons = [single_beacon; pb2_opt];
log_det_double = -fval2;

fprintf('双信标优化完成: log(det(FIM)) = %.4f\n', log_det_double);

%% 6. 三信标优化
fprintf('\n=== 开始三信标优化 ===\n');
single_beacon=[600,800];

objective_function3 = @(pb3) -calculate_log_det_FIM_three_beacons(...
    single_beacon, pb2_opt, pb3, P_auv, sigma_r);

% 多起点优化策略
x0_options = [1000, 0; -1000, 0; 0, 1000; 0, -1000; 800, 800; -800, -800];
best_x0 = x0_options(1, :);
best_value = -inf;

fprintf('测试不同初始点...\n');
for i = 1:size(x0_options, 1)
    test_x0 = x0_options(i, :);
    if norm(test_x0) <= R_region
        test_value = calculate_log_det_FIM_three_beacons(...
            single_beacon, pb2_opt, test_x0, P_auv, sigma_r);
        fprintf('初始点 [%.0f, %.0f]: log(det(FIM)) = %.4f\n', ...
            test_x0(1), test_x0(2), test_value);
        if test_value > best_value && ~isinf(test_value)
            best_value = test_value;
            best_x0 = test_x0;
        end
    end
end

% x0 = best_x0;
x0 = [20,20];
fprintf('选择最佳初始点: [%.1f, %.1f]\n', x0(1), x0(2));

[pb3_opt, fval3, exitflag3, output3] = fmincon(objective_function3, x0, A, b, Aeq, beq, lb, ub, nonlcon, options);

triple_beacons = [single_beacon; pb2_opt; pb3_opt];
log_det_triple = -fval3;

fprintf('三信标优化完成: log(det(FIM)) = %.4f\n', log_det_triple);

%% 7. 性能对比分析
performance_improvement_double = exp(log_det_double - log_det_single);
performance_improvement_triple = exp(log_det_triple - log_det_single);
improvement_triple_vs_double = exp(log_det_triple - log_det_double);

%% 8. 综合可视化


% 子图1：信标布局和AUV轨迹
myfigurestartup(12,4,'paper')


% 设置采样间隔，避免观测线过于密集
sample_interval = max(1, round(size(P_auv, 1) / 20)); % 采样约20个点

% 创建三个子图
subplot(1,3,1);
hold on; grid on; axis equal;
title('单信标系统布局与观测几何', 'FontSize', 10, 'FontWeight', 'bold');

% 绘制部署区域
theta = linspace(0, 2*pi, 100);
x_circle = R_region * cos(theta);
y_circle = R_region * sin(theta);
plot(x_circle, y_circle, 'k--', 'LineWidth', 0.5, 'DisplayName', '部署区域');

% 绘制AUV轨迹
plot(P_auv(:,1), P_auv(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'AUV轨迹');
plot(0, 0, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'DisplayName', '起点');
plot(P_auv(end,1), P_auv(end,2), 'rs', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'DisplayName', '终点');

% 绘制单信标位置
plot(pb_opt(1), pb_opt(2), 'ms', 'MarkerSize', 10, 'MarkerFaceColor', 'm', 'DisplayName', '信标');

% 添加观测几何线（从信标到AUV轨迹）
for i = 1:sample_interval:size(P_auv, 1)
    plot([pb_opt(1), P_auv(i,1)], [pb_opt(2), P_auv(i,2)], ...
        'm:', 'LineWidth', 0.5, 'Color', [0.7 0.2 0.7 0.3], 'HandleVisibility', 'off');
end

xlabel('东向距离 (m)'); ylabel('北向距离 (m)');
legend('Location', 'best');
axis([-4000 4000 -4000 4000])
subplot(1,3,2);
hold on; grid on; axis equal;
title('双信标系统布局与观测几何', 'FontSize', 10, 'FontWeight', 'bold');

% 绘制部署区域
plot(x_circle, y_circle, 'k--', 'LineWidth', 0.5, 'DisplayName', '部署区域');

% 绘制AUV轨迹
plot(P_auv(:,1), P_auv(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'AUV轨迹');
plot(0, 0, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'DisplayName', '起点');
plot(P_auv(end,1), P_auv(end,2), 'rs', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'DisplayName', '终点');

% 绘制双信标位置
plot(double_beacons(:,1), double_beacons(:,2), 'g^', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', '信标');
plot([double_beacons(1,1), double_beacons(2,1)], [double_beacons(1,2), double_beacons(2,2)], ...
    'g--', 'LineWidth', 1, 'DisplayName', '信标连线');

% 添加观测几何线（从两个信标到AUV轨迹）
for i = 1:sample_interval:size(P_auv, 1)
    % 从第一个信标到AUV
    plot([double_beacons(1,1), P_auv(i,1)], [double_beacons(1,2), P_auv(i,2)], ...
        'g:', 'LineWidth', 0.5, 'Color', [0.2 0.7 0.2 0.3], 'HandleVisibility', 'off');
    % 从第二个信标到AUV
    plot([double_beacons(2,1), P_auv(i,1)], [double_beacons(2,2), P_auv(i,2)], ...
        'g:', 'LineWidth', 0.5, 'Color', [0.2 0.7 0.2 0.3], 'HandleVisibility', 'off');
end

xlabel('东向距离 (m)'); 
legend('Location', 'best');
axis([-4000 4000 -4000 4000])
subplot(1,3,3);
hold on; grid on; axis equal;
title('三信标系统布局与观测几何', 'FontSize', 10, 'FontWeight', 'bold');

% 绘制部署区域
plot(x_circle, y_circle, 'k--', 'LineWidth', 0.5, 'DisplayName', '部署区域');

% 绘制AUV轨迹
plot(P_auv(:,1), P_auv(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'AUV轨迹');
plot(0, 0, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'DisplayName', '起点');
plot(P_auv(end,1), P_auv(end,2), 'rs', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'DisplayName', '终点');

% 绘制三信标位置
plot(triple_beacons(:,1), triple_beacons(:,2), 'bd', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', '信标');

% 连接信标形成三角形
if size(triple_beacons, 1) == 3
    plot([triple_beacons(:,1); triple_beacons(1,1)], ...
         [triple_beacons(:,2); triple_beacons(1,2)], 'b--', 'LineWidth', 1, 'DisplayName', '信标三角形');
end

% 添加观测几何线（从三个信标到AUV轨迹）
for i = 1:sample_interval:size(P_auv, 1)
    for j = 1:size(triple_beacons, 1)
        plot([triple_beacons(j,1), P_auv(i,1)], [triple_beacons(j,2), P_auv(i,2)], ...
            'b:', 'LineWidth', 0.5, 'Color', [0.2 0.2 0.7 0.3], 'HandleVisibility', 'off');
    end
end

xlabel('东向距离 (m)'); 
legend('Location', 'best');
axis([-4000 4000 -4000 4000])
% 添加整体标题
sgtitle('不同信标系统布局与观测几何对比', 'FontSize', 12, 'FontWeight', 'bold');

% 添加性能对比文本
% annotation('textbox', [0.02, 0.02, 0.96, 0.05], 'String', ...
%     sprintf('性能对比: 单信标=%.4f | 双信标=%.4f (提升%.2fx) | 三信标=%.4f (提升%.2fx)', ...
%     log_det_single, log_det_double, exp(log_det_double - log_det_single), ...
%     log_det_triple, exp(log_det_triple - log_det_single)), ...
%     'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
%     'FontSize', 9, 'EdgeColor', 'none', 'BackgroundColor', [0.95 0.95 0.95]);
%%
% 创建三种情况的目标函数等值线图
myfigurestartup(12,4,'paper')

% 情况1：单信标系统目标函数等值线
subplot(1,3,1);
hold on; grid on;


% 生成网格数据
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


% 标记最优解
plot(pb_opt(1), pb_opt(2), 'm*', 'MarkerSize', 15, 'LineWidth', 2,'DisplayName','最优解');
plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% 绘制部署区域边界
plot(x_circle, y_circle, 'k--', 'LineWidth', 1.5);

% 情况2：双信标系统目标函数等值线
subplot(1,3,2);
hold on; grid on;

% 在部署区域内采样，计算目标函数值
[x_grid, y_grid] = meshgrid(linspace(-R_region, R_region, 50));
J_grid = zeros(size(x_grid));

for i = 1:size(x_grid, 1)
    for j = 1:size(x_grid, 2)
        pb2_test = [x_grid(i,j), y_grid(i,j)];
        if norm(pb2_test) <= R_region % 只在区域内计算
            J_grid(i,j) = -calculate_log_det_FIM_two_beacons(single_beacon, pb2_test, P_auv, sigma_r);
        else
            J_grid(i,j) = NaN;
        end
    end
end
% 计算双信标系统的目标函数值（固定第一个信标，优化第二个）
% 绘制目标函数等值线图
contourf(x_grid, y_grid, -J_grid, 20);
colorbar;
xlabel('东向距离 (m)');
ylabel('北向距离 (m)');


% 标记固定信标1和最优信标2
plot(single_beacon(1), single_beacon(2), 'g.', 'MarkerSize', 10, 'LineWidth', 2);
plot(pb2_opt(1), pb2_opt(2), 'm*', 'MarkerSize', 8, 'LineWidth', 2);

% 绘制部署区域边界
plot(x_circle, y_circle, 'k--', 'LineWidth', 1.5);
% 情况3：三信标系统目标函数等值线
subplot(1,3,3);
hold on; grid on;
[x_grid, y_grid] = meshgrid(linspace(-R_region, R_region, 40));
J_grid = zeros(size(x_grid));

for i = 1:size(x_grid, 1)
    for j = 1:size(x_grid, 2)
        pb3_test = [x_grid(i,j), y_grid(i,j)];
        if norm(pb3_test) <= R_region
            J_grid(i,j) = -calculate_log_det_FIM_three_beacons(...
                single_beacon, pb2_opt, pb3_test, P_auv, sigma_r);
        else
            J_grid(i,j) = NaN;
        end
    end
end

contourf(x_grid, y_grid, -J_grid, 30);
colorbar;
plot(single_beacon(1), single_beacon(2), 'g.', 'MarkerSize', 10, 'LineWidth', 3);
plot(pb2_opt(1), pb2_opt(2), 'b.', 'MarkerSize', 10, 'LineWidth', 3);
plot(pb3_opt(1), pb3_opt(2), 'm*', 'MarkerSize', 15, 'LineWidth', 2);
plot(x_circle, y_circle, 'k--', 'LineWidth', 1.5);

xlabel('东向距离 (m)'); ylabel('北向距离 (m)');


% 添加整体标题
sgtitle('不同信标系统目标函数等值线对比', 'FontSize', 12, 'FontWeight', 'bold');

% 添加图例说明
% annotation('textbox', [0.02, 0.02, 0.96, 0.05], 'String', ...
%     '图例说明: *-信标1, ○-信标2, ◇-信标3 | 等值线表示log(det(FIM))值', ...
%     'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
%     'FontSize', 8, 'EdgeColor', 'none', 'BackgroundColor', [0.95 0.95 0.95]);
%% 子图2：性能对比柱状图
myfigurestartup(7,7,'prese')
performance_data = [log_det_single, log_det_double, log_det_triple];
improvement_data = [1, performance_improvement_double, performance_improvement_triple];

bar(performance_data);
set(gca, 'XTickLabel', {'单信标', '双信标', '三信标'});
ylabel('log(det(FIM))');
title('观测性能对比', 'FontSize', 12, 'FontWeight', 'bold');
grid on;

% 在柱子上添加数值标签
for i = 1:length(performance_data)
    text(i, performance_data(i) + 0.1, sprintf('%.3f', performance_data(i)), ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end

% 子图3：性能提升倍数
myfigurestartup(7,7,'prese')
bar(improvement_data);
set(gca, 'XTickLabel', {'单信标', '双信标', '三信标'});
ylabel('性能提升倍数');
title('相对性能提升', 'FontSize', 12, 'FontWeight', 'bold');
grid on;

for i = 1:length(improvement_data)
    text(i, improvement_data(i) + 0.1, sprintf('%.2fx', improvement_data(i)), ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold');
end

% 子图4：观测几何示意图
myfigurestartup(7,7,'prese')
hold on; grid on; axis equal;
title('最优观测几何示意图', 'FontSize', 12, 'FontWeight', 'bold');

% 绘制AUV轨迹
plot(P_auv(:,1), P_auv(:,2), 'b-', 'LineWidth', 2);

% 绘制三信标系统
plot(triple_beacons(:,1), triple_beacons(:,2), 'rd', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% 绘制从信标到轨迹的观测线（采样显示，避免太密集）
sample_interval = round(N/20); % 采样20个点
for i = 1:sample_interval:N
    for j = 1:size(triple_beacons, 1)
        plot([triple_beacons(j,1), P_auv(i,1)], [triple_beacons(j,2), P_auv(i,2)], ...
            'g:', 'LineWidth', 0.5, 'Color', [0.5 0.5 0.5 0.3]);
    end
end

xlabel('东向距离 (m)'); ylabel('北向距离 (m)');

% % 子图5：收敛曲线对比
% subplot(2,3,5);
% hold on; grid on;
% title('优化收敛过程', 'FontSize', 12, 'FontWeight', 'bold');
% 
% % 绘制收敛曲线（如果优化输出中包含迭代历史）
% if isfield(output, 'fval') && length(output.fval) > 1
%     plot(-output.fval, 'r-', 'LineWidth', 2, 'DisplayName', '单信标');
% end
% if isfield(output2, 'fval') && length(output2.fval) > 1
%     plot(-output2.fval, 'g-', 'LineWidth', 2, 'DisplayName', '双信标');
% end
% if isfield(output3, 'fval') && length(output3.fval) > 1
%     plot(-output3.fval, 'b-', 'LineWidth', 2, 'DisplayName', '三信标');
% end
% 
% xlabel('迭代次数'); ylabel('log(det(FIM))');
% legend('Location', 'southeast');
% 
% % 子图6：详细结果汇总
% subplot(2,3,6);
% axis off; % 关闭坐标轴，用于文本显示

% 创建详细结果文本
result_text = sprintf(['优化结果汇总\n\n', ...
    '单信标系统:\n', ...
    '  位置: [%.1f, %.1f] m\n', ...
    '  距离起点: %.1f m\n', ...
    '  log(det(FIM)): %.4f\n\n', ...
    '双信标系统:\n', ...
    '  信标1: [%.1f, %.1f] m\n', ...
    '  信标2: [%.1f, %.1f] m\n', ...
    '  信标间距: %.1f m\n', ...
    '  log(det(FIM)): %.4f\n', ...
    '  性能提升: %.2f倍\n\n', ...
    '三信标系统:\n', ...
    '  信标3: [%.1f, %.1f] m\n', ...
    '  log(det(FIM)): %.4f\n', ...
    '  性能提升: %.2f倍\n', ...
    '  相对双信标: %.2f倍'], ...
    single_beacon(1), single_beacon(2), norm(single_beacon), ...
    log_det_single, ...
    double_beacons(1,1), double_beacons(1,2), ...
    double_beacons(2,1), double_beacons(2,2), ...
    norm(double_beacons(1,:) - double_beacons(2,:)), ...
    log_det_double, performance_improvement_double, ...
    triple_beacons(3,1), triple_beacons(3,2), ...
    log_det_triple, performance_improvement_triple, improvement_triple_vs_double);

% text(0.05, 0.95, result_text, 'FontSize', 10, 'VerticalAlignment', 'top', ...
%     'FontName', 'Monospaced', 'Interpreter', 'none');
fprintf('\n=== 最终优化结果汇总 ===\n');
fprintf(result_text);
%% 9. 输出详细结果
fprintf('\n=== 最终优化结果汇总 ===\n');
fprintf('单信标系统:   log(det(FIM)) = %.4f\n', log_det_single);
fprintf('双信标系统:   log(det(FIM)) = %.4f (提升%.2f倍)\n', log_det_double, performance_improvement_double);
fprintf('三信标系统:   log(det(FIM)) = %.4f (提升%.2f倍)\n', log_det_triple, performance_improvement_triple);
fprintf('三信标 vs 双信标: 提升%.2f倍\n', improvement_triple_vs_double);

%% 10. 保存结果
save('beacon_optimization_results.mat', 'single_beacon', 'double_beacons', 'triple_beacons', ...
    'log_det_single', 'log_det_double', 'log_det_triple', 'P_auv', 'R_region');

fprintf('\n结果已保存到 beacon_optimization_results.mat\n');
%% 8. 综合可视化


% 子图1：信标布局和AUV轨迹
myfigurestartup(4,4,'paper')


% 设置采样间隔，避免观测线过于密集
sample_interval = max(1, round(size(P_auv, 1) / 20)); % 采样约20个点

% 创建三个子图
% subplot(1,3,1);

hold on; grid on; axis equal;
title('单信标系统布局与观测几何', 'FontSize', 10, 'FontWeight', 'bold');

% 绘制部署区域
theta = linspace(0, 2*pi, 100);
x_circle = R_region * cos(theta);
y_circle = R_region * sin(theta);
plot(x_circle, y_circle, 'k--', 'LineWidth', 0.5, 'DisplayName', '部署区域');

% 绘制AUV轨迹
plot(P_auv(:,1), P_auv(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'AUV轨迹');
plot(0, 0, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'DisplayName', '起点');
plot(P_auv(end,1), P_auv(end,2), 'rs', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'DisplayName', '终点');

% 绘制单信标位置
plot(pb_opt(1), pb_opt(2), 'ms', 'MarkerSize', 10, 'MarkerFaceColor', 'm', 'DisplayName', '信标');

% 添加观测几何线（从信标到AUV轨迹）
for i = 1:sample_interval:size(P_auv, 1)
    plot([pb_opt(1), P_auv(i,1)], [pb_opt(2), P_auv(i,2)], ...
        'm:', 'LineWidth', 0.5, 'Color', [0.7 0.2 0.7 0.3], 'HandleVisibility', 'off');
end

xlabel('东向距离 (m)'); ylabel('北向距离 (m)');
legend('Location', 'best');
axis([-4000 4000 -4000 4000])
% subplot(1,3,2);
figure
hold on; grid on; axis equal;
title('双信标系统布局与观测几何', 'FontSize', 10, 'FontWeight', 'bold');

% 绘制部署区域
plot(x_circle, y_circle, 'k--', 'LineWidth', 0.5, 'DisplayName', '部署区域');

% 绘制AUV轨迹
plot(P_auv(:,1), P_auv(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'AUV轨迹');
plot(0, 0, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'DisplayName', '起点');
plot(P_auv(end,1), P_auv(end,2), 'rs', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'DisplayName', '终点');

% 绘制双信标位置
plot(double_beacons(:,1), double_beacons(:,2), 'g^', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', '信标');
plot([double_beacons(1,1), double_beacons(2,1)], [double_beacons(1,2), double_beacons(2,2)], ...
    'g--', 'LineWidth', 1, 'DisplayName', '信标连线');

% 添加观测几何线（从两个信标到AUV轨迹）
for i = 1:sample_interval:size(P_auv, 1)
    % 从第一个信标到AUV
    plot([double_beacons(1,1), P_auv(i,1)], [double_beacons(1,2), P_auv(i,2)], ...
        'g:', 'LineWidth', 0.5, 'Color', [0.2 0.7 0.2 0.3], 'HandleVisibility', 'off');
    % 从第二个信标到AUV
    plot([double_beacons(2,1), P_auv(i,1)], [double_beacons(2,2), P_auv(i,2)], ...
        'g:', 'LineWidth', 0.5, 'Color', [0.2 0.7 0.2 0.3], 'HandleVisibility', 'off');
end

xlabel('东向距离 (m)'); 
legend('Location', 'best');
axis([-4000 4000 -4000 4000])
% subplot(1,3,3);
figure
hold on; grid on; axis equal;
title('三信标系统布局与观测几何', 'FontSize', 10, 'FontWeight', 'bold');

% 绘制部署区域
plot(x_circle, y_circle, 'k--', 'LineWidth', 0.5, 'DisplayName', '部署区域');

% 绘制AUV轨迹
plot(P_auv(:,1), P_auv(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'AUV轨迹');
plot(0, 0, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'DisplayName', '起点');
plot(P_auv(end,1), P_auv(end,2), 'rs', 'MarkerSize', 6, 'MarkerFaceColor', 'r', 'DisplayName', '终点');

% 绘制三信标位置
plot(triple_beacons(:,1), triple_beacons(:,2), 'bd', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', '信标');

% 连接信标形成三角形
if size(triple_beacons, 1) == 3
    plot([triple_beacons(:,1); triple_beacons(1,1)], ...
         [triple_beacons(:,2); triple_beacons(1,2)], 'b--', 'LineWidth', 1, 'DisplayName', '信标三角形');
end

% 添加观测几何线（从三个信标到AUV轨迹）
for i = 1:sample_interval:size(P_auv, 1)
    for j = 1:size(triple_beacons, 1)
        plot([triple_beacons(j,1), P_auv(i,1)], [triple_beacons(j,2), P_auv(i,2)], ...
            'b:', 'LineWidth', 0.5, 'Color', [0.2 0.2 0.7 0.3], 'HandleVisibility', 'off');
    end
end

xlabel('东向距离 (m)'); 
legend('Location', 'best');
axis([-4000 4000 -4000 4000])
%% 创建三种情况的目标函数等值线图
myfigurestartup(4,4,'paper')

% 情况1：单信标系统目标函数等值线
% subplot(1,3,1);
hold on; grid on;


% 生成网格数据
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


% 标记最优解
plot(pb_opt(1), pb_opt(2), 'm*', 'MarkerSize', 15, 'LineWidth', 2,'DisplayName','最优解');
plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% 绘制部署区域边界
plot(x_circle, y_circle, 'k--', 'LineWidth', 1.5);

% 情况2：双信标系统目标函数等值线
% subplot(1,3,2);
figure
hold on; grid on;

% 在部署区域内采样，计算目标函数值
[x_grid, y_grid] = meshgrid(linspace(-R_region, R_region, 50));
J_grid = zeros(size(x_grid));

for i = 1:size(x_grid, 1)
    for j = 1:size(x_grid, 2)
        pb2_test = [x_grid(i,j), y_grid(i,j)];
        if norm(pb2_test) <= R_region % 只在区域内计算
            J_grid(i,j) = -calculate_log_det_FIM_two_beacons(single_beacon, pb2_test, P_auv, sigma_r);
        else
            J_grid(i,j) = NaN;
        end
    end
end
% 计算双信标系统的目标函数值（固定第一个信标，优化第二个）
% 绘制目标函数等值线图
contourf(x_grid, y_grid, -J_grid, 20);
colorbar;
xlabel('东向距离 (m)');
ylabel('北向距离 (m)');


% 标记固定信标1和最优信标2
plot(single_beacon(1), single_beacon(2), 'g.', 'MarkerSize', 10, 'LineWidth', 2);
plot(pb2_opt(1), pb2_opt(2), 'm*', 'MarkerSize', 8, 'LineWidth', 2);

% 绘制部署区域边界
plot(x_circle, y_circle, 'k--', 'LineWidth', 1.5);
% 情况3：三信标系统目标函数等值线
% subplot(1,3,3);
figure
hold on; grid on;
[x_grid, y_grid] = meshgrid(linspace(-R_region, R_region, 40));
J_grid = zeros(size(x_grid));

for i = 1:size(x_grid, 1)
    for j = 1:size(x_grid, 2)
        pb3_test = [x_grid(i,j), y_grid(i,j)];
        if norm(pb3_test) <= R_region
            J_grid(i,j) = -calculate_log_det_FIM_three_beacons(...
                single_beacon, pb2_opt, pb3_test, P_auv, sigma_r);
        else
            J_grid(i,j) = NaN;
        end
    end
end

contourf(x_grid, y_grid, -J_grid, 30);
colorbar;
plot(single_beacon(1), single_beacon(2), 'g.', 'MarkerSize', 10, 'LineWidth', 3);
plot(pb2_opt(1), pb2_opt(2), 'b.', 'MarkerSize', 10, 'LineWidth', 3);
plot(pb3_opt(1), pb3_opt(2), 'm*', 'MarkerSize', 15, 'LineWidth', 2);
plot(x_circle, y_circle, 'k--', 'LineWidth', 1.5);

xlabel('东向距离 (m)'); ylabel('北向距离 (m)');
%% 辅助函数（确保这些函数在路径中）
function [c, ceq] = circle_constraint(pb)
    c = pb(1)^2 + pb(2)^2 - 4000^2;
    ceq = [];
end
