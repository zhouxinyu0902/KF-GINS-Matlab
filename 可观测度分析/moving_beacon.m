% 清空环境
clear; clc; 
% close all;

%% 1. 参数设置
v_auv = 2;          % AUV速度 (m/s)
v_beacon_max = 4;   % 移动信标最大速度 (m/s)
T = 30 * 60;        % 任务时长 (s)
dt = 10;            % 采样间隔 (s)，为了减少计算量
t = 0:dt:T;         % 时间向量
N = length(t);      % 采样点数

% 起点 (度)
lat0_deg = 36;
lon0_deg = 118;

% 转换为弧度
lat0 = deg2rad(lat0_deg);
lon0 = deg2rad(lon0_deg);

Re = 6371000; % 地球半径 (m)

% 距离测量噪声标准差 (m)
sigma_r = 5.0;

% 移动信标活动区域: 以起点为圆心，半径4000m的圆
R_region = 4000;

%% 2. 生成AUV轨迹
distance = v_auv * t; % 行驶距离
x_auv = distance;     % 向东运动
y_auv = zeros(size(x_auv)); % 向北运动为0，直线向东
% 改为曲线或折线轨迹
theta = linspace(0, 4*pi, N);
x_auv = 2000 * sin(theta);
y_auv = 1000 * cos(2*theta);  % 更复杂的运动模式
P_auv = [ y_auv',x_auv']; % 轨迹点坐标矩阵

%% 3. 定义移动信标轨迹参数化方式
% 使用B样条曲线参数化移动信标轨迹
num_control_points = 8; % 控制点数量
control_points_initial = initialize_control_points(num_control_points, R_region);

%% 4. 定义目标函数

% 确保control_points在传入前正确reshape
objective_function = @(control_points_vec) -calculate_log_det_FIM_trajectory(...
    generate_beacon_trajectory(control_points_vec, t, P_auv), P_auv, sigma_r);
%% 5. 设置优化约束
% 变量上下界
lb = -R_region * ones(num_control_points * 2, 1);
ub = R_region * ones(num_control_points * 2, 1);

% 非线性约束：活动区域约束和速度约束
nonlcon = @(control_points_vec) moving_beacon_constraints(...
    control_points_vec, t, R_region, v_beacon_max, P_auv);

% 优化选项
options = optimoptions('fmincon', ...
    'Display', 'iter', ...
    'Algorithm', 'sqp', ...
    'MaxFunctionEvaluations', 50000, ...
    'MaxIterations', 1000);

% 调用 fmincon 进行优化
x0 = control_points_initial(:); % 将控制点展开为向量
[control_points_opt, fval, exitflag, output] = fmincon(...
    objective_function, x0, [], [], [], [], lb, ub, nonlcon, options);

% 生成最优轨迹
pb_trajectory_opt = generate_beacon_trajectory(...
    reshape(control_points_opt, num_control_points, 2), t, P_auv);

%% 6. 结果显示
fprintf('优化结果:\n');
fprintf('退出标志: %d\n', exitflag);
fprintf('最大 log(det(FIM)) 值: %.4f\n', -fval);

%% 7. 结果可视化
figure('Position', [100, 100, 1400, 600]);

% 子图1：轨迹对比
subplot(1,2,1);
hold on; grid on; axis equal;

% 绘制AUV轨迹
plot(P_auv(:,1), P_auv(:,2), 'b-', 'LineWidth', 3, 'DisplayName', 'AUV轨迹');
plot(P_auv(1,1), P_auv(1,2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', '起点');
plot(P_auv(end,1), P_auv(end,2), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', '终点');

% 绘制移动信标最优轨迹
plot(pb_trajectory_opt(:,1), pb_trajectory_opt(:,2), 'g-', 'LineWidth', 2, 'DisplayName', '移动信标轨迹');
plot(pb_trajectory_opt(1,1), pb_trajectory_opt(1,2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', '信标起点');
plot(pb_trajectory_opt(end,1), pb_trajectory_opt(end,2), 'gs', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', '信标终点');

% 绘制活动区域
theta_circle = linspace(0, 2*pi, 100);
x_circle = R_region * cos(theta_circle);
y_circle = R_region * sin(theta_circle);
plot(x_circle, y_circle, 'k--', 'LineWidth', 1, 'DisplayName', '活动区域边界');

xlabel('东向距离 (m)');
ylabel('北向距离 (m)');
title('AUV轨迹与移动信标最优轨迹');
legend('Location', 'best');

% 子图2：相对距离变化
subplot(1,2,2);
relative_distance = vecnorm(P_auv - pb_trajectory_opt, 2, 2);
plot(t/60, relative_distance, 'LineWidth', 2);
grid on;
xlabel('时间 (分钟)');
ylabel('相对距离 (m)');
title('AUV与移动信标相对距离变化');
%%
% 验证移动信标理论上应该优于固定信标
fixed_pb = [-1374.37659305944	-17.2446097866857]; % 之前优化的固定信标位置
mobile_pb = pb_trajectory_opt; % 移动信标轨迹

% 计算固定信标的FIM
fixed_fim = calculate_log_det_FIM(fixed_pb, P_auv, sigma_r);

% 验证：如果移动信标停留在固定信标的最优位置
static_trajectory = repmat(fixed_pb, size(P_auv, 1), 1);
static_mobile_fim = calculate_log_det_FIM_trajectory(static_trajectory, P_auv, sigma_r);

fprintf('固定信标性能: %.4f\n', fixed_fim);
fprintf('移动信标(静态)性能: %.4f\n', static_mobile_fim);
fprintf('移动信标(优化)性能: %.4f\n', -fval);
function control_points = initialize_control_points(num_points, R_region)
    % 初始化控制点，在活动区域内随机分布
    angles = linspace(0, 2*pi, num_points+1);
    angles = angles(1:end-1);
    radii = R_region * 0.7 * (0.8 + 0.4 * rand(1, num_points)); % 随机半径
    
    control_points = [radii' .* cos(angles)', radii' .* sin(angles)'];
end

function trajectory = generate_beacon_trajectory(control_points, t, P_auv)
    % 使用B样条生成平滑的移动信标轨迹
    num_points = length(t);
    
    % 确保control_points是正确形状的矩阵
    if isvector(control_points)
        % 如果是向量，reshape为矩阵
        num_control_points = length(control_points) / 2;
        control_points = reshape(control_points, num_control_points, 2);
    end
    
    % 确保控制点数量与时间点匹配
    t_control = linspace(t(1), t(end), size(control_points, 1));
    
    % 样条插值 - 添加边界检查
    if size(control_points, 1) < 2
        error('控制点数量至少需要2个才能进行样条插值');
    end
    
    % 安全的样条插值
    try
        trajectory_x = spline(t_control, control_points(:,1)', t);
        trajectory_y = spline(t_control, control_points(:,2)', t);
    catch ME
        % 如果样条插值失败，使用线性插值作为备选
        warning('样条插值失败，使用线性插值替代');
        trajectory_x = interp1(t_control, control_points(:,1), t, 'linear');
        trajectory_y = interp1(t_control, control_points(:,2), t, 'linear');
    end
    
    trajectory = [trajectory_x', trajectory_y'];
end

function [c, ceq] = moving_beacon_constraints(control_points_vec, t, R_region, v_max, P_auv)
    % 非线性约束函数
    num_control_points = length(control_points_vec) / 2;
    control_points = reshape(control_points_vec, num_control_points, 2);
    
    % 生成轨迹
    trajectory = generate_beacon_trajectory(control_points, t, P_auv);
    
    % 不等式约束 c <= 0
    c = [];
    
    % 1. 活动区域约束
    distances = vecnorm(trajectory, 2, 2);
    c = [c; distances - R_region];
    
    % 2. 速度约束
    if size(trajectory, 1) > 1
        velocities = vecnorm(diff(trajectory) / (t(2)-t(1)), 2, 2);
        c = [c; velocities - v_max];
    end
    
    % 等式约束 ceq = 0
    ceq = [];
end