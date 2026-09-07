function [optimized_xy, history] = steepest_descent_for_localization(initial_xy, target_z, sensor_positions, measured_ranges, learning_rate, max_iterations, tolerance)
% steepest_descent_for_localization: 为定位问题实现最速下降法
%
% 输入:
%   initial_xy: 初始猜测的目标XY坐标 [x; y]
%   target_z: 目标已知的Z坐标
%   sensor_positions: N x 3 传感器位置矩阵
%   measured_ranges: N x 1 测量距离向量
%   learning_rate: 学习率（步长），需要仔细调整
%   max_iterations: 最大迭代次数
%   tolerance: 收敛容差 (基于参数变化范数)
%
% 输出:
%   optimized_xy: 优化后的XY坐标
%   history: 包含每次迭代的参数和代价函数值

current_xy = initial_xy;

% 存储历史数据
history.xy_history = {};
history.cost_history = [];

for iter = 1:max_iterations
    [current_cost, grad_S] = range_residuals_and_gradient(current_xy, target_z, sensor_positions, measured_ranges);

    history.xy_history{end+1} = current_xy;
    history.cost_history(end+1) = current_cost;

    % 计算参数更新步长
    delta_xy = -learning_rate * grad_S;

    % 检查收敛条件
    if norm(delta_xy) < tolerance
        % fprintf('最速下降法在迭代 %d 处收敛：参数变化范数 %.4e < 容差 %.4e\n', iter, norm(delta_xy), tolerance);
        break;
    end

    % 更新参数
    current_xy = current_xy + delta_xy;

    % 检查发散
    if any(isnan(current_xy)) || any(isinf(current_xy))
        warning('最速下降法: 参数发散 (NaN或Inf)。请尝试更小的学习率或更好的初始值。');
        break;
    end
end
optimized_xy = current_xy;

% 如果未在循环中提前退出，可能是达到最大迭代次数
if iter == max_iterations && norm(delta_xy) >= tolerance
    % fprintf('最速下降法达到最大迭代次数 %d，未达到收敛容差。\n', max_iterations);
end

end
function [S, grad_S_xy] = range_residuals_and_gradient(xy_current, target_z, sensor_positions, measured_ranges)
% range_residuals_and_gradient: 计算当前XY位置下，测距残差的平方和及其梯度。
% 适用于最速下降法。
%
% 输入:
%   xy_current: 当前估计的目标XY坐标 [x; y]
%   target_z: 目标已知的Z坐标
%   sensor_positions: N x 3 传感器位置矩阵
%   measured_ranges: N x 1 测量距离向量
%
% 输出:
%   S: 测距残差的平方和 (scalar)
%   grad_S_xy: 残差平方和对XY的梯度向量 [dS/dx; dS/dy]

x = xy_current(1);
y = xy_current(2);
N = size(sensor_positions, 1);

% 构建当前目标位置
current_target_pos = [x, y, target_z];

% 计算理论距离
predicted_ranges = zeros(N, 1);
for k = 1:N
    predicted_ranges(k) = norm(current_target_pos - sensor_positions(k,:));
end

% 计算残差
residuals = measured_ranges - predicted_ranges;

% 残差平方和
S = sum(residuals.^2);

% 计算梯度 (dS/dx, dS/dy)
% S = sum((ri_measured - ri_predicted)^2)
% dS/dx = sum(2 * (ri_measured - ri_predicted) * d(ri_predicted)/dx)
% d(ri_predicted)/dx = (x - sensor_x_k) / ri_predicted
% 同样适用于 dS/dy

grad_x = 0;
grad_y = 0;

for k = 1:N
    sensor_x = sensor_positions(k,1);
    sensor_y = sensor_positions(k,2);
    
    % ri_predicted = sqrt((x - sensor_x)^2 + (y - sensor_y)^2 + (target_z - sensor_z_k)^2)
    % 这里我们已经计算了 predicted_ranges(k)，所以可以直接用
    
    % d(ri_predicted)/dx
    d_ri_dx = (x - sensor_x) / predicted_ranges(k);
    
    % d(ri_predicted)/dy
    d_ri_dy = (y - sensor_y) / predicted_ranges(k);
    
    grad_x = grad_x + 2 * residuals(k) * (-d_ri_dx); % 注意这里的负号来自 d(残差)/d(predicted_range) = -1
    grad_y = grad_y + 2 * residuals(k) * (-d_ri_dy);
end

grad_S_xy = [grad_x; grad_y];

end