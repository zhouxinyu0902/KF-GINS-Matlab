%% ==================== 7. 标定成果综合评估对账与准静态距离数据导出 ====================
fprintf('\n========================================================================\n');
fprintf('         7. 正在执行标定成果全要素评估对账 与 补偿数据导出流程            \n');
fprintf('========================================================================\n');

% --- 【核心输入提取】从 xk 终点状态中提取你的流场反演成果 ---
theta_est = xk(end, 10); 
phi_est   = xk(end, 11); 

% 1. 参数准备与度数转换
if ~exist('glv','var') || ~isfield(glv,'deg')
    glv.deg = pi/180;
    fprintf('注意：已自动定义 glv.deg = pi/180\n');
end

theta_true_rad = theta_true;   % 继承数据发生器的真值（20*glv.deg）
phi_true_rad   = phi_true;     % 继承真值（45*glv.deg）
theta_est_rad  = theta_est;
phi_est_rad    = phi_est;

theta_true_deg = theta_true_rad / glv.deg;
phi_true_deg   = phi_true_rad   / glv.deg;
theta_est_deg  = theta_est_rad  / glv.deg;
phi_est_deg    = phi_est_rad    / glv.deg;

theta_err_deg = theta_est_deg - theta_true_deg;
phi_err_deg   = phi_est_deg   - phi_true_deg;

% 2. 打印：角度估计质量
fprintf('\n==================== 1. 角度估计质量 ====================\n');
fprintf('真值     : theta = %.6f°, phi = %.6f°\n', theta_true_deg, phi_true_deg);
fprintf('估计值   : theta = %.6f°, phi = %.6f°\n', theta_est_deg, phi_est_deg);
fprintf('绝对误差 : Δtheta = %.6f°, Δphi = %.6f°\n', theta_err_deg, phi_err_deg);

fprintf('结论     : ');
if abs(theta_err_deg) < 0.1 && abs(phi_err_deg) < 0.5
    fprintf('角度估计精度很高，完美满足深海标定工程需求。\n');
elseif abs(theta_err_deg) < 0.5 && abs(phi_err_deg) < 2
    fprintf('角度估计精度尚可，可接受。\n');
else
    fprintf('角度估计误差偏大，建议检查滤波 P0 阵设置或观测数据时间对齐。\n');
end

% 3. 无补偿前的真实偏移分量计算
depth = -S_gnss_xyz(:,3);   % 提取已知发声深度：[1000; 1015; 985] 米
n = length(depth);

% 【物理对齐修复】：严格契合数据发生器几何投影：X对应东向(sin)，Y对应北向(cos)
delta_x_true = depth .* tan(theta_true_rad) .* sin(phi_true_rad);
delta_y_true = depth .* tan(theta_true_rad) .* cos(phi_true_rad);

delta_x_est = depth .* tan(theta_est_rad) .* sin(phi_est_rad);
delta_y_est = depth .* tan(theta_est_rad) .* cos(phi_est_rad);

fprintf('\n==================== 2. 水平偏移分量对账单（单位：米） ====================\n');
fprintf('潜标编号\t深度(m)\t真实dx(东)\t真实dy(北)\t估计dx(东)\t估计dy(北)\tdx误差\tdy误差\n');
for i = 1:n
    fprintf('%d\t\t%.0f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n', ...
        i, depth(i), ...
        delta_x_true(i), delta_y_true(i), ...
        delta_x_est(i), delta_y_est(i), ...
        delta_x_est(i)-delta_x_true(i), delta_y_est(i)-delta_y_true(i));
end

% 4. 位置还原误差与补偿坐标重构
S_est_xyz = zeros(n, 3);
for i = 1:n
    surf_x = S_gnss_xyz(i,1);
    surf_y = S_gnss_xyz(i,2);
    S_est_xyz(i,:) = [surf_x + delta_x_est(i), surf_y + delta_y_est(i), -depth(i)];
end
pos_err_3d = sqrt(sum((S_est_xyz - S_true_xyz).^2, 2));  % 三维位置还原绝对误差
pos_err_horiz = sqrt( (delta_x_est - delta_x_true).^2 + (delta_y_est - delta_y_true).^2 ); % 水平平面误差

fprintf('\n==================== 3. 潜标位置还原误差评估（米） ====================\n');
fprintf('潜标编号\t深度(m)\t三维位置误差\t水平位置误差\n');
for i = 1:n
    fprintf('%d\t\t%.0f\t%.3f\t\t%.3f\n', i, depth(i), pos_err_3d(i), pos_err_horiz(i));
end
fprintf('平均三维位置误差: %.3f m, 平均水平位置误差: %.3f m\n', mean(pos_err_3d), mean(pos_err_horiz));

% 5. 综合评语打分
fprintf('\n==================== 4. 综合评估结论 ====================\n');
fprintf('1. 角度标定误差：倾角误差 %.4f°，方位角误差 %.4f°。\n', theta_err_deg, phi_err_deg);
fprintf('2. 深度 %d m 处，最大水平偏移估计误差被压缩至 %.3f m。\n', round(max(depth)), max(pos_err_horiz));
fprintf('3. 全部潜标阵列反演平均三维点位残差为 %.3f m。\n', mean(pos_err_3d));

if mean(pos_err_3d) < 0.5
    fprintf('4. 最终评价：估计效果极其优秀，已成功逆向抹除海流摆动，可直接用于导航补偿。\n');
elseif mean(pos_err_3d) < 1.0
    fprintf('4. 最终评价：估计效果良好，满足高精度水声标定应用需求。\n');
else
    fprintf('4. 最终评价：估计效果一般，建议调优滤波器过程噪声 Qc 或检查大噪声 R 阵阻尼。\n');
end


% 6. 图形可视化展示
figure('Color', [1 1 1], 'Position', [100 100 1000 400]);
subplot(1,2,1);
bar(1:n, [delta_x_true, delta_x_est]);
xlabel('潜标编号'); ylabel('东向偏移 (m)');
legend('真实(真值模型)', '估计(10维滤波器)', 'Location','best');
title('X方向（东向）流场位移对比');
grid on;

subplot(1,2,2);
bar(1:n, [delta_y_true, delta_y_est]);
xlabel('潜标编号'); ylabel('北向偏移 (m)');
legend('真实(真值模型)', '估计(10维滤波器)', 'Location','best');
title('Y方向（北向）流场位移对比');
grid on;

sgtitle('深海潜标阵列水平偏移在线估计与标定补偿效果对账大盘');