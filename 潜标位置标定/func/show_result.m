function [S_est_xyz,theta_next,phi_next] = show_result(id, S_gnss_geo, S_true_geo, pos0_geo, theta_est, phi_est, theta_ref, phi_ref)
%% ==================== 标定成果综合评估对账与表格统一看板 ====================
glvs
S_gnss_xyz = pos2dxyz(S_gnss_geo, pos0_geo);
S_true_xyz = pos2dxyz(S_true_geo, pos0_geo);

% 1. 参数准备与度数转换
if ~exist('glv','var') || ~isfield(glv,'deg')
    glv.deg = pi/180;
end

% 💡 真值角度是标量，不再进行多维数组处理
theta_true_rad = d2r(theta_ref); 
phi_true_rad   = d2r(phi_ref);     
theta_est_rad  = theta_est;
phi_est_rad    = phi_est;

theta_true_deg = theta_true_rad / glv.deg;
phi_true_deg   = phi_true_rad   / glv.deg;
theta_est_deg  = theta_est_rad  / glv.deg;
phi_est_deg    = phi_est_rad    / glv.deg;

% 计算每一个潜标的角度估计误差
theta_err_deg  = theta_est_deg - theta_true_deg;
phi_err_deg    = phi_est_deg   - phi_true_deg;

% 2. 无补偿前的真实偏移分量计算
depth = -S_gnss_xyz(:,3);   % 提取已知发声深度
n = length(depth);

% 【物理对齐】：X对应东向(sin)，Y对应北向(cos)
delta_x_true = depth .* tan(theta_true_rad) .* sin(phi_true_rad);
delta_y_true = depth .* tan(theta_true_rad) .* cos(phi_true_rad);
delta_x_est  = depth .* tan(theta_est_rad)  .* sin(phi_est_rad);
delta_y_est  = depth .* tan(theta_est_rad)  .* cos(phi_est_rad);

% 3. 位置还原误差与补偿坐标重构
S_est_xyz = zeros(n, 3);
for i = 1:n
    surf_x = S_gnss_xyz(i,1);
    surf_y = S_gnss_xyz(i,2);
    S_est_xyz(i,:) = [surf_x + delta_x_est(i), surf_y + delta_y_est(i), -depth(i)];
end
pos_err_3d = sqrt(sum((S_est_xyz - S_true_xyz).^2, 2));  % 三维位置还原绝对误差
pos_err_horiz = sqrt( (delta_x_est - delta_x_true).^2 + (delta_y_est - delta_y_true).^2 ); % 水平平面误差

% 4. 二次联合估计：残差角度计算
delta_x_rem = delta_x_true - delta_x_est; 
delta_y_rem = delta_y_true - delta_y_est; 
theta_rem_rad = zeros(n, 1);
phi_rem_rad   = zeros(n, 1);

% 预分配 17 列“标定全链条集成对账矩阵”
combinedTableData = zeros(n, 17);

for i = 1:n
    horiz_rem = sqrt(delta_x_rem(i)^2 + delta_y_rem(i)^2);
    theta_rem_rad(i) = atan2(horiz_rem, depth(i));
    phi_rem_rad(i) = atan2(delta_x_rem(i), delta_y_rem(i));
    
    if phi_rem_rad(i) < 0
        phi_rem_rad(i) = phi_rem_rad(i) + 2*pi;
    end
    
    theta_rem_deg = theta_rem_rad(i) / glv.deg;
    phi_rem_deg   = phi_rem_rad(i) / glv.deg;
    
    % ✨【Bug 修复核心位置】
    % 将 theta_true_deg 和 phi_true_deg 的 (i) 索引去掉，作为固定的标量填入每行
    combinedTableData(i, :) = [ ...
        i, depth(i), ...                                                % 1, 2: 基础属性
        theta_true_deg, phi_true_deg, ...                              % 3, 4: ✨ 修复：角度真值为固定单值
        theta_est_deg,  phi_est_deg, ...                         % 5, 6: 滤波估计角度(各个潜标不同)
        theta_err_deg,  phi_err_deg, ...                         % 7, 8: 角度绝对误差
        delta_x_true(i),   delta_y_true(i), ...                        % 9, 10: 真实流场位移(m)
        delta_x_est(i),    delta_y_est(i), ...                         % 11, 12: 估计流场位移(m)
        delta_x_est(i)-delta_x_true(i), delta_y_est(i)-delta_y_true(i), ... % 13, 14: 位置标定分量误差(m)
        pos_err_horiz(i), ...                                           % 15: 一次水平位置还原误差(m)
        theta_rem_deg,     phi_rem_deg ...                              % 16, 17: 二次迭代角度建议值(°)
    ];
end

theta_next_step = mean(theta_rem_rad);
phi_next_step   = mean(phi_rem_rad);

% 5. 图形可视化展示 (保留柱状图，不涉及命令行打印)
figure('Name', '流场位移对比柱状图', 'Color', [1 1 1], 'Position', [100 650 1000 320]);
subplot(1,2,1);
bar(1:n, [delta_x_true, delta_x_est]);
xlabel('潜标编号'); ylabel('东向偏移 (m)');
legend('真实(真值模型)', '估计(10维滤波器)', 'Location','best');
title('X方向（东向）流场位移对比'); grid on;

subplot(1,2,2);
bar(1:n, [delta_y_true, delta_y_est]);
xlabel('潜标编号'); ylabel('北向偏移 (m)');
legend('真实(真值模型)', '估计(10维滤波器)', 'Location','best');
title('Y方向（北向）流场位移对比'); grid on;
sgtitle('深海潜标阵列水平偏移在线估计与标定补偿效果');


%% ==================== UI集成大看板窗口弹出 ====================

% 创建超宽屏分辨率窗口 (1620x340)
fig = uifigure('Name', '深海潜标一/二次联合反演标定全要素融合看板', 'Position', [50 200 1620 340]);

% 1. 顶部级联参数与指标综合对账面板
lbl = uilabel(fig, 'Position', [20 225 1580 100], 'WordWrap', 'on', 'FontSize', 12);
lbl.BackgroundColor = [0.94 0.97 1.0]; % 浅蓝色背景
lbl.Text = sprintf([ ...
    ' 📊【评估结论】\n', ...
    ' 1. 角度标定误差：全阵列平均倾角误差 %.4f°，平均方位角误差 %.4f°。\n', ...
    ' 2. 潜标阵列还原：平均三维位置误差 %.3f m，平均水平还原误差被压缩至 %.3f m。\n\n', ...
    ' 🎯【下级迭代级联参数建议】\n', ...
    ' 二次联合估计初始注入建议值：倾角 theta_next = %.6f° (%.8f rad)  |  方位角 phi_next = %.6f° (%.8f rad)' ...
    ], mean(theta_err_deg), mean(phi_err_deg), mean(pos_err_3d), mean(pos_err_horiz), ...
       theta_next_step / glv.deg, theta_next_step, phi_next_step / glv.deg, phi_next_step);
theta_next = r2d(theta_next_step);
phi_next = r2d(phi_next_step);
% 2. 落地全维度集成表格
uit = uitable(fig, 'Position', [20 15 1580 195]);
uit.Data = combinedTableData;

% 17列逻辑环环相扣的表头栏
uit.ColumnName = { ...
    '潜标编号', '深度 (m)', ...
    '真值 theta(°)', '真值 phi(°)', ...
    '估计 theta(°)', '估计 phi(°)', ...
    'theta 误差(°)', 'phi 误差(°)', ...
    '真实 dx(东)', '真实 dy(北)', ...
    '估计 dx(东)', '估计 dy(北)', ...
    'dx 标定误差', 'dy 标定误差', ...
    '水平还原误差 (m)', ...
    '二次迭代 theta(°)', '二次迭代 phi(°)' ...
};

% 批量精细格式化数据列
columnFormats = repmat({'numeric'}, 1, 17);
columnFormats{1} = 'char';
uit.ColumnFormat = columnFormats;

% 17列专属像素宽度分配
uit.ColumnWidth = {65, 75, 95, 95, 95, 95, 95, 95, 90, 90, 90, 90, 95, 95, 115, 120, 120};
% ==================== ✨ 追加的自动导出 Excel 逻辑 ====================
% 1. 组装带有表头的数据 Cell 矩阵
exportHeader = { ...
    '潜标编号', '深度 (m)', ...
    '真值 theta(°)', '真值 phi(°)', ...
    '估计 theta(°)', '估计 phi(°)', ...
    'theta 误差(°)', 'phi 误差(°)', ...
    '真实 dx(东)', '真实 dy(北)', ...
    '估计 dx(东)', '估计 dy(北)', ...
    'dx 标定误差', 'dy 标定误差', ...
    '水平还原误差 (m)', ...
    '二次迭代 theta(°)', '二次迭代 phi(°)' ...
};

% 将数据转为 cell 并与表头垂直拼接
exportData = [exportHeader; num2cell(combinedTableData)];

% 2. 自动保存至当前目录下的 Excel 文件中
outputExcelName = ['深海潜标标定结果_',id,'.xlsx'];
try
    writecell(exportData, outputExcelName);
    % 如果是老版本 MATLAB，请使用下行命令：
    % xlswrite(outputExcelName, exportData); 
catch
    warning('Excel 文件正被占用，自动导出失败，请先关闭该 Excel。');
end
% =====================================================================
end