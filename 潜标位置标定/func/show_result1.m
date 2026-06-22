function [S_est_xyz, theta_next, phi_next] = show_result1( ...
    id, S_gnss_geo, S_true_geo, pos0_geo, ...
    theta_est, phi_est, theta_ref, phi_ref, outputExcelName, S_base_xyz)

%% ==================== 第 id 次迭代标定成果综合评估 ====================
% 说明：
% id         ：当前迭代次数
% theta_ref  ：本轮真实参考倾角，单位：deg
% phi_ref    ：本轮真实参考方位角，单位：deg
% theta_est  ：本轮估计倾角，单位：rad
% phi_est    ：本轮估计方位角，单位：rad
%
% S_base_xyz ：本轮补偿基准点，单位：m
%              第1轮可以不传，默认使用 S_gnss_xyz
%              第2轮应传入第1轮输出的 S_est_xyz
%
% 输出：
% S_est_xyz  ：本轮补偿后的累计发声点估计位置，单位：m
% theta_next ：下一轮残余真实参考倾角，单位：deg
% phi_next   ：下一轮残余真实参考方位角，单位：deg

glvs;

S_gnss_xyz = pos2dxyz(S_gnss_geo, pos0_geo);
S_true_xyz = pos2dxyz(S_true_geo, pos0_geo);

if ~exist('glv','var') || ~isfield(glv,'deg')
    glv.deg = pi/180;
end

if isnumeric(id)
    iterStr = num2str(id);
    iterIdForTable = id;
else
    iterStr = char(id);
    iterIdForTable = str2double(iterStr);
    if isnan(iterIdForTable)
        iterIdForTable = 0;
    end
end

%% ==================== 1. 本轮补偿基准点 ====================

n = size(S_gnss_xyz, 1);

if nargin < 10 || isempty(S_base_xyz)
    % 第1轮默认从 GNSS 原始位置开始补偿
    S_base_xyz = S_gnss_xyz;
    baseSourceName = 'GNSS原始点';
else
    % 第2轮及以后，从上一轮补偿后的发声点位置继续补偿
    if size(S_base_xyz, 1) ~= n || size(S_base_xyz, 2) ~= 3
        error('S_base_xyz 尺寸错误：必须为 %d x 3。', n);
    end
    baseSourceName = '上一轮补偿后的发声点';
end

% 深度以本轮补偿基准点为准
depth = -S_base_xyz(:,3);

%% ==================== 2. 角度参数准备 ====================

% theta_ref / phi_ref：本轮真实参考残余角，单位 deg
theta_ref_rad = theta_ref(:) * glv.deg;
phi_ref_rad   = phi_ref(:)   * glv.deg;

% theta_est / phi_est：本轮估计残余角，单位 rad
theta_est_rad = theta_est(:);
phi_est_rad   = phi_est(:);

% 支持标量输入
if numel(theta_ref_rad) == 1
    theta_ref_rad = repmat(theta_ref_rad, n, 1);
end
if numel(phi_ref_rad) == 1
    phi_ref_rad = repmat(phi_ref_rad, n, 1);
end
if numel(theta_est_rad) == 1
    theta_est_rad = repmat(theta_est_rad, n, 1);
end
if numel(phi_est_rad) == 1
    phi_est_rad = repmat(phi_est_rad, n, 1);
end

if numel(theta_ref_rad) ~= n || numel(phi_ref_rad) ~= n || ...
   numel(theta_est_rad) ~= n || numel(phi_est_rad) ~= n
    error('角度输入维度不匹配：theta_ref/phi_ref/theta_est/phi_est 必须是标量或长度为 n 的向量。');
end

theta_ref_deg = theta_ref_rad / glv.deg;
phi_ref_deg   = phi_ref_rad   / glv.deg;
theta_est_deg = theta_est_rad / glv.deg;
phi_est_deg   = phi_est_rad   / glv.deg;

theta_err_deg = theta_est_deg - theta_ref_deg;

phi_err_rad = mod((phi_est_rad - phi_ref_rad) + pi, 2*pi) - pi;
phi_err_deg = phi_err_rad / glv.deg;

theta_ref_show = mean(theta_ref_deg);
phi_ref_show   = mean(phi_ref_deg);
theta_est_show = mean(theta_est_deg);
phi_est_show   = mean(phi_est_deg);

%% ==================== 3. 本轮真实剩余偏移与估计补偿偏移 ====================
% 关键修正：
% 本轮真实剩余偏移不再默认从 GNSS 算，
% 而是从本轮基准点 S_base_xyz 指向真实点 S_true_xyz。

delta_x_true = S_true_xyz(:,1) - S_base_xyz(:,1);
delta_y_true = S_true_xyz(:,2) - S_base_xyz(:,2);

% 本轮估计出来的是相对于 S_base_xyz 的残余补偿量
delta_x_est = depth .* tan(theta_est_rad) .* sin(phi_est_rad);
delta_y_est = depth .* tan(theta_est_rad) .* cos(phi_est_rad);

%% ==================== 4. 本轮补偿后的累计发声点坐标 ====================

S_est_xyz = zeros(n, 3);

for i = 1:n
    S_est_xyz(i,:) = [ ...
        S_base_xyz(i,1) + delta_x_est(i), ...
        S_base_xyz(i,2) + delta_y_est(i), ...
        S_base_xyz(i,3) ...
    ];
end

pos_err_3d = sqrt(sum((S_est_xyz - S_true_xyz).^2, 2));

pos_err_horiz = sqrt( ...
    (S_est_xyz(:,1) - S_true_xyz(:,1)).^2 + ...
    (S_est_xyz(:,2) - S_true_xyz(:,2)).^2 );

%% ==================== 5. 计算补偿后的剩余残差，作为下一轮参考 ====================

delta_x_rem = S_true_xyz(:,1) - S_est_xyz(:,1);
delta_y_rem = S_true_xyz(:,2) - S_est_xyz(:,2);

theta_rem_rad = zeros(n, 1);
phi_rem_rad   = zeros(n, 1);

combinedTableData = zeros(n, 10);

for i = 1:n

    horiz_rem = sqrt(delta_x_rem(i)^2 + delta_y_rem(i)^2);

    theta_rem_rad(i) = atan2(horiz_rem, depth(i));

    phi_rem_rad(i) = atan2(delta_x_rem(i), delta_y_rem(i));

    if phi_rem_rad(i) < 0
        phi_rem_rad(i) = phi_rem_rad(i) + 2*pi;
    end

    combinedTableData(i, :) = [ ...
        iterIdForTable, i, depth(i), ...
        delta_x_true(i), delta_y_true(i), ...
        delta_x_est(i),  delta_y_est(i), ...
        delta_x_rem(i),  delta_y_rem(i), ...
        pos_err_horiz(i) ...
    ];
end

%% ==================== 6. 下一轮参考角输出 ====================

theta_next_step = mean(theta_rem_rad);

validPhi = sqrt(delta_x_rem.^2 + delta_y_rem.^2) > eps;

if any(validPhi)
    phi_next_step = atan2( ...
        mean(sin(phi_rem_rad(validPhi))), ...
        mean(cos(phi_rem_rad(validPhi))) ...
    );

    if phi_next_step < 0
        phi_next_step = phi_next_step + 2*pi;
    end
else
    phi_next_step = 0;
end

theta_next = theta_next_step / glv.deg;
phi_next   = phi_next_step   / glv.deg;

%% ==================== 7. 图形可视化 ====================

figure( ...
    'Name', ['第 ', iterStr, ' 次迭代：残余位移补偿对比柱状图'], ...
    'Color', [1 1 1]);

subplot(1,2,1);
bar(1:n, [delta_x_true, delta_x_est]);
xlabel('潜标编号');
ylabel('东向偏移 (m)');
legend('本轮真实剩余偏移', '本轮估计补偿偏移', 'Location','best');
title('X方向（东向）残余偏移补偿对比');
grid on;

subplot(1,2,2);
bar(1:n, [delta_y_true, delta_y_est]);
xlabel('潜标编号');
ylabel('北向偏移 (m)');
legend('本轮真实剩余偏移', '本轮估计补偿偏移', 'Location','best');
title('Y方向（北向）残余偏移补偿对比');
grid on;

sgtitle(['第 ', iterStr, ' 次迭代：基于 ', baseSourceName, ' 的残余补偿效果']);

%% ==================== 8. UI集成大看板窗口 ====================

fig = uifigure( ...
    'Name', ['第 ', iterStr, ' 次迭代：深海潜标联合反演标定看板'], ...
    'Position', [50 200 1180 360]);

lbl = uilabel(fig, ...
    'Position', [20 235 1140 105], ...
    'WordWrap', 'on', ...
    'FontSize', 12);

lbl.BackgroundColor = [0.94 0.97 1.0];

lbl.Text = sprintf([ ...
    ' 📊【第 %s 次迭代评估结论】\n', ...
    ' 0. 本轮补偿基准：%s。\n', ...
    ' 1. 本轮角度参考：theta_ref = %.6f°，phi_ref = %.6f°；本轮估计：theta_est = %.6f°，phi_est = %.6f°。\n', ...
    ' 2. 本轮角度估计误差：平均倾角绝对误差 %.4f°，平均方位角绝对误差 %.4f°。\n', ...
    ' 3. 补偿后位置误差：平均三维位置误差 %.3f m，平均水平位置误差 %.3f m。\n\n', ...
    ' 🎯【下一轮残余角参考】theta_next = %.6f° (%.8f rad)  |  phi_next = %.6f° (%.8f rad)' ...
    ], ...
    iterStr, ...
    baseSourceName, ...
    theta_ref_show, phi_ref_show, ...
    theta_est_show, phi_est_show, ...
    mean(abs(theta_err_deg)), ...
    mean(abs(phi_err_deg)), ...
    mean(pos_err_3d), ...
    mean(pos_err_horiz), ...
    theta_next, theta_next_step, ...
    phi_next, phi_next_step);

uit = uitable(fig, 'Position', [20 15 1140 205]);
uit.Data = combinedTableData;

uit.ColumnName = { ...
    '迭代次数', '潜标编号', '深度 (m)', ...
    '真实剩余 dx(东)', '真实剩余 dy(北)', ...
    '估计补偿 dx(东)', '估计补偿 dy(北)', ...
    '补偿后剩余 dx', '补偿后剩余 dy', ...
    '水平位置误差 (m)' ...
};

uit.ColumnFormat = repmat({'numeric'}, 1, 10);

uit.ColumnWidth = { ...
    75, 75, 85, ...
    125, 125, ...
    125, 125, ...
    125, 125, ...
    140 ...
};

%% ==================== 9. 自动导出 Excel ====================

exportHeader = { ...
    '迭代次数', '潜标编号', '深度 (m)', ...
    '真实剩余 dx(东)', '真实剩余 dy(北)', ...
    '估计补偿 dx(东)', '估计补偿 dy(北)', ...
    '补偿后剩余 dx', '补偿后剩余 dy', ...
    '水平位置误差 (m)' ...
};

exportData = [exportHeader; num2cell(combinedTableData)];

if nargin < 9 || isempty(outputExcelName)
    outputExcelName = ['第', iterStr, '次迭代_深海潜标标定结果.xlsx'];
end

try
    writecell(exportData, outputExcelName);
catch
    warning('Excel 文件正被占用，自动导出失败，请先关闭该 Excel。');
end

end