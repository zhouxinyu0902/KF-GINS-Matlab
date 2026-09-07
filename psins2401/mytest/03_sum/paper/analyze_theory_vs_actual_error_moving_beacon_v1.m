function analyze_theory_vs_actual_error_moving_beacon_v1(avp_ref, avp_dr, aided_avp_cell, prefix, XK_cell, beacon_pos_cell, labels, dk, dphi_dyn, dt)
% analyze_theory_vs_actual_error_moving_beacon:
% 集成空间分布、理论敏感度分解、以及隐状态收敛分析
glvs;
num_tracks = length(aided_avp_cell);

% --- 统一配色方案 ---
colors_paper = [0 0.447 0.741; 0.85 0.325 0.098; 0.466 0.674 0.188; 0.929 0.694 0.125; 0.494 0.184 0.556; 0.301 0.745 0.933; 0.172 0.243 0.314; 0.737 0.741 0.133];
colors = [colors_paper; 0.635 0.078 0.184; 0.25 0.25 0.25; 0.89 0.467 0.761; 0.58 0.58 0.58];
% 方案一：高对比度科学配色
colors = [
    0.835, 0.369, 0.000; % 1. 朱红色 (极强对比)
    0.000, 0.620, 0.451; % 3. 蓝绿色
    0.337, 0.706, 0.914; % 2. 天蓝色 
    
    0.902, 0.624, 0.000; % 4. 桔黄色
    % 备用色
    0.800, 0.475, 0.655; % 5. 紫红色
    0.000, 0.447, 0.698  % 6. 深蓝色
];
Re = glv.Re; Rm = 6356752; Rn = 6378137;

% 创建 Figure
figA = myfigurestartup(6, 6, 'zxy');
figB = myfigurestartup(9, 3, 'zxy');

%% 数据预处理
t_ref = avp_ref(:, 10); pos_ref = avp_ref(:, 7:9);
t_dr = avp_dr(:, 10);
pos_dr_at_ref = [interp1(t_dr, avp_dr(:,7), t_ref), interp1(t_dr, unwrap(avp_dr(:,8)), t_ref), interp1(t_dr, avp_dr(:,9), t_ref)];

%% 图 1: 空间几何分布
figure(figA); ax1 = subplot(2, 2, 1); hold on; grid on;
ref_L0 = pos_ref(1,1); ref_lam0 = pos_ref(1,2);
x_ref = (pos_ref(:,2)-ref_lam0)*Re*cos(ref_L0); y_ref = (pos_ref(:,1)-ref_L0)*Re;
plot(x_ref, y_ref, 'k-', 'LineWidth', 2.5, 'DisplayName', 'True Path');

for i = 1:num_tracks
    b_p = beacon_pos_cell{i};
    xb = (b_p(:,2)-ref_lam0)*Re*cos(ref_L0);
    yb = (b_p(:,1)-ref_L0)*Re;
    if size(b_p, 1) == 1
        plot(xb, yb, 'p', 'MarkerSize', 12, 'Color', colors(i,:), 'DisplayName', labels{i+1});
    else
        plot(xb, yb, '-.', 'LineWidth', 1.5, 'Color', colors(i,:), 'DisplayName', [labels{i+1} ' (Moving)']);
    end
end
axis equal; xlabel('East (m)'); ylabel('North (m)'); 
% title('信标与载体空间分布');
legend('Location', 'best');

%% 图 2: 位置径向误差
figure(figA); ax2 = subplot(2, 2, 2); hold on; grid on;
err_dr = RCompu(pos_ref, pos_dr_at_ref);
plot(t_ref, err_dr, 'k--', 'LineWidth', 1.5, 'DisplayName', 'DR Error');
for i = 1:num_tracks
    curr = aided_avp_cell{i};
    plot(t_ref, RCompu(pos_ref, curr(:,7:9)), 'Color', colors(i,:), 'LineWidth', 1.8, 'DisplayName', labels{i+1});
end
xlabel('Time/s'); ylabel('Error/m'); 
% title('位置径向误差对比');
legend('Location', 'best');
xlim([t_ref(1) t_ref(end)])
%% 图 3 & 4: 理论 vs 实际残差 & 几何分解
% figure(figB); ax4 = subplot(1, 2, 1); hold on; grid on;
% ax3 = subplot(1, 2, 2); hold on; grid on;
%
% for i = 1:num_tracks
%     b_p_orig = beacon_pos_cell{i};
%     curr_full = aided_avp_cell{i};
%
%     % 统一采样：由于轨迹和动态信标长度不完全匹配(17602 vs 1101)，进行对齐
%     idx_sub = 15:16:size(curr_full, 1);
%     curr = curr_full(idx_sub, :);
%     t_sub = curr(:, 10);
%
%     if size(b_p_orig, 1) == 1
%         b_p_dyn = repmat(b_p_orig, length(idx_sub), 1);
%     else
%         % 动态信标对齐
%         len = min(length(idx_sub), size(b_p_orig, 1));
%         curr = curr(1:len, :);
%         t_sub = t_sub(1:len);
%         b_p_dyn = b_p_orig(1:len, :);
%     end
%
%     % 核心物理量计算
%     psi = curr(:,3); VN = curr(:,5); VE = curr(:,4); V = sqrt(VN.^2 + VE.^2);
%     L = curr(:,7); lam = curr(:,8); h = curr(:,9);
%     dN = (L - b_p_dyn(:,1)) .* (Rm + h);
%     dE = (lam - b_p_dyn(:,2)) .* (Rn + h) .* cos(L);
%     alpha = -atan2(dE, dN) - psi;
%
%     % 误差分解 (dt 在此处需反映下采样间隔)
%     dt_eff = dt * 16;
%     dZ_k_step = V .* dt_eff .* (dk .* cos(alpha));
%     dZ_p_step = V .* dt_eff .* (dphi_dyn(idx_sub(1:length(alpha))) .* sin(alpha));
%
%     dZ_theory = abs(cumsum(dZ_k_step + dZ_p_step));
%
%     % 绘图 3 (理论 vs 实际)
%     subplot(ax4);
%     plot(t_sub, dZ_theory, '-', 'Color', colors(i,:), 'LineWidth', 2, 'DisplayName', [labels{i+1} ' Theory']);
%     % 计算实际残差 (DR vs Ref 在信标投影上的差异)
%     dr_p = [interp1(t_dr, avp_dr(:,7), t_sub), interp1(t_dr, unwrap(avp_dr(:,8)), t_sub), zeros(length(t_sub),1)];
%     ref_p = [interp1(t_ref, avp_ref(:,7), t_sub), interp1(t_ref, unwrap(avp_ref(:,8)), t_sub), zeros(length(t_sub),1)];
%     res_actual = abs(RCompu(dr_p, b_p_dyn) - RCompu(ref_p, b_p_dyn));
%     plot(t_sub, res_actual, '--', 'Color', colors(i,:), 'HandleVisibility', 'off');
%
%     % 绘图 4 (敏感度分解)
%     subplot(ax3);
%     plot(t_sub, abs(cumsum(dZ_k_step)), '-', 'Color', colors(i,:), 'LineWidth', 1.2, 'DisplayName', [labels{i+1} ' \delta k']);
%     plot(t_sub, abs(cumsum(dZ_p_step)), ':', 'Color', colors(i,:), 'LineWidth', 1.2, 'DisplayName', [labels{i+1} ' \delta \psi']);
% end
% subplot(ax4); title('理论预测(实) vs 实际测距残差(虚)'); xlabel('Time/s'); ylabel('|dZ| (m)');
% subplot(ax3); title('dZ 几何投影分解 (实:\delta k, 点:\delta\psi)'); xlabel('Time/s'); ylabel('Sensitivity/m');
%% 图 3 & 4: 理论 vs 实际残差 & 几何分解 (改为插值信标思路)
figure(figB); ax4 = subplot(1, 3, 3); hold on; grid on;
ax3 = subplot(1, 3, 2); hold on; grid on;
ax34 = subplot(1, 3, 1); hold on; grid on;
for i = 1:num_tracks
    b_p_orig = beacon_pos_cell{i};
    curr_full = aided_avp_cell{i};
    t_full = curr_full(:, 10); % 导航级时间戳 (17602行)

    % --- 第一步：将信标位置插值到全频率 ---
    if size(b_p_orig, 1) == 1
        % 静态信标：直接复制
        b_p_full = repmat(b_p_orig, length(t_full), 1);
    else
        % 动态信标：假设动态信标采样点对应原下采样时间点 (15:16:end)
        % 如果你有动态信标自带的时间戳，请替换下面的 t_beacon_orig
        t_beacon_orig = t_full(15:16:min(length(t_full), 15 + (size(b_p_orig,1)-1)*16));
        len_b = min(length(t_beacon_orig), size(b_p_orig, 1));

        % 插值到全频率时间戳
        b_p_full = interp1(t_beacon_orig(1:len_b), b_p_orig(1:len_b, :), t_full, 'linear', 'extrap');
    end

    % --- 第二步：在全频率 (dt) 下计算物理量 ---
    psi = curr_full(:,3);
    VN = curr_full(:,5); VE = curr_full(:,4); V = sqrt(VN.^2 + VE.^2);
    L = curr_full(:,7); lam = curr_full(:,8); h = curr_full(:,9);

    % 计算视线角 alpha (全频率)
    dN = (L - b_p_full(:,1)) .* (Rm + h);
    dE = (lam - b_p_full(:,2)) .* (Rn + h) .* cos(L);
    theta = -atan2(dE, dN);
    alpha = theta - psi;

    % 误差分解步长 (使用原始 dt)
    dZ_k_step = V .* dt .* (dk .* cos(alpha));
    % 注意：dphi_dyn 如果是全频率的则直接用，如果是下采样的需要插值
    if length(dphi_dyn) < length(t_full)
        dphi_full = interp1(t_beacon_orig(1:len_b), dphi_dyn(1:len_b), t_full, 'nearest', 'extrap');
    else
        dphi_full = dphi_dyn(1:length(t_full));
    end
    dZ_p_step = V .* dt .* (dphi_full .* sin(alpha));
    dZ_k_step = cumsum(dZ_k_step);
    dZ_p_step = cumsum(dZ_p_step);
    % 理论累计误差
    % dZ_theory = abs(cumsum(dZ_k_step + dZ_p_step));
    dZ_theory = abs(dZ_k_step + dZ_p_step);
    % --- 第三步：计算实际测距残差 (全频率) ---
    % 插值 DR 和 Ref 轨迹（确保时间轴完全一致）
    dr_p = [interp1(t_dr, avp_dr(:,7), t_full), interp1(t_dr, unwrap(avp_dr(:,8)), t_full), zeros(length(t_full),1)];
    ref_p = [interp1(t_ref, avp_ref(:,7), t_full), interp1(t_ref, unwrap(avp_ref(:,8)), t_full), zeros(length(t_full),1)];

    % 全频率计算实际测距残差
    res_actual = abs(RCompu(dr_p, b_p_full) - RCompu(ref_p, b_p_full));

    % --- 第四步：绘图 (为了美观，绘图时可以每16个点点一个，或直接画全线) ---

    % 绘图 4
    subplot(ax3);
    plot(t_full, abs(dZ_k_step), '-', 'Color', colors(i,:), 'LineWidth', 1.2, 'DisplayName', [labels{i+1} ' \delta k']);
    legend('Location', 'southoutside', 'NumColumns', 2);
    xlim([t_full(1) t_full(end)])
    xlabel('Time/s');
    
    subplot(ax4);
    plot(t_full, abs(dZ_p_step), ':', 'Color', colors(i,:), 'LineWidth', 1.2, 'DisplayName', [labels{i+1} ' \delta \psi']);
    legend('Location', 'southoutside','NumColumns', 2);
    xlim([t_full(1) t_full(end)])

    subplot(ax34);
    plot(t_full, dZ_theory, '-', 'Color', colors(i,:), 'LineWidth', 2, 'DisplayName', [labels{i+1}]);
    % plot(t_full, res_actual, '--', 'Color', colors(i,:), 'HandleVisibility', 'off');
    legend('Location', 'southoutside', 'NumColumns', 2);
    xlim([t_full(1) t_full(end)])
end

%% 图 5: DVL 刻度因子误差估计
figure(figA); ax5 = subplot(2, 2, 3); hold on; grid on;
for i = 1:num_tracks
    plot(XK_cell{i}(:,end), XK_cell{i}(:,1), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', labels{i+1});
end
yline(dk, 'k--', 'LineWidth', 2); 
% title('DVL 刻度因子误差估计 (\delta k)');
legend('Location', 'best');
xlim([XK_cell{i}(1,end) XK_cell{i}(end,end)])
%% 图 6: 航向角偏差估计
curr_full = aided_avp_cell{i};
% 统一采样：由于轨迹和动态信标长度不完全匹配(17602 vs 1101)，进行对齐
idx_sub = 15:16:size(curr_full, 1);
figure(figA); ax6 = subplot(2, 2, 4); hold on; grid on;
plot(t_ref(idx_sub), rad2deg(dphi_dyn(idx_sub)), 'k--', 'LineWidth', 1.2, 'DisplayName', 'Reference');
for i = 1:num_tracks

    curr_sub = aided_avp_cell{i}(idx_sub, :);
    xk = XK_cell{i};
    len = min(size(xk,1), size(curr_sub,1));
    if size(xk, 2) == 6
        dphi_est = rad2deg(xk(1:len,2).*cos(2*curr_sub(1:len,3)) + xk(1:len,3).*sin(2*curr_sub(1:len,3)));
    else
        dphi_est = rad2deg(xk(1:len,2));
    end
    plot(xk(1:len, end), dphi_est, 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', labels{i+1});
end
% title('航向角偏差估计 (\delta \psi)'); 
xlabel('Time/s'); ylabel('Deg');
legend('Location', 'best');
xlim([xk(1, end) xk(len, end)])
linkaxes([ax2, ax3, ax4], 'x');
%% ==========================================
% 自动导出图片 (针对不同 Figure 比例优化)
% ==========================================
fprintf('\n%s 开始导出独立分析图 (分组处理) %s\n', repmat('-', 1, 10), repmat('-', 1, 10));

export_folder = 'Exported_Figures_Standardized';
if ~exist(export_folder, 'dir'), mkdir(export_folder); end

% --- 分组定义 ---
% 第一组：来自 figA (ax1, ax2, ax5, ax6) - 偏方比例
groupA_axes = [ax1, ax2, ax5, ax6];
groupA_names = {'1_Spatial_Distribution', '2_Radial_Error', '5_Scale_Factor_Error', '6_Heading_Error'};

% 第二组：来自 figB/figC (ax3, ax4, ax34) - 偏宽比例
groupB_axes = [ax3, ax4, ax34];
groupB_names = {'4_Sensitivity_Decomposition_2', '3_Sensitivity_Decomposition_1', '34_Theory_vs_Actual_Residual'};

% 合并用于循环
all_axes = [groupA_axes, groupB_axes];
all_names = [groupA_names, groupB_names];

for k = 1:length(all_axes)
    curr_ax = all_axes(k);
    
    % 构造导出路径
    file_path = fullfile('D:\WPS云盘\469639050\WPS云盘\Draft\LATEX\els-cas-templates\fig', [prefix, all_names{k}, '.pdf']);
    
    % 【核心修改：真正的所见即所得】
    % 直接将 curr_ax（坐标轴对象）传给 exportgraphics，而不是 figure。
    % 它会自动捕捉该坐标轴的标题、XY轴标签、刻度和图例，保持原比例紧凑导出。
    % 加上 'ContentType', 'vector' 可以保证输出的 PDF 是纯矢量图，非常适合 LaTeX。
    exportgraphics(curr_ax, file_path, 'Resolution', 600, 'ContentType', 'vector');
    
    % fprintf('已保存 [%d/%d]: %s \n', k, length(all_axes), all_names{k});
end

% for k = 1:length(all_axes)
%     curr_ax = all_axes(k);
% 
%     % 【关键修正】：动态获取父窗口及其尺寸单位
%     parent_fig = get(curr_ax, 'Parent');
%     old_units = get(parent_fig, 'Units');
%     set(parent_fig, 'Units', 'centimeters');
%     parent_pos = get(parent_fig, 'Position'); % [left bottom width height]
% 
%     % 计算该子图在父窗口中的实际厘米尺寸
%     ax_pos_norm = get(curr_ax, 'Position'); % 归一化坐标 [x y w h]
%     target_w = parent_pos(3) * ax_pos_norm(3);
%     target_h = parent_pos(4) * ax_pos_norm(4);
% 
%     % 创建临时画布 (稍作放大以容纳图例和标签，保持原比例)
%     % 1.8倍是经验值，既能保证高清，又能防止四周文字被切
%     tmp_fig = figure('Visible', 'off', 'Units', 'centimeters', ...
%         'Position', [5, 5, target_w * 1.2, target_h * 1.2], 'Color', 'w');
% 
%     % 复制坐标轴和图例
%     hLgd = curr_ax.Legend;
%     if ~isempty(hLgd)
%         % 复制时连同图例一起拷贝
%         new_objs = copyobj([curr_ax, hLgd], tmp_fig);
%         new_ax = new_objs(1);
%         new_lgd = new_objs(2);
%         set(new_lgd, 'Location', hLgd.Location);
%     else
%         new_ax = copyobj(curr_ax, tmp_fig);
%     end
% 
%     % 调整新图中坐标轴的位置，留出边距 (Margins)
%     set(new_ax, 'Units', 'normalized', 'Position', [0.15, 0.22, 0.75, 0.68]);
% 
%     % 执行 600 DPI 导出
%     % file_path = fullfile(export_folder, [all_names{k}, '.png']);
%     % exportgraphics(tmp_fig, file_path, 'Resolution', 600);
% 
%     file_path = fullfile('D:\WPS云盘\469639050\WPS云盘\Draft\LATEX\els-cas-templates\fig', [prefix, all_names{k}, '.pdf']);
%     exportgraphics(tmp_fig, file_path, 'Resolution', 600);
%     % 恢复父窗口原始单位并关闭临时窗口
%     set(parent_fig, 'Units', old_units);
%     close(tmp_fig);
% 
%     % fprintf('已保存 [%d/%d]: %s (来源: %s)\n', k, length(all_axes), all_names{k}, get(parent_fig, 'Name'));
% end

% 导出总览对比图
exportgraphics(figA, fullfile(export_folder, 'Preview_Figure_A.png'), 'Resolution', 600);
exportgraphics(figB, fullfile(export_folder, 'Preview_Figure_B.png'), 'Resolution', 600);

fprintf('所有子图已按比例独立导出完成！\n');
end