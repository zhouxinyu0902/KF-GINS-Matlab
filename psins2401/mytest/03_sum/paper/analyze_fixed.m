function H = analyze_fixed(avp_ref, avp_dr, aided_avp_cell, prefix, XK_cell, beacon_pos_cell, labels, dk, dphi_dyn, dt)
% analyze_theory_vs_actual_error_moving_beacon:
% 集成空间分布、理论敏感度分解、以及隐状态收敛分析
glvs;
num_tracks = length(aided_avp_cell);

% === 仅替换为高对比度科学配色方案 ===
% 方案一
if num_tracks<4
    colors = [
        % 0.900, 0.520, 0.550;   % wine red, light
        % 0.450, 0.650, 0.850;   % blue, light
        0.120, 0.330, 0.600;   % blue, main
        0.720, 0.180, 0.220;   % wine red, main
        0.000, 0.500, 0.360;   % group 3, main green
        ];
else
    % 方案二：Okabe-Ito 黄金配色
    colors = [
        % 0.900, 0.520, 0.550;   % wine red, light
        % 0.720, 0.180, 0.220;   % wine red, main
        % 0.450, 0.650, 0.850;   % blue, light
        % 0.120, 0.330, 0.600;   % blue, main
        % 0.500, 0.760, 0.640;   % group 3, light green
        % 0.000, 0.500, 0.360;   % group 3, main green

        %% 固定信标对比
        0.930, 0.650, 0.670;   % group 1, light wine red
        0.760, 0.260, 0.300;   % group 1, medium wine red
        0.520, 0.080, 0.130;   % group 1, dark wine red

        0.620, 0.760, 0.900;   % group 2, light blue
        0.280, 0.500, 0.740;   % group 2, medium blue
        0.070, 0.230, 0.480;   % group 2, dark blue

        ];

end
% style={'--','-','--','-','--','-','-'};
style={'--','-','--','--','-','--'};
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
legend('Location', 'best');

%% 图 2: 位置径向误差
figure(figA); ax2 = subplot(2, 2, 2); hold on; grid on;
err_dr = RCompu(pos_ref, pos_dr_at_ref);
plot(t_ref, err_dr, 'k--', 'LineWidth', 1.5, 'DisplayName', 'DR Error');
for i = 1:num_tracks
    curr = aided_avp_cell{i};
    plot(t_ref, RCompu(pos_ref, curr(:,7:9)), 'Color', colors(i,:),'LineStyle',style{i}, 'LineWidth', 1.8, 'DisplayName', labels{i+1});
end
xlabel('Time/s'); ylabel('Error/m');
legend('Location', 'best');
xlim([t_ref(1) t_ref(end)])

%% 图 3 & 4: 理论 vs 实际残差 & 几何分解 (保持您原有的全频率插值逻辑)
figure(figB); ax4 = subplot(1, 3, 3); hold on; grid on;
ax3 = subplot(1, 3, 2); hold on; grid on;
ax34 = subplot(1, 3, 1); hold on; grid on;
for i = 1:num_tracks
    b_p_orig = beacon_pos_cell{i};
    curr_full = aided_avp_cell{i};
    t_full = curr_full(:, 10);

    if size(b_p_orig, 1) == 1
        b_p_full = repmat(b_p_orig, length(t_full), 1);
    else
        t_beacon_orig = t_full(15:16:min(length(t_full), 15 + (size(b_p_orig,1)-1)*16));
        len_b = min(length(t_beacon_orig), size(b_p_orig, 1));
        b_p_full = interp1(t_beacon_orig(1:len_b), b_p_orig(1:len_b, :), t_full, 'linear', 'extrap');
    end

    psi = curr_full(:,3);
    VN = curr_full(:,5); VE = curr_full(:,4); V = sqrt(VN.^2 + VE.^2);
    L = curr_full(:,7); lam = curr_full(:,8); h = curr_full(:,9);

    dN = (L - b_p_full(:,1)) .* (Rm + h);
    dE = (lam - b_p_full(:,2)) .* (Rn + h) .* cos(L);
    theta = -atan2(dE, dN);
    alpha = theta - psi;

    dZ_k_step = V .* dt .* (dk .* cos(alpha));
    if length(dphi_dyn) < length(t_full)
        dphi_full = interp1(t_beacon_orig(1:len_b), dphi_dyn(1:len_b), t_full, 'nearest', 'extrap');
    else
        dphi_full = dphi_dyn(1:length(t_full));
    end
    dZ_p_step = V .* dt .* (dphi_full .* sin(alpha));

    dZ_k_step = cumsum(dZ_k_step);
    dZ_p_step = cumsum(dZ_p_step);
    dZ_theory = abs(dZ_k_step + dZ_p_step);

    subplot(ax3);
    plot(t_full, abs(dZ_k_step), '-', 'Color', colors(i,:), 'LineWidth', 1.2, 'DisplayName', [labels{i+1} ' \delta k']);
    legend('Location', 'southoutside', 'NumColumns', 2);
    xlim([t_full(1) t_full(end)])
    xlabel('Time/s');

    subplot(ax4);
    plot(t_full, abs(dZ_p_step), ':', 'Color', colors(i,:), 'LineWidth', 1.2, 'DisplayName', [labels{i+1} ' \delta \psi']);
    legend('Location', 'southoutside','NumColumns', 2);
    xlim([t_full(1) t_full(end)])
    xlabel('Time/s');

    subplot(ax34);
    plot(t_full, dZ_theory, '-', 'Color', colors(i,:), 'LineWidth', 2, 'DisplayName', [labels{i+1}]);
    legend('Location', 'southoutside', 'NumColumns', 2);
    xlim([t_full(1) t_full(end)])
    xlabel('Time/s');
end

%% 图 5: DVL 刻度因子误差估计
figure(figA); ax5 = subplot(2, 2, 3); hold on; grid on;
for i = 1:num_tracks
    plot(XK_cell{i}(:,end), XK_cell{i}(:,1), 'Color', colors(i,:), 'LineWidth', 1.5, 'DisplayName', labels{i+1});
end
yline(dk, 'k--', 'LineWidth', 2);
legend('Location', 'best');
xlim([XK_cell{1}(1,end) XK_cell{1}(end,end)])

%% 图 6: 航向角偏差估计
curr_full = aided_avp_cell{i};
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
    plot(xk(1:len, end), dphi_est, 'Color', colors(i,:), 'LineWidth', 1.5,'LineStyle',style{i}, 'DisplayName', labels{i+1});
end
xlabel('Time/s'); ylabel('Deg');
legend('Location', 'best');
xlim([xk(1, end) xk(len, end)])
linkaxes([ax2, ax3, ax4], 'x');
%% Return figure and axes handles
H = struct();

% Figure handles
H.figA = figA;
H.figB = figB;
H.figures = [figA, figB];

% Axes handles
H.axes = struct();
H.axes.SpatialDistribution      = ax1;
H.axes.RadialError              = ax2;
H.axes.SensitivityScaleFactor   = ax3;
H.axes.SensitivityHeading       = ax4;
H.axes.TheoryResidual           = ax34;
H.axes.ScaleFactorError         = ax5;
H.axes.HeadingError             = ax6;

% Export groups
H.groupA_axes = [ax1, ax2, ax5, ax6];
H.groupA_names = {
    '1_Spatial_Distribution', ...
    '2_Radial_Error', ...
    '5_Scale_Factor_Error', ...
    '6_Heading_Error'
    };

H.groupB_axes = [ax3, ax4, ax34];
H.groupB_names = {
    '3_Sensitivity_ScaleFactor', ...
    '4_Sensitivity_Heading', ...
    '34_Theory_vs_Actual_Residual'
    };

H.all_axes = [H.groupA_axes, H.groupB_axes];
H.all_names = [H.groupA_names, H.groupB_names];

% Store prefix and labels for external export
H.prefix = prefix;
H.labels = labels;
% %% 自动导出图片 (针对不同 Figure 比例优化)
% fprintf('\n%s 开始导出独立分析图 (分组处理) %s\n', repmat('-', 1, 10), repmat('-', 1, 10));
% export_folder = 'Exported_Figures_Standardized';
% if ~exist(export_folder, 'dir'), mkdir(export_folder); end
%
% groupA_axes = [ax1, ax2, ax5, ax6];
% groupA_names = {'1_Spatial_Distribution', '2_Radial_Error', '5_Scale_Factor_Error', '6_Heading_Error'};
% groupB_axes = [ax3, ax4, ax34];
% groupB_names = {'4_Sensitivity_Decomposition_2', '3_Sensitivity_Decomposition_1', '34_Theory_vs_Actual_Residual'};
% all_axes = [groupA_axes, groupB_axes];
% all_names = [groupA_names, groupB_names];
%
% for k = 1:length(all_axes)
%     curr_ax = all_axes(k);
%     file_path = fullfile('D:\WPS云盘\469639050\WPS云盘\Draft\LATEX\els-cas-templates\fig', [prefix, all_names{k}, '.pdf']);
%     exportgraphics(curr_ax, file_path, 'Resolution', 600, 'ContentType', 'vector');
% end
%
% exportgraphics(figA, fullfile(export_folder, 'Preview_Figure_A.png'), 'Resolution', 600);
% exportgraphics(figB, fullfile(export_folder, 'Preview_Figure_B.png'), 'Resolution', 600);
% fprintf('所有子图已按比例独立导出完成！\n');
end