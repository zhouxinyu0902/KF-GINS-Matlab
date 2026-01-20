% 提取和整理数据 (单位: m)
% 排除 pureins，只比较 EKF, AEKF, FR-IFA AEKF, FFBS-Fusion
% 系统顺序：[EKF, AEKF, FR-IFA AEKF, FFBS-Fusion]

% Max 误差数据 (4 个场景 x 4 个系统)
Max_Error = [
    423.00, 423.00, 184.91, 99.78;  % Scenario 1
    284.73, 270.04, 107.03, 59.21;  % Scenario 2
    558.17, 558.17, 268.31, 116.37; % Scenario 3
    367.99, 367.99, 158.31, 64.25   % Scenario 4
];

% RMS 误差数据 (4 个场景 x 4 个系统)
RMS_Error = [
    209.40, 202.62, 78.12, 34.35;   % Scenario 1
    134.88, 129.90, 40.99, 21.68;   % Scenario 2
    217.25, 209.06, 77.10, 47.10;   % Scenario 3
    166.81, 161.40, 58.07, 26.76    % Scenario 4
];

% 系统名称标签
% System_Labels = {'EKF', 'AEKF', 'FR-IFA AEKF', 'FFBS-Fusion'};
System_Labels = {'EKF', 'AEKF', 'FR-IFA AEKF', 'FFBS-Fusion'};
Scenario_Labels = {'Scenario 1', 'Scenario 2', 'Scenario 3', 'Scenario 4'};

% -------------------- 绘图开始 --------------------
% figure('Position', [100, 100, 1200, 400]); % 设置图窗大小
myfigurestartup(7,3,'paper')

% ==================== Max 误差绘图 (子图 1: Max) ====================
subplot(1, 2, 1);
bar(Max_Error, 'grouped');
% title('Max Error Comparison Across Scenarios (Maximal Error)');
title('不同场景最大误差值对比');
ylabel('Max Error (m)');
xlabel('Scenario');
set(gca, 'XTickLabel', Scenario_Labels);
legend(System_Labels, 'Location', 'southoutside');
grid on;
ylim([0, 600]); % 统一Y轴，排除 pureins 的影响

% 在图上标注 pureins 的平均 Max 误差作为参考
Avg_Max_Pureins = mean([1993.43, 1748.24, 2091.99, 1825.98]);
% text(1, 550, ['pureins Avg Max: ' num2str(Avg_Max_Pureins, '%.1f') ' m'], 'Color', 'r', 'FontWeight', 'bold');
text(1, 570, ['纯惯导平均Max误差值: ' num2str(Avg_Max_Pureins, '%.1f') ' m'], 'Color', 'k', 'FontWeight', 'bold');

% ==================== RMS 误差绘图 (子图 2: RMS) ====================
subplot(1, 2, 2);
bar(RMS_Error, 'grouped');
% title('RMS Error Comparison Across Scenarios (Root Mean Square Error)');
title('不同场景均方误差值对比');
ylabel('RMS Error (m)');
xlabel('Scenario');
set(gca, 'XTickLabel', Scenario_Labels);
legend(System_Labels, 'Location', 'southoutside');
grid on;
ylim([0, 600]); % 统一Y轴，排除 pureins 的影响

% 在图上标注 pureins 的平均 RMS 误差作为参考
Avg_RMS_Pureins = mean([1445.25, 1257.87, 1510.67, 1321.97]);
% text(1, 230, ['pureins Avg RMS: ' num2str(Avg_RMS_Pureins, '%.1f') ' m'], 'Color', 'r', 'FontWeight', 'bold');
text(1, 570, ['纯惯导平均RMS误差值: ' num2str(Avg_RMS_Pureins, '%.1f') ' m'], 'Color', 'k', 'FontWeight', 'bold');

%%
% 提取和整理数据 (单位: m)
% 数据矩阵结构：行代表系统，列代表场景 (用于分组柱状图)
System_Labels = {'PureIns', 'EKF', 'AEKF', 'FR-IFA AEKF', 'FFBS-Fusion'};
Scenario_Labels = {'Scenario 1', 'Scenario 2', 'Scenario 3', 'Scenario 4'};

% Max 误差数据 (5 个系统 x 4 个场景)
Max_Error_Data = [
    1993.43, 1748.24, 2091.99, 1825.98; % PureIns
    423.00, 284.73, 558.17, 367.99;     % EKF
    423.00, 270.04, 558.17, 367.99;     % AEKF
    184.91, 107.03, 268.31, 158.31;     % FR-IFA AEKF
    99.78, 59.21, 116.37, 64.25         % FFBS-Fusion
]'; % 转置后，行代表场景，列代表系统 (为 bar(X) 准备)
Max_Error_Data = Max_Error_Data'; % 再次转置，让 bar(X) 默认分组更自然

% RMS 误差数据 (5 个系统 x 4 个场景)
RMS_Error_Data = [
    1445.25, 1257.87, 1510.67, 1321.97; % PureIns
    209.40, 134.88, 217.25, 166.81;     % EKF
    202.62, 129.90, 209.06, 161.40;     % AEKF
    78.12, 40.99, 77.10, 58.07;         % FR-IFA AEKF
    34.35, 21.68, 47.10, 26.76          % FFBS-Fusion
]'; % 转置后，行代表场景，列代表系统 (为 bar(X) 准备)
RMS_Error_Data = RMS_Error_Data'; % 再次转置，让 bar(X) 默认分组更自然

myfigurestartup(7,3,'paper')
% ==================== 图 1: Max 误差分组柱状图 ====================
subplot 121
bar(Max_Error_Data, 'grouped');
title('Maximal Error (Max) Comparison Across Scenarios');
ylabel('Max Error (m)');
set(gca, 'XTickLabel', System_Labels);
legend(Scenario_Labels, 'Location', 'NorthEast');
grid on;
set(gca, 'YGrid', 'on', 'GridAlpha', 0.5); % 增强网格线
ylim([0, 2500]); % 设定Y轴上限以包含 PureIns


% ==================== 图 2: RMS 误差分组柱状图 ====================
subplot 122
bar(RMS_Error_Data, 'grouped');
title('Root Mean Square Error (RMS) Comparison Across Scenarios');
ylabel('RMS Error (m)');
set(gca, 'XTickLabel', System_Labels);
legend(Scenario_Labels, 'Location', 'NorthEast');
grid on;
set(gca, 'YGrid', 'on', 'GridAlpha', 0.5); % 增强网格线
ylim([0, 1600]); % 设定Y轴上限以包含 PureIns


% 提示：如果您在 MATLAB 环境中执行，这两张图将分别在 Figure 1 和 Figure 2 中显示。
%%
%==========================================================================
% 关键数据定义 (基于前文计算结果)
%==========================================================================

% RMS 误差提升比数据 (相对于 pureins, 单位: %) - 用于左侧子图
% 注意：用户在输入代码中将此矩阵转置了两次，我们使用正确的数据结构
RMS_Improvement = [
    85.53, 89.29, 85.62, 87.49;  % EKF
    85.96, 89.67, 86.10, 87.78;  % AEKF
    94.59, 96.74, 94.89, 95.60;  % FR-IFA AEKF
    97.62, 98.27, 96.88, 97.98   % FFBS-Fusion
]; 

% 计算 RMS vs pureins 的平均值
RMS_Avg = mean(RMS_Improvement, 2);
System_Labels = {
    ['EKF vs PureIns (Avg: ' num2str(RMS_Avg(1), '%.2f') '%)'],
    ['AEKF vs PureIns (Avg: ' num2str(RMS_Avg(2), '%.2f') '%)'],
    ['FR-IFA AEKF vs PureIns (Avg: ' num2str(RMS_Avg(3), '%.2f') '%)'],
    ['FFBS-Fusion vs PureIns (Avg: ' num2str(RMS_Avg(4), '%.2f') '%)']
};

Scenario_Points = 1:4;
Scenario_Labels = {'Scenario 1', 'Scenario 2', 'Scenario 3', 'Scenario 4'};
Best_System_Index = 4;

% 递进提升比数据 (全部相对于 EKF, 单位: %) - 用于右侧子图
StepUp_Improvement_EKF_Basis = [
    3.24,  3.69,  3.77,  3.24;   % AEKF vs EKF
    62.65, 69.61, 64.51, 65.20;  % FR-IFA AEKF vs EKF
    83.50, 83.92, 78.32, 83.95   % FFBS-Fusion vs EKF
];

% 计算 vs EKF 的平均值
EKF_Avg = mean(StepUp_Improvement_EKF_Basis, 2);
Comparison_Labels_EKF_Basis = {
    ['AEKF vs EKF (Avg: ' num2str(EKF_Avg(1), '%.2f') '%)'],
    ['FR-IFA AEKF vs EKF (Avg: ' num2str(EKF_Avg(2), '%.2f') '%)'],
    ['FFBS-Fusion vs EKF (Avg: ' num2str(EKF_Avg(3), '%.2f') '%)']
};
myfigurestartup(8,5,'paper')
% -------------------- 绘图区域 --------------------


%--- 左侧子图 (1/2): 绝对性能对比 (vs pureins) ---
subplot(1, 2, 1);
h_rms = plot(Scenario_Points, RMS_Improvement', '-o', 'LineWidth', 2, 'MarkerSize', 8);
hold on;

title('RMS 误差提升比趋势 (相对于纯惯导)', 'FontSize', 14);
ylabel('提升比 (%)', 'FontSize', 12);
xlabel('轨迹场景', 'FontSize', 12);
set(gca, 'XTick', Scenario_Points, 'XTickLabel', Scenario_Labels);
ylim([80, 100]); 
grid on;

% 突出最佳算法 (FFBS-Fusion)
set(h_rms(Best_System_Index), 'Color', 'r', 'LineWidth', 3, 'MarkerFaceColor', 'r');

% 标注所有折线的数据点
Vertical_Offsets = [-0.9, 0.9, 0.4, 0.6]; % 设置标注垂直偏移量，避免重叠
Colors = get(gca, 'ColorOrder'); % 获取线条颜色

for line_idx = 1:size(RMS_Improvement, 1) % 遍历所有系统
    current_color = h_rms(line_idx).Color;
    for i = 1:length(Scenario_Points) % 遍历所有场景
        x_pos = Scenario_Points(i);
        y_pos = RMS_Improvement(line_idx, i);
        % 标注
        text(x_pos, y_pos + Vertical_Offsets(line_idx), [num2str(y_pos, '%.2f') '%'], ...
             'HorizontalAlignment', 'center', ...
             'Color', current_color, ...
             'FontSize', 8);
    end
end

legend(System_Labels, 'Location', 'SouthEast', 'FontSize', 9);
hold off;


%--- 右侧子图 (2/2): 相对性能对比 (全部 vs EKF) ---
subplot(1, 2, 2);
h_plot_step = plot(Scenario_Points, StepUp_Improvement_EKF_Basis', '-o', 'LineWidth', 2, 'MarkerSize', 8);
hold on;

title('RMS 误差提升比趋势 (相对于 EKF)', 'FontSize', 14);
ylabel('提升比 (%)', 'FontSize', 12);
xlabel('轨迹场景', 'FontSize', 12);
set(gca, 'XTick', Scenario_Points, 'XTickLabel', Scenario_Labels);
ylim([0, 95]); % 调整 Y 轴范围
grid on;

% 突出最高性能线 (FFBS-Fusion vs EKF)
set(h_plot_step(3), 'Color', 'r', 'LineWidth', 3, 'MarkerFaceColor', 'r'); 

% 标注所有折线的数据点
Vertical_Offsets_Right = [4, 4, -4]; % 设置标注垂直偏移量
Colors_Right = get(gca, 'ColorOrder');

for line_idx = 1:size(StepUp_Improvement_EKF_Basis, 1) % 遍历所有比较路径
    current_color = h_plot_step(line_idx).Color;
    for i = 1:length(Scenario_Points) % 遍历所有场景
        x_pos = Scenario_Points(i);
        y_pos = StepUp_Improvement_EKF_Basis(line_idx, i);
        % 标注
        text(x_pos, y_pos + Vertical_Offsets_Right(line_idx), [num2str(y_pos, '%.2f') '%'], ...
             'HorizontalAlignment', 'center', ...
             'Color', current_color, ...
             'FontSize', 8);
    end
end


legend(Comparison_Labels_EKF_Basis, 'Location', 'NorthEast', 'FontSize', 9);
hold off;