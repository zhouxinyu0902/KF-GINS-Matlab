% 提取和整理数据 (单位: m)
% 原始表格数据：测试编号 4, 5, 6, 未编号 (作为新的 Scenario 1, 2, 3, 4)
% 系统顺序：[EKF, AEKF, BRC+AEKF, GTS+BRC+AEKF]

% -------------------- 提取 Max 误差数据 --------------------
% Max 误差数据 (4 个场景 x 4 个系统)
% 场景 1 (Test 1) Max: [302.60, 298.00, 150.70, 87.70]
% 场景 2 (Test 2) Max: [1047.21, 1047.21, 399.84, 241.96]
% 场景 3 (Test 3) Max: [537.83, 510.56, 211.13, 137.12]
% 场景 4 (未编号) Max: [1273.47, 1272.88, 532.63, 407.17]
Max_Error = [
    302.60, 298.00, 150.70, 87.70;      % Scenario 1 (Test 1)
    1047.21, 1047.21, 399.84, 241.96;   % Scenario 2 (Test 2)
    537.83, 510.56, 211.13, 137.12;     % Scenario 3 (Test 3)
    1273.47, 1272.88, 532.63, 407.17    % Scenario 4 (Test 4)
];

% -------------------- 提取 RMS 误差数据 --------------------
% RMS 误差数据 (4 个场景 x 4 个系统)
% 场景 1 (Test 1) RMS: [128.70, 127.26, 59.94, 45.44]
% 场景 2 (Test 2) RMS: [343.89, 335.81, 122.54, 81.59]
% 场景 3 (Test 3) RMS: [214.87, 206.05, 77.77, 41.54]
% 场景 4 (未编号) RMS: [506.71, 512.05, 235.62, 184.53]
RMS_Error = [
    128.70, 127.26, 59.94, 45.44;       % Scenario 1 (Test 1)
    343.89, 335.81, 122.54, 81.59;      % Scenario 2 (Test 2)
    214.87, 206.05, 77.77, 41.54;       % Scenario 3 (Test 3)
    506.71, 512.05, 235.62, 184.53      % Scenario 4 (Test 4)
];

% -------------------- PureIns 平均误差 (作为参考) --------------------
% PureIns Max: [1370.42, 3267.90, 2865.63, 3576.09]
% PureIns RMS: [734.47, 2377.81, 1319.13, 2697.93]
Avg_Max_Pureins = mean([1370.42, 3267.90, 2865.63, 3576.09]);
Avg_RMS_Pureins = mean([734.47, 2377.81, 1319.13, 2697.93]);


% 系统名称标签
System_Labels = {'EKF', 'AEKF', 'BRC+AEKF', 'GTS+BRC+AEKF'};
Scenario_Labels = {'Test 1', 'Test 2', 'Test 3', 'Test 4'}; % 更改场景标签以反映数据来源
% -------------------- 绘图开始 (Max & RMS 对比) --------------------
% figure('Position', [100, 100, 1200, 400]); % 设置图窗大小
% 假设 myfigurestartup 是一个自定义函数，此处保持原样
myfigurestartup(7,3,'paper') 

% ==================== Max 误差绘图 (子图 1: Max) ====================
subplot(1, 2, 1);
bar(Max_Error, 'grouped');
title('不同场景最大误差值对比 (Max Error)');
ylabel('Max Error (m)');
xlabel('测试场景 (Test Scenario)');
set(gca, 'XTickLabel', Scenario_Labels);
legend(System_Labels, 'Location', 'southoutside');
grid on;
ylim([0, 1400]); % 统一Y轴，根据新数据调整上限
% 在图上标注 pureins 的平均 Max 误差作为参考
text(1, 1350, ['纯惯导平均Max误差值: ' num2str(Avg_Max_Pureins, '%.1f') ' m'], 'Color', 'k', 'FontWeight', 'bold');

% ==================== RMS 误差绘图 (子图 2: RMS) ====================
subplot(1, 2, 2);
bar(RMS_Error, 'grouped');
title('不同场景均方误差值对比 (RMS Error)');
ylabel('RMS Error (m)');
xlabel('测试场景 (Test Scenario)');
set(gca, 'XTickLabel', Scenario_Labels);
legend(System_Labels, 'Location', 'southoutside');
grid on;
ylim([0, 600]); % 统一Y轴，根据新数据调整上限
% 在图上标注 pureins 的平均 RMS 误差作为参考
text(1, 580, ['纯惯导平均RMS误差值: ' num2str(Avg_RMS_Pureins, '%.1f') ' m'], 'Color', 'k', 'FontWeight', 'bold');


%% 以下是根据原始数据计算提升比的第二段代码，也需要相应调整：

% -------------------- 提升比计算与绘图 --------------------
% 原始数据 (5 个系统 x 4 个场景)
% Max_Error_Data 和 RMS_Error_Data 需要重新定义，以便计算提升比
% PureIns RMS: [734.47, 2377.81, 1319.13, 2697.93]
% EKF RMS:     [128.70, 343.89, 214.87, 506.71]
% AEKF RMS:    [127.26, 335.81, 206.05, 512.05]
% BRC+AEKF RMS:[59.94, 122.54, 77.77, 235.62]
% GTS+BRC+AEKF RMS: [45.44, 81.59, 41.54, 184.53]

RMS_PureIns = [734.47, 2377.81, 1319.13, 2697.93];
RMS_EKF = [128.70, 343.89, 214.87, 506.71];
RMS_AEKF = [127.26, 335.81, 206.05, 512.05];
RMS_BRC_AEKF = [59.94, 122.54, 77.77, 235.62];
RMS_GTS_BRC_AEKF = [45.44, 81.59, 41.54, 184.53];

% RMS 误差提升比数据 (相对于 pureins, 单位: %) - 用于左侧子图
RMS_Improvement = [
    (1 - RMS_EKF ./ RMS_PureIns) * 100;
    (1 - RMS_AEKF ./ RMS_PureIns) * 100;
    (1 - RMS_BRC_AEKF ./ RMS_PureIns) * 100;
    (1 - RMS_GTS_BRC_AEKF ./ RMS_PureIns) * 100
];

% 递进提升比数据 (全部相对于 EKF, 单位: %) - 用于右侧子图
StepUp_Improvement_EKF_Basis = [
    (1 - RMS_AEKF ./ RMS_EKF) * 100;           % AEKF vs EKF
    (1 - RMS_BRC_AEKF ./ RMS_EKF) * 100;       % BRC+AEKF vs EKF
    (1 - RMS_GTS_BRC_AEKF ./ RMS_EKF) * 100    % GTS+BRC+AEKF vs EKF
];

% 重新定义绘图标签 (与原代码保持一致的风格)
RMS_Avg = mean(RMS_Improvement, 2);
System_Labels_Plot = {
    ['EKF vs PureIns (Avg: ' num2str(RMS_Avg(1), '%.2f') '%)'],
    ['AEKF vs PureIns (Avg: ' num2str(RMS_Avg(2), '%.2f') '%)'],
    ['BRC+AEKF vs PureIns (Avg: ' num2str(RMS_Avg(3), '%.2f') '%)'],
    ['GTS+BRC+AEKF vs PureIns (Avg: ' num2str(RMS_Avg(4), '%.2f') '%)']
};
EKF_Avg = mean(StepUp_Improvement_EKF_Basis, 2);
Comparison_Labels_EKF_Basis = {
    ['AEKF vs EKF (Avg: ' num2str(EKF_Avg(1), '%.2f') '%)'],
    ['BRC+AEKF vs EKF (Avg: ' num2str(EKF_Avg(2), '%.2f') '%)'],
    ['GTS+BRC+AEKF vs EKF (Avg: ' num2str(EKF_Avg(3), '%.2f') '%)']
};
Scenario_Points = 1:4;
Scenario_Labels_Plot = {'Test 1', 'Test 2', 'Test 3', 'Test 4'};
Best_System_Index = 4;


% -------------------- 绘图区域 (提升比) --------------------
figure('Position', [100, 100, 1400, 600]); 
%--- 左侧子图 (1/2): 绝对性能对比 (vs pureins) ---
subplot(1, 2, 1);
h_rms = plot(Scenario_Points, RMS_Improvement', '-o', 'LineWidth', 2, 'MarkerSize', 8);
hold on;
title('RMS 误差提升比趋势 (相对于纯惯导)', 'FontSize', 14);
ylabel('提升比 (%)', 'FontSize', 12);
xlabel('测试场景', 'FontSize', 12);
set(gca, 'XTick', Scenario_Points, 'XTickLabel', Scenario_Labels_Plot);
ylim([50, 100]); % 调整Y轴范围适应新数据
grid on;
% 突出最佳算法 (GTS+BRC+AEKF)
set(h_rms(Best_System_Index), 'Color', 'r', 'LineWidth', 3, 'MarkerFaceColor', 'r');
% 标注所有折线的数据点
Vertical_Offsets = [-3, 3, 3, 3]; % 调整标注垂直偏移量，避免重叠
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
legend(System_Labels_Plot, 'Location', 'SouthEast', 'FontSize', 9);
hold off;

%--- 右侧子图 (2/2): 相对性能对比 (全部 vs EKF) ---
subplot(1, 2, 2);
h_plot_step = plot(Scenario_Points, StepUp_Improvement_EKF_Basis', '-o', 'LineWidth', 2, 'MarkerSize', 8);
hold on;
title('RMS 误差提升比趋势 (相对于 EKF)', 'FontSize', 14);
ylabel('提升比 (%)', 'FontSize', 12);
xlabel('测试场景', 'FontSize', 12);
set(gca, 'XTick', Scenario_Points, 'XTickLabel', Scenario_Labels_Plot);
ylim([0, 85]); % 调整 Y 轴范围
grid on;
% 突出最高性能线 (GTS+BRC+AEKF vs EKF)
set(h_plot_step(3), 'Color', 'r', 'LineWidth', 3, 'MarkerFaceColor', 'r'); 
% 标注所有折线的数据点
Vertical_Offsets_Right = [3, 3, -3]; % 设置标注垂直偏移量
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