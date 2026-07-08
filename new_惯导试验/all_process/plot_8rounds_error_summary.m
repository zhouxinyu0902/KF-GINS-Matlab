%% =========================================================================
% 8组数据误差统计批量对比与画图 (Standalone Plotting)
% 目的：横向对比 8 组实验的 Max, RMS, Mean 三大核心误差指标
% =========================================================================
clear; clc; close all;

% ==================== 1. 基础配置 ====================
base_output_dir = 'D:\Github\KF-GINS-Matlab\new_惯导试验\output';
target_rounds = 1:8;          
unitType = 'rad'; % 这里假设你只对比 'm' 模式的误差。如果想看 'rad' 也可以改。

% 预分配三维数组存放数据：大小为 (3个指标) x (3种算法) x (8组数据)
% 维度1(指标): 1=Max, 2=RMS, 3=Mean
% 维度2(算法): 1=Origin, 2=DoubleSmooth, 3=SingleSmooth
all_data_matrix = NaN(3, 3, 8); 

algorithm_names = {'Origin', 'RTS-DoubleSmooth', 'RTS-SingleSmooth'};
metric_names = {'最大径向误差 (Max / m)', '均方根误差 (RMS / m)', '均值 (Mean / m)'};
round_labels = {'Round 1', 'Round 2', 'Round 3', 'Round 4', 'Round 5', 'Round 6', 'Round 7', 'Round 8'};

fprintf('🚀 正在读取 %d 组误差统计 Excel 文件...\n', length(target_rounds));

% ==================== 2. 读取所有 Excel 数据 ====================
for id = target_rounds
    excel_path = fullfile(base_output_dir, sprintf('output%d', id), ...
                          sprintf('导航系统径向误差统计报告-%s-%d.xlsx', unitType, id));
    
    if ~exist(excel_path, 'file')
        warning('未找到第 %d 组的 Excel 文件，跳过此组数据。', id);
        continue;
    end
    
    % 读取 Excel 数据
    % 假设你的 Excel 格式固定为：
    % 第1行: 表头
    % 第2行: Origin-m 数据 (B列=Max, C列=RMS, D列=Mean)
    % 第3行: RTS-DoubleSmooth-m 数据
    % 第4行: RTS-SingleSmooth-m 数据
    try
        [num_data, txt_data, raw_data] = xlsread(excel_path);
        
        % 提取 Origin (Excel 第 2 行 -> 矩阵第 1 行)
        all_data_matrix(1, 1, id) = raw_data{2, 2}; % Max
        all_data_matrix(2, 1, id) = raw_data{2, 3}; % RMS
        all_data_matrix(3, 1, id) = raw_data{2, 4}; % Mean
        
        % 提取 RTS-DoubleSmooth (Excel 第 3 行 -> 矩阵第 2 行)
        all_data_matrix(1, 2, id) = raw_data{3, 2}; % Max
        all_data_matrix(2, 2, id) = raw_data{3, 3}; % RMS
        all_data_matrix(3, 2, id) = raw_data{3, 4}; % Mean
        
        % 提取 RTS-SingleSmooth (Excel 第 4 行 -> 矩阵第 3 行)
        all_data_matrix(1, 3, id) = raw_data{4, 2}; % Max
        all_data_matrix(2, 3, id) = raw_data{4, 3}; % RMS
        all_data_matrix(3, 3, id) = raw_data{4, 4}; % Mean
        
    catch ME
        warning('读取第 %d 组 Excel 文件失败: %s', id, ME.message);
    end
end

fprintf('✅ 数据读取完毕，开始生成对比图表...\n');

% ==================== 3. 绘制并精细化标注 ====================
bar_colors = [0.85 0.33 0.10; 0.00 0.45 0.74; 0.47 0.67 0.19]; 
% fig = figure('Name', '8组数据误差对比汇总', 'Position', [50, 50, 1600, 900], 'Color', 'w');
fig = myfigurestartup(10,7,'prese');
for metric_idx = 1:3
    subplot(3, 1, metric_idx);
    plot_data = squeeze(all_data_matrix(metric_idx, :, :)); 
    b = bar(plot_data', 0.8); % 增加宽度系数 0.8
    
    for i = 1:3
        b(i).FaceColor = bar_colors(i, :);
    end
    
    % --- 🌟 稳定版数值标注逻辑 ---
    for i = 1:3
        % 获取当前这一组柱子的所有中心 X 坐标
        xtips = b(i).XEndPoints; 
        % 获取对应的 Y 坐标 (即数据值)
        ytips = b(i).YEndPoints;
        
        for j = 1:length(ytips)
            % 如果数值为 NaN (某些组缺失)，跳过不标
            if isnan(ytips(j)), continue; end
            
            % 使用 text 标注
            text(xtips(j), ytips(j), num2str(ytips(j), '%.1f'), ...
                'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'bottom', ...
                'FontSize', 8, ...
                'Rotation', 0, ...
                'FontWeight', 'bold');
        end
    end
    % ---------------------------
    
    title(metric_names{metric_idx}, 'FontSize', 14);
    ylabel('误差 (m)', 'FontSize', 12);
    set(gca, 'XTickLabel', round_labels, 'FontSize', 10);
    
    % 自动留出上方空间
    yl = ylim;
    ylim([0, yl(2) * 1.3]); 
    
    if metric_idx == 1
        legend(algorithm_names, 'Location', 'best', 'FontSize', 10);
    end
end

% ==================== 4. 保存汇总大图 ====================
output_image_path = fullfile(base_output_dir, sprintf('8组数据误差汇总对比图-%s.png', unitType));
exportgraphics(fig, output_image_path, 'Resolution', 600);
fprintf('\n🎉 对比图表生成完毕！已保存至: %s\n', output_image_path);