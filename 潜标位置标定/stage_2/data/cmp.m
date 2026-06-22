close all hidden
% -------------------------------------------------------------------------
% 功能：
%   对比三组实验的两次迭代标定效果。
%
% 对比对象：
%   Circle_ArrayCenter_Trj
%   Line_ArrayCenter_Trj
%   Square_ArrayCenter_Trj
%
% 对比指标：
%   1. 第1轮平均水平误差
%   2. 第2轮平均水平误差
%   3. 平均水平误差改善量
%   4. 平均水平误差改善率
%   5. 每个潜标的水平误差变化
%
% 说明：
%   不比较 Z 轴，不比较角度。
%   只比较 S_est_xyz 与 S_true_xyz 的水平位置误差。
% -------------------------------------------------------------------------

    %% ==================== 1. 三组实验路径配置 ====================

    % baseFolders = { ...
    %     'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Circle_ArrayCenter_Trj\', ...
    %     'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Line_ArrayCenter_Trj\', ...
    %     'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Square_ArrayCenter_Trj\' ...
    % };
    
    baseFolders{1}   = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Circle_Beacon1_Trj\';
    baseFolders{2}   = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Line_Beacon1_Trj\';
    baseFolders{3}  = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Square_Beacon1_Trj\';

    expNames = { ...
        'Circle', ...
        'Line', ...
        'Square' ...
    };

    iterA = 1;
    iterB = 2;

    numExp = numel(baseFolders);

    %% ==================== 2. 结果预分配 ====================

    meanErrIter1 = zeros(numExp, 1);
    meanErrIter2 = zeros(numExp, 1);

    maxErrIter1 = zeros(numExp, 1);
    maxErrIter2 = zeros(numExp, 1);

    improveMean = zeros(numExp, 1);
    improveRate = zeros(numExp, 1);

    allErrIter1 = cell(numExp, 1);
    allErrIter2 = cell(numExp, 1);

    %% ==================== 3. 循环读取三组实验 ====================

    fprintf('\n==================== 三组实验两次迭代效果对比 ====================\n');
    fprintf('对比方式：只比较 S_est_xyz 与 S_true_xyz 的水平误差，不比较 Z 轴。\n\n');

    for e = 1:numExp

        baseFolder = baseFolders{e};
        pathpos = fullfile(baseFolder, '/input/beacon_pos.mat');

        fileA = fullfile(baseFolder, sprintf('iter_%d_next_ref.mat', iterA));
        fileB = fullfile(baseFolder, sprintf('iter_%d_next_ref.mat', iterB));

        if ~exist(pathpos, 'file')
            error('找不到 beacon_pos.mat：%s', pathpos);
        end

        if ~exist(fileA, 'file')
            error('找不到第 %d 轮结果文件：%s', iterA, fileA);
        end

        if ~exist(fileB, 'file')
            error('找不到第 %d 轮结果文件：%s', iterB, fileB);
        end

        mapData = load(pathpos);
        dataA = load(fileA);
        dataB = load(fileB);

        if ~isfield(mapData, 'S_true_xyz')
            error('%s 中缺少 S_true_xyz。', pathpos);
        end

        if ~isfield(dataA, 'S_est_xyz')
            error('%s 中缺少 S_est_xyz。', fileA);
        end

        if ~isfield(dataB, 'S_est_xyz')
            error('%s 中缺少 S_est_xyz。', fileB);
        end

        S_true_xyz = mapData.S_true_xyz;
        S_est_A = dataA.S_est_xyz;
        S_est_B = dataB.S_est_xyz;

        n = size(S_true_xyz, 1);

        S_true_xy = S_true_xyz(1:n, 1:2);
        S_est_A_xy = S_est_A(1:n, 1:2);
        S_est_B_xy = S_est_B(1:n, 1:2);

        errA_xy = S_est_A_xy - S_true_xy;
        errB_xy = S_est_B_xy - S_true_xy;

        errA_horiz = sqrt(sum(errA_xy.^2, 2));
        errB_horiz = sqrt(sum(errB_xy.^2, 2));

        allErrIter1{e} = errA_horiz;
        allErrIter2{e} = errB_horiz;

        meanErrIter1(e) = mean(errA_horiz);
        meanErrIter2(e) = mean(errB_horiz);

        maxErrIter1(e) = max(errA_horiz);
        maxErrIter2(e) = max(errB_horiz);

        improveMean(e) = meanErrIter1(e) - meanErrIter2(e);
        improveRate(e) = improveMean(e) / max(meanErrIter1(e), eps) * 100;

        %% 命令行逐组打印

        fprintf('【%s】\n', expNames{e});
        fprintf('  第1轮平均水平误差：%.4f m\n', meanErrIter1(e));
        fprintf('  第2轮平均水平误差：%.4f m\n', meanErrIter2(e));
        fprintf('  平均改善量：%.4f m\n', improveMean(e));
        fprintf('  平均改善率：%.2f%%\n', improveRate(e));
        fprintf('  第1轮最大水平误差：%.4f m\n', maxErrIter1(e));
        fprintf('  第2轮最大水平误差：%.4f m\n', maxErrIter2(e));

        fprintf('  逐潜标水平误差：\n');
        for i = 1:n
            fprintf('    潜标%d：%.4f m -> %.4f m，改善 %.4f m\n', ...
                i, errA_horiz(i), errB_horiz(i), errA_horiz(i) - errB_horiz(i));
        end

        fprintf('\n');

    end

    fprintf('==================== 对比完成 ====================\n\n');

    %% ==================== 4. 图1：三组实验平均水平误差对比 ====================

    % figure('Name', '三组实验两次迭代平均水平误差对比', 'Color', [1 1 1]);
    myfigurestartup(3,3,'zxy');

    b = bar([meanErrIter1, meanErrIter2]);
    grid on;

    set(gca, 'XTickLabel', expNames);
    ylabel('平均水平位置误差 (m)');
    legend('第1轮', '第2轮');
    title('三组实验两次迭代平均水平误差对比');

    % 柱状图标数字
    for k = 1:numel(b)
        x = b(k).XEndPoints;
        y = b(k).YEndPoints;
        labels = string(round(b(k).YData, 3));

        text(x, y, labels, ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'bottom', ...
            'FontSize', 9);
    end

    ymax = max([meanErrIter1; meanErrIter2]);
    if ymax <= 0
        ymax = 1;
    end
    ylim([0, ymax * 1.20]);

    % sgtitle('Circle / Line / Square 三组实验二次迭代效果对比');

    exportgraphics(gcf, fullfile(baseFolders{1}, '三组实验_平均水平误差对比.png'), 'Resolution', 600);

    %% ==================== 5. 图2：三组实验改善率对比 ====================

    % figure('Name', '三组实验二次迭代改善率对比', 'Color', [1 1 1]);
    myfigurestartup(3,3,'zxy');
    b2 = bar(improveRate);
    grid on;

    set(gca, 'XTickLabel', expNames);
    ylabel('平均水平误差改善率 (%)');
    title('三组实验二次迭代改善率对比');

    x = b2.XEndPoints;
    y = b2.YEndPoints;
    labels = string(round(b2.YData, 2)) + "%";

    text(x, y, labels, ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'bottom', ...
        'FontSize', 9);

    ymax = max(improveRate);
    ymin = min(improveRate);

    if ymax <= 0
        ylim([ymin * 1.20, 1]);
    else
        ylim([min(0, ymin * 1.20), ymax * 1.20]);
    end

    exportgraphics(gcf, fullfile(baseFolders{1}, '三组实验_平均水平误差改善率对比.png'), 'Resolution', 600);

    %% ==================== 6. 图3：逐潜标水平误差对比 ====================

    % figure('Name', '三组实验逐潜标水平误差对比', 'Color', [1 1 1]);
    myfigurestartup(8,3,'zxy');
    for e = 1:numExp

        subplot(1, numExp, e);

        err1 = allErrIter1{e};
        err2 = allErrIter2{e};
        n = numel(err1);

        b3 = bar(1:n, [err1, err2]);
        grid on;

        xlabel('潜标编号');
        ylabel('水平位置误差 (m)');
        title(expNames{e});
        legend('第1轮', '第2轮', 'Location', 'best');

        for k = 1:numel(b3)
            x = b3(k).XEndPoints;
            y = b3(k).YEndPoints;
            labels = string(round(b3(k).YData, 3));

            text(x, y, labels, ...
                'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'bottom', ...
                'FontSize', 8);
        end

        ymax = max([err1; err2]);
        if ymax <= 0
            ymax = 1;
        end
        ylim([0, ymax * 1.20]);

    end

    sgtitle('三组实验逐潜标水平误差：第1轮 vs 第2轮');

    exportgraphics(gcf, fullfile(baseFolders{1}, '三组实验_逐潜标水平误差对比.png'), 'Resolution', 600);

