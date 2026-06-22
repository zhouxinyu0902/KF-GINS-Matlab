function compare_two_iter_position(baseFolder, pathpos, iterA, iterB)
% -------------------------------------------------------------------------
% 功能：
%   直接比较两轮 S_est_xyz 与 S_true_xyz 的水平位置误差。
%
% 对比逻辑：
%   第 iterA 轮：S_est_xyz_A(:,1:2) - S_true_xyz(:,1:2)
%   第 iterB 轮：S_est_xyz_B(:,1:2) - S_true_xyz(:,1:2)
%
% 只比较水平位置，不比较 Z 轴，不比较角度。
% -------------------------------------------------------------------------

    %% ==================== 1. 加载两轮结果 ====================

    fileA = fullfile(baseFolder, sprintf('iter_%d_next_ref.mat', iterA));
    fileB = fullfile(baseFolder, sprintf('iter_%d_next_ref.mat', iterB));

    if ~exist(fileA, 'file')
        error('找不到第 %d 轮结果文件：%s', iterA, fileA);
    end

    if ~exist(fileB, 'file')
        error('找不到第 %d 轮结果文件：%s', iterB, fileB);
    end

    dataA = load(fileA);
    dataB = load(fileB);

    if ~isfield(dataA, 'S_est_xyz')
        error('第 %d 轮结果文件中缺少 S_est_xyz。', iterA);
    end

    if ~isfield(dataB, 'S_est_xyz')
        error('第 %d 轮结果文件中缺少 S_est_xyz。', iterB);
    end

    mapData = load(pathpos);

    if ~isfield(mapData, 'S_true_xyz')
        error('beacon_pos.mat 中缺少 S_true_xyz。');
    end

    S_true_xyz = mapData.S_true_xyz;
    S_est_A = dataA.S_est_xyz;
    S_est_B = dataB.S_est_xyz;

    %% ==================== 2. 尺寸检查 ====================

    n = size(S_true_xyz, 1);

    if size(S_est_A, 1) < n || size(S_est_A, 2) ~= 3
        error('第 %d 轮 S_est_xyz 尺寸错误，应至少为 %d x 3。', iterA, n);
    end

    if size(S_est_B, 1) < n || size(S_est_B, 2) ~= 3
        error('第 %d 轮 S_est_xyz 尺寸错误，应至少为 %d x 3。', iterB, n);
    end

    S_est_A = S_est_A(1:n, :);
    S_est_B = S_est_B(1:n, :);
    S_true_xyz = S_true_xyz(1:n, :);

    %% ==================== 3. 只计算水平位置误差 ====================

    errA_xy = S_est_A(:,1:2) - S_true_xyz(:,1:2);
    errB_xy = S_est_B(:,1:2) - S_true_xyz(:,1:2);

    errA_x = errA_xy(:,1);
    errA_y = errA_xy(:,2);

    errB_x = errB_xy(:,1);
    errB_y = errB_xy(:,2);

    errA_horiz = sqrt(sum(errA_xy.^2, 2));
    errB_horiz = sqrt(sum(errB_xy.^2, 2));

    improve_horiz = errA_horiz - errB_horiz;
    improveRate_horiz = improve_horiz ./ max(errA_horiz, eps) * 100;

    %% ==================== 4. 命令行打印 ====================

    fprintf('\n==================== 第 %d 轮 vs 第 %d 轮水平位置误差对比 ====================\n', iterA, iterB);
    fprintf('对比方式：只比较 S_est_xyz 与 S_true_xyz 的 X/Y 水平误差，不比较 Z 轴。\n\n');

    fprintf('潜标编号 | 第%d轮X误差(m) | 第%d轮Y误差(m) | 第%d轮水平误差(m) || 第%d轮X误差(m) | 第%d轮Y误差(m) | 第%d轮水平误差(m) || 水平改善(m) | 改善率(%%)\n', ...
        iterA, iterA, iterA, ...
        iterB, iterB, iterB);

    fprintf('---------------------------------------------------------------------------------------------------------------------------------\n');

    for i = 1:n
        fprintf('   %d     | %12.4f | %12.4f | %14.4f || %12.4f | %12.4f | %14.4f || %11.4f | %9.2f\n', ...
            i, ...
            errA_x(i), errA_y(i), errA_horiz(i), ...
            errB_x(i), errB_y(i), errB_horiz(i), ...
            improve_horiz(i), improveRate_horiz(i));
    end

    fprintf('\n【总体统计】\n');

    fprintf('平均水平误差：第%d轮 %.4f m -> 第%d轮 %.4f m，改善 %.4f m，改善率 %.2f%%\n', ...
        iterA, mean(errA_horiz), ...
        iterB, mean(errB_horiz), ...
        mean(errA_horiz) - mean(errB_horiz), ...
        (mean(errA_horiz) - mean(errB_horiz)) / max(mean(errA_horiz), eps) * 100);

    fprintf('最大水平误差：第%d轮 %.4f m -> 第%d轮 %.4f m，改善 %.4f m\n', ...
        iterA, max(errA_horiz), ...
        iterB, max(errB_horiz), ...
        max(errA_horiz) - max(errB_horiz));

    fprintf('最小水平误差：第%d轮 %.4f m -> 第%d轮 %.4f m\n', ...
        iterA, min(errA_horiz), ...
        iterB, min(errB_horiz));

    fprintf('================================================================================\n\n');

    %% ==================== 5. 图1：水平误差对比 ====================

    % figure('Name', sprintf('第%d轮与第%d轮水平位置误差对比', iterA, iterB), ...
    %     'Color', [1 1 1]);
    myfigurestartup(4,3,'zxy');
    b = bar(1:n, [errA_horiz, errB_horiz]);
    grid on;
    xlabel('潜标编号');
    ylabel('水平位置误差 (m)');
    legend(sprintf('第%d轮', iterA), sprintf('第%d轮', iterB), 'Location', 'best');
    title('水平位置误差对比');

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

    ymax = max([errA_horiz; errB_horiz]);
    if ymax <= 0
        ymax = 1;
    end
    ylim([0, ymax * 1.15]);

    % sgtitle(sprintf('第%d轮 vs 第%d轮：S\\_est\\_xyz 与 S\\_true\\_xyz 水平误差对比', iterA, iterB));

    %% ==================== 6. 图2：X/Y分量误差对比 ====================

    % figure('Name', sprintf('第%d轮与第%d轮 XY分量误差对比', iterA, iterB), ...
    %     'Color', [1 1 1]);
    % 
    % subplot(1,2,1);
    % b1 = bar(1:n, [errA_x, errB_x]);
    % grid on;
    % xlabel('潜标编号');
    % ylabel('X误差 / 东向误差 (m)');
    % legend(sprintf('第%d轮', iterA), sprintf('第%d轮', iterB), 'Location', 'best');
    % title('X方向误差对比');
    % 
    % for k = 1:numel(b1)
    %     x = b1(k).XEndPoints;
    %     y = b1(k).YEndPoints;
    %     labels = string(round(b1(k).YData, 3));
    % 
    %     text(x, y, labels, ...
    %         'HorizontalAlignment', 'center', ...
    %         'VerticalAlignment', 'bottom', ...
    %         'FontSize', 9);
    % end
    % 
    % subplot(1,2,2);
    % b2 = bar(1:n, [errA_y, errB_y]);
    % grid on;
    % xlabel('潜标编号');
    % ylabel('Y误差 / 北向误差 (m)');
    % legend(sprintf('第%d轮', iterA), sprintf('第%d轮', iterB), 'Location', 'best');
    % title('Y方向误差对比');
    % 
    % for k = 1:numel(b2)
    %     x = b2(k).XEndPoints;
    %     y = b2(k).YEndPoints;
    %     labels = string(round(b2(k).YData, 3));
    % 
    %     text(x, y, labels, ...
    %         'HorizontalAlignment', 'center', ...
    %         'VerticalAlignment', 'bottom', ...
    %         'FontSize', 9);
    % end
    % 
    % sgtitle('两轮 XY 分量误差对比');

end