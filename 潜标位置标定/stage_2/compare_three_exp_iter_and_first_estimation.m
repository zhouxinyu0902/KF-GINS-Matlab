close all hidden
clear; clc;

%% ========================================================================
%  功能：
%    对比三组实验的两次迭代标定效果，并额外分析第一轮估计能力。
%
%  对比对象：
%    Circle_Beacon1_Trj
%    Line_Beacon1_Trj
%    Square_Beacon1_Trj
%
%  第一部分：两轮迭代效果
%    1. 第1轮平均水平误差
%    2. 第2轮平均水平误差
%    3. 平均水平误差改善量
%    4. 平均水平误差改善率
%    5. 每个潜标的水平误差变化
%
%  第二部分：第一轮估计能力
%    1. 第一轮 theta / phi 角度估计误差
%    2. 第一轮位移偏差估计误差
%
%  说明：
%    不比较 Z 轴。
%    位置部分只比较 S_est_xyz 与 S_true_xyz 的水平误差。
% ========================================================================

%% ==================== 1. 三组实验路径配置 ====================

baseFolders = cell(3,1);

baseFolders{1} = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Circle_Beacon1_Trj\';
baseFolders{2} = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Line_Beacon1_Trj\';
baseFolders{3} = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Square_Beacon1_Trj\';

% 如果你想换成 ArrayCenter，就改成下面这组：
% baseFolders{1} = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Circle_ArrayCenter_Trj\';
% baseFolders{2} = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Line_ArrayCenter_Trj\';
% baseFolders{3} = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Square_ArrayCenter_Trj\';

expNames = { ...
    'Circle', ...
    'Line', ...
    'Square' ...
};
expNames = { ...
    '圆形轨迹', ...
    '直线往返轨迹', ...
    '方形轨迹' ...
};

iterA = 1;
iterB = 2;

numExp = numel(baseFolders);

saveFig = true;
figDpi  = 600;

saveFolder = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\fig';

%% ==================== 2. 结果预分配 ====================

meanErrIter1 = zeros(numExp, 1);
meanErrIter2 = zeros(numExp, 1);

maxErrIter1 = zeros(numExp, 1);
maxErrIter2 = zeros(numExp, 1);

improveMean = zeros(numExp, 1);
improveRate = zeros(numExp, 1);

allErrIter1 = cell(numExp, 1);
allErrIter2 = cell(numExp, 1);

thetaEstIter1 = zeros(numExp, 1);
phiEstIter1   = zeros(numExp, 1);

thetaRefIter1 = zeros(numExp, 1);
phiRefIter1   = zeros(numExp, 1);

thetaErrIter1 = zeros(numExp, 1);
phiErrIter1   = zeros(numExp, 1);

meanBiasErrIter1 = zeros(numExp, 1);
maxBiasErrIter1  = zeros(numExp, 1);

allBiasErrIter1 = cell(numExp, 1);

allDxTrue = cell(numExp, 1);
allDyTrue = cell(numExp, 1);
allDxEst  = cell(numExp, 1);
allDyEst  = cell(numExp, 1);

%% ==================== 3. 循环读取三组实验 ====================

fprintf('\n==================== 三组实验两次迭代效果对比 ====================\n');
fprintf('对比方式：只比较 S_est_xyz 与 S_true_xyz 的水平误差，不比较 Z 轴。\n');
fprintf('额外分析：第一轮角度估计误差 + 第一轮位移偏差估计误差。\n\n');

for e = 1:numExp

    baseFolder = baseFolders{e};

    pathpos = fullfile(baseFolder, 'input_stage2', 'beacon_pos.mat');

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

    if ~isfield(mapData, 'S_gnss_xyz')
        error('%s 中缺少 S_gnss_xyz，无法计算第一轮位移偏差估计误差。', pathpos);
    end

    if ~isfield(dataA, 'S_est_xyz')
        error('%s 中缺少 S_est_xyz。', fileA);
    end

    if ~isfield(dataB, 'S_est_xyz')
        error('%s 中缺少 S_est_xyz。', fileB);
    end

    S_true_xyz = mapData.S_true_xyz;
    S_gnss_xyz = mapData.S_gnss_xyz;

    S_est_A = dataA.S_est_xyz;
    S_est_B = dataB.S_est_xyz;

    n = size(S_true_xyz, 1);

    S_true_xy = S_true_xyz(1:n, 1:2);
    S_est_A_xy = S_est_A(1:n, 1:2);
    S_est_B_xy = S_est_B(1:n, 1:2);
    S_gnss_xy  = S_gnss_xyz(1:n, 1:2);

    %% ==================== 3.1 两轮水平位置误差 ====================

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

    %% ==================== 3.2 第一轮角度估计误差 ====================

    if ~isfield(dataA, 'theta_est') || ~isfield(dataA, 'phi_est')
        error('%s 中缺少 theta_est 或 phi_est。', fileA);
    end

    thetaEstIter1(e) = local_rad_to_deg_scalar(dataA.theta_est);
    phiEstIter1(e)   = local_rad_to_deg_scalar(dataA.phi_est);

    if isfield(dataA, 'theta_ref_deg')
        thetaRefIter1(e) = local_deg_scalar(dataA.theta_ref_deg);
    elseif isfield(mapData, 'theta_true')
        thetaRefIter1(e) = local_rad_to_deg_scalar(mapData.theta_true);
    else
        thetaRefIter1(e) = 20;
    end

    if isfield(dataA, 'phi_ref_deg')
        phiRefIter1(e) = local_deg_scalar(dataA.phi_ref_deg);
    elseif isfield(mapData, 'phi_true')
        phiRefIter1(e) = local_rad_to_deg_scalar(mapData.phi_true);
    else
        phiRefIter1(e) = 45;
    end

    thetaErrIter1(e) = thetaEstIter1(e) - thetaRefIter1(e);
    phiErrIter1(e)   = local_angle_diff_deg(phiEstIter1(e), phiRefIter1(e));

    %% ==================== 3.3 第一轮位移偏差估计误差 ====================

    trueBias_xy = S_true_xy - S_gnss_xy;
    estBias_xy  = S_est_A_xy - S_gnss_xy;

    biasErr_xy = estBias_xy - trueBias_xy;

    biasErrIter1 = sqrt(sum(biasErr_xy.^2, 2));

    meanBiasErrIter1(e) = mean(biasErrIter1);
    maxBiasErrIter1(e)  = max(biasErrIter1);

    allBiasErrIter1{e} = biasErrIter1;

    allDxTrue{e} = trueBias_xy(:,1);
    allDyTrue{e} = trueBias_xy(:,2);
    allDxEst{e}  = estBias_xy(:,1);
    allDyEst{e}  = estBias_xy(:,2);

    %% ==================== 3.4 命令行逐组打印 ====================

    fprintf('【%s】\n', expNames{e});

    fprintf('  第一轮角度估计：\n');
    fprintf('    theta_ref = %.4f°，theta_est = %.4f°，theta_err = %.4f°\n', ...
        thetaRefIter1(e), thetaEstIter1(e), thetaErrIter1(e));
    fprintf('    phi_ref   = %.4f°，phi_est   = %.4f°，phi_err   = %.4f°\n', ...
        phiRefIter1(e), phiEstIter1(e), phiErrIter1(e));

    fprintf('  第一轮位移偏差估计：\n');
    fprintf('    平均位移偏差估计误差：%.4f m\n', meanBiasErrIter1(e));
    fprintf('    最大位移偏差估计误差：%.4f m\n', maxBiasErrIter1(e));

    fprintf('  两轮水平位置误差：\n');
    fprintf('    第1轮平均水平误差：%.4f m\n', meanErrIter1(e));
    fprintf('    第2轮平均水平误差：%.4f m\n', meanErrIter2(e));
    fprintf('    平均改善量：%.4f m\n', improveMean(e));
    fprintf('    平均改善率：%.2f%%\n', improveRate(e));
    fprintf('    第1轮最大水平误差：%.4f m\n', maxErrIter1(e));
    fprintf('    第2轮最大水平误差：%.4f m\n', maxErrIter2(e));

    fprintf('  逐潜标水平误差：\n');
    for i = 1:n
        fprintf('    潜标%d：%.4f m -> %.4f m，改善 %.4f m\n', ...
            i, errA_horiz(i), errB_horiz(i), errA_horiz(i) - errB_horiz(i));
    end

    fprintf('\n');

end

fprintf('==================== 对比完成 ====================\n\n');

%% ==================== 4. 图1：平均水平误差 + 改善率 ====================

myfigurestartup(7,3,'zxy');

subplot(1,2,1);

b = bar([meanErrIter1, meanErrIter2]);
local_set_bar_gray(b, 'iter');

grid on;
set(gca, 'XTickLabel', expNames);
ylabel('平均水平位置误差 (m)');
legend('第1轮', '第2轮');
title('两次迭代平均水平误差对比');

local_add_bar_labels(b, 3, 9);

ymax = max([meanErrIter1; meanErrIter2]);
if ymax <= 0
    ymax = 1;
end
ylim([0, ymax * 1.20]);

subplot(1,2,2);

b2 = bar(improveRate);
local_set_bar_gray(b2, 'single');

grid on;
set(gca, 'XTickLabel', expNames);
ylabel('平均水平误差改善率 (%)');
title('二次迭代改善率对比');

local_add_bar_percent_labels(b2, 2, 9);

ymax = max(improveRate);
ymin = min(improveRate);

if ymax <= 0
    ylim([ymin * 1.20, 1]);
else
    ylim([min(0, ymin * 1.20), ymax * 1.20]);
end

if saveFig
    exportgraphics(gcf, ...
        fullfile(saveFolder, '三组实验_平均水平误差改善率对比+平均水平误差对比_BEA.png'), ...
        'Resolution', figDpi);
end

%% ==================== 5. 图2：逐潜标水平误差对比 ====================

myfigurestartup(7,3,'zxy');

for e = 1:numExp

    subplot(1, numExp, e);

    err1 = allErrIter1{e};
    err2 = allErrIter2{e};
    n = numel(err1);

    b3 = bar(1:n, [err1, err2]);
    local_set_bar_gray(b3, 'iter');

    grid on;
    xlabel('潜标编号');
    ylabel('水平位置误差 (m)');
    title(expNames{e});
    legend('第1轮', '第2轮', 'Location', 'best');

    local_add_bar_labels(b3, 3, 8);

    ymax = max([err1; err2]);
    if ymax <= 0
        ymax = 1;
    end
    ylim([0, ymax * 1.20]);

end

sgtitle('三组实验逐潜标水平误差：第1轮 vs 第2轮');

if saveFig
    exportgraphics(gcf, ...
        fullfile(saveFolder, '三组实验_逐潜标水平误差对比_BEA.png'), ...
        'Resolution', figDpi);
end

%% ==================== 6. 图3：第一轮角度误差 + 位移偏差估计误差 ====================

myfigurestartup(7,3,'zxy');

subplot(1,2,1);

b4 = bar([abs(thetaErrIter1), abs(phiErrIter1)]);
local_set_bar_gray(b4, 'angle');

grid on;
set(gca, 'XTickLabel', expNames);
ylabel('角度估计绝对误差 (deg)');
legend('|theta误差|', '|phi误差|');
title('第一轮角度估计误差');

local_add_bar_labels(b4, 3, 9);

ymax = max([abs(thetaErrIter1); abs(phiErrIter1)]);
if ymax <= 0
    ymax = 1;
end
ylim([0, ymax * 1.50]);

subplot(1,2,2);

b5 = bar(meanBiasErrIter1);
local_set_bar_gray(b5, 'single');

grid on;
set(gca, 'XTickLabel', expNames);
ylabel('平均位移偏差估计误差 (m)');
title('第一轮位移偏差估计误差');

local_add_bar_labels(b5, 3, 9);

ymax = max(meanBiasErrIter1);
if ymax <= 0
    ymax = 1;
end
ylim([0, ymax * 1.20]);

% sgtitle('三组实验第一轮估计性能对比');

if saveFig
    exportgraphics(gcf, ...
        fullfile(saveFolder, '三组实验_第一轮角度误差与位移偏差估计误差合并图_BEA.png'), ...
        'Resolution', figDpi);
end

%% ==================== 7. 图4：逐潜标第一轮位移偏差估计误差 ====================

myfigurestartup(7,3,'zxy');

for e = 1:numExp

    subplot(1, numExp, e);

    biasErr = allBiasErrIter1{e};
    n = numel(biasErr);

    b6 = bar(1:n, biasErr);
    local_set_bar_gray(b6, 'single');

    grid on;
    xlabel('潜标编号');
    ylabel('位移偏差估计误差 (m)');
    title(expNames{e});

    local_add_bar_labels(b6, 3, 8);

    ymax = max(biasErr);
    if ymax <= 0
        ymax = 1;
    end
    ylim([0, ymax * 1.20]);

end

sgtitle('三组实验逐潜标第一轮位移偏差估计误差');

if saveFig
    exportgraphics(gcf, ...
        fullfile(saveFolder, '三组实验_逐潜标第一轮位移偏差估计误差_BEA.png'), ...
        'Resolution', figDpi);
end

%% ==================== 8. 图5：第一轮真实位移偏差 vs 估计位移偏差 ====================

figure('Name', '三组实验第一轮真实位移偏差与估计位移偏差对比', ...
    'Color', [1 1 1], ...
    'Position', [100 100 1300 650]);

for e = 1:numExp

    dxTrue = allDxTrue{e};
    dyTrue = allDyTrue{e};
    dxEst  = allDxEst{e};
    dyEst  = allDyEst{e};

    n = numel(dxTrue);

    subplot(2, numExp, e);

    bdx = bar(1:n, [dxTrue, dxEst]);
    local_set_bar_gray(bdx, 'true_est');

    grid on;
    xlabel('潜标编号');
    ylabel('dx / 东向偏差 (m)');
    title([expNames{e}, '：dx偏差']);
    legend('真实dx', '估计dx', 'Location', 'best');

    local_add_bar_labels(bdx, 2, 7);

    subplot(2, numExp, e + numExp);

    bdy = bar(1:n, [dyTrue, dyEst]);
    local_set_bar_gray(bdy, 'true_est');

    grid on;
    xlabel('潜标编号');
    ylabel('dy / 北向偏差 (m)');
    title([expNames{e}, '：dy偏差']);
    legend('真实dy', '估计dy', 'Location', 'best');

    local_add_bar_labels(bdy, 2, 7);

end

sgtitle('第一轮位移偏差估计效果：真实偏差 vs 估计偏差');

if saveFig
    exportgraphics(gcf, ...
        fullfile(saveFolder, '三组实验_第一轮真实位移偏差与估计位移偏差对比_BEA.png'), ...
        'Resolution', figDpi);
end

fprintf('\n所有图像已导出至：%s\n', saveFolder);

%% ==================== 本地函数 ====================

function deg = local_rad_to_deg_scalar(x)
    deg = mean(x(:)) * 180 / pi;
end

function deg = local_deg_scalar(x)
    deg = mean(x(:));
end

function d = local_angle_diff_deg(angleA, angleB)
% 计算两个角度的最小差值，单位 deg，范围 [-180, 180]
    d = mod((angleA - angleB) + 180, 360) - 180;
end

function local_set_bar_gray(b, mode)
% 给柱状图设置明显灰度区分
%
% mode:
%   'iter'      : 第1轮 / 第2轮
%   'angle'     : theta误差 / phi误差
%   'single'    : 单组柱状图
%   'true_est'  : 真实值 / 估计值

    if nargin < 2
        mode = 'iter';
    end

    switch mode
        case 'iter'
            colors = [
                0.18 0.18 0.18;
                0.72 0.72 0.72
            ];

        case 'angle'
            colors = [
                0.15 0.15 0.15;
                0.62 0.62 0.62
            ];

        case 'single'
            colors = [
                0.42 0.42 0.42
            ];

        case 'true_est'
            colors = [
                0.12 0.12 0.12;
                0.78 0.78 0.78
            ];

        otherwise
            colors = [
                0.18 0.18 0.18;
                0.72 0.72 0.72
            ];
    end

    for k = 1:numel(b)
        colorIdx = min(k, size(colors, 1));
        b(k).FaceColor = colors(colorIdx, :);
        b(k).EdgeColor = [0 0 0];
        b(k).LineWidth = 0.8;
    end
end

function local_add_bar_labels(b, ndigits, fontSize)
% 给 bar 图自动标注数值，兼容单组和多组柱状图

    if nargin < 2
        ndigits = 3;
    end
    if nargin < 3
        fontSize = 9;
    end

    for k = 1:numel(b)

        try
            x = b(k).XEndPoints;
            y = b(k).YEndPoints;
        catch
            x = b(k).XData + b(k).XOffset;
            y = b(k).YData;
        end

        yData = b(k).YData;
        labels = string(round(yData, ndigits));

        for i = 1:numel(yData)

            if yData(i) >= 0
                vAlign = 'bottom';
            else
                vAlign = 'top';
            end

            text(x(i), y(i), labels(i), ...
                'HorizontalAlignment', 'center', ...
                'VerticalAlignment', vAlign, ...
                'FontSize', fontSize);
        end
    end
end

function local_add_bar_percent_labels(b, ndigits, fontSize)
% 给百分比 bar 图标注百分号

    if nargin < 2
        ndigits = 2;
    end
    if nargin < 3
        fontSize = 9;
    end

    try
        x = b.XEndPoints;
        y = b.YEndPoints;
    catch
        x = b.XData + b.XOffset;
        y = b.YData;
    end

    yData = b.YData;
    labels = string(round(yData, ndigits)) + "%";

    for i = 1:numel(yData)

        if yData(i) >= 0
            vAlign = 'bottom';
        else
            vAlign = 'top';
        end

        text(x(i), y(i), labels(i), ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', vAlign, ...
            'FontSize', fontSize);
    end
end