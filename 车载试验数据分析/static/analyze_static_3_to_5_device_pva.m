clear;
clc;
close all;

%% static-3 ~ static-5 设备解算与重新对准纯惯导结果分析
% 说明：
% 1. 三批数据均为同一静止位置，统一使用三份 pva_initial 初始位置的平均值作为参考位置；
% 2. 分析设备输出 pva_file.txt，同时分析重新对准脚本生成的两次纯惯导：
%    - 使用原 pva_initial 姿态；
%    - 使用数学粗对准 + Kalman 精对准后的新姿态；
% 3. 默认评价各批连续有效数据的前 3600 s；
% 4. 每批单次试验只输出 RMS、Max、终点 RER 和 Median；
% 5. RER = 评价终点水平径向误差(n mile) / 累计运行时间(h)；
% 6. 本脚本不计算R50。R50应由多次独立试验各自的RER另行统计。

%% 1. 路径与参数
scriptDirectory = fileparts(mfilename('fullpath'));
analysisDirectory = fileparts(scriptDirectory);
repositoryDirectory = fileparts(analysisDirectory);

addpath(genpath(fullfile(repositoryDirectory, 'function')));

dataRoot = fullfile(repositoryDirectory, 'data', 'experiment-data');
caseNumbers = 3:5;
evaluationDurationSeconds = 3600;

% 必须与 run_static_coarse_fine_alignment.m 中的约束模式对应。
% 当前已有结果采用“零垂向速度 + 固定高度”。如重新生成了仅固定高度结果，
% 将此处改为 "FixedHeight" 即可。
pureInsConstraintName = "ZeroVelFixedHeight";

% 单次试验终点RER计算参数
meterPerNauticalMile = 1852;
secondsPerHour = 3600;

% 跨数据集的汇总结果单独存放，不写回各批 input/output 目录。
outputDirectory = fullfile(dataRoot, 'static-3-to-5-summary');
if ~exist(outputDirectory, 'dir')
    mkdir(outputDirectory);
end

%% 2. 读取三批初始位置，得到统一参考位置
initialPositionDegM = nan(numel(caseNumbers), 3);
for index = 1:numel(caseNumbers)
    caseNumber = caseNumbers(index);
    initialFile = fullfile(dataRoot, sprintf('static-%d', caseNumber), ...
        'input', 'pva_initial.txt');
    if ~isfile(initialFile)
        error('找不到 static-%d 的初始 PVA：%s', caseNumber, initialFile);
    end

    initialPva = readmatrix(initialFile, 'FileType', 'text');
    if isempty(initialPva) || size(initialPva, 2) < 5
        error('static-%d 的 pva_initial.txt 格式不正确。', caseNumber);
    end
    initialPositionDegM(index, :) = initialPva(1, 3:5);
end

% [纬度/deg, 经度/deg, 高度/m]
commonReferencePositionDegM = mean(initialPositionDegM, 1, 'omitnan');
referenceLatitudeRad = deg2rad(commonReferencePositionDegM(1));
referenceHeightM = commonReferencePositionDegM(3);

param = Param();
[referenceRm, referenceRn] = getRmRn(referenceLatitudeRad, param);

fprintf('\n============================================================\n');
fprintf('static-3 ~ static-5 设备 PVA 静态位置分析\n');
fprintf('============================================================\n');
fprintf('统一参考纬度：%.12f deg\n', commonReferencePositionDegM(1));
fprintf('统一参考经度：%.12f deg\n', commonReferencePositionDegM(2));
fprintf('统一参考高度：%.4f m\n', commonReferencePositionDegM(3));
fprintf('评价时长上限：%.0f s\n', evaluationDurationSeconds);
fprintf('单次试验指标：RMS、Max、终点RER、Median（不计算R50）\n');
fprintf('============================================================\n\n');

%% 3. 分析每批设备 PVA
resultTemplate = struct( ...
    'caseNumber', NaN, ...
    'timeSeconds', [], ...
    'northErrorM', [], ...
    'eastErrorM', [], ...
    'downErrorM', [], ...
    'radialErrorM', [], ...
    'initialNorthOffsetM', NaN, ...
    'initialEastOffsetM', NaN, ...
    'availableDurationSeconds', NaN, ...
    'evaluatedDurationSeconds', NaN, ...
    'sampleCount', 0, ...
    'radialRmsM', NaN, ...
    'radialMaxM', NaN, ...
    'radialMedianM', NaN, ...
    'endpointRerNmiPerHour', NaN);
results = repmat(resultTemplate, numel(caseNumbers), 1);

for index = 1:numel(caseNumbers)
    caseNumber = caseNumbers(index);
    pvaFile = fullfile(dataRoot, sprintf('static-%d', caseNumber), ...
        'input', 'pva_file.txt');
    if ~isfile(pvaFile)
        error('找不到 static-%d 的设备 PVA：%s', caseNumber, pvaFile);
    end

    fprintf('正在读取 static-%d：%s\n', caseNumber, pvaFile);
    pva = readmatrix(pvaFile, 'FileType', 'text');
    if isempty(pva) || size(pva, 2) < 11
        error('static-%d 的 pva_file.txt 应至少包含 11 列。', caseNumber);
    end

    % 只保留时间、位置均有效的数据，再按导航时间排序、去重。
    validRow = all(isfinite(pva(:, 2:5)), 2);
    pva = pva(validRow, :);
    pva = sortrows(pva, 2);
    [~, uniqueIndex] = unique(pva(:, 2), 'stable');
    pva = pva(uniqueIndex, :);
    if size(pva, 1) < 2
        error('static-%d 删除无效行后数据不足。', caseNumber);
    end

    % 防止异常时间跳变把不相邻的数据拼成一段；只采用最长连续段。
    pva = select_longest_continuous_pva_segment(pva, caseNumber);
    elapsedAll = pva(:, 2) - pva(1, 2);
    availableDuration = elapsedAll(end);
    evaluationEnd = min(evaluationDurationSeconds, availableDuration);
    evaluationMask = elapsedAll >= 0 & elapsedAll <= evaluationEnd;
    pvaEvaluation = pva(evaluationMask, :);
    elapsedSeconds = elapsedAll(evaluationMask);

    latitudeDifferenceRad = deg2rad(pvaEvaluation(:, 3) - commonReferencePositionDegM(1));
    longitudeDifferenceRad = deg2rad(pvaEvaluation(:, 4) - commonReferencePositionDegM(2));
    northErrorM = latitudeDifferenceRad * (referenceRm + referenceHeightM);
    eastErrorM = longitudeDifferenceRad * ...
        (referenceRn + referenceHeightM) * cos(referenceLatitudeRad);
    downErrorM = -(pvaEvaluation(:, 5) - commonReferencePositionDegM(3));
    radialErrorM = hypot(northErrorM, eastErrorM);

    validError = isfinite(elapsedSeconds) & isfinite(radialErrorM);
    elapsedSeconds = elapsedSeconds(validError);
    northErrorM = northErrorM(validError);
    eastErrorM = eastErrorM(validError);
    downErrorM = downErrorM(validError);
    radialErrorM = radialErrorM(validError);
    if numel(radialErrorM) < 2
        error('static-%d 没有足够的有效位置误差数据。', caseNumber);
    end

    % 单次试验只取评价终点的一个RER，不把各时间历元当成独立试验。
    endpointTimeSeconds = elapsedSeconds(end);
    endpointRadialErrorM = radialErrorM(end);
    if endpointTimeSeconds > 0
        endpointRerNmiPerHour = ...
            (endpointRadialErrorM / meterPerNauticalMile) / ...
            (endpointTimeSeconds / secondsPerHour);
    else
        endpointRerNmiPerHour = NaN;
    end

    % 初始点相对于统一参考位置的偏差，用于确认三批初始位置确实相同。
    initialLatitudeDifferenceRad = deg2rad(initialPositionDegM(index, 1) - ...
        commonReferencePositionDegM(1));
    initialLongitudeDifferenceRad = deg2rad(initialPositionDegM(index, 2) - ...
        commonReferencePositionDegM(2));

    results(index).caseNumber = caseNumber;
    results(index).timeSeconds = elapsedSeconds;
    results(index).northErrorM = northErrorM;
    results(index).eastErrorM = eastErrorM;
    results(index).downErrorM = downErrorM;
    results(index).radialErrorM = radialErrorM;
    results(index).initialNorthOffsetM = initialLatitudeDifferenceRad * ...
        (referenceRm + referenceHeightM);
    results(index).initialEastOffsetM = initialLongitudeDifferenceRad * ...
        (referenceRn + referenceHeightM) * cos(referenceLatitudeRad);
    results(index).availableDurationSeconds = availableDuration;
    results(index).evaluatedDurationSeconds = elapsedSeconds(end);
    results(index).sampleCount = numel(radialErrorM);
    results(index).radialRmsM = sqrt(mean(radialErrorM .^ 2));
    results(index).radialMaxM = max(radialErrorM);
    results(index).radialMedianM = median(radialErrorM);
    results(index).endpointRerNmiPerHour = endpointRerNmiPerHour;

    fprintf(['static-%d：有效 %.2f s，评价 %.2f s，RMS %.3f m，' ...
        'Max %.3f m，RER %.6f n mile/h，Median %.3f m\n'], ...
        caseNumber, availableDuration, elapsedSeconds(end), ...
        results(index).radialRmsM, results(index).radialMaxM, ...
        endpointRerNmiPerHour, results(index).radialMedianM);
end

%% 4. 分析重新对准前后的纯惯导结果
% 这里使用 run_static_coarse_fine_alignment.m 中的已知装订位置。
% 两次纯惯导只有初始姿态不同，初始位置、初始速度、IMU数据和约束方式相同，
% 因此二者的误差差异可以直接反映重新对准姿态对后续推算的影响。
alignmentReferencePositionDegM = [ ...
    36.36533670805072, ...     % 纬度/deg
    120.68946110621751, ...    % 经度/deg
    5];                        % 高度/m

pureInsSolutionKeys = ["PvaInitialAtt", "AlignedAtt"];
pureInsSolutionLabels = ["原pva_initial姿态", "重新对准姿态"];
pureInsResults = cell(numel(caseNumbers), numel(pureInsSolutionKeys));

fprintf('\n============================================================\n');
fprintf('重新对准前后纯惯导分析\n');
fprintf('装订纬度：%.12f deg\n', alignmentReferencePositionDegM(1));
fprintf('装订经度：%.12f deg\n', alignmentReferencePositionDegM(2));
fprintf('装订高度：%.3f m\n', alignmentReferencePositionDegM(3));
fprintf('约束模式：%s\n', pureInsConstraintName);
fprintf('============================================================\n\n');

for index = 1:numel(caseNumbers)
    caseNumber = caseNumbers(index);
    navigationResultDirectory = fullfile(dataRoot, ...
        sprintf('static-%d', caseNumber), 'output', 'navigation-results');

    for solutionIndex = 1:numel(pureInsSolutionKeys)
        solutionKey = pureInsSolutionKeys(solutionIndex);
        solutionLabel = pureInsSolutionLabels(solutionIndex);
        navigationFile = fullfile(navigationResultDirectory, sprintf( ...
            'PureIns-%s-%s.nav', solutionKey, pureInsConstraintName));
        if ~isfile(navigationFile)
            error(['找不到 static-%d 的“%s”纯惯导结果：\n%s\n' ...
                '请先运行 run_static_coarse_fine_alignment.m 生成该结果。'], ...
                caseNumber, solutionLabel, navigationFile);
        end

        fprintf('正在分析 static-%d，%s：\n%s\n', ...
            caseNumber, solutionLabel, navigationFile);
        solutionResult = analyze_navigation_position_file( ...
            navigationFile, alignmentReferencePositionDegM, ...
            evaluationDurationSeconds, meterPerNauticalMile, ...
            secondsPerHour, caseNumber);
        solutionResult.caseNumber = caseNumber;
        solutionResult.solutionKey = solutionKey;
        solutionResult.solutionLabel = solutionLabel;
        solutionResult.sourceFile = string(navigationFile);
        pureInsResults{index, solutionIndex} = solutionResult;

        fprintf(['  有效 %.2f s，评价 %.2f s，RMS %.3f m，' ...
            'Max %.3f m，RER %.6f n mile/h，Median %.3f m\n'], ...
            solutionResult.availableDurationSeconds, ...
            solutionResult.evaluatedDurationSeconds, ...
            solutionResult.radialRmsM, ...
            solutionResult.radialMaxM, ...
            solutionResult.endpointRerNmiPerHour, ...
            solutionResult.radialMedianM);
    end
end

%% 5. 汇总表
caseColumn = reshape([results.caseNumber], [], 1);
radialRmsColumn = reshape([results.radialRmsM], [], 1);
radialMaxColumn = reshape([results.radialMaxM], [], 1);
endpointRerColumn = reshape([results.endpointRerNmiPerHour], [], 1);
radialMedianColumn = reshape([results.radialMedianM], [], 1);

summaryTable = table(caseColumn, radialRmsColumn, radialMaxColumn, ...
    endpointRerColumn, radialMedianColumn, ...
    'VariableNames', {'Case', 'RMS_m', 'Max_m', ...
    'RER_nmi_per_h', 'Median_m'});

referenceTable = table(commonReferencePositionDegM(1), ...
    commonReferencePositionDegM(2), commonReferencePositionDegM(3), ...
    'VariableNames', {'Latitude_deg', 'Longitude_deg', 'Height_m'});

% 重新对准前后纯惯导的长表：每个数据集两行，便于跨数据集筛选和绘图。
pureInsRowCount = numel(caseNumbers) * numel(pureInsSolutionKeys);
pureInsCaseColumn = zeros(pureInsRowCount, 1);
pureInsSolutionColumn = strings(pureInsRowCount, 1);
pureInsRmsColumn = zeros(pureInsRowCount, 1);
pureInsMaxColumn = zeros(pureInsRowCount, 1);
pureInsRerColumn = zeros(pureInsRowCount, 1);
pureInsMedianColumn = zeros(pureInsRowCount, 1);

rowIndex = 0;
for index = 1:numel(caseNumbers)
    for solutionIndex = 1:numel(pureInsSolutionKeys)
        rowIndex = rowIndex + 1;
        thisResult = pureInsResults{index, solutionIndex};
        pureInsCaseColumn(rowIndex) = thisResult.caseNumber;
        pureInsSolutionColumn(rowIndex) = thisResult.solutionLabel;
        pureInsRmsColumn(rowIndex) = thisResult.radialRmsM;
        pureInsMaxColumn(rowIndex) = thisResult.radialMaxM;
        pureInsRerColumn(rowIndex) = thisResult.endpointRerNmiPerHour;
        pureInsMedianColumn(rowIndex) = thisResult.radialMedianM;
    end
end

pureInsSummaryTable = table(pureInsCaseColumn, pureInsSolutionColumn, ...
    pureInsRmsColumn, pureInsMaxColumn, pureInsRerColumn, ...
    pureInsMedianColumn, ...
    'VariableNames', {'Case', 'InitialAttitude', 'RMS_m', 'Max_m', ...
    'RER_nmi_per_h', 'Median_m'});

% 正值表示重新对准改善，负值表示重新对准后反而变差。
initialRmsColumn = zeros(numel(caseNumbers), 1);
alignedRmsColumn = zeros(numel(caseNumbers), 1);
initialRerColumn = zeros(numel(caseNumbers), 1);
alignedRerColumn = zeros(numel(caseNumbers), 1);
for index = 1:numel(caseNumbers)
    initialRmsColumn(index) = pureInsResults{index, 1}.radialRmsM;
    alignedRmsColumn(index) = pureInsResults{index, 2}.radialRmsM;
    initialRerColumn(index) = pureInsResults{index, 1}.endpointRerNmiPerHour;
    alignedRerColumn(index) = pureInsResults{index, 2}.endpointRerNmiPerHour;
end
rmsImprovementPercent = (initialRmsColumn - alignedRmsColumn) ./ ...
    initialRmsColumn * 100;
rerImprovementPercent = (initialRerColumn - alignedRerColumn) ./ ...
    initialRerColumn * 100;

pureInsImprovementTable = table(caseNumbers(:), initialRmsColumn, ...
    alignedRmsColumn, rmsImprovementPercent, initialRerColumn, ...
    alignedRerColumn, rerImprovementPercent, ...
    'VariableNames', {'Case', 'PvaInitialAtt_RMS_m', 'AlignedAtt_RMS_m', ...
    'RMS_Improvement_percent', 'PvaInitialAtt_RER_nmi_per_h', ...
    'AlignedAtt_RER_nmi_per_h', 'RER_Improvement_percent'});

disp(summaryTable);
fprintf('\n重新对准前后纯惯导汇总：\n');
disp(pureInsSummaryTable);
fprintf('\n重新对准改善量（正数=改善，负数=恶化）：\n');
disp(pureInsImprovementTable);

writetable(summaryTable, fullfile(outputDirectory, ...
    'static_3_to_5_device_pva_summary.csv'));
writetable(referenceTable, fullfile(outputDirectory, ...
    'static_3_to_5_common_reference.csv'));
writetable(pureInsSummaryTable, fullfile(outputDirectory, ...
    'static_3_to_5_realign_pureins_summary.csv'));
writetable(pureInsImprovementTable, fullfile(outputDirectory, ...
    'static_3_to_5_realign_improvement.csv'));
save(fullfile(outputDirectory, 'static_3_to_5_device_pva_analysis.mat'), ...
    'results', 'summaryTable', 'referenceTable', 'initialPositionDegM', ...
    'commonReferencePositionDegM', 'pureInsResults', ...
    'pureInsSummaryTable', 'pureInsImprovementTable', ...
    'alignmentReferencePositionDegM', 'pureInsConstraintName', ...
    'evaluationDurationSeconds');

%% 6. 绘图
colors = lines(numel(caseNumbers));
% figureHandle = figure('Color', 'w', 'Position', [80, 80, 1450, 880], ...
%     'Name', 'static-3~5 设备 PVA 静态位置误差');
figureHandle = myfigurestartup(7,5,'paper');
layout = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, 'static-3~5 设备解算静态位置误差（统一参考位置）');

nexttile;
hold on;
for index = 1:numel(results)
    plot(results(index).timeSeconds, results(index).northErrorM, ...
        'LineWidth', 1.0, 'Color', colors(index, :));
end
grid on;
box on;
xlabel('相对时间 / s');
ylabel('北向误差 / m');
title('北向位置误差');
legend(compose('static-%d', caseNumbers), 'Location', 'best');
xlim([0, evaluationDurationSeconds]);

nexttile;
hold on;
for index = 1:numel(results)
    plot(results(index).timeSeconds, results(index).eastErrorM, ...
        'LineWidth', 1.0, 'Color', colors(index, :));
end
grid on;
box on;
xlabel('相对时间 / s');
ylabel('东向误差 / m');
title('东向位置误差');
legend(compose('static-%d', caseNumbers), 'Location', 'best');
xlim([0, evaluationDurationSeconds]);

nexttile;
hold on;
for index = 1:numel(results)
    plot(results(index).timeSeconds, results(index).radialErrorM, ...
        'LineWidth', 1.0, 'Color', colors(index, :));
end
grid on;
box on;
xlabel('相对时间 / s');
ylabel('水平径向误差 / m');
title('水平径向误差');
legend(compose('static-%d', caseNumbers), 'Location', 'best');
xlim([0, evaluationDurationSeconds]);

nexttile;
barHandle = bar(caseColumn, endpointRerColumn, 0.65, 'FaceColor', 'flat');
barHandle.CData = colors;
grid on;
box on;
xlabel('数据集');
ylabel('单次试验 RER / (n mile/h)');
title('评价终点位置误差率 RER');
xticks(caseColumn);
xticklabels(compose('static-%d', caseColumn));
for index = 1:numel(endpointRerColumn)
    text(caseColumn(index), endpointRerColumn(index), ...
        sprintf(' %.5f', endpointRerColumn(index)), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
end

figurePng = fullfile(outputDirectory, ...
    'static_3_to_5_device_pva_position_error.png');
figureFig = fullfile(outputDirectory, ...
    'static_3_to_5_device_pva_position_error.fig');
exportgraphics(figureHandle, figurePng, 'Resolution', 300);
savefig(figureHandle, figureFig);

% 每行对应一个数据集：左侧比较径向误差，右侧比较单次试验终点RER。
% realignFigure = figure('Color', 'w', 'Position', [100, 50, 1500, 1150], ...
%     'Name', 'static-3~5 重新对准前后纯惯导对比');
realignFigure = myfigurestartup(7,5,'paper');
realignLayout = tiledlayout(numel(caseNumbers), 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
title(realignLayout, sprintf(['static-3~5 重新对准前后纯惯导对比' ...
    '（%s）'], pureInsConstraintName));

solutionColors = [0.0000, 0.4470, 0.7410; 0.8500, 0.3250, 0.0980];
for index = 1:numel(caseNumbers)
    nexttile;
    hold on;
    for solutionIndex = 1:numel(pureInsSolutionKeys)
        thisResult = pureInsResults{index, solutionIndex};
        plot(thisResult.timeSeconds, thisResult.radialErrorM, ...
            'LineWidth', 1.1, 'Color', solutionColors(solutionIndex, :));
    end
    grid on;
    box on;
    xlabel('相对时间 / s');
    ylabel('水平径向误差 / m');
    title(sprintf('static-%d 水平径向误差', caseNumbers(index)));
    legend(pureInsSolutionLabels, 'Location', 'best');
    xlim([0, evaluationDurationSeconds]);

    nexttile;
    rerPair = [pureInsResults{index, 1}.endpointRerNmiPerHour, ...
        pureInsResults{index, 2}.endpointRerNmiPerHour];
    pairBar = bar(1:2, rerPair, 0.62, 'FaceColor', 'flat');
    pairBar.CData = solutionColors;
    grid on;
    box on;
    ylabel('单次试验 RER / (n mile/h)');
    title(sprintf('static-%d 评价终点RER', caseNumbers(index)));
    xticks(1:2);
    xticklabels(pureInsSolutionLabels);
    for solutionIndex = 1:2
        text(solutionIndex, rerPair(solutionIndex), ...
            sprintf(' %.5f', rerPair(solutionIndex)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');
    end
end

realignFigurePng = fullfile(outputDirectory, ...
    'static_3_to_5_realign_pureins_comparison.png');
realignFigureFig = fullfile(outputDirectory, ...
    'static_3_to_5_realign_pureins_comparison.fig');
exportgraphics(realignFigure, realignFigurePng, 'Resolution', 300);
savefig(realignFigure, realignFigureFig);

fprintf('\n汇总结果已保存：\n%s\n', outputDirectory);
fprintf('本脚本只计算每批单次试验的终点RER，不计算R50。\n');

%% 局部辅助函数
function result = analyze_navigation_position_file( ...
        navigationFile, referencePositionDegM, evaluationDurationSeconds, ...
        meterPerNauticalMile, secondsPerHour, caseNumber)
% 读取一个11列导航结果，按固定静态位置计算前指定时长的位置误差。
    navigation = readmatrix(navigationFile, 'FileType', 'text');
    if isempty(navigation) || size(navigation, 2) < 11
        error('导航结果应至少包含11列：%s', navigationFile);
    end

    validRow = all(isfinite(navigation(:, 2:5)), 2);
    navigation = navigation(validRow, :);
    navigation = sortrows(navigation, 2);
    [~, uniqueIndex] = unique(navigation(:, 2), 'stable');
    navigation = navigation(uniqueIndex, :);
    if size(navigation, 1) < 2
        error('导航结果删除无效行后数据不足：%s', navigationFile);
    end

    navigation = select_longest_continuous_pva_segment(navigation, caseNumber);
    elapsedAll = navigation(:, 2) - navigation(1, 2);
    availableDuration = elapsedAll(end);
    evaluationEnd = min(evaluationDurationSeconds, availableDuration);
    evaluationMask = elapsedAll >= 0 & elapsedAll <= evaluationEnd;
    navigation = navigation(evaluationMask, :);
    elapsedSeconds = elapsedAll(evaluationMask);

    param = Param();
    referenceLatitudeRad = deg2rad(referencePositionDegM(1));
    referenceHeightM = referencePositionDegM(3);
    [referenceRm, referenceRn] = getRmRn(referenceLatitudeRad, param);

    latitudeDifferenceRad = deg2rad(navigation(:, 3) - referencePositionDegM(1));
    longitudeDifferenceRad = deg2rad(navigation(:, 4) - referencePositionDegM(2));
    northErrorM = latitudeDifferenceRad * (referenceRm + referenceHeightM);
    eastErrorM = longitudeDifferenceRad * ...
        (referenceRn + referenceHeightM) * cos(referenceLatitudeRad);
    downErrorM = -(navigation(:, 5) - referencePositionDegM(3));
    radialErrorM = hypot(northErrorM, eastErrorM);

    validError = isfinite(elapsedSeconds) & isfinite(radialErrorM);
    elapsedSeconds = elapsedSeconds(validError);
    northErrorM = northErrorM(validError);
    eastErrorM = eastErrorM(validError);
    downErrorM = downErrorM(validError);
    radialErrorM = radialErrorM(validError);
    if numel(radialErrorM) < 2
        error('导航结果没有足够的有效位置误差数据：%s', navigationFile);
    end

    endpointTimeSeconds = elapsedSeconds(end);
    endpointRadialErrorM = radialErrorM(end);
    if endpointTimeSeconds > 0
        endpointRerNmiPerHour = ...
            (endpointRadialErrorM / meterPerNauticalMile) / ...
            (endpointTimeSeconds / secondsPerHour);
    else
        endpointRerNmiPerHour = NaN;
    end

    result = struct();
    result.timeSeconds = elapsedSeconds;
    result.northErrorM = northErrorM;
    result.eastErrorM = eastErrorM;
    result.downErrorM = downErrorM;
    result.radialErrorM = radialErrorM;
    result.availableDurationSeconds = availableDuration;
    result.evaluatedDurationSeconds = elapsedSeconds(end);
    result.sampleCount = numel(radialErrorM);
    result.radialRmsM = sqrt(mean(radialErrorM .^ 2));
    result.radialMaxM = max(radialErrorM);
    result.radialMedianM = median(radialErrorM);
    result.endpointRerNmiPerHour = endpointRerNmiPerHour;
end

function continuousPva = select_longest_continuous_pva_segment(pva, caseNumber)
% 选择最长连续导航时间段，避免异常跳时造成虚假的超长评价区间。
% 设备 PVA 偶尔会丢失几十个历元（static-3 最大约 0.38 s），这仍属于
% 同一次静态试验，不能据此重置累计运行时间。因此这里只把超过 60 s
% 的跳变视为真正断段；小范围丢帧保留，并使用真实时间差计算RER。
    timeSeconds = pva(:, 2);
    timeDifference = diff(timeSeconds);
    positiveDifference = timeDifference(timeDifference > 0 & isfinite(timeDifference));
    if isempty(positiveDifference)
        error('static-%d 的导航时间没有递增。', caseNumber);
    end

    nominalInterval = median(positiveDifference);
    gapThreshold = max(60, 1000 * nominalInterval);
    breakIndex = find(timeDifference <= 0 | timeDifference > gapThreshold);
    segmentStart = [1; breakIndex + 1];
    segmentEnd = [breakIndex; size(pva, 1)];
    segmentLength = segmentEnd - segmentStart + 1;
    [~, longestIndex] = max(segmentLength);
    keepIndex = segmentStart(longestIndex):segmentEnd(longestIndex);
    continuousPva = pva(keepIndex, :);

    discardedCount = size(pva, 1) - size(continuousPva, 1);
    if discardedCount > 0
        warning('static-%d 检测到时间断点，仅保留最长连续段，舍弃 %d 行。', ...
            caseNumber, discardedCount);
    end
end
