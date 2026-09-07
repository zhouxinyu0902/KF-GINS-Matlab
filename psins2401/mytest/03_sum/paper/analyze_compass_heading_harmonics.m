function result = analyze_compass_heading_harmonics( ...
    tCompass, psiCompassDeg, tOctans, psiOctansDeg, varargin)
%ANALYZE_COMPASS_HEADING_HARMONICS_FOLDCV
% 面向论文展示的罗盘航向相关残余误差谐波阶次比较。
%
% 本函数重点实现两项分析：
%   1) 180 deg 周期折叠图，用于直观检验二倍角周期特征；
%   2) 常值、一倍角、二倍角和三倍角模型的公平比较表。
%
% 候选模型：
%   M0: e(psi) = b
%   M1: e(psi) = b + a1*cos(psi)   + c1*sin(psi)
%   M2: e(psi) = b + a2*cos(2*psi) + c2*sin(2*psi)
%   M3: e(psi) = b + a3*cos(3*psi) + c3*sin(3*psi)
%
% 一、二、三倍角模型均包含3个参数，因而阶次比较是公平的。
% 模型拟合基于航向分箱中值，各有效航向分箱等权参与，避免长时间
% 保持某一航向时产生样本数量偏置。
%
% 基本调用：
%   result = analyze_compass_heading_harmonics_foldcv( ...
%       compass(:,4), r2d(compass(:,3)), ...
%       avp_ref(:,end), r2d(avp_ref(:,3)));
%
% 可选参数：
%   'BinWidth'       - 航向分箱宽度，默认10 deg；应能整除360
%   'MinBinCount'    - 每个分箱最少样本数，默认10
%   'OutlierSigma'   - 分箱内MAD异常阈值，默认6；Inf表示不剔除
%   'MaxAbsResidual' - 最大允许航向残差绝对值，默认30 deg
%   'ErrorSign'      - 'compass-minus-octans'（默认）或
%                      'octans-minus-compass'
%   'OutputDir'      - 输出根目录，默认当前目录
%   'FilePrefix'     - 输出文件前缀，默认'compass_harmonic_order'
%   'FigureWidth'    - 'single'或'double'，默认'double'
%   'ShowFigure'     - 是否显示图窗，默认true
%   'ExportFiles'    - 是否导出PDF、PNG、CSV和LaTeX，默认true
%
% 主要输出：
%   result.modelComparison - 论文主表：拟合与交叉验证指标
%   result.modelDiagnostics- 秩、条件数及有效CV折数
%   result.periodicity180  - 180 deg 周期一致性指标
%   result.binnedData      - 0--360 deg分箱统计
%   result.foldedData      - 180 deg折叠分箱统计
%   result.secondHarmonic  - “常值+二倍角”模型参数
%   result.fiveStateModel  - 不含常值项的二倍角参数，供5-state模型参考
%   result.files           - 导出文件路径
%
% 说明：
%   本函数能够使模型比较更直观，但不能替代航向覆盖。若有效航向区间
%   过少，结果只能说明已观测航向范围内的拟合与预测表现。

p = inputParser;
p.addParameter('BinWidth', 10, ...
    @(x) isscalar(x) && isfinite(x) && x > 0 && x <= 90);
p.addParameter('MinBinCount', 10, ...
    @(x) isscalar(x) && isfinite(x) && x >= 1);
p.addParameter('OutlierSigma', 6, ...
    @(x) isscalar(x) && x > 0);
p.addParameter('MaxAbsResidual', 30, ...
    @(x) isscalar(x) && x > 0);
p.addParameter('ErrorSign', 'compass-minus-octans', ...
    @(x) ischar(x) || isstring(x));
p.addParameter('OutputDir', pwd, @(x) ischar(x) || isstring(x));
p.addParameter('FilePrefix', 'compass_harmonic_order', ...
    @(x) ischar(x) || isstring(x));
p.addParameter('FigureWidth', 'double', ...
    @(x) any(strcmpi(string(x), ["single", "double"])));
p.addParameter('ShowFigure', true, ...
    @(x) islogical(x) || isnumeric(x));
p.addParameter('ExportFiles', true, ...
    @(x) islogical(x) || isnumeric(x));
p.parse(varargin{:});
opt = p.Results;

binWidth = double(opt.BinWidth);
numBins = round(360 / binWidth);
if abs(numBins * binWidth - 360) > 1e-10
    error('BinWidth必须能够整除360，例如5、10、15或20 deg。');
end
if mod(numBins, 2) ~= 0
    error('BinWidth还需使180/BinWidth为整数。');
end

outputDir = char(opt.OutputDir);
figureDir = fullfile(outputDir, 'figures');
tableDir = fullfile(outputDir, 'tables');
filePrefix = char(opt.FilePrefix);
if logical(opt.ExportFiles)
    if ~exist(figureDir, 'dir'), mkdir(figureDir); end
    if ~exist(tableDir, 'dir'), mkdir(tableDir); end
end

%% 1. 数据清理与时间同步
tCompass = tCompass(:);
psiCompassDeg = psiCompassDeg(:);
tOctans = tOctans(:);
psiOctansDeg = psiOctansDeg(:);

validC = isfinite(tCompass) & isfinite(psiCompassDeg);
validO = isfinite(tOctans) & isfinite(psiOctansDeg);
tCompass = tCompass(validC);
psiCompassDeg = psiCompassDeg(validC);
tOctans = tOctans(validO);
psiOctansDeg = psiOctansDeg(validO);

if numel(tCompass) < 3 || numel(tOctans) < 3
    error('罗盘或OCTANS有效样本不足。');
end

[tCompass, idxC] = unique(tCompass, 'stable');
psiCompassDeg = psiCompassDeg(idxC);
[tOctans, idxO] = unique(tOctans, 'stable');
psiOctansDeg = psiOctansDeg(idxO);

[tCompass, orderC] = sort(tCompass);
psiCompassDeg = psiCompassDeg(orderC);
[tOctans, orderO] = sort(tOctans);
psiOctansDeg = psiOctansDeg(orderO);

% 参考航向先解缠再插值，避免0/360 deg附近的插值跳变。
psiOctansUnwrapped = unwrap(deg2rad(psiOctansDeg));
psiRefRad = interp1(tOctans, psiOctansUnwrapped, ...
    tCompass, 'linear', NaN);
psiRefDeg = mod(rad2deg(psiRefRad), 360);
psiCompassDeg = mod(psiCompassDeg, 360);

switch lower(string(opt.ErrorSign))
    case "compass-minus-octans"
        headingErrorDeg = localWrapTo180(psiCompassDeg - psiRefDeg);
    case "octans-minus-compass"
        headingErrorDeg = localWrapTo180(psiRefDeg - psiCompassDeg);
    otherwise
        error(['ErrorSign必须为compass-minus-octans或', ...
            'octans-minus-compass。']);
end

valid = isfinite(psiRefDeg) & isfinite(headingErrorDeg) & ...
    abs(headingErrorDeg) <= opt.MaxAbsResidual;
t = tCompass(valid);
psiRefDeg = psiRefDeg(valid);
headingErrorDeg = headingErrorDeg(valid);

if numel(headingErrorDeg) < 30
    error('同步并筛选后的有效样本少于30个，无法进行模型比较。');
end

%% 2. 在航向分箱内剔除孤立异常点
[binIdAll, binCenters] = localCircularBinId(psiRefDeg, binWidth);
keep = true(size(headingErrorDeg));
if isfinite(opt.OutlierSigma)
    for k = 1:numBins
        idx = find(binIdAll == k);
        if numel(idx) < 5
            continue;
        end
        e = headingErrorDeg(idx);
        medE = median(e);
        sigmaE = 1.4826 * median(abs(e - medE));
        if sigmaE > eps
            keep(idx) = abs(e - medE) <= opt.OutlierSigma * sigmaE;
        end
    end
end

t = t(keep);
psiRefDeg = psiRefDeg(keep);
headingErrorDeg = headingErrorDeg(keep);

%% 3. 0--360 deg航向分箱统计
[binId, binCenters] = localCircularBinId(psiRefDeg, binWidth);
[countBin, medBin, q25Bin, q75Bin] = localBinStatistics( ...
    headingErrorDeg, binId, numBins);
validBin = countBin >= opt.MinBinCount & isfinite(medBin);

headingBinDeg = binCenters(validBin);
errorBinDeg = medBin(validBin);
q25Valid = q25Bin(validBin);
q75Valid = q75Bin(validBin);
countValid = countBin(validBin);

[headingBinDeg, sortIdx] = sort(headingBinDeg);
errorBinDeg = errorBinDeg(sortIdx);
q25Valid = q25Valid(sortIdx);
q75Valid = q75Valid(sortIdx);
countValid = countValid(sortIdx);

nBin = numel(errorBinDeg);
if nBin < 4
    error(['有效航向分箱少于4个，无法公平比较常值及三参数谐波模型。', ...
        '请减小MinBinCount或增加航向覆盖。']);
end

%% 4. 候选模型拟合与留一航向分箱交叉验证
modelNames = ["Constant bias"; "First harmonic"; ...
    "Second harmonic"; "Third harmonic"];
modelTypes = ["constant"; "harmonic1"; "harmonic2"; "harmonic3"];
numModels = numel(modelNames);

numParameters = zeros(numModels, 1);
fitRMSE = nan(numModels, 1);
fitMAE = nan(numModels, 1);
adjustedR2 = nan(numModels, 1);
bic = nan(numModels, 1);
cvRMSE = nan(numModels, 1);
rankX = zeros(numModels, 1);
conditionNumber = inf(numModels, 1);
validCVFolds = zeros(numModels, 1);
coefficients = cell(numModels, 1);
predictedBins = cell(numModels, 1);
cvPredictions = cell(numModels, 1);
status = strings(numModels, 1);

for i = 1:numModels
    X = localDesignMatrix(headingBinDeg, modelTypes(i));
    k = size(X, 2);
    numParameters(i) = k;
    rankX(i) = rank(X);

    if rankX(i) < k
        status(i) = "Rank-deficient";
        coefficients{i} = pinv(X) * errorBinDeg;
        predictedBins{i} = X * coefficients{i};
        continue;
    end

    conditionNumber(i) = cond(X);
    beta = X \ errorBinDeg;
    yhat = X * beta;
    residual = errorBinDeg - yhat;
    rss = sum(residual.^2);
    tss = sum((errorBinDeg - mean(errorBinDeg)).^2);

    coefficients{i} = beta;
    predictedBins{i} = yhat;
    fitRMSE(i) = sqrt(mean(residual.^2));
    fitMAE(i) = mean(abs(residual));

    if tss > eps
        r2 = 1 - rss / tss;
        if nBin > k
            adjustedR2(i) = 1 - (1 - r2) * (nBin - 1) / (nBin - k);
        end
    end

    rssForBIC = max(rss, eps);
    bic(i) = nBin * log(rssForBIC / nBin) + k * log(nBin);

    % 留一航向分箱交叉验证：每次整块留出一个航向分箱。
    ycv = nan(nBin, 1);
    for j = 1:nBin
        train = true(nBin, 1);
        train(j) = false;
        Xtrain = X(train, :);
        ytrain = errorBinDeg(train);
        if size(Xtrain, 1) >= k && rank(Xtrain) == k
            betaCV = Xtrain \ ytrain;
            ycv(j) = X(j, :) * betaCV;
        end
    end
    cvPredictions{i} = ycv;
    validCVFolds(i) = nnz(isfinite(ycv));
    if validCVFolds(i) == nBin
        cvRMSE(i) = sqrt(mean((errorBinDeg - ycv).^2));
    end

    if validCVFolds(i) < nBin
        status(i) = "Insufficient CV excitation";
    elseif conditionNumber(i) > 1e4
        status(i) = "Ill-conditioned";
    else
        status(i) = "Supported";
    end
end

finiteBIC = isfinite(bic);
deltaBIC = nan(size(bic));
if any(finiteBIC)
    deltaBIC(finiteBIC) = bic(finiteBIC) - min(bic(finiteBIC));
end

modelComparison = table(modelNames, numParameters, fitRMSE, cvRMSE, ...
    adjustedR2, bic, deltaBIC, ...
    'VariableNames', {'Model', 'NumParameters', 'FitRMSE_deg', ...
    'HeadingBlockCVRMSE_deg', 'AdjustedR2', 'BIC', 'DeltaBIC'});

modelDiagnostics = table(modelNames, rankX, conditionNumber, ...
    validCVFolds, repmat(nBin, numModels, 1), status, ...
    'VariableNames', {'Model', 'DesignRank', 'ConditionNumber', ...
    'ValidCVFolds', 'TotalCVFolds', 'Status'});

%% 5. 180 deg周期折叠统计
numFoldBins = numBins / 2;
foldCenters = (0:numFoldBins-1).' * binWidth;
psiFoldDeg = mod(psiRefDeg, 180);
halfGroup = psiRefDeg >= 180; % 0: 0--180; 1: 180--360
foldId = mod(floor((psiFoldDeg + binWidth/2) / binWidth), ...
    numFoldBins) + 1;

countA = zeros(numFoldBins, 1);
medA = nan(numFoldBins, 1);
q25A = nan(numFoldBins, 1);
q75A = nan(numFoldBins, 1);
countB = zeros(numFoldBins, 1);
medB = nan(numFoldBins, 1);
q25B = nan(numFoldBins, 1);
q75B = nan(numFoldBins, 1);

[countA, medA, q25A, q75A] = localBinStatistics( ...
    headingErrorDeg(~halfGroup), foldId(~halfGroup), numFoldBins);
[countB, medB, q25B, q75B] = localBinStatistics( ...
    headingErrorDeg(halfGroup), foldId(halfGroup), numFoldBins);

validA = countA >= opt.MinBinCount & isfinite(medA);
validB = countB >= opt.MinBinCount & isfinite(medB);
paired = validA & validB;

if any(paired)
    pairedDiff = medA(paired) - medB(paired);
    j180RMSE = sqrt(mean(pairedDiff.^2));
    j180MAE = mean(abs(pairedDiff));
else
    pairedDiff = [];
    j180RMSE = NaN;
    j180MAE = NaN;
end

if nnz(paired) >= 3 && std(medA(paired)) > eps && std(medB(paired)) > eps
    corrMat = corrcoef(medA(paired), medB(paired));
    corr180 = corrMat(1, 2);
else
    corr180 = NaN;
end

periodicity180 = struct();
periodicity180.J180_RMSE_deg = j180RMSE;
periodicity180.J180_MAE_deg = j180MAE;
periodicity180.Correlation = corr180;
periodicity180.NumPairedBins = nnz(paired);
periodicity180.PairedDifference_deg = pairedDiff;

%% 6. 提取二倍角模型参数
idxSecond = find(modelTypes == "harmonic2", 1);
betaSecond = coefficients{idxSecond};
secondHarmonic = struct('bias', NaN, 'c1', NaN, 'c2', NaN, ...
    'amplitude', NaN, 'phaseDeg', NaN);
if numel(betaSecond) == 3
    secondHarmonic.bias = betaSecond(1);
    secondHarmonic.c1 = betaSecond(2);
    secondHarmonic.c2 = betaSecond(3);
    secondHarmonic.amplitude = hypot(betaSecond(2), betaSecond(3));
    secondHarmonic.phaseDeg = atan2d(betaSecond(3), betaSecond(2));
end

% 另行拟合论文5-state对应的不含常值项二倍角模型。
X5 = [cosd(2 * headingBinDeg), sind(2 * headingBinDeg)];
fiveStateModel = struct('c1', NaN, 'c2', NaN, 'amplitude', NaN, ...
    'phaseDeg', NaN, 'fitRMSE_deg', NaN, 'rank', rank(X5), ...
    'conditionNumber', Inf);
if rank(X5) == 2
    beta5 = X5 \ errorBinDeg;
    residual5 = errorBinDeg - X5 * beta5;
    fiveStateModel.c1 = beta5(1);
    fiveStateModel.c2 = beta5(2);
    fiveStateModel.amplitude = hypot(beta5(1), beta5(2));
    fiveStateModel.phaseDeg = atan2d(beta5(2), beta5(1));
    fiveStateModel.fitRMSE_deg = sqrt(mean(residual5.^2));
    fiveStateModel.conditionNumber = cond(X5);
end

%% 7. 论文图：候选模型比较 + 180 deg折叠
if strcmpi(opt.FigureWidth, 'single')
    figPos = [2, 2, 17.5, 7.6];
else
    figPos = [2, 2, 18.5, 7.8];
end

% fig = figure('Name', 'Compass harmonic-order comparison', ...
%     'Units', 'centimeters', 'Position', figPos, ...
%     'Color', 'w', 'Visible', localOnOff(logical(opt.ShowFigure)));
%  Msize = 8;
fig = myfigurestartup(7,3,'prese');
Msize = 9;
tl = tiledlayout(fig, 1, 2, 'TileSpacing', 'compact', ...
    'Padding', 'compact');

% (a) 航向域模型比较
ax1 = nexttile(tl, 1);
hold(ax1, 'on');
lowerErr = errorBinDeg - q25Valid;
upperErr = q75Valid - errorBinDeg;
errorbar(ax1, headingBinDeg, errorBinDeg, lowerErr, upperErr, ...
    'o', 'LineWidth', 1.0, 'MarkerSize', 4.5, ...
    'DisplayName', 'Bin median and IQR');

lineStyles = {'--', '-.', '-', ':'};
lineWidths = [1.2, 1.2, 1.8, 1.4];
for i = 1:numModels
    if isempty(predictedBins{i}) || all(~isfinite(predictedBins{i}))
        continue;
    end
    plot(ax1, headingBinDeg, predictedBins{i}, ...
        'LineStyle', lineStyles{i}, 'LineWidth', lineWidths(i), ...
        'Marker', 'none', 'DisplayName', modelNames(i));
end
xlabel(ax1, 'OCTANS reference heading (deg)');
ylabel(ax1, 'Compass residual (deg)');
xlim(ax1, [0, 360]);
xticks(ax1, 0:60:360);
grid(ax1, 'on');
box(ax1, 'on');
% title(ax1, '(a) Harmonic-order comparison');
legend(ax1, 'Location', 'best', 'FontSize', Msize);

% (b) 180 deg折叠
ax2 = nexttile(tl, 2);
hold(ax2, 'on');
if any(validA)
    errorbar(ax2, foldCenters(validA), medA(validA), ...
        medA(validA)-q25A(validA), q75A(validA)-medA(validA), ...
        'o-', 'LineWidth', 1.1, 'MarkerSize', 4.5, ...
        'DisplayName', 'Original heading: 0--180 deg');
end
if any(validB)
    errorbar(ax2, foldCenters(validB), medB(validB), ...
        medB(validB)-q25B(validB), q75B(validB)-medB(validB), ...
        's--', 'LineWidth', 1.1, 'MarkerSize', 4.5, ...
        'DisplayName', 'Original heading: 180--360 deg');
end

% 二倍角模型在折叠域中的预测。
if numel(betaSecond) == 3 && all(isfinite(betaSecond))
    foldGrid = linspace(0, 180, 361).';
    Xfold = localDesignMatrix(foldGrid, "harmonic2");
    plot(ax2, foldGrid, Xfold * betaSecond, '-', ...
        'LineWidth', 1.8, 'DisplayName', 'Second-harmonic fit');
end
xlabel(ax2, 'Folded heading, mod(\psi,180 deg) (deg)');
ylabel(ax2, 'Compass residual (deg)');
xlim(ax2, [0, 180]);
xticks(ax2, 0:30:180);
grid(ax2, 'on');
box(ax2, 'on');
title(ax2, '(b) 180-deg periodicity check');
legend(ax2, 'Location', 'best', 'FontSize', 8);

if isfinite(j180RMSE)
    txt = sprintf('$J_{180}=%.3f^{\\circ}$, paired bins = %d', ...
        j180RMSE, nnz(paired));
else
    txt = sprintf('Paired bins = %d', nnz(paired));
end
text(ax2, 0.03, 0.96, txt, 'Units', 'normalized', ...
    'VerticalAlignment', 'top', 'Interpreter', 'latex', ...
    'FontSize', 8, 'BackgroundColor', 'w', 'Margin', 2);

set([ax1, ax2], 'FontName', 'Times New Roman', 'FontSize', 9);

%% 8. 输出结果与文件
binnedData = table(headingBinDeg, errorBinDeg, q25Valid, q75Valid, ...
    countValid, 'VariableNames', {'HeadingBin_deg', ...
    'MedianResidual_deg', 'Q25_deg', 'Q75_deg', 'SampleCount'});

foldedData = table(foldCenters, countA, medA, q25A, q75A, ...
    countB, medB, q25B, q75B, paired, ...
    'VariableNames', {'FoldedHeading_deg', 'Count_0_180', ...
    'Median_0_180_deg', 'Q25_0_180_deg', 'Q75_0_180_deg', ...
    'Count_180_360', 'Median_180_360_deg', 'Q25_180_360_deg', ...
    'Q75_180_360_deg', 'PairedValid'});

syncedData = table(t, psiRefDeg, headingErrorDeg, ...
    'VariableNames', {'Time', 'OCTANSHeading_deg', ...
    'CompassResidual_deg'});

coverageDeg = localCircularCoverage(headingBinDeg);
dataSummary = table(numel(headingErrorDeg), nBin, coverageDeg, ...
    nnz(paired), 'VariableNames', {'NumSynchronizedSamples', ...
    'NumValidHeadingBins', 'CircularHeadingCoverage_deg', ...
    'NumPairedFoldedBins'});

files = struct('figurePDF', '', 'figurePNG', '', ...
    'modelCSV', '', 'diagnosticsCSV', '', 'foldedCSV', '', ...
    'latexTable', '');

if logical(opt.ExportFiles)
    files.figurePDF = fullfile(figureDir, [filePrefix, '_foldcv.pdf']);
    files.figurePNG = fullfile(figureDir, [filePrefix, '_foldcv.png']);
    files.modelCSV = fullfile(tableDir, ...
        [filePrefix, '_model_comparison.csv']);
    files.diagnosticsCSV = fullfile(tableDir, ...
        [filePrefix, '_model_diagnostics.csv']);
    files.foldedCSV = fullfile(tableDir, ...
        [filePrefix, '_folded_180.csv']);
    files.latexTable = fullfile(tableDir, ...
        [filePrefix, '_model_comparison.tex']);

    exportgraphics(fig, files.figurePDF, 'ContentType', 'vector');
    exportgraphics(fig, files.figurePNG, 'Resolution', 600);
    writetable(modelComparison, files.modelCSV);
    writetable(modelDiagnostics, files.diagnosticsCSV);
    writetable(foldedData, files.foldedCSV);
    localWriteLatexTable(modelComparison, files.latexTable);
end


result = struct();
result.ax1= ax1;
result.modelComparison = modelComparison;
result.modelDiagnostics = modelDiagnostics;
result.periodicity180 = periodicity180;
result.secondHarmonic = secondHarmonic;
result.fiveStateModel = fiveStateModel;
result.syncedData = syncedData;
result.binnedData = binnedData;
result.foldedData = foldedData;
result.dataSummary = dataSummary;
result.coefficients.Model = modelNames;
result.coefficients.Value = coefficients;
result.figureHandle = fig;
result.files = files;

fprintf('\n候选模型比较（各有效航向分箱等权）：\n');
disp(modelComparison);
fprintf('模型数值诊断：\n');
disp(modelDiagnostics);
fprintf(['180 deg周期一致性：J180(RMSE) = %.4f deg, ', ...
    'J180(MAE) = %.4f deg, paired bins = %d, correlation = %.4f\n'], ...
    j180RMSE, j180MAE, nnz(paired), corr180);
fprintf('有效航向分箱数 = %d，圆周航向覆盖约 %.1f deg。\n', ...
    nBin, coverageDeg);
end

%% 局部函数
function [binId, centers] = localCircularBinId(angleDeg, binWidth)
numBins = round(360 / binWidth);
centers = (0:numBins-1).' * binWidth;
binId = mod(floor((mod(angleDeg, 360) + binWidth/2) / binWidth), ...
    numBins) + 1;
end

function [counts, medians, q25, q75] = localBinStatistics(y, binId, nBins)
counts = zeros(nBins, 1);
medians = nan(nBins, 1);
q25 = nan(nBins, 1);
q75 = nan(nBins, 1);
for k = 1:nBins
    idx = binId == k;
    counts(k) = nnz(idx);
    if counts(k) > 0
        values = y(idx);
        medians(k) = median(values);
        q25(k) = prctile(values, 25);
        q75(k) = prctile(values, 75);
    end
end
end

function X = localDesignMatrix(headingDeg, modelType)
headingDeg = headingDeg(:);
switch lower(string(modelType))
    case "constant"
        X = ones(size(headingDeg));
    case "harmonic1"
        X = [ones(size(headingDeg)), ...
            cosd(headingDeg), sind(headingDeg)];
    case "harmonic2"
        X = [ones(size(headingDeg)), ...
            cosd(2 * headingDeg), sind(2 * headingDeg)];
    case "harmonic3"
        X = [ones(size(headingDeg)), ...
            cosd(3 * headingDeg), sind(3 * headingDeg)];
    otherwise
        error('未知模型类型：%s', modelType);
end
end

function angleDeg = localWrapTo180(angleDeg)
angleDeg = mod(angleDeg + 180, 360) - 180;
end

function coverageDeg = localCircularCoverage(headingDeg)
headingDeg = sort(unique(mod(headingDeg(:), 360)));
if numel(headingDeg) <= 1
    coverageDeg = 0;
    return;
end
gaps = diff([headingDeg; headingDeg(1) + 360]);
coverageDeg = 360 - max(gaps);
end

function value = localOnOff(tf)
if tf
    value = 'on';
else
    value = 'off';
end
end

function localWriteLatexTable(tbl, filePath)
fid = fopen(filePath, 'w');
if fid < 0
    warning('无法写入LaTeX表格：%s', filePath);
    return;
end
cleanupObj = onCleanup(@() fclose(fid)); 
fprintf(fid, '%% Auto-generated model comparison table\n');
fprintf(fid, '\\begin{table}[t]\n');
fprintf(fid, '\\centering\n');
fprintf(fid, '\\caption{Comparison of candidate compass-error models.}\n');
fprintf(fid, '\\label{tab:compass_harmonic_order}\n');
fprintf(fid, '\\begin{tabular}{lrrrrrr}\n');
fprintf(fid, '\\hline\n');
fprintf(fid, ['Model & $p$ & Fit RMSE & Block-CV RMSE & ', ...
    '$\\bar{R}^2$ & BIC & $\\Delta$BIC \\\\ \n']);
fprintf(fid, '\\hline\n');
for i = 1:height(tbl)
    model = strrep(char(tbl.Model(i)), '_', '\\_');
    fprintf(fid, '%s & %d & %.4f & %.4f & %.4f & %.3f & %.3f \\\\ \n', ...
        model, tbl.NumParameters(i), tbl.FitRMSE_deg(i), ...
        tbl.HeadingBlockCVRMSE_deg(i), tbl.AdjustedR2(i), ...
        tbl.BIC(i), tbl.DeltaBIC(i));
end
fprintf(fid, '\\hline\n');
fprintf(fid, '\\end{tabular}\n');
fprintf(fid, '\\end{table}\n');
end
