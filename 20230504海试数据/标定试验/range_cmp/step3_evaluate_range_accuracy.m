%% Step 3：水平测距结果精度评估
% 本文件是可直接运行的脚本，不是函数。请先依次运行Step 2A和Step 2B。
%
% 作用
%   1. 读取USBL几何测距结果和传播时延测距结果；
%   2. 以“自动标定信标坐标 + PPC船位”的水平距离为参考；
%   3. 计算PTSAX、PTSAG、Equation、Zhou、AMS、GMS的误差指标；
%   4. 导出逐条对比表、总体/分ID统计表、Excel和对比图。
%
% 评估指标
%   有效数量、平均误差、中位误差、标准差、MAE、RMSE和95%绝对误差。
%
% 重要说明
%   - 所有指标均比较“水平距离”，不是斜距。
%   - PTSAX、PTSAG使用Step 2A剔除异常值后的结果。
%   - 时延方法使用Step 2B中CH1--CH3融合且完成预处理后的结果。
%   - 参考信标位置来自PTSAG点云稳健中位数，因此PTSAG与参考值同源；
%     这适合内部一致性分析，但不能替代独立的大地测量真值评估。


runBeaconId = 91;

clearvars -except runBeaconId;
clc;
close all;

%% 路径与输入输出文件
scriptDir = fileparts(mfilename('fullpath'));
calibrationScriptDir = fileparts(scriptDir);
legacyDataDir = fileparts(calibrationScriptDir);
repositoryDir = fileparts(legacyDataDir);
analysisDir = string(fullfile(repositoryDir, 'data', '20230504', ...
    '标定实验分析'));
outputDir = fullfile(analysisDir, '解析后的数据', 'USBL_range_cmp');
[runSuffix, singleIdRun] = make_suffix(runBeaconId);

step2aFile = fullfile(outputDir, ...
    ['step2a_usbl_horizontal_range' runSuffix '.mat']);
step2bFile = fullfile(outputDir, ...
    ['step2b_delay_horizontal_range' runSuffix '.mat']);
outputMat = fullfile(outputDir, ...
    ['step3_range_accuracy' runSuffix '.mat']);
comparisonCsv = fullfile(outputDir, ...
    ['step3_range_comparison' runSuffix '.csv']);
overallCsv = fullfile(outputDir, ...
    ['step3_accuracy_overall' runSuffix '.csv']);
byIdCsv = fullfile(outputDir, ...
    ['step3_accuracy_by_id' runSuffix '.csv']);
workbookFile = fullfile(outputDir, ...
    ['step3_range_accuracy' runSuffix '.xlsx']);
figurePng = fullfile(outputDir, ...
    ['step3_range_accuracy' runSuffix '.png']);
figureFig = fullfile(outputDir, ...
    ['step3_range_accuracy' runSuffix '.fig']);

mustBeFile(step2aFile);
mustBeFile(step2bFile);
geometry = load(step2aFile, 'usblRange', 'beaconCalibration', ...
    'positionPreprocessingSummary');
delay = load(step2bFile, 'delayRange', 'channelPreprocessingSummary', ...
    'turnaroundConvention');
usblRange = geometry.usblRange;
delayRange = delay.delayRange;

%% 按唯一观测键对齐两个阶段
% 不依赖两个MAT文件中的行顺序，避免后续筛选或排序造成错行比较。
[found, delayIndex] = ismember(usblRange.observation_key, ...
    delayRange.observation_key);
if any(~found) || numel(unique(delayIndex)) ~= height(usblRange)
    error('USBL:StageAlignment', ...
        'Step 2A与Step 2B的观测键无法一一对应。');
end
delayRange = delayRange(delayIndex, :);
n = height(usblRange);

%% 形成统一评估矩阵
methodNames = ["PTSAX"; "PTSAG"; "Delay-Equation"; "Delay-Zhou"; ...
    "Delay-AMS"; "Delay-GMS"];
referenceHorizontalM = usblRange.reference_horizontal_m;
estimateMatrix = [usblRange.filtered_ptsax_horizontal_m, ...
    usblRange.filtered_ptsag_horizontal_m, ...
    delayRange.equation_horizontal_m, delayRange.zhou_horizontal_m, ...
    delayRange.ams_horizontal_m, delayRange.gms_horizontal_m];
errorMatrix = estimateMatrix - referenceHorizontalM;

methodSummary = summarize_methods(usblRange.beacon_id, ...
    referenceHorizontalM, estimateMatrix, methodNames);
overallMethodSummary = summarize_methods(ones(n, 1), ...
    referenceHorizontalM, estimateMatrix, methodNames);
overallMethodSummary.beacon_id = [];

%% 保存逐条对比结果
rangeComparison = table(usblRange.observation_key, usblRange.source_name, ...
    usblRange.utc_time, usblRange.local_time_utc8, usblRange.beacon_id, ...
    referenceHorizontalM, estimateMatrix(:, 1), estimateMatrix(:, 2), ...
    estimateMatrix(:, 3), estimateMatrix(:, 4), estimateMatrix(:, 5), ...
    estimateMatrix(:, 6), errorMatrix(:, 1), errorMatrix(:, 2), ...
    errorMatrix(:, 3), errorMatrix(:, 4), errorMatrix(:, 5), ...
    errorMatrix(:, 6), usblRange.ptsax_outlier, ...
    usblRange.ptsag_outlier, delayRange.delay_observation_outlier, ...
    'VariableNames', {'observation_key', 'source_name', 'utc_time', ...
    'local_time_utc8', 'beacon_id', 'reference_horizontal_m', ...
    'ptsax_horizontal_m', 'ptsag_horizontal_m', ...
    'delay_equation_horizontal_m', 'delay_zhou_horizontal_m', ...
    'delay_ams_horizontal_m', 'delay_gms_horizontal_m', ...
    'ptsax_error_m', 'ptsag_error_m', 'delay_equation_error_m', ...
    'delay_zhou_error_m', 'delay_ams_error_m', 'delay_gms_error_m', ...
    'ptsax_outlier', 'ptsag_outlier', 'delay_observation_outlier'});

%% 绘制距离、误差和总体RMSE
x = (1:n)';
% fig = figure('Visible', 'on', 'Color', 'w', ...
%     'Position', [100 100 1550 920]);
fig = myfigurestartup(7,7,'paper');
tiledlayout(fig, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(x, referenceHorizontalM, 'k-', 'LineWidth', 1.2); hold on;
plot(x, estimateMatrix(:, 1), 'Color', [0 0.45 0.74]);
plot(x, estimateMatrix(:, 2), 'Color', [0.85 0.33 0.10]);
grid on; xlabel('观测序号'); ylabel('水平距离 / m');
title('USBL几何水平距离');
legend('参考距离', 'PTSAX', 'PTSAG', 'Location', 'best');

nexttile;
plot(x, errorMatrix(:, 1), '.'); hold on;
plot(x, errorMatrix(:, 2), '.'); yline(0, 'k-');
grid on; xlabel('观测序号'); ylabel('误差 / m');
title('PTSAX与PTSAG水平距离误差');
legend('PTSAX', 'PTSAG', 'Location', 'best');

nexttile;
plot(x, errorMatrix(:, 3), '.'); hold on;
plot(x, errorMatrix(:, 4), '.'); plot(x, errorMatrix(:, 5), '.');
plot(x, errorMatrix(:, 6), '.'); yline(0, 'k-');
grid on; xlabel('观测序号'); ylabel('误差 / m');
title('传播时延四种算法水平距离误差');
legend('Equation', 'Zhou', 'AMS', 'GMS', 'Location', 'best');

nexttile;
bar(categorical(overallMethodSummary.method, methodNames), ...
    overallMethodSummary.rmse_m);
grid on; ylabel('RMSE / m'); title('各方法总体RMSE');

exportgraphics(fig, figurePng, 'Resolution', 180);
savefig(fig, figureFig);
% close(fig);

%% 导出表格与MAT
writetable(rangeComparison, comparisonCsv);
writetable(overallMethodSummary, overallCsv);
writetable(methodSummary, byIdCsv);
if isfile(workbookFile)
    delete(workbookFile);
end
writetable(overallMethodSummary, workbookFile, 'Sheet', 'Overall');
writetable(methodSummary, workbookFile, 'Sheet', 'ByID');
writetable(geometry.beaconCalibration, workbookFile, ...
    'Sheet', 'BeaconCalibration');
writetable(geometry.positionPreprocessingSummary, workbookFile, ...
    'Sheet', 'PositionPreprocessing');
writetable(delay.channelPreprocessingSummary, workbookFile, ...
    'Sheet', 'DelayPreprocessing');
writetable(delay.turnaroundConvention, workbookFile, ...
    'Sheet', 'TurnaroundConvention');

settings = struct;
settings.run_beacon_id = runBeaconId;
settings.single_id_run = singleIdRun;
settings.step2a_file = step2aFile;
settings.step2b_file = step2bFile;
settings.distance_type = "全部为水平距离";
settings.reference_definition = ...
    "PPC船位到PTSAG点云稳健中位数信标位置的WGS-84水平距离";
settings.created_at = datetime('now', 'TimeZone', 'Asia/Shanghai');
beaconCalibration = geometry.beaconCalibration;
positionPreprocessingSummary = geometry.positionPreprocessingSummary;
channelPreprocessingSummary = delay.channelPreprocessingSummary;
turnaroundConvention = delay.turnaroundConvention;
save(outputMat, 'rangeComparison', 'methodSummary', ...
    'overallMethodSummary', 'beaconCalibration', ...
    'positionPreprocessingSummary', 'channelPreprocessingSummary', ...
    'turnaroundConvention', 'settings', '-v7.3');

fprintf('\nStep 3完成：水平测距精度评估结果已保存。\n');
fprintf('  观测数：%d\n', n);
fprintf('  Excel：%s\n', workbookFile);
fprintf('  图：%s\n\n', figurePng);
disp(methodSummary);
disp(overallMethodSummary);

%% 本脚本使用的局部函数
function [suffix, singleId] = make_suffix(id)
singleId = ~isempty(id);
if singleId
    if ~isnumeric(id) || ~isscalar(id) || ~isfinite(id) || fix(id) ~= id
        error('USBL:InvalidBeaconId', 'runBeaconId必须为一个整数或[]。');
    end
    suffix = sprintf('_id%d', id);
else
    suffix = '';
end
end

function summary = summarize_methods(beaconIds, referenceM, ...
        estimateMatrix, methodNames)
uniqueIds = unique(beaconIds);
nRows = numel(uniqueIds) * numel(methodNames);
beaconId = nan(nRows, 1); method = strings(nRows, 1);
validCount = zeros(nRows, 1); meanErrorM = nan(nRows, 1);
medianErrorM = nan(nRows, 1); stdErrorM = nan(nRows, 1);
maeM = nan(nRows, 1); rmseM = nan(nRows, 1);
p95AbsoluteErrorM = nan(nRows, 1); row = 0;
for idIndex = 1:numel(uniqueIds)
    useId = beaconIds == uniqueIds(idIndex);
    for methodIndex = 1:numel(methodNames)
        row = row + 1;
        errorM = estimateMatrix(useId, methodIndex) - referenceM(useId);
        errorM = errorM(isfinite(errorM));
        beaconId(row) = uniqueIds(idIndex);
        method(row) = methodNames(methodIndex);
        validCount(row) = numel(errorM);
        if ~isempty(errorM)
            meanErrorM(row) = mean(errorM);
            medianErrorM(row) = median(errorM);
            stdErrorM(row) = std(errorM);
            maeM(row) = mean(abs(errorM));
            rmseM(row) = sqrt(mean(errorM.^2));
            p95AbsoluteErrorM(row) = prctile(abs(errorM), 95);
        end
    end
end
summary = table(beaconId, method, validCount, meanErrorM, ...
    medianErrorM, stdErrorM, maeM, rmseM, p95AbsoluteErrorM, ...
    'VariableNames', {'beacon_id', 'method', 'valid_count', ...
    'mean_error_m', 'median_error_m', 'std_error_m', 'mae_m', ...
    'rmse_m', 'p95_absolute_error_m'});
end
