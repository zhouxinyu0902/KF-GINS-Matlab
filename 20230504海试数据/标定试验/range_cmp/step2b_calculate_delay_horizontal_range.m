%% Step 2B：由四通道传播时延计算水平距离
% 本文件是可直接运行的脚本，不是函数。请先运行Step 2A。
%
% 作用
%   1. 读取step1中的四通道PIXOG/DBG/DETEC传播时延；
%   2. 统一20 ms应答周转时延和单双程时间基准；
%   3. 清除缺失通道、通道离群值和孤立时间突跳；
%   4. 使用实测SVP分别计算Equation、Zhou、AMS、GMS水平距离；
%   5. 同时保存四通道结果和CH1--CH3融合结果。
%
% ID选择
%   与Step 2A保持一致。单独运行时默认ID93；也可在运行前设置：
%     runBeaconId = 93;
%     runBeaconId = [];
%
% 特殊处理说明
%   - 原始时延按双程时间处理，转换公式为：
%       单程传播时间 = (原始双程时间 - 应答周转时间) / 2
%   - 早期文件中CH0已扣除20 ms，而CH1--CH3未扣除；后期文件中
%     四个通道均含20 ms。脚本按源文件自动识别这两种约定。
%   - 通道一致性：校正后通道偏离四通道中位数超过1 ms则剔除。
%   - 主结果使用CH1--CH3中位数；至少需要两个有效主通道。
%   - 时间序列孤立突跳阈值为max(5 ms, 6倍局部稳健标准差)。
%   - SVP下降段和上升段按1 m深度箱取声速中位数；剖面末端未覆盖的
%     几米使用最深10 m声速的稳健中位数作常值延伸。

if ~exist('runBeaconId', 'var')
    runBeaconId = 93;
end
clearvars -except runBeaconId;
clc;

%% 路径与输入输出文件
scriptDir = fileparts(mfilename('fullpath'));
calibrationScriptDir = fileparts(scriptDir);
legacyDataDir = fileparts(calibrationScriptDir);
repositoryDir = fileparts(legacyDataDir);
analysisDir = string(fullfile(repositoryDir, 'data', '20230504', ...
    '标定实验分析'));
outputDir = fullfile(analysisDir, '解析后的数据', 'USBL_range_cmp');
svpFile = fullfile(analysisDir, '信标标定试验数据', ...
    'svp2023-04-27.txt');

[step1File, runSuffix, singleIdRun] = select_step1_input( ...
    outputDir, runBeaconId);
step2aFile = fullfile(outputDir, ...
    ['step2a_usbl_horizontal_range' runSuffix '.mat']);
outputMat = fullfile(outputDir, ...
    ['step2b_delay_horizontal_range' runSuffix '.mat']);
outputCsv = fullfile(outputDir, ...
    ['step2b_delay_horizontal_range' runSuffix '.csv']);
channelCsv = fullfile(outputDir, ...
    ['step2b_delay_channel_preprocessing' runSuffix '.csv']);
conventionCsv = fullfile(outputDir, ...
    ['step2b_turnaround_convention' runSuffix '.csv']);

mustBeFile(step1File);
mustBeFile(step2aFile);
mustBeFile(svpFile);
core = load(step1File, 'usblSolutions', 'travelTimeFourChannel');
geometry = load(step2aFile, 'usblRange', 'beaconCalibration');
if ~isfield(core, 'usblSolutions') || ...
        ~isfield(core, 'travelTimeFourChannel') || ...
        ~isfield(geometry, 'usblRange')
    error('USBL:MissingInput', 'Step 1或Step 2A文件缺少必要变量。');
end
usblSolutions = core.usblSolutions;
travelTimeFourChannel = core.travelTimeFourChannel;
usblRange = geometry.usblRange;
beaconCalibration = geometry.beaconCalibration;
if singleIdRun
    usblSolutions = usblSolutions( ...
        usblSolutions.transponder_id == runBeaconId, :);
    travelTimeFourChannel = travelTimeFourChannel( ...
        travelTimeFourChannel.transponder_id == runBeaconId, :);
end
n = height(usblRange);
if height(usblSolutions) ~= n
    error('USBL:StageMismatch', 'Step 1与Step 2A观测数量不一致。');
end

%% 按源文件和PPC行号关联四通道时延
solutionKey = usblRange.source_name + "|" + ...
    string(usblRange.ppc_line_number);
linked = isfinite(travelTimeFourChannel.ppc_line_number);
fourChannelKey = travelTimeFourChannel.source_name(linked) + "|" + ...
    string(travelTimeFourChannel.ppc_line_number(linked));
[found, location] = ismember(solutionKey, fourChannelKey);
if any(~found)
    error('USBL:UnmatchedDelay', '%d条观测没有四通道传播时延。', ...
        sum(~found));
end
linkedRows = find(linked);
travelIndex = linkedRows(location);
rawRoundTripTimeS = [ ...
    travelTimeFourChannel.ch0_travel_time_s(travelIndex), ...
    travelTimeFourChannel.ch1_travel_time_s(travelIndex), ...
    travelTimeFourChannel.ch2_travel_time_s(travelIndex), ...
    travelTimeFourChannel.ch3_travel_time_s(travelIndex)];

%% 20 ms周转时间与单双程基准统一
turnaroundTimeS = 0.020;
[turnaroundCorrectionS, turnaroundConvention] = ...
    infer_turnaround_correction(rawRoundTripTimeS, ...
    usblRange.source_name, turnaroundTimeS);
propagationRoundTripTimeS = rawRoundTripTimeS - turnaroundCorrectionS;

%% 四通道异常值预处理
channelConsistencyThresholdS = 0.001;
channelMedianTimeS = median(propagationRoundTripTimeS, 2, 'omitnan');
channelConsistencyResidualS = abs(propagationRoundTripTimeS - ...
    channelMedianTimeS);
channelMissing = ~isfinite(propagationRoundTripTimeS);
channelOutlier = isfinite(propagationRoundTripTimeS) & ...
    channelConsistencyResidualS > channelConsistencyThresholdS;
channelRejected = channelMissing | channelOutlier;

filteredPropagationRoundTripTimeS = propagationRoundTripTimeS;
filteredPropagationRoundTripTimeS(channelRejected) = NaN;
validMainChannelCount = sum(isfinite( ...
    filteredPropagationRoundTripTimeS(:, 2:4)), 2);
preliminaryFusedTimeS = median( ...
    filteredPropagationRoundTripTimeS(:, 2:4), 2, 'omitnan');
temporalOutlier = detect_local_outliers(preliminaryFusedTimeS, ...
    usblRange.source_name, usblRange.utc_time, usblRange.beacon_id, ...
    5, 0.005, 6);
insufficientMainChannels = validMainChannelCount < 2;
delayObservationOutlier = temporalOutlier | insufficientMainChannels;
filteredPropagationRoundTripTimeS(delayObservationOutlier, :) = NaN;
oneWayTimeS = filteredPropagationRoundTripTimeS / 2;

%% 读取并整理实测声速剖面
svp = read_svp_file(svpFile);
maximumBeaconDepthM = max(usblRange.beacon_depth_m);
if max(svp.depth_m) < maximumBeaconDepthM
    deepestSpeedMps = median(svp.sound_speed_mps( ...
        svp.depth_m >= max(svp.depth_m) - 10), 'omitnan');
    svp = [svp; table(maximumBeaconDepthM, deepestSpeedMps, ...
        'VariableNames', svp.Properties.VariableNames)];
end

%% 四通道乘四种SVP方法计算水平距离
equationHorizontalM = nan(n, 4);
zhouHorizontalM = nan(n, 4);
amsHorizontalM = nan(n, 4);
gmsHorizontalM = nan(n, 4);
for row = 1:n
    for channelIndex = 1:4
        [equationHorizontalM(row, channelIndex), ...
            zhouHorizontalM(row, channelIndex), ...
            amsHorizontalM(row, channelIndex), ...
            gmsHorizontalM(row, channelIndex)] = ...
            four_svp_horizontal_ranges(usblRange.ship_depth_m(row), ...
            usblRange.beacon_depth_m(row), ...
            oneWayTimeS(row, channelIndex), svp);
    end
end

%% CH1--CH3融合结果
% PPC/DETEC实际保存的主要测量来自CH1--CH3，因此主结果不使用CH0，
% 而是对三个有效主通道的校正后双程时间取中位数。
fusedPropagationRoundTripTimeS = median( ...
    filteredPropagationRoundTripTimeS(:, 2:4), 2, 'omitnan');
fusedOneWayTimeS = fusedPropagationRoundTripTimeS / 2;
fusedEquationHorizontalM = nan(n, 1);
fusedZhouHorizontalM = nan(n, 1);
fusedAmsHorizontalM = nan(n, 1);
fusedGmsHorizontalM = nan(n, 1);
for row = 1:n
    [fusedEquationHorizontalM(row), fusedZhouHorizontalM(row), ...
        fusedAmsHorizontalM(row), fusedGmsHorizontalM(row)] = ...
        four_svp_horizontal_ranges(usblRange.ship_depth_m(row), ...
        usblRange.beacon_depth_m(row), fusedOneWayTimeS(row), svp);
end

%% 保存一行一条观测的融合结果
delayRange = table(usblRange.observation_key, usblRange.source_name, ...
    usblRange.utc_time, usblRange.local_time_utc8, usblRange.beacon_id, ...
    validMainChannelCount, temporalOutlier, insufficientMainChannels, ...
    delayObservationOutlier, rawRoundTripTimeS(:, 1), ...
    rawRoundTripTimeS(:, 2), rawRoundTripTimeS(:, 3), ...
    rawRoundTripTimeS(:, 4), fusedPropagationRoundTripTimeS, ...
    fusedOneWayTimeS, fusedEquationHorizontalM, fusedZhouHorizontalM, ...
    fusedAmsHorizontalM, fusedGmsHorizontalM, ...
    'VariableNames', {'observation_key', 'source_name', 'utc_time', ...
    'local_time_utc8', 'beacon_id', 'valid_main_channel_count', ...
    'temporal_time_outlier', 'insufficient_main_channels', ...
    'delay_observation_outlier', 'ch0_raw_round_trip_time_s', ...
    'ch1_raw_round_trip_time_s', 'ch2_raw_round_trip_time_s', ...
    'ch3_raw_round_trip_time_s', ...
    'fused_propagation_round_trip_time_s', 'fused_one_way_time_s', ...
    'equation_horizontal_m', 'zhou_horizontal_m', ...
    'ams_horizontal_m', 'gms_horizontal_m'});

%% 保存四通道明细长表
observationIndex = repelem((1:n)', 4);
channel = repmat((0:3)', n, 1);
delayRangeByChannel = table(observationIndex, ...
    repelem(usblRange.observation_key, 4), ...
    repelem(usblRange.source_name, 4), repelem(usblRange.utc_time, 4), ...
    repelem(usblRange.beacon_id, 4), channel, ...
    reshape(rawRoundTripTimeS.', [], 1), ...
    reshape(turnaroundCorrectionS.', [], 1), ...
    reshape(propagationRoundTripTimeS.', [], 1), ...
    reshape(channelConsistencyResidualS.', [], 1), ...
    reshape(channelMissing.', [], 1), reshape(channelOutlier.', [], 1), ...
    repelem(delayObservationOutlier, 4), ...
    reshape(oneWayTimeS.', [], 1), ...
    reshape(equationHorizontalM.', [], 1), ...
    reshape(zhouHorizontalM.', [], 1), reshape(amsHorizontalM.', [], 1), ...
    reshape(gmsHorizontalM.', [], 1), ...
    'VariableNames', {'observation_index', 'observation_key', ...
    'source_name', 'utc_time', 'beacon_id', 'channel', ...
    'raw_round_trip_time_s', 'turnaround_correction_s', ...
    'propagation_round_trip_time_s', ...
    'channel_consistency_residual_s', 'channel_missing', ...
    'channel_outlier', 'delay_observation_outlier', 'one_way_time_s', ...
    'equation_horizontal_m', 'zhou_horizontal_m', ...
    'ams_horizontal_m', 'gms_horizontal_m'});

channelNumber = (0:3)';
channelPreprocessingSummary = table(channelNumber, repmat(n, 4, 1), ...
    sum(channelMissing, 1)', sum(channelOutlier, 1)', ...
    (repmat(n, 4, 1) - sum(channelMissing, 1)' - ...
    sum(channelOutlier, 1)'), ...
    'VariableNames', {'channel', 'total_count', 'missing_count', ...
    'removed_outlier_count', 'retained_count'});

settings = struct;
settings.run_beacon_id = runBeaconId;
settings.step1_file = step1File;
settings.step2a_file = step2aFile;
settings.svp_file = svpFile;
settings.transponder_turnaround_time_s = turnaroundTimeS;
settings.time_definition = ...
    "单程传播时间=(原始双程时间-按文件识别的周转时间)/2";
settings.main_channel_fusion = "校正后CH1--CH3双程传播时间中位数";
settings.channel_consistency_threshold_s = channelConsistencyThresholdS;
settings.temporal_minimum_threshold_s = 0.005;
settings.created_at = datetime('now', 'TimeZone', 'Asia/Shanghai');

writetable(delayRange, outputCsv);
writetable(channelPreprocessingSummary, channelCsv);
writetable(turnaroundConvention, conventionCsv);
save(outputMat, 'delayRange', 'delayRangeByChannel', ...
    'channelPreprocessingSummary', 'turnaroundConvention', 'svp', ...
    'settings', '-v7.3');

fprintf('\nStep 2B完成：传播时延水平距离已保存。\n');
fprintf('  观测数：%d\n', n);
fprintf('  通道异常值：%d\n', sum(channelOutlier, 'all'));
fprintf('  整条时延观测剔除：%d\n', sum(delayObservationOutlier));
fprintf('  MAT：%s\n', outputMat);
fprintf('  CSV：%s\n\n', outputCsv);
disp(turnaroundConvention);
disp(channelPreprocessingSummary);

%% 本脚本使用的局部函数
function [inputFile, suffix, singleId] = select_step1_input(outputDir, id)
singleId = ~isempty(id);
if singleId
    if ~isnumeric(id) || ~isscalar(id) || ~isfinite(id) || fix(id) ~= id
        error('USBL:InvalidBeaconId', 'runBeaconId必须为一个整数或[]。');
    end
    suffix = sprintf('_id%d', id);
    inputFile = string(fullfile(outputDir, ...
        sprintf('step1_usbl_core_id%d.mat', id)));
else
    suffix = '';
    inputFile = string(fullfile(outputDir, 'step1_usbl_core.mat'));
end
end

function svp = read_svp_file(filename)
fileId = fopen(filename, 'rt');
if fileId < 0, error('USBL:CannotOpenSVP', '无法打开SVP：%s', filename); end
firstLine = strtrim(fgetl(fileId)); fclose(fileId);
if ~isempty(regexp(firstLine, '^\d{4}-\d{2}-\d{2}\s', 'once'))
    fileId = fopen(filename, 'rt'); cleanup = onCleanup(@() fclose(fileId));
    columns = textscan(fileId, '%s %s %f %f %f', ...
        'Delimiter', ' ', 'MultipleDelimsAsOne', true);
    rawDepthM = columns{3}; rawSpeedMps = columns{4};
    valid = isfinite(rawDepthM) & isfinite(rawSpeedMps) & ...
        rawDepthM >= 0 & rawSpeedMps > 1300 & rawSpeedMps < 1700;
    rawDepthM = rawDepthM(valid); rawSpeedMps = rawSpeedMps(valid);
    [depthM, ~, group] = unique(round(rawDepthM));
    speedMps = splitapply(@median, rawSpeedMps, group);
    if depthM(1) > 0
        depthM = [0; depthM]; speedMps = [speedMps(1); speedMps];
    end
else
    raw = readlines(filename, 'EmptyLineRule', 'skip');
    depthM = nan(numel(raw), 1); speedMps = nan(numel(raw), 1); row = 0;
    for k = 1:numel(raw)
        values = sscanf(char(raw(k)), '%f %f /');
        if numel(values) >= 2
            row = row + 1; depthM(row) = values(1); speedMps(row) = values(2);
        end
    end
    depthM = depthM(1:row); speedMps = speedMps(1:row);
    [depthM, order] = sort(depthM); speedMps = speedMps(order);
end
svp = table(depthM, speedMps, ...
    'VariableNames', {'depth_m', 'sound_speed_mps'});
end

function [correctionS, convention] = infer_turnaround_correction( ...
        rawTimeS, sourceName, turnaroundTimeS)
correctionS = nan(size(rawTimeS)); sources = unique(sourceName);
nSources = numel(sources); observationCount = zeros(nSources, 1);
medianDifferenceS = nan(nSources, 1); inferredMode = strings(nSources, 1);
ch0CorrectionS = nan(nSources, 1);
mainCorrectionS = turnaroundTimeS .* ones(nSources, 1);
for k = 1:nSources
    use = sourceName == sources(k); observationCount(k) = sum(use);
    mainMedianS = median(rawTimeS(use, 2:4), 2, 'omitnan');
    medianDifferenceS(k) = median( ...
        mainMedianS - rawTimeS(use, 1), 'omitnan');
    if abs(medianDifferenceS(k) - turnaroundTimeS) < ...
            abs(medianDifferenceS(k))
        correctionS(use, :) = repmat([0 turnaroundTimeS turnaroundTimeS ...
            turnaroundTimeS], sum(use), 1);
        inferredMode(k) = "CH0已扣周转时间，CH1--CH3含周转时间";
        ch0CorrectionS(k) = 0;
    else
        correctionS(use, :) = turnaroundTimeS;
        inferredMode(k) = "四通道均含周转时间";
        ch0CorrectionS(k) = turnaroundTimeS;
    end
end
convention = table(sources, observationCount, medianDifferenceS, ...
    inferredMode, ch0CorrectionS, mainCorrectionS, ...
    'VariableNames', {'source_name', 'observation_count', ...
    'median_ch1_ch3_minus_ch0_s', 'inferred_mode', ...
    'ch0_correction_s', 'ch1_ch3_correction_s'});
end

function [equationM, zhouM, amsM, gmsM] = ...
        four_svp_horizontal_ranges(sourceDepthM, receiverDepthM, ...
        oneWayTimeS, svp)
equationM = NaN; zhouM = NaN; amsM = NaN; gmsM = NaN;
if ~all(isfinite([sourceDepthM receiverDepthM oneWayTimeS])) || ...
        oneWayTimeS <= 0
    return;
end
depthDifferenceM = abs(receiverDepthM - sourceDepthM);
if depthDifferenceM < eps
    speedMps = interp1(svp.depth_m, svp.sound_speed_mps, ...
        sourceDepthM, 'linear', 'extrap');
    equationM = oneWayTimeS * speedMps; zhouM = equationM;
    amsM = equationM; gmsM = equationM; return;
end
minimumDepthM = min(sourceDepthM, receiverDepthM);
maximumDepthM = max(sourceDepthM, receiverDepthM);
internal = svp.depth_m > minimumDepthM & svp.depth_m < maximumDepthM;
profileDepthM = [minimumDepthM; svp.depth_m(internal); maximumDepthM];
profileSpeedMps = interp1(svp.depth_m, svp.sound_speed_mps, ...
    profileDepthM, 'linear', 'extrap');
sourceSpeedMps = interp1(svp.depth_m, svp.sound_speed_mps, ...
    sourceDepthM, 'linear', 'extrap');
speedIntegral = trapz(profileDepthM, profileSpeedMps);
arithmeticMeanMps = speedIntegral / depthDifferenceM;
harmonicMeanMps = depthDifferenceM / ...
    trapz(profileDepthM, 1 ./ profileSpeedMps);
amsM = safe_sqrt((oneWayTimeS * arithmeticMeanMps)^2 - depthDifferenceM^2);
gmsM = safe_sqrt((oneWayTimeS * harmonicMeanMps)^2 - depthDifferenceM^2);

% Equation：按参考range.m中的线性声速梯度解析式。
if sourceDepthM < receiverDepthM
    gradient = 2 * speedIntegral / depthDifferenceM^2 - ...
        2 * sourceSpeedMps / depthDifferenceM;
else
    gradient = 2 * sourceSpeedMps / depthDifferenceM - ...
        2 * speedIntegral / depthDifferenceM^2;
end
if abs(gradient) < 1e-12
    equationM = amsM;
else
    receiverSpeedMps = sourceSpeedMps + gradient * ...
        (receiverDepthM - sourceDepthM);
    exponential = exp(oneWayTimeS * gradient);
    expression = (sourceSpeedMps * exponential - receiverSpeedMps) * ...
        (sourceSpeedMps - receiverSpeedMps * exponential) / exponential;
    equationM = abs(1 / gradient) * sqrt(abs(expression));
end

% Zhou：通过二分法求射线参数，再沿深度积分水平距离。
verticalTimeS = trapz(profileDepthM, 1 ./ profileSpeedMps);
if oneWayTimeS < verticalTimeS, return; end
lower = 0; upper = (1 - 1e-12) / max(profileSpeedMps);
rayParameter = 0;
for iteration = 1:100
    rayParameter = (lower + upper) / 2;
    denominator = sqrt(1 - (rayParameter * profileSpeedMps).^2);
    calculatedTimeS = trapz(profileDepthM, ...
        (1 ./ profileSpeedMps) ./ denominator);
    if abs(calculatedTimeS - oneWayTimeS) < 1e-8
        break;
    elseif calculatedTimeS > oneWayTimeS
        upper = rayParameter;
    else
        lower = rayParameter;
    end
end
denominator = sqrt(1 - (rayParameter * profileSpeedMps).^2);
zhouM = trapz(profileDepthM, ...
    rayParameter * profileSpeedMps ./ denominator);
end

function value = safe_sqrt(valueSquared)
if isfinite(valueSquared) && valueSquared >= 0
    value = sqrt(valueSquared);
else
    value = NaN;
end
end

function outlier = detect_local_outliers(values, sources, times, ids, ...
        halfWindow, minimumThreshold, robustSigmaFactor)
outlier = false(size(values));
for source = unique(sources)'
    sourceRows = sources == source;
    for id = unique(ids(sourceRows))'
        rows = find(sourceRows & ids == id);
        [~, order] = sort(times(rows)); rows = rows(order);
        if numel(rows) < 2 * halfWindow + 1, continue; end
        for p = (halfWindow + 1):(numel(rows) - halfWindow)
            localRows = rows(p-halfWindow:p+halfWindow);
            centerRow = rows(p); localRows(halfWindow + 1) = [];
            neighbors = values(localRows); neighbors = neighbors(isfinite(neighbors));
            if ~isfinite(values(centerRow)) || numel(neighbors) < halfWindow
                continue;
            end
            center = median(neighbors);
            sigma = 1.4826 * median(abs(neighbors - center));
            threshold = max(minimumThreshold, robustSigmaFactor * sigma);
            outlier(centerRow) = abs(values(centerRow) - center) > threshold;
        end
    end
end
end
