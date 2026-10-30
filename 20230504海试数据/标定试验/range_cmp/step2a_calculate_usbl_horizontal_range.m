%% Step 2A：由USBL定位报文计算水平距离
% 本文件是可直接运行的脚本，不是函数。
%
% 作用
%   1. 从step1结果读取PTSAX、PTSAY、PTSAG和PPC船位；
%   2. 按信标ID自动计算稳健的信标标定坐标；
%   3. 分别计算参考水平距离、PTSAX水平距离和PTSAG水平距离；
%   4. 对PTSAX、PTSAG中的明显异常值进行标记并保存。
%
% ID选择
%   单独运行时，默认处理ID93。可在运行前执行：
%     runBeaconId = 93;   % 只处理ID93
%     runBeaconId = [];   % 处理step1合并文件内的全部ID
%
% 特殊处理说明
%   - 参考距离：船舶PPC位置到自动标定信标位置的WGS-84水平距离。
%   - 自动标定：先对PTSAG经纬度/深度作稳健粗差筛选，再取中位数。
%     这是USBL点云的稳健汇总，不是独立的声学网络反演真值。
%   - PTSAX：由前向、右舷两个船体系分量取模；航向只用于转换到东北坐标。
%   - PTSAG：由PPC船位与PTSAG信标经纬度计算WGS-84局部水平距离。
%   - 异常值：参考距离差超过100 m，或局部突跳超过
%     max(50 m, 6倍局部稳健标准差)时标记为异常并置NaN。

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
if ~isfolder(outputDir)
    mkdir(outputDir);
end

[inputFile, runSuffix, singleIdRun] = select_step1_input( ...
    outputDir, runBeaconId);
outputMat = fullfile(outputDir, ...
    ['step2a_usbl_horizontal_range' runSuffix '.mat']);
outputCsv = fullfile(outputDir, ...
    ['step2a_usbl_horizontal_range' runSuffix '.csv']);
calibrationCsv = fullfile(outputDir, ...
    ['step2a_beacon_calibration' runSuffix '.csv']);
preprocessingCsv = fullfile(outputDir, ...
    ['step2a_position_preprocessing' runSuffix '.csv']);

mustBeFile(inputFile);
loaded = load(inputFile, 'usblSolutions', 'ppcDetection');
if ~isfield(loaded, 'usblSolutions') || ~isfield(loaded, 'ppcDetection')
    error('USBL:MissingInput', 'step1文件缺少usblSolutions或ppcDetection。');
end
usblSolutions = loaded.usblSolutions;
ppcDetection = loaded.ppcDetection;

if singleIdRun
    usblSolutions = usblSolutions( ...
        usblSolutions.transponder_id == runBeaconId, :);
    ppcDetection = ppcDetection( ...
        ppcDetection.transponder_id == runBeaconId, :);
end
if isempty(usblSolutions)
    error('USBL:NoSelectedData', '所选ID没有USBL解算记录。');
end
n = height(usblSolutions);

%% 自动计算信标标定坐标
[beaconCalibration, calibrationPositionOutlier] = ...
    estimate_beacon_calibration(usblSolutions);
writetable(beaconCalibration, calibrationCsv);

%% 将每条USBL解与最近的PPC船位记录配对
% 限定同一源文件、同一信标ID，最大时间差为0.02 s，防止跨批次误配。
[ppcIndex, ppcTimeDifferenceS] = nearest_source_id_time( ...
    usblSolutions.source_name, usblSolutions.utc_time, ...
    usblSolutions.transponder_id, ppcDetection.source_name, ...
    ppcDetection.utc_time, ppcDetection.transponder_id, 0.02);
if any(~isfinite(ppcIndex))
    error('USBL:UnmatchedPPC', '%d条USBL记录未找到匹配PPC记录。', ...
        sum(~isfinite(ppcIndex)));
end
ppcLineNumber = ppcDetection.line_number(ppcIndex);
observationKey = usblSolutions.source_name + "|" + string(ppcLineNumber);

shipLatitudeDeg = usblSolutions.ppc_ship_latitude_deg;
shipLongitudeDeg = usblSolutions.ppc_ship_longitude_deg;
shipDepthM = usblSolutions.ppc_ship_depth_m;
shipHeadingDeg = usblSolutions.ppc_ship_heading_deg;
[beaconLatitudeDeg, beaconLongitudeDeg, beaconDepthM] = ...
    lookup_beacon_calibration(usblSolutions.transponder_id, ...
    beaconCalibration);

%% 参考水平距离：最终信标坐标与PPC船位
[referenceEastM, referenceNorthM, referenceHorizontalM] = ...
    wgs84_local_offset(shipLatitudeDeg, shipLongitudeDeg, ...
    beaconLatitudeDeg, beaconLongitudeDeg);

%% PTSAX水平距离
bodyForwardM = usblSolutions.body_forward_m;
bodyStarboardM = usblSolutions.body_starboard_m;
ptsaxHorizontalM = hypot(bodyForwardM, bodyStarboardM);
ptsaxNorthM = bodyForwardM .* cosd(shipHeadingDeg) - ...
    bodyStarboardM .* sind(shipHeadingDeg);
ptsaxEastM = bodyForwardM .* sind(shipHeadingDeg) + ...
    bodyStarboardM .* cosd(shipHeadingDeg);

% 先做局部突跳检测，再用参考距离作100 m量级的粗差门限。
positionReferenceThresholdM = 100;
positionLocalMinimumThresholdM = 50;
ptsaxTemporalOutlier = detect_local_outliers(ptsaxHorizontalM, ...
    usblSolutions.source_name, usblSolutions.utc_time, ...
    usblSolutions.transponder_id, 5, positionLocalMinimumThresholdM, 6);
ptsaxReferenceOutlier = isfinite(ptsaxHorizontalM) & ...
    abs(ptsaxHorizontalM - referenceHorizontalM) > ...
    positionReferenceThresholdM;
ptsaxOutlier = ~isfinite(ptsaxHorizontalM) | ptsaxTemporalOutlier | ...
    ptsaxReferenceOutlier;
filteredPtsaxHorizontalM = ptsaxHorizontalM;
filteredPtsaxHorizontalM(ptsaxOutlier) = NaN;

%% PTSAY一致性检查
% PTSAY已经是东北坐标分量，仅用于检查PTSAX结合航向后的方向是否一致。
ptsayNorthM = usblSolutions.north_offset_m;
ptsayEastM = usblSolutions.east_offset_m;
ptsayHorizontalM = hypot(ptsayNorthM, ptsayEastM);
ptsaxPtsayNorthDifferenceM = ptsaxNorthM - ptsayNorthM;
ptsaxPtsayEastDifferenceM = ptsaxEastM - ptsayEastM;

%% PTSAG水平距离
[ptsagEastM, ptsagNorthM, ptsagHorizontalM] = wgs84_local_offset( ...
    shipLatitudeDeg, shipLongitudeDeg, usblSolutions.latitude_deg, ...
    usblSolutions.longitude_deg);
ptsagTemporalOutlier = detect_local_outliers(ptsagHorizontalM, ...
    usblSolutions.source_name, usblSolutions.utc_time, ...
    usblSolutions.transponder_id, 5, positionLocalMinimumThresholdM, 6);
ptsagReferenceOutlier = isfinite(ptsagHorizontalM) & ...
    abs(ptsagHorizontalM - referenceHorizontalM) > ...
    positionReferenceThresholdM;
ptsagOutlier = ~isfinite(ptsagHorizontalM) | ptsagTemporalOutlier | ...
    ptsagReferenceOutlier;
filteredPtsagHorizontalM = ptsagHorizontalM;
filteredPtsagHorizontalM(ptsagOutlier) = NaN;

%% 保存一行一条观测的USBL水平距离表
usblRange = table(observationKey, usblSolutions.source_name, ...
    usblSolutions.utc_time, usblSolutions.local_time_utc8, ...
    usblSolutions.transponder_id, ppcLineNumber, ppcTimeDifferenceS, ...
    shipLatitudeDeg, shipLongitudeDeg, shipDepthM, shipHeadingDeg, ...
    beaconLatitudeDeg, beaconLongitudeDeg, beaconDepthM, ...
    calibrationPositionOutlier, referenceEastM, referenceNorthM, ...
    referenceHorizontalM, bodyForwardM, bodyStarboardM, ...
    ptsaxNorthM, ptsaxEastM, ptsaxHorizontalM, ...
    filteredPtsaxHorizontalM, ptsaxTemporalOutlier, ...
    ptsaxReferenceOutlier, ptsaxOutlier, ptsayNorthM, ptsayEastM, ...
    ptsayHorizontalM, ptsaxPtsayNorthDifferenceM, ...
    ptsaxPtsayEastDifferenceM, ptsagEastM, ptsagNorthM, ...
    ptsagHorizontalM, filteredPtsagHorizontalM, ptsagTemporalOutlier, ...
    ptsagReferenceOutlier, ptsagOutlier, ...
    'VariableNames', {'observation_key', 'source_name', 'utc_time', ...
    'local_time_utc8', 'beacon_id', 'ppc_line_number', ...
    'ppc_time_difference_s', 'ship_latitude_deg', ...
    'ship_longitude_deg', 'ship_depth_m', 'ship_heading_deg', ...
    'beacon_latitude_deg', 'beacon_longitude_deg', 'beacon_depth_m', ...
    'calibration_position_outlier', 'reference_east_m', ...
    'reference_north_m', 'reference_horizontal_m', ...
    'ptsax_body_forward_m', 'ptsax_body_starboard_m', ...
    'ptsax_north_m', 'ptsax_east_m', 'ptsax_horizontal_m', ...
    'filtered_ptsax_horizontal_m', 'ptsax_temporal_outlier', ...
    'ptsax_reference_outlier', 'ptsax_outlier', 'ptsay_north_m', ...
    'ptsay_east_m', 'ptsay_horizontal_m', ...
    'ptsax_ptsay_north_difference_m', ...
    'ptsax_ptsay_east_difference_m', 'ptsag_east_m', ...
    'ptsag_north_m', 'ptsag_horizontal_m', ...
    'filtered_ptsag_horizontal_m', 'ptsag_temporal_outlier', ...
    'ptsag_reference_outlier', 'ptsag_outlier'});

positionPreprocessingSummary = table(["PTSAX"; "PTSAG"], [n; n], ...
    [sum(~isfinite(ptsaxHorizontalM)); sum(~isfinite(ptsagHorizontalM))], ...
    [sum(ptsaxTemporalOutlier); sum(ptsagTemporalOutlier)], ...
    [sum(ptsaxReferenceOutlier); sum(ptsagReferenceOutlier)], ...
    [sum(ptsaxOutlier); sum(ptsagOutlier)], ...
    [sum(~ptsaxOutlier); sum(~ptsagOutlier)], ...
    'VariableNames', {'method', 'total_count', 'missing_count', ...
    'temporal_outlier_count', 'reference_outlier_count', ...
    'removed_count', 'retained_count'});

settings = struct;
settings.run_beacon_id = runBeaconId;
settings.input_file = inputFile;
settings.reference_definition = ...
    "PPC船位到PTSAG点云稳健中位数信标位置的WGS-84水平距离";
settings.position_reference_threshold_m = positionReferenceThresholdM;
settings.position_local_minimum_threshold_m = ...
    positionLocalMinimumThresholdM;
settings.created_at = datetime('now', 'TimeZone', 'Asia/Shanghai');

writetable(usblRange, outputCsv);
writetable(positionPreprocessingSummary, preprocessingCsv);
save(outputMat, 'usblRange', 'beaconCalibration', ...
    'positionPreprocessingSummary', 'settings', '-v7.3');

fprintf('\nStep 2A完成：USBL水平距离已保存。\n');
fprintf('  观测数：%d\n', n);
fprintf('  PTSAX剔除：%d\n', sum(ptsaxOutlier));
fprintf('  PTSAG剔除：%d\n', sum(ptsagOutlier));
fprintf('  MAT：%s\n', outputMat);
fprintf('  CSV：%s\n\n', outputCsv);
disp(beaconCalibration);
disp(positionPreprocessingSummary);

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

function [calibration, observationOutlier] = ...
        estimate_beacon_calibration(data)
ids = unique(data.transponder_id(isfinite(data.transponder_id)));
nIds = numel(ids);
totalCount = zeros(nIds, 1); validInputCount = zeros(nIds, 1);
retainedCount = zeros(nIds, 1); removedCount = zeros(nIds, 1);
latitudeDeg = nan(nIds, 1); longitudeDeg = nan(nIds, 1);
depthM = nan(nIds, 1); horizontalThresholdM = nan(nIds, 1);
depthThresholdM = nan(nIds, 1); horizontalRmsM = nan(nIds, 1);
depthStdM = nan(nIds, 1);
observationOutlier = true(height(data), 1);
for k = 1:nIds
    rows = find(data.transponder_id == ids(k));
    lat = data.latitude_deg(rows); lon = data.longitude_deg(rows);
    dep = data.computed_depth_m(rows);
    valid = isfinite(lat) & isfinite(lon) & isfinite(dep);
    totalCount(k) = numel(rows); validInputCount(k) = sum(valid);
    if sum(valid) < 3
        error('USBL:InsufficientCalibration', ...
            'ID%d有效标定观测少于3条。', ids(k));
    end
    lat0 = median(lat(valid)); lon0 = median(lon(valid));
    dep0 = median(dep(valid));
    [~, ~, radius] = wgs84_local_offset(lat0 .* ones(size(lat)), ...
        lon0 .* ones(size(lon)), lat, lon);
    radiusMedian = median(radius(valid));
    radiusSigma = 1.4826 * median(abs(radius(valid) - radiusMedian));
    depthSigma = 1.4826 * median(abs(dep(valid) - dep0));
    horizontalThresholdM(k) = max(100, radiusMedian + 6 * radiusSigma);
    depthThresholdM(k) = max(50, 6 * depthSigma);
    retained = valid & radius <= horizontalThresholdM(k) & ...
        abs(dep - dep0) <= depthThresholdM(k);
    if sum(retained) < 3
        error('USBL:CalibrationScreening', ...
            'ID%d粗差剔除后有效观测少于3条。', ids(k));
    end
    latitudeDeg(k) = median(lat(retained));
    longitudeDeg(k) = median(lon(retained));
    depthM(k) = median(dep(retained));
    retainedCount(k) = sum(retained);
    removedCount(k) = totalCount(k) - retainedCount(k);
    observationOutlier(rows(retained)) = false;
    [~, ~, finalRadius] = wgs84_local_offset( ...
        latitudeDeg(k) .* ones(size(lat)), ...
        longitudeDeg(k) .* ones(size(lon)), lat, lon);
    horizontalRmsM(k) = sqrt(mean(finalRadius(retained).^2));
    depthStdM(k) = std(dep(retained), 0);
end
calibration = table(ids, latitudeDeg, longitudeDeg, depthM, totalCount, ...
    validInputCount, retainedCount, removedCount, horizontalThresholdM, ...
    depthThresholdM, horizontalRmsM, depthStdM, ...
    'VariableNames', {'beacon_id', 'latitude_deg', 'longitude_deg', ...
    'depth_m', 'total_count', 'valid_input_count', 'retained_count', ...
    'removed_count', 'horizontal_screening_threshold_m', ...
    'depth_screening_threshold_m', 'horizontal_rms_m', 'depth_std_m'});
end

function [latitudeDeg, longitudeDeg, depthM] = ...
        lookup_beacon_calibration(ids, calibration)
latitudeDeg = nan(size(ids)); longitudeDeg = nan(size(ids));
depthM = nan(size(ids));
for k = 1:height(calibration)
    use = ids == calibration.beacon_id(k);
    latitudeDeg(use) = calibration.latitude_deg(k);
    longitudeDeg(use) = calibration.longitude_deg(k);
    depthM(use) = calibration.depth_m(k);
end
if any(~isfinite(latitudeDeg))
    error('USBL:MissingCalibration', '部分ID没有信标标定坐标。');
end
end

function [indices, differenceS] = nearest_source_id_time( ...
        querySource, queryTime, queryId, dataSource, dataTime, dataId, limitS)
indices = nan(numel(queryTime), 1); differenceS = nan(numel(queryTime), 1);
for source = unique(querySource)'
    sourceQuery = querySource == source;
    for id = unique(queryId(sourceQuery))'
        q = find(sourceQuery & queryId == id);
        d = find(dataSource == source & dataId == id);
        if isempty(d), continue; end
        [secondsData, order] = sort(posixtime(dataTime(d)));
        nearest = interp1(secondsData, (1:numel(secondsData))', ...
            posixtime(queryTime(q)), 'nearest', 'extrap');
        selected = d(order(nearest));
        delta = seconds(dataTime(selected) - queryTime(q));
        good = abs(delta) <= limitS;
        indices(q(good)) = selected(good);
        differenceS(q(good)) = delta(good);
    end
end
end

function [eastM, northM, horizontalM] = wgs84_local_offset( ...
        lat1Deg, lon1Deg, lat2Deg, lon2Deg)
a = 6378137.0; e2 = 6.6943799901413165e-3;
latMeanRad = deg2rad((lat1Deg + lat2Deg) / 2);
dLatRad = deg2rad(lat2Deg - lat1Deg);
dLonRad = deg2rad(lon2Deg - lon1Deg);
sinLat = sin(latMeanRad);
primeRadiusM = a ./ sqrt(1 - e2 .* sinLat.^2);
meridianRadiusM = a * (1 - e2) ./ (1 - e2 .* sinLat.^2).^(3/2);
eastM = dLonRad .* primeRadiusM .* cos(latMeanRad);
northM = dLatRad .* meridianRadiusM;
horizontalM = hypot(eastM, northM);
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
            neighbors = values(localRows);
            neighbors = neighbors(isfinite(neighbors));
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
