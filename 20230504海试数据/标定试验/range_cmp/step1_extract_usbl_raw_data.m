%% Step 1: extract useful fields from the two original USBL calibration logs
% Purpose
%   This script performs extraction only. It does not calculate range,
%   estimate beacon coordinates, reject outliers, or modify raw values.
%
% Usage
%   Save this file under:
%     20230504海试数据\scripts\range_cmp
%   Then open it in MATLAB and click Run. No input arguments are required.
%
% Input logs
%   USBL-BOX-R-20230429-230048.log  (beacon ID93 calibration)
%   USBL-BOX-R-20230430-030644.log  (beacon ID91 calibration)
%
% Core output: step1_usbl_core.mat
%   ptsag                 all PTSAG records, including ship ID0 and beacons
%   shipNavigation       ID0 PTSAG ship positions with nearest PTSAH heading
%   usblSolutions        beacon PTSAG joined with PTSAX, PTSAY, PPC ship pose
%                        and the nearest independent ID0/PTSAH ship solution
%   travelTimeChannel    long-form DBG/DETEC data, one row per channel
%   travelTimeFourChannel wide-form channel 0..3 propagation times and metrics
%   ppcDetection         PPC/DETEC ship pose and selected channel information
%
% Auxiliary output: step1_usbl_auxiliary.mat
%   auxiliaryMessages    every non-core raw message, unchanged
%   messageCatalog       counts of all message types in each source file
%
% Per-beacon outputs
%   step1_usbl_core_id91.mat / step1_usbl_core_id93.mat
%   step1_usbl_auxiliary_id91.mat / step1_usbl_auxiliary_id93.mat
%   Every combined table also contains calibration_beacon_id, so the two
%   calibration stages remain explicit even for ship ID0 and auxiliary rows.
%
% Time convention
%   utc_time is the timestamp encoded in the USBL messages. local_time_utc8
%   is the same instant displayed in UTC+8 for comparison with filenames.

clearvars;
clc;

%% Paths and source files
scriptDir = fileparts(mfilename('fullpath'));
scriptsDir = fileparts(scriptDir);
projectDir = fileparts(scriptsDir);
dataDir = string(fullfile(projectDir, '信标标定试验数据'));
outputDir = string(fullfile(projectDir, '解析后的数据', 'range_cmp'));

fileNames = ["USBL-BOX-R-20230429-230048.log"; ...
    "USBL-BOX-R-20230430-030644.log";...
    "USBL-BOX-R-20230501-033958.log";
    "USBL-BOX-R-20230501-081206.log"];
expectedBeaconIds = [93; 91; 92; 90];
fileNames=fileNames(3:4);
expectedBeaconIds=expectedBeaconIds(3:4);
if ~isfolder(dataDir)
    error('USBL:MissingDataDirectory', ...
        'Raw USBL directory does not exist: %s', dataDir);
end
if ~isfolder(outputDir)
    mkdir(outputDir);
end

%% Extract each file without filtering
nFiles = numel(fileNames);
ptsagCell = cell(nFiles, 1);
shipCell = cell(nFiles, 1);
solutionCell = cell(nFiles, 1);
channelCell = cell(nFiles, 1);
fourChannelCell = cell(nFiles, 1);
ppcCell = cell(nFiles, 1);
auxiliaryCell = cell(nFiles, 1);
catalogCell = cell(nFiles, 1);

for fileIndex = 1:nFiles
    inputFile = fullfile(dataDir, fileNames(fileIndex));
    mustBeFile(inputFile);
    fprintf('Extracting %s (expected beacon ID %d) ...\n', ...
        inputFile, expectedBeaconIds(fileIndex));

    lines = readlines(inputFile, 'WhitespaceRule', 'preserve', ...
        'EmptyLineRule', 'skip');
    sourceName = fileNames(fileIndex);

    ptsagFile = parse_ptsag(lines, sourceName);
    ptsahFile = parse_ptsah(lines, sourceName);
    ptsaxFile = parse_ptsax(lines, sourceName);
    ptsayFile = parse_ptsay(lines, sourceName);
    ptsaqFile = parse_ptsaq(lines, sourceName);
    ppcFile = parse_ppc_detection(lines, sourceName);
    shipFile = build_ship_navigation(ptsagFile, ptsahFile);
    solutionFile = build_usbl_solutions(ptsagFile, ptsaxFile, ...
        ptsayFile, ptsaqFile, ppcFile, shipFile);
    [channelFile, fourChannelFile] = parse_travel_time( ...
        lines, sourceName, ppcFile);
    [auxiliaryFile, catalogFile] = collect_auxiliary_messages( ...
        lines, sourceName);

    ptsagCell{fileIndex} = ptsagFile;
    shipCell{fileIndex} = shipFile;
    solutionCell{fileIndex} = solutionFile;
    channelCell{fileIndex} = channelFile;
    fourChannelCell{fileIndex} = fourChannelFile;
    ppcCell{fileIndex} = ppcFile;
    auxiliaryCell{fileIndex} = auxiliaryFile;
    catalogCell{fileIndex} = catalogFile;
end

%% Combine the two calibration stages
ptsag = vertcat(ptsagCell{:});
shipNavigation = vertcat(shipCell{:});
usblSolutions = vertcat(solutionCell{:});
travelTimeChannel = vertcat(channelCell{:});
travelTimeFourChannel = vertcat(fourChannelCell{:});
ppcDetection = vertcat(ppcCell{:});
auxiliaryMessages = vertcat(auxiliaryCell{:});
messageCatalog = vertcat(catalogCell{:});

% Explicitly label every row by the beacon calibrated in its source file.
% This is different from transponder_id: ship PTSAG rows have ID0, but still
% belong to either the ID91 or ID93 calibration stage.
ptsag = add_calibration_beacon_id(ptsag, fileNames, expectedBeaconIds);
shipNavigation = add_calibration_beacon_id(shipNavigation, fileNames, ...
    expectedBeaconIds);
usblSolutions = add_calibration_beacon_id(usblSolutions, fileNames, ...
    expectedBeaconIds);
travelTimeChannel = add_calibration_beacon_id(travelTimeChannel, ...
    fileNames, expectedBeaconIds);
travelTimeFourChannel = add_calibration_beacon_id( ...
    travelTimeFourChannel, fileNames, expectedBeaconIds);
ppcDetection = add_calibration_beacon_id(ppcDetection, fileNames, ...
    expectedBeaconIds);
auxiliaryMessages = add_calibration_beacon_id(auxiliaryMessages, ...
    fileNames, expectedBeaconIds);
messageCatalog = add_calibration_beacon_id(messageCatalog, fileNames, ...
    expectedBeaconIds);

sourceFiles = table(fileNames, expectedBeaconIds, ...
    'VariableNames', {'source_name', 'expected_beacon_id'});

metadata = struct;
metadata.step = 1;
metadata.description = "Raw USBL extraction only; no ranging or filtering";
metadata.time_zone_source = "UTC";
metadata.time_zone_display = "Asia/Shanghai (UTC+8)";
metadata.ptsax_interpretation = ...
    "body_forward_m and body_starboard_m are the ship-frame offsets " + ...
    "reported by PTSAX; no rotation is applied in step 1";
metadata.ptsay_interpretation = ...
    "north_offset_m and east_offset_m are the geographic-frame offsets " + ...
    "reported by PTSAY; no distance is calculated in step 1";
metadata.travel_time_interpretation = ...
    "travel_time_s is copied from each PIXOG,DBG,DETEC channel; " + ...
    "one-way/two-way conversion is intentionally deferred to step 2";
metadata.created_at = datetime('now', 'TimeZone', 'Asia/Shanghai');

solutionCount = zeros(nFiles, 1);
shipNavigationCount = zeros(nFiles, 1);
fourChannelGroupCount = zeros(nFiles, 1);
completeFourChannelCount = zeros(nFiles, 1);
ppcDetectionCount = zeros(nFiles, 1);
for fileIndex = 1:nFiles
    id = expectedBeaconIds(fileIndex);
    solutionCount(fileIndex) = sum( ...
        usblSolutions.calibration_beacon_id == id);
    shipNavigationCount(fileIndex) = sum( ...
        shipNavigation.calibration_beacon_id == id);
    fourChannelGroupCount(fileIndex) = sum( ...
        travelTimeFourChannel.calibration_beacon_id == id);
    completeFourChannelCount(fileIndex) = sum( ...
        travelTimeFourChannel.calibration_beacon_id == id & ...
        travelTimeFourChannel.complete_four_channels);
    ppcDetectionCount(fileIndex) = sum( ...
        ppcDetection.calibration_beacon_id == id);
end
batchSummary = table(expectedBeaconIds, fileNames, solutionCount, ...
    shipNavigationCount, ppcDetectionCount, fourChannelGroupCount, ...
    completeFourChannelCount, 'VariableNames', {'beacon_id', ...
    'source_name', 'usbl_solution_count', 'ship_navigation_count', ...
    'ppc_detection_count', 'four_channel_group_count', ...
    'complete_four_channel_count'});

coreFile = fullfile(outputDir, 'step1_usbl_core.mat');
auxiliaryFile = fullfile(outputDir, 'step1_usbl_auxiliary.mat');
save(coreFile, 'ptsag', 'shipNavigation', 'usblSolutions', ...
    'travelTimeChannel', 'travelTimeFourChannel', 'ppcDetection', ...
    'sourceFiles', 'batchSummary', 'metadata', '-v7.3');
save(auxiliaryFile, 'auxiliaryMessages', 'messageCatalog', ...
    'sourceFiles', 'batchSummary', 'metadata', '-v7.3');
writetable(batchSummary, fullfile(outputDir, 'step1_batch_summary.csv'));

% Save each calibration stage separately while retaining the combined MATs.
for fileIndex = 1:nFiles
    id = expectedBeaconIds(fileIndex);
    usePtsag = ptsag.calibration_beacon_id == id;
    useShip = shipNavigation.calibration_beacon_id == id;
    useSolution = usblSolutions.calibration_beacon_id == id;
    useChannel = travelTimeChannel.calibration_beacon_id == id;
    useFourChannel = travelTimeFourChannel.calibration_beacon_id == id;
    usePpc = ppcDetection.calibration_beacon_id == id;
    useAuxiliary = auxiliaryMessages.calibration_beacon_id == id;
    useCatalog = messageCatalog.calibration_beacon_id == id;

    batchCore = struct;
    batchCore.ptsag = ptsag(usePtsag, :);
    batchCore.shipNavigation = shipNavigation(useShip, :);
    batchCore.usblSolutions = usblSolutions(useSolution, :);
    batchCore.travelTimeChannel = travelTimeChannel(useChannel, :);
    batchCore.travelTimeFourChannel = ...
        travelTimeFourChannel(useFourChannel, :);
    batchCore.ppcDetection = ppcDetection(usePpc, :);
    batchCore.sourceFiles = sourceFiles(fileIndex, :);
    batchCore.batchSummary = batchSummary(fileIndex, :);
    batchCore.metadata = metadata;
    batchAuxiliary = struct;
    batchAuxiliary.auxiliaryMessages = ...
        auxiliaryMessages(useAuxiliary, :);
    batchAuxiliary.messageCatalog = messageCatalog(useCatalog, :);
    batchAuxiliary.sourceFiles = sourceFiles(fileIndex, :);
    batchAuxiliary.batchSummary = batchSummary(fileIndex, :);
    batchAuxiliary.metadata = metadata;

    batchCoreFile = fullfile(outputDir, sprintf( ...
        'step1_usbl_core_id%d.mat', id));
    batchAuxiliaryFile = fullfile(outputDir, sprintf( ...
        'step1_usbl_auxiliary_id%d.mat', id));
    save(batchCoreFile, '-struct', 'batchCore', '-v7.3');
    save(batchAuxiliaryFile, '-struct', 'batchAuxiliary', '-v7.3');
end

fprintf('\nStep 1 extraction complete.\n');
fprintf('  PTSAG rows:                 %d\n', height(ptsag));
fprintf('  Ship navigation rows:       %d\n', height(shipNavigation));
fprintf('  USBL solution rows:         %d\n', height(usblSolutions));
fprintf('  Travel-time channel rows:   %d\n', height(travelTimeChannel));
fprintf('  Four-channel groups:        %d\n', height(travelTimeFourChannel));
fprintf('  Complete four-channel sets: %d\n', ...
    sum(travelTimeFourChannel.complete_four_channels));
fprintf('  Auxiliary raw messages:     %d\n', height(auxiliaryMessages));
fprintf('  Core MAT: %s\n', coreFile);
fprintf('  Auxiliary MAT: %s\n', auxiliaryFile);
disp(batchSummary);

%% Local parsers: the file itself is a directly runnable script
function ptsag = parse_ptsag(lines, sourceName)
mask = startsWith(lines, "$PTSAG,");
lineNumber = find(mask);
raw = lines(mask);
n = numel(raw);
frameNumber = nan(n, 1);
utcTime = NaT(n, 1, 'TimeZone', 'UTC');
transponderId = nan(n, 1);
latitudeDeg = nan(n, 1);
longitudeDeg = nan(n, 1);
positionFlag = strings(n, 1);
computedDepthM = nan(n, 1);
depthValidity = nan(n, 1);
deviceDepthM = nan(n, 1);
checksumOk = false(n, 1);
for i = 1:n
    f = split_sentence(raw(i));
    frameNumber(i) = str2double(erase(field_or_empty(f, 2), "#"));
    utcTime(i) = make_utc(field_or_empty(f, 3), field_or_empty(f, 4), ...
        field_or_empty(f, 5), field_or_empty(f, 6));
    transponderId(i) = number_or_nan(f, 7);
    latitudeDeg(i) = dm_to_degrees(number_or_nan(f, 8), ...
        field_or_empty(f, 9));
    longitudeDeg(i) = dm_to_degrees(number_or_nan(f, 10), ...
        field_or_empty(f, 11));
    positionFlag(i) = field_or_empty(f, 12);
    computedDepthM(i) = number_or_nan(f, 13);
    depthValidity(i) = number_or_nan(f, 14);
    deviceDepthM(i) = number_or_nan(f, 15);
    checksumOk(i) = nmea_checksum_ok(raw(i));
end
localTime = as_utc8(utcTime);
ptsag = table(repmat(sourceName, n, 1), lineNumber, frameNumber, ...
    utcTime, localTime, transponderId, latitudeDeg, longitudeDeg, ...
    positionFlag, computedDepthM, depthValidity, deviceDepthM, ...
    checksumOk, raw, 'VariableNames', {'source_name', 'line_number', ...
    'frame_number', 'utc_time', 'local_time_utc8', 'transponder_id', ...
    'latitude_deg', 'longitude_deg', 'position_flag', ...
    'computed_depth_m', 'depth_validity', 'device_depth_m', ...
    'checksum_ok', 'raw_line'});
end

function ptsah = parse_ptsah(lines, sourceName)
mask = startsWith(lines, "$PTSAH,");
lineNumber = find(mask);
raw = lines(mask);
n = numel(raw);
referenceId = nan(n, 1);
headingDeg = nan(n, 1);
heaveM = nan(n, 1);
checksumOk = false(n, 1);
for i = 1:n
    f = split_sentence(raw(i));
    referenceId(i) = number_or_nan(f, 2);
    headingDeg(i) = number_or_nan(f, 3);
    heaveM(i) = number_or_nan(f, 4);
    checksumOk(i) = nmea_checksum_ok(raw(i));
end
ptsah = table(repmat(sourceName, n, 1), lineNumber, referenceId, ...
    headingDeg, heaveM, checksumOk, raw, ...
    'VariableNames', {'source_name', 'line_number', 'reference_id', ...
    'heading_deg', 'heave_m', 'checksum_ok', 'raw_line'});
end

function ptsax = parse_ptsax(lines, sourceName)
mask = startsWith(lines, "$PTSAX,");
lineNumber = find(mask);
raw = lines(mask);
n = numel(raw);
[frameNumber, utcTime, transponderId, firstValue, secondValue, ...
    positionFlag, computedDepthM, depthValidity, deviceDepthM, ...
    checksumOk] = parse_offset_sentence(raw);
localTime = as_utc8(utcTime);
ptsax = table(repmat(sourceName, n, 1), lineNumber, frameNumber, ...
    utcTime, localTime, transponderId, firstValue, secondValue, ...
    positionFlag, computedDepthM, depthValidity, deviceDepthM, ...
    checksumOk, raw, 'VariableNames', {'source_name', 'line_number', ...
    'frame_number', 'utc_time', 'local_time_utc8', 'transponder_id', ...
    'body_forward_m', 'body_starboard_m', 'position_flag', ...
    'computed_depth_m', 'depth_validity', 'device_depth_m', ...
    'checksum_ok', 'raw_line'});
end

function ptsay = parse_ptsay(lines, sourceName)
mask = startsWith(lines, "$PTSAY,");
lineNumber = find(mask);
raw = lines(mask);
n = numel(raw);
[frameNumber, utcTime, transponderId, firstValue, secondValue, ...
    positionFlag, computedDepthM, depthValidity, deviceDepthM, ...
    checksumOk] = parse_offset_sentence(raw);
localTime = as_utc8(utcTime);
ptsay = table(repmat(sourceName, n, 1), lineNumber, frameNumber, ...
    utcTime, localTime, transponderId, firstValue, secondValue, ...
    positionFlag, computedDepthM, depthValidity, deviceDepthM, ...
    checksumOk, raw, 'VariableNames', {'source_name', 'line_number', ...
    'frame_number', 'utc_time', 'local_time_utc8', 'transponder_id', ...
    'north_offset_m', 'east_offset_m', 'position_flag', ...
    'computed_depth_m', 'depth_validity', 'device_depth_m', ...
    'checksum_ok', 'raw_line'});
end

function [frameNumber, utcTime, transponderId, firstValue, ...
        secondValue, positionFlag, computedDepthM, depthValidity, ...
        deviceDepthM, checksumOk] = parse_offset_sentence(raw)
n = numel(raw);
frameNumber = nan(n, 1);
utcTime = NaT(n, 1, 'TimeZone', 'UTC');
transponderId = nan(n, 1);
firstValue = nan(n, 1);
secondValue = nan(n, 1);
positionFlag = strings(n, 1);
computedDepthM = nan(n, 1);
depthValidity = nan(n, 1);
deviceDepthM = nan(n, 1);
checksumOk = false(n, 1);
for i = 1:n
    f = split_sentence(raw(i));
    frameNumber(i) = str2double(erase(field_or_empty(f, 2), "#"));
    utcTime(i) = make_utc(field_or_empty(f, 3), field_or_empty(f, 4), ...
        field_or_empty(f, 5), field_or_empty(f, 6));
    transponderId(i) = number_or_nan(f, 7);
    firstValue(i) = number_or_nan(f, 8);
    secondValue(i) = number_or_nan(f, 9);
    positionFlag(i) = field_or_empty(f, 10);
    computedDepthM(i) = number_or_nan(f, 11);
    depthValidity(i) = number_or_nan(f, 12);
    deviceDepthM(i) = number_or_nan(f, 13);
    checksumOk(i) = nmea_checksum_ok(raw(i));
end
end

function ptsaq = parse_ptsaq(lines, sourceName)
mask = startsWith(lines, "$PTSAQ,");
lineNumber = find(mask);
raw = lines(mask);
n = numel(raw);
utcTime = NaT(n, 1, 'TimeZone', 'UTC');
transponderId = nan(n, 1);
quality1 = nan(n, 1);
quality2 = nan(n, 1);
quality3 = nan(n, 1);
quality4 = nan(n, 1);
checksumOk = false(n, 1);
for i = 1:n
    f = split_sentence(raw(i));
    utcTime(i) = make_utc(field_or_empty(f, 2), field_or_empty(f, 3), ...
        field_or_empty(f, 4), field_or_empty(f, 5));
    transponderId(i) = number_or_nan(f, 6);
    quality1(i) = number_or_nan(f, 7);
    quality2(i) = number_or_nan(f, 8);
    quality3(i) = number_or_nan(f, 9);
    quality4(i) = number_or_nan(f, 10);
    checksumOk(i) = nmea_checksum_ok(raw(i));
end
localTime = as_utc8(utcTime);
ptsaq = table(repmat(sourceName, n, 1), lineNumber, utcTime, localTime, ...
    transponderId, quality1, quality2, quality3, quality4, checksumOk, ...
    raw, 'VariableNames', {'source_name', 'line_number', 'utc_time', ...
    'local_time_utc8', 'transponder_id', 'quality_1', 'quality_2', ...
    'quality_3', 'quality_4', 'checksum_ok', 'raw_line'});
end

function ppc = parse_ppc_detection(lines, sourceName)
mask = startsWith(lines, "$PIXOG,PPC,DETEC,");
lineNumber = find(mask);
raw = lines(mask);
n = numel(raw);
utcTime = NaT(n, 1, 'TimeZone', 'UTC');
transponderId = nan(n, 1);
shipLatitudeRad = nan(n, 1);
shipLongitudeRad = nan(n, 1);
shipDepthM = nan(n, 1);
shipHeadingRad = nan(n, 1);
shipRollRad = nan(n, 1);
shipPitchRad = nan(n, 1);
ppcDelayCh1S = nan(n, 1);
ppcDelayCh2S = nan(n, 1);
ppcDelayCh3S = nan(n, 1);
channel1Phase = nan(n, 1);
channel2Phase = nan(n, 1);
channel3Phase = nan(n, 1);
channel1Quality = nan(n, 1);
channel2Quality = nan(n, 1);
channel3Quality = nan(n, 1);
checksumOk = false(n, 1);
for i = 1:n
    f = split_sentence(raw(i));
    utcTime(i) = make_utc(field_or_empty(f, 4), field_or_empty(f, 5), ...
        field_or_empty(f, 6), field_or_empty(f, 7));
    transponderId(i) = number_or_nan(f, 8);
    shipLatitudeRad(i) = number_or_nan(f, 9);
    shipLongitudeRad(i) = number_or_nan(f, 10);
    shipDepthM(i) = number_or_nan(f, 11);
    shipHeadingRad(i) = number_or_nan(f, 12);
    shipRollRad(i) = number_or_nan(f, 13);
    shipPitchRad(i) = number_or_nan(f, 14);
    ppcDelayCh1S(i) = number_or_nan(f, 22) / 1e6;
    channel1Phase(i) = number_or_nan(f, 23);
    channel1Quality(i) = number_or_nan(f, 24);
    ppcDelayCh2S(i) = number_or_nan(f, 26) / 1e6;
    channel2Phase(i) = number_or_nan(f, 27);
    channel2Quality(i) = number_or_nan(f, 28);
    ppcDelayCh3S(i) = number_or_nan(f, 30) / 1e6;
    channel3Phase(i) = number_or_nan(f, 31);
    channel3Quality(i) = number_or_nan(f, 32);
    checksumOk(i) = nmea_checksum_ok(raw(i));
end
localTime = as_utc8(utcTime);
ppc = table(repmat(sourceName, n, 1), lineNumber, utcTime, localTime, ...
    transponderId, shipLatitudeRad, shipLongitudeRad, ...
    rad2deg(shipLatitudeRad), rad2deg(shipLongitudeRad), shipDepthM, ...
    shipHeadingRad, wrap_to_360(rad2deg(shipHeadingRad)), shipRollRad, ...
    rad2deg(shipRollRad), shipPitchRad, rad2deg(shipPitchRad), ...
    ppcDelayCh1S, ppcDelayCh2S, ppcDelayCh3S, channel1Phase, ...
    channel2Phase, channel3Phase, channel1Quality, channel2Quality, ...
    channel3Quality, checksumOk, raw, ...
    'VariableNames', {'source_name', 'line_number', 'utc_time', ...
    'local_time_utc8', 'transponder_id', 'ship_latitude_rad', ...
    'ship_longitude_rad', 'ship_latitude_deg', 'ship_longitude_deg', ...
    'ship_depth_m', 'ship_heading_rad', 'ship_heading_deg', ...
    'ship_roll_rad', 'ship_roll_deg', 'ship_pitch_rad', 'ship_pitch_deg', ...
    'ppc_delay_ch1_s', 'ppc_delay_ch2_s', 'ppc_delay_ch3_s', ...
    'channel1_phase', 'channel2_phase', 'channel3_phase', ...
    'channel1_quality', 'channel2_quality', 'channel3_quality', ...
    'checksum_ok', 'raw_line'});
end

function ship = build_ship_navigation(ptsag, ptsah)
shipPtsag = ptsag(ptsag.transponder_id == 0, :);
n = height(shipPtsag);
headingDeg = nan(n, 1);
heaveM = nan(n, 1);
headingLineNumber = nan(n, 1);
headingLineDifference = nan(n, 1);
headingChecksumOk = false(n, 1);
if ~isempty(ptsah)
    nearest = interp1(double(ptsah.line_number), (1:height(ptsah))', ...
        double(shipPtsag.line_number), 'nearest', NaN);
    good = isfinite(nearest);
    j = nearest(good);
    headingDeg(good) = ptsah.heading_deg(j);
    heaveM(good) = ptsah.heave_m(j);
    headingLineNumber(good) = ptsah.line_number(j);
    headingLineDifference(good) = double(ptsah.line_number(j)) - ...
        double(shipPtsag.line_number(good));
    headingChecksumOk(good) = ptsah.checksum_ok(j);
end
ship = table(shipPtsag.source_name, shipPtsag.line_number, ...
    shipPtsag.frame_number, shipPtsag.utc_time, shipPtsag.local_time_utc8, ...
    shipPtsag.latitude_deg, shipPtsag.longitude_deg, ...
    shipPtsag.computed_depth_m, shipPtsag.device_depth_m, ...
    shipPtsag.position_flag, headingDeg, heaveM, headingLineNumber, ...
    headingLineDifference, shipPtsag.checksum_ok, headingChecksumOk, ...
    shipPtsag.raw_line, 'VariableNames', {'source_name', ...
    'ptsag_line_number', 'ptsag_frame_number', 'utc_time', ...
    'local_time_utc8', 'latitude_deg', 'longitude_deg', ...
    'computed_depth_m', 'device_depth_m', 'position_flag', ...
    'heading_deg', 'heave_m', 'ptsah_line_number', ...
    'heading_line_difference', 'ptsag_checksum_ok', ...
    'ptsah_checksum_ok', 'ptsag_raw_line'});
end

function solution = build_usbl_solutions(ptsag, ptsax, ptsay, ptsaq, ppc, ship)
target = ptsag(ptsag.transponder_id ~= 0, :);
n = height(target);
[ix, dx] = nearest_same_id_time(target.utc_time, target.transponder_id, ...
    ptsax.utc_time, ptsax.transponder_id, 0.02);
[iy, dy] = nearest_same_id_time(target.utc_time, target.transponder_id, ...
    ptsay.utc_time, ptsay.transponder_id, 0.02);
[iq, dq] = nearest_same_id_time(target.utc_time, target.transponder_id, ...
    ptsaq.utc_time, ptsaq.transponder_id, 0.02);
[ip, dp] = nearest_same_id_time(target.utc_time, target.transponder_id, ...
    ppc.utc_time, ppc.transponder_id, 0.02);
[is, ds] = nearest_time(target.utc_time, ship.utc_time, 1.0);

bodyForwardM = take_numeric(ptsax.body_forward_m, ix, n);
bodyStarboardM = take_numeric(ptsax.body_starboard_m, ix, n);
northOffsetM = take_numeric(ptsay.north_offset_m, iy, n);
eastOffsetM = take_numeric(ptsay.east_offset_m, iy, n);
quality1 = take_numeric(ptsaq.quality_1, iq, n);
quality2 = take_numeric(ptsaq.quality_2, iq, n);
quality3 = take_numeric(ptsaq.quality_3, iq, n);
quality4 = take_numeric(ptsaq.quality_4, iq, n);
ppcShipLatitudeDeg = take_numeric(ppc.ship_latitude_deg, ip, n);
ppcShipLongitudeDeg = take_numeric(ppc.ship_longitude_deg, ip, n);
ppcShipDepthM = take_numeric(ppc.ship_depth_m, ip, n);
ppcShipHeadingDeg = take_numeric(ppc.ship_heading_deg, ip, n);
ppcShipRollDeg = take_numeric(ppc.ship_roll_deg, ip, n);
ppcShipPitchDeg = take_numeric(ppc.ship_pitch_deg, ip, n);
nmeaShipLatitudeDeg = take_numeric(ship.latitude_deg, is, n);
nmeaShipLongitudeDeg = take_numeric(ship.longitude_deg, is, n);
nmeaShipDepthM = take_numeric(ship.computed_depth_m, is, n);
nmeaShipHeadingDeg = take_numeric(ship.heading_deg, is, n);

solution = table(target.source_name, target.line_number, ...
    target.frame_number, target.utc_time, target.local_time_utc8, ...
    target.transponder_id, target.latitude_deg, target.longitude_deg, ...
    target.computed_depth_m, target.device_depth_m, target.position_flag, ...
    bodyForwardM, bodyStarboardM, northOffsetM, eastOffsetM, ...
    quality1, quality2, quality3, quality4, ppcShipLatitudeDeg, ...
    ppcShipLongitudeDeg, ppcShipDepthM, ppcShipHeadingDeg, ...
    ppcShipRollDeg, ppcShipPitchDeg, nmeaShipLatitudeDeg, ...
    nmeaShipLongitudeDeg, nmeaShipDepthM, nmeaShipHeadingDeg, ...
    dx, dy, dq, dp, ds, target.checksum_ok, target.raw_line, ...
    'VariableNames', {'source_name', 'ptsag_line_number', ...
    'ptsag_frame_number', 'utc_time', 'local_time_utc8', ...
    'transponder_id', 'latitude_deg', 'longitude_deg', ...
    'computed_depth_m', 'device_depth_m', 'position_flag', ...
    'body_forward_m', 'body_starboard_m', 'north_offset_m', ...
    'east_offset_m', 'quality_1', 'quality_2', 'quality_3', 'quality_4', ...
    'ppc_ship_latitude_deg', 'ppc_ship_longitude_deg', ...
    'ppc_ship_depth_m', 'ppc_ship_heading_deg', 'ppc_ship_roll_deg', ...
    'ppc_ship_pitch_deg', 'nmea_ship_latitude_deg', ...
    'nmea_ship_longitude_deg', 'nmea_ship_depth_m', ...
    'nmea_ship_heading_deg', 'ptsax_time_difference_s', ...
    'ptsay_time_difference_s', 'ptsaq_time_difference_s', ...
    'ppc_time_difference_s', 'nmea_ship_time_difference_s', ...
    'ptsag_checksum_ok', 'ptsag_raw_line'});
end

function [channelTable, fourTable] = parse_travel_time(lines, sourceName, ppc)
mask = startsWith(lines, "$PIXOG,DBG,DETEC,");
lineNumber = find(mask);
raw = lines(mask);
n = numel(raw);
debugGroup = nan(n, 1);
channel = nan(n, 1);
detectorCode = nan(n, 1);
metric1 = nan(n, 1);
metric2 = nan(n, 1);
metric3 = nan(n, 1);
signalQuality = nan(n, 1);
travelTimeS = nan(n, 1);
checksumOk = false(n, 1);
for i = 1:n
    f = split_sentence(raw(i));
    debugGroup(i) = number_or_nan(f, 4);
    channel(i) = number_or_nan(f, 5);
    detectorCode(i) = number_or_nan(f, 6);
    metric1(i) = number_or_nan(f, 7);
    metric2(i) = number_or_nan(f, 8);
    metric3(i) = number_or_nan(f, 9);
    signalQuality(i) = number_or_nan(f, 10);
    travelTimeS(i) = number_or_nan(f, 11);
    checksumOk(i) = nmea_checksum_ok(raw(i));
end
channelTable = table(repmat(sourceName, n, 1), lineNumber, debugGroup, ...
    channel, detectorCode, metric1, metric2, metric3, signalQuality, ...
    travelTimeS, checksumOk, raw, 'VariableNames', {'source_name', ...
    'line_number', 'debug_group', 'channel', 'detector_code', ...
    'metric_1', 'metric_2', 'metric_3', 'signal_quality', ...
    'travel_time_s', 'checksum_ok', 'raw_line'});

if n == 0
    fourTable = table;
    return;
end
% A debug-group number can be reused for a second detection attempt. A
% channel-number reset (for example 3 -> 0) therefore starts a new set even
% when debug_group itself has not changed.
newRun = [true; diff(debugGroup) ~= 0 | ...
    channel(2:end) <= channel(1:end-1)];
runId = cumsum(newRun);
numberOfRuns = runId(end);
firstLine = nan(numberOfRuns, 1);
lastLine = nan(numberOfRuns, 1);
groupValue = nan(numberOfRuns, 1);
complete = false(numberOfRuns, 1);
detector = nan(numberOfRuns, 4);
value1 = nan(numberOfRuns, 4);
value2 = nan(numberOfRuns, 4);
value3 = nan(numberOfRuns, 4);
quality = nan(numberOfRuns, 4);
delay = nan(numberOfRuns, 4);
channelChecksum = false(numberOfRuns, 4);
ppcIndex = nan(numberOfRuns, 1);
ppcDelayMatchErrorS = nan(numberOfRuns, 1);
for g = 1:numberOfRuns
    rows = find(runId == g);
    firstLine(g) = lineNumber(rows(1));
    lastLine(g) = lineNumber(rows(end));
    groupValue(g) = debugGroup(rows(1));
    for ch = 0:3
        r = rows(find(channel(rows) == ch, 1, 'first'));
        if ~isempty(r)
            detector(g, ch + 1) = detectorCode(r);
            value1(g, ch + 1) = metric1(r);
            value2(g, ch + 1) = metric2(r);
            value3(g, ch + 1) = metric3(r);
            quality(g, ch + 1) = signalQuality(r);
            delay(g, ch + 1) = travelTimeS(r);
            channelChecksum(g, ch + 1) = checksumOk(r);
        end
    end
    complete(g) = all(isfinite(delay(g, :)));
end

% Perform a one-to-one association from each PPC solution back to a prior
% detection set. The median CH1..CH3 delay difference is the main criterion;
% line distance only breaks ties. This handles delayed/asynchronous messages
% and repeated debug-group numbers without assigning one PPC twice.
for p = 1:height(ppc)
    lineGap = double(ppc.line_number(p)) - lastLine;
    candidates = find(lineGap > 0 & lineGap <= 2000 & ...
        ~isfinite(ppcIndex));
    if isempty(candidates)
        continue;
    end
    ppcDelay = [ppc.ppc_delay_ch1_s(p), ppc.ppc_delay_ch2_s(p), ...
        ppc.ppc_delay_ch3_s(p)];
    absoluteDifference = abs(delay(candidates, 2:4) - ppcDelay);
    validCount = sum(isfinite(absoluteDifference), 2);
    matchError = median(absoluteDifference, 2, 'omitnan');
    matchError(validCount < 2) = inf;
    score = matchError + lineGap(candidates) * 1e-12;
    [bestScore, localBest] = min(score);
    if ~isfinite(bestScore)
        [~, localBest] = min(lineGap(candidates));
    end
    best = candidates(localBest);
    ppcIndex(best) = p;
    if isfinite(matchError(localBest))
        ppcDelayMatchErrorS(best) = matchError(localBest);
    end
end
ppcUtcTime = NaT(numberOfRuns, 1, 'TimeZone', 'UTC');
ppcTransponderId = nan(numberOfRuns, 1);
ppcLineNumber = nan(numberOfRuns, 1);
good = isfinite(ppcIndex);
j = ppcIndex(good);
ppcUtcTime(good) = ppc.utc_time(j);
ppcTransponderId(good) = ppc.transponder_id(j);
ppcLineNumber(good) = ppc.line_number(j);
ppcLocalTime = as_utc8(ppcUtcTime);

fourTable = table(repmat(sourceName, numberOfRuns, 1), groupValue, ...
    firstLine, lastLine, complete, detector(:, 1), value1(:, 1), ...
    value2(:, 1), value3(:, 1), quality(:, 1), delay(:, 1), ...
    detector(:, 2), value1(:, 2), value2(:, 2), value3(:, 2), ...
    quality(:, 2), delay(:, 2), detector(:, 3), value1(:, 3), ...
    value2(:, 3), value3(:, 3), quality(:, 3), delay(:, 3), ...
    detector(:, 4), value1(:, 4), value2(:, 4), value3(:, 4), ...
    quality(:, 4), delay(:, 4), channelChecksum(:, 1), ...
    channelChecksum(:, 2), channelChecksum(:, 3), ...
    channelChecksum(:, 4), ppcLineNumber, ppcUtcTime, ppcLocalTime, ...
    ppcTransponderId, ppcDelayMatchErrorS, ...
    'VariableNames', {'source_name', 'debug_group', ...
    'first_line_number', 'last_line_number', 'complete_four_channels', ...
    'ch0_detector_code', 'ch0_metric_1', 'ch0_metric_2', 'ch0_metric_3', ...
    'ch0_signal_quality', 'ch0_travel_time_s', 'ch1_detector_code', ...
    'ch1_metric_1', 'ch1_metric_2', 'ch1_metric_3', ...
    'ch1_signal_quality', 'ch1_travel_time_s', 'ch2_detector_code', ...
    'ch2_metric_1', 'ch2_metric_2', 'ch2_metric_3', ...
    'ch2_signal_quality', 'ch2_travel_time_s', 'ch3_detector_code', ...
    'ch3_metric_1', 'ch3_metric_2', 'ch3_metric_3', ...
    'ch3_signal_quality', 'ch3_travel_time_s', 'ch0_checksum_ok', ...
    'ch1_checksum_ok', 'ch2_checksum_ok', 'ch3_checksum_ok', ...
    'ppc_line_number', 'utc_time', 'local_time_utc8', 'transponder_id', ...
    'ppc_delay_match_error_s'});
end

function [auxiliary, catalog] = collect_auxiliary_messages(lines, sourceName)
n = numel(lines);
lineNumber = (1:n)';
messageType = strings(n, 1);
isCore = false(n, 1);
checksumOk = false(n, 1);
for i = 1:n
    messageType(i) = classify_message(lines(i));
    isCore(i) = is_core_type(messageType(i));
    checksumOk(i) = nmea_checksum_ok(lines(i));
end
allMessages = table(repmat(sourceName, n, 1), lineNumber, ...
    messageType, isCore, checksumOk, lines, ...
    'VariableNames', {'source_name', 'line_number', 'message_type', ...
    'is_core_message', 'checksum_ok', 'raw_line'});
auxiliary = allMessages(~isCore, :);
[groups, source, type] = findgroups(allMessages.source_name, ...
    allMessages.message_type);
count = splitapply(@numel, allMessages.line_number, groups);
catalog = table(source, type, count, ...
    'VariableNames', {'source_name', 'message_type', 'message_count'});
catalog = sortrows(catalog, 'message_count', 'descend');
end

function type = classify_message(line)
f = split_sentence(line);
if isempty(f)
    type = "EMPTY";
    return;
end
first = strip(erase(field_or_empty(f, 1), "$"));
if first == "PIXOG"
    second = field_or_empty(f, 2);
    third = field_or_empty(f, 3);
    if second == "DBG" && any(third == ["0", "2"])
        type = strjoin([first, second, third, field_or_empty(f, 4), ...
            field_or_empty(f, 5)], ",");
    else
        type = strjoin([first, second, third], ",");
    end
elseif first == "PIXSE"
    type = strjoin([first, field_or_empty(f, 2)], ",");
else
    type = first;
end
end

function tf = is_core_type(type)
tf = any(type == ["PTSAG", "PTSAX", "PTSAY", "PTSAH", "PTSAQ", ...
    "PIXOG,PPC,DETEC", "PIXOG,DBG,DETEC"]);
end

function [indices, timeDifferenceS] = nearest_same_id_time( ...
        queryTime, queryId, dataTime, dataId, maximumDifferenceS)
n = numel(queryTime);
indices = nan(n, 1);
timeDifferenceS = nan(n, 1);
for id = unique(queryId(:))'
    q = find(queryId == id);
    d = find(dataId == id);
    if isempty(d)
        continue;
    end
    [localIndex, localDifference] = nearest_time(queryTime(q), ...
        dataTime(d), maximumDifferenceS);
    good = isfinite(localIndex);
    indices(q(good)) = d(localIndex(good));
    timeDifferenceS(q(good)) = localDifference(good);
end
end

function [indices, timeDifferenceS] = nearest_time( ...
        queryTime, dataTime, maximumDifferenceS)
n = numel(queryTime);
indices = nan(n, 1);
timeDifferenceS = nan(n, 1);
if isempty(dataTime)
    return;
end
[dataSeconds, order] = sort(posixtime(dataTime));
nearestSorted = interp1(dataSeconds, (1:numel(dataSeconds))', ...
    posixtime(queryTime), 'nearest', 'extrap');
good = isfinite(nearestSorted);
indices(good) = order(nearestSorted(good));
timeDifferenceS(good) = seconds(dataTime(indices(good)) - queryTime(good));
tooFar = abs(timeDifferenceS) > maximumDifferenceS;
indices(tooFar) = NaN;
timeDifferenceS(tooFar) = NaN;
end

function output = take_numeric(values, indices, n)
output = nan(n, 1);
good = isfinite(indices);
output(good) = values(indices(good));
end

function f = split_sentence(line)
star = strfind(line, "*");
if isempty(star)
    body = line;
else
    body = extractBefore(line, star(1));
end
f = split(body, ",");
end

function value = field_or_empty(fields, index)
if index > numel(fields)
    value = "";
else
    value = fields(index);
end
end

function value = number_or_nan(fields, index)
value = str2double(field_or_empty(fields, index));
end

function utc = make_utc(timeText, dayText, monthText, yearText)
tod = str2double(timeText);
day = str2double(dayText);
month = str2double(monthText);
year = str2double(yearText);
hh = floor(tod / 10000);
mm = floor((tod - hh * 10000) / 100);
ss = tod - hh * 10000 - mm * 100;
utc = datetime(year, month, day, hh, mm, ss, 'TimeZone', 'UTC');
utc.Format = 'yyyy-MM-dd HH:mm:ss.SSSSSS';
end

function localTime = as_utc8(utcTime)
localTime = utcTime;
localTime.TimeZone = 'Asia/Shanghai';
localTime.Format = 'yyyy-MM-dd HH:mm:ss.SSSSSS Z';
end

function degrees = dm_to_degrees(dm, hemisphere)
degrees = floor(dm / 100) + mod(dm, 100) / 60;
if any(strcmpi(hemisphere, ["S", "W"]))
    degrees = -degrees;
end
end

function ok = nmea_checksum_ok(line)
line = char(strip(line));
star = find(line == '*', 1, 'last');
if isempty(star) || star + 2 > numel(line) || isempty(line) || line(1) ~= '$'
    ok = false;
    return;
end
actual = uint8(0);
for i = 2:(star - 1)
    actual = bitxor(actual, uint8(line(i)));
end
ok = double(actual) == hex2dec(line(star + 1:star + 2));
end

function angle = wrap_to_360(angle)
angle = mod(angle, 360);
end

function output = add_calibration_beacon_id(input, fileNames, beaconIds)
calibrationBeaconId = nan(height(input), 1);
for fileIndex = 1:numel(fileNames)
    calibrationBeaconId(input.source_name == fileNames(fileIndex)) = ...
        beaconIds(fileIndex);
end
output = addvars(input, calibrationBeaconId, 'After', 'source_name', ...
    'NewVariableNames', 'calibration_beacon_id');
end
