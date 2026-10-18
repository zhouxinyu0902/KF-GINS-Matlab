%% Step 2: calculate and compare USBL ranging results
% This is a directly runnable script, not a function.
%
% Input
%   解析后的数据\range_cmp\step1_usbl_core.mat
%   信标标定试验数据\svp2023-04-27.txt  (measured cast at the trial site)
%
% Ranging sections
%   A. Reference: final calibrated beacon position + PPC ship position
%   B. PTSAX: ship-frame forward/starboard offsets; heading is also used to
%      rotate the offsets into north/east for checking against PTSAY
%   C. PTSAG: WGS-84 position difference between PPC ship and USBL beacon
%   D. Propagation time: four channels and four algorithms from
%      Exper0_PreData.m -> PropaTcmp.m (consistent with range.m)
%      1) Equation: linear sound-speed-gradient analytical expression
%      2) Zhou: ray-parameter integral solved by bisection
%      3) AMS: arithmetic-mean sound speed
%      4) GMS: name retained from range.m; its formula is a harmonic mean
%
% Important time convention
%   Exper0_PreData.m first removes the 20 ms transponder turnaround time,
%   and PropaTcmp.m then accepts a one-way propagation time. In the present
%   2023 DBG/DETEC logs, CH1--CH3 are about 20 ms later than CH0 and are the
%   three values copied to PPC/DETEC. Therefore this script uses:
%     CH0: one_way_time = raw_time / 2
%     CH1--CH3: one_way_time = (raw_time - 0.020 s) / 2
%   This makes the four channels share the same physical time convention.
%
% Preliminary travel-time preprocessing
%   1) Remove a channel when its corrected round-trip time differs from the
%      four-channel median by more than 1 ms.
%   2) Detect isolated fused-time spikes inside each source/beacon sequence
%      with an 11-point local window, excluding sequence endpoints. The
%      threshold is max(5 ms, 6 x local robust sigma).
%   3) Require at least two valid channels among CH1--CH3. Removed values
%      are stored as NaN and excluded from ranging and error statistics;
%      the raw values and outlier flags remain in the output MAT file.
%   PTSAX and PTSAG are filtered independently: values with more than
%   100 m error against the calibrated horizontal reference, or an isolated
%   local jump exceeding max(50 m, 6 x local robust sigma), are removed.
%
% Main output
%   step2_range_comparison.mat
%     rangeComparison  one row per PPC/USBL solution (main comparison)
%     travelTimeRange  four rows per solution (CH0..CH3 x four algorithms)
%     methodSummary    error statistics relative to reference horizontal range
%     beaconCalibration fixed final calibration coordinates used as reference
%   step2_range_comparison_id91.mat / step2_range_comparison_id93.mat
%     the same results split into the two calibration stages
%   step2_accuracy_comparison_overall.csv
%   step2_accuracy_comparison_by_id.csv
%   step2_accuracy_comparison.xlsx (Overall, ID91 and ID93 sheets)
%
% All method comparisons and error statistics use horizontal distance. Slant
% distance is retained only as supplementary diagnostic information.

clearvars;
clc;
close all;

%% Paths and settings
scriptDir = fileparts(mfilename('fullpath'));
scriptsDir = fileparts(scriptDir);
projectDir = fileparts(scriptsDir);
inputFile = string(fullfile(projectDir, '解析后的数据', 'range_cmp', ...
    'step1_usbl_core.mat'));
outputDir = string(fullfile(projectDir, '解析后的数据', 'range_cmp'));
svpFile = string(fullfile(projectDir, '信标标定试验数据', ...
    'svp2023-04-27.txt'));
outputFile = fullfile(outputDir, 'step2_range_comparison.mat');
figurePng = fullfile(outputDir, 'step2_range_comparison.png');
figureFig = fullfile(outputDir, 'step2_range_comparison.fig');
accuracyOverallCsv = fullfile(outputDir, ...
    'step2_accuracy_comparison_overall.csv');
accuracyByIdCsv = fullfile(outputDir, ...
    'step2_accuracy_comparison_by_id.csv');
accuracyWorkbook = fullfile(outputDir, 'step2_accuracy_comparison.xlsx');
batchSummaryCsv = fullfile(outputDir, 'step2_batch_summary.csv');

mustBeFile(inputFile);
mustBeFile(svpFile);
if ~isfolder(outputDir)
    mkdir(outputDir);
end

% Final beacon calibration used as the distance reference. These values are
% deliberately explicit rather than re-estimated in this comparison script.
beaconId = [93; 91];
calibratedLatitudeDeg = [15.8374720000000; 15.8516811666667];
calibratedLongitudeDeg = [115.157920500000; 115.158053166667];
calibratedDepthM = [4202.07; 4202.46];
beaconCalibration = table(beaconId, calibratedLatitudeDeg, ...
    calibratedLongitudeDeg, calibratedDepthM, ...
    'VariableNames', {'beacon_id', 'latitude_deg', 'longitude_deg', ...
    'depth_m'});

% Use the measured down/up cast in the beacon-calibration folder. Repeated
% samples from the descending and ascending legs are combined by the median
% in 1 m depth bins in read_svp_file().
svp = read_svp_file(svpFile);
if max(svp.depth_m) < max(calibratedDepthM)
    % The measured cast ends only about 5 m above the calibrated beacons.
    % Extend that short uncovered interval with the robust deepest speed,
    % avoiding noisy linear extrapolation from the final two samples.
    deepestSpeedMps = median(svp.sound_speed_mps( ...
        svp.depth_m >= max(svp.depth_m) - 10), 'omitnan');
    svp = [svp; table(max(calibratedDepthM), deepestSpeedMps, ...
        'VariableNames', svp.Properties.VariableNames)];
end

requiredVariables = {'usblSolutions', 'travelTimeFourChannel', ...
    'ppcDetection'};
loaded = load(inputFile, requiredVariables{:});
for k = 1:numel(requiredVariables)
    if ~isfield(loaded, requiredVariables{k})
        error('USBL:MissingVariable', 'Missing %s in %s.', ...
            requiredVariables{k}, inputFile);
    end
end
usblSolutions = loaded.usblSolutions;
travelTimeFourChannel = loaded.travelTimeFourChannel;
ppcDetection = loaded.ppcDetection;
n = height(usblSolutions);

%% Associate every USBL solution with its PPC line and four-channel set
[ppcIndex, ppcTimeDifferenceS] = nearest_source_id_time( ...
    usblSolutions.source_name, usblSolutions.utc_time, ...
    usblSolutions.transponder_id, ppcDetection.source_name, ...
    ppcDetection.utc_time, ppcDetection.transponder_id, 0.02);
if any(~isfinite(ppcIndex))
    error('USBL:UnmatchedPPC', '%d solutions do not have a matching PPC row.', ...
        sum(~isfinite(ppcIndex)));
end
ppcLineNumber = ppcDetection.line_number(ppcIndex);
solutionKey = usblSolutions.source_name + "|" + string(ppcLineNumber);
linked = isfinite(travelTimeFourChannel.ppc_line_number);
fourChannelKey = travelTimeFourChannel.source_name(linked) + "|" + ...
    string(travelTimeFourChannel.ppc_line_number(linked));
[hasFourChannel, locationInLinked] = ismember(solutionKey, fourChannelKey);
if any(~hasFourChannel)
    error('USBL:UnmatchedTravelTime', ...
        '%d PPC rows do not have a matched DBG/DETEC set.', ...
        sum(~hasFourChannel));
end
linkedRows = find(linked);
travelIndex = linkedRows(locationInLinked);

rawRoundTripTimeS = [ ...
    travelTimeFourChannel.ch0_travel_time_s(travelIndex), ...
    travelTimeFourChannel.ch1_travel_time_s(travelIndex), ...
    travelTimeFourChannel.ch2_travel_time_s(travelIndex), ...
    travelTimeFourChannel.ch3_travel_time_s(travelIndex)];
turnaroundTimeS = 0.020;
turnaroundCorrectionS = repmat([0, turnaroundTimeS, ...
    turnaroundTimeS, turnaroundTimeS], n, 1);
propagationRoundTripTimeS = rawRoundTripTimeS - turnaroundCorrectionS;

% Preliminary outlier removal. The normal corrected inter-channel spread in
% this data set is below 0.25 ms, whereas the obvious spikes are 56--65 ms.
channelConsistencyThresholdS = 0.001;
channelMedianTimeS = median(propagationRoundTripTimeS, 2, 'omitnan');
channelConsistencyResidualS = abs(propagationRoundTripTimeS - ...
    channelMedianTimeS);
channelMissing = ~isfinite(propagationRoundTripTimeS);
channelOutlier = isfinite(propagationRoundTripTimeS) & ...
    channelConsistencyResidualS > channelConsistencyThresholdS;
channelRejected = channelMissing | channelOutlier;

filteredRawRoundTripTimeS = rawRoundTripTimeS;
filteredPropagationRoundTripTimeS = propagationRoundTripTimeS;
filteredRawRoundTripTimeS(channelRejected) = NaN;
filteredPropagationRoundTripTimeS(channelRejected) = NaN;

validMainChannelCount = sum(isfinite( ...
    filteredPropagationRoundTripTimeS(:, 2:4)), 2);
preliminaryFusedPropagationTimeS = median( ...
    filteredPropagationRoundTripTimeS(:, 2:4), 2, 'omitnan');
temporalOutlier = detect_local_outliers( ...
    preliminaryFusedPropagationTimeS, usblSolutions.source_name, ...
    usblSolutions.utc_time, usblSolutions.transponder_id, 5, 0.005, 6);
insufficientMainChannels = validMainChannelCount < 2;
delayObservationOutlier = temporalOutlier | insufficientMainChannels;
filteredRawRoundTripTimeS(delayObservationOutlier, :) = NaN;
filteredPropagationRoundTripTimeS(delayObservationOutlier, :) = NaN;
oneWayTimeS = filteredPropagationRoundTripTimeS / 2;

%% Look up the fixed calibration coordinate for every observation
[beaconFinalLatitudeDeg, beaconFinalLongitudeDeg, beaconFinalDepthM] = ...
    lookup_beacon_calibration(usblSolutions.transponder_id, ...
    beaconCalibration);

shipLatitudeDeg = usblSolutions.ppc_ship_latitude_deg;
shipLongitudeDeg = usblSolutions.ppc_ship_longitude_deg;
shipDepthM = usblSolutions.ppc_ship_depth_m;
shipHeadingDeg = usblSolutions.ppc_ship_heading_deg;

%% A. Reference distance: final beacon calibration + PPC ship position
[referenceEastM, referenceNorthM, referenceHorizontalM] = ...
    wgs84_local_offset(shipLatitudeDeg, shipLongitudeDeg, ...
    beaconFinalLatitudeDeg, beaconFinalLongitudeDeg);
referenceVerticalM = beaconFinalDepthM - shipDepthM;
referenceSlantM = hypot(referenceHorizontalM, referenceVerticalM);

%% B. PTSAX distance and heading rotation check against PTSAY
bodyForwardM = usblSolutions.body_forward_m;
bodyStarboardM = usblSolutions.body_starboard_m;
ptsaxNorthM = bodyForwardM .* cosd(shipHeadingDeg) - ...
    bodyStarboardM .* sind(shipHeadingDeg);
ptsaxEastM = bodyForwardM .* sind(shipHeadingDeg) + ...
    bodyStarboardM .* cosd(shipHeadingDeg);
ptsaxHorizontalM = hypot(bodyForwardM, bodyStarboardM);
ptsaxVerticalM = usblSolutions.computed_depth_m - shipDepthM;
ptsaxSlantM = hypot(ptsaxHorizontalM, ptsaxVerticalM);

positionReferenceOutlierThresholdM = 100;
positionLocalMinimumThresholdM = 50;
ptsaxTemporalOutlier = detect_local_outliers(ptsaxHorizontalM, ...
    usblSolutions.source_name, usblSolutions.utc_time, ...
    usblSolutions.transponder_id, 5, positionLocalMinimumThresholdM, 6);
ptsaxReferenceOutlier = isfinite(ptsaxHorizontalM) & ...
    abs(ptsaxHorizontalM - referenceHorizontalM) > ...
    positionReferenceOutlierThresholdM;
ptsaxOutlier = ~isfinite(ptsaxHorizontalM) | ptsaxTemporalOutlier | ...
    ptsaxReferenceOutlier;
filteredPtsaxHorizontalM = ptsaxHorizontalM;
filteredPtsaxHorizontalM(ptsaxOutlier) = NaN;

% PTSAY is retained as a geographic-frame consistency check. Rotation does
% not change the PTSAX horizontal norm.
ptsayNorthM = usblSolutions.north_offset_m;
ptsayEastM = usblSolutions.east_offset_m;
ptsayHorizontalM = hypot(ptsayNorthM, ptsayEastM);
ptsaxPtsayNorthDifferenceM = ptsaxNorthM - ptsayNorthM;
ptsaxPtsayEastDifferenceM = ptsaxEastM - ptsayEastM;

%% C. PTSAG distance: PPC ship position + each USBL PTSAG solution
[ptsagEastM, ptsagNorthM, ptsagHorizontalM] = wgs84_local_offset( ...
    shipLatitudeDeg, shipLongitudeDeg, usblSolutions.latitude_deg, ...
    usblSolutions.longitude_deg);
ptsagVerticalM = usblSolutions.computed_depth_m - shipDepthM;
ptsagSlantM = hypot(ptsagHorizontalM, ptsagVerticalM);
ptsagTemporalOutlier = detect_local_outliers(ptsagHorizontalM, ...
    usblSolutions.source_name, usblSolutions.utc_time, ...
    usblSolutions.transponder_id, 5, positionLocalMinimumThresholdM, 6);
ptsagReferenceOutlier = isfinite(ptsagHorizontalM) & ...
    abs(ptsagHorizontalM - referenceHorizontalM) > ...
    positionReferenceOutlierThresholdM;
ptsagOutlier = ~isfinite(ptsagHorizontalM) | ptsagTemporalOutlier | ...
    ptsagReferenceOutlier;
filteredPtsagHorizontalM = ptsagHorizontalM;
filteredPtsagHorizontalM(ptsagOutlier) = NaN;

%% D. Propagation-time distance: 4 channels x 4 algorithms
delayEquationHorizontalM = nan(n, 4);
delayZhouHorizontalM = nan(n, 4);
delayAmsHorizontalM = nan(n, 4);
delayGmsHorizontalM = nan(n, 4);
for row = 1:n
    for channelIndex = 1:4
        [delayEquationHorizontalM(row, channelIndex), ...
            delayZhouHorizontalM(row, channelIndex), ...
            delayAmsHorizontalM(row, channelIndex), ...
            delayGmsHorizontalM(row, channelIndex)] = ...
            four_svp_horizontal_ranges(shipDepthM(row), ...
            beaconFinalDepthM(row), oneWayTimeS(row, channelIndex), svp);
    end
end

depthDifferenceM = abs(beaconFinalDepthM - shipDepthM);
delayEquationSlantM = hypot(delayEquationHorizontalM, depthDifferenceM);
delayZhouSlantM = hypot(delayZhouHorizontalM, depthDifferenceM);
delayAmsSlantM = hypot(delayAmsHorizontalM, depthDifferenceM);
delayGmsSlantM = hypot(delayGmsHorizontalM, depthDifferenceM);

% CH1..CH3 are the three channels copied into PPC/DETEC. Their filtered
% median after removing the 20 ms turnaround time is used for the main
% comparison. All raw values and flags remain available in travelTimeRange.
fusedRawRoundTripTimeS = median( ...
    filteredRawRoundTripTimeS(:, 2:4), 2, 'omitnan');
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
        four_svp_horizontal_ranges(shipDepthM(row), ...
        beaconFinalDepthM(row), fusedOneWayTimeS(row), svp);
end
fusedEquationSlantM = hypot(fusedEquationHorizontalM, depthDifferenceM);
fusedZhouSlantM = hypot(fusedZhouHorizontalM, depthDifferenceM);
fusedAmsSlantM = hypot(fusedAmsHorizontalM, depthDifferenceM);
fusedGmsSlantM = hypot(fusedGmsHorizontalM, depthDifferenceM);

%% Main one-row-per-observation comparison table
rangeComparison = table(usblSolutions.source_name, ...
    usblSolutions.utc_time, usblSolutions.local_time_utc8, ...
    usblSolutions.transponder_id, ppcLineNumber, ppcTimeDifferenceS, ...
    shipLatitudeDeg, shipLongitudeDeg, shipDepthM, shipHeadingDeg, ...
    beaconFinalLatitudeDeg, beaconFinalLongitudeDeg, beaconFinalDepthM, ...
    referenceEastM, referenceNorthM, referenceHorizontalM, ...
    referenceVerticalM, referenceSlantM, bodyForwardM, bodyStarboardM, ...
    ptsaxNorthM, ptsaxEastM, ptsaxHorizontalM, ptsaxVerticalM, ...
    ptsaxSlantM, filteredPtsaxHorizontalM, ptsaxTemporalOutlier, ...
    ptsaxReferenceOutlier, ptsaxOutlier, ...
    ptsayNorthM, ptsayEastM, ptsayHorizontalM, ...
    ptsaxPtsayNorthDifferenceM, ptsaxPtsayEastDifferenceM, ...
    ptsagEastM, ptsagNorthM, ptsagHorizontalM, ptsagVerticalM, ...
    ptsagSlantM, filteredPtsagHorizontalM, ptsagTemporalOutlier, ...
    ptsagReferenceOutlier, ptsagOutlier, ...
    validMainChannelCount, temporalOutlier, ...
    insufficientMainChannels, delayObservationOutlier, ...
    fusedRawRoundTripTimeS, ...
    fusedPropagationRoundTripTimeS, fusedOneWayTimeS, ...
    fusedEquationHorizontalM, fusedZhouHorizontalM, ...
    fusedAmsHorizontalM, fusedGmsHorizontalM, ...
    fusedEquationSlantM, fusedZhouSlantM, fusedAmsSlantM, ...
    fusedGmsSlantM, ptsaxHorizontalM - referenceHorizontalM, ...
    ptsagHorizontalM - referenceHorizontalM, ...
    filteredPtsaxHorizontalM - referenceHorizontalM, ...
    filteredPtsagHorizontalM - referenceHorizontalM, ...
    fusedEquationHorizontalM - referenceHorizontalM, ...
    fusedZhouHorizontalM - referenceHorizontalM, ...
    fusedAmsHorizontalM - referenceHorizontalM, ...
    fusedGmsHorizontalM - referenceHorizontalM, ...
    'VariableNames', {'source_name', 'utc_time', 'local_time_utc8', ...
    'beacon_id', 'ppc_line_number', 'ppc_time_difference_s', ...
    'ship_latitude_deg', 'ship_longitude_deg', 'ship_depth_m', ...
    'ship_heading_deg', 'beacon_final_latitude_deg', ...
    'beacon_final_longitude_deg', 'beacon_final_depth_m', ...
    'reference_east_m', 'reference_north_m', ...
    'reference_horizontal_m', 'reference_vertical_m', ...
    'reference_slant_m', 'ptsax_body_forward_m', ...
    'ptsax_body_starboard_m', 'ptsax_north_m', 'ptsax_east_m', ...
    'ptsax_horizontal_m', 'ptsax_vertical_m', 'ptsax_slant_m', ...
    'filtered_ptsax_horizontal_m', 'ptsax_temporal_outlier', ...
    'ptsax_reference_outlier', 'ptsax_outlier', ...
    'ptsay_north_m', 'ptsay_east_m', 'ptsay_horizontal_m', ...
    'ptsax_ptsay_north_difference_m', ...
    'ptsax_ptsay_east_difference_m', 'ptsag_east_m', 'ptsag_north_m', ...
    'ptsag_horizontal_m', 'ptsag_vertical_m', 'ptsag_slant_m', ...
    'filtered_ptsag_horizontal_m', 'ptsag_temporal_outlier', ...
    'ptsag_reference_outlier', 'ptsag_outlier', ...
    'valid_main_channel_count', 'temporal_time_outlier', ...
    'insufficient_main_channels', 'delay_observation_outlier', ...
    'fused_raw_round_trip_time_s', ...
    'fused_propagation_round_trip_time_s', 'fused_one_way_time_s', ...
    'delay_equation_horizontal_m', 'delay_zhou_horizontal_m', ...
    'delay_ams_horizontal_m', 'delay_gms_horizontal_m', ...
    'delay_equation_slant_m', 'delay_zhou_slant_m', ...
    'delay_ams_slant_m', 'delay_gms_slant_m', ...
    'ptsax_raw_horizontal_error_m', 'ptsag_raw_horizontal_error_m', ...
    'ptsax_horizontal_error_m', 'ptsag_horizontal_error_m', ...
    'delay_equation_horizontal_error_m', ...
    'delay_zhou_horizontal_error_m', 'delay_ams_horizontal_error_m', ...
    'delay_gms_horizontal_error_m'});

%% Long table preserving all four channels and all four delay algorithms
observationIndex = repelem((1:n)', 4);
channel = repmat((0:3)', n, 1);
travelTimeRange = table(observationIndex, ...
    repelem(usblSolutions.source_name, 4), ...
    repelem(usblSolutions.utc_time, 4), ...
    repelem(usblSolutions.local_time_utc8, 4), ...
    repelem(usblSolutions.transponder_id, 4), channel, ...
    reshape(rawRoundTripTimeS.', [], 1), ...
    reshape(turnaroundCorrectionS.', [], 1), ...
    reshape(propagationRoundTripTimeS.', [], 1), ...
    reshape(channelConsistencyResidualS.', [], 1), ...
    reshape(channelMissing.', [], 1), ...
    reshape(channelOutlier.', [], 1), ...
    repelem(temporalOutlier, 4), ...
    repelem(delayObservationOutlier, 4), ...
    reshape(filteredPropagationRoundTripTimeS.', [], 1), ...
    reshape(oneWayTimeS.', [], 1), ...
    reshape(delayEquationHorizontalM.', [], 1), ...
    reshape(delayZhouHorizontalM.', [], 1), ...
    reshape(delayAmsHorizontalM.', [], 1), ...
    reshape(delayGmsHorizontalM.', [], 1), ...
    reshape(delayEquationSlantM.', [], 1), ...
    reshape(delayZhouSlantM.', [], 1), ...
    reshape(delayAmsSlantM.', [], 1), ...
    reshape(delayGmsSlantM.', [], 1), ...
    repelem(referenceHorizontalM, 4), repelem(referenceSlantM, 4), ...
    reshape(delayEquationHorizontalM.', [], 1) - ...
        repelem(referenceHorizontalM, 4), ...
    reshape(delayZhouHorizontalM.', [], 1) - ...
        repelem(referenceHorizontalM, 4), ...
    reshape(delayAmsHorizontalM.', [], 1) - ...
        repelem(referenceHorizontalM, 4), ...
    reshape(delayGmsHorizontalM.', [], 1) - ...
        repelem(referenceHorizontalM, 4), ...
    'VariableNames', {'observation_index', 'source_name', 'utc_time', ...
    'local_time_utc8', 'beacon_id', 'channel', ...
    'raw_round_trip_time_s', 'turnaround_correction_s', ...
    'propagation_round_trip_time_s', ...
    'channel_consistency_residual_s', 'channel_missing', ...
    'channel_outlier', ...
    'temporal_time_outlier', 'delay_observation_outlier', ...
    'filtered_propagation_round_trip_time_s', 'one_way_time_s', ...
    'equation_horizontal_m', 'zhou_horizontal_m', ...
    'ams_horizontal_m', 'gms_horizontal_m', 'equation_slant_m', ...
    'zhou_slant_m', 'ams_slant_m', 'gms_slant_m', ...
    'reference_horizontal_m', 'reference_slant_m', ...
    'equation_horizontal_error_m', 'zhou_horizontal_error_m', ...
    'ams_horizontal_error_m', 'gms_horizontal_error_m'});

%% Error statistics and plots
methodNames = ["PTSAX"; "PTSAG"; "Delay-Equation"; "Delay-Zhou"; ...
    "Delay-AMS"; "Delay-GMS"];
estimateMatrix = [filteredPtsaxHorizontalM, filteredPtsagHorizontalM, ...
    fusedEquationHorizontalM, fusedZhouHorizontalM, ...
    fusedAmsHorizontalM, fusedGmsHorizontalM];
methodSummary = summarize_methods(usblSolutions.transponder_id, ...
    referenceHorizontalM, estimateMatrix, methodNames);
overallMethodSummary = summarize_methods(ones(n, 1), ...
    referenceHorizontalM, estimateMatrix, methodNames);
overallMethodSummary.beacon_id = [];

beaconList = unique(usblSolutions.transponder_id);
batchSourceName = strings(numel(beaconList), 1);
batchObservationCount = zeros(numel(beaconList), 1);
batchPtsaxRemovedCount = zeros(numel(beaconList), 1);
batchPtsagRemovedCount = zeros(numel(beaconList), 1);
batchDelayRetainedCount = zeros(numel(beaconList), 1);
for beaconIndex = 1:numel(beaconList)
    id = beaconList(beaconIndex);
    use = usblSolutions.transponder_id == id;
    sourceForId = unique(usblSolutions.source_name(use));
    batchSourceName(beaconIndex) = strjoin(sourceForId, "; ");
    batchObservationCount(beaconIndex) = sum(use);
    batchPtsaxRemovedCount(beaconIndex) = sum(ptsaxOutlier(use));
    batchPtsagRemovedCount(beaconIndex) = sum(ptsagOutlier(use));
    batchDelayRetainedCount(beaconIndex) = sum( ...
        ~delayObservationOutlier(use));
end
batchSummary = table(beaconList, batchSourceName, ...
    batchObservationCount, batchPtsaxRemovedCount, ...
    batchPtsagRemovedCount, batchDelayRetainedCount, ...
    'VariableNames', {'beacon_id', 'source_name', 'observation_count', ...
    'ptsax_removed_count', 'ptsag_removed_count', ...
    'delay_retained_count'});

channelNumber = (0:3)';
channelValueCount = repmat(n, 4, 1);
missingChannelCount = sum(channelMissing, 1)';
removedChannelCount = sum(channelOutlier, 1)';
retainedChannelCount = channelValueCount - missingChannelCount - ...
    removedChannelCount;
channelPreprocessingSummary = table(channelNumber, channelValueCount, ...
    missingChannelCount, removedChannelCount, retainedChannelCount, ...
    'VariableNames', {'channel', 'total_count', 'missing_count', ...
    'removed_outlier_count', 'retained_count'});
preprocessingSummary = struct;
preprocessingSummary.total_observation_count = n;
preprocessingSummary.channel_consistency_outlier_count = ...
    sum(channelOutlier, 'all');
preprocessingSummary.original_missing_channel_count = ...
    sum(channelMissing, 'all');
preprocessingSummary.temporal_outlier_observation_count = ...
    sum(temporalOutlier);
preprocessingSummary.insufficient_channel_observation_count = ...
    sum(insufficientMainChannels);
preprocessingSummary.removed_delay_observation_count = ...
    sum(delayObservationOutlier);
preprocessingSummary.retained_delay_observation_count = ...
    sum(~delayObservationOutlier);

positionMethod = ["PTSAX"; "PTSAG"];
totalPositionCount = [n; n];
missingPositionCount = [sum(~isfinite(ptsaxHorizontalM)); ...
    sum(~isfinite(ptsagHorizontalM))];
temporalPositionOutlierCount = [sum(ptsaxTemporalOutlier); ...
    sum(ptsagTemporalOutlier)];
referencePositionOutlierCount = [sum(ptsaxReferenceOutlier); ...
    sum(ptsagReferenceOutlier)];
removedPositionOutlierCount = [sum(ptsaxOutlier); sum(ptsagOutlier)];
retainedPositionCount = totalPositionCount - removedPositionOutlierCount;
positionPreprocessingSummary = table(positionMethod, totalPositionCount, ...
    missingPositionCount, temporalPositionOutlierCount, ...
    referencePositionOutlierCount, removedPositionOutlierCount, ...
    retainedPositionCount, 'VariableNames', {'method', 'total_count', ...
    'missing_count', 'temporal_outlier_count', ...
    'reference_outlier_count', 'removed_count', 'retained_count'});

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1500 900]);
tiledlayout(fig, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');
x = (1:n)';

nexttile;
plot(x, referenceHorizontalM, 'k-', 'LineWidth', 1.2); hold on;
plot(x, filteredPtsaxHorizontalM, 'Color', [0 0.45 0.74]);
plot(x, filteredPtsagHorizontalM, 'Color', [0.85 0.33 0.10]);
grid on; xlabel('观测序号'); ylabel('水平距离 / m');
title('几何测距'); legend('参考距离', 'PTSAX', 'PTSAG', ...
    'Location', 'best');

nexttile;
plot(x, filteredPtsaxHorizontalM - referenceHorizontalM, '.'); hold on;
plot(x, filteredPtsagHorizontalM - referenceHorizontalM, '.');
yline(0, 'k-'); grid on; xlabel('观测序号'); ylabel('误差 / m');
title('PTSAX与PTSAG水平距离误差'); legend('PTSAX', 'PTSAG', ...
    'Location', 'best');

nexttile;
plot(x, fusedEquationHorizontalM - referenceHorizontalM, '.'); hold on;
plot(x, fusedZhouHorizontalM - referenceHorizontalM, '.');
plot(x, fusedAmsHorizontalM - referenceHorizontalM, '.');
plot(x, fusedGmsHorizontalM - referenceHorizontalM, '.');
yline(0, 'k-'); grid on; xlabel('观测序号'); ylabel('误差 / m');
title('CH1--CH3扣除20 ms周转后：四种算法');
legend('Equation', 'Zhou', 'AMS', 'GMS', 'Location', 'best');

nexttile;
channelColors = lines(4);
for ch = 1:4
    plot(x, delayAmsHorizontalM(:, ch) - referenceHorizontalM, '.', ...
        'Color', channelColors(ch, :)); hold on;
end
yline(0, 'k-'); grid on; xlabel('观测序号'); ylabel('误差 / m');
title('AMS方法的四通道对比');
legend('CH0', 'CH1', 'CH2', 'CH3', 'Location', 'best');

exportgraphics(fig, figurePng, 'Resolution', 180);
savefig(fig, figureFig);
close(fig);

% Generate one plot per calibration beacon so the two stages are not
% visually concatenated into a single observation axis.
for beaconIndex = 1:numel(beaconList)
    id = beaconList(beaconIndex);
    use = usblSolutions.transponder_id == id;
    localX = (1:sum(use))';
    idFig = figure('Visible', 'off', 'Color', 'w', ...
        'Position', [100 100 1500 900]);
    tiledlayout(idFig, 2, 2, 'Padding', 'compact', ...
        'TileSpacing', 'compact');

    nexttile;
    plot(localX, referenceHorizontalM(use), 'k-', 'LineWidth', 1.2);
    hold on;
    plot(localX, filteredPtsaxHorizontalM(use), ...
        'Color', [0 0.45 0.74]);
    plot(localX, filteredPtsagHorizontalM(use), ...
        'Color', [0.85 0.33 0.10]);
    grid on; xlabel('观测序号'); ylabel('水平距离 / m');
    title(sprintf('ID%d：几何测距', id));
    legend('参考距离', 'PTSAX', 'PTSAG', 'Location', 'best');

    nexttile;
    plot(localX, filteredPtsaxHorizontalM(use) - ...
        referenceHorizontalM(use), '.'); hold on;
    plot(localX, filteredPtsagHorizontalM(use) - ...
        referenceHorizontalM(use), '.');
    yline(0, 'k-'); grid on; xlabel('观测序号'); ylabel('误差 / m');
    title(sprintf('ID%d：PTSAX与PTSAG误差', id));
    legend('PTSAX', 'PTSAG', 'Location', 'best');

    nexttile;
    plot(localX, fusedEquationHorizontalM(use) - ...
        referenceHorizontalM(use), '.'); hold on;
    plot(localX, fusedZhouHorizontalM(use) - ...
        referenceHorizontalM(use), '.');
    plot(localX, fusedAmsHorizontalM(use) - ...
        referenceHorizontalM(use), '.');
    plot(localX, fusedGmsHorizontalM(use) - ...
        referenceHorizontalM(use), '.');
    yline(0, 'k-'); grid on; xlabel('观测序号'); ylabel('误差 / m');
    title(sprintf('ID%d：四种传播时延算法', id));
    legend('Equation', 'Zhou', 'AMS', 'GMS', 'Location', 'best');

    nexttile;
    for ch = 1:4
        plot(localX, delayAmsHorizontalM(use, ch) - ...
            referenceHorizontalM(use), '.', ...
            'Color', channelColors(ch, :));
        hold on;
    end
    yline(0, 'k-'); grid on; xlabel('观测序号'); ylabel('误差 / m');
    title(sprintf('ID%d：AMS四通道对比', id));
    legend('CH0', 'CH1', 'CH2', 'CH3', 'Location', 'best');

    idFigurePng = fullfile(outputDir, sprintf( ...
        'step2_range_comparison_id%d.png', id));
    idFigureFig = fullfile(outputDir, sprintf( ...
        'step2_range_comparison_id%d.fig', id));
    exportgraphics(idFig, idFigurePng, 'Resolution', 180);
    savefig(idFig, idFigureFig);
    close(idFig);
end

% Export user-readable accuracy tables in the requested range_cmp folder.
writetable(overallMethodSummary, accuracyOverallCsv);
writetable(methodSummary, accuracyByIdCsv);
writetable(batchSummary, batchSummaryCsv);
if isfile(accuracyWorkbook)
    delete(accuracyWorkbook);
end
writetable(overallMethodSummary, accuracyWorkbook, 'Sheet', 'Overall');
for beaconIndex = 1:numel(beaconList)
    id = beaconList(beaconIndex);
    writetable(methodSummary(methodSummary.beacon_id == id, :), ...
        accuracyWorkbook, 'Sheet', sprintf('ID%d', id));
end
writetable(positionPreprocessingSummary, accuracyWorkbook, ...
    'Sheet', 'PositionPreprocessing');
writetable(channelPreprocessingSummary, accuracyWorkbook, ...
    'Sheet', 'DelayPreprocessing');
writetable(batchSummary, accuracyWorkbook, 'Sheet', 'BatchSummary');

settings = struct;
settings.input_file = inputFile;
settings.svp_file = svpFile;
settings.svp_processing = ...
    "measured down/up cast; median sound speed in 1 m depth bins; " + ...
    "constant extension over the final approximately 5 m to beacon depth";
settings.travel_time_is_round_trip = true;
settings.transponder_turnaround_time_s = turnaroundTimeS;
settings.turnaround_correction = ...
    "CH0: 0 s; CH1--CH3: subtract 0.020 s before halving";
settings.one_way_time_definition = ...
    "(raw round-trip time - applicable turnaround correction) / 2";
settings.channel_consistency_threshold_s = channelConsistencyThresholdS;
settings.temporal_window_half_width = 5;
settings.temporal_minimum_threshold_s = 0.005;
settings.temporal_robust_sigma_factor = 6;
settings.outlier_handling = ...
    "removed channel/observation values are set to NaN; raw values and " + ...
    "logical outlier flags are retained";
settings.position_reference_outlier_threshold_m = ...
    positionReferenceOutlierThresholdM;
settings.position_local_minimum_threshold_m = ...
    positionLocalMinimumThresholdM;
settings.main_delay_fusion = ...
    "median of corrected CH1, CH2 and CH3 propagation times";
settings.delay_methods = methodNames(3:end);
settings.reference_definition = ...
    "WGS-84 horizontal distance between PPC ship position and the fixed " + ...
    "final calibrated beacon coordinate";
settings.created_at = datetime('now', 'TimeZone', 'Asia/Shanghai');

save(outputFile, 'rangeComparison', 'travelTimeRange', 'methodSummary', ...
    'overallMethodSummary', 'channelPreprocessingSummary', ...
    'positionPreprocessingSummary', 'preprocessingSummary', ...
    'batchSummary', 'beaconCalibration', 'svp', 'settings', '-v7.3');

% Save a self-contained MAT for each beacon calibration stage.
for beaconIndex = 1:numel(beaconList)
    id = beaconList(beaconIndex);
    useObservation = rangeComparison.beacon_id == id;
    useTravelTime = travelTimeRange.beacon_id == id;
    batchOutput = struct;
    batchOutput.rangeComparison = rangeComparison(useObservation, :);
    batchOutput.travelTimeRange = travelTimeRange(useTravelTime, :);
    batchOutput.methodSummary = methodSummary( ...
        methodSummary.beacon_id == id, :);
    batchOutput.overallMethodSummary = summarize_methods( ...
        ones(sum(useObservation), 1), ...
        referenceHorizontalM(useObservation), ...
        estimateMatrix(useObservation, :), methodNames);
    batchOutput.overallMethodSummary.beacon_id = [];
    batchOutput.batchSummary = batchSummary(batchSummary.beacon_id == id, :);
    batchOutput.beaconCalibration = beaconCalibration( ...
        beaconCalibration.beacon_id == id, :);
    batchOutput.svp = svp;
    batchOutput.settings = settings;
    batchOutputFile = fullfile(outputDir, sprintf( ...
        'step2_range_comparison_id%d.mat', id));
    save(batchOutputFile, '-struct', 'batchOutput', '-v7.3');
end

fprintf('\nStep 2 range calculation complete.\n');
fprintf('  Observations: %d\n', height(rangeComparison));
fprintf('  Per-channel delay rows: %d\n', height(travelTimeRange));
fprintf('  Removed channel values: %d\n', sum(channelOutlier, 'all'));
fprintf('  Removed delay observations: %d\n', sum(delayObservationOutlier));
fprintf('  Removed PTSAX values: %d\n', sum(ptsaxOutlier));
fprintf('  Removed PTSAG values: %d\n', sum(ptsagOutlier));
fprintf('  Output MAT: %s\n', outputFile);
fprintf('  Figure: %s\n\n', figurePng);
fprintf('  Accuracy workbook: %s\n\n', accuracyWorkbook);
disp(methodSummary);
disp(overallMethodSummary);
disp(channelPreprocessingSummary);
disp(positionPreprocessingSummary);
disp(batchSummary);

%% Local helpers used by this directly runnable script
function svp = read_svp_file(filename)
headerId = fopen(filename, 'rt');
if headerId < 0
    error('USBL:CannotOpenSVP', 'Cannot open SVP file: %s', filename);
end
firstLine = strtrim(fgetl(headerId));
fclose(headerId);
if ~isempty(regexp(firstLine, '^\d{4}-\d{2}-\d{2}\s', 'once'))
    % Measured file columns: date, time, depth (m), sound speed (m/s),
    % auxiliary sensor value. It contains both the downcast and upcast.
    fileId = fopen(filename, 'rt');
    cleanup = onCleanup(@() fclose(fileId));
    columns = textscan(fileId, '%s %s %f %f %f', ...
        'Delimiter', ' ', 'MultipleDelimsAsOne', true);
    rawDepthM = columns{3};
    rawSoundSpeedMps = columns{4};
    valid = isfinite(rawDepthM) & isfinite(rawSoundSpeedMps) & ...
        rawDepthM >= 0 & rawSoundSpeedMps > 1300 & ...
        rawSoundSpeedMps < 1700;
    rawDepthM = rawDepthM(valid);
    rawSoundSpeedMps = rawSoundSpeedMps(valid);

    depthBinM = round(rawDepthM);
    [depthM, ~, group] = unique(depthBinM);
    soundSpeedMps = splitapply(@median, rawSoundSpeedMps, group);
    if depthM(1) > 0
        depthM = [0; depthM];
        soundSpeedMps = [soundSpeedMps(1); soundSpeedMps];
    end
else
    % Two-column profile format used by the supplied range.m.
    raw = readlines(filename, 'EmptyLineRule', 'skip');
    depthM = nan(numel(raw), 1);
    soundSpeedMps = nan(numel(raw), 1);
    row = 0;
    for i = 1:numel(raw)
        values = sscanf(char(raw(i)), '%f %f /');
        if numel(values) >= 2
            row = row + 1;
            depthM(row) = values(1);
            soundSpeedMps(row) = values(2);
        end
    end
    depthM = depthM(1:row);
    soundSpeedMps = soundSpeedMps(1:row);
    [depthM, order] = sort(depthM);
    soundSpeedMps = soundSpeedMps(order);
end
svp = table(depthM, soundSpeedMps, ...
    'VariableNames', {'depth_m', 'sound_speed_mps'});
end

function [latitudeDeg, longitudeDeg, depthM] = ...
        lookup_beacon_calibration(ids, calibration)
n = numel(ids);
latitudeDeg = nan(n, 1);
longitudeDeg = nan(n, 1);
depthM = nan(n, 1);
for k = 1:height(calibration)
    use = ids == calibration.beacon_id(k);
    latitudeDeg(use) = calibration.latitude_deg(k);
    longitudeDeg(use) = calibration.longitude_deg(k);
    depthM(use) = calibration.depth_m(k);
end
if any(~isfinite(latitudeDeg))
    error('USBL:MissingCalibration', ...
        'Some beacon IDs do not have final calibration coordinates.');
end
end

function [indices, timeDifferenceS] = nearest_source_id_time( ...
        querySource, queryTime, queryId, dataSource, dataTime, dataId, ...
        maximumDifferenceS)
n = numel(queryTime);
indices = nan(n, 1);
timeDifferenceS = nan(n, 1);
sources = unique(querySource);
for sourceIndex = 1:numel(sources)
    source = sources(sourceIndex);
    sourceQuery = querySource == source;
    for id = unique(queryId(sourceQuery))'
        q = find(sourceQuery & queryId == id);
        d = find(dataSource == source & dataId == id);
        if isempty(d)
            continue;
        end
        [dataSeconds, order] = sort(posixtime(dataTime(d)));
        nearestSorted = interp1(dataSeconds, (1:numel(dataSeconds))', ...
            posixtime(queryTime(q)), 'nearest', 'extrap');
        selected = d(order(nearestSorted));
        difference = seconds(dataTime(selected) - queryTime(q));
        good = abs(difference) <= maximumDifferenceS;
        indices(q(good)) = selected(good);
        timeDifferenceS(q(good)) = difference(good);
    end
end
end

function [eastM, northM, horizontalM] = wgs84_local_offset( ...
        latitude1Deg, longitude1Deg, latitude2Deg, longitude2Deg)
a = 6378137.0;
e2 = 6.6943799901413165e-3;
latitudeMeanRad = deg2rad((latitude1Deg + latitude2Deg) / 2);
dLatitudeRad = deg2rad(latitude2Deg - latitude1Deg);
dLongitudeRad = deg2rad(longitude2Deg - longitude1Deg);
sinLatitude = sin(latitudeMeanRad);
primeVerticalRadiusM = a ./ sqrt(1 - e2 .* sinLatitude.^2);
meridianRadiusM = a * (1 - e2) ./ ...
    (1 - e2 .* sinLatitude.^2).^(3/2);
eastM = dLongitudeRad .* primeVerticalRadiusM .* cos(latitudeMeanRad);
northM = dLatitudeRad .* meridianRadiusM;
horizontalM = hypot(eastM, northM);
end

function [equationM, zhouM, amsM, gmsM] = ...
        four_svp_horizontal_ranges(sourceDepthM, receiverDepthM, ...
        oneWayTimeS, svp)
equationM = NaN;
zhouM = NaN;
amsM = NaN;
gmsM = NaN;
if ~all(isfinite([sourceDepthM, receiverDepthM, oneWayTimeS])) || ...
        oneWayTimeS <= 0
    return;
end

depthDifferenceM = abs(receiverDepthM - sourceDepthM);
if depthDifferenceM < eps
    localSpeed = interp1(svp.depth_m, svp.sound_speed_mps, ...
        sourceDepthM, 'linear', 'extrap');
    equationM = oneWayTimeS * localSpeed;
    zhouM = equationM;
    amsM = equationM;
    gmsM = equationM;
    return;
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

% Arithmetic mean and the GMS expression copied from range.m. The latter is
% mathematically the harmonic mean along depth.
arithmeticMeanMps = speedIntegral / depthDifferenceM;
gmsMeanMps = depthDifferenceM / ...
    trapz(profileDepthM, 1 ./ profileSpeedMps);
amsM = real_sqrt_nonnegative((oneWayTimeS * arithmeticMeanMps)^2 - ...
    depthDifferenceM^2);
gmsM = real_sqrt_nonnegative((oneWayTimeS * gmsMeanMps)^2 - ...
    depthDifferenceM^2);

% Linear-gradient analytical equation, following the supplied range.m.
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
    receiverLinearSpeedMps = sourceSpeedMps + gradient * ...
        (receiverDepthM - sourceDepthM);
    exponential = exp(oneWayTimeS * gradient);
    expression = (sourceSpeedMps * exponential - ...
        receiverLinearSpeedMps) * (sourceSpeedMps - ...
        receiverLinearSpeedMps * exponential) / exponential;
    equationM = abs(1 / gradient) * sqrt(abs(expression));
end

% Zhou ray-parameter method.
verticalTimeS = trapz(profileDepthM, 1 ./ profileSpeedMps);
if oneWayTimeS < verticalTimeS
    return;
end
lower = 0;
upper = (1 - 1e-12) / max(profileSpeedMps);
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

function value = real_sqrt_nonnegative(valueSquared)
if isfinite(valueSquared) && valueSquared >= 0
    value = sqrt(valueSquared);
else
    value = NaN;
end
end

function outlier = detect_local_outliers(values, sources, times, ids, ...
        halfWindow, minimumThreshold, robustSigmaFactor)
outlier = false(size(values));
sourceList = unique(sources);
for sourceIndex = 1:numel(sourceList)
    source = sourceList(sourceIndex);
    sourceRows = sources == source;
    idList = unique(ids(sourceRows));
    for idIndex = 1:numel(idList)
        rows = find(sourceRows & ids == idList(idIndex));
        [~, order] = sort(times(rows));
        rows = rows(order);
        if numel(rows) < 2 * halfWindow + 1
            continue;
        end
        for position = (halfWindow + 1):(numel(rows) - halfWindow)
            localRows = rows(position-halfWindow:position+halfWindow);
            centerRow = rows(position);
            localRows(halfWindow + 1) = [];
            neighbors = values(localRows);
            neighbors = neighbors(isfinite(neighbors));
            if ~isfinite(values(centerRow)) || numel(neighbors) < halfWindow
                continue;
            end
            localMedian = median(neighbors);
            localMad = median(abs(neighbors - localMedian));
            localRobustSigma = 1.4826 * localMad;
            threshold = max(minimumThreshold, ...
                robustSigmaFactor * localRobustSigma);
            outlier(centerRow) = ...
                abs(values(centerRow) - localMedian) > threshold;
        end
    end
end
end

function summary = summarize_methods(beaconIds, referenceM, ...
        estimateMatrix, methodNames)
uniqueBeaconIds = unique(beaconIds);
nRows = numel(uniqueBeaconIds) * numel(methodNames);
beaconId = nan(nRows, 1);
method = strings(nRows, 1);
validCount = zeros(nRows, 1);
meanErrorM = nan(nRows, 1);
medianErrorM = nan(nRows, 1);
stdErrorM = nan(nRows, 1);
maeM = nan(nRows, 1);
rmseM = nan(nRows, 1);
p95AbsoluteErrorM = nan(nRows, 1);
row = 0;
for beaconIndex = 1:numel(uniqueBeaconIds)
    useBeacon = beaconIds == uniqueBeaconIds(beaconIndex);
    for methodIndex = 1:numel(methodNames)
        row = row + 1;
        errorM = estimateMatrix(useBeacon, methodIndex) - ...
            referenceM(useBeacon);
        errorM = errorM(isfinite(errorM));
        beaconId(row) = uniqueBeaconIds(beaconIndex);
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
