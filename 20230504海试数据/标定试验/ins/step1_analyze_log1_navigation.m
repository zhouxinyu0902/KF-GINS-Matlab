%% STEP1_ANALYZE_LOG1_NAVIGATION Analyse the PHINS navigation solution in log1.
% Executable script (not a function).
%
% Navigation provenance:
%   log1-*.dat
%       -> PHINS/PostproDecode vendor decoding
%       -> IMU (50 Hz) + DVL bottom track + depth aiding
%       -> proprietary PHINS navigation/Kalman solution
%       -> log1-*-ins-INSNAV.xpf.txt
%
% This script analyses the supplied navigation solution. It does not try to
% reproduce the proprietary PHINS navigation filter. GPS, LBL and USBL are
% explicitly marked as not received in log1-*-ins.conf.xml.
%
% Figures follow the project convention and are created only through
% myfigurestartup(..., 'paper'). All table exports are tab-delimited UTF-8
% TXT files.

clear;
clc;
close all;

%% Paths
scriptsDir = string(fileparts(mfilename('fullpath')));
projectDir = string(fileparts(scriptsDir));
repositoryDir = string(fileparts(projectDir));
dataDir = fullfile(projectDir, "信标标定试验数据");
parsedDir = fullfile(projectDir, "解析后的数据", "ins_batch");
outputDir = fullfile(projectDir, "解析后的数据", "step1_nav_analysis");
figureHelperDir = fullfile(repositoryDir, "psins2401", "mytest", ...
    "00_all_func");

navFile = fullfile(dataDir, "log1-20230430-0347-ins-INSNAV.xpf.txt");
imuFile = fullfile(dataDir, "log1-20230430-0347-ins-IMU.xpf.txt");
dvlFile = fullfile(dataDir, "log1-20230430-0347-ins-DVL.xpf.txt");
depthFile = fullfile(dataDir, "log1-20230430-0347-ins-DEPTH.xpf.txt");
configurationFile = fullfile(dataDir, "log1-20230430-0347-ins.conf.xml");
decodeLogFile = fullfile(dataDir, "log1-20230430-0347.log.xml");
decodedNavFile = fullfile(parsedDir, "log1_navigation.txt");
figureHelperFile = fullfile(figureHelperDir, "myfigurestartup.m");

requiredFiles = [navFile, imuFile, dvlFile, depthFile, ...
    configurationFile, decodeLogFile, figureHelperFile];
for k = 1:numel(requiredFiles)
    mustBeFile(requiredFiles(k));
end
if ~isfolder(outputDir), mkdir(outputDir); end
addpath(figureHelperDir);
cleanupFigurePath = onCleanup(@() rmpath(figureHelperDir)); %#ok<NASGU>

fprintf('Step 1: analysing PHINS log1 navigation\n');
fprintf('  Navigation source: %s\n', navFile);
fprintf('  Output directory : %s\n', outputDir);

%% Read the high-precision vendor exports
navNames = {'algo_status_1','algo_status_2','algo_status_3', ...
    'latitude_deg','longitude_deg','altitude_m','heading_deg','roll_deg', ...
    'pitch_deg','velocity_north_mps','velocity_east_mps', ...
    'velocity_up_mps','heave_m','surge_m','sway_m'};
imuNames = {'frame_counter','fog_1','fog_2','fog_3', ...
    'accelerometer_1','accelerometer_2','accelerometer_3','sensor_status'};
dvlNames = {'velocity_x_mps','velocity_y_mps','velocity_z_mps', ...
    'sensor_sound_speed_mps','external_sound_speed_mps','altitude_m', ...
    'velocity_x_std_mps','velocity_y_std_mps','velocity_z_std_mps'};
depthNames = {'status','depth_m','depth_std_m','pressure_pa'};

nav = read_xpf_export(navFile, navNames);
imu = read_xpf_export(imuFile, imuNames);
dvl = read_xpf_export(dvlFile, dvlNames);
depth = read_xpf_export(depthFile, depthNames);

assert(~isempty(nav), 'STEP1:EmptyNavigation', ...
    'No navigation rows were read from %s.', navFile);

%% Navigation statistics and aiding consistency
earthRadiusM = 6378137;
lat0Deg = nav.latitude_deg(1);
lon0Deg = nav.longitude_deg(1);
northM = deg2rad(nav.latitude_deg-lat0Deg)*earthRadiusM;
eastM = deg2rad(nav.longitude_deg-lon0Deg)*earthRadiusM*cosd(lat0Deg);
elapsedMin = minutes(nav.utc_time-nav.utc_time(1));
horizontalSpeed = hypot(nav.velocity_north_mps, nav.velocity_east_mps);
trackDistanceM = sum(hypot(diff(eastM), diff(northM)), 'omitnan');
displacementM = hypot(eastM(end)-eastM(1), northM(end)-northM(1));
durationS = seconds(nav.utc_time(end)-nav.utc_time(1));
navRateHz = 1/median(seconds(diff(nav.utc_time)), 'omitnan');
imuRateHz = 1/median(seconds(diff(imu.utc_time)), 'omitnan');

navSeconds = posixtime(nav.utc_time);
depthSeconds = posixtime(depth.utc_time);
dvlSeconds = posixtime(dvl.utc_time);

depthAtNav = interp_valid(depthSeconds, depth.depth_m, navSeconds);
depthResidualM = -nav.altitude_m-depthAtNav;
depthMask = isfinite(depthResidualM);

navSpeedAtDvl = interp_valid(navSeconds, horizontalSpeed, dvlSeconds);
dvlHorizontalSpeed = hypot(dvl.velocity_x_mps, dvl.velocity_y_mps);
speedMask = isfinite(navSpeedAtDvl) & isfinite(dvlHorizontalSpeed);
if nnz(speedMask) >= 2
    speedCorrelation = corr(navSpeedAtDvl(speedMask), ...
        dvlHorizontalSpeed(speedMask));
    speedDifferenceRmsMps = sqrt(mean((navSpeedAtDvl(speedMask)- ...
        dvlHorizontalSpeed(speedMask)).^2));
else
    speedCorrelation = NaN;
    speedDifferenceRmsMps = NaN;
end

metric = ["navigation rows"; "navigation duration"; ...
    "navigation median rate"; "IMU median rate"; "DVL rows"; ...
    "depth rows"; "horizontal track length"; ...
    "start-to-end displacement"; "median horizontal speed"; ...
    "maximum horizontal speed"; "median INS-depth residual"; ...
    "RMS INS-depth residual"; "INS/DVL horizontal-speed correlation"; ...
    "INS/DVL horizontal-speed RMS difference"];
value = [height(nav); durationS; navRateHz; imuRateHz; height(dvl); ...
    height(depth); trackDistanceM; displacementM; ...
    median(horizontalSpeed,'omitnan'); max(horizontalSpeed,[],'omitnan'); ...
    median(depthResidualM(depthMask),'omitnan'); ...
    sqrt(mean(depthResidualM(depthMask).^2,'omitnan')); ...
    speedCorrelation; speedDifferenceRmsMps];
unit = ["row"; "s"; "Hz"; "Hz"; "row"; "row"; "m"; "m"; ...
    "m/s"; "m/s"; "m"; "m"; "1"; "m/s"];
navSummary = table(metric, value, unit);

%% Algorithm-status occurrence summary
statusMatrix = [nav.algo_status_1, nav.algo_status_2, nav.algo_status_3];
[uniqueStatus, ~, statusGroup] = unique(statusMatrix, 'rows', 'stable');
rowCount = accumarray(statusGroup, 1);
percentage = 100*rowCount/height(nav);
statusSummary = table(uniqueStatus(:,1), uniqueStatus(:,2), ...
    uniqueStatus(:,3), rowCount, percentage, ...
    'VariableNames', {'algo_status_1','algo_status_2','algo_status_3', ...
    'row_count','percentage'});

%% Record what produced the navigation result
dataItem = ["IMU"; "DVL bottom track"; "depth sensor"; "GPS 1/2/manual"; ...
    "LBL"; "USBL"; "UTC synchronization"; "INSNAV result"];
receivedOrAvailable = ["yes"; "yes"; "yes"; "no"; "no"; "no"; ...
    "yes"; "yes"];
role = [ ...
    "Primary inertial increments; vendor log states 50 Hz"; ...
    "External bottom-track velocity aiding"; ...
    "Vertical/depth aiding; altitude mode is depth"; ...
    "Not received according to the PHINS configuration"; ...
    "Not received according to the PHINS configuration"; ...
    "Not received according to the PHINS configuration"; ...
    "Time and PPS; INS clock drift is compensated"; ...
    "PHINS proprietary navigation/Kalman output, not recomputed here"];
evidenceFile = [repmat(string(imuFile),1,1); string(dvlFile); ...
    string(depthFile); repmat(string(configurationFile),3,1); ...
    string(decodeLogFile); string(navFile)];
sourceTrace = table(dataItem, receivedOrAvailable, role, evidenceFile, ...
    'VariableNames', {'data_item','received_or_available','role', ...
    'evidence_file'});

%% Compare the high-precision INSNAV export with the decoded PhinsNav table
comparisonRows = cell(0,1);
if isfile(decodedNavFile)
    decoded = readtable(decodedNavFile, 'FileType','text', ...
        'Delimiter','\t', 'VariableNamingRule','preserve');
    nCompare = min(height(nav), height(decoded));
    officialFields = {'latitude_deg','longitude_deg','altitude_m', ...
        'heading_deg','roll_deg','pitch_deg','velocity_north_mps', ...
        'velocity_east_mps','velocity_up_mps'};
    decodedFields = {'latitude_deg','longitude_deg','altitude_m', ...
        'heading_deg','roll_deg','pitch_deg','velocity_north_mps', ...
        'velocity_east_mps','velocity_up_mps'};
    units = {'deg','deg','m','deg','deg','deg','m/s','m/s','m/s'};
    for k = 1:numel(officialFields)
        a = nav.(officialFields{k})(1:nCompare);
        b = decoded.(decodedFields{k})(1:nCompare);
        delta = b-a;
        comparisonRows{end+1,1} = table(string(officialFields{k}), ... %#ok<SAGROW>
            nCompare, mean(abs(delta),'omitnan'), ...
            max(abs(delta),[],'omitnan'), string(units{k}), ...
            'VariableNames', {'variable','compared_rows', ...
            'mean_absolute_difference','maximum_absolute_difference','unit'});
    end
    sourceComparison = vertcat(comparisonRows{:});
else
    sourceComparison = table("decoded navigation file not found", 0, NaN, ...
        NaN, "", 'VariableNames', {'variable','compared_rows', ...
        'mean_absolute_difference','maximum_absolute_difference','unit'});
end

%% Export text results
write_txt(navSummary, fullfile(outputDir, "step1_nav_summary.txt"));
write_txt(statusSummary, fullfile(outputDir, "step1_algorithm_status_summary.txt"));
write_txt(sourceTrace, fullfile(outputDir, "step1_navigation_source_trace.txt"));
write_txt(sourceComparison, ...
    fullfile(outputDir, "step1_navigation_source_comparison.txt"));

%% Figure 1: local navigation track (1-by-1 -> 3 by 3 paper size)
figTrack = myfigurestartup(3, 3, 'paper');
set(figTrack, 'Visible','off');
axTrack = gca(figTrack);
cla(axTrack);
scatter(axTrack, eastM, northM, 8, elapsedMin, 'filled', ...
    'HandleVisibility','off');
plot(axTrack, eastM(1), northM(1), 'go', 'MarkerFaceColor','g', ...
    'DisplayName','Start');
plot(axTrack, eastM(end), northM(end), 'rs', 'MarkerFaceColor','r', ...
    'DisplayName','End');
axis(axTrack, 'equal');
grid(axTrack, 'on');
xlabel(axTrack, 'Local east (m)');
ylabel(axTrack, 'Local north (m)');
title(axTrack, 'PHINS log1 navigation track');
cb = colorbar(axTrack);
cb.Label.String = 'Elapsed time (min)';
legend(axTrack, 'Location','best');
export_figure(figTrack, outputDir, "step1_nav_track");
close(figTrack);

%% Figure 2: navigation states and aiding checks (2-by-2 -> 7 by 7)
figStates = myfigurestartup(7, 7, 'paper');
set(figStates, 'Visible','off');
delete(findall(figStates, 'Type','axes'));
layout = tiledlayout(figStates, 2, 2, 'Padding','compact', ...
    'TileSpacing','compact');

ax1 = nexttile(layout, 1);
hold(ax1, 'on');
plot(ax1, nav.utc_time, nav.heading_deg, 'DisplayName','Heading');
plot(ax1, nav.utc_time, nav.roll_deg, 'DisplayName','Roll');
plot(ax1, nav.utc_time, nav.pitch_deg, 'DisplayName','Pitch');
grid(ax1,'on'); ylabel(ax1,'Angle (deg)');
title(ax1,'(a) Attitude'); legend(ax1,'Location','best');
xtickformat(ax1,'HH:mm');

ax2 = nexttile(layout, 2);
hold(ax2, 'on');
plot(ax2, nav.utc_time, nav.velocity_north_mps, 'DisplayName','North');
plot(ax2, nav.utc_time, nav.velocity_east_mps, 'DisplayName','East');
plot(ax2, nav.utc_time, nav.velocity_up_mps, 'DisplayName','Up');
grid(ax2,'on'); ylabel(ax2,'Velocity (m/s)');
title(ax2,'(b) Navigation velocity'); legend(ax2,'Location','best');
xtickformat(ax2,'HH:mm');

ax3 = nexttile(layout, 3);
hold(ax3, 'on');
plot(ax3, nav.utc_time, -nav.altitude_m, 'DisplayName','-INS altitude');
plot(ax3, depth.utc_time, depth.depth_m, 'DisplayName','Depth sensor');
grid(ax3,'on'); xlabel(ax3,'UTC'); ylabel(ax3,'Depth (m)');
title(ax3,'(c) Depth aiding consistency'); legend(ax3,'Location','best');
xtickformat(ax3,'HH:mm');

ax4 = nexttile(layout, 4);
hold(ax4, 'on');
plot(ax4, nav.utc_time, horizontalSpeed, 'DisplayName','INS horizontal');
plot(ax4, dvl.utc_time, dvlHorizontalSpeed, '.', ...
    'DisplayName','DVL horizontal magnitude');
grid(ax4,'on'); xlabel(ax4,'UTC'); ylabel(ax4,'Speed (m/s)');
title(ax4,'(d) DVL/INS speed consistency'); legend(ax4,'Location','best');
xtickformat(ax4,'HH:mm');

title(layout, 'PHINS log1 navigation result and aiding-data checks');
export_figure(figStates, outputDir, "step1_nav_states");
close(figStates);

save(fullfile(outputDir, "step1_log1_navigation_analysis.mat"), ...
    'nav','imu','dvl','depth','navSummary','statusSummary','sourceTrace', ...
    'sourceComparison','eastM','northM','horizontalSpeed', ...
    'depthResidualM','navSpeedAtDvl','dvlHorizontalSpeed','-v7.3');

fprintf('\nStep 1 completed.\n');
fprintf('  Navigation rows       : %d\n', height(nav));
fprintf('  Duration              : %.2f min\n', durationS/60);
fprintf('  Track length          : %.1f m\n', trackDistanceM);
fprintf('  IMU / nav median rate : %.2f / %.2f Hz\n', imuRateHz, navRateHz);
fprintf('  DVL / depth rows      : %d / %d\n', height(dvl), height(depth));
fprintf('  Results               : %s\n', outputDir);

%% Local helpers
function tbl = read_xpf_export(file, numericNames)
fid = fopen(file, 'rt');
assert(fid >= 0, 'STEP1:OpenFailed', 'Cannot open %s.', file);
cleanupFile = onCleanup(@() fclose(fid)); %#ok<NASGU>
format = ['%s%s', repmat('%f', 1, numel(numericNames))];
values = textscan(fid, format, 'Delimiter','\t', 'CommentStyle','#', ...
    'MultipleDelimsAsOne',false, 'CollectOutput',false, ...
    'ReturnOnError',false);
dateText = string(values{1});
timeText = string(values{2});
utcTime = datetime(dateText+" "+timeText, ...
    'InputFormat','yyyy/MM/dd HH:mm:ss.SSSS', 'TimeZone','UTC');
utcTime.Format = 'yyyy-MM-dd HH:mm:ss.SSSS';
numericData = [values{3:end}];
tbl = array2table(numericData, 'VariableNames', numericNames);
tbl.utc_time = utcTime;
tbl = movevars(tbl, 'utc_time', 'Before', 1);
end

function yq = interp_valid(x, y, xq)
valid = isfinite(x) & isfinite(y);
x = x(valid); y = y(valid);
[x, keep] = unique(x, 'stable');
y = y(keep);
if numel(x) < 2
    yq = nan(size(xq));
else
    yq = interp1(x, y, xq, 'linear', NaN);
end
end

function write_txt(tbl, file)
writetable(tbl, file, 'FileType','text', 'Delimiter','\t', ...
    'Encoding','UTF-8');
end

function export_figure(fig, outputDir, stem)
exportgraphics(fig, fullfile(outputDir, stem+".png"), 'Resolution',220);
exportgraphics(fig, fullfile(outputDir, stem+".pdf"), ...
    'ContentType','vector');
end
