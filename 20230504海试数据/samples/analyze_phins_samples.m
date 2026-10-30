%% ANALYZE_PHINS_SAMPLES
% Read the PHINS XPF/TXT sample exports in this folder and create a first
% navigation overview without using USBL observations.
%
% Original sample files are read only. Results are written to the local
% analysis_results subfolder next to this script.

clear;
clc;
close all;

scriptDir = string(fileparts(mfilename('fullpath')));
repoDir = fileparts(fileparts(scriptDir));
dataDir = fullfile(repoDir, "data", "20230504", "实际海试", "samples");
outputDir = fullfile(scriptDir, "analysis_results");
if ~isfolder(outputDir)
    mkdir(outputDir);
end

base = "PHINS_6000-D-20140924-103004";
navFile = fullfile(dataDir, base + "-ins-nav.xpf.txt");
imuFile = fullfile(dataDir, base + "-ins-imu.xpf.txt");
dvlFile = fullfile(dataDir, base + "-ins-dvl.xpf.txt");
depthFile = fullfile(dataDir, base + "-ins-depth.xpf.txt");
gpsFile = fullfile(dataDir, base + "-ins-gps.xpf.txt");
lblFile = fullfile(dataDir, base + "-ins-lbl.xpf.txt");

requiredFiles = [navFile, imuFile, dvlFile, depthFile, gpsFile, lblFile];
for k = 1:numel(requiredFiles)
    if ~isfile(requiredFiles(k))
        error('PHINS:MissingSampleFile', 'Missing sample file: %s', requiredFiles(k));
    end
end

fprintf('PHINS sample analysis without USBL\n');
fprintf('Input : %s\n', dataDir);
fprintf('Output: %s\n\n', outputDir);

nav = read_xpf_table(navFile, ...
    {'algoStatus1','algoStatus2','algoStatus3','latitude_deg', ...
     'longitude_deg','altitude_m','heading_deg','roll_deg','pitch_deg', ...
     'speedNorth_mps','speedEast_mps','speedUp_mps','heave_m','surge_m','sway_m'});

dvl = read_xpf_table(dvlFile, ...
    {'speedX_mps','speedY_mps','speedZ_mps','sensorSoundVelocity_mps', ...
     'externalSoundVelocity_mps','altitude_m','speedXSd_mps', ...
     'speedYSd_mps','speedZSd_mps'});

depth = read_xpf_table(depthFile, ...
    {'status','depth_m','depthSd_m','pressure_Pa'});

gps = read_xpf_table(gpsFile, ...
    {'gpsMode','latitude_deg','longitude_deg','altitude_m', ...
     'northingSd_m','eastingSd_m','altitudeSd_m','northingEastingCv', ...
     'geoidalSep_m'});

lbl = read_xpf_table(lblFile, ...
    {'beaconId','latitude_deg','longitude_deg','depth_m','range_m','rangeSd_m'});

% The IMU is large, so only its compact statistics are retained.
imuStats = read_imu_statistics(imuFile);

%% Basic time and trajectory quantities
earthRadiusM = 6378137;
lat0Deg = nav.latitude_deg(1);
lon0Deg = nav.longitude_deg(1);
[navEastM, navNorthM] = ll_to_en(nav.latitude_deg, nav.longitude_deg, ...
    lat0Deg, lon0Deg, earthRadiusM);
[gpsEastM, gpsNorthM] = ll_to_en(gps.latitude_deg, gps.longitude_deg, ...
    lat0Deg, lon0Deg, earthRadiusM);
% GPS exports may contain no-fix/zero coordinates. Keep only positions that
% are finite, geographically valid, and close to the INS working area.
gpsValidMask = isfinite(gps.latitude_deg) & isfinite(gps.longitude_deg) & ...
    abs(gps.latitude_deg) <= 90 & abs(gps.longitude_deg) <= 180 & ...
    abs(gps.latitude_deg-lat0Deg) < 1 & abs(gps.longitude_deg-lon0Deg) < 1;
lblValidMask = isfinite(lbl.beaconId) & isfinite(lbl.latitude_deg) & ...
    isfinite(lbl.longitude_deg) & abs(lbl.latitude_deg) <= 90 & ...
    abs(lbl.longitude_deg) <= 180 & abs(lbl.latitude_deg-lat0Deg) < 1 & ...
    abs(lbl.longitude_deg-lon0Deg) < 1;

navHorizontalSpeed = hypot(nav.speedNorth_mps, nav.speedEast_mps);
dvlHorizontalSpeed = hypot(dvl.speedX_mps, dvl.speedY_mps);
navStepM = hypot(diff(navEastM), diff(navNorthM));
navDtS = seconds(diff(nav.utc_time));
navJumpThresholdM = 20;
navJumpMask = navStepM > navJumpThresholdM;

gpsTimeS = posixtime(gps.utc_time);
navTimeS = posixtime(nav.utc_time);
dvlTimeS = posixtime(dvl.utc_time);
depthTimeS = posixtime(depth.utc_time);

dvlAtNav = interp_valid(dvlTimeS, dvlHorizontalSpeed, navTimeS);
depthAtNav = interp_valid(depthTimeS, depth.depth_m, navTimeS);
gpsAtNav = interp_valid(gpsTimeS(gpsValidMask), gps.altitude_m(gpsValidMask), navTimeS);

dvlSpeedMask = isfinite(dvlAtNav) & isfinite(navHorizontalSpeed);
if nnz(dvlSpeedMask) >= 2
    dvlSpeedCorrelation = corr(navHorizontalSpeed(dvlSpeedMask), ...
        dvlAtNav(dvlSpeedMask));
    dvlSpeedRmsDifference = sqrt(mean((navHorizontalSpeed(dvlSpeedMask) - ...
        dvlAtNav(dvlSpeedMask)).^2));
else
    dvlSpeedCorrelation = NaN;
    dvlSpeedRmsDifference = NaN;
end

depthConsistency = -nav.altitude_m - depthAtNav;
depthConsistencyMask = isfinite(depthConsistency);
gpsAltitudeDifference = nav.altitude_m - gpsAtNav;
gpsAltitudeMask = isfinite(gpsAltitudeDifference);

summary = table( ...
    ["navigation rows"; "navigation duration"; "navigation median rate"; ...
     "IMU rows"; "IMU median rate"; "DVL rows"; "depth rows"; "GPS rows"; ...
     "LBL rows"; "horizontal track length"; "start-to-end displacement"; ...
     "median horizontal speed"; "maximum horizontal speed"; ...
     "navigation jumps above threshold"; "maximum navigation step"; ...
     "DVL/INS speed correlation"; "DVL/INS speed RMS difference"; ...
     "median INS-depth difference"; "RMS INS-depth difference"; ...
     "median INS-GPS altitude difference"; "USBL files detected"], ...
    [height(nav); seconds(nav.utc_time(end)-nav.utc_time(1)); ...
     1/median(seconds(diff(nav.utc_time)), 'omitnan'); imuStats.row_count; ...
     imuStats.median_rate_hz; height(dvl); height(depth); height(gps); height(lbl); ...
     sum(navStepM(~navJumpMask), 'omitnan'); ...
     hypot(navEastM(end)-navEastM(1), navNorthM(end)-navNorthM(1)); ...
     median(navHorizontalSpeed, 'omitnan'); max(navHorizontalSpeed, [], 'omitnan'); ...
     nnz(navJumpMask); max(navStepM, [], 'omitnan'); dvlSpeedCorrelation; ...
     dvlSpeedRmsDifference; median(depthConsistency(depthConsistencyMask), 'omitnan'); ...
     sqrt(mean(depthConsistency(depthConsistencyMask).^2, 'omitnan')); ...
     median(gpsAltitudeDifference(gpsAltitudeMask), 'omitnan'); ...
     double(~isempty(dir(fullfile(dataDir, '*usbl*'))))], ...
    ["row"; "s"; "Hz"; "row"; "Hz"; "row"; "row"; "row"; "row"; ...
     "m"; "m"; "m/s"; "m/s"; "count"; "m"; "1"; "m/s"; "m"; "m"; "m"; "count"], ...
    'VariableNames', {'metric','value','unit'});

%% Algorithm status summary
statusMatrix = [nav.algoStatus1, nav.algoStatus2, nav.algoStatus3];
[uniqueStatus, ~, statusGroup] = unique(statusMatrix, 'rows', 'stable');
statusCount = accumarray(statusGroup, 1);
statusSummary = table(uniqueStatus(:,1), uniqueStatus(:,2), uniqueStatus(:,3), ...
    statusCount, 100*statusCount/height(nav), ...
    repmat("Numeric PHINS status code; semantic mapping requires PHINS documentation", ...
        size(uniqueStatus,1), 1), ...
    'VariableNames', {'algoStatus1','algoStatus2','algoStatus3', ...
    'row_count','percentage','interpretation'});

%% Data availability and role notes
usblFiles = dir(fullfile(dataDir, '*usbl*'));
dataAvailability = table( ...
    ["IMU"; "INSNAV"; "DVL bottom track"; "Depth sensor"; "GPS"; "LBL"; "USBL"], ...
    [true; true; true; true; true; true; ~isempty(usblFiles)], ...
    ["Primary inertial increments"; "PHINS navigation output and algorithm states"; ...
     "External bottom-track velocity aiding"; "Vertical/depth aiding"; ...
     "Available reference/position stream"; "Beacon positions/ranges, not USBL"; ...
     "No USBL file was found in this sample folder"], ...
    'VariableNames', {'data_item','available','role_or_note'});

%% Write text summaries
write_txt(summary, fullfile(outputDir, 'sample_summary.txt'));
write_txt(statusSummary, fullfile(outputDir, 'algorithm_status_summary.txt'));
write_txt(dataAvailability, fullfile(outputDir, 'data_availability_and_notes.txt'));
write_txt(imuStats.table, fullfile(outputDir, 'imu_statistics.txt'));

jumpIndex = find(navJumpMask) + 1;
jumpTable = table(nav.utc_time(jumpIndex), navStepM(navJumpMask), ...
    navDtS(navJumpMask), nav.algoStatus1(jumpIndex), ...
    nav.algoStatus2(jumpIndex), nav.algoStatus3(jumpIndex), ...
    'VariableNames', {'utc_time','position_step_m','time_step_s', ...
    'algoStatus1','algoStatus2','algoStatus3'});
write_txt(jumpTable, fullfile(outputDir, 'navigation_jumps_over_20m.txt'));

%% Figure 1: trajectory and navigation states
fig1 = figure('Visible','off','Color','w','Position',[100 100 1400 900]);
tiledlayout(fig1, 2, 2, 'Padding','compact','TileSpacing','compact');

ax = nexttile; hold(ax,'on');
plot_decimated(ax, navEastM, navNorthM, 40000, [0.05 0.25 0.70], '-', 'INSNAV');
plot_decimated(ax, gpsEastM(gpsValidMask), gpsNorthM(gpsValidMask), ...
    20000, [0.85 0.25 0.10], '--', 'GPS valid positions');
if any(lblValidMask)
    [beaconId,~,beaconGroup] = unique(lbl.beaconId(lblValidMask)); %#ok<ASGLU>
    beaconLat = accumarray(beaconGroup, lbl.latitude_deg(lblValidMask), [], @median);
    beaconLon = accumarray(beaconGroup, lbl.longitude_deg(lblValidMask), [], @median);
    [beaconE, beaconN] = ll_to_en(beaconLat, beaconLon, lat0Deg, lon0Deg, earthRadiusM);
    scatter(ax, beaconE, beaconN, 55, 'kd', 'filled', 'DisplayName','LBL beacons');
end
plot(ax, navEastM(1), navNorthM(1), 'go', 'MarkerFaceColor','g', 'DisplayName','Start');
plot(ax, navEastM(end), navNorthM(end), 'rs', 'MarkerFaceColor','r', 'DisplayName','End');
if ~isempty(jumpIndex)
    scatter(ax, navEastM(jumpIndex), navNorthM(jumpIndex), 25, 'rx', ...
        'DisplayName','Step > 20 m');
end
axis(ax,'equal'); grid(ax,'on'); xlabel(ax,'Local east (m)'); ylabel(ax,'Local north (m)');
title(ax,'INSNAV/GPS trajectory'); legend(ax,'Location','best');

ax = nexttile; hold(ax,'on');
plot(ax, nav.utc_time, nav.heading_deg, 'DisplayName','Heading');
plot(ax, nav.utc_time, nav.roll_deg, 'DisplayName','Roll');
plot(ax, nav.utc_time, nav.pitch_deg, 'DisplayName','Pitch');
grid(ax,'on'); ylabel(ax,'Angle (deg)'); title(ax,'Attitude states');
legend(ax,'Location','best'); xtickformat(ax,'HH:mm');

ax = nexttile; hold(ax,'on');
plot(ax, nav.utc_time, navHorizontalSpeed, 'DisplayName','INS horizontal');
plot(ax, dvl.utc_time, dvlHorizontalSpeed, '.', 'DisplayName','DVL horizontal');
grid(ax,'on'); ylabel(ax,'Speed (m/s)'); xlabel(ax,'UTC');
title(ax,'INS/DVL speed'); legend(ax,'Location','best'); xtickformat(ax,'HH:mm');

ax = nexttile; hold(ax,'on');
plot(ax, nav.utc_time, -nav.altitude_m, 'DisplayName','-INS altitude');
plot(ax, depth.utc_time, depth.depth_m, 'DisplayName','Depth sensor');
set(ax,'YDir','reverse'); grid(ax,'on'); ylabel(ax,'Depth (m)'); xlabel(ax,'UTC');
title(ax,'Depth consistency'); legend(ax,'Location','best'); xtickformat(ax,'HH:mm');

exportgraphics(fig1, fullfile(outputDir,'trajectory_and_states.png'), 'Resolution', 180);
exportgraphics(fig1, fullfile(outputDir,'trajectory_and_states.pdf'), 'ContentType','vector');
close(fig1);

%% Figure 2: data quality and algorithm status
fig2 = figure('Visible','off','Color','w','Position',[100 100 1400 900]);
tiledlayout(fig2, 2, 2, 'Padding','compact','TileSpacing','compact');

ax = nexttile;
semilogy(ax, nav.utc_time(2:end), max(navStepM,0.01), 'Color',[0.15 0.35 0.70]);
hold(ax,'on'); yline(ax,20,'--','20 m threshold','HandleVisibility','off');
grid(ax,'on'); ylabel(ax,'Position step (m)'); title(ax,'Navigation discontinuities');
xtickformat(ax,'HH:mm');

ax = nexttile;
plot(ax, dvl.utc_time, dvl.altitude_m, 'Color',[0.10 0.55 0.30]);
grid(ax,'on'); ylabel(ax,'DVL altitude (m)'); title(ax,'DVL bottom-track altitude');
xtickformat(ax,'HH:mm');

ax = nexttile; hold(ax,'on');
plot(ax, nav.utc_time, nav.algoStatus1, 'DisplayName','Status 1');
plot(ax, nav.utc_time, nav.algoStatus2, 'DisplayName','Status 2');
plot(ax, nav.utc_time, nav.algoStatus3, 'DisplayName','Status 3');
grid(ax,'on'); ylabel(ax,'Numeric code'); xlabel(ax,'UTC');
title(ax,'PHINS algorithm status codes'); legend(ax,'Location','best'); xtickformat(ax,'HH:mm');

ax = nexttile; hold(ax,'on');
plot(ax, nav.utc_time, gpsAltitudeDifference, 'DisplayName','INS altitude - GPS altitude');
plot(ax, nav.utc_time, depthConsistency, 'DisplayName','-INS altitude - depth');
grid(ax,'on'); ylabel(ax,'Difference (m)'); xlabel(ax,'UTC');
title(ax,'Reference consistency checks'); legend(ax,'Location','best'); xtickformat(ax,'HH:mm');

exportgraphics(fig2, fullfile(outputDir,'quality_and_status.png'), 'Resolution', 180);
exportgraphics(fig2, fullfile(outputDir,'quality_and_status.pdf'), 'ContentType','vector');
close(fig2);

save(fullfile(outputDir,'sample_analysis.mat'), 'nav','dvl','depth','gps','lbl', ...
    'gpsValidMask', ...
    'summary','statusSummary','dataAvailability','jumpTable','imuStats', '-v7.3');

fprintf('Completed.\n');
fprintf('  NAV rows: %d; IMU rows: %d; DVL rows: %d\n', ...
    height(nav), imuStats.row_count, height(dvl));
fprintf('  Duration: %.2f min; track length: %.1f m\n', ...
    summary.value(2)/60, summary.value(10));
fprintf('  Status combinations: %d; NAV jumps > 20 m: %d\n', ...
    height(statusSummary), height(jumpTable));
fprintf('  USBL files detected: %d\n', ~isempty(usblFiles));
fprintf('  GPS valid positions plotted: %d / %d\n', nnz(gpsValidMask), height(gps));
fprintf('  Results: %s\n', outputDir);

%% Local functions
function tbl = read_xpf_table(file, numericNames)
fid = fopen(file, 'rt');
if fid < 0
    error('PHINS:OpenFailed','Cannot open %s',file);
end
cleanupFile = onCleanup(@() fclose(fid)); %#ok<NASGU>
format = ['%s%s', repmat('%f',1,numel(numericNames))];
values = textscan(fid, format, 'Delimiter','\t', 'CommentStyle','#', ...
    'MultipleDelimsAsOne',false, 'CollectOutput',false, ...
    'ReturnOnError',false);
dateText = string(values{1});
timeText = string(values{2});
utcTime = datetime(dateText + " " + timeText, ...
    'InputFormat','yyyy/MM/dd HH:mm:ss.SSSS', 'TimeZone','UTC');
utcTime.Format = 'yyyy-MM-dd HH:mm:ss.SSSS';
numericData = [values{3:end}];
tbl = array2table(numericData, 'VariableNames',numericNames);
tbl.utc_time = utcTime;
tbl = movevars(tbl,'utc_time','Before',1);
end

function stats = read_imu_statistics(file)
fid = fopen(file,'rt');
if fid < 0
    error('PHINS:OpenFailed','Cannot open %s',file);
end
cleanupFile = onCleanup(@() fclose(fid)); %#ok<NASGU>
values = textscan(fid, ['%s%s',repmat('%f',1,8)], ...
    'Delimiter','\t', 'CommentStyle','#', 'MultipleDelimsAsOne',false, ...
    'CollectOutput',false, 'ReturnOnError',false);
dateText = string(values{1});
timeText = string(values{2});
time = datetime(dateText + " " + timeText, ...
    'InputFormat','yyyy/MM/dd HH:mm:ss.SSSS', 'TimeZone','UTC');
frameCounter = values{3};
sensorStatus = values{10};
rateHz = 1/median(seconds(diff(time)), 'omitnan');
stats.row_count = numel(time);
stats.median_rate_hz = rateHz;
stats.sensor_status_nonzero_count = nnz(sensorStatus ~= 0);
stats.frame_counter_resets = nnz(diff(frameCounter) < 0);
stats.table = table( ...
    ["row_count"; "median_rate_hz"; "sensor_status_nonzero_count"; "frame_counter_resets"], ...
    [stats.row_count; stats.median_rate_hz; stats.sensor_status_nonzero_count; stats.frame_counter_resets], ...
    ["row"; "Hz"; "count"; "count"], ...
    'VariableNames', {'metric','value','unit'});
end

function [eastM,northM] = ll_to_en(latDeg,lonDeg,lat0Deg,lon0Deg,R)
northM = deg2rad(latDeg-lat0Deg)*R;
eastM = deg2rad(lonDeg-lon0Deg)*R*cosd(lat0Deg);
end

function out = interp_valid(x,y,xq)
mask = isfinite(x) & isfinite(y);
if nnz(mask) < 2
    out = nan(size(xq));
    return;
end
[xUnique,idx] = unique(x(mask));
yUnique = y(mask); yUnique = yUnique(idx);
out = interp1(xUnique,yUnique,xq,'linear',NaN);
end

function write_txt(tbl,file)
writetable(tbl,file,'FileType','text','Delimiter','\t','Encoding','UTF-8');
end

function plot_decimated(ax,x,y,maxPoints,color,lineStyle,label)
idx = 1:numel(x);
if numel(idx) > maxPoints
    idx = round(linspace(1,numel(x),maxPoints));
end
plot(ax,x(idx),y(idx),'Color',color,'LineStyle',lineStyle, ...
    'DisplayName',label,'LineWidth',1.0);
end
