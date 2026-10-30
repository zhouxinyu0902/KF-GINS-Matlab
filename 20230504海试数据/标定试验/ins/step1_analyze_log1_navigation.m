%% STEP1_ANALYZE_LOG1_NAVIGATION  分析log1中的PHINS导航结果
% 本文件为可直接执行的脚本，不是函数。
%
% 导航结果来源：
%   log1-*.dat
%       -> PHINS/PostproDecode厂家解码
%       -> IMU（50 Hz）+ DVL底跟踪速度 + 深度辅助
%       -> PHINS专有导航/Kalman结果
%       -> log1-*-ins-INSNAV.xpf.txt
%
% 本脚本分析厂家输出的导航结果，不尝试复现PHINS内部滤波器。
% log1-*-ins.conf.xml明确记录GPS、LBL和USBL均未接收。
%
% 所有图形均按照工程约定使用myfigurestartup(...,'paper')创建；
% 所有表格均以UTF-8编码、制表符分隔的TXT文件保存。

clear;
clc;
close all;

%% 固定工程路径
scriptsDir = "D:\Github\KF-GINS-Matlab\20230504海试数据\标定试验";
analysisDir = "D:\Github\KF-GINS-Matlab\data\20230504\标定实验分析";
dataDir = fullfile(analysisDir, "信标标定试验数据");
parsedDir = fullfile(analysisDir, "解析后的数据", "ins_batch");
outputDir = fullfile(analysisDir, "解析后的数据", "step1_nav_analysis");
figureHelperDir = "D:\Github\KF-GINS-Matlab\psins2401\mytest\00_all_func";

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
cleanupFigurePath = onCleanup(@() rmpath(figureHelperDir)); 

fprintf('Step 1: analysing PHINS log1 navigation\n');
fprintf('  Navigation source: %s\n', navFile);
fprintf('  Output directory : %s\n', outputDir);

%% 读取厂家导出的高精度数据
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

%% 计算导航统计量与辅助数据一致性
earthRadiusM = 6378137;
lat0Deg = nav.latitude_deg(1);
lon0Deg = nav.longitude_deg(1);
northM = deg2rad(nav.latitude_deg-lat0Deg)*earthRadiusM;
eastM = deg2rad(nav.longitude_deg-lon0Deg)*earthRadiusM*cosd(lat0Deg);
elapsedMin = minutes(nav.utc_time-nav.utc_time(1));
horizontalSpeed = hypot(nav.velocity_north_mps, nav.velocity_east_mps);
navDtS = seconds(diff(nav.utc_time));
navStepM = hypot(diff(eastM), diff(northM));
jumpThresholdM = 20;
jumpMask = navStepM > jumpThresholdM;
rawTrackDistanceM = sum(navStepM, 'omitnan');
trackDistanceM = sum(navStepM(~jumpMask), 'omitnan');
displacementM = hypot(eastM(end)-eastM(1), northM(end)-northM(1));
durationS = seconds(nav.utc_time(end)-nav.utc_time(1));
navRateHz = 1/median(navDtS, 'omitnan');
imuRateHz = 1/median(seconds(diff(imu.utc_time)), 'omitnan');

navSeconds = posixtime(nav.utc_time);
depthSeconds = posixtime(depth.utc_time);
dvlSeconds = posixtime(dvl.utc_time);
dvlDtS = seconds(diff(dvl.utc_time));
dvlGapThresholdS = 2;
dvlGapMask = dvlDtS > dvlGapThresholdS;

depthAtNav = interp_valid(depthSeconds, depth.depth_m, navSeconds);
depthResidualM = -nav.altitude_m-depthAtNav;
depthMask = isfinite(depthResidualM);

depthAtDvl = interp_valid(depthSeconds, depth.depth_m, dvlSeconds);
inferredBottomDepthM = depthAtDvl+dvl.altitude_m;
nearestDvlSeconds = interp1(dvlSeconds, dvlSeconds, navSeconds, ...
    'nearest', NaN);
dvlAgeAtNavS = abs(navSeconds-nearestDvlSeconds);
dvlNearbyMask = dvlAgeAtNavS <= dvlGapThresholdS;
dvlCoveragePct = 100*mean(dvlNearbyMask);

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
    "depth rows"; "segmented horizontal track length"; ...
    "raw horizontal polyline length"; ...
    "start-to-end displacement"; "median horizontal speed"; ...
    "maximum horizontal speed"; "median INS-depth residual"; ...
    "RMS INS-depth residual"; "INS/DVL horizontal-speed correlation"; ...
    "INS/DVL horizontal-speed RMS difference"; ...
    "navigation jumps above 20 m"; "maximum navigation step"; ...
    "maximum navigation time gap"; "DVL nearby navigation coverage"; ...
    "DVL gaps above 2 s"; "maximum DVL gap"; ...
    "median DVL altitude"; "median inferred seabed depth"];
value = [height(nav); durationS; navRateHz; imuRateHz; height(dvl); ...
    height(depth); trackDistanceM; rawTrackDistanceM; displacementM; ...
    median(horizontalSpeed,'omitnan'); max(horizontalSpeed,[],'omitnan'); ...
    median(depthResidualM(depthMask),'omitnan'); ...
    sqrt(mean(depthResidualM(depthMask).^2,'omitnan')); ...
    speedCorrelation; speedDifferenceRmsMps; nnz(jumpMask); ...
    max(navStepM,[],'omitnan'); max(navDtS,[],'omitnan'); ...
    dvlCoveragePct; nnz(dvlGapMask); max(dvlDtS,[],'omitnan'); ...
    median(dvl.altitude_m,'omitnan'); ...
    median(inferredBottomDepthM,'omitnan')];
unit = ["row"; "s"; "Hz"; "Hz"; "row"; "row"; "m"; "m"; "m"; ...
    "m/s"; "m/s"; "m"; "m"; "1"; "m/s"; "count"; "m"; "s"; ...
    "%"; "count"; "s"; "m"; "m"];
navSummary = table(metric, value, unit);

%% 汇总导航跳变与DVL数据间断
jumpIndex = find(jumpMask)+1;
jumpSummary = table(nav.utc_time(jumpIndex), navStepM(jumpMask), ...
    navDtS(jumpMask), dvlAgeAtNavS(jumpIndex), ...
    dvlNearbyMask(jumpIndex), nav.algo_status_1(jumpIndex), ...
    nav.algo_status_2(jumpIndex), nav.algo_status_3(jumpIndex), ...
    'VariableNames', {'utc_time','position_step_m','time_step_s', ...
    'nearest_dvl_age_s','dvl_nearby','algo_status_1','algo_status_2', ...
    'algo_status_3'});

dvlGapIndex = find(dvlGapMask);
dvlGapSummary = table(dvl.utc_time(dvlGapIndex), ...
    dvl.utc_time(dvlGapIndex+1), dvlDtS(dvlGapMask), ...
    'VariableNames', {'gap_start_utc','gap_end_utc','gap_duration_s'});

%% 汇总导航算法状态出现次数
statusMatrix = [nav.algo_status_1, nav.algo_status_2, nav.algo_status_3];
[uniqueStatus, ~, statusGroup] = unique(statusMatrix, 'rows', 'stable');
rowCount = accumarray(statusGroup, 1);
percentage = 100*rowCount/height(nav);
statusSummary = table(uniqueStatus(:,1), uniqueStatus(:,2), ...
    uniqueStatus(:,3), rowCount, percentage, ...
    'VariableNames', {'algo_status_1','algo_status_2','algo_status_3', ...
    'row_count','percentage'});

%% 记录导航结果的数据来源
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

%% 比较高精度INSNAV与低精度PhinsNav解码表
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
    decodedNorthM = deg2rad(decoded.latitude_deg(1:nCompare)- ...
        nav.latitude_deg(1:nCompare))*earthRadiusM;
    decodedEastM = deg2rad(decoded.longitude_deg(1:nCompare)- ...
        nav.longitude_deg(1:nCompare))*earthRadiusM*cosd(lat0Deg);
    decodedHorizontalDifferenceM = hypot(decodedEastM,decodedNorthM);
    comparisonRows{end+1,1} = table("horizontal_position", nCompare, ...
        mean(decodedHorizontalDifferenceM,'omitnan'), ...
        max(decodedHorizontalDifferenceM,[],'omitnan'), "m", ...
        'VariableNames', {'variable','compared_rows', ...
        'mean_absolute_difference','maximum_absolute_difference','unit'});
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

%% 导出TXT结果
write_txt(navSummary, fullfile(outputDir, "step1_nav_summary.txt"));
write_txt(statusSummary, fullfile(outputDir, "step1_algorithm_status_summary.txt"));
write_txt(sourceTrace, fullfile(outputDir, "step1_navigation_source_trace.txt"));
write_txt(sourceComparison, ...
    fullfile(outputDir, "step1_navigation_source_comparison.txt"));
write_txt(jumpSummary, fullfile(outputDir, "step1_navigation_jumps.txt"));
write_txt(dvlGapSummary, fullfile(outputDir, "step1_dvl_gaps.txt"));

%% 图1：局部导航轨迹（单图使用3×3 paper画布）
figTrack = myfigurestartup(3, 3, 'paper');
set(figTrack, 'Visible','off');
axTrack = gca(figTrack);
cla(axTrack);
hold(axTrack, 'on');
scatter(axTrack, eastM, northM, 8, elapsedMin, 'filled', ...
    'HandleVisibility','off');
scatter(axTrack, eastM(jumpIndex), northM(jumpIndex), 24, 'rx', ...
    'LineWidth',1.0, 'DisplayName','Jump > 20 m');
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
legend(axTrack, 'Location','northwest');
export_figure(figTrack, outputDir, "step1_nav_track");
close(figTrack);

%% 图2：导航状态与辅助数据一致性（两行两列使用7×7 paper画布）
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

%% 图3：导航跳变与DVL底跟踪覆盖（两行两列使用7×7 paper画布）
figQuality = myfigurestartup(7, 7, 'paper');
set(figQuality, 'Visible','off');
delete(findall(figQuality, 'Type','axes'));
layoutQuality = tiledlayout(figQuality, 2, 2, 'Padding','compact', ...
    'TileSpacing','compact');

ax5 = nexttile(layoutQuality, 1);
semilogy(ax5, nav.utc_time(2:end), max(navStepM,0.01), ...
    'Color',[0.15,0.35,0.70]);
hold(ax5,'on');
yline(ax5,jumpThresholdM,'--','20 m threshold', ...
    'HandleVisibility','off');
xline(ax5,min(dvl.utc_time),':','DVL starts', ...
    'HandleVisibility','off');
grid(ax5,'on'); xlabel(ax5,'UTC'); ylabel(ax5,'Position step (m)');
title(ax5,'(a) Navigation position discontinuities');
xtickformat(ax5,'HH:mm');

ax6 = nexttile(layoutQuality, 2);
plot(ax6,dvl.utc_time,dvl.altitude_m,'Color',[0.15,0.55,0.35]);
grid(ax6,'on'); xlabel(ax6,'UTC'); ylabel(ax6,'DVL altitude (m)');
title(ax6,'(b) Valid DVL bottom-track altitude');
xtickformat(ax6,'HH:mm');

ax7 = nexttile(layoutQuality, 3);
hold(ax7,'on');
plot(ax7,dvl.utc_time,depthAtDvl,'DisplayName','Vehicle depth');
plot(ax7,dvl.utc_time,inferredBottomDepthM, ...
    'DisplayName','Depth + DVL altitude');
set(ax7,'YDir','reverse');
grid(ax7,'on'); xlabel(ax7,'UTC'); ylabel(ax7,'Depth (m)');
title(ax7,'(c) Inferred seabed depth');
legend(ax7,'Location','best'); xtickformat(ax7,'HH:mm');

ax8 = nexttile(layoutQuality, 4);
scatter(ax8,dvl.utc_time(2:end),max(dvlDtS,0.01),7, ...
    [0.55,0.25,0.65],'filled');
set(ax8,'YScale','log');
hold(ax8,'on');
yline(ax8,dvlGapThresholdS,'--','2 s gap threshold', ...
    'HandleVisibility','off');
grid(ax8,'on'); xlabel(ax8,'UTC'); ylabel(ax8,'DVL interval (s)');
title(ax8,'(d) DVL sampling gaps'); xtickformat(ax8,'HH:mm');

title(layoutQuality,'Navigation discontinuities and DVL availability');
export_figure(figQuality, outputDir, "step1_nav_dvl_quality");
close(figQuality);

save(fullfile(outputDir, "step1_log1_navigation_analysis.mat"), ...
    'nav','imu','dvl','depth','navSummary','statusSummary','sourceTrace', ...
    'sourceComparison','jumpSummary','dvlGapSummary','eastM','northM', ...
    'horizontalSpeed','navStepM','depthResidualM','navSpeedAtDvl', ...
    'dvlHorizontalSpeed','inferredBottomDepthM','dvlAgeAtNavS','-v7.3');

fprintf('\nStep 1 completed.\n');
fprintf('  Navigation rows       : %d\n', height(nav));
fprintf('  Duration              : %.2f min\n', durationS/60);
fprintf('  Track length          : %.1f m\n', trackDistanceM);
fprintf('  IMU / nav median rate : %.2f / %.2f Hz\n', imuRateHz, navRateHz);
fprintf('  DVL / depth rows      : %d / %d\n', height(dvl), height(depth));
fprintf('  Results               : %s\n', outputDir);

%% 本脚本使用的局部辅助函数
function tbl = read_xpf_export(file, numericNames)
fid = fopen(file, 'rt');
assert(fid >= 0, 'STEP1:OpenFailed', 'Cannot open %s.', file);
cleanupFile = onCleanup(@() fclose(fid)); 
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
