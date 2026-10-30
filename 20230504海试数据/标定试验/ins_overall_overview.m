%% STEP0_INS_OVERALL_OVERVIEW  INS、LBL和USBL数据总体概览
% 本文件为可直接执行的脚本，不是函数。
%
% 数据关系：
%   1）以log1中的高精度INSNAV作为主要惯导导航结果；
%   2）log2包含发送给RAMSES的导航数据、RAMSES/LBL ID 0输出、
%      4个固定LBL信标（90--93）以及稀疏的TTR观测；
%   3）所有USBL日志仅统计时间覆盖范围和记录数量，
%      本脚本不评估USBL/LBL定位精度。
%
% LBL报文日期为占位日期（1970年）。TTR观测按照文件顺序与最近的
% log2 TIM_IN记录关联。USBL时间保留原始记录，UTC+8仅用于时间覆盖图显示。
%
% 所有图形均使用myfigurestartup(...,'paper')创建；表格以UTF-8编码、
% 制表符分隔的TXT文件保存。

clear;
clc;
close all;

%% 固定工程路径
scriptsDir = "D:\Github\KF-GINS-Matlab\20230504海试数据\标定试验";
analysisDir = "D:\Github\KF-GINS-Matlab\data\20230504\标定实验分析";
dataDir = fullfile(analysisDir, "信标标定试验数据");
outputDir = fullfile(analysisDir, "解析后的数据", "ins_USBL_overview");
figureHelperDir = "D:\Github\KF-GINS-Matlab\psins2401\mytest\00_all_func";

log1NavFile = fullfile(dataDir, "log1-20230430-0347-ins-INSNAV.xpf.txt");
log1DvlFile = fullfile(dataDir, "log1-20230430-0347-ins-DVL.xpf.txt");
log2File = fullfile(dataDir, "log2-20230430-0347.dat");
log2Parser = fullfile(scriptsDir, "parse_phins_log2.m");
usblParser = fullfile(scriptsDir, "parse_usbl.m");
figureHelper = fullfile(figureHelperDir, "myfigurestartup.m");

requiredFiles = [log1NavFile, log1DvlFile, log2File, log2Parser, ...
    usblParser, figureHelper];
for k = 1:numel(requiredFiles)
    mustBeFile(requiredFiles(k));
end
if ~isfolder(outputDir), mkdir(outputDir); end

addpath(scriptsDir, figureHelperDir);
cleanupPaths = onCleanup(@() remove_paths(scriptsDir, figureHelperDir)); 
scratchDir = string(tempname);
mkdir(scratchDir);
cleanupScratch = onCleanup(@() cleanup_scratch(scratchDir)); 

fprintf('INS/LBL/USBL overall overview\n');
fprintf('  Scripts : %s\n', scriptsDir);
fprintf('  Data    : %s\n', dataDir);
fprintf('  Output  : %s\n', outputDir);

%% 解析并读取三组数据
navNames = {'algo_status_1','algo_status_2','algo_status_3', ...
    'latitude_deg','longitude_deg','altitude_m','heading_deg','roll_deg', ...
    'pitch_deg','velocity_north_mps','velocity_east_mps', ...
    'velocity_up_mps','heave_m','surge_m','sway_m'};
dvlNames = {'velocity_x_mps','velocity_y_mps','velocity_z_mps', ...
    'sensor_sound_speed_mps','external_sound_speed_mps','altitude_m', ...
    'velocity_x_std_mps','velocity_y_std_mps','velocity_z_std_mps'};

log1Nav = read_xpf_export(log1NavFile, navNames);
log1Dvl = read_xpf_export(log1DvlFile, dvlNames);
log2 = parse_phins_log2(log2File, scratchDir);
usbl = parse_usbl(fullfile(dataDir, "USBL-BOX-R-*.log"), scratchDir);
ttr = parse_log2_ttr(log2File, datetime(2023,4,30,'TimeZone','UTC'));

assert(~isempty(log1Nav), 'OVERVIEW:NoLog1Nav', 'log1 INSNAV is empty.');
assert(~isempty(log2.navigation), 'OVERVIEW:NoLog2Nav', ...
    'log2 navigation is empty.');
assert(~isempty(log2.ptsag), 'OVERVIEW:NoLbl', 'log2 PTSAG is empty.');
assert(~isempty(usbl.ptsag), 'OVERVIEW:NoUsbl', 'USBL PTSAG is empty.');

%% 将LBL输出与对应的log2导航历元关联
nav2 = log2.navigation;
lblPtsag = log2.ptsag;
lbl0 = lblPtsag(lblPtsag.transponder_id == 0,:);
nPair = min(height(nav2), height(lbl0));
nav2Pair = nav2(1:nPair,:);
lbl0 = lbl0(1:nPair,:);
lbl0.nav_time = nav2Pair.utc_time;
lbl0 = movevars(lbl0, 'nav_time', 'After','sequence');

earthRadiusM = 6378137;
referenceLatDeg = median(lblPtsag.latitude_deg( ...
    ismember(lblPtsag.transponder_id,90:93)), 'omitnan');
referenceLonDeg = median(lblPtsag.longitude_deg( ...
    ismember(lblPtsag.transponder_id,90:93)), 'omitnan');

[nav1E,nav1N] = ll_to_en(log1Nav.latitude_deg,log1Nav.longitude_deg, ...
    referenceLatDeg,referenceLonDeg,earthRadiusM);
[nav2E,nav2N] = ll_to_en(nav2Pair.latitude_deg,nav2Pair.longitude_deg, ...
    referenceLatDeg,referenceLonDeg,earthRadiusM);
[lbl0E,lbl0N] = ll_to_en(lbl0.latitude_deg,lbl0.longitude_deg, ...
    referenceLatDeg,referenceLonDeg,earthRadiusM);

navLblEastM = lbl0E-nav2E;
navLblNorthM = lbl0N-nav2N;
navLblHorizontalM = hypot(navLblEastM,navLblNorthM);

%% 分析导航轨迹不连续性以及log1/log2一致性
nav2DtS = seconds(diff(nav2.utc_time));
[allNav2E,allNav2N] = ll_to_en(nav2.latitude_deg,nav2.longitude_deg, ...
    referenceLatDeg,referenceLonDeg,earthRadiusM);
nav2StepM = hypot(diff(allNav2E),diff(allNav2N));
jumpThresholdM = 20;
jumpMask = nav2StepM > jumpThresholdM;
jumpSummary = table(nav2.utc_time(find(jumpMask)+1), ...
    nav2StepM(jumpMask), nav2DtS(jumpMask), ...
    nav2.latitude_std_m(find(jumpMask)+1), ...
    nav2.longitude_std_m(find(jumpMask)+1), ...
    nav2.heading_deg(find(jumpMask)+1), ...
    'VariableNames', {'utc_time','position_step_m','time_step_s', ...
    'latitude_std_m','longitude_std_m','heading_deg'});

nNavCompare = min(height(log1Nav),height(nav2));
[log1CmpE,log1CmpN] = ll_to_en(log1Nav.latitude_deg(1:nNavCompare), ...
    log1Nav.longitude_deg(1:nNavCompare),referenceLatDeg,referenceLonDeg, ...
    earthRadiusM);
[log2CmpE,log2CmpN] = ll_to_en(nav2.latitude_deg(1:nNavCompare), ...
    nav2.longitude_deg(1:nNavCompare),referenceLatDeg,referenceLonDeg, ...
    earthRadiusM);
log1Log2PositionDifferenceM = hypot(log1CmpE-log2CmpE,log1CmpN-log2CmpN);

%% 汇总4个LBL信标及对应USBL数据量
beaconIds = (90:93)';
nBeacons = numel(beaconIds);
lblLatitudeDeg = nan(nBeacons,1);
lblLongitudeDeg = nan(nBeacons,1);
lblDepthM = nan(nBeacons,1);
ttrSignalId = nan(nBeacons,1);
ttrCount = zeros(nBeacons,1);
ttrStartNavTime = NaT(nBeacons,1,'TimeZone','UTC');
ttrEndNavTime = NaT(nBeacons,1,'TimeZone','UTC');
usblCount = zeros(nBeacons,1);

for k = 1:nBeacons
    id = beaconIds(k);
    qLbl = lblPtsag(lblPtsag.transponder_id == id,:);
    lblLatitudeDeg(k) = median(qLbl.latitude_deg,'omitnan');
    lblLongitudeDeg(k) = median(qLbl.longitude_deg,'omitnan');
    lblDepthM(k) = median(valid_depth(qLbl.computed_depth_m),'omitnan');

    qTtr = ttr(ttr.transponder_id == id,:);
    ttrCount(k) = height(qTtr);
    if ~isempty(qTtr)
        ttrSignalId(k) = mode(qTtr.signal_id);
        ttrStartNavTime(k) = min(qTtr.nav_time);
        ttrEndNavTime(k) = max(qTtr.nav_time);
    end

    qUsbl = usbl.ptsag(usbl.ptsag.transponder_id == id & ...
        isfinite(usbl.ptsag.latitude_deg) & ...
        isfinite(usbl.ptsag.longitude_deg),:);
    usblCount(k) = height(qUsbl);
end

lblBeaconSummary = table(beaconIds,lblLatitudeDeg,lblLongitudeDeg, ...
    lblDepthM,ttrSignalId,ttrCount,ttrStartNavTime,ttrEndNavTime, ...
    usblCount, ...
    'VariableNames', {'transponder_id','lbl_latitude_deg', ...
    'lbl_longitude_deg','lbl_depth_m','ttr_signal_id','ttr_count', ...
    'ttr_start_nav_time','ttr_end_nav_time','usbl_count'});

%% 按应答器编号汇总USBL数据
usblIds = unique(usbl.ptsag.transponder_id(isfinite(usbl.ptsag.transponder_id)));
nUsblIds = numel(usblIds);
usblRows = zeros(nUsblIds,1);
usblStart = NaT(nUsblIds,1,'TimeZone','UTC');
usblEnd = NaT(nUsblIds,1,'TimeZone','UTC');
usblChecksumOkPct = nan(nUsblIds,1);
for k = 1:nUsblIds
    q = usbl.ptsag(usbl.ptsag.transponder_id == usblIds(k),:);
    usblRows(k) = height(q);
    usblStart(k) = min(q.utc_time);
    usblEnd(k) = max(q.utc_time);
    usblChecksumOkPct(k) = 100*mean(q.checksum_ok);
end
usblIdSummary = table(usblIds,usblRows,usblStart,usblEnd, ...
    usblChecksumOkPct, ...
    'VariableNames', {'transponder_id','row_count','raw_utc_start', ...
    'raw_utc_end','checksum_ok_pct'});

%% 生成数据集和对比结果摘要
dataset = ["log1 high-precision INSNAV"; "log2 navigation input"; ...
    "log2 LBL ID 0 output"; "log2 LBL TTR"; "all USBL PTSAG"];
rowCount = [height(log1Nav);height(nav2);height(lbl0);height(ttr); ...
    height(usbl.ptsag)];
startTime = [min(log1Nav.utc_time);min(nav2.utc_time);min(lbl0.nav_time); ...
    min(ttr.nav_time);min(usbl.ptsag.utc_time)];
endTime = [max(log1Nav.utc_time);max(nav2.utc_time);max(lbl0.nav_time); ...
    max(ttr.nav_time);max(usbl.ptsag.utc_time)];
timeBasis = ["UTC from vendor INSNAV export"; ...
    "UTC from PIXSE TIME__"; ...
    "associated with log2 navigation by epoch order"; ...
    "LBL clock linked to nearest TIM_IN by file order"; ...
    "raw USBL UTC; add 8 h only for local-clock display"];
datasetSummary = table(dataset,rowCount,startTime,endTime,timeBasis, ...
    'VariableNames', {'dataset','row_count','start_time','end_time', ...
    'time_basis'});

metric = ["log1/log2 navigation median horizontal difference"; ...
    "log1/log2 navigation maximum horizontal difference"; ...
    "LBL ID0/NAV median horizontal difference"; ...
    "LBL ID0/NAV 95th percentile horizontal difference"; ...
    "LBL ID0/NAV maximum horizontal difference"; ...
    "NAV maximum one-second position step"; ...
    "NAV steps greater than 20 m"; "DVL first record time"];
value = [string(median(log1Log2PositionDifferenceM,'omitnan')); ...
    string(max(log1Log2PositionDifferenceM,[],'omitnan')); ...
    string(median(navLblHorizontalM,'omitnan')); ...
    string(prctile(navLblHorizontalM,95)); ...
    string(max(navLblHorizontalM,[],'omitnan')); ...
    string(max(nav2StepM,[],'omitnan')); string(nnz(jumpMask)); ...
    string(min(log1Dvl.utc_time))];
unit = ["m";"m";"m";"m";"m";"m";"count";"UTC"];
comparisonSummary = table(metric,value,unit);

%% 导出制表符分隔的TXT结果
write_txt(datasetSummary,fullfile(outputDir,"overview_dataset_summary.txt"));
write_txt(comparisonSummary,fullfile(outputDir,"overview_comparison_summary.txt"));
write_txt(lblBeaconSummary,fullfile(outputDir,"overview_lbl_beacons.txt"));
write_txt(ttr,fullfile(outputDir,"overview_lbl_ttr.txt"));
write_txt(usblIdSummary,fullfile(outputDir,"overview_usbl_id_summary.txt"));
write_txt(jumpSummary,fullfile(outputDir,"overview_navigation_jumps.txt"));

%% 图1：总体空间关系（两行两列，使用7×7 paper画布）
fig1 = myfigurestartup(7,7,'paper');
set(fig1,'Visible','off');
delete(findall(fig1,'Type','axes'));
layout1 = tiledlayout(fig1,2,2,'Padding','compact','TileSpacing','compact');

usblColors = lines(max(6,nUsblIds));
for k = 1:nUsblIds
    switch usblIds(k)
        case 0
            usblColors(k,:) = [0.60,0.60,0.60];
        case 17
            usblColors(k,:) = [0.50,0.20,0.65];
        case 90
            usblColors(k,:) = [0.10,0.55,0.25];
        case 91
            usblColors(k,:) = [0.85,0.35,0.10];
        case 92
            usblColors(k,:) = [0.93,0.68,0.12];
        case 93
            usblColors(k,:) = [0.25,0.60,0.80];
    end
end
beaconColors = lines(nBeacons);
ax1 = nexttile(layout1,1); hold(ax1,'on');
plot_segmented_track(ax1,nav1E,nav1N,log1Nav.utc_time,20, ...
    [0.05,0.25,0.65],'-','log1 INSNAV');
plot_segmented_track(ax1,lbl0E,lbl0N,lbl0.nav_time,20, ...
    [0.90,0.35,0.05],'--','log2 LBL ID 0');
for k = 1:nUsblIds
    id = usblIds(k);
    q = usbl.ptsag(usbl.ptsag.transponder_id == id & ...
        isfinite(usbl.ptsag.latitude_deg) & ...
        isfinite(usbl.ptsag.longitude_deg),:);
    idx = decimate_indices((1:height(q))',1800);
    [qE,qN] = ll_to_en(q.latitude_deg(idx),q.longitude_deg(idx), ...
        referenceLatDeg,referenceLonDeg,earthRadiusM);
    scatter(ax1,qE,qN,6,usblColors(k,:),'filled', ...
        'DisplayName',"USBL ID "+id);
end
[bE,bN] = ll_to_en(lblLatitudeDeg,lblLongitudeDeg, ...
    referenceLatDeg,referenceLonDeg,earthRadiusM);
scatter(ax1,bE,bN,55,'kd','filled','DisplayName','LBL beacons 90--93');
grid(ax1,'on'); axis(ax1,'equal');
xlabel(ax1,'Local east (m)'); ylabel(ax1,'Local north (m)');
title(ax1,'(a) All navigation and acoustic positions');
legend(ax1,'Location','best','NumColumns',2);

ax2 = nexttile(layout1,2); hold(ax2,'on');
plot_segmented_track(ax2,nav2E,nav2N,nav2Pair.utc_time,20, ...
    [0.05,0.25,0.65],'-','NAV input');
plot_segmented_track(ax2,lbl0E,lbl0N,lbl0.nav_time,20, ...
    [0.90,0.35,0.05],'--','LBL ID 0');
scatter(ax2,bE,bN,55,'kd','filled','DisplayName','LBL beacons');
plot(ax2,nav2E(1),nav2N(1),'go','MarkerFaceColor','g', ...
    'DisplayName','Start');
plot(ax2,nav2E(end),nav2N(end),'rs','MarkerFaceColor','r', ...
    'DisplayName','End');
grid(ax2,'on'); axis(ax2,'equal');
xlabel(ax2,'Local east (m)'); ylabel(ax2,'Local north (m)');
title(ax2,'(b) NAV input and LBL ID 0');
legend(ax2,'Location','best');

ax3 = nexttile(layout1,3,[1 2]); hold(ax3,'on');
if ~isempty(ttr)
    ttrStart = min(ttr.nav_time);
    ttrEnd = max(ttr.nav_time);
    duringTtr = lbl0.nav_time >= ttrStart & lbl0.nav_time <= ttrEnd;
    plot(ax3,lbl0.nav_time(duringTtr),navLblHorizontalM(duringTtr), ...
        'Color',[0.25,0.25,0.25],'DisplayName','RAMSES ID 0 - NAV');
    for k = 1:nBeacons
        if ~isnat(ttrStartNavTime(k))
            xline(ax3,ttrStartNavTime(k),'--', ...
                'Color',beaconColors(k,:), ...
                'DisplayName',"ID "+beaconIds(k)+" first TTR");
        end
    end
else
    text(ax3,0.5,0.5,'No LBL TTR observations','Units','normalized', ...
        'HorizontalAlignment','center');
end
grid(ax3,'on'); xlabel(ax3,'NAV UTC'); ylabel(ax3,'Horizontal difference (m)');
title(ax3,'(c) RAMSES ID 0 minus NAV during TTR input'); xtickformat(ax3,'HH:mm');
legend(ax3,'Location','northeast');

title(layout1,'INS, LBL and USBL spatial overview');
export_figure(fig1,outputDir,"ins_lbl_usbl_spatial_overview");
close(fig1);

%% 图2：数据质量与时间覆盖范围（两行两列，使用7×7 paper画布）
fig2 = myfigurestartup(7,7,'paper');
set(fig2,'Visible','off');
delete(findall(fig2,'Type','axes'));
layout2 = tiledlayout(fig2,2,2,'Padding','compact','TileSpacing','compact');

ax5 = nexttile(layout2,1); hold(ax5,'on');
semilogy(ax5,nav2.utc_time(2:end),max(nav2StepM,0.01), ...
    'Color',[0.15,0.35,0.70]);
set(ax5,'YScale','log');
yline(ax5,jumpThresholdM,'--','20 m segmentation threshold', ...
    'HandleVisibility','off');
xline(ax5,min(log1Dvl.utc_time),'-.','DVL starts', ...
    'HandleVisibility','off');
if ~isempty(ttr)
    xline(ax5,min(ttr.nav_time),':','LBL TTR starts', ...
        'HandleVisibility','off');
end
grid(ax5,'on'); xlabel(ax5,'NAV UTC'); ylabel(ax5,'Position step (m/s)');
title(ax5,'(a) Navigation discontinuities'); xtickformat(ax5,'HH:mm');

ax6 = nexttile(layout2,2); hold(ax6,'on');
plot(ax6,nav2.utc_time,nav2.latitude_std_m,'DisplayName','Latitude SD');
plot(ax6,nav2.utc_time,nav2.longitude_std_m,'DisplayName','Longitude SD');
grid(ax6,'on'); xlabel(ax6,'NAV UTC'); ylabel(ax6,'Position SD (m)');
title(ax6,'(b) Reported navigation uncertainty');
legend(ax6,'Location','best'); xtickformat(ax6,'HH:mm');

ax7 = nexttile(layout2,3); hold(ax7,'on');
for k = 1:nBeacons
    q = ttr(ttr.transponder_id == beaconIds(k),:);
    scatter(ax7,q.nav_time,q.ttr_s,12,beaconColors(k,:),'filled', ...
        'DisplayName',"ID "+beaconIds(k));
end
grid(ax7,'on'); xlabel(ax7,'Associated NAV UTC'); ylabel(ax7,'TTR (s)');
title(ax7,'(c) Four-beacon LBL observations');
legend(ax7,'Location','best'); xtickformat(ax7,'HH:mm');

ax8 = nexttile(layout2,4); hold(ax8,'on');
sourceFiles = unique(string(usbl.ptsag.source_file),'stable');
for b = 1:numel(sourceFiles)
    q = usbl.ptsag(string(usbl.ptsag.source_file)==sourceFiles(b),:);
    plot(ax8,[min(q.utc_time)+hours(8),max(q.utc_time)+hours(8)], ...
        [b,b],'-','Color',[0.78,0.78,0.78],'LineWidth',5, ...
        'HandleVisibility','off');
    idsHere = unique(q.transponder_id);
    for k = 1:numel(idsHere)
        z = q(q.transponder_id==idsHere(k),:);
        idx = decimate_indices((1:height(z))',400);
        colorIndex = find(usblIds==idsHere(k),1);
        scatter(ax8,z.utc_time(idx)+hours(8), ...
            repmat(b+0.06*(k-(numel(idsHere)+1)/2),numel(idx),1), ...
            8,usblColors(colorIndex,:),'filled','HandleVisibility','off');
    end
end
grid(ax8,'on'); yticks(ax8,1:numel(sourceFiles));
yticklabels(ax8,"B"+(1:numel(sourceFiles)));
xlabel(ax8,'USBL UTC + 8 h (display clock)'); ylabel(ax8,'USBL file batch');
title(ax8,'(d) USBL temporal coverage'); xtickformat(ax8,'MM-dd HH:mm');

title(layout2,'INS/LBL/USBL quality and time coverage');
export_figure(fig2,outputDir,"ins_lbl_usbl_quality_overview");
close(fig2);

save(fullfile(outputDir,"ins_lbl_usbl_overview.mat"), ...
    'log1Nav','log1Dvl','log2','lbl0','ttr','usbl','datasetSummary', ...
    'comparisonSummary','lblBeaconSummary','usblIdSummary','jumpSummary', ...
    'navLblHorizontalM','-v7.3');

fprintf('\nOverview completed.\n');
fprintf('  log1 INSNAV          : %d rows\n',height(log1Nav));
fprintf('  log2 NAV / LBL ID 0 : %d / %d rows\n',height(nav2),height(lbl0));
fprintf('  LBL TTR              : %d rows\n',height(ttr));
fprintf('  USBL files / PTSAG   : %d / %d\n',numel(usbl.source_files), ...
    height(usbl.ptsag));
fprintf('  NAV jumps > %.0f m   : %d\n',jumpThresholdM,height(jumpSummary));
fprintf('  Results              : %s\n',outputDir);

%% 本脚本使用的局部辅助函数
function tbl = read_xpf_export(file,numericNames)
fid = fopen(file,'rt');
assert(fid>=0,'OVERVIEW:OpenFailed','Cannot open %s.',file);
cleanupFile = onCleanup(@() fclose(fid)); 
format = ['%s%s',repmat('%f',1,numel(numericNames))];
values = textscan(fid,format,'Delimiter','\t','CommentStyle','#', ...
    'MultipleDelimsAsOne',false,'CollectOutput',false, ...
    'ReturnOnError',false);
dateText = string(values{1});
timeText = string(values{2});
utcTime = datetime(dateText+" "+timeText, ...
    'InputFormat','yyyy/MM/dd HH:mm:ss.SSSS','TimeZone','UTC');
utcTime.Format = 'yyyy-MM-dd HH:mm:ss.SSSS';
tbl = array2table([values{3:end}],'VariableNames',numericNames);
tbl.utc_time = utcTime;
tbl = movevars(tbl,'utc_time','Before',1);
end

function ttr = parse_log2_ttr(file,baseDate)
lines = readlines(file,'WhitespaceRule','preserve','EmptyLineRule','skip');
timIndex = find(startsWith(lines,"$PIXOR,TIM_IN,"));
timTime = NaT(numel(timIndex),1,'TimeZone','UTC');
for k = 1:numel(timIndex)
    f = split(extractBefore(lines(timIndex(k))+"*","*"),",");
    timTime(k) = baseDate+hours(str2double(f(3)))+ ...
        minutes(str2double(f(4)))+seconds(str2double(f(5)))+ ...
        milliseconds(str2double(f(6)));
end

ttrIndex = find(startsWith(lines,"$PIXOR,TTR_IN,"));
n = numel(ttrIndex);
navTime = NaT(n,1,'TimeZone','UTC');
lblClockSeconds = nan(n,1);
transponderId = nan(n,1);
signalId = nan(n,1);
ttrS = nan(n,1);
ttrStdS = nan(n,1);
status = nan(n,1);
checksumOk = false(n,1);
nearestTim = interp1(double(timIndex),(1:numel(timIndex))', ...
    double(ttrIndex),'nearest','extrap');
for k = 1:n
    line = lines(ttrIndex(k));
    f = split(extractBefore(line+"*","*"),",");
    navTime(k) = timTime(nearestTim(k));
    lblClockSeconds(k) = hhmmss_to_seconds(f(3));
    transponderId(k) = str2double(f(8));
    signalId(k) = str2double(f(9));
    ttrS(k) = str2double(f(10));
    ttrStdS(k) = str2double(f(11));
    status(k) = str2double(f(12));
    checksumOk(k) = nmea_checksum_ok(line);
end
ttr = table(navTime,lblClockSeconds,transponderId,signalId,ttrS, ...
    ttrStdS,status,checksumOk,'VariableNames', {'nav_time', ...
    'lbl_clock_seconds','transponder_id','signal_id','ttr_s', ...
    'ttr_std_s','status','checksum_ok'});
end

function [eastM,northM] = ll_to_en(latDeg,lonDeg,lat0Deg,lon0Deg,R)
northM = deg2rad(latDeg-lat0Deg)*R;
eastM = deg2rad(lonDeg-lon0Deg)*R*cosd(lat0Deg);
end

function x = valid_depth(x)
x = x(isfinite(x) & abs(x)<9000);
if isempty(x), x = NaN; end
end

function plot_segmented_track(ax,eastM,northM,time,jumpThresholdM, ...
        color,lineStyle,label)
step = hypot(diff(eastM),diff(northM));
segment = cumsum([1;step>jumpThresholdM]);
segments = unique(segment,'stable');
labelUsed = false;
for k = 1:numel(segments)
    mask = segment==segments(k) & isfinite(eastM) & isfinite(northM) & ...
        ~isnat(time);
    if nnz(mask)<2, continue; end
    if labelUsed
        h = plot(ax,eastM(mask),northM(mask),'Color',color, ...
            'LineStyle',lineStyle,'HandleVisibility','off');
    else
        h = plot(ax,eastM(mask),northM(mask),'Color',color, ...
            'LineStyle',lineStyle,'DisplayName',label);
        labelUsed = true;
    end
    h.LineWidth = 1.1;
end
end

function idx = decimate_indices(idx,maxPoints)
if numel(idx)>maxPoints
    idx = idx(round(linspace(1,numel(idx),maxPoints)));
end
end

function secondsOfDay = hhmmss_to_seconds(value)
v = str2double(value);
hour = floor(v/10000);
minute = floor((v-hour*10000)/100);
second = v-hour*10000-minute*100;
secondsOfDay = hour*3600+minute*60+second;
end

function ok = nmea_checksum_ok(line)
line = char(strip(line));
star = find(line=='*',1,'last');
if isempty(star) || star+2>numel(line) || line(1)~='$'
    ok = false;
    return;
end
actual = uint8(0);
for k = 2:star-1
    actual = bitxor(actual,uint8(line(k)));
end
ok = double(actual)==hex2dec(line(star+1:star+2));
end

function write_txt(tbl,file)
writetable(tbl,file,'FileType','text','Delimiter','\t','Encoding','UTF-8');
end

function export_figure(fig,outputDir,stem)
exportgraphics(fig,fullfile(outputDir,stem+".png"),'Resolution',220);
exportgraphics(fig,fullfile(outputDir,stem+".pdf"),'ContentType','vector');
end

function cleanup_scratch(folder)
folder = string(folder);
if isfolder(folder) && startsWith(lower(folder),lower(string(tempdir)))
    rmdir(folder,'s');
end
end

function remove_paths(scriptsDir,figureHelperDir)
rmpath(scriptsDir);
rmpath(figureHelperDir);
end
