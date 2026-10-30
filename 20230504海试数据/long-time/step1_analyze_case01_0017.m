%% 第一步：分析 case01 的 0017 海试数据
% 本脚本只读取 raw 原始数据，不修改原始文件。
% log1 用于分析 IMU、深度计、导航、DVL 和 LBL 观测。
% log2 重点分析长基线观测、信标信息和定位结果。
% 本步骤只输出分析图、统计报告和摘要；数据转换由 step2 执行。
% 结果保存到 data/20230504/实际海试/case01/processed/stage1。

clear; clc; close all;

%% 路径和输入文件
scriptDir = string(fileparts(mfilename('fullpath')));
repoDir = fileparts(fileparts(scriptDir));
dataDir = fullfile(repoDir,"data","20230504","实际海试","case01","raw");
caseDir = fileparts(dataDir);
outputDir = fullfile(caseDir,"processed","stage1");
if ~isfolder(outputDir), mkdir(outputDir); end
figureHelperDir = fullfile(repoDir,"psins2401","mytest","00_all_func");
if ~isfolder(figureHelperDir), error('PHINS:MissingFigureHelper','找不到 myfigurestartup 所在目录：%s',figureHelperDir); end
addpath(figureHelperDir);
cleanupFigurePath = onCleanup(@() rmpath(figureHelperDir)); %#ok<NASGU>

base = "log1-20230504-0017";
navFile = fullfile(dataDir,base+"-ins-nav.xpf.txt");
imuFile = fullfile(dataDir,base+"-ins.xpf.txt");
dvlFile = fullfile(dataDir,base+"-dvl.xpf.txt");
depthFile = fullfile(dataDir,base+"-ins-depth.xpf.txt");
lblFile = fullfile(dataDir,base+"-lbl.xpf.txt");
log2File = fullfile(dataDir,"log2-20230504-0017.dat");

requiredFiles = [navFile,imuFile,dvlFile,depthFile,lblFile,log2File];
for k = 1:numel(requiredFiles)
    if ~isfile(requiredFiles(k))
        error('PHINS:MissingInput','找不到输入文件：%s',requiredFiles(k));
    end
end
fprintf('第一步：分析 case01 / 0017 海试数据\n');
fprintf('原始数据：%s\n结果目录：%s\n\n',dataDir,outputDir);

%% 读取 log1
nav = read_xpf_table(navFile,{'algoStatus1','algoStatus2','algoStatus3','latitude_deg', ...
    'longitude_deg','altitude_m','heading_deg','roll_deg','pitch_deg','speedNorth_mps', ...
    'speedEast_mps','speedUp_mps','heave_m','surge_m','sway_m'});
dvl = read_xpf_table(dvlFile,{'speedX_mps','speedY_mps','speedZ_mps','sensorSoundVelocity_mps', ...
    'externalSoundVelocity_mps','altitude_m','speedXSd_mps','speedYSd_mps','speedZSd_mps'});
depth = read_xpf_table(depthFile,{'status','depth_m','depthSd_m','pressure_Pa'});
lbl = read_xpf_table(lblFile,{'beaconId','latitude_deg','longitude_deg','depth_m','range_m','rangeSd_m'});
imu = read_xpf_table(imuFile,{'frameCounter','fog1','fog2','fog3','acc1','acc2','acc3','sensorStatus'});

%% 读取并拆分 log2
log2 = parse_log2(log2File);

%% log1 计算：轨迹、速度、深度和质量
earthRadiusM = 6378137;
navValid = isfinite(nav.latitude_deg) & isfinite(nav.longitude_deg) & ...
    abs(nav.latitude_deg) <= 90 & abs(nav.longitude_deg) <= 180;
if ~any(navValid), error('PHINS:NoValidNavigation','0017 中没有有效的 INS 经纬度。'); end
firstNav = find(navValid,1,'first');
lat0Deg = nav.latitude_deg(firstNav);
lon0Deg = nav.longitude_deg(firstNav);
[navEastM,navNorthM] = ll_to_en(nav.latitude_deg,nav.longitude_deg,lat0Deg,lon0Deg,earthRadiusM);

navHorizontalSpeed = hypot(nav.speedNorth_mps,nav.speedEast_mps);
dvlHorizontalSpeed = hypot(dvl.speedX_mps,dvl.speedY_mps);
navTimeS = posixtime(nav.utc_time);
navDtS = seconds(diff(nav.utc_time));
navStepM = hypot(diff(navEastM),diff(navNorthM));
jumpThresholdM = 20;
navJumpMask = isfinite(navStepM) & navStepM > jumpThresholdM;
dvlAtNav = interp_valid(posixtime(dvl.utc_time),dvlHorizontalSpeed,navTimeS);
depthAtNav = interp_valid(posixtime(depth.utc_time),depth.depth_m,navTimeS);

dvlSpeedMask = isfinite(dvlAtNav) & isfinite(navHorizontalSpeed);
if nnz(dvlSpeedMask) >= 2
    dvlSpeedCorrelation = corr(navHorizontalSpeed(dvlSpeedMask),dvlAtNav(dvlSpeedMask));
    dvlSpeedRmsDifference = sqrt(mean((navHorizontalSpeed(dvlSpeedMask)-dvlAtNav(dvlSpeedMask)).^2));
else
    dvlSpeedCorrelation = NaN;
    dvlSpeedRmsDifference = NaN;
end
% INS 高度通常为负值，深度计深度为正值，这里统一成正向下。
depthDifference = -nav.altitude_m - depthAtNav;
imuStats = make_imu_statistics(imu);
statusSummary = make_status_summary(nav);
completeness = make_completeness_audit(nav,imu,dvl,depth,lbl,log2);
write_txt(completeness.summary,fullfile(outputDir,'step1_数据完整性审计.txt'));
write_txt(completeness.timeGaps,fullfile(outputDir,'step1_时间间隔异常.txt'));
write_txt(log2.checksumSummary,fullfile(outputDir,'log2_校验和完整性统计.txt'));

%% log2 长基线定位结果与 INS 的局部坐标对比
[log2EastM,log2NorthM] = ll_to_en(log2.navigation.latitude_deg, ...
    log2.navigation.longitude_deg,lat0Deg,lon0Deg,earthRadiusM);
shipPtsag = log2.ptsag(log2.ptsag.transponder_id == 0,:);
beaconPtsag = log2.ptsag(ismember(log2.ptsag.transponder_id,[90 91 92 93]),:);
[shipPtsagEastM,shipPtsagNorthM] = ll_to_en(shipPtsag.latitude_deg,shipPtsag.longitude_deg, ...
    lat0Deg,lon0Deg,earthRadiusM);

if height(shipPtsag) >= 2
    shipPtsagTimeS = posixtime(shipPtsag.utc_time);
    shipPtsagEastAtNavM = interp_valid(shipPtsagTimeS,shipPtsagEastM,navTimeS);
    shipPtsagNorthAtNavM = interp_valid(shipPtsagTimeS,shipPtsagNorthM,navTimeS);
    insPtsagEastErrorM = navEastM - shipPtsagEastAtNavM;
    insPtsagNorthErrorM = navNorthM - shipPtsagNorthAtNavM;
    insPtsagHorizontalErrorM = hypot(insPtsagEastErrorM,insPtsagNorthErrorM);
else
    shipPtsagEastAtNavM = nan(size(navEastM));
    shipPtsagNorthAtNavM = nan(size(navNorthM));
    insPtsagEastErrorM = nan(size(navEastM));
    insPtsagNorthErrorM = nan(size(navNorthM));
    insPtsagHorizontalErrorM = nan(size(navEastM));
end
comparisonMask = isfinite(insPtsagHorizontalErrorM);
positionComparison = table(nav.utc_time(comparisonMask),navEastM(comparisonMask), ...
    navNorthM(comparisonMask),shipPtsagEastAtNavM(comparisonMask), ...
    shipPtsagNorthAtNavM(comparisonMask),insPtsagEastErrorM(comparisonMask), ...
    insPtsagNorthErrorM(comparisonMask),insPtsagHorizontalErrorM(comparisonMask), ...
    'VariableNames',{'utc_time','ins_east_m','ins_north_m','ptsag_east_m', ...
    'ptsag_north_m','east_error_m','north_error_m','horizontal_error_m'});

%% 输出统计结果
% 原始解析表和转换数据不在 step1 中落盘，避免与 step2 的数据产品混在一起。
write_txt(imuStats.table,fullfile(outputDir,'log1_IMU统计.txt'));
write_txt(statusSummary,fullfile(outputDir,'log1_算法状态统计.txt'));
write_txt(positionComparison,fullfile(outputDir,'INS与PTSAG载体位置对比.txt'));

jumpIndex = find(navJumpMask)+1;
jumpTable = table(nav.utc_time(jumpIndex),navStepM(navJumpMask),navDtS(navJumpMask), ...
    nav.algoStatus1(jumpIndex),nav.algoStatus2(jumpIndex),nav.algoStatus3(jumpIndex), ...
    'VariableNames',{'utc_time','position_step_m','time_step_s', ...
    'algoStatus1','algoStatus2','algoStatus3'});
write_txt(jumpTable,fullfile(outputDir,'log1_导航跳变_大于20m.txt'));

lblSummary = make_lbl_summary(lbl);
ptsagSummary = make_ptsag_summary(log2.ptsag);
write_txt(lblSummary,fullfile(outputDir,'log1_LBL信标统计.txt'));
write_txt(ptsagSummary,fullfile(outputDir,'log2_PTSAG按编号统计.txt'));

summary = make_summary(nav,dvl,depth,lbl,log2,imuStats,navEastM,navNorthM, ...
    navStepM,navJumpMask,dvlSpeedCorrelation,dvlSpeedRmsDifference,depthDifference, ...
    insPtsagHorizontalErrorM);
write_txt(summary,fullfile(outputDir,'step1_分析摘要.txt'));

%% 图1：log1 导航、姿态和速度
fig1 = myfigurestartup(7,7,'paper');
set(fig1,'Visible','off'); delete(findall(fig1,'Type','axes'));
tiledlayout(fig1,2,2,'Padding','compact','TileSpacing','compact');

ax = nexttile; hold(ax,'on');
plot_decimated(ax,navEastM,navNorthM,40000,[0.05 0.25 0.70],'-','INS 导航轨迹');
plot(ax,navEastM(firstNav),navNorthM(firstNav),'go','MarkerFaceColor','g','DisplayName','起点');
plot(ax,navEastM(end),navNorthM(end),'rs','MarkerFaceColor','r','DisplayName','终点');
if ~isempty(jumpIndex), scatter(ax,navEastM(jumpIndex),navNorthM(jumpIndex),24,'rx','DisplayName','位置跳变 > 20 m'); end
axis(ax,'equal'); grid(ax,'on'); xlabel(ax,'局部东向 / m'); ylabel(ax,'局部北向 / m');
title(ax,'（a）INS 导航轨迹'); legend(ax,'Location','best');

ax = nexttile; hold(ax,'on');
plot(ax,nav.utc_time,nav.heading_deg,'DisplayName','航向角');
plot(ax,nav.utc_time,nav.roll_deg,'DisplayName','横滚角');
plot(ax,nav.utc_time,nav.pitch_deg,'DisplayName','俯仰角');
grid(ax,'on'); ylabel(ax,'角度 / deg'); xlabel(ax,'时间');
title(ax,'（b）INS 姿态角'); legend(ax,'Location','best'); xtickformat(ax,'HH:mm');

ax = nexttile; hold(ax,'on');
plot(ax,nav.utc_time,navHorizontalSpeed,'DisplayName','INS 水平速度');
plot(ax,dvl.utc_time,dvlHorizontalSpeed,'.','DisplayName','DVL 水平速度');
grid(ax,'on'); ylabel(ax,'速度 / m·s^{-1}'); xlabel(ax,'时间');
title(ax,'（c）INS 与 DVL 速度'); legend(ax,'Location','best'); xtickformat(ax,'HH:mm');

ax = nexttile; hold(ax,'on');
plot(ax,nav.utc_time,-nav.altitude_m,'DisplayName','-INS 高度');
plot(ax,depth.utc_time,depth.depth_m,'DisplayName','深度计');
grid(ax,'on'); ylabel(ax,'深度 / m'); xlabel(ax,'时间');
title(ax,'（d）INS 高度与深度计'); legend(ax,'Location','best'); xtickformat(ax,'HH:mm');
export_png(fig1,fullfile(outputDir,'图1_log1导航姿态速度.png')); close(fig1);

%% 图2：log1 深度计、DVL 和 LBL 观测
fig2 = myfigurestartup(7,7,'paper');
set(fig2,'Visible','off'); delete(findall(fig2,'Type','axes'));
tiledlayout(fig2,2,2,'Padding','compact','TileSpacing','compact');

ax = nexttile; hold(ax,'on');
plot(ax,nav.utc_time,depthDifference,'DisplayName','-INS 高度 - 深度计');
yline(ax,0,'k--','HandleVisibility','off'); grid(ax,'on');
ylabel(ax,'差值 / m'); xlabel(ax,'时间'); title(ax,'（a）INS 与深度计差值');
legend(ax,'Location','best'); xtickformat(ax,'HH:mm');

ax = nexttile;
plot(ax,dvl.utc_time,dvl.altitude_m,'Color',[0.10 0.55 0.30]);
grid(ax,'on'); ylabel(ax,'底高程 / m'); xlabel(ax,'时间');
title(ax,'（b）DVL 底高程'); xtickformat(ax,'HH:mm');

ax = nexttile; hold(ax,'on');
lblIds = unique(lbl.beaconId(isfinite(lbl.beaconId)),'stable');
colors = lines(max(1,numel(lblIds)));
for k=1:numel(lblIds)
    mask = lbl.beaconId==lblIds(k)&isfinite(lbl.range_m);
    plot(ax,lbl.utc_time(mask),lbl.range_m(mask),'.','Color',colors(k,:), ...
        'DisplayName',sprintf('信标 %g',lblIds(k)));
end
grid(ax,'on'); ylabel(ax,'距离 / m'); xlabel(ax,'时间');
title(ax,'（c）LBL 距离观测'); legend(ax,'Location','best'); xtickformat(ax,'HH:mm');

ax = nexttile; hold(ax,'on');
for k=1:numel(lblIds)
    mask = lbl.beaconId==lblIds(k)&isfinite(lbl.rangeSd_m);
    plot(ax,lbl.utc_time(mask),lbl.rangeSd_m(mask),'.','Color',colors(k,:), ...
        'DisplayName',sprintf('信标 %g',lblIds(k)));
end
grid(ax,'on'); ylabel(ax,'距离标准差 / m'); xlabel(ax,'时间');
title(ax,'（d）LBL 距离标准差'); legend(ax,'Location','best'); xtickformat(ax,'HH:mm');
export_png(fig2,fullfile(outputDir,'图2_log1深度DVL_LBL.png')); close(fig2);

%% 图3：log2 长基线与定位结果
fig3 = myfigurestartup(7,7,'paper');
set(fig3,'Visible','off'); delete(findall(fig3,'Type','axes'));
tiledlayout(fig3,2,2,'Padding','compact','TileSpacing','compact');

ax = nexttile; hold(ax,'on');
plot_decimated(ax,navEastM,navNorthM,40000,[0.05 0.25 0.70],'-','INS 导航');
if ~isempty(shipPtsag)
    plot_decimated(ax,shipPtsagEastM,shipPtsagNorthM,40000,[0.90 0.25 0.05],'--','PTSAG 载体结果');
end
axis(ax,'equal'); grid(ax,'on'); xlabel(ax,'局部东向 / m'); ylabel(ax,'局部北向 / m');
title(ax,'（a）INS 与 PTSAG 载体定位轨迹'); legend(ax,'Location','best');

ax = nexttile; hold(ax,'on');
if ~isempty(shipPtsag)
    plot_decimated(ax,shipPtsagEastM,shipPtsagNorthM,40000,[0.90 0.25 0.05],'-','载体 ID 0');
end
beaconIds = unique(beaconPtsag.transponder_id(isfinite(beaconPtsag.transponder_id)),'stable');
colors = lines(max(1,numel(beaconIds)));
for k=1:numel(beaconIds)
    mask = beaconPtsag.transponder_id==beaconIds(k)&isfinite(beaconPtsag.latitude_deg)&isfinite(beaconPtsag.longitude_deg);
    [beaconEastM,beaconNorthM] = ll_to_en(beaconPtsag.latitude_deg(mask),beaconPtsag.longitude_deg(mask),lat0Deg,lon0Deg,earthRadiusM);
    if ~isempty(beaconEastM)
        scatter(ax,median(beaconEastM,'omitnan'),median(beaconNorthM,'omitnan'),70,colors(k,:), ...
            'filled','DisplayName',sprintf('固定信标 %g',beaconIds(k)));
    end
end
axis(ax,'equal'); grid(ax,'on'); xlabel(ax,'局部东向 / m'); ylabel(ax,'局部北向 / m');
title(ax,'（b）载体轨迹与固定信标位置'); legend(ax,'Location','best');

ax = nexttile; hold(ax,'on');
ttrIds = unique(log2.ttrIn.beaconId(isfinite(log2.ttrIn.beaconId)),'stable');
colors = lines(max(1,numel(ttrIds)));
for k=1:numel(ttrIds)
    mask = log2.ttrIn.beaconId==ttrIds(k)&isfinite(log2.ttrIn.ttr_value);
    plot(ax,log2.ttrIn.utc_time(mask),log2.ttrIn.ttr_value(mask),'.', ...
        'Color',colors(k,:),'DisplayName',sprintf('信标 %g',ttrIds(k)));
end
grid(ax,'on'); ylabel(ax,'TTR 观测值'); xlabel(ax,'时间');
title(ax,'（c）长基线 TTR 观测'); legend(ax,'Location','best'); xtickformat(ax,'HH:mm');

ax = nexttile; hold(ax,'on');
if ~isempty(shipPtsag)
    plot(ax,shipPtsag.utc_time,shipPtsag.computed_depth_m,'DisplayName','PTSAG 计算深度');
    plot(ax,shipPtsag.utc_time,shipPtsag.device_depth_m,'DisplayName','PTSAG 设备深度');
end
plot(ax,log2.immIn.utc_time,log2.immIn.depth_m,'DisplayName','IMM 输入深度');
grid(ax,'on'); ylabel(ax,'深度 / m'); xlabel(ax,'时间');
title(ax,'（d）长基线相关深度结果'); legend(ax,'Location','best'); xtickformat(ax,'HH:mm');
export_png(fig3,fullfile(outputDir,'图3_log2长基线与定位结果.png')); close(fig3);

%% 图4：不同数据源的定位和深度对比
fig4 = myfigurestartup(7,7,'paper');
set(fig4,'Visible','off'); delete(findall(fig4,'Type','axes'));
tiledlayout(fig4,2,2,'Padding','compact','TileSpacing','compact');

ax = nexttile; hold(ax,'on');
plot(ax,nav.utc_time,navEastM,'DisplayName','INS 东向');
plot(ax,log2.navigation.utc_time,log2EastM,'--','DisplayName','log2 PHINS 东向');
if ~isempty(shipPtsag), plot(ax,shipPtsag.utc_time,shipPtsagEastM,':','DisplayName','PTSAG 载体东向'); end
grid(ax,'on'); ylabel(ax,'东向位置 / m'); xlabel(ax,'时间');
title(ax,'（a）东向位置对比'); legend(ax,'Location','best'); xtickformat(ax,'HH:mm');

ax = nexttile; hold(ax,'on');
plot(ax,nav.utc_time,navNorthM,'DisplayName','INS 北向');
plot(ax,log2.navigation.utc_time,log2NorthM,'--','DisplayName','log2 PHINS 北向');
if ~isempty(shipPtsag), plot(ax,shipPtsag.utc_time,shipPtsagNorthM,':','DisplayName','PTSAG 载体北向'); end
grid(ax,'on'); ylabel(ax,'北向位置 / m'); xlabel(ax,'时间');
title(ax,'（b）北向位置对比'); legend(ax,'Location','best'); xtickformat(ax,'HH:mm');

ax = nexttile;
if any(comparisonMask)
    plot(ax,nav.utc_time(comparisonMask),insPtsagHorizontalErrorM(comparisonMask), ...
        'Color',[0.85 0.20 0.10]);
end
grid(ax,'on'); ylabel(ax,'水平位置差 / m'); xlabel(ax,'时间');
title(ax,'（c）INS 与 PTSAG 载体定位水平差'); xtickformat(ax,'HH:mm');

ax = nexttile; hold(ax,'on');
plot(ax,nav.utc_time,-nav.altitude_m,'DisplayName','-INS 高度');
plot(ax,depth.utc_time,depth.depth_m,'DisplayName','深度计');
plot(ax,log2.immIn.utc_time,log2.immIn.depth_m,'DisplayName','log2 IMM 输入');
if ~isempty(shipPtsag), plot(ax,shipPtsag.utc_time,shipPtsag.device_depth_m,'DisplayName','PTSAG 设备深度'); end
grid(ax,'on'); ylabel(ax,'深度 / m'); xlabel(ax,'时间');
title(ax,'（d）不同来源深度对比'); legend(ax,'Location','best'); xtickformat(ax,'HH:mm');
export_png(fig4,fullfile(outputDir,'图4_多源定位与深度对比.png')); close(fig4);

fprintf('第一步分析完成。\n');
fprintf('INSNAV：%d 行；IMU：%d 行；DVL：%d 行；深度计：%d 行；LBL：%d 行。\n', ...
    height(nav),height(imu),height(dvl),height(depth),height(lbl));
fprintf('log2：PTSAG %d 行；TTR %d 行；位置/速度/深度输入 %d/%d/%d 行。\n', ...
    height(log2.ptsag),height(log2.ttrIn),height(log2.posIn),height(log2.spdIn),height(log2.immIn));
fprintf('结果目录：%s\n',outputDir);

%% 本地函数
function tbl = read_xpf_table(file,numericNames)
fid = fopen(file,'rt');
if fid < 0, error('PHINS:OpenFailed','无法打开：%s',file); end
cleanupFile = onCleanup(@() fclose(fid)); %#ok<NASGU>
format = ['%s%s',repmat('%f',1,numel(numericNames))];
% 原始导出中既有制表符，也可能出现制表符与空格混用，因此统一按空白分隔。
values = textscan(fid,format,'Delimiter',{'\t',' '},'CommentStyle','#', ...
    'MultipleDelimsAsOne',true,'CollectOutput',false,'ReturnOnError',false);
dateText = string(values{1}); timeText = string(values{2});
utcTime = datetime(dateText+" "+timeText,'InputFormat','yyyy/MM/dd HH:mm:ss.SSSS','TimeZone','UTC');
utcTime.Format = 'yyyy-MM-dd HH:mm:ss.SSSS';
tbl = array2table([values{3:end}],'VariableNames',numericNames);
tbl.utc_time = utcTime; tbl = movevars(tbl,'utc_time','Before',1);
end

function result = parse_log2(file)
lines = readlines(file,'WhitespaceRule','preserve','EmptyLineRule','skip');
result.lineCount = numel(lines);
result.messageCounts = count_message_types(lines);
result.statusCounts = count_message_types(lines(startsWith(lines,'$PIXOG,')));
result.checksumSummary = make_checksum_summary(lines);
result.navigation = parse_pixse_navigation(lines);
result.posIn = parse_pos_in(lines(startsWith(lines,'$PIXOR,POS_IN,')));
result.spdIn = parse_spd_in(lines(startsWith(lines,'$PIXOR,SPD_IN,')));
result.immIn = parse_imm_in(lines(startsWith(lines,'$PIXOR,IMM_IN,')));
result.ttrIn = parse_ttr_in(lines(startsWith(lines,'$PIXOR,TTR_IN,')));
result.ptsag = parse_ptsag(lines(startsWith(lines,'$PTSAG,')));
result.dbgPos = parse_dbg_pos(lines(startsWith(lines,'$PIXOR,DBG,POS,')));
end

function navigation = parse_pixse_navigation(lines)
headingLines = lines(startsWith(lines,'$HEHDT,'));
positionLines = lines(startsWith(lines,'$PIXSE,POSITI,'));
timeLines = lines(startsWith(lines,'$PIXSE,TIME__,'));
stdLines = lines(startsWith(lines,'$PIXSE,STDPOS,'));
n = min([numel(headingLines),numel(positionLines),numel(timeLines),numel(stdLines)]);
navigation = table('Size',[n 8],'VariableTypes',{'datetime','double','double','double','double','double','double','double'}, ...
    'VariableNames',{'utc_time','heading_deg','latitude_deg','longitude_deg','altitude_m', ...
    'latitude_std_m','longitude_std_m','altitude_std_m'});
navigation.utc_time.TimeZone = 'UTC';
for k=1:n
    h=split(strip_checksum(headingLines(k)),','); p=split(strip_checksum(positionLines(k)),',');
    t=split(strip_checksum(timeLines(k)),','); s=split(strip_checksum(stdLines(k)),',');
    navigation.heading_deg(k)=str2double(h(2)); navigation.latitude_deg(k)=str2double(p(3));
    navigation.longitude_deg(k)=str2double(p(4)); navigation.altitude_m(k)=str2double(p(5));
    navigation.utc_time(k)=parse_time(t(3),"04","05","2023"); navigation.latitude_std_m(k)=str2double(s(3));
    navigation.longitude_std_m(k)=str2double(s(4)); navigation.altitude_std_m(k)=str2double(s(5));
end
navigation.utc_time.Format='yyyy-MM-dd HH:mm:ss.SSSS';
end

function tbl = parse_pos_in(lines)
n=numel(lines);
tbl=table('Size',[n 7],'VariableTypes',{'datetime','double','double','double','double','double','double'}, ...
    'VariableNames',{'utc_time','source_code','source_id','latitude_deg','longitude_deg','latitude_std_m','longitude_std_m'});
tbl.utc_time.TimeZone='UTC';
for k=1:n
    f=split(strip_checksum(lines(k)),','); if numel(f)<14, continue; end
    tbl.utc_time(k)=parse_time(f(3),f(4),f(5),f(6)); tbl.source_code(k)=str2double(f(7));
    tbl.source_id(k)=str2double(f(8)); tbl.latitude_deg(k)=dm_to_deg(f(9),f(10));
    tbl.longitude_deg(k)=dm_to_deg(f(11),f(12)); tbl.latitude_std_m(k)=str2double(f(13));
    tbl.longitude_std_m(k)=str2double(f(14));
end
tbl.utc_time.Format='yyyy-MM-dd HH:mm:ss.SSSS';
end

function tbl = parse_spd_in(lines)
n=numel(lines);
tbl=table('Size',[n 8],'VariableTypes',{'datetime','double','double','double','double','double','double','double'}, ...
    'VariableNames',{'utc_time','source_code','value1','value2','value3','value4','value5','value6'});
tbl.utc_time.TimeZone='UTC';
for k=1:n
    f=split(strip_checksum(lines(k)),','); if numel(f)<13, continue; end
    tbl.utc_time(k)=parse_time(f(3),f(4),f(5),f(6));
    for j=1:7, tbl{k,j+1}=str2double(f(6+j)); end
end
tbl.utc_time.Format='yyyy-MM-dd HH:mm:ss.SSSS';
end

function tbl = parse_imm_in(lines)
n=numel(lines);
tbl=table('Size',[n 5],'VariableTypes',{'datetime','double','double','double','double'}, ...
    'VariableNames',{'utc_time','source_code','value1','depth_m','depth_std_m'});
tbl.utc_time.TimeZone='UTC';
for k=1:n
    f=split(strip_checksum(lines(k)),','); if numel(f)<10, continue; end
    tbl.utc_time(k)=parse_time(f(3),f(4),f(5),f(6)); tbl.source_code(k)=str2double(f(7));
    tbl.value1(k)=str2double(f(8)); tbl.depth_m(k)=str2double(f(9)); tbl.depth_std_m(k)=str2double(f(10));
end
tbl.utc_time.Format='yyyy-MM-dd HH:mm:ss.SSSS';
end

function tbl = parse_ttr_in(lines)
n=numel(lines);
tbl=table('Size',[n 7],'VariableTypes',{'datetime','double','double','double','double','double','double'}, ...
    'VariableNames',{'utc_time','source_code','beaconId','signalId','ttr_value','ttr_std','status'});
tbl.utc_time.TimeZone='UTC';
for k=1:n
    f=split(strip_checksum(lines(k)),','); if numel(f)<12, continue; end
    tbl.utc_time(k)=parse_time(f(3),f(4),f(5),f(6)); tbl.source_code(k)=str2double(f(7));
    tbl.beaconId(k)=str2double(f(8)); tbl.signalId(k)=str2double(f(9));
    tbl.ttr_value(k)=str2double(f(10)); tbl.ttr_std(k)=str2double(f(11)); tbl.status(k)=str2double(f(12));
end
tbl.utc_time.Format='yyyy-MM-dd HH:mm:ss.SSSS';
end

function tbl = parse_ptsag(lines)
n=numel(lines);
tbl=table('Size',[n 11],'VariableTypes',{'datetime','double','double','double','double','string','double','double','double','double','logical'}, ...
    'VariableNames',{'utc_time','frame_number','transponder_id','latitude_deg','longitude_deg','position_flag', ...
    'computed_depth_m','depth_validity','device_depth_m','sequence','checksum_ok'});
tbl.utc_time.TimeZone='UTC';
for k=1:n
    f=split(strip_checksum(lines(k)),','); if numel(f)<15, continue; end
    tbl.frame_number(k)=str2double(erase(f(2),'#')); tbl.utc_time(k)=parse_time(f(3),f(4),f(5),f(6));
    tbl.transponder_id(k)=str2double(f(7)); tbl.latitude_deg(k)=dm_to_deg(f(8),f(9));
    tbl.longitude_deg(k)=dm_to_deg(f(10),f(11)); tbl.position_flag(k)=f(12);
    tbl.computed_depth_m(k)=str2double(f(13)); tbl.depth_validity(k)=str2double(f(14));
    tbl.device_depth_m(k)=str2double(f(15)); tbl.sequence(k)=k; tbl.checksum_ok(k)=checksum_ok(lines(k));
end
tbl.utc_time.Format='yyyy-MM-dd HH:mm:ss.SSSS';
end

function tbl = parse_dbg_pos(lines)
n=numel(lines);
tbl=table('Size',[n 8],'VariableTypes',{'datetime','double','double','double','double','double','double','double'}, ...
    'VariableNames',{'utc_time','transponder_id','latitude_deg','longitude_deg','depth_m','std1_m','std2_m','std3_m'});
tbl.utc_time.TimeZone='UTC';
for k=1:n
    f=split(strip_checksum(lines(k)),','); if numel(f)<17, continue; end
    tbl.utc_time(k)=parse_time(f(4),f(5),f(6),f(7)); tbl.transponder_id(k)=str2double(f(8));
    tbl.latitude_deg(k)=dm_to_deg(f(9),f(10)); tbl.longitude_deg(k)=dm_to_deg(f(11),f(12));
    tbl.depth_m(k)=str2double(f(13)); tbl.std1_m(k)=str2double(f(15));
    tbl.std2_m(k)=str2double(f(16)); tbl.std3_m(k)=str2double(f(17));
end
tbl.utc_time.Format='yyyy-MM-dd HH:mm:ss.SSSS';
end

function key = strip_checksum(line)
key=extractBefore(string(line)+"*","*");
end

function t = parse_time(value,day,month,year)
v=str2double(value); day=str2double(day); month=str2double(month); year=str2double(year);
if any(isnan([v,day,month,year])), t=NaT(1,1,'TimeZone','UTC'); return; end
h=floor(v/10000); m=floor((v-h*10000)/100); s=v-h*10000-m*100;
t=datetime(year,month,day,'TimeZone','UTC')+hours(h)+minutes(m)+seconds(s);
end

function deg = dm_to_deg(value,hemisphere)
v=str2double(value); if isnan(v), deg=NaN; return; end
deg=floor(v/100)+mod(v,100)/60;
if any(strcmpi(string(hemisphere),["S","W"])), deg=-deg; end
end

function ok = checksum_ok(line)
txt=char(strip(line)); star=find(txt=='*',1,'last');
if isempty(star)||star+2>numel(txt)||txt(1)~='$', ok=false; return; end
actual=uint8(0); for k=2:(star-1), actual=bitxor(actual,uint8(txt(k))); end
ok=double(actual)==hex2dec(txt(star+1:star+2));
end

function [eastM,northM] = ll_to_en(latDeg,lonDeg,lat0Deg,lon0Deg,R)
northM=deg2rad(latDeg-lat0Deg)*R; eastM=deg2rad(lonDeg-lon0Deg)*R*cosd(lat0Deg);
end

function out = interp_valid(x,y,xq)
mask=isfinite(x)&isfinite(y);
if nnz(mask)<2, out=nan(size(xq)); return; end
[xUnique,idx]=unique(x(mask)); yUnique=y(mask); yUnique=yUnique(idx);
out=interp1(xUnique,yUnique,xq,'linear',NaN);
end

function stats = make_imu_statistics(imu)
dt=seconds(diff(imu.utc_time)); stats.row_count=height(imu);
stats.median_rate_hz=1/median(dt,'omitnan');
stats.sensor_status_nonzero_count=nnz(imu.sensorStatus~=0);
stats.frame_counter_resets=nnz(diff(imu.frameCounter)<0);
stats.table=table(["数据行数";"中位采样率";"非零传感器状态数";"帧计数回零次数"], ...
    [stats.row_count;stats.median_rate_hz;stats.sensor_status_nonzero_count;stats.frame_counter_resets], ...
    ["行";"Hz";"次";"次"],'VariableNames',{'统计项','数值','单位'});
end

function tbl = make_status_summary(nav)
M=[nav.algoStatus1,nav.algoStatus2,nav.algoStatus3]; [U,~,g]=unique(M,'rows','stable'); c=accumarray(g,1);
tbl=table(U(:,1),U(:,2),U(:,3),c,100*c/height(nav), ...
    repmat("状态码含义需要结合 PHINS 厂商文档",size(U,1),1), ...
    'VariableNames',{'算法状态1','算法状态2','算法状态3','行数','比例_pct','说明'});
end

function tbl = make_lbl_summary(lbl)
ids=unique(lbl.beaconId(isfinite(lbl.beaconId)),'stable'); n=numel(ids);
tbl=table('Size',[n 9],'VariableTypes',repmat({'double'},1,9), ...
    'VariableNames',{'beacon_id','row_count','valid_range_count','range_median_m','range_std_m', ...
    'latitude_median_deg','longitude_median_deg','depth_median_m','range_sd_median'});
for k=1:n
    q=lbl(lbl.beaconId==ids(k),:); valid=isfinite(q.range_m); tbl.beacon_id(k)=ids(k);
    tbl.row_count(k)=height(q); tbl.valid_range_count(k)=nnz(valid);
    tbl.range_median_m(k)=median(q.range_m(valid),'omitnan'); tbl.range_std_m(k)=std(q.range_m(valid),'omitnan');
    tbl.latitude_median_deg(k)=median(q.latitude_deg,'omitnan'); tbl.longitude_median_deg(k)=median(q.longitude_deg,'omitnan');
    tbl.depth_median_m(k)=median(q.depth_m,'omitnan'); tbl.range_sd_median(k)=median(q.rangeSd_m,'omitnan');
end
end

function tbl = make_ptsag_summary(ptsag)
ids=unique(ptsag.transponder_id(isfinite(ptsag.transponder_id)),'stable'); n=numel(ids);
tbl=table('Size',[n 8],'VariableTypes',repmat({'double'},1,8), ...
    'VariableNames',{'transponder_id','row_count','checksum_ok_count','checksum_ok_pct', ...
    'computed_depth_median','device_depth_median','latitude_median_deg','longitude_median_deg'});
for k=1:n
    q=ptsag(ptsag.transponder_id==ids(k),:); tbl.transponder_id(k)=ids(k); tbl.row_count(k)=height(q);
    tbl.checksum_ok_count(k)=nnz(q.checksum_ok); tbl.checksum_ok_pct(k)=100*mean(q.checksum_ok);
    tbl.computed_depth_median(k)=median(q.computed_depth_m,'omitnan'); tbl.device_depth_median(k)=median(q.device_depth_m,'omitnan');
    tbl.latitude_median_deg(k)=median(q.latitude_deg,'omitnan'); tbl.longitude_median_deg(k)=median(q.longitude_deg,'omitnan');
end
end

function tbl = make_summary(nav,dvl,depth,lbl,log2,imuStats,eastM,northM,stepM,jumpMask,dvlCorr,dvlRms,depthDiff,positionDiff)
durationS=seconds(nav.utc_time(end)-nav.utc_time(1));
tbl=table(["INS 导航行数";"INS 导航时长";"INS 中位采样率";"IMU 行数";"IMU 中位采样率"; ...
    "DVL 行数";"深度计行数";"LBL 观测行数";"log2 总行数";"log2 TTR 行数";"log2 PTSAG 行数"; ...
    "INS 轨迹长度";"起终点位移";"INS 中位水平速度";"导航跳变次数";"最大导航步长"; ...
    "DVL/INS 速度相关系数";"DVL/INS 速度均方根差";"INS-深度计中位差";"INS-PTSAG0 中位水平差"], ...
    [height(nav);durationS;1/median(seconds(diff(nav.utc_time)),'omitnan');imuStats.row_count;imuStats.median_rate_hz; ...
    height(dvl);height(depth);height(lbl);log2.lineCount;height(log2.ttrIn);height(log2.ptsag); ...
    sum(stepM(~jumpMask),'omitnan');hypot(eastM(end)-eastM(1),northM(end)-northM(1)); ...
    median(hypot(nav.speedNorth_mps,nav.speedEast_mps),'omitnan');nnz(jumpMask);max(stepM,[],'omitnan'); ...
    dvlCorr;dvlRms;median(depthDiff,'omitnan');median(positionDiff,'omitnan')], ...
    ["行";"s";"Hz";"行";"Hz";"行";"行";"行";"行";"行";"行";"m";"m";"m/s";"次";"m";"1";"m/s";"m";"m"], ...
    'VariableNames',{'分析项','数值','单位'});
end

function audit = make_completeness_audit(nav,imu,dvl,depth,lbl,log2)
% 对各数据源检查时间有效性、重复时间、逆序时间、时间间隔和数值缺失。
names = ["log1 INS导航";"log1 IMU";"log1 DVL";"log1 深度计";"log1 LBL"; ...
    "log2 PHINS导航";"log2 定位输入位置";"log2 定位输入速度"; ...
    "log2 定位输入深度";"log2 TTR观测";"log2 PTSAG结果"];
tables = {nav,imu,dvl,depth,lbl,log2.navigation,log2.posIn,log2.spdIn, ...
    log2.immIn,log2.ttrIn,log2.ptsag};
n = numel(tables);
summaryRows = cell(n,1);
timeRows = cell(0,1);
for k = 1:n
    [summaryRows{k},gapRows] = audit_one_table(names(k),tables{k});
    if ~isempty(gapRows), timeRows{end+1,1} = gapRows; end %#ok<AGROW>
end
summary = vertcat(summaryRows{:});
if isempty(timeRows)
    timeGaps = table(strings(0,1),NaT(0,1,'TimeZone','UTC'),NaN(0,1), ...
        strings(0,1),'VariableNames',{'数据源','时间','间隔_s','说明'});
else
    timeGaps = vertcat(timeRows{:});
end
audit.summary = summary;
audit.timeGaps = timeGaps;
end

function [row,gaps] = audit_one_table(name,tbl)
n = height(tbl);
time = tbl.utc_time;
validTime = ~isnat(time);
timeS = posixtime(time(validTime));
if numel(timeS) >= 2
    dt = diff(timeS);
    medianDt = median(dt(dt > 0),'omitnan');
    duplicateCount = nnz(dt == 0);
    reverseCount = nnz(dt < 0);
    if isfinite(medianDt) && medianDt > 0
        gapMask = dt > max(3*medianDt,medianDt+0.5);
        gapIndex = find(gapMask)+1;
        gaps = table(repmat(name,numel(gapIndex),1),time(gapIndex),dt(gapMask), ...
            repmat("时间间隔明显大于该数据源中位间隔；事件型数据不直接判定为缺失",numel(gapIndex),1), ...
            'VariableNames',{'数据源','时间','间隔_s','说明'});
        gapCount = nnz(gapMask);
    else
        gaps = empty_gap_table();
        gapCount = 0;
    end
else
    medianDt = NaN; duplicateCount = 0; reverseCount = 0; gapCount = 0;
    gaps = empty_gap_table();
end
numericMask = false(1,width(tbl));
for j = 1:width(tbl), numericMask(j) = isnumeric(tbl{:,j}); end
if any(numericMask), missingCount = sum(~isfinite(tbl{:,numericMask}),'all');
else, missingCount = 0; end
if isempty(timeS), startTime = NaT(1,1,'TimeZone','UTC'); endTime = startTime;
else, startTime = min(time(validTime)); endTime = max(time(validTime)); end
row = table(name,n,nnz(validTime),startTime,endTime,seconds(endTime-startTime), ...
    medianDt,duplicateCount,reverseCount,gapCount,missingCount, ...
    'VariableNames',{'数据源','行数','有效时间行数','开始时间','结束时间','覆盖时长_s', ...
    '中位时间间隔_s','重复时间数','逆序时间数','明显间隔数','数值缺失数'});
end

function tbl = empty_gap_table()
tbl = table(strings(0,1),NaT(0,1,'TimeZone','UTC'),NaN(0,1),strings(0,1), ...
    'VariableNames',{'数据源','时间','间隔_s','说明'});
end

function tbl = make_checksum_summary(lines)
n = numel(lines); ok = false(n,1);
for k = 1:n, ok(k) = checksum_ok(lines(k)); end
tbl = table(n,nnz(ok),nnz(~ok),100*mean(ok), ...
    'VariableNames',{'总报文数','校验和正确数','校验和错误数','正确率_pct'});
end

function tbl = count_message_types(lines)
keys=strings(numel(lines),1);
for k=1:numel(lines)
    f=split(strip_checksum(lines(k)),',');
    if isempty(f), keys(k)="空行";
    elseif f(1)=="$PIXOG"&&numel(f)>=3, keys(k)=f(1)+","+f(2)+","+f(3);
    elseif any(f(1)==["$PIXSE","$PIXOR"])&&numel(f)>=2, keys(k)=f(1)+","+f(2);
    else, keys(k)=f(1);
    end
end
[u,~,g]=unique(keys,'stable'); c=accumarray(g,1);
tbl=sortrows(table(u,c,'VariableNames',{'报文类型','行数'}),'行数','descend');
end

function write_txt(tbl,file)
writetable(tbl,file,'FileType','text','Delimiter','\t','Encoding','UTF-8');
end

function export_png(fig,file)
try
    exportgraphics(fig,file,'Resolution',180);
catch ME
    warning('PHINS:图形导出失败','图形导出失败：%s',ME.message);
end
end

function plot_decimated(ax,x,y,maxPoints,color,lineStyle,label)
idx=1:numel(x); if numel(idx)>maxPoints, idx=round(linspace(1,numel(x),maxPoints)); end
plot(ax,x(idx),y(idx),'Color',color,'LineStyle',lineStyle,'DisplayName',label,'LineWidth',1.0);
end

