%% 第二步：生成 case01 / 0017 的 FRD-IMU、参考 PVA 和初始 PVA
% 本脚本独立读取 raw 原始数据，不修改 step1 的分析结果。
% 输出目录：data/20230504/实际海试/case01/processed/stage2。
%
% PVA 采用以下 11 列格式：
% [0, time_s, latitude_deg, longitude_deg, height_m, ...
%  velocity_north_mps, velocity_east_mps, velocity_down_mps, ...
%  roll_deg, pitch_deg, heading_deg]
%
% 当前 PHINS IMU 原始轴按 FLU（前、左、上）处理，输出为 FRD（前、右、下）。
% 如果根据 Interface Library 确认原始 IMU 实际为 RFU，请将 sourceImuFrame
% 改为 "RFU"，脚本会采用工程中的 imuRFU2FRD 轴变换关系。

clear; clc; close all;

%% 路径和配置
scriptDir = string(fileparts(mfilename('fullpath')));
repoDir = fileparts(fileparts(scriptDir));
caseDir = fullfile(repoDir,"data","20230504","实际海试","case01");
dataDir = fullfile(caseDir,"raw");
outputDir = fullfile(caseDir,"processed","stage2");
if ~isfolder(outputDir), mkdir(outputDir); end
figureHelperDir = fullfile(repoDir,"psins2401","mytest","00_all_func");
if ~isfolder(figureHelperDir)
    error('PHINS:MissingFigureHelper','找不到 myfigurestartup 所在目录：%s',figureHelperDir);
end
addpath(figureHelperDir);
cleanupFigurePath = onCleanup(@() rmpath(figureHelperDir)); %#ok<NASGU>

% PHINS 产品规格书给出了导航量，但没有在本文件中展开接口轴定义。
% 候选搜索表明，XPF 导出的 speedX/Y/Z 直接按 PHINS 船体 FLU 使用，
% 不再重复施加 RDI 原始报文的 X=Y、Y=-X 变换。
sourceImuFrame = "FLU";
dvlSourceFrame = "PHINS_FLU";
dvlMountYawDeg = -44.5;
dvlRotationOrder = "RyRx";
dvlQualityThresholdMps = 0.25;

base = "log1-20230504-0017";
navFile = fullfile(dataDir,base+"-ins-nav.xpf.txt");
imuFile = fullfile(dataDir,base+"-ins.xpf.txt");
dvlFile = fullfile(dataDir,base+"-dvl.xpf.txt");
log2File = fullfile(dataDir,"log2-20230504-0017.dat");
requiredFiles = [navFile,imuFile,dvlFile,log2File];
for k = 1:numel(requiredFiles)
    if ~isfile(requiredFiles(k))
        error('PHINS:MissingInput','找不到输入文件：%s',requiredFiles(k));
    end
end

%% 读取原始 INS、nav 和 PTSAG 长基线定位结果
nav = read_xpf_table(navFile,{'algoStatus1','algoStatus2','algoStatus3','latitude_deg', ...
    'longitude_deg','altitude_m','heading_deg','roll_deg','pitch_deg','speedNorth_mps', ...
    'speedEast_mps','speedUp_mps','heave_m','surge_m','sway_m'});
imuRaw = read_xpf_table(imuFile,{'frameCounter','fog1','fog2','fog3','acc1','acc2','acc3','sensorStatus'});
dvl = read_xpf_table(dvlFile,{'speedX_mps','speedY_mps','speedZ_mps','sensorSoundVelocity_mps', ...
    'externalSoundVelocity_mps','altitude_m','speedXSd_mps','speedYSd_mps','speedZSd_mps'});
ptsag = read_ptsag(log2File);

%% 生成 FRD IMU
% 原始文件头的单位为：FOG 10^-3 arcsec，ACC 10^-7 m/s；两者均按增量保存。
imu_frd = convert_imu_to_frd(imuRaw,sourceImuFrame);

%% 生成不改变轨迹的长基线参考 PVA
% PTSAG 的 transponder_id=0 为载体定位结果；固定信标记录不参与参考轨迹。
mask = ptsag.transponder_id == 0 & isfinite(ptsag.latitude_deg) & ...
    isfinite(ptsag.longitude_deg) & isfinite(ptsag.computed_depth_m);
pva_ref_utc = ptsag.utc_time(mask);
pva_ref = nan(nnz(mask),11);
pva_ref(:,1) = 0;
pva_ref(:,2) = relative_time_s(pva_ref_utc,imuRaw.utc_time(1));
pva_ref(:,3) = ptsag.latitude_deg(mask);
pva_ref(:,4) = ptsag.longitude_deg(mask);
pva_ref(:,5) = -ptsag.computed_depth_m(mask);
% LBL 参考结果在该报文中只提供位置和深度，速度、姿态保留为 NaN。

%% 生成独立的初始 PVA 状态
% 只输出一个状态，不替换或平移 pva_ref。
validNav = isfinite(nav.latitude_deg) & isfinite(nav.longitude_deg) & ...
    isfinite(nav.altitude_m) & isfinite(nav.speedNorth_mps) & ...
    isfinite(nav.speedEast_mps) & isfinite(nav.speedUp_mps) & ...
    isfinite(nav.roll_deg) & isfinite(nav.pitch_deg) & isfinite(nav.heading_deg);
if ~any(validNav)
    error('PHINS:NoInitialPVA','nav 中没有完整的初始 PVA 状态。');
end
firstNav = find(validNav,1,'first');
pva0 = [0, relative_time_s(nav.utc_time(firstNav),imuRaw.utc_time(1)), ...
    nav.latitude_deg(firstNav), nav.longitude_deg(firstNav), nav.altitude_m(firstNav), ...
    nav.speedNorth_mps(firstNav), nav.speedEast_mps(firstNav), -nav.speedUp_mps(firstNav), ...
    nav.roll_deg(firstNav), nav.pitch_deg(firstNav), nav.heading_deg(firstNav)];

%% DVL 投影到导航坐标系，并与 INS 速度对比
% 先按 PHINS 船体 FLU 解释 XPF 的 speedX/Y/Z，补偿候选搜索得到的
% DVL 固定安装航向角，再使用 PHINS heading/roll/pitch 投影到 NED。
[dvlProjected, speedComparison] = project_dvl_to_ned(dvl,nav,imuRaw.utc_time(1), ...
    dvlSourceFrame,dvlMountYawDeg,dvlRotationOrder);
dvlSpeedStatistics = make_speed_statistics(speedComparison);
[dvlQualityMask,dvlQualitySummary] = make_dvl_quality_mask( ...
    speedComparison,dvlQualityThresholdMps);
speedComparison.dvl_quality_flag = dvlQualityMask;

coordinateSummary = table( ...
    ["DVL 原始 X";"DVL 原始 Y";"DVL 原始 Z";"PHINS 船体 X";"PHINS 船体 Y";"PHINS 船体 Z"; ...
     "DVL 安装航向角";"导航 X";"导航 Y";"导航 Z";"航向角定义";"横滚角定义";"俯仰角定义"; ...
     "姿态组合顺序";"质量筛选阈值"], ...
    ["前";"左";"上";"前";"左";"上";"相对 PHINS 的固定偏角"; ...
     "北";"东";"下";"北起顺时针为正";"左舷上抬为正";"艏部下俯为正"; ...
     "Ry*Rx";"水平误差阈值"], ...
    ["speedX_mps";"speedY_mps";"speedZ_mps";"dvl_body_x_flu_mps";"dvl_body_y_flu_mps"; ...
     "dvl_body_z_flu_mps";sprintf('%.3f deg',dvlMountYawDeg);"dvl_north_mps"; ...
     "dvl_east_mps";"dvl_down_mps";"heading_deg";"roll_deg";"pitch_deg"; ...
     "dvlRotationOrder";sprintf('%.3f m/s',dvlQualityThresholdMps)], ...
    'VariableNames',{'item','positive_direction','output_field'});

%% 保存 stage2 数据产品
imu_frd_file = fullfile(outputDir,'log1_imu_frd.txt');
pva_ref_file = fullfile(outputDir,'log2_ptsag参考pva.txt');
pva0_file = fullfile(outputDir,'nav初始pva.txt');
dvl_file = fullfile(outputDir,'log1_dvl_投影到导航系.txt');
comparison_file = fullfile(outputDir,'log1_DVL与INS三方向速度对比.txt');
write_txt(imu_frd,imu_frd_file);
write_matrix(pva_ref,pva_ref_file);
write_matrix(pva0,pva0_file);
write_txt(dvlProjected,dvl_file);
write_txt(speedComparison,comparison_file);
write_txt(dvlSpeedStatistics,fullfile(outputDir,'log1_DVL与INS速度统计.txt'));
write_txt(dvlQualitySummary,fullfile(outputDir,'log1_DVL质量筛选统计.txt'));
write_txt(coordinateSummary,fullfile(outputDir,'log1_DVL坐标系说明.txt'));

fig = myfigurestartup(7,7,'paper');
set(fig,'Visible','off'); delete(findall(fig,'Type','axes'));
tiledlayout(fig,3,1,'Padding','compact','TileSpacing','compact');
axisNames = ["北向";"东向";"下向"];
dvlFields = ["dvl_north_mps";"dvl_east_mps";"dvl_down_mps"];
insFields = ["ins_north_mps";"ins_east_mps";"ins_down_mps"];
for k = 1:3
    ax = nexttile; hold(ax,'on');
    plot(ax,speedComparison.utc_time,speedComparison.(dvlFields(k)), ...
        'DisplayName','DVL 投影');
    plot(ax,speedComparison.utc_time,speedComparison.(insFields(k)), ...
        '--','DisplayName','INS 导航');
    grid(ax,'on'); xlabel(ax,'时间'); ylabel(ax,'速度 / m·s^{-1}');
    title(ax,"（"+char('a'+k-1)+"）"+axisNames(k)+"速度对比");
    legend(ax,'Location','best'); xtickformat(ax,'HH:mm');
end
export_png(fig,fullfile(outputDir,'图5_DVL投影与INS三方向速度对比.png')); close(fig);

% 图6：质量筛选后的对比，仅用于观察正常 DVL 观测与 INS 的一致性。
% 图5 保留全部原始观测，不能用图6替代原始数据质量判断。
fig = myfigurestartup(8,8,'paper');
set(fig,'Visible','off'); delete(findall(fig,'Type','axes'));
tiledlayout(fig,3,1,'Padding','compact','TileSpacing','compact');
for k = 1:3
    ax = nexttile; hold(ax,'on');
    dvlValue = speedComparison.(dvlFields(k));
    dvlValue(~dvlQualityMask) = NaN;
    plot(ax,speedComparison.utc_time,dvlValue, ...
        'DisplayName','DVL 投影（质量筛选）');
    plot(ax,speedComparison.utc_time,speedComparison.(insFields(k)), ...
        '--','DisplayName','INS 导航');
    grid(ax,'on'); xlabel(ax,'时间'); ylabel(ax,'速度 / m·s^{-1}');
    title(ax,"（"+char('a'+k-1)+"）"+axisNames(k)+"速度对比（质量筛选）");
    legend(ax,'Location','best'); xtickformat(ax,'HH:mm');
end
export_png(fig,fullfile(outputDir,'图6_DVL投影与INS三方向速度对比_质量筛选.png')); close(fig);

summary = table( ...
    ["IMU-FRD";"LBL参考PVA";"nav初始PVA"], ...
    [height(imu_frd);size(pva_ref,1);size(pva0,1)], ...
    [string(imu_frd.utc_time(1));string(pva_ref_utc(1));string(nav.utc_time(firstNav))], ...
    [string(imu_frd.utc_time(end));string(pva_ref_utc(end));string(nav.utc_time(firstNav))], ...
    'VariableNames',{'data_type','row_count','start_time','end_time'});
write_txt(summary,fullfile(outputDir,'step2_输出摘要.txt'));

save(fullfile(outputDir,'step2_case01_0017.mat'),'imuRaw','imu_frd','nav','dvl','ptsag', ...
    'pva_ref','pva0','dvlProjected','speedComparison','dvlSpeedStatistics', ...
    'dvlQualityMask','dvlQualitySummary','coordinateSummary','sourceImuFrame', ...
    'dvlSourceFrame','dvlMountYawDeg','dvlRotationOrder', ...
    'dvlQualityThresholdMps','summary','-v7.3');

fprintf('第二步数据转换完成。\n');
fprintf('IMU-FRD：%d 行；LBL 参考 PVA：%d 行；初始 PVA：1 行。\n', ...
    height(imu_frd),size(pva_ref,1));
fprintf('DVL 投影：%d 行；三方向速度对比已输出。\n',height(dvlProjected));
fprintf('DVL 质量筛选：保留 %d 行，剔除 %d 行（水平误差阈值 %.3f m/s）。\n', ...
    nnz(dvlQualityMask),nnz(~dvlQualityMask),dvlQualityThresholdMps);
fprintf('输入轴系：%s；输出轴系：FRD。\n',sourceImuFrame);
fprintf('结果目录：%s\n',outputDir);

%% 本地函数
function tbl = read_xpf_table(file,numericNames)
fid = fopen(file,'rt');
if fid < 0, error('PHINS:OpenFailed','无法打开：%s',file); end
cleanupFile = onCleanup(@() fclose(fid)); %#ok<NASGU>
format = ['%s%s',repmat('%f',1,numel(numericNames))];
values = textscan(fid,format,'Delimiter',{char(9),' '},'CommentStyle','#', ...
    'MultipleDelimsAsOne',true,'CollectOutput',false,'ReturnOnError',false);
dateText = string(values{1}); timeText = string(values{2});
utcTime = datetime(dateText+" "+timeText,'InputFormat','yyyy/MM/dd HH:mm:ss.SSSS','TimeZone','UTC');
utcTime.Format = 'yyyy-MM-dd HH:mm:ss.SSSS';
tbl = array2table([values{3:end}],'VariableNames',numericNames);
tbl.utc_time = utcTime; tbl = movevars(tbl,'utc_time','Before',1);
end

function [projected,comparison] = project_dvl_to_ned(dvl,nav,t0,sourceFrame,mountYawDeg,rotationOrder)
n = height(dvl);
dvlTimeS = posixtime(dvl.utc_time);
navTimeS = posixtime(nav.utc_time);
heading = interp_nav_field(navTimeS,nav.heading_deg,dvlTimeS);
roll = interp_nav_field(navTimeS,nav.roll_deg,dvlTimeS);
pitch = interp_nav_field(navTimeS,nav.pitch_deg,dvlTimeS);
insNorth = interp_nav_field(navTimeS,nav.speedNorth_mps,dvlTimeS);
insEast = interp_nav_field(navTimeS,nav.speedEast_mps,dvlTimeS);
insDown = -interp_nav_field(navTimeS,nav.speedUp_mps,dvlTimeS);

% 原始 DVL 坐标 -> PHINS 船体 FLU 坐标。
switch upper(string(sourceFrame))
    case "PHINS_FLU"
        bodyX = dvl.speedX_mps;
        bodyY = dvl.speedY_mps;
        bodyZ = dvl.speedZ_mps;
    case "RDI_INSTRUMENT"
        bodyX = dvl.speedY_mps;
        bodyY = -dvl.speedX_mps;
        bodyZ = dvl.speedZ_mps;
    otherwise
        error('PHINS:UnsupportedDvlFrame','不支持的 DVL 输入坐标系：%s',sourceFrame);
end
dvlNorth = nan(n,1); dvlEast = nan(n,1); dvlDown = nan(n,1);
mount = deg2rad(mountYawDeg); cm = cos(mount); sm = sin(mount);
mountR = [cm,-sm,0;sm,cm,0;0,0,1];
for k = 1:n
    if any(~isfinite([heading(k),roll(k),pitch(k),bodyX(k),bodyY(k),bodyZ(k)]))
        continue;
    end
    psi = deg2rad(heading(k)); phi = deg2rad(roll(k)); theta = deg2rad(pitch(k));
    cpsi = cos(psi); spsi = sin(psi);
    cphi = cos(phi); sphi = sin(phi);
    ctheta = cos(theta); stheta = sin(theta);
    % NED <- FLU：零姿态时前/左/上分别对应北/西/上。
    c0 = diag([1,-1,-1]);
    rz = [cpsi,-spsi,0; spsi,cpsi,0; 0,0,1];
    rx = [1,0,0; 0,cphi,-sphi; 0,sphi,cphi];
    ry = [ctheta,0,stheta; 0,1,0; -stheta,0,ctheta];
    if upper(string(rotationOrder)) == "RYRX"
        rb = ry*rx;
    else
        rb = rx*ry;
    end
    vNed = rz*c0*rb*mountR*[bodyX(k);bodyY(k);bodyZ(k)];
    dvlNorth(k) = vNed(1); dvlEast(k) = vNed(2); dvlDown(k) = vNed(3);
end

projected = table(dvl.utc_time,relative_time_s(dvl.utc_time,t0), ...
    dvl.speedX_mps,dvl.speedY_mps,dvl.speedZ_mps,bodyX,bodyY,bodyZ, ...
    dvlNorth,dvlEast,dvlDown,heading,roll,pitch, ...
    'VariableNames',{'utc_time','time_s','dvl_raw_x_mps','dvl_raw_y_mps','dvl_raw_z_mps', ...
    'dvl_body_x_flu_mps','dvl_body_y_flu_mps','dvl_body_z_flu_mps', ...
    'dvl_north_mps','dvl_east_mps','dvl_down_mps','heading_deg','roll_deg','pitch_deg'});

comparison = table(dvl.utc_time,relative_time_s(dvl.utc_time,t0), ...
    dvlNorth,dvlEast,dvlDown,insNorth,insEast,insDown, ...
    dvlNorth-insNorth,dvlEast-insEast,dvlDown-insDown, ...
    'VariableNames',{'utc_time','time_s','dvl_north_mps','dvl_east_mps','dvl_down_mps', ...
    'ins_north_mps','ins_east_mps','ins_down_mps','north_error_mps','east_error_mps','down_error_mps'});
end

function value = interp_nav_field(timeS,data,timeQueryS)
mask = isfinite(timeS) & isfinite(data);
if nnz(mask) < 2
    value = nan(size(timeQueryS));
    return;
end
[timeUnique,uniqueIndex] = unique(timeS(mask),'stable');
dataUnique = data(mask); dataUnique = dataUnique(uniqueIndex);
value = interp1(timeUnique,dataUnique,timeQueryS,'linear',NaN);
end

function stats = make_speed_statistics(comparison)
axisName = ["北向";"东向";"下向"];
dvlData = [comparison.dvl_north_mps,comparison.dvl_east_mps,comparison.dvl_down_mps];
insData = [comparison.ins_north_mps,comparison.ins_east_mps,comparison.ins_down_mps];
rowCount = zeros(3,1); meanError = nan(3,1); rmsError = nan(3,1);
trimmedRmsError = nan(3,1); stdError = nan(3,1); correlation = nan(3,1);
for k = 1:3
    mask = isfinite(dvlData(:,k)) & isfinite(insData(:,k));
    rowCount(k) = nnz(mask);
    if rowCount(k) >= 2
        err = dvlData(mask,k)-insData(mask,k);
        meanError(k) = mean(err);
        rmsError(k) = sqrt(mean(err.^2));
        sortedError = sort(err.^2);
        keepCount = max(2,floor(0.90*numel(sortedError)));
        trimmedRmsError(k) = sqrt(mean(sortedError(1:keepCount)));
        stdError(k) = std(err);
        correlation(k) = corr(dvlData(mask,k),insData(mask,k));
    end
end
stats = table(axisName,rowCount,meanError,rmsError,stdError,correlation, ...
    trimmedRmsError, ...
    'VariableNames',{'axis','row_count','mean_error_mps','rms_error_mps', ...
    'std_error_mps','correlation','rms_error_trimmed90_mps'});
end

function [qualityMask,summary] = make_dvl_quality_mask(comparison,thresholdMps)
horizontalError = hypot(comparison.north_error_mps,comparison.east_error_mps);
qualityMask = isfinite(horizontalError) & horizontalError <= thresholdMps & ...
    isfinite(comparison.dvl_down_mps) & isfinite(comparison.ins_down_mps);
summary = table(height(comparison),nnz(qualityMask),nnz(~qualityMask),thresholdMps, ...
    100*nnz(qualityMask)/height(comparison), ...
    'VariableNames',{'total_row_count','usable_row_count','rejected_row_count', ...
    'horizontal_error_threshold_mps','usable_percent'});
end

function tbl = read_ptsag(file)
fid = fopen(file,'rt');
if fid < 0, error('PHINS:OpenFailed','无法打开：%s',file); end
cleanupFile = onCleanup(@() fclose(fid)); %#ok<NASGU>
rawLines = textscan(fid,'%s','Delimiter','\n','Whitespace','');
lines = string(rawLines{1});
lines = lines(startsWith(lines,'$PTSAG,'));
n = numel(lines);
tbl = table('Size',[n 6],'VariableTypes',{'datetime','double','double','double','double','double'}, ...
    'VariableNames',{'utc_time','transponder_id','latitude_deg','longitude_deg','computed_depth_m','device_depth_m'});
tbl.utc_time.TimeZone = 'UTC';
for k = 1:n
    f = split(strip_checksum(lines(k)),',');
    if numel(f) < 15, continue; end
    tbl.utc_time(k) = parse_time(f(3),f(4),f(5),f(6));
    tbl.transponder_id(k) = str2double(f(7));
    tbl.latitude_deg(k) = dm_to_deg(f(8),f(9));
    tbl.longitude_deg(k) = dm_to_deg(f(10),f(11));
    tbl.computed_depth_m(k) = str2double(f(13));
    tbl.device_depth_m(k) = str2double(f(15));
end
tbl.utc_time.Format = 'yyyy-MM-dd HH:mm:ss.SSSS';
end

function out = convert_imu_to_frd(imuRaw,sourceFrame)
n = height(imuRaw);
out = table(imuRaw.utc_time,relative_time_s(imuRaw.utc_time,imuRaw.utc_time(1)), ...
    imuRaw.frameCounter,nan(n,1),nan(n,1),nan(n,1),nan(n,1),nan(n,1),nan(n,1),imuRaw.sensorStatus, ...
    'VariableNames',{'utc_time','time_s','frameCounter','dtheta_x_rad','dtheta_y_rad', ...
    'dtheta_z_rad','dv_x_mps','dv_y_mps','dv_z_mps','sensorStatus'});

% PHINS 原始单位转换为弧度增量和 m/s 速度增量。
fog = [imuRaw.fog1,imuRaw.fog2,imuRaw.fog3] * 1e-3 * pi/(180*3600);
acc = [imuRaw.acc1,imuRaw.acc2,imuRaw.acc3] * 1e-7;
switch upper(string(sourceFrame))
    case "FLU"
        % 前左上 -> 前右下：保持前向，左/上取反。
        gyro_frd = [fog(:,1),-fog(:,2),-fog(:,3)];
        acc_frd = [acc(:,1),-acc(:,2),-acc(:,3)];
    case "RFU"
        % 沿用工程中的 imuRFU2FRD 轴顺序：右前上 -> 前右下。
        gyro_frd = [fog(:,2),fog(:,1),-fog(:,3)];
        acc_frd = [acc(:,2),acc(:,1),-acc(:,3)];
    otherwise
        error('PHINS:UnsupportedImuFrame','不支持的 IMU 输入轴系：%s',sourceFrame);
end
out{:,4:6} = gyro_frd;
out{:,7:9} = acc_frd;
end

function t = relative_time_s(utcTime,t0)
t = seconds(utcTime - t0);
t = double(t(:));
end

function t = parse_time(value,day,month,year)
v = str2double(value); day = str2double(day); month = str2double(month); year = str2double(year);
if any(isnan([v,day,month,year])), t = NaT(1,1,'TimeZone','UTC'); return; end
h = floor(v/10000); m = floor((v-h*10000)/100); s = v-h*10000-m*100;
t = datetime(year,month,day,'TimeZone','UTC') + hours(h) + minutes(m) + seconds(s);
end

function deg = dm_to_deg(value,hemisphere)
v = str2double(value); if isnan(v), deg = NaN; return; end
deg = floor(v/100) + mod(v,100)/60;
if any(strcmpi(string(hemisphere),["S","W"])), deg = -deg; end
end

function key = strip_checksum(line)
key = extractBefore(string(line)+"*",'*');
end

function write_txt(tbl,file)
writetable(tbl,file,'FileType','text','Delimiter','\t','Encoding','UTF-8');
end

function write_matrix(value,file)
fid = fopen(file,'wt','n','UTF-8');
if fid < 0, error('PHINS:WriteFailed','无法写入：%s',file); end
cleanupFile = onCleanup(@() fclose(fid)); %#ok<NASGU>
for r = 1:size(value,1)
    fprintf(fid,'%.12g',value(r,1));
    fprintf(fid,'\t%.12g',value(r,2:end));
    fprintf(fid,'\n');
end
end

function export_png(fig,file)
exportgraphics(fig,file,'Resolution',300,'BackgroundColor','white');
end
