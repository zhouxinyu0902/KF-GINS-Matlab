clear;
clc;
close all;
%% ========================================================================
% data07_read03_830.m
% 2025-12-07
% 根据 read01 Compass + read02 120 IMU 时间提取830静止段和导航段。
%
% 时间规则：
%   静止段起点：120静止IMU最早有效GPS时间
%   导航段起点：120对准后IMU第一个GPS时间
%   导航段结束：120对准后IMU最后一个GPS时间
%
% 830划分：
%   静止段：[t_static_start,t_nav_start)
%   导航段：[t_nav_start,t_nav_end]
%
% Compass仅用于实验对应和时间核对，不参与830截取起止时间计算。
%
% 输出：
%   temp\ref830_1207_new.mat
%
% Ref830{k}：
%   *_static 静止段
%   *_nav    实际导航段
%
% 兼容字段：
%   Ref830{k}.pva
%   Ref830{k}.imu_binary_frd
%   Ref830{k}.imu_gpchcx_frd
%   Ref830{k}.gpchcx
%   Ref830{k}.gpgga
%
% 上述兼容字段均指导航段。
% ========================================================================
%% 0. 路径
raw_dir = 'D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1207-longtime\raw';
temp_dir = 'D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1207-longtime\temp';
compass_file = fullfile(temp_dir,'compass.mat');
alignment120_file = fullfile(temp_dir,'alignment120_1207.mat');
save_file = fullfile(temp_dir,'ref830_1207_new.mat');
%% 1. 参数
GPS_WEEK = 2396;
GPS_UTC_LEAP = 18;
DATA_DATE = [2025,12,7];
%% ========================================================================
%% 2. 读取 read01 + read02 时间
% ========================================================================
fprintf('\n============================================================\n');
fprintf('1. 根据120 IMU + Compass确定830截取时间\n');
fprintf('============================================================\n');
load(compass_file,'Compass','experiment_time');
load(alignment120_file,'Alignment120');
if numel(Compass) ~= 2 || numel(Alignment120) ~= 2
    error('Compass和Alignment120均应包含下午、晚上两段数据。');
end
Window = struct();
for k = 1:2
    compass_start = Compass{k}.time(1);
    compass_end = Compass{k}.time(end);
    compass_duration = compass_end-compass_start;
    if isfield(Alignment120(k),'imu_static_time_gps') && ~isempty(Alignment120(k).imu_static_time_gps)
        t_static_start = Alignment120(k).imu_static_time_gps(1);
    elseif isfield(Alignment120(k),'static_start_gps') && ~isempty(Alignment120(k).static_start_gps)
        t_static_start = Alignment120(k).static_start_gps;
    else
        error('%s：read02中没有找到120静止IMU起始GPS时间。',Compass{k}.name);
    end
    if isfield(Alignment120(k),'imu_time_gps') && ~isempty(Alignment120(k).imu_time_gps)
        t_nav_start = Alignment120(k).imu_time_gps(1);
        t_nav_end = Alignment120(k).imu_time_gps(end);
    else
        error('%s：read02中没有找到120对准后IMU有效时间。',Compass{k}.name);
    end
    Window(k).name = Compass{k}.name;
    Window(k).static_start = t_static_start;
    Window(k).static_end = t_nav_start;
    Window(k).nav_start = t_nav_start;
    Window(k).nav_end = t_nav_end;
    Window(k).compass_duration = compass_duration;
    Window(k).compass_range = [compass_start,compass_end];
    Window(k).imu120_end = t_nav_end;
    fprintf('\n---------------- %s ----------------\n',Compass{k}.name);
    fprintf('120静止IMU起点：      %.3f\n',t_static_start);
    fprintf('120对准后IMU起点：    %.3f\n',t_nav_start);
    fprintf('120对准后IMU结束：    %.3f\n',t_nav_end);
    fprintf('830静止段：           %.3f ~ %.3f  (%.2f min)\n',t_static_start,t_nav_start,(t_nav_start-t_static_start)/60);
    fprintf('830导航段：           %.3f ~ %.3f  (%.2f min)\n',t_nav_start,t_nav_end,(t_nav_end-t_nav_start)/60);
    fprintf('Compass有效时长：     %.3f s = %.2f min，仅用于核对\n',compass_duration,compass_duration/60);
end
%% ========================================================================
%% 3. 配置830文件
% ========================================================================
REF_FILE = cell(2,1);
REF_FILE{1} = {
    '043941_4752450_binary.dat','043941_4752450_GPCHCX.txt','043941_4752450_GPGGA.txt'
    '083942_4752450_binary.dat','083942_4752450_GPCHCX.txt','083942_4752450_GPGGA.txt'
};
REF_FILE{2} = {
    '110223_4752450_binary.dat','110223_4752450_GPCHCX.txt','110223_4752450_GPGGA.txt'
    '150224_4752450_binary.dat','150224_4752450_GPCHCX.txt','150224_4752450_GPGGA.txt'
};
%% ========================================================================
%% 4. 读取并处理830
% ========================================================================
Ref830 = cell(2,1);
fprintf('\n============================================================\n');
fprintf('2. 开始处理830数据\n');
fprintf('============================================================\n');
for k = 1:2
    fprintf('\n============================================================\n');
    fprintf('%s\n',Compass{k}.name);
    fprintf('============================================================\n');
    % 4.1 读取全部830三件套
    IMU_BINARY_all = [];
    GPCHCX_all = [];
    GPGGA_all = [];
    for j = 1:size(REF_FILE{k},1)
        file_binary = fullfile(raw_dir,REF_FILE{k}{j,1});
        file_gpchcx = fullfile(raw_dir,REF_FILE{k}{j,2});
        file_gpgga = fullfile(raw_dir,REF_FILE{k}{j,3});
        fprintf('\n读取第%d组：\n',j);
        fprintf('  %s\n',REF_FILE{k}{j,1});
        fprintf('  %s\n',REF_FILE{k}{j,2});
        fprintf('  %s\n',REF_FILE{k}{j,3});
        if ~exist(file_binary,'file')
            error('文件不存在：%s',file_binary);
        end
        if ~exist(file_gpchcx,'file')
            error('文件不存在：%s',file_gpchcx);
        end
        if ~exist(file_gpgga,'file')
            error('文件不存在：%s',file_gpgga);
        end
        tic;
        [IMU_tmp,GPCHCX_tmp,GPGGA_tmp] = read_mems_ins(file_binary,file_gpchcx,file_gpgga);
        fprintf('  读取完成：%.2f s\n',toc);
        fprintf('  IMU：%d × %d\n',size(IMU_tmp,1),size(IMU_tmp,2));
        fprintf('  GPCHCX：%d × %d\n',size(GPCHCX_tmp,1),size(GPCHCX_tmp,2));
        fprintf('  GPGGA：%d × %d\n',size(GPGGA_tmp,1),size(GPGGA_tmp,2));
        IMU_BINARY_all = [IMU_BINARY_all;IMU_tmp];
        if isempty(GPCHCX_all)
            GPCHCX_all = GPCHCX_tmp;
        else
            if size(GPCHCX_all,1) ~= size(GPCHCX_tmp,1)
                error('不同GPCHCX文件行数不一致。');
            end
            GPCHCX_all = [GPCHCX_all,GPCHCX_tmp];
        end
        if isempty(GPGGA_all)
            GPGGA_all = GPGGA_tmp;
        else
            if size(GPGGA_all,1) ~= size(GPGGA_tmp,1)
                error('不同GPGGA文件行数不一致。');
            end
            GPGGA_all = [GPGGA_all,GPGGA_tmp];
        end
    end
    fprintf('\n合并后：\n');
    fprintf('  binary IMU：%d × %d\n',size(IMU_BINARY_all,1),size(IMU_BINARY_all,2));
    fprintf('  GPCHCX：%d × %d\n',size(GPCHCX_all,1),size(GPCHCX_all,2));
    fprintf('  GPGGA：%d × %d\n',size(GPGGA_all,1),size(GPGGA_all,2));
    % 4.2 GPCHCX清理
    if size(GPCHCX_all,1) < 21
        error('GPCHCX行数不足21。');
    end
    state_valid = GPCHCX_all(21,:) ~= 0;
    lat_valid = GPCHCX_all(12,:) >= 36 & GPCHCX_all(12,:) <= 37;
    time_valid = isfinite(GPCHCX_all(2,:)) & GPCHCX_all(2,:) > 100;
    valid = state_valid & lat_valid & time_valid;
    fprintf('\nGPCHCX筛选：\n');
    fprintf('  原始：%d\n',size(GPCHCX_all,2));
    fprintf('  状态无效：%d\n',sum(~state_valid));
    fprintf('  纬度异常：%d\n',sum(~lat_valid));
    fprintf('  时间无效：%d\n',sum(~time_valid));
    GPCHCX_valid = GPCHCX_all(:,valid);
    if isempty(GPCHCX_valid)
        error('%s：GPCHCX清理后为空。',Compass{k}.name);
    end
    [~,idx_sort] = sort(GPCHCX_valid(2,:));
    GPCHCX_valid = GPCHCX_valid(:,idx_sort);
    [~,idx_unique] = unique(GPCHCX_valid(2,:),'stable');
    GPCHCX_valid = GPCHCX_valid(:,idx_unique);
    fprintf('  最终有效：%d\n',size(GPCHCX_valid,2));
    fprintf('  GPS：%.3f ~ %.3f\n',GPCHCX_valid(2,1),GPCHCX_valid(2,end));
    % 4.3 binary IMU清理
    if size(IMU_BINARY_all,2) < 7
        error('binary IMU不足7列。');
    end
    valid_binary = isfinite(IMU_BINARY_all(:,7)) & IMU_BINARY_all(:,7) > 100;
    IMU_BINARY_valid = IMU_BINARY_all(valid_binary,:);
    [~,idx_sort] = sort(IMU_BINARY_valid(:,7));
    IMU_BINARY_valid = IMU_BINARY_valid(idx_sort,:);
    [~,idx_unique] = unique(IMU_BINARY_valid(:,7),'stable');
    IMU_BINARY_valid = IMU_BINARY_valid(idx_unique,:);
    fprintf('\nbinary IMU有效时间：%.3f ~ %.3f，%d历元\n',IMU_BINARY_valid(1,7),IMU_BINARY_valid(end,7),size(IMU_BINARY_valid,1));
    % 4.4 GPGGA构造GPS周秒、排序、去重
    GPGGA_time = [];
    if ~isempty(GPGGA_all)
        GPGGA_time = gpgga_to_gps_time(GPGGA_all,DATA_DATE,GPS_UTC_LEAP);
        valid_gpgga_time = isfinite(GPGGA_time);
        GPGGA_all = GPGGA_all(:,valid_gpgga_time);
        GPGGA_time = GPGGA_time(valid_gpgga_time);
        [GPGGA_time,idx_sort] = sort(GPGGA_time);
        GPGGA_all = GPGGA_all(:,idx_sort);
        [GPGGA_time,idx_unique] = unique(GPGGA_time,'stable');
        GPGGA_all = GPGGA_all(:,idx_unique);
        fprintf('GPGGA有效时间：%.3f ~ %.3f，%d历元\n',GPGGA_time(1),GPGGA_time(end),numel(GPGGA_time));
    end
    % 4.5 根据120 IMU时间窗口切静止段 / 导航段
    t_static_start = Window(k).static_start;
    t_nav_start = Window(k).nav_start;
    t_nav_end = Window(k).nav_end;
    idx_gpchcx_static = GPCHCX_valid(2,:) >= t_static_start & GPCHCX_valid(2,:) < t_nav_start;
    idx_gpchcx_nav = GPCHCX_valid(2,:) >= t_nav_start & GPCHCX_valid(2,:) <= t_nav_end;
    GPCHCX_static = GPCHCX_valid(:,idx_gpchcx_static);
    GPCHCX_nav = GPCHCX_valid(:,idx_gpchcx_nav);
    idx_binary_static = IMU_BINARY_valid(:,7) >= t_static_start & IMU_BINARY_valid(:,7) < t_nav_start;
    idx_binary_nav = IMU_BINARY_valid(:,7) >= t_nav_start & IMU_BINARY_valid(:,7) <= t_nav_end;
    IMU_BINARY_static = IMU_BINARY_valid(idx_binary_static,:);
    IMU_BINARY_nav = IMU_BINARY_valid(idx_binary_nav,:);
    GPGGA_static = [];
    GPGGA_nav = [];
    GPGGA_time_static = [];
    GPGGA_time_nav = [];
    if ~isempty(GPGGA_time)
        id_static = GPGGA_time >= t_static_start & GPGGA_time < t_nav_start;
        id_nav = GPGGA_time >= t_nav_start & GPGGA_time <= t_nav_end;
        GPGGA_static = GPGGA_all(:,id_static);
        GPGGA_nav = GPGGA_all(:,id_nav);
        GPGGA_time_static = GPGGA_time(id_static);
        GPGGA_time_nav = GPGGA_time(id_nav);
    end
    fprintf('\n830最终截取窗口：\n');
    fprintf('  静止：%.3f ~ %.3f\n',t_static_start,t_nav_start);
    fprintf('    GPCHCX：%d\n',size(GPCHCX_static,2));
    fprintf('    binary：%d\n',size(IMU_BINARY_static,1));
    fprintf('    GPGGA： %d\n',size(GPGGA_static,2));
    fprintf('  导航：%.3f ~ %.3f\n',t_nav_start,t_nav_end);
    fprintf('    GPCHCX：%d\n',size(GPCHCX_nav,2));
    fprintf('    binary：%d\n',size(IMU_BINARY_nav,1));
    fprintf('    GPGGA： %d\n',size(GPGGA_nav,2));
    if isempty(GPCHCX_nav)
        error('%s：导航段GPCHCX为空。',Compass{k}.name);
    end
    if isempty(IMU_BINARY_nav)
        warning('%s：导航段binary IMU为空。',Compass{k}.name);
    end
    % 4.6 静止GPCHCX -> PVA + GPCHCX-derived IMU
    imu_gpchcx_static_raw = [];
    pva_830_static = [];
    imu_830_static = [];
    IMU_GPCHCX_FRD_static = [];
    if ~isempty(GPCHCX_static)
        [imu_gpchcx_static_raw,pva_830_static] = visualize_gpchcx_navigation_results(GPCHCX_static);
        imu_830_static = imuRFU2FRD(imu_gpchcx_static_raw);
        IMU_GPCHCX_FRD_static = convert_gpchcx_imu_frd(imu_gpchcx_static_raw);
    end
    % 4.7 导航GPCHCX -> PVA + GPCHCX-derived IMU
    [imu_gpchcx_nav_raw,pva_830_nav] = visualize_gpchcx_navigation_results(GPCHCX_nav);
    imu_830_nav = imuRFU2FRD(imu_gpchcx_nav_raw);
    IMU_GPCHCX_FRD_nav = convert_gpchcx_imu_frd(imu_gpchcx_nav_raw);
    % 4.8 binary IMU -> FRD
    IMU_BINARY_FRD_static = convert_binary_imu_frd(IMU_BINARY_static);
    IMU_BINARY_FRD_nav = convert_binary_imu_frd(IMU_BINARY_nav);
    % 4.9 GPGGA -> GPS输出格式
    GPS_830_static = convert_gpgga_to_gps(GPGGA_static,GPGGA_time_static,t_static_start);
    GPS_830_nav = convert_gpgga_to_gps(GPGGA_nav,GPGGA_time_nav,t_nav_start);
    % 4.10 保存结构
    Ref830{k}.name = Compass{k}.name;
    Ref830{k}.static_time_range = [t_static_start,t_nav_start];
    Ref830{k}.nav_time_range = [t_nav_start,t_nav_end];
    Ref830{k}.time_range = Ref830{k}.nav_time_range;
    Ref830{k}.compass_duration = Window(k).compass_duration;
    Ref830{k}.window_source = 'static/nav range follows 120 IMU; Compass only used for experiment identification/check';
    Ref830{k}.gpchcx_static = GPCHCX_static;
    Ref830{k}.gpgga_static = GPGGA_static;
    Ref830{k}.gpgga_time_static = GPGGA_time_static;
    Ref830{k}.gps_static = GPS_830_static;
    Ref830{k}.imu_binary_raw_static = IMU_BINARY_static;
    Ref830{k}.imu_binary_frd_static = IMU_BINARY_FRD_static;
    Ref830{k}.imu_gpchcx_raw_static = imu_gpchcx_static_raw;
    Ref830{k}.imu_gpchcx_frd_static = IMU_GPCHCX_FRD_static;
    Ref830{k}.imu_static = imu_830_static;
    Ref830{k}.pva_static = pva_830_static;
    Ref830{k}.gpchcx_nav = GPCHCX_nav;
    Ref830{k}.gpgga_nav = GPGGA_nav;
    Ref830{k}.gpgga_time_nav = GPGGA_time_nav;
    Ref830{k}.gps_nav = GPS_830_nav;
    Ref830{k}.imu_binary_raw_nav = IMU_BINARY_nav;
    Ref830{k}.imu_binary_frd_nav = IMU_BINARY_FRD_nav;
    Ref830{k}.imu_gpchcx_raw_nav = imu_gpchcx_nav_raw;
    Ref830{k}.imu_gpchcx_frd_nav = IMU_GPCHCX_FRD_nav;
    Ref830{k}.imu_nav = imu_830_nav;
    Ref830{k}.pva_nav = pva_830_nav;
    Ref830{k}.gpchcx = GPCHCX_nav;
    Ref830{k}.gpgga = GPGGA_nav;
    Ref830{k}.imu_binary_raw = IMU_BINARY_nav;
    Ref830{k}.imu_gpchcx_raw = imu_gpchcx_nav_raw;
    Ref830{k}.imu_binary_frd = IMU_BINARY_FRD_nav;
    Ref830{k}.imu_gpchcx_frd = IMU_GPCHCX_FRD_nav;
    Ref830{k}.imu = imu_830_nav;
    Ref830{k}.pva = pva_830_nav;
    if ~isempty(pva_830_nav)
        Ref830{k}.time = pva_830_nav(:,2)';
        Ref830{k}.heading = pva_830_nav(:,11)';
    else
        Ref830{k}.time = [];
        Ref830{k}.heading = [];
    end
    fprintf('\n最终830：\n');
    if ~isempty(pva_830_static)
        fprintf('  静止PVA：%d历元，%.3f ~ %.3f\n',size(pva_830_static,1),pva_830_static(1,2),pva_830_static(end,2));
    end
    fprintf('  导航PVA：%d历元，%.3f ~ %.3f\n',size(pva_830_nav,1),pva_830_nav(1,2),pva_830_nav(end,2));
end
%% ========================================================================
%% 5. 时间覆盖图
% ========================================================================
figure('Name','120 / Compass / 830时间窗口','NumberTitle','off');
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
for k = 1:2
    nexttile;
    hold on;
    grid on;
    t0 = Window(k).static_start;
    plot([Window(k).static_start,Window(k).static_end]-t0,[5 5],'LineWidth',6);
    plot([Window(k).nav_start,Window(k).nav_end]-t0,[4 4],'LineWidth',6);
    if ~isempty(Ref830{k}.gpchcx_static)
        plot([Ref830{k}.gpchcx_static(2,1),Ref830{k}.gpchcx_static(2,end)]-t0,[3 3],'LineWidth',6);
    end
    if ~isempty(Ref830{k}.gpchcx_nav)
        plot([Ref830{k}.gpchcx_nav(2,1),Ref830{k}.gpchcx_nav(2,end)]-t0,[2 2],'LineWidth',6);
    end
    if ~isempty(Ref830{k}.imu_binary_raw_nav)
        plot([Ref830{k}.imu_binary_raw_nav(1,7),Ref830{k}.imu_binary_raw_nav(end,7)]-t0,[1 1],'LineWidth',6);
    end
    xline(Window(k).nav_start-t0,'--','120对准后起点');
    yticks(1:5);
    yticklabels({'830 binary导航','830 GPCHCX导航','830 GPCHCX静止','目标导航窗口','目标静止窗口'});
    xlabel('相对120静止IMU起点 / s');
    title(sprintf('%s：830截取窗口',Compass{k}.name));
end
%% ========================================================================
%% 6. 导航段 binary IMU 与 GPCHCX-derived IMU比较
% ========================================================================
for k = 1:2
    imu1 = Ref830{k}.imu_binary_frd_nav;
    imu2 = Ref830{k}.imu_gpchcx_frd_nav;
    if isempty(imu1) || isempty(imu2)
        continue;
    end
    t0 = max(imu1(1,1),imu2(1,1));
    t1 = min(imu1(end,1),imu2(end,1));
    if t1 <= t0
        continue;
    end
    id1 = imu1(:,1) >= t0 & imu1(:,1) <= t1;
    id2 = imu2(:,1) >= t0 & imu2(:,1) <= t1;
    figure('Name',sprintf('%s 830导航段 binary-GPCHCX IMU',Ref830{k}.name),'NumberTitle','off');
    tiledlayout(3,2,'TileSpacing','compact','Padding','compact');
    labels = {'Gyro F','Gyro R','Gyro D','Acc F','Acc R','Acc D'};
    for j = 1:6
        nexttile;
        hold on;
        grid on;
        plot(imu2(id2,1)-t0,imu2(id2,j+1),'LineWidth',0.8);
        plot(imu1(id1,1)-t0,imu1(id1,j+1),'LineWidth',0.8);
        ylabel(labels{j});
        if j == 1
            legend('GPCHCX','binary','Location','best');
        end
        if j >= 5
            xlabel('导航段共同运行时间 / s');
        end
    end
    sgtitle(sprintf('%s：830导航段 binary 与 GPCHCX IMU',Ref830{k}.name));
end
%% ========================================================================
%% 7. 830导航段航向
% ========================================================================
figure('Name','830导航段参考航向','NumberTitle','off');
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
for k = 1:2
    nexttile;
    if isempty(Ref830{k}.time)
        continue;
    end
    t = Ref830{k}.time-Ref830{k}.time(1);
    plot(t,Ref830{k}.heading,'LineWidth',1);
    grid on;
    xlabel('导航运行时间 / s');
    ylabel('830航向 / °');
    title(sprintf('%s 830导航数据',Ref830{k}.name));
end
%% ========================================================================
%% 8. 保存
% ========================================================================
save(save_file,'Ref830','REF_FILE','Window','-v7.3');
fprintf('\n============================================================\n');
fprintf('830静止段 + 导航段数据保存完成\n');
fprintf('%s\n',save_file);
fprintf('============================================================\n');
%% ========================================================================
% 局部函数
% ========================================================================
function IMU_FRD = convert_binary_imu_frd(IMU_RAW)
IMU_FRD = [];
if isempty(IMU_RAW)
    return;
end
imu_rfu = zeros(size(IMU_RAW,1),7);
imu_rfu(:,1:3) = IMU_RAW(:,1:3)/180*pi;
imu_rfu(:,4:6) = IMU_RAW(:,4:6)*9.806;
imu_rfu(:,[2,5]) = -imu_rfu(:,[2,5]);
imu_rfu(:,7) = IMU_RAW(:,7);
IMU_FRD = imuRFU2FRD(imu_rfu);
end
function IMU_FRD = convert_gpchcx_imu_frd(IMU_RAW)
IMU_FRD = [];
if isempty(IMU_RAW)
    return;
end
imu_rfu = IMU_RAW;
imu_rfu(:,1:3) = imu_rfu(:,1:3)/180*pi*0.01;
imu_rfu(:,4:6) = imu_rfu(:,4:6)*9.806*0.01;
IMU_FRD = imuRFU2FRD(imu_rfu);
end
function gps_time = gpgga_to_gps_time(gpgga,data_date,leap_sec)
gps_time = [];
if isempty(gpgga) || size(gpgga,1) < 3
    return;
end
[~,base_sec] = bjt_to_utc_week_seconds(data_date(1),data_date(2),data_date(3),gpgga(1,1)+8,gpgga(2,1),gpgga(3,1),0);
total_s = gpgga(1,:)*3600+gpgga(2,:)*60+gpgga(3,:);
gps_time = (total_s-total_s(1))+base_sec+leap_sec;
end
function gps = convert_gpgga_to_gps(gpgga,gps_time,t0)
gps = [];
if isempty(gpgga) || isempty(gps_time) || size(gpgga,1) < 15
    return;
end
fix_valid = gpgga(10,:) == 4;
gpgga = gpgga(:,fix_valid);
gps_time = gps_time(fix_valid);
if isempty(gpgga)
    return;
end
gps_sub = gpgga';
lat = gps_sub(:,4)+gps_sub(:,5)/60;
lon = gps_sub(:,7)+gps_sub(:,8)/60;
alt = gps_sub(:,13)+gps_sub(:,15);
gps = [gps_time(:),gps_time(:)-t0,lat,lon,alt,gps_sub(:,10)];
end