clear;
clc;
close all;
%% ========================================================================
% 2025-12-07
% 120 AUAX / 830参考系统 / 罗盘 航向角对比
%
% 试验1：下午
%   120 AUAX : Disk1_003.dat
%   830      : 043941_4752450.dat + 083942_4752450.dat
%   Compass  : 2025_12_07_12_56_09_连续.txt
%
% 试验2：晚上
%   120 AUAX : Disk1_006.dat
%   830      : 110223_4752450.dat + 150224_4752450.dat
%   Compass  : 2025_12_07_19_31_38_连续.txt
%
%
% 核心处理：
%
% 120 AUAX
%     ↓
% 导航状态 0x4300
%     ↓
% 航向角
%
% 830 GPCHCX
%     ↓
% 有效定位定向状态
%     ↓
% PVA
%     ↓
% 830参考航向
%
% Compass
%     ↓
% 北京时间 → GPS周秒
%     ↓
% 航向角
%
% 三者统一到 830 时间轴
%     ↓
% AUAX - 830
% Compass - 830
% Compass - AUAX
%
% ========================================================================
%% ========================================================================
%% 0. 参数
% ========================================================================
root_dir = ...
    'D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1207-longtime\raw';
data120_dir = root_dir;
data830_dir = root_dir;
compass_dir = root_dir;
% -------------------------------------------------------------------------
% GPS 周
% 2025-12-07 对应原程序中的 2396
% -------------------------------------------------------------------------
GPS_WEEK = 2396;
% -------------------------------------------------------------------------
% 120 AUAX 时间修正
%
% 原程序：
% AUAX{i}(2,:) = AUAX{i}(2,:) + 18;
% -------------------------------------------------------------------------
AUAX_TIME_OFFSET = 18;
% -------------------------------------------------------------------------
% 罗盘时间修正
%
% 原程序最终：
% compass_time = converted_time - 21
% -------------------------------------------------------------------------
COMPASS_POST_OFFSET = -21;
% -------------------------------------------------------------------------
% AUAX 状态
% -------------------------------------------------------------------------
STATUS_FINE = hex2dec('5820');
STATUS_NAV = hex2dec('4300');
% -------------------------------------------------------------------------
% 830 数据有效纬度范围
%
% 保持原程序：
%
% 36 <= latitude <= 37
% -------------------------------------------------------------------------
REF_LAT_MIN = 36;
REF_LAT_MAX = 37;
% -------------------------------------------------------------------------
% 是否保存最终结果
% -------------------------------------------------------------------------
SAVE_RESULT = true;
%% ========================================================================
%% 1. 两次实验文件配置
% ========================================================================
EXP = struct();
%% ------------------------------------------------------------------------
% 实验 1：下午
% -------------------------------------------------------------------------
EXP(1).name = '下午实验';
EXP(1).auax_file = ...
    'Disk1_003.dat';
EXP(1).ref830_files = {
    '043941_4752450.dat'
    '083942_4752450.dat'
    };
EXP(1).compass_file = ...
    '2025_12_07_12_56_09_连续.txt';
% 原程序 bjt_to_utc_week_seconds 最后一个参数
EXP(1).compass_convert_param = 18;
%% ------------------------------------------------------------------------
% 实验 2：晚上
% -------------------------------------------------------------------------
EXP(2).name = '晚上实验';
EXP(2).auax_file = ...
    'Disk1_006.dat';
EXP(2).ref830_files = {
    '110223_4752450.dat'
    '150224_4752450.dat'
    };
EXP(2).compass_file = ...
    '2025_12_07_19_31_38_连续.txt';
EXP(2).compass_convert_param = 0;
%% ========================================================================
%% 2. 数据读取
% ========================================================================
Data = struct();
fprintf('\n');
fprintf('============================================================\n');
fprintf('开始读取 2025-12-07 航向对比数据\n');
fprintf('============================================================\n');
for k = 1:length(EXP)
    fprintf('\n');
    fprintf('============================================================\n');
    fprintf('实验 %d：%s\n',k,EXP(k).name);
    fprintf('============================================================\n');
    %% --------------------------------------------------------------------
    % 2.1 120 AUAX
    % ---------------------------------------------------------------------
    auax_filename = ...
        fullfile(data120_dir,EXP(k).auax_file);
    fprintf('\n[1] 读取 120 AUAX\n');
    fprintf('%s\n',auax_filename);
    Data(k).auax = ...
        read_prepare_auax( ...
        auax_filename, ...
        AUAX_TIME_OFFSET, ...
        STATUS_FINE, ...
        STATUS_NAV);
    %% --------------------------------------------------------------------
    % 2.2 830参考系统
    % ---------------------------------------------------------------------
    fprintf('\n[2] 读取 830 参考系统\n');
    ref_files = cell(size(EXP(k).ref830_files));
    for j = 1:length(EXP(k).ref830_files)
        ref_files{j} = ...
            fullfile( ...
            data830_dir, ...
            EXP(k).ref830_files{j});
        fprintf('%s\n',ref_files{j});
    end
    Data(k).ref830 = ...
        read_prepare_830( ...
        ref_files, ...
        REF_LAT_MIN, ...
        REF_LAT_MAX);
    %% --------------------------------------------------------------------
    % 2.3 罗盘
    % ---------------------------------------------------------------------
    compass_filename = ...
        fullfile( ...
        compass_dir, ...
        EXP(k).compass_file);
    fprintf('\n[3] 读取罗盘\n');
    fprintf('%s\n',compass_filename);
    Data(k).compass = ...
        read_prepare_compass( ...
        compass_filename, ...
        2025,12,7, ...
        EXP(k).compass_convert_param, ...
        COMPASS_POST_OFFSET);
end
%% ========================================================================
%% 3. 打印三套数据时间范围
% ========================================================================
fprintf('\n');
fprintf('============================================================\n');
fprintf('三套数据时间覆盖\n');
fprintf('============================================================\n');
for k = 1:length(EXP)
    A = Data(k).auax;
    R = Data(k).ref830;
    C = Data(k).compass;
    fprintf('\n');
    fprintf('-------------------- 实验 %d：%s --------------------\n', ...
        k,EXP(k).name);
    fprintf( ...
        '120 AUAX : %.3f ~ %.3f，持续 %.3f s\n', ...
        A.time(1), ...
        A.time(end), ...
        A.time(end)-A.time(1));
    fprintf( ...
        '830 REF  : %.3f ~ %.3f，持续 %.3f s\n', ...
        R.time(1), ...
        R.time(end), ...
        R.time(end)-R.time(1));
    fprintf( ...
        'Compass  : %.3f ~ %.3f，持续 %.3f s\n', ...
        C.time(1), ...
        C.time(end), ...
        C.time(end)-C.time(1));
    % ---------------------------------------------------------------------
    % 三者公共时间区间
    % ---------------------------------------------------------------------
    common_start = max([ ...
        A.time(1), ...
        R.time(1), ...
        C.time(1)]);
    common_end = min([ ...
        A.time(end), ...
        R.time(end), ...
        C.time(end)]);
    if common_end <= common_start
        fprintf('警告：三套数据没有公共时间区间！\n');
    else
        fprintf( ...
            '公共区间 : %.3f ~ %.3f，持续 %.3f s\n', ...
            common_start, ...
            common_end, ...
            common_end-common_start);
    end
end
%% ========================================================================
%% 4. 以 830 时间为统一参考时间轴
% ========================================================================
Results = struct();
for k = 1:length(EXP)
    A = Data(k).auax;
    R = Data(k).ref830;
    C = Data(k).compass;
    %% --------------------------------------------------------------------
    % 三者公共时间
    % ---------------------------------------------------------------------
    common_start = max([ ...
        A.time(1), ...
        R.time(1), ...
        C.time(1)]);
    common_end = min([ ...
        A.time(end), ...
        R.time(end), ...
        C.time(end)]);
    if common_end <= common_start
        warning('实验 %d 没有三者公共时间范围。',k);
        continue;
    end
    %% --------------------------------------------------------------------
    % 选择公共区间中的 830 时刻
    % ---------------------------------------------------------------------
    idx_ref = ...
        R.time >= common_start & ...
        R.time <= common_end;
    t = R.time(idx_ref);
    heading830 = ...
        R.heading(idx_ref);
    %% --------------------------------------------------------------------
    % AUAX → 830 时间轴
    %
    % 使用圆周角插值
    % 不直接 interp1(angle)
    % ---------------------------------------------------------------------
    heading120 = ...
        interp_heading( ...
        A.time, ...
        A.heading, ...
        t);
    %% --------------------------------------------------------------------
    % Compass → 830 时间轴
    % ---------------------------------------------------------------------
    headingCompass = ...
        interp_heading( ...
        C.time, ...
        C.heading, ...
        t);
    %% --------------------------------------------------------------------
    % 最终公共有效点
    % ---------------------------------------------------------------------
    valid = ...
        isfinite(t) & ...
        isfinite(heading830) & ...
        isfinite(heading120) & ...
        isfinite(headingCompass);
    t = t(valid);
    heading830 = heading830(valid);
    heading120 = heading120(valid);
    headingCompass = headingCompass(valid);
    %% --------------------------------------------------------------------
    % 航向误差
    %
    % 全部限制到 [-180,180)
    % ---------------------------------------------------------------------
    error120_830 = ...
        wrap180(heading120-heading830);
    errorCompass_830 = ...
        wrap180(headingCompass-heading830);
    errorCompass_120 = ...
        wrap180(headingCompass-heading120);
    %% --------------------------------------------------------------------
    % 保存
    % ---------------------------------------------------------------------
    Results(k).name = EXP(k).name;
    Results(k).time = t;
    Results(k).time_relative_auax = ...
        t-A.time(1);
    Results(k).heading830 = heading830;
    Results(k).heading120 = heading120;
    Results(k).headingCompass = headingCompass;
    Results(k).error120_830 = ...
        error120_830;
    Results(k).errorCompass_830 = ...
        errorCompass_830;
    Results(k).errorCompass_120 = ...
        errorCompass_120;
    %% --------------------------------------------------------------------
    % 统计
    % ---------------------------------------------------------------------
    Results(k).stats120 = ...
        calculate_error_statistics(error120_830);
    Results(k).statsCompass = ...
        calculate_error_statistics(errorCompass_830);
    Results(k).statsCompass120 = ...
        calculate_error_statistics(errorCompass_120);
end
%% ========================================================================
%% 5. 打印误差统计
% ========================================================================
fprintf('\n');
fprintf('============================================================\n');
fprintf('航向误差统计\n');
fprintf('============================================================\n');
for k = 1:length(Results)
    if ~isfield(Results(k),'time') || isempty(Results(k).time)
        continue;
    end
    fprintf('\n');
    fprintf('-------------------- 实验 %d：%s --------------------\n', ...
        k,Results(k).name);
    fprintf('公共有效历元：%d\n', ...
        length(Results(k).time));
    %% AUAX vs 830
    s = Results(k).stats120;
    fprintf('\n120 AUAX - 830：\n');
    fprintf('  Mean = %8.4f deg\n',s.mean);
    fprintf('  STD  = %8.4f deg\n',s.std);
    fprintf('  RMS  = %8.4f deg\n',s.rms);
    fprintf('  MAE  = %8.4f deg\n',s.mae);
    fprintf('  Max  = %8.4f deg\n',s.maxabs);
    %% Compass vs 830
    s = Results(k).statsCompass;
    fprintf('\nCompass - 830：\n');
    fprintf('  Mean = %8.4f deg\n',s.mean);
    fprintf('  STD  = %8.4f deg\n',s.std);
    fprintf('  RMS  = %8.4f deg\n',s.rms);
    fprintf('  MAE  = %8.4f deg\n',s.mae);
    fprintf('  Max  = %8.4f deg\n',s.maxabs);
    %% Compass vs AUAX
    s = Results(k).statsCompass120;
    fprintf('\nCompass - 120 AUAX：\n');
    fprintf('  Mean = %8.4f deg\n',s.mean);
    fprintf('  STD  = %8.4f deg\n',s.std);
    fprintf('  RMS  = %8.4f deg\n',s.rms);
    fprintf('  MAE  = %8.4f deg\n',s.mae);
    fprintf('  Max  = %8.4f deg\n',s.maxabs);
end
%% ========================================================================
%% 6. 时间覆盖关系
% ========================================================================
for k = 1:length(EXP)
    A = Data(k).auax;
    R = Data(k).ref830;
    C = Data(k).compass;
    % 统一以 AUAX 开始时刻为 0
    t0 = A.time(1);
    figure( ...
        'Name',sprintf('实验%d 数据时间覆盖',k), ...
        'NumberTitle','off');
    hold on;
    grid on;
    plot( ...
        [A.time(1),A.time(end)]-t0, ...
        [3,3], ...
        'LineWidth',7);
    plot( ...
        [R.time(1),R.time(end)]-t0, ...
        [2,2], ...
        'LineWidth',7);
    plot( ...
        [C.time(1),C.time(end)]-t0, ...
        [1,1], ...
        'LineWidth',7);
    yticks([1 2 3]);
    yticklabels({ ...
        'Compass', ...
        '830参考', ...
        '120 AUAX'});
    xlabel('相对 AUAX 开始时间 / s');
    title(sprintf( ...
        '实验 %d：%s 数据覆盖', ...
        k,EXP(k).name));
    ylim([0.5 3.5]);
end
%% ========================================================================
%% 7. 三套航向原始时间对比
%
% 横坐标统一：
%
% AUAX实验开始时刻 = 0 s
%
% 因此不会再出现：
% “罗盘开始时刻被错误当成0”
% ========================================================================
for k = 1:length(EXP)
    A = Data(k).auax;
    R = Data(k).ref830;
    C = Data(k).compass;
    t0 = A.time(1);
    figure( ...
        'Name',sprintf('实验%d 三套航向原始对比',k), ...
        'NumberTitle','off');
    hold on;
    grid on;
    plot( ...
        A.time-t0, ...
        A.heading, ...
        'LineWidth',1);
    plot( ...
        R.time-t0, ...
        R.heading, ...
        'LineWidth',1);
    plot( ...
        C.time-t0, ...
        C.heading, ...
        'LineWidth',1);
    xlabel('AUAX 运行时间 / s');
    ylabel('航向角 / °');
    title(sprintf( ...
        '实验 %d：%s', ...
        k,EXP(k).name));
    legend( ...
        '120 AUAX', ...
        '830参考', ...
        '罗盘', ...
        'Location','best');
    ylim([0 360]);
    % 以 AUAX 整个实验时长为横轴
    xlim([ ...
        0, ...
        A.time(end)-A.time(1)]);
end
%% ========================================================================
%% 8. 统一时间后的航向与误差
% ========================================================================
for k = 1:length(Results)
    if ~isfield(Results(k),'time') || isempty(Results(k).time)
        continue;
    end
    R = Results(k);
    t = R.time_relative_auax;
    figure( ...
        'Name',sprintf('实验%d 航向误差',k), ...
        'NumberTitle','off');
    tiledlayout( ...
        3,1, ...
        'TileSpacing','compact', ...
        'Padding','compact');
    %% --------------------------------------------------------------------
    % 航向
    % ---------------------------------------------------------------------
    nexttile;
    hold on;
    grid on;
    plot( ...
        t, ...
        R.heading830, ...
        'LineWidth',1.1);
    plot( ...
        t, ...
        R.heading120, ...
        'LineWidth',1);
    plot( ...
        t, ...
        R.headingCompass, ...
        'LineWidth',1);
    ylabel('航向角 / °');
    title(sprintf( ...
        '实验 %d：三套航向共同有效区间',k));
    legend( ...
        '830参考', ...
        '120 AUAX', ...
        '罗盘', ...
        'Location','best');
    ylim([0 360]);
    %% --------------------------------------------------------------------
    % AUAX和Compass分别相对830
    % ---------------------------------------------------------------------
    nexttile;
    hold on;
    grid on;
    plot( ...
        t, ...
        R.error120_830, ...
        'LineWidth',1);
    plot( ...
        t, ...
        R.errorCompass_830, ...
        'LineWidth',1);
    yline(0,'--');
    ylabel('\Delta\psi / °');
    legend( ...
        '120 AUAX - 830', ...
        '罗盘 - 830', ...
        'Location','best');
    title('相对于830参考系统的航向误差');
    %% --------------------------------------------------------------------
    % Compass - AUAX
    % ---------------------------------------------------------------------
    nexttile;
    hold on;
    grid on;
    plot( ...
        t, ...
        R.errorCompass_120, ...
        'LineWidth',1);
    yline(0,'--');
    xlabel('AUAX 运行时间 / s');
    ylabel('\Delta\psi / °');
    title('罗盘 - 120 AUAX');
end
%% ========================================================================
%% 9. 航向相关误差
%
% 这张图后续用于研究：
%
% 罗盘误差是否随真实航向呈现周期变化
%
% 此时横轴采用830参考航向，而不是AUAX航向。
% ========================================================================
for k = 1:length(Results)
    if ~isfield(Results(k),'time') || isempty(Results(k).time)
        continue;
    end
    R = Results(k);
    figure( ...
        'Name',sprintf('实验%d 航向相关误差',k), ...
        'NumberTitle','off');
    tiledlayout(2,1, ...
        'TileSpacing','compact', ...
        'Padding','compact');
    %% Compass - 830
    nexttile;
    plot( ...
        R.heading830, ...
        R.errorCompass_830, ...
        '.', ...
        'MarkerSize',5);
    grid on;
    xlim([0 360]);
    xlabel('830参考航向 / °');
    ylabel('罗盘 - 830 / °');
    title(sprintf( ...
        '实验 %d：罗盘航向相关误差',k));
    %% AUAX - 830
    nexttile;
    plot( ...
        R.heading830, ...
        R.error120_830, ...
        '.', ...
        'MarkerSize',5);
    grid on;
    xlim([0 360]);
    xlabel('830参考航向 / °');
    ylabel('120 AUAX - 830 / °');
    title('120 AUAX 航向误差');
end
%% ========================================================================
%% 10. 保存
% ========================================================================
if SAVE_RESULT
    save_filename = ...
        fullfile( ...
        root_dir, ...
        'heading_compare_120_830_compass.mat');
    save( ...
        save_filename, ...
        'Data', ...
        'Results', ...
        'EXP');
    fprintf('\n结果已保存：\n%s\n',save_filename);
end
%% ========================================================================
%% 局部函数
% ========================================================================
function data = read_prepare_auax( ...
    filename, ...
    time_offset, ...
    STATUS_FINE, ...
    STATUS_NAV)
%READ_PREPARE_AUAX
%
% 读取120 AUAX，并提取有效导航阶段
%
% 输出：
%
% data.time
% data.heading
% data.raw
    AUAX = read_auax_120(filename);
    %% --------------------------------------------------------------------
    % 时间修正
    % ---------------------------------------------------------------------
    AUAX(2,:) = ...
        AUAX(2,:) + time_offset;
    %% --------------------------------------------------------------------
    % 删除明显错误时间
    % ---------------------------------------------------------------------
    valid_time = ...
        isfinite(AUAX(2,:)) & ...
        AUAX(2,:) > 100;
    AUAX = AUAX(:,valid_time);
    %% --------------------------------------------------------------------
    % 状态
    % ---------------------------------------------------------------------
    status_row = size(AUAX,1)-2;
    status = AUAX(status_row,:);
    idx_fine = ...
        find(status == STATUS_FINE);
    idx_nav = ...
        find(status == STATUS_NAV);
    if isempty(idx_nav)
        error( ...
            'AUAX文件中没有找到导航状态0x4300：%s', ...
            filename);
    end
    %% --------------------------------------------------------------------
    % 如果存在精对准：
    %
    % 仅取最后一次精对准结束后的导航数据
    % ---------------------------------------------------------------------
    if ~isempty(idx_fine)
        last_fine = idx_fine(end);
        idx_nav = ...
            idx_nav(idx_nav > last_fine);
    end
    if isempty(idx_nav)
        error( ...
            '最后一次精对准后没有导航数据：%s', ...
            filename);
    end
    %% --------------------------------------------------------------------
    % 严格仅保留0x4300状态
    % ---------------------------------------------------------------------
    AUAX = AUAX(:,idx_nav);
    %% --------------------------------------------------------------------
    % 时间
    % ---------------------------------------------------------------------
    t = AUAX(2,:);
    %% --------------------------------------------------------------------
    % 120航向
    %
    % 保持你此前分析中的转换：
    %
    % 原值 psi
    %
    % >0 : 360-psi
    % <0 : -psi
    %
    % 等价于：
    %
    % mod(-psi,360)
    % ---------------------------------------------------------------------
    heading_raw = AUAX(3,:);
    heading = mod(-heading_raw,360);
    %% --------------------------------------------------------------------
    % 时间排序 + 去重
    % ---------------------------------------------------------------------
    [t,idx] = sort(t);
    heading = heading(idx);
    AUAX = AUAX(:,idx);
    [t,idx_unique] = unique(t,'stable');
    heading = heading(idx_unique);
    AUAX = AUAX(:,idx_unique);
    %% 输出
    data.time = t(:)';
    data.heading = heading(:)';
    data.raw = AUAX;
    fprintf( ...
        '120 AUAX：%d 个导航历元，%.3f ~ %.3f\n', ...
        length(data.time), ...
        data.time(1), ...
        data.time(end));
end
function data = read_prepare_830( ...
    filenames, ...
    lat_min, ...
    lat_max)
%READ_PREPARE_830
%
% 读取多个830 GPCHCX文件并合并。
%
% 使用：
%
% pva(:,2)  → GPS时间
% pva(:,11) → 航向角
    GPCHCX = [];
    %% --------------------------------------------------------------------
    % 读取并合并
    % ---------------------------------------------------------------------
    for i = 1:length(filenames)
        tmp = ...
            read_gpchcx_430_830( ...
            filenames{i});
        GPCHCX = ...
            [GPCHCX,tmp];
    end
    %% --------------------------------------------------------------------
    % 有效状态
    %
    % 原程序：
    %
    % GPCHCX(21,:) ~= 0
    % ---------------------------------------------------------------------
    valid = ...
        GPCHCX(21,:) ~= 0;
    %% --------------------------------------------------------------------
    % 纬度范围
    % ---------------------------------------------------------------------
    valid = valid & ...
        GPCHCX(12,:) >= lat_min & ...
        GPCHCX(12,:) <= lat_max;
    %% --------------------------------------------------------------------
    % 时间有效
    % ---------------------------------------------------------------------
    valid = valid & ...
        GPCHCX(2,:) > 100;
    nav = GPCHCX(:,valid);
    %% --------------------------------------------------------------------
    % 时间排序
    % ---------------------------------------------------------------------
    [~,idx] = sort(nav(2,:));
    nav = nav(:,idx);
    %% --------------------------------------------------------------------
    % 转换为PVA
    %
    % 该函数是你现有的830数据处理函数
    % ---------------------------------------------------------------------
    [~,pva] = ...
        visualize_gpchcx_navigation_results(nav);
    %% --------------------------------------------------------------------
    % 读取时间与航向
    % ---------------------------------------------------------------------
    t = pva(:,2)';
    heading = pva(:,11)';
    heading = mod(heading,360);
    %% --------------------------------------------------------------------
    % 排序及去重
    % ---------------------------------------------------------------------
    [t,idx] = sort(t);
    heading = heading(idx);
    pva = pva(idx,:);
    [t,idx_unique] = unique(t,'stable');
    heading = heading(idx_unique);
    pva = pva(idx_unique,:);
    %% 输出
    data.time = t(:)';
    data.heading = heading(:)';
    data.pva = pva;
    data.nav = nav;
    fprintf( ...
        '830参考：%d 个历元，%.3f ~ %.3f\n', ...
        length(data.time), ...
        data.time(1), ...
        data.time(end));
end
function data = read_prepare_compass( ...
    filename, ...
    year,month,day, ...
    convert_param, ...
    post_offset)
%READ_PREPARE_COMPASS
%
% 罗盘数据：
%
% raw(1,:)  hour
% raw(2,:)  minute
% raw(3,:)  second
% raw(4,:)  heading
%
% 转换后：
%
% data.time
% data.heading
    fid = fopen(filename,'r');
    if fid < 0
        error( ...
            '无法打开罗盘文件：%s', ...
            filename);
    end
    %% 跳过标题
    fgets(fid);
    %% --------------------------------------------------------------------
    % 保持原程序读取格式
    % ---------------------------------------------------------------------
    raw = fscanf( ...
        fid, ...
        ['%d:%d:%f ', ...
         '%f %f %f %f %f %f %f %f %f ', ...
         '%f %f %f %f %f %f %f %f %f %f %f\n'], ...
        [23,inf]);
    fclose(fid);
    if isempty(raw)
        error( ...
            '罗盘文件为空：%s', ...
            filename);
    end
    %% --------------------------------------------------------------------
    % 时间转换
    % ---------------------------------------------------------------------
    n = size(raw,2);
    t = nan(1,n);
    for i = 1:n
        [~,t(i)] = ...
            bjt_to_utc_week_seconds( ...
            year, ...
            month, ...
            day, ...
            raw(1,i), ...
            raw(2,i), ...
            raw(3,i), ...
            convert_param);
    end
    t = t + post_offset;
    %% 航向
    heading = mod(raw(4,:),360);
    %% --------------------------------------------------------------------
    % 删除非法值
    % ---------------------------------------------------------------------
    valid = ...
        isfinite(t) & ...
        isfinite(heading);
    t = t(valid);
    heading = heading(valid);
    %% --------------------------------------------------------------------
    % 排序
    % ---------------------------------------------------------------------
    [t,idx] = sort(t);
    heading = heading(idx);
    %% --------------------------------------------------------------------
    % 重复时间采用圆周平均
    %
    % 例如：
    %
    % 359° + 1° = 0°
    %
    % 而不是180°
    % ---------------------------------------------------------------------
    [t_unique,~,group] = ...
        unique(t);
    sin_mean = ...
        accumarray( ...
        group(:), ...
        sind(heading(:)), ...
        [], ...
        @mean);
    cos_mean = ...
        accumarray( ...
        group(:), ...
        cosd(heading(:)), ...
        [], ...
        @mean);
    heading_unique = ...
        atan2d( ...
        sin_mean, ...
        cos_mean);
    heading_unique = ...
        mod(heading_unique,360);
    %% 输出
    data.time = t_unique(:)';
    data.heading = heading_unique(:)';
    data.raw = raw;
    fprintf( ...
        'Compass：%d 个唯一历元，%.3f ~ %.3f\n', ...
        length(data.time), ...
        data.time(1), ...
        data.time(end));
end
function heading_query = ...
    interp_heading( ...
    time, ...
    heading, ...
    time_query)
%INTERP_HEADING
%
% 航向角圆周插值。
%
% 不直接：
%
% interp1(time,heading,...)
%
% 因为：
%
% 359° → 1°
%
% 普通线性插值会错误经过180°。
%
%
% 方法：
%
% heading
%    ↓
% sin(heading), cos(heading)
%    ↓
% 分别插值
%    ↓
% atan2
%
% -------------------------------------------------------------------------
    time = time(:);
    heading = heading(:);
    time_query_shape = size(time_query);
    time_query_col = time_query(:);
    %% sin/cos
    s = sind(heading);
    c = cosd(heading);
    %% 插值
    si = interp1( ...
        time, ...
        s, ...
        time_query_col, ...
        'linear', ...
        NaN);
    ci = interp1( ...
        time, ...
        c, ...
        time_query_col, ...
        'linear', ...
        NaN);
    %% 恢复航向
    heading_query = ...
        atan2d(si,ci);
    heading_query = ...
        mod(heading_query,360);
    heading_query = ...
        reshape( ...
        heading_query, ...
        time_query_shape);
end
function angle = wrap180(angle)
%WRAP180
%
% 将角度转换到
%
% [-180,180)
    angle = ...
        mod(angle+180,360)-180;
end
function stats = ...
    calculate_error_statistics(error)
%CALCULATE_ERROR_STATISTICS
    error = error(isfinite(error));
    if isempty(error)
        stats.mean = NaN;
        stats.std = NaN;
        stats.rms = NaN;
        stats.mae = NaN;
        stats.maxabs = NaN;
        return;
    end
    stats.mean = ...
        mean(error);
    stats.std = ...
        std(error);
    stats.rms = ...
        sqrt(mean(error.^2));
    stats.mae = ...
        mean(abs(error));
    stats.maxabs = ...
        max(abs(error));
end