clear;
clc;
close all;
% data07_read01_compass
% data07_read02_830
% data07_read03_auxa_120
%% ========================================================================
% 2025-12-07
% 120 / 830 / Compass 数据统一检查
%
% 输入：
%
%   temp\compass.mat
%   temp\ref830_1207.mat
%   temp\alignment120_1207.mat
%
%
% 分析顺序：
%
% 1. 120原始IMU vs 830 IMU
%       -> 检查时间覆盖
%       -> 三轴信号标准化后叠加
%       -> 判断时间是否对齐
%
% 2. 轨迹
%       -> 830始终绘制
%       -> 120 AUAX存在则叠加
%
% 3. 航向
%       -> 830始终绘制
%       -> Compass始终绘制
%       -> 120 AUAX存在则加入
%
% ========================================================================


%% ========================================================================
%% 0. 参数
% ========================================================================

temp_dir = ...
    'D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1207-longtime\temp';


compass_file = ...
    fullfile(temp_dir,'compass.mat');

ref830_file = ...
    fullfile(temp_dir,'ref830_1207.mat');

alignment120_file = ...
    fullfile(temp_dir,'alignment120_1207.mat');


%% ------------------------------------------------------------------------
% 830 PVA格式
%
% 当前已有程序中：
%
% pva(:,2)  GPS周秒
% pva(:,11) 航向
%
% 轨迹默认：
%
% pva(:,3) 纬度
% pva(:,4) 经度
% -------------------------------------------------------------------------

PVA_TIME_COL    = 2;
PVA_LAT_COL     = 3;
PVA_LON_COL     = 4;
PVA_HEADING_COL = 11;


%% ------------------------------------------------------------------------
% AUAX格式
% -------------------------------------------------------------------------

AUAX_TIME_ROW    = 2;
AUAX_HEADING_ROW = 3;
AUAX_LAT_ROW     = 6;
AUAX_LON_ROW     = 7;


%% ------------------------------------------------------------------------
% 120原始IMU
%
% AUAX(12:14) <-> IMU(:,1:3)
% -------------------------------------------------------------------------

IMU120_SIGNAL_COLS = 1:3;


%% ------------------------------------------------------------------------
% 830 IMU设置
%
% 因目前现有脚本没有明确写出 imu_830 每一列的定义，
% 所以默认：
%
%   时间列自动识别
%   信号列自动取除时间列以外的前三列
%
% 如果运行后自动识别不正确，可以直接人工设置，例如：
%
% REF830_IMU_TIME_COL = 7;
% REF830_IMU_SIGNAL_COLS = 1:3;
%
% -------------------------------------------------------------------------

REF830_IMU_TIME_COL = [];

REF830_IMU_SIGNAL_COLS = [];


%% ------------------------------------------------------------------------
% AUAX如果有缺失，不允许跨大缺口插值航向
% -------------------------------------------------------------------------

AUAX_MAX_INTERP_GAP = 2.0;     % s


%% ------------------------------------------------------------------------
% 为避免百万点直接绘图太慢
% -------------------------------------------------------------------------

MAX_PLOT_POINTS = 120000;


%% ========================================================================
%% 1. 加载数据
% ========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('1. 加载三个MAT文件\n');
fprintf('============================================================\n');


%% Compass

S = load(compass_file);

if ~isfield(S,'Compass')
    error('compass.mat 中没有 Compass。');
end

Compass = S.Compass;


%% 830

S = load(ref830_file);

if ~isfield(S,'Ref830')
    error('ref830_1207.mat 中没有 Ref830。');
end

Ref830 = S.Ref830;


%% 120

S = load(alignment120_file);

if ~isfield(S,'Alignment120')
    error('alignment120_1207.mat 中没有 Alignment120。');
end

Alignment120 = S.Alignment120;


fprintf('Compass：     %d 段\n',numel(Compass));
fprintf('Ref830：      %d 段\n',numel(Ref830));
fprintf('Alignment120：%d 段\n',numel(Alignment120));


nExp = min([ ...
    numel(Compass), ...
    numel(Ref830), ...
    numel(Alignment120)]);


%% ========================================================================
%% 2. 数据结构检查
% ========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('2. 数据结构检查\n');
fprintf('============================================================\n');


for k = 1:nExp

    fprintf('\n');
    fprintf('---------------- 实验 %d ----------------\n',k);


    %% Compass

    fprintf('Compass：%d 点，GPS %.3f ~ %.3f\n', ...
        length(Compass{k}.time), ...
        Compass{k}.time(1), ...
        Compass{k}.time(end));


    %% 830 PVA

    fprintf('830 PVA：%d × %d\n', ...
        size(Ref830{k}.pva,1), ...
        size(Ref830{k}.pva,2));


    %% 830 IMU

    if isfield(Ref830{k},'imu') && ...
            ~isempty(Ref830{k}.imu)

        fprintf('830 IMU：%d × %d\n', ...
            size(Ref830{k}.imu,1), ...
            size(Ref830{k}.imu,2));

    else

        fprintf('830 IMU：不存在\n');

    end


    %% 120 IMU

    if isfield(Alignment120(k),'imu120FRD') && ...
            ~isempty(Alignment120(k).imu120FRD)

        fprintf('120 IMU：%d × %d\n', ...
            size(Alignment120(k).imu120FRD,1), ...
            size(Alignment120(k).imu120FRD,2));

    else

        fprintf('120 IMU：不存在\n');

    end


    %% AUAX

    if isfield(Alignment120(k),'auax') && ...
            ~isempty(Alignment120(k).auax)

        fprintf('120 AUAX：%d 历元\n', ...
            size(Alignment120(k).auax,2));

    else

        fprintf('120 AUAX：无完整轨迹数据\n');

    end

end


%% ========================================================================
%% 3. 第一部分：120 IMU vs 830 IMU
%
% IMU数据格式：
%
%   第1列：GPS时间
%   第2列：F轴
%   第3列：R轴
%   第4列：D轴
%
% 这里只检查两套IMU的时间是否对齐，
% 不比较绝对幅值，因此分别标准化后叠加显示。
%
% ========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('3. 120 IMU 与 830 IMU 时间对齐检查\n');
fprintf('============================================================\n');

axis_name = {'F','R','D'};

for k = 1:nExp

    fprintf('\n');
    fprintf('---------------- 实验 %d ----------------\n',k);

    %% --------------------------------------------------------------------
    % 读取120 IMU
    % ---------------------------------------------------------------------
    if ~isfield(Alignment120(k),'imu120FRD') || ...
            isempty(Alignment120(k).imu120FRD)

        warning('实验%d没有120 IMU数据。',k);
        continue;

    end

    imu120 = Alignment120(k).imu120FRD;

    % 第1列：GPS时间
    % 第2:4列：FRD
    t120   = imu120(:,1);
    sig120 = imu120(:,2:4);


    %% --------------------------------------------------------------------
    % 读取830 IMU
    % ---------------------------------------------------------------------
    if ~isfield(Ref830{k},'imu') || ...
            isempty(Ref830{k}.imu)

        warning('实验%d没有830 IMU数据。',k);
        continue;

    end

    imu830 = Ref830{k}.imu;

    % 第1列：GPS时间
    % 第2:4列：FRD
    t830   = imu830(:,1);
    sig830 = imu830(:,2:4);


    %% --------------------------------------------------------------------
    % 删除非法数据
    % ---------------------------------------------------------------------
    valid120 = ...
        isfinite(t120) & ...
        all(isfinite(sig120),2);

    t120   = t120(valid120);
    sig120 = sig120(valid120,:);


    valid830 = ...
        isfinite(t830) & ...
        all(isfinite(sig830),2);

    t830   = t830(valid830);
    sig830 = sig830(valid830,:);


    %% --------------------------------------------------------------------
    % 按时间排序
    % ---------------------------------------------------------------------
    [t120,idx] = sort(t120);
    sig120 = sig120(idx,:);

    [t830,idx] = sort(t830);
    sig830 = sig830(idx,:);


    %% --------------------------------------------------------------------
    % 公共时间范围
    % ---------------------------------------------------------------------
    common_start = max(t120(1),t830(1));
    common_end   = min(t120(end),t830(end));

    if common_end <= common_start
        warning('实验%d两套IMU没有时间重叠。',k);
        continue;
    end

    fprintf('120 IMU：%.3f ~ %.3f\n',t120(1),t120(end));
    fprintf('830 IMU：%.3f ~ %.3f\n',t830(1),t830(end));
    fprintf('公共区间：%.3f ~ %.3f，共 %.2f s\n', ...
        common_start,common_end,common_end-common_start);


    %% --------------------------------------------------------------------
    % 截取公共区间
    % ---------------------------------------------------------------------
    idx120 = ...
        t120 >= common_start & ...
        t120 <= common_end;

    idx830 = ...
        t830 >= common_start & ...
        t830 <= common_end;


    %% --------------------------------------------------------------------
    % 标准化
    %
    % 这里只看波形事件是否同时出现：
    % 转弯、启动、静止、峰值等。
    % ---------------------------------------------------------------------
    sig120_std = standardize_columns(sig120);
    sig830_std = standardize_columns(sig830);


    %% --------------------------------------------------------------------
    % 绘图降采样
    % ---------------------------------------------------------------------
    id_plot_120 = ...
        plot_index(find(idx120),MAX_PLOT_POINTS);

    id_plot_830 = ...
        plot_index(find(idx830),MAX_PLOT_POINTS);


    %% ====================================================================
    % 图1：FRD三轴
    % =====================================================================
    figure( ...
        'Name',sprintf('实验%d 120-830 IMU时间对齐',k), ...
        'NumberTitle','off');

    tiledlayout(3,1, ...
        'TileSpacing','compact', ...
        'Padding','compact');

    for axis_id = 1:3

        nexttile;
        hold on;
        grid on;

        plot( ...
            t830(id_plot_830)-common_start, ...
            sig830_std(id_plot_830,axis_id), ...
            'LineWidth',0.8, ...
            'DisplayName','830 IMU');

        plot( ...
            t120(id_plot_120)-common_start, ...
            sig120_std(id_plot_120,axis_id), ...
            'LineWidth',0.8, ...
            'DisplayName','120 IMU');

        
        ylabel(axis_name{axis_id});

        if axis_id == 1
            title(sprintf( ...
                '实验%d：120与830 IMU时间对齐',k));

            legend('Location','best');
        end

        if axis_id == 3
            xlabel('共同运行时间 / s');
        end

    end


    %% ====================================================================
    % 图2：FRD三轴标准化模长
    % =====================================================================
    norm120 = vecnorm(sig120_std,2,2);
    norm830 = vecnorm(sig830_std,2,2);

    figure( ...
        'Name',sprintf('实验%d IMU模长时间检查',k), ...
        'NumberTitle','off');

    hold on;
    grid on;

    plot( ...
        t120(id_plot_120)-common_start, ...
        norm120(id_plot_120), ...
        'LineWidth',0.8, ...
        'DisplayName','120 IMU');

    plot( ...
        t830(id_plot_830)-common_start, ...
        norm830(id_plot_830), ...
        'LineWidth',0.8, ...
        'DisplayName','830 IMU');

    xlabel('共同运行时间 / s');
    ylabel('标准化FRD三轴模长');

    title(sprintf( ...
        '实验%d：IMU机动特征时间检查',k));

    legend('Location','best');

end

%% ========================================================================
%% 4. 第二部分：轨迹
%
% 830一定绘制。
%
% 如果AUAX缺失：
%
%   只画830。
%
% 如果AUAX存在：
%
%   叠加AUAX实际存在的轨迹。
%
% ========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('4. 830 / AUAX 轨迹\n');
fprintf('============================================================\n');


for k = 1:nExp

    pva830 = ...
        Ref830{k}.pva;


    %% --------------------------------------------------------------------
    % 830轨迹
    % ---------------------------------------------------------------------

    lat830 = ...
        pva830(:,PVA_LAT_COL);

    lon830 = ...
        pva830(:,PVA_LON_COL);


    valid830 = ...
        isfinite(lat830) & ...
        isfinite(lon830) & ...
        lat830 ~= 0 & ...
        lon830 ~= 0;


    lat830 = lat830(valid830);

    lon830 = lon830(valid830);


    %% --------------------------------------------------------------------
    % 图
    % ---------------------------------------------------------------------

    figure( ...
        'Name',sprintf('实验%d 830-AUAX轨迹',k), ...
        'NumberTitle','off');


    hold on;
    grid on;
    axis equal;


    plot( ...
        lon830, ...
        lat830, ...
        'LineWidth',1.2, ...
        'DisplayName','830参考');


    plot( ...
        lon830(1), ...
        lat830(1), ...
        '*', ...
        'MarkerSize',10, ...
        'DisplayName','830起点');


    %% ====================================================================
    % AUAX存在则叠加
    % =====================================================================

    has_auax = ...
        isfield(Alignment120(k),'auax') && ...
        ~isempty(Alignment120(k).auax);


    if has_auax

        A = ...
            Alignment120(k).auax;


        lat120 = ...
            A(AUAX_LAT_ROW,:);

        lon120 = ...
            A(AUAX_LON_ROW,:);


        valid120 = ...
            isfinite(lat120) & ...
            isfinite(lon120) & ...
            lat120 > 30 & ...
            lat120 < 40 & ...
            lon120 > 100 & ...
            lon120 < 130;


        lat120 = lat120(valid120);

        lon120 = lon120(valid120);


        if ~isempty(lat120)

            plot( ...
                lon120, ...
                lat120, ...
                'LineWidth',1, ...
                'DisplayName','120 AUAX');


            plot( ...
                lon120(1), ...
                lat120(1), ...
                'o', ...
                'MarkerSize',8, ...
                'DisplayName','AUAX起点');

        end

    end


    xlabel('Longitude / °');

    ylabel('Latitude / °');


    title(sprintf( ...
        '实验%d：830参考轨迹',k));


    legend('Location','best');


    if ~has_auax

        fprintf( ...
            '实验%d AUAX轨迹缺失，本次仅显示830。\n',k);

    end

end


%% ========================================================================
%% 5. 第三部分：航向角
%
% 基本原则：
%
% 830      一定存在
% Compass  一定存在
%
% AUAX：
%
%   有数据 -> 三者比较
%   无数据 -> 只比较830和Compass
%
% ========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('5. 120 / 830 / Compass 航向\n');
fprintf('============================================================\n');


for k = 1:nExp

    %% ====================================================================
    % 830
    % =====================================================================

    t830 = ...
        Ref830{k}.time(:)';

    heading830 = ...
        mod( ...
        Ref830{k}.heading(:)', ...
        360);


    %% ====================================================================
    % Compass
    % =====================================================================

    tCompass = ...
        Compass{k}.time(:)';

    headingCompass = ...
        mod( ...
        Compass{k}.heading(:)', ...
        360);


    %% ====================================================================
    % 是否有120 AUAX
    % =====================================================================

    has_auax = ...
        isfield(Alignment120(k),'auax') && ...
        ~isempty(Alignment120(k).auax);


    %% --------------------------------------------------------------------
    % 先确定830 / Compass公共时间
    % ---------------------------------------------------------------------

    t_start = ...
        max( ...
        min(t830), ...
        min(tCompass));


    t_end = ...
        min( ...
        max(t830), ...
        max(tCompass));


    %% 如果有AUAX，再进一步缩小

    if has_auax

        A = ...
            Alignment120(k).auax;


        t120 = ...
            A(AUAX_TIME_ROW,:);


        heading120 = ...
            mod( ...
            -A(AUAX_HEADING_ROW,:), ...
            360);


        t_start = ...
            max(t_start,min(t120));

        t_end = ...
            min(t_end,max(t120));

    end


    if t_end <= t_start

        warning('实验%d航向数据没有有效公共区间。',k);

        continue;

    end


    %% ====================================================================
    % 使用830作为统一时间轴
    % =====================================================================

    idx830 = ...
        t830 >= t_start & ...
        t830 <= t_end;


    t = ...
        t830(idx830);


    h830 = ...
        heading830(idx830);


    %% --------------------------------------------------------------------
    % Compass圆周插值
    % ---------------------------------------------------------------------

    hCompass = ...
        interp_heading_circular( ...
        tCompass, ...
        headingCompass, ...
        t);


    %% --------------------------------------------------------------------
    % AUAX圆周插值
    %
    % 注意AUAX可能存在缺口，因此采用分段插值。
    % ---------------------------------------------------------------------

    if has_auax

        h120 = ...
            interp_heading_gap( ...
            t120, ...
            heading120, ...
            t, ...
            AUAX_MAX_INTERP_GAP);

    else

        h120 = ...
            nan(size(t));

    end


    %% ====================================================================
    % 有效点
    % =====================================================================

    if has_auax

        valid = ...
            isfinite(h830) & ...
            isfinite(hCompass) & ...
            isfinite(h120);

    else

        valid = ...
            isfinite(h830) & ...
            isfinite(hCompass);

    end


    t_plot = t(valid);

    h830_plot = h830(valid);

    hCompass_plot = hCompass(valid);

    h120_plot = h120(valid);


    if isempty(t_plot)

        warning('实验%d航向有效数据为空。',k);

        continue;

    end


    %% 相对时间

    elapsed = ...
        t_plot-t_plot(1);


    %% ====================================================================
    % 航向误差
    % =====================================================================

    errorCompass = ...
        wrap180_local( ...
        hCompass_plot-h830_plot);


    if has_auax

        error120 = ...
            wrap180_local( ...
            h120_plot-h830_plot);

    end


    %% ====================================================================
    % 绘图
    % =====================================================================

    figure( ...
        'Name',sprintf('实验%d 航向对比',k), ...
        'NumberTitle','off');


    tiledlayout(2,1, ...
        'TileSpacing','compact', ...
        'Padding','compact');


    %% --------------------------------------------------------------------
    % 航向
    % ---------------------------------------------------------------------

    nexttile;

    hold on;
    grid on;


    plot( ...
        elapsed, ...
        h830_plot, ...
        'LineWidth',1.1, ...
        'DisplayName','830参考');


    plot( ...
        elapsed, ...
        hCompass_plot, ...
        'LineWidth',1, ...
        'DisplayName','罗盘');


    if has_auax

        plot( ...
            elapsed, ...
            h120_plot, ...
            'LineWidth',1, ...
            'DisplayName','120 AUAX');

    end


    ylabel('航向角 / °');

    ylim([0 360]);


    title(sprintf( ...
        '实验%d：航向角对比',k));


    legend('Location','best');


    %% --------------------------------------------------------------------
    % 航向差
    % ---------------------------------------------------------------------

    nexttile;

    hold on;
    grid on;


    plot( ...
        elapsed, ...
        errorCompass, ...
        'LineWidth',1, ...
        'DisplayName','罗盘 - 830');


    if has_auax

        plot( ...
            elapsed, ...
            error120, ...
            'LineWidth',1, ...
            'DisplayName','120 - 830');

    end


    yline(0,'--');


    xlabel('共同运行时间 / s');

    ylabel('\Delta\psi / °');


    title('相对于830参考系统的航向差');


    legend('Location','best');


    %% ====================================================================
    % 统计
    % =====================================================================

    fprintf('\n');
    fprintf('---------------- 实验 %d ----------------\n',k);


    fprintf('\nCompass - 830\n');

    print_stats(errorCompass);


    if has_auax

        fprintf('\n120 AUAX - 830\n');

        print_stats(error120);

    else

        fprintf('\n120 AUAX：数据缺失，不统计。\n');

    end

end


%% ========================================================================
%% 局部函数
% ========================================================================


function col = detect_830_time_column(data,time_range)

%DETECT_830_TIME_COLUMN
%
% 自动从830 IMU矩阵中寻找GPS周秒列。
%
% 条件：
%
% 1. 基本单调递增
% 2. 中值位于Ref830 GPS时间范围附近
% 3. 有明显时间跨度


    col = [];


    t_min = time_range(1);

    t_max = time_range(2);


    margin = ...
        max(300,0.1*(t_max-t_min));


    best_score = inf;


    for j = 1:size(data,2)

        x = data(:,j);


        x = x(isfinite(x));


        if length(x) < 100
            continue;
        end


        dx = diff(x);


        monotonic_ratio = ...
            mean(dx >= 0);


        if monotonic_ratio < 0.95
            continue;
        end


        span = ...
            max(x)-min(x);


        if span < 10
            continue;
        end


        mid = median(x);


        if mid < t_min-margin || ...
                mid > t_max+margin

            continue;

        end


        target_mid = ...
            (t_min+t_max)/2;


        score = ...
            abs(mid-target_mid);


        if score < best_score

            best_score = score;

            col = j;

        end

    end

end


function print_column_information(data)

%PRINT_COLUMN_INFORMATION


    fprintf('\n830 IMU各列基本信息：\n');

    fprintf( ...
        'Col      Min               Max               Median\n');


    for j = 1:size(data,2)

        x = data(:,j);

        x = x(isfinite(x));


        if isempty(x)
            continue;
        end


        fprintf( ...
            '%3d  %16.6f  %16.6f  %16.6f\n', ...
            j, ...
            min(x), ...
            max(x), ...
            median(x));

    end

end


function data_out = standardize_columns(data)

%STANDARDIZE_COLUMNS
%
% 每列分别标准化。


    data_out = ...
        nan(size(data));


    for j = 1:size(data,2)

        x = data(:,j);


        mu = ...
            mean(x,'omitnan');


        sigma = ...
            std(x,'omitnan');


        if ~isfinite(sigma) || sigma == 0
            sigma = 1;
        end


        data_out(:,j) = ...
            (x-mu)/sigma;

    end

end


function idx = plot_index(index,max_points)

%PLOT_INDEX
%
% 降采样仅用于画图，不改变原数据。


    if isempty(index)

        idx = index;
        return;

    end


    if length(index) <= max_points

        idx = index;
        return;

    end


    step = ...
        ceil(length(index)/max_points);


    idx = ...
        index(1:step:end);

end


function hq = interp_heading_circular(t,h,tq)

%INTERP_HEADING_CIRCULAR
%
% 普通连续航向圆周插值。


    t = t(:);

    h = h(:);

    tq_shape = size(tq);

    tq = tq(:);


    valid = ...
        isfinite(t) & ...
        isfinite(h);


    t = t(valid);

    h = h(valid);


    [t,idx] = sort(t);

    h = h(idx);


    [t,idx] = unique(t,'stable');

    h = h(idx);


    s = sind(h);

    c = cosd(h);


    si = interp1( ...
        t,s,tq, ...
        'linear',NaN);


    ci = interp1( ...
        t,c,tq, ...
        'linear',NaN);


    hq = ...
        mod( ...
        atan2d(si,ci), ...
        360);


    hq = ...
        reshape(hq,tq_shape);

end


function hq = interp_heading_gap(t,h,tq,max_gap)

%INTERP_HEADING_GAP
%
% 对存在数据缺失的AUAX进行分段圆周插值。
%
% 如果相邻AUAX时间间隔 > max_gap：
%
% 不跨缺口插值。


    t = t(:);

    h = h(:);

    tq_shape = size(tq);

    tq_col = tq(:);


    valid = ...
        isfinite(t) & ...
        isfinite(h);


    t = t(valid);

    h = h(valid);


    [t,idx] = sort(t);

    h = h(idx);


    [t,idx] = unique(t,'stable');

    h = h(idx);


    hq_col = ...
        nan(size(tq_col));


    if length(t) < 2

        hq = reshape(hq_col,tq_shape);
        return;

    end


    break_idx = ...
        find(diff(t) > max_gap);


    seg_start = ...
        [1;break_idx+1];


    seg_end = ...
        [break_idx;length(t)];


    for i = 1:length(seg_start)

        ids = ...
            seg_start(i):seg_end(i);


        if length(ids) < 2
            continue;
        end


        tq_id = ...
            tq_col >= t(ids(1)) & ...
            tq_col <= t(ids(end));


        if ~any(tq_id)
            continue;
        end


        s = ...
            sind(h(ids));


        c = ...
            cosd(h(ids));


        si = interp1( ...
            t(ids), ...
            s, ...
            tq_col(tq_id), ...
            'linear');


        ci = interp1( ...
            t(ids), ...
            c, ...
            tq_col(tq_id), ...
            'linear');


        hq_col(tq_id) = ...
            mod( ...
            atan2d(si,ci), ...
            360);

    end


    hq = ...
        reshape(hq_col,tq_shape);

end


function a = wrap180_local(a)

%WRAP180_LOCAL

    a = ...
        mod(a+180,360)-180;

end


function print_stats(error)

%PRINT_STATS


    error = ...
        error(isfinite(error));


    if isempty(error)

        fprintf('无有效数据。\n');

        return;

    end


    fprintf('Mean = %8.4f deg\n', ...
        mean(error));

    fprintf('STD  = %8.4f deg\n', ...
        std(error));

    fprintf('RMS  = %8.4f deg\n', ...
        sqrt(mean(error.^2)));

    fprintf('MAE  = %8.4f deg\n', ...
        mean(abs(error)));

    fprintf('MAX  = %8.4f deg\n', ...
        max(abs(error)));

end