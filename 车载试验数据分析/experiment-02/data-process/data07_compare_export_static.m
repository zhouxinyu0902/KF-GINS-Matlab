clear;
clc;
close all;
% data07_read01_compass
% data07_read02_830_merged
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
% 1. 三套IMU统一比较
%       -> 120 IMU
%       -> 830 binary IMU
%       -> 830 GPCHCX-derived IMU
%       -> 检查时间覆盖
%       -> FRD三轴/六轴标准化后叠加
%       -> 保存最终确认正确的三套FRD IMU到CompareIMU
%
% 2. 轨迹
% 3. 航向
% 4. TXT导出
%       -> 动态IMU直接导出第3部分确认正确的CompareIMU
%       -> 同步导出120/830 binary/830 GPCHCX静态IMU
%       -> 同步导出830静态PVA/GPS
%       -> 输出case-01 / case-02
%
% ========================================================================

%% ========================================================================
%% 0. 参数
% ========================================================================

base_dir = ...
    'D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1207-longtime';

temp_dir = ...
    fullfile(base_dir,'temp');

out_dir = {
    fullfile(base_dir,'case-01')
    fullfile(base_dir,'case-02')
};

compass_file = ...
    fullfile(temp_dir,'compass.mat');

ref830_file = ...
    fullfile(temp_dir,'ref830_1207_new.mat');

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
% IMU统一格式
%
% 当前三个数据源均采用：
%
%   第1列    GPS时间
%   第2:4列  FRD前三轴
%
% 如果数据具有完整六轴，则：
%
%   第2:4列  Gyro F/R/D
%   第5:7列  Acc  F/R/D
%
% compare阶段不再重新做坐标转换和单位转换。
% -------------------------------------------------------------------------
IMU_TIME_COL = 1;
IMU_FRD3_COLS = 2:4;
IMU_FRD6_COLS = 2:7;

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

    %% 830 GPCHCX-derived IMU
    if isfield(Ref830{k},'imu_gpchcx_frd') && ...
            ~isempty(Ref830{k}.imu_gpchcx_frd)
        fprintf('830 GPCHCX IMU：%d × %d\n', ...
            size(Ref830{k}.imu_gpchcx_frd,1), ...
            size(Ref830{k}.imu_gpchcx_frd,2));
    elseif isfield(Ref830{k},'imu') && ...
            ~isempty(Ref830{k}.imu)
        fprintf('830 GPCHCX IMU（旧字段imu）：%d × %d\n', ...
            size(Ref830{k}.imu,1), ...
            size(Ref830{k}.imu,2));
    else
        fprintf('830 GPCHCX IMU：不存在\n');
    end

    %% 830 binary IMU
    if isfield(Ref830{k},'imu_binary_frd') && ...
            ~isempty(Ref830{k}.imu_binary_frd)
        fprintf('830 binary IMU：%d × %d\n', ...
            size(Ref830{k}.imu_binary_frd,1), ...
            size(Ref830{k}.imu_binary_frd,2));
    else
        fprintf('830 binary IMU：不存在\n');
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
%% 3. 第一部分：120 / 830 binary / 830 GPCHCX IMU
%
% 三套数据：
%
%   Alignment120(k).imu120FRD
%   Ref830{k}.imu_binary_frd
%   Ref830{k}.imu_gpchcx_frd
%
% 默认统一格式：
%
%   第1列    GPS时间
%   第2:4列  FRD前三轴
%   第5:7列  如果存在，则为另外三轴
%
% 本部分主要检查：
%
%   1. 三套IMU绝对时间覆盖是否一致
%   2. 转弯、启动、静止、峰值是否发生在相同时间
%   3. F/R/D坐标轴是否对应
%
% 由于120、830 binary、GPCHCX-derived当前可能分别为增量或速率，
% 因此时间对齐图采用“各数据源各轴独立标准化”，不直接比较幅值。
% ========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('3. 120 / 830 binary / 830 GPCHCX IMU 比较\n');
fprintf('============================================================\n');

axis_name_3 = {'F','R','D'};
axis_name_6 = {'Gyro F','Gyro R','Gyro D','Acc F','Acc R','Acc D'};
CompareIMU = cell(nExp,1);

for k = 1:nExp

    fprintf('\n');
    fprintf('---------------- 实验 %d ----------------\n',k);

    %% --------------------------------------------------------------------
    % 3.1 读取120
    % ---------------------------------------------------------------------
    if ~isfield(Alignment120(k),'imu120FRD') || ...
            isempty(Alignment120(k).imu120FRD)
        warning('实验%d没有120 imu120FRD。',k);
        continue;
    end

    imu120 = Alignment120(k).imu120FRD;

    %% --------------------------------------------------------------------
    % 3.2 读取830 binary
    % ---------------------------------------------------------------------
    has_binary = ...
        isfield(Ref830{k},'imu_binary_frd') && ...
        ~isempty(Ref830{k}.imu_binary_frd);

    if has_binary
        imu830_binary = Ref830{k}.imu_binary_frd;
    else
        imu830_binary = [];
        warning('实验%d没有830 binary IMU。',k);
    end

    %% --------------------------------------------------------------------
    % 3.3 读取830 GPCHCX-derived
    %
    % 优先使用新字段imu_gpchcx_frd。
    % 如果MAT是旧版本，则回退到Ref830{k}.imu。
    % ---------------------------------------------------------------------
    has_gpchcx = ...
        isfield(Ref830{k},'imu_gpchcx_frd') && ...
        ~isempty(Ref830{k}.imu_gpchcx_frd);

    if has_gpchcx
        imu830_gpchcx = Ref830{k}.imu_gpchcx_frd;
    elseif isfield(Ref830{k},'imu') && ~isempty(Ref830{k}.imu)
        imu830_gpchcx = Ref830{k}.imu;
        has_gpchcx = true;
        fprintf('830 GPCHCX IMU：使用旧字段 Ref830{%d}.imu。\n',k);
    else
        imu830_gpchcx = [];
        warning('实验%d没有830 GPCHCX-derived IMU。',k);
    end

    if ~has_binary && ~has_gpchcx
        warning('实验%d两套830 IMU都不存在，无法进行IMU比较。',k);
        continue;
    end

    %% --------------------------------------------------------------------
    % 3.4 统一提取时间和FRD前三轴
    % ---------------------------------------------------------------------
    [t120,sig120] = prepare_imu_data( ...
        imu120,IMU_TIME_COL,IMU_FRD3_COLS);

    if has_binary
        [t_bin,sig_bin] = prepare_imu_data( ...
            imu830_binary,IMU_TIME_COL,IMU_FRD3_COLS);
    else
        t_bin = [];
        sig_bin = [];
    end

    if has_gpchcx
        [t_gpc,sig_gpc] = prepare_imu_data( ...
            imu830_gpchcx,IMU_TIME_COL,IMU_FRD3_COLS);
    else
        t_gpc = [];
        sig_gpc = [];
    end

    fprintf('120 IMU：        %.3f ~ %.3f，%d点\n', ...
        t120(1),t120(end),length(t120));

    if has_binary
        fprintf('830 binary IMU： %.3f ~ %.3f，%d点\n', ...
            t_bin(1),t_bin(end),length(t_bin));
    end

    if has_gpchcx
        fprintf('830 GPCHCX IMU： %.3f ~ %.3f，%d点\n', ...
            t_gpc(1),t_gpc(end),length(t_gpc));
    end

    %% --------------------------------------------------------------------
    % 3.5 三套数据公共时间
    % ---------------------------------------------------------------------
    common_start_list = t120(1);
    common_end_list = t120(end);

    if has_binary
        common_start_list(end+1) = t_bin(1);
        common_end_list(end+1) = t_bin(end);
    end

    if has_gpchcx
        common_start_list(end+1) = t_gpc(1);
        common_end_list(end+1) = t_gpc(end);
    end

    common_start = max(common_start_list);
    common_end = min(common_end_list);

    if common_end <= common_start
        warning('实验%d三套IMU没有公共时间范围。',k);
        continue;
    end

    fprintf('三套公共区间：%.3f ~ %.3f，共 %.2f s\n', ...
        common_start,common_end,common_end-common_start);

    %% --------------------------------------------------------------------
    % 3.6 截取公共时间
    % ---------------------------------------------------------------------
    idx120 = t120 >= common_start & t120 <= common_end;

    if has_binary
        idx_bin = t_bin >= common_start & t_bin <= common_end;
    else
        idx_bin = false(size(t_bin));
    end

    if has_gpchcx
        idx_gpc = t_gpc >= common_start & t_gpc <= common_end;
    else
        idx_gpc = false(size(t_gpc));
    end
    CompareIMU{k}.time_range = [common_start,common_end];
    idx_export_120 = imu120(:,IMU_TIME_COL) >= common_start & imu120(:,IMU_TIME_COL) <= common_end;
    CompareIMU{k}.imu120 = imu120(idx_export_120,:);
    if has_binary
        idx_export_bin = imu830_binary(:,IMU_TIME_COL) >= common_start & imu830_binary(:,IMU_TIME_COL) <= common_end;
        CompareIMU{k}.imu830_binary = imu830_binary(idx_export_bin,:);
    else
        CompareIMU{k}.imu830_binary = [];
    end
    if has_gpchcx
        idx_export_gpc = imu830_gpchcx(:,IMU_TIME_COL) >= common_start & imu830_gpchcx(:,IMU_TIME_COL) <= common_end;
        CompareIMU{k}.imu830_gpchcx = imu830_gpchcx(idx_export_gpc,:);
    else
        CompareIMU{k}.imu830_gpchcx = [];
    end
    fprintf('最终导出IMU公共区间：%.3f ~ %.3f\n',common_start,common_end);
    fprintf('  120 IMU：       %d × %d\n',size(CompareIMU{k}.imu120,1),size(CompareIMU{k}.imu120,2));
    fprintf('  830 binary：    %d × %d\n',size(CompareIMU{k}.imu830_binary,1),size(CompareIMU{k}.imu830_binary,2));
    fprintf('  830 GPCHCX：    %d × %d\n',size(CompareIMU{k}.imu830_gpchcx,1),size(CompareIMU{k}.imu830_gpchcx,2));
    %% 3.7 静态三套IMU公共时间并保存
    imu120_static = [];
    imu830_binary_static = [];
    imu830_gpchcx_static = [];
    if isfield(Alignment120(k),'imu120FRD_static') && ~isempty(Alignment120(k).imu120FRD_static)
        imu120_static = Alignment120(k).imu120FRD_static;
    end
    if isfield(Ref830{k},'imu_binary_frd_static') && ~isempty(Ref830{k}.imu_binary_frd_static)
        imu830_binary_static = Ref830{k}.imu_binary_frd_static;
    end
    if isfield(Ref830{k},'imu_gpchcx_frd_static') && ~isempty(Ref830{k}.imu_gpchcx_frd_static)
        imu830_gpchcx_static = Ref830{k}.imu_gpchcx_frd_static;
    end
    CompareIMU{k}.static_time_range = [];
    CompareIMU{k}.imu120_static = [];
    CompareIMU{k}.imu830_binary_static = [];
    CompareIMU{k}.imu830_gpchcx_static = [];
    static_start_list = [];
    static_end_list = [];
    if ~isempty(imu120_static)
        static_start_list(end+1) = imu120_static(1,IMU_TIME_COL);
        static_end_list(end+1) = imu120_static(end,IMU_TIME_COL);
    end
    if ~isempty(imu830_binary_static)
        static_start_list(end+1) = imu830_binary_static(1,IMU_TIME_COL);
        static_end_list(end+1) = imu830_binary_static(end,IMU_TIME_COL);
    end
    if ~isempty(imu830_gpchcx_static)
        static_start_list(end+1) = imu830_gpchcx_static(1,IMU_TIME_COL);
        static_end_list(end+1) = imu830_gpchcx_static(end,IMU_TIME_COL);
    end
    if ~isempty(static_start_list)
        static_start = max(static_start_list);
        static_end = min(static_end_list);
        if static_end > static_start
            CompareIMU{k}.static_time_range = [static_start,static_end];
            if ~isempty(imu120_static)
                idx = imu120_static(:,IMU_TIME_COL) >= static_start & imu120_static(:,IMU_TIME_COL) <= static_end;
                CompareIMU{k}.imu120_static = imu120_static(idx,:);
            end
            if ~isempty(imu830_binary_static)
                idx = imu830_binary_static(:,IMU_TIME_COL) >= static_start & imu830_binary_static(:,IMU_TIME_COL) <= static_end;
                CompareIMU{k}.imu830_binary_static = imu830_binary_static(idx,:);
            end
            if ~isempty(imu830_gpchcx_static)
                idx = imu830_gpchcx_static(:,IMU_TIME_COL) >= static_start & imu830_gpchcx_static(:,IMU_TIME_COL) <= static_end;
                CompareIMU{k}.imu830_gpchcx_static = imu830_gpchcx_static(idx,:);
            end
            fprintf('静态IMU公共区间：%.3f ~ %.3f，共 %.2f s\n',static_start,static_end,static_end-static_start);
            fprintf('  120静态IMU：    %d × %d\n',size(CompareIMU{k}.imu120_static,1),size(CompareIMU{k}.imu120_static,2));
            fprintf('  830 binary静态：%d × %d\n',size(CompareIMU{k}.imu830_binary_static,1),size(CompareIMU{k}.imu830_binary_static,2));
            fprintf('  830 GPCHCX静态：%d × %d\n',size(CompareIMU{k}.imu830_gpchcx_static,1),size(CompareIMU{k}.imu830_gpchcx_static,2));
        else
            warning('实验%d静态IMU没有公共时间。',k);
        end
    else
        warning('实验%d没有可用静态IMU。',k);
    end

    %% --------------------------------------------------------------------
    % 3.7 各数据源独立标准化
    % ---------------------------------------------------------------------
    sig120_std = standardize_columns(sig120);

    if has_binary
        sig_bin_std = standardize_columns(sig_bin);
    else
        sig_bin_std = [];
    end

    if has_gpchcx
        sig_gpc_std = standardize_columns(sig_gpc);
    else
        sig_gpc_std = [];
    end

    %% --------------------------------------------------------------------
    % 3.8 绘图降采样
    % ---------------------------------------------------------------------
    id_plot_120 = ...
        plot_index(find(idx120),MAX_PLOT_POINTS);

    if has_binary
        id_plot_bin = ...
            plot_index(find(idx_bin),MAX_PLOT_POINTS);
    else
        id_plot_bin = [];
    end

    if has_gpchcx
        id_plot_gpc = ...
            plot_index(find(idx_gpc),MAX_PLOT_POINTS);
    else
        id_plot_gpc = [];
    end

    %% ====================================================================
    % 图1：三套IMU的FRD前三轴
    % =====================================================================
    figure( ...
        'Name',sprintf('实验%d 三套IMU FRD时间对齐',k), ...
        'NumberTitle','off');

    tiledlayout(3,1, ...
        'TileSpacing','compact', ...
        'Padding','compact');

    for axis_id = 1:3

        nexttile;
        hold on;
        grid on;

        plot( ...
            t120(id_plot_120)-common_start, ...
            sig120_std(id_plot_120,axis_id), ...
            'LineWidth',0.8, ...
            'DisplayName','120 IMU');

        if has_binary
            plot( ...
                t_bin(id_plot_bin)-common_start, ...
                sig_bin_std(id_plot_bin,axis_id), ...
                'LineWidth',0.8, ...
                'DisplayName','830 binary');
        end

        if has_gpchcx
            plot( ...
                t_gpc(id_plot_gpc)-common_start, ...
                sig_gpc_std(id_plot_gpc,axis_id), ...
                'LineWidth',0.8, ...
                'DisplayName','830 GPCHCX');
        end

        ylabel(axis_name_3{axis_id});

        if axis_id == 1
            title(sprintf( ...
                '实验%d：120 / 830 binary / 830 GPCHCX FRD时间对齐',k));
            legend('Location','best');
        end

        if axis_id == 3
            xlabel('共同运行时间 / s');
        end

    end

    %% ====================================================================
    % 图2：FRD前三轴标准化模长
    % =====================================================================
    norm120 = vecnorm(sig120_std,2,2);

    if has_binary
        norm_bin = vecnorm(sig_bin_std,2,2);
    end

    if has_gpchcx
        norm_gpc = vecnorm(sig_gpc_std,2,2);
    end

    figure( ...
        'Name',sprintf('实验%d 三套IMU机动特征',k), ...
        'NumberTitle','off');

    hold on;
    grid on;

    plot( ...
        t120(id_plot_120)-common_start, ...
        norm120(id_plot_120), ...
        'LineWidth',0.8, ...
        'DisplayName','120 IMU');

    if has_binary
        plot( ...
            t_bin(id_plot_bin)-common_start, ...
            norm_bin(id_plot_bin), ...
            'LineWidth',0.8, ...
            'DisplayName','830 binary');
    end

    if has_gpchcx
        plot( ...
            t_gpc(id_plot_gpc)-common_start, ...
            norm_gpc(id_plot_gpc), ...
            'LineWidth',0.8, ...
            'DisplayName','830 GPCHCX');
    end

    xlabel('共同运行时间 / s');
    ylabel('标准化FRD三轴模长');
    title(sprintf('实验%d：三套IMU机动特征时间检查',k));
    legend('Location','best');

    %% ====================================================================
    % 图3：如果三套数据都有完整6轴，则再比较六轴
    %
    % 这里仍然仅作标准化波形比较，不直接比较绝对幅值。
    % =====================================================================
    has_120_6 = size(imu120,2) >= max(IMU_FRD6_COLS);
    has_bin_6 = has_binary && ...
        size(imu830_binary,2) >= max(IMU_FRD6_COLS);
    has_gpc_6 = has_gpchcx && ...
        size(imu830_gpchcx,2) >= max(IMU_FRD6_COLS);

    if has_120_6 && has_bin_6 && has_gpc_6

        [t120_6,sig120_6] = prepare_imu_data( ...
            imu120,IMU_TIME_COL,IMU_FRD6_COLS);

        [tbin_6,sigbin_6] = prepare_imu_data( ...
            imu830_binary,IMU_TIME_COL,IMU_FRD6_COLS);

        [tgpc_6,siggpc_6] = prepare_imu_data( ...
            imu830_gpchcx,IMU_TIME_COL,IMU_FRD6_COLS);

        idx120_6 = t120_6 >= common_start & t120_6 <= common_end;
        idxbin_6 = tbin_6 >= common_start & tbin_6 <= common_end;
        idxgpc_6 = tgpc_6 >= common_start & tgpc_6 <= common_end;

        sig120_6 = standardize_columns(sig120_6);
        sigbin_6 = standardize_columns(sigbin_6);
        siggpc_6 = standardize_columns(siggpc_6);

        id120_6 = plot_index(find(idx120_6),MAX_PLOT_POINTS);
        idbin_6 = plot_index(find(idxbin_6),MAX_PLOT_POINTS);
        idgpc_6 = plot_index(find(idxgpc_6),MAX_PLOT_POINTS);

        figure( ...
            'Name',sprintf('实验%d 三套IMU六轴比较',k), ...
            'NumberTitle','off');

        tiledlayout(3,2, ...
            'TileSpacing','compact', ...
            'Padding','compact');

        for axis_id = 1:6

            nexttile;
            hold on;
            grid on;

            plot( ...
                t120_6(id120_6)-common_start, ...
                sig120_6(id120_6,axis_id), ...
                'LineWidth',0.8, ...
                'DisplayName','120 IMU');

            plot( ...
                tbin_6(idbin_6)-common_start, ...
                sigbin_6(idbin_6,axis_id), ...
                'LineWidth',0.8, ...
                'DisplayName','830 binary');

            plot( ...
                tgpc_6(idgpc_6)-common_start, ...
                siggpc_6(idgpc_6,axis_id), ...
                'LineWidth',0.8, ...
                'DisplayName','830 GPCHCX');

            ylabel(axis_name_6{axis_id});

            if axis_id == 1
                legend('Location','best');
            end

            if axis_id >= 5
                xlabel('共同运行时间 / s');
            end

        end

        sgtitle(sprintf( ...
            '实验%d：120 / 830 binary / 830 GPCHCX 六轴标准化比较',k));

    end

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
        t_plot, ...
        h830_plot, ...
        'LineWidth',1.1, ...
        'DisplayName','830参考');

    plot( ...
        t_plot, ...
        hCompass_plot, ...
        'LineWidth',1, ...
        'DisplayName','罗盘');

    if has_auax

        plot( ...
            t_plot, ...
            h120_plot, ...
            'LineWidth',1, ...
            'DisplayName','120 AUAX');

    end

    ylabel('航向角 / °');

    ylim([0 360]);

    title(sprintf( ...
        '实验%d：航向角对比',k));

    legend('Location','best');
    xlim([t_plot(1),t_plot(end)]);
    ax = gca;
    ax.XAxis.Exponent = 0;
    xtickformat('%.0f');

    %% --------------------------------------------------------------------
    % 航向差
    % ---------------------------------------------------------------------

    nexttile;

    hold on;
    grid on;

    plot( ...
        t_plot, ...
        errorCompass, ...
        'LineWidth',1, ...
        'DisplayName','罗盘 - 830');

    if has_auax

        plot( ...
            t_plot, ...
            error120, ...
            'LineWidth',1, ...
            'DisplayName','120 - 830');

    end

    yline(0,'--');

    xlabel('GPS周秒 / s');

    ylabel('\Delta\psi / °');

    title('相对于830参考系统的航向差');

    legend('Location','best');
    xlim([t_plot(1),t_plot(end)]);
    ax = gca;
    ax.XAxis.Exponent = 0;
    xtickformat('%.0f');

    %% ====================================================================
    % 统计
    % =====================================================================

    fprintf('\n');
    fprintf('---------------- 实验 %d ----------------\n',k);
    fprintf('公共绝对时间：%.3f ~ %.3f\n',t_plot(1),t_plot(end));

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
%% 6. 导出两个有效动态数据集
%
% IMU不再重新从MAT字段截取。
% 直接使用第3部分已经完成比较并确认正确的：
%
%   CompareIMU{k}.imu120
%   CompareIMU{k}.imu830_binary
%   CompareIMU{k}.imu830_gpchcx
%
% 三套IMU均采用同一个common_start ~ common_end。
% PVA / Compass / GPS也按该公共区间截取。
% ========================================================================
fprintf('\n');
fprintf('============================================================\n');
fprintf('6. 导出两个有效动态数据集\n');
fprintf('============================================================\n');
for k = 1:nExp
    if ~exist(out_dir{k},'dir')
        mkdir(out_dir{k});
    end
    fprintf('\n============================================================\n');
    fprintf('导出数据集 %d：%s\n',k,Compass{k}.name);
    fprintf('输出目录：%s\n',out_dir{k});
    fprintf('============================================================\n');
    if k > numel(CompareIMU) || isempty(CompareIMU{k}) || ~isfield(CompareIMU{k},'time_range')
        warning('实验%d没有第3部分确认后的CompareIMU，跳过导出。',k);
        continue;
    end
    t_start = CompareIMU{k}.time_range(1);
    t_end = CompareIMU{k}.time_range(2);
    fprintf('有效时间：%.3f ~ %.3f，共 %.2f min\n',t_start,t_end,(t_end-t_start)/60);
    %% 6.1 Compass
    t_compass = Compass{k}.time(:);
    heading_compass = Compass{k}.heading(:);
    idx = t_compass >= t_start & t_compass <= t_end;
    compass_out = [t_compass(idx),heading_compass(idx)];
    %% 6.2 120 AUAX -> PVA
    pva_120 = [];
    if isfield(Alignment120(k),'auax') && ~isempty(Alignment120(k).auax)
        A = Alignment120(k).auax;
        idx = A(2,:) >= t_start & A(2,:) <= t_end;
        A = A(:,idx);
        if ~isempty(A)
            nav120 = A';
            heading_new = mod(-nav120(:,3),360);
            pva_120 = [ ...
                nav120(:,1:2), ...
                nav120(:,6:10), ...
                zeros(size(nav120,1),1), ...
                nav120(:,5), ...
                nav120(:,4), ...
                heading_new];
        end
    end
    %% 6.3 三套IMU：直接使用第3部分确认后的数据
    imu_120 = CompareIMU{k}.imu120;
    imu_830_binary = CompareIMU{k}.imu830_binary;
    imu_830_gpchcx = CompareIMU{k}.imu830_gpchcx;
    %% 6.4 静态三套IMU
    imu_120_static = [];
    imu_830_binary_static = [];
    imu_830_gpchcx_static = [];
    if isfield(CompareIMU{k},'imu120_static')
        imu_120_static = CompareIMU{k}.imu120_static;
    end
    if isfield(CompareIMU{k},'imu830_binary_static')
        imu_830_binary_static = CompareIMU{k}.imu830_binary_static;
    end
    if isfield(CompareIMU{k},'imu830_gpchcx_static')
        imu_830_gpchcx_static = CompareIMU{k}.imu830_gpchcx_static;
    end
    %% 6.5 830 PVA
    pva_830 = [];
    if isfield(Ref830{k},'pva') && ~isempty(Ref830{k}.pva)
        pva_830_all = Ref830{k}.pva;
        idx = pva_830_all(:,2) >= t_start & pva_830_all(:,2) <= t_end;
        pva_830 = pva_830_all(idx,:);
    end
    %% 6.6 830静态PVA
    pva_830_static = [];
    if isfield(Ref830{k},'pva_static') && ~isempty(Ref830{k}.pva_static)
        pva_830_static = Ref830{k}.pva_static;
        if isfield(CompareIMU{k},'static_time_range') && numel(CompareIMU{k}.static_time_range) == 2
            ts0 = CompareIMU{k}.static_time_range(1);
            ts1 = CompareIMU{k}.static_time_range(2);
            idx = pva_830_static(:,2) >= ts0 & pva_830_static(:,2) <= ts1;
            pva_830_static = pva_830_static(idx,:);
        end
    end
    %% 6.7 830静态GPS
    gps_830_static = [];
    if isfield(Ref830{k},'gps_static') && ~isempty(Ref830{k}.gps_static)
        gps_830_static = Ref830{k}.gps_static;
        if isfield(CompareIMU{k},'static_time_range') && numel(CompareIMU{k}.static_time_range) == 2
            ts0 = CompareIMU{k}.static_time_range(1);
            ts1 = CompareIMU{k}.static_time_range(2);
            idx = gps_830_static(:,1) >= ts0 & gps_830_static(:,1) <= ts1;
            gps_830_static = gps_830_static(idx,:);
        end
    end
    %% 6.8 830动态GPS
    gps_830 = [];
    if isfield(Ref830{k},'gps_nav') && ~isempty(Ref830{k}.gps_nav)
        gps_all = Ref830{k}.gps_nav;
        idx = gps_all(:,1) >= t_start & gps_all(:,1) <= t_end;
        gps_830 = gps_all(idx,:);
    elseif isfield(Ref830{k},'gpgga') && ~isempty(Ref830{k}.gpgga)
        gpgga = Ref830{k}.gpgga;
        if size(gpgga,1) >= 15
            data_date = [2025,12,7];
            [~,base_sec] = bjt_to_utc_week_seconds( ...
                data_date(1),data_date(2),data_date(3), ...
                gpgga(1,1)+8,gpgga(2,1),gpgga(3,1),0);
            total_s = gpgga(1,:)*3600+gpgga(2,:)*60+gpgga(3,:);
            utc_sec = (total_s-total_s(1))+base_sec+18;
            gps_mask = utc_sec >= t_start & utc_sec <= t_end & gpgga(10,:) == 4;
            gps_sub = gpgga(:,gps_mask)';
            if ~isempty(gps_sub)
                lat = gps_sub(:,4)+gps_sub(:,5)/60;
                lon = gps_sub(:,7)+gps_sub(:,8)/60;
                alt = gps_sub(:,13)+gps_sub(:,15);
                gps_time = utc_sec(gps_mask)';
                gps_830 = [gps_time,gps_time-t_start,lat,lon,alt,gps_sub(:,10)];
            end
        end
    end
    %% 6.9 数据范围检查
    fprintf('\n导出数据：\n');
    print_range('Compass',compass_out,1);
    print_range('120 PVA',pva_120,2);
    print_range('120 IMU',imu_120,1);
    print_range('830 PVA',pva_830,2);
    print_range('830 binary IMU',imu_830_binary,1);
    print_range('830 GPCHCX IMU',imu_830_gpchcx,1);
    print_range('830 GPS',gps_830,1);
    fprintf('\n静态导出数据：\n');
    print_range('120 static IMU',imu_120_static,1);
    print_range('830 binary static',imu_830_binary_static,1);
    print_range('830 GPCHCX static',imu_830_gpchcx_static,1);
    print_range('830 static PVA',pva_830_static,2);
    print_range('830 static GPS',gps_830_static,1);
    %% 6.10 写入TXT
    save_list = {
        pva_120,             'pva_file.txt'
        imu_120,             'IMU_120.txt'
        pva_830,             'pva_830.txt'
        imu_830_binary,      'imu_830.txt'
        imu_830_binary,      'imu_830_binary.txt'
        imu_830_gpchcx,      'imu_830_gpchcx.txt'
        gps_830,             'gps_830.txt'
        compass_out,               'compass.txt'
        imu_120_static,            'IMU_120_static.txt'
        imu_830_binary_static,     'imu_830_static.txt'
        imu_830_binary_static,     'imu_830_binary_static.txt'
        imu_830_gpchcx_static,     'imu_830_gpchcx_static.txt'
        pva_830_static,            'pva_830_static.txt'
        gps_830_static,            'gps_830_static.txt'
    };
    fprintf('\n写入文件：\n');
    for i = 1:size(save_list,1)
        data = save_list{i,1};
        filename = save_list{i,2};
        if isempty(data)
            fprintf('  [SKIP] %s：无数据\n',filename);
            continue;
        end
        filepath = fullfile(out_dir{k},filename);
        writematrix(data,filepath,'Delimiter',' ');
        fprintf('  [OK] %s\n',filename);
    end
end
fprintf('\n============================================================\n');
fprintf('比较 + 两个有效动态数据集导出完成\n');
fprintf('case-01：%s\n',out_dir{1});
fprintf('case-02：%s\n',out_dir{2});
fprintf('============================================================\n');

%% ========================================================================
%% 局部函数
% ========================================================================

function [t,sig] = prepare_imu_data(data,time_col,signal_cols)
%PREPARE_IMU_DATA 提取IMU时间与信号，并完成基本清理。
%
% 输入：
%   data        IMU矩阵
%   time_col    时间列
%   signal_cols 信号列
%
% 输出：
%   t           排序、去重后的时间
%   sig         与t对应的信号

if isempty(data)
    t = [];
    sig = [];
    return;
end

if size(data,2) < max([time_col,signal_cols])
    error('IMU数据列数不足：当前%d列，需要至少%d列。', ...
        size(data,2),max([time_col,signal_cols]));
end

t = data(:,time_col);
sig = data(:,signal_cols);

valid = ...
    isfinite(t) & ...
    all(isfinite(sig),2);

t = t(valid);
sig = sig(valid,:);

[t,idx] = sort(t);
sig = sig(idx,:);

[t,idx_unique] = unique(t,'stable');
sig = sig(idx_unique,:);

if isempty(t)
    error('IMU清理后没有有效数据。');
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

function print_range(name,data,time_col)
if isempty(data)
    fprintf('  %-18s：无数据\n',name);
    return;
end
if size(data,2) < time_col
    fprintf('  %-18s：%d × %d，时间列不存在\n',name,size(data,1),size(data,2));
    return;
end
t = data(:,time_col);
t = t(isfinite(t));
if isempty(t)
    fprintf('  %-18s：%d × %d，无有效时间\n',name,size(data,1),size(data,2));
else
    fprintf('  %-18s：%d × %d，%.3f ~ %.3f\n', ...
        name,size(data,1),size(data,2),min(t),max(t));
end
end
