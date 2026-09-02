clear;
clc;
close all;
%% ========================================================================
% 2025-12-07 830参考数据读取与截取
%
% 思路：
%
% compass_1207.mat
%       ↓
% Compass{1}.time → 下午实验时间范围
% Compass{2}.time → 晚上实验时间范围
%
% 830原始数据：
%
% 下午：
%   043941_4752450.dat
%   083942_4752450.dat
%
% 晚上：
%   110223_4752450.dat
%   150224_4752450.dat
%
%       ↓
% 分别合并
%       ↓
% 删除无效830数据
%       ↓
% 根据Compass时间范围截取
%       ↓
% visualize_gpchcx_navigation_results
%       ↓
% Ref830{1}
% Ref830{2}
%
% ========================================================================
%% ========================================================================
%% 0. 路径
% ========================================================================
% 830原始数据目录
raw_dir = ...
    'D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1207-longtime\raw';
% 临时MAT数据目录
temp_dir = ...
    'D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1207-longtime\temp';
%% 罗盘MAT
compass_file = ...
    fullfile(temp_dir,'compass.mat');
%% 830保存文件
save_file = ...
    fullfile(temp_dir,'ref830_1207.mat');
%% ========================================================================
%% 1. 读取罗盘时间
% ========================================================================
fprintf('\n');
fprintf('============================================================\n');
fprintf('读取罗盘时间范围\n');
fprintf('============================================================\n');
load(compass_file, ...
    'Compass', ...
    'experiment_time');
if numel(Compass) ~= 2
    error('Compass 应包含两段实验数据。');
end
for k = 1:2
    fprintf('\n实验 %d：%s\n', ...
        k,Compass{k}.name);
    fprintf('Compass GPS：%.3f ~ %.3f\n', ...
        Compass{k}.time(1), ...
        Compass{k}.time(end));
    fprintf('持续时间：%.2f s = %.2f min\n', ...
        Compass{k}.time(end)-Compass{k}.time(1), ...
        (Compass{k}.time(end)-Compass{k}.time(1))/60);
end
%% ========================================================================
%% 2. 配置830文件
% ========================================================================
REF_FILE = cell(2,1);
% -------------------------------------------------------------------------
% 下午
% -------------------------------------------------------------------------
REF_FILE{1} = {
    '043941_4752450.dat'
    '083942_4752450.dat'
    };
% -------------------------------------------------------------------------
% 晚上
% -------------------------------------------------------------------------
REF_FILE{2} = {
    '110223_4752450.dat'
    '150224_4752450.dat'
    };
%% ========================================================================
%% 3. 读取并处理830
% ========================================================================
Ref830 = cell(2,1);
fprintf('\n');
fprintf('============================================================\n');
fprintf('开始处理830参考数据\n');
fprintf('============================================================\n');
for k = 1:2
    fprintf('\n');
    fprintf('============================================================\n');
    fprintf('实验 %d：%s\n',k,Compass{k}.name);
    fprintf('============================================================\n');
    %% ====================================================================
    % 3.1 读取该次实验对应的全部830文件
    % =====================================================================
    GPCHCX_all = [];
    for j = 1:length(REF_FILE{k})
        filename = ...
            fullfile(raw_dir,REF_FILE{k}{j});
        fprintf('\n读取830：%s\n',filename);
        if ~exist(filename,'file')
            error('830文件不存在：%s',filename);
        end
        GPCHCX_tmp = ...
            read_gpchcx_430_830(filename);
        fprintf('  数据维数：%d × %d\n', ...
            size(GPCHCX_tmp,1), ...
            size(GPCHCX_tmp,2));
        GPCHCX_all = ...
            [GPCHCX_all,GPCHCX_tmp];
    end
    fprintf('\n合并后830数据：%d 个历元\n', ...
        size(GPCHCX_all,2));
    %% ====================================================================
    % 3.2 删除明显无效830数据
    %
    % 保持你原程序中的逻辑：
    %
    % GPCHCX(21,:) ~= 0
    % 36 <= GPCHCX(12,:) <= 37
    % GPCHCX(2,:) > 100
    %
    % =====================================================================
    state_valid = ...
        GPCHCX_all(21,:) ~= 0;
    lat_valid = ...
        GPCHCX_all(12,:) >= 36 & ...
        GPCHCX_all(12,:) <= 37;
    time_valid = ...
        isfinite(GPCHCX_all(2,:)) & ...
        GPCHCX_all(2,:) > 100;
    valid = ...
        state_valid & ...
        lat_valid & ...
        time_valid;
    fprintf('\n830数据筛选：\n');
    fprintf('  原始：%d\n', ...
        size(GPCHCX_all,2));
    fprintf('  状态无效：%d\n', ...
        sum(~state_valid));
    fprintf('  纬度异常：%d\n', ...
        sum(~lat_valid));
    fprintf('  时间无效：%d\n', ...
        sum(~time_valid));
    GPCHCX_valid = ...
        GPCHCX_all(:,valid);
    fprintf('  最终有效：%d\n', ...
        size(GPCHCX_valid,2));
    %% ====================================================================
    % 3.3 按GPS时间排序
    % =====================================================================
    [~,idx_sort] = ...
        sort(GPCHCX_valid(2,:));
    GPCHCX_valid = ...
        GPCHCX_valid(:,idx_sort);
    %% --------------------------------------------------------------------
    % 去除重复时间
    % ---------------------------------------------------------------------
    [~,idx_unique] = ...
        unique( ...
        GPCHCX_valid(2,:), ...
        'stable');
    GPCHCX_valid = ...
        GPCHCX_valid(:,idx_unique);
    fprintf('\n830完整有效时间：\n');
    fprintf('  %.3f ~ %.3f\n', ...
        GPCHCX_valid(2,1), ...
        GPCHCX_valid(2,end));
    fprintf('  持续 %.2f min\n', ...
        (GPCHCX_valid(2,end)-GPCHCX_valid(2,1))/60);
    %% ====================================================================
    % 3.4 根据罗盘时间范围截取830
    % =====================================================================
    t_start = ...
        Compass{k}.time(1);
    t_end = ...
        Compass{k}.time(end);
    idx_time = ...
        GPCHCX_valid(2,:) >= t_start & ...
        GPCHCX_valid(2,:) <= t_end;
    GPCHCX_select = ...
        GPCHCX_valid(:,idx_time);
    if isempty(GPCHCX_select)
        error( ...
            '实验%d的830数据与Compass没有时间重叠。',k);
    end
    fprintf('\n根据Compass截取830：\n');
    fprintf('  Compass：%.3f ~ %.3f\n', ...
        t_start,t_end);
    fprintf('  830：    %.3f ~ %.3f\n', ...
        GPCHCX_select(2,1), ...
        GPCHCX_select(2,end));
    fprintf('  830历元：%d\n', ...
        size(GPCHCX_select,2));
    %% ====================================================================
    % 3.5 转换成标准830 PVA
    % =====================================================================
    [imu_830,pva_830] = ...
        visualize_gpchcx_navigation_results( ...
        GPCHCX_select);
    imu_830 = imuRFU2FRD(imu_830);
    
    %% ====================================================================
    % 3.6 保存为统一结构
    % =====================================================================
    Ref830{k}.name = ...
        Compass{k}.name;
    Ref830{k}.time_range = [ ...
        t_start, ...
        t_end];
    % 截取后的原始GPCHCX
    Ref830{k}.gpchcx = ...
        GPCHCX_select;
    % visualize函数输出
    Ref830{k}.imu = ...
        imu_830;
    Ref830{k}.pva = ...
        pva_830;
    %% --------------------------------------------------------------------
    % 根据你之前PVA定义：
    %
    % 第2列：GPS周秒
    % 第11列：航向角
    % ---------------------------------------------------------------------
    Ref830{k}.time = ...
        pva_830(:,2)';
    Ref830{k}.heading = ...
        pva_830(:,11)';
    %% ====================================================================
    % 3.7 输出最终结果
    % =====================================================================
    fprintf('\n最终830 PVA：\n');
    fprintf('  历元数：%d\n', ...
        size(pva_830,1));
    fprintf('  时间：%.3f ~ %.3f\n', ...
        Ref830{k}.time(1), ...
        Ref830{k}.time(end));
    fprintf('  持续：%.2f s = %.2f min\n', ...
        Ref830{k}.time(end)-Ref830{k}.time(1), ...
        (Ref830{k}.time(end)-Ref830{k}.time(1))/60);
end
%% ========================================================================
%% 4. 检查Compass和830时间覆盖
% ========================================================================
figure( ...
    'Name','Compass与830时间范围', ...
    'NumberTitle','off');
tiledlayout(2,1, ...
    'TileSpacing','compact', ...
    'Padding','compact');
for k = 1:2
    nexttile;
    hold on;
    grid on;
    t0 = Compass{k}.time(1);
    %% Compass
    plot( ...
        [ ...
        Compass{k}.time(1), ...
        Compass{k}.time(end)] - t0, ...
        [2 2], ...
        'LineWidth',6);
    %% 830
    plot( ...
        [ ...
        Ref830{k}.time(1), ...
        Ref830{k}.time(end)] - t0, ...
        [1 1], ...
        'LineWidth',6);
    yticks([1 2]);
    yticklabels({ ...
        '830', ...
        'Compass'});
    xlabel('相对Compass起点时间 / s');
    title(sprintf( ...
        '%s 时间覆盖', ...
        Compass{k}.name));
    ylim([0.5 2.5]);
end
%% ========================================================================
%% 5. 简单画830航向
%
% 这里只确认数据是否正常。
% 现在还不和罗盘比较。
% ========================================================================
figure( ...
    'Name','830参考航向', ...
    'NumberTitle','off');
tiledlayout(2,1, ...
    'TileSpacing','compact', ...
    'Padding','compact');
for k = 1:2
    nexttile;
    t = ...
        Ref830{k}.time ...
        - Ref830{k}.time(1);
    plot( ...
        t, ...
        Ref830{k}.heading, ...
        'LineWidth',1);
    grid on;
    xlabel('运行时间 / s');
    ylabel('830航向 / °');
    title(sprintf( ...
        '%s 830参考数据', ...
        Ref830{k}.name));
end
%% ========================================================================
%% 6. 保存
% ========================================================================
save( ...
    save_file, ...
    'Ref830');
fprintf('\n');
fprintf('============================================================\n');
fprintf('830参考数据保存完成\n');
fprintf('============================================================\n');
fprintf('%s\n',save_file);