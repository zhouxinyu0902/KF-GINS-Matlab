function [DistData, TT_LBL, tt_lbl, USBL_sync] = sync_nav_time(DistData_rw, PTSAGShip_rw, PTSAGHov_rw, PTSAX_rw, PIXOG_rw, cfg, isfig)
% SYNC_NAV_TIME 同步并截取导航数据的时间段，处理异常时间戳
%
% 输入:
%   DistData_rw 等 : load_nav_raw_data 输出的原始数据矩阵
%   cfg            : 时间配置结构体 (包含 TimeLBL, TimeUSBL, ATdelay 等)
%   isfig          : 是否绘制时间截取和对齐的对比图 (1/0)
%
% 输出:
%   DistData       : 时间同步并处理异常后的 AUV/LBL 综合数据矩阵
%   TT_LBL, tt_lbl : 对应的绝对时间和相对时间向量
%   USBL_sync      : 包含处理后所有 USBL 数据及其时间轴的结构体

    if nargin < 7, isfig = 0; end
    fprintf('开始处理时间同步与异常时间戳修复...\n');

    %% 1. 航行器本体数据 (AUV & LBL) 时间处理
    TimeZone_AUV = 0; % 航行控制计算机设定的时区
    DistData = DistData_rw;
    
    % 第一次查询并截取时间段
    [TT, ~, DistData] = timepro(cfg.TimeLBL, TimeZone_AUV, DistData, 'LBL');
    
    % 修复只出现了一次的时间点 (复制补齐)
    [~, rptedN1, ~, ~, ~] = repeated(TT); 
    if ~isempty(rptedN1)
        B = zeros(size(DistData, 1), length(DistData) + length(rptedN1));
        B(:, 1:length(DistData)) = DistData;
        for i = 1:length(rptedN1)
            B(:, 1 : rptedN1(i)+i-1) = B(:, 1 : rptedN1(i)+i-1);
            B(:, rptedN1(i)+i) = DistData(:, rptedN1(i));
            B(:, rptedN1(i)+i+1 : end-length(rptedN1)+i) = DistData(:, rptedN1(i)+1 : end);
        end
        [TT, ~, B] = timepro(cfg.TimeLBL, TimeZone_AUV, B, 'LBL');
    else
        B = DistData;
    end
    
    % 删除出现了三次的时间点 (剔除冗余)
    [~, ~, rptedN3, ~, ~] = repeated(TT);
    if ~isempty(rptedN3)
        colsToKeep = setdiff(1:size(B, 2), rptedN3);
        DistData = B(:, colsToKeep);
    else
        DistData = B;
    end
    
    % 最终时间生成
    [TT_LBL, tt_lbl, DistData] = timepro(cfg.TimeLBL, TimeZone_AUV, DistData, 'LBL');

    %% 2. 超短基线数据 (USBL) 时间处理
    TimeZone_USBL = 8; % USBL 的时区
    
    [USBL_sync.TimePTSAGShip, USBL_sync.ttPTSAGShip, USBL_sync.PTSAGShip] = ...
        timepro(cfg.TimeUSBL, TimeZone_USBL, PTSAGShip_rw, 'USBL');
        
    [USBL_sync.TimePTSAGHov,  USBL_sync.ttPTSAGHov,  USBL_sync.PTSAGHov] = ...
        timepro(cfg.TimeUSBL, TimeZone_USBL, PTSAGHov_rw, 'USBL');
        
    [USBL_sync.TimePTSAX,     USBL_sync.ttPTSAX,     USBL_sync.PTSAX] = ...
        timepro(cfg.TimeUSBL, TimeZone_USBL, PTSAX_rw, 'USBL');
        
    [USBL_sync.TimePIXOG,     USBL_sync.ttPIXOG,     USBL_sync.PIXOG] = ...
        timepro(cfg.TimeUSBL, TimeZone_USBL, PIXOG_rw, 'USBL');

    fprintf('时间同步与截取完成！\n\n');

    %% 3. 可视化校验 (如果 isfig == 1)
    if isfig
        % --- 图 1: AUV 数据截取前后对比 (DVL 速度) ---
        myfigurestartup(5,2,'paper');
        TT_raw_AUV = DistData_rw(end-2,:)*3600 + DistData_rw(end-1,:)*60 + DistData_rw(end,:);
        TT_new_AUV = DistData(end-2,:)*3600 + DistData(end-1,:)*60 + DistData(end,:);
        
        subplot(1, 2, 1);
        plot(TT_raw_AUV, DistData_rw(7,:), '.', 'MarkerSize', 6); hold on;
        plot(TT_new_AUV, DistData(7,:), '.', 'MarkerSize', 4);
        xlim([TT_raw_AUV(1), TT_raw_AUV(end)]); ylim([-0.2, 0.2]);
        ConvertXAxisTime; xygo('hh mm ss', 'velocity-x (m/s)');
        legend('All Time', 'Chosen Time', 'Location', 'best'); DeciPoin(0,2);
        
        subplot(1, 2, 2);
        plot(TT_raw_AUV, DistData_rw(8,:), '.', 'MarkerSize', 6); hold on;
        plot(TT_new_AUV, DistData(8,:), '.', 'MarkerSize', 4);
        xlim([TT_raw_AUV(1), TT_raw_AUV(end)]); ylim([-0.2, 1]);
        ConvertXAxisTime; xygo('hh mm ss', 'velocity-y (m/s)');
        legend('All Time', 'Chosen Time', 'Location', 'best'); DeciPoin(0,1);

        % --- 图 2: USBL 数据截取前后对比 (纬度经度) ---
        myfigurestartup(5,2,'paper');
        TT_raw_USBL = PTSAGHov_rw(end-2,:)*3600 + PTSAGHov_rw(end-1,:)*60 + PTSAGHov_rw(end,:) + 8*3600;
        TT_new_USBL = USBL_sync.PTSAGHov(end-2,:)*3600 + USBL_sync.PTSAGHov(end-1,:)*60 + USBL_sync.PTSAGHov(end,:) + 8*3600;
        
        subplot(1, 2, 1);
        plot(TT_raw_USBL, PTSAGHov_rw(3,:), '.', 'MarkerSize', 6); hold on;
        plot(TT_new_USBL, USBL_sync.PTSAGHov(3,:), '.', 'MarkerSize', 4);
        xlim([TT_raw_USBL(1), TT_raw_USBL(end)]);
        ConvertXAxisTime; xygo('hh mm ss', 'lat (deg)'); legend('All', 'Chosen'); DeciPoin(0,2);
        
        subplot(1, 2, 2);
        plot(TT_raw_USBL, PTSAGHov_rw(4,:), '.', 'MarkerSize', 6); hold on;
        plot(TT_new_USBL, USBL_sync.PTSAGHov(4,:), '.', 'MarkerSize', 4);
        xlim([TT_raw_USBL(1), TT_raw_USBL(end)]);
        ConvertXAxisTime; xygo('hh mm ss', 'lon (deg)'); legend('All', 'Chosen'); DeciPoin(0,3);

        % --- 图 3: 超短基线与长基线对齐校验 ---
        myfigurestartup(7,3,'paper');
        subplot(1, 2, 1);
        plot(USBL_sync.ttPTSAGShip, USBL_sync.PTSAGShip(3,:), 'DisplayName', 'USBL-raw'); hold on;
        plot(tt_lbl + cfg.ATdelay, DistData(10,:), '.', 'DisplayName', 'AUV');
        plot(tt_lbl, DistData(10,:), '.', 'DisplayName', 'AUV-processed');
        xygo('t/s', 'lat/deg'); xlim([0 8800]); legend('Location', 'best'); DeciPoin(0,3);
        
        subplot(1, 2, 2);
        plot(USBL_sync.ttPTSAGShip, USBL_sync.PTSAGShip(3,:)); hold on;
        plot(tt_lbl + cfg.ATdelay, DistData(10,:), '.');
        plot(tt_lbl, DistData(10,:), '.');
        xygo('t/s', 'lat/deg'); xlim([2000 3500]); DeciPoin(0,3);
    end
end