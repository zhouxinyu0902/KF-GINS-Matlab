%% =========================================================================
% 多传感器数据纯导出批处理流水线
% 功能：基于陀螺波形对齐 -> 截取原生帧率数据 -> 导出 pva/imu/gps/std .txt
% 新增：支持截取运动前静止段数据，用于初始对准 (Initial Alignment)
% =========================================================================

% target_datasets = 5; % 选择要导出的数据组

for target_datasets = 1:8
    % ==================== 1. 全局配置 ====================
    clearvars -except target_datasets 
    clc;
    cfg.inDir  = 'D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-01\'; 
    cfg.leapSec     = 18;
    cfg.fs_120      = 200;
    
    % ==================== 2. 批处理主循环 ====================
    for ds_idx = target_datasets
        fprintf('\n======================================================\n');
        fprintf('[Extract TXT]开始导出第 %d 组数据 (Round %d)...\n', ds_idx, ds_idx);
        fprintf('======================================================\n');
        
        % --- 2.1 加载数据 ---
        inDir = fullfile(cfg.inDir, sprintf('case-0%d', ds_idx),'mat');
        try
            load(fullfile(inDir, '830.mat'));
            load(fullfile(inDir, '430.mat'));
            load(fullfile(inDir, 'auax.mat'));
            load(fullfile(inDir, 'stdimu.mat'));
        catch ME
            warning('Round %d 数据加载失败，跳过... (%s)', ds_idx, ME.message);
            continue;
        end
        outDir = fullfile(cfg.inDir, sprintf('case-0%d', ds_idx),'input');
        if ~exist(outDir, 'dir'), mkdir(outDir); end
        
        % 日期配置 (用于 GPGGA 时间计算)
        if ismember(ds_idx, [1, 2])
            data_date = [2025, 9, 28];
        elseif ismember(ds_idx, [3, 4, 5, 6])
            data_date = [2025, 9, 29];
        else
            data_date = [2025, 9, 30];
        end
        
        % --- 2.2 处理 120 导航数据 (参考你的逻辑：默认用 RS) ---
        if exist('AUAX_RS', 'var') && ~isempty(AUAX_RS)
            AUAX_USED = AUAX_RS;
        else
            AUAX_USED = AUAX_file;
        end
        AUAX_USED(2,:) = AUAX_USED(2,:) + cfg.leapSec;
        idx_auax = conclude_auax_headless(AUAX_USED);
        nav_120 = AUAX_USED(:, idx_auax.nav)';
        
        % 还原 visualize_auax_navigation_results 提取的 pva 和 imu
        heading_new = mod(-nav_120(:,3), 360);
        pva_120 = [nav_120(:,1:2), nav_120(:,6:10), zeros(size(nav_120,1),1), nav_120(:,5), nav_120(:,4), heading_new];
        imu_120_ref = nav_120(:, 12:14); % 提取 gyro_x, y, z 用于对齐
        
        % --- 2.3 处理 120 IMU 原始数据 (参考你的逻辑：默认用 file) ---
        if exist('IMU_FUR_file_inte', 'var') && ~isempty(IMU_FUR_file_inte)
            IMU_120_RAW = IMU_FUR_file_inte;
        elseif exist('IMU_FUR_file', 'var') && ~isempty(IMU_FUR_file)
            IMU_120_RAW = IMU_FUR_file;
        else
            IMU_120_RAW = IMU_FUR_RS;
        end
        
        % 使用三轴陀螺向量模差寻找起始点
        gyro_ref = imu_120_ref(1, 1:3);
        [~, startid] = min(sum((IMU_120_RAW(:,1:3) - gyro_ref).^2, 2));
        
        % 修复：在循环内部应该判断 ds_idx，而不是 target_datasets
        if ds_idx == 2
            startid = startid + 5000 - 150;
        elseif ds_idx == 8
            startid = startid - 40000 + 1100;
        end
        
        % ================== 🌟 新增：静止对准数据截取模块 🌟 ==================
        num_static_points = 430000; % 截取约 35.8 分钟 (200Hz) 或 71.6 分钟 (100Hz) 的静止数据
        static_start = max(1, startid - num_static_points); % 防止越界
        
        % 1. 计算全局时间 (先不切割，保证静止段和运动段时间完全连续)
        IMU_120_RAW(:,7) = IMU_120_RAW(:,7) / cfg.fs_120; % 计数转秒
        ref_time_counter = IMU_120_RAW(startid, 7);       % 记录运动起点的本地秒
        IMU_120_RAW(:,8) = nav_120(1,2) + (IMU_120_RAW(:,7) - ref_time_counter); % 绝对时间戳对齐
        
        % 2. 切割【静止段】数据
        IMU_120_STATIC = IMU_120_RAW(static_start : startid-1, :);
        t_static_start = IMU_120_STATIC(1,8);
        t_static_end   = IMU_120_STATIC(end,8);
        IMU_120_STATIC_FRD = imuFUR2FRD(IMU_120_STATIC);
        
        fprintf('正在提取 430/830 的【静止对准段】数据...\n');
        data430_static = extract_novatel_headless(GPCHCX_430, IMU_DATA_430, GPGGA_430, t_static_start, t_static_end, cfg, data_date);
        data830_static = extract_novatel_headless(GPCHCX_830, IMU_DATA_830, GPGGA_830, t_static_start, t_static_end, cfg, data_date);
        
        % 3. 切割【运动段】数据 (原有逻辑)
        IMU_120_RAW = IMU_120_RAW(startid:end, :);
        t_start = IMU_120_RAW(1,8);
        t_end   = IMU_120_RAW(end,8);
        IMU_120_FRD = imuFUR2FRD(IMU_120_RAW);
        % ====================================================================

        % --- 2.4 处理 430 & 830 【运动段】数据 (无头模式模块化) ---
        fprintf('正在提取 430/830 的【动态运动段】数据...\n');
        data430 = extract_novatel_headless(GPCHCX_430, IMU_DATA_430, GPGGA_430, t_start, t_end, cfg, data_date);
        data830 = extract_novatel_headless(GPCHCX_830, IMU_DATA_830, GPGGA_830, t_start, t_end, cfg, data_date);
        
        % --- 2.5 动态组装导出列表 ---
        fprintf('正在将数据写入: %s\n', outDir);
        
        % 基础列表：运动段 120 + 静止段 120
        save_list = {
            pva_120,             'pva_file.txt'; 
            IMU_120_FRD,         'IMU_120.txt';
            IMU_120_STATIC_FRD,  'IMU_120_static.txt'; % 静止段 120 IMU
        };

        % 装载 430 数据 (动态段 + 静止段)
        if ~isempty(data430.pva)
            save_list(end+1:end+4, :) = {
                data430.pva,     'pva_430.txt';
                data430.imu,     'imu_430.txt';
                data430.gps,     'gps_430.txt';
                data430.posstd,  'std_430.txt'  
            };
        end
        if ~isempty(data430_static.pva)
            save_list(end+1:end+4, :) = {
                data430_static.pva,     'pva_430_static.txt';
                data430_static.imu,     'imu_430_static.txt';
                data430_static.gps,     'gps_430_static.txt';
                data430_static.posstd,  'std_430_static.txt'  
            };
        end

        % 装载 830 数据 (动态段 + 静止段)
        if ~isempty(data830.pva)
            save_list(end+1:end+4, :) = {
                data830.pva,     'pva_830.txt';
                data830.imu,     'imu_830.txt';
                data830.gps,     'gps_830.txt';
                data830.posstd,  'std_830.txt'
            };
        end
        if ~isempty(data830_static.pva)
            save_list(end+1:end+4, :) = {
                data830_static.pva,     'pva_830_static.txt';
                data830_static.imu,     'imu_830_static.txt';
                data830_static.gps,     'gps_830_static.txt';
                data830_static.posstd,  'std_830_static.txt'
            };
        end

        % 执行写入
        for i = 1:size(save_list, 1)
            out_filepath = fullfile(outDir, save_list{i,2});
            try
                writematrix(save_list{i,1}, out_filepath, 'Delimiter', ' ');
                fprintf('  [OK] %s\n', save_list{i,2});
            catch ME
                warning('  [FAIL] 无法写入 %s: %s', save_list{i,2}, ME.message);
            end
        end
    end
    fprintf('\n🎉 批量导出工作全部完成！\n');
end

%% =========================================================================
% 内部无头封装函数 (严格还原 visualize_* 输出格式，移除插值)
% =========================================================================
function index = conclude_auax_headless(AUAX)
    index.nav = find(AUAX(end-2,:) == hex2dec('4300'));
end

function data = extract_novatel_headless(gpchcx, imu_raw, gpgga, t_start, t_end, cfg, data_date)
    data.pva = []; data.imu = []; data.gps = []; data.posstd = [];
    if isempty(gpchcx) || isempty(imu_raw), return; end
    
    % ==============================================================
    % 1. GPCHCX (PVA & POSSTD) 提取与异常值插值修复
    % ==============================================================
    time_mask = (gpchcx(2,:) >= t_start) & (gpchcx(2,:) <= t_end);
    nav_sub = gpchcx(:, time_mask)';
    
    if ~isempty(nav_sub)
        % pva = [week, sec, lat, lon, alt, vel_n, vel_e, -vel_u, roll, pitch, heading]
        data.pva = [nav_sub(:,1:2), nav_sub(:,12:14), nav_sub(:,16), nav_sub(:,15), -nav_sub(:,17), nav_sub(:,5), nav_sub(:,4), nav_sub(:,3)];
        % posstd = [gnss_seconds, Lattitude_std, Longitude_std, Altitude_std, Vn_std, Ve_std, Vu_std]
        data.posstd = [nav_sub(:,2), nav_sub(:,24:29)];
        
        % ----- 🌟 剔除纬度异常点并线性插值 -----
        valid_mask = (data.pva(:,3) <= 37) & (data.pva(:,3) >= 36);
        
        if any(~valid_mask) && sum(valid_mask) > 1
            t_all = data.pva(:, 2);       
            t_valid_raw = t_all(valid_mask);  
            data_valid_pva = data.pva(valid_mask, 3:end);
            data_valid_std = data.posstd(valid_mask, 2:end);
            
            % 【核心修复】：对时间戳进行去重，防止 interp1 报 "采样点必须唯一" 错误
            [t_valid_unique, unique_idx] = unique(t_valid_raw);
            data_valid_pva_unique = data_valid_pva(unique_idx, :);
            data_valid_std_unique = data_valid_std(unique_idx, :);
            
            % 确保去重后仍有两个以上的数据点可供插值
            if length(t_valid_unique) > 1
                % 1. 对 PVA 矩阵插值
                data.pva(:, 3:end) = interp1(t_valid_unique, data_valid_pva_unique, t_all, 'linear', 'extrap');
                % 2. 对 STD 矩阵插值
                data.posstd(:, 2:end) = interp1(t_valid_unique, data_valid_std_unique, t_all, 'linear', 'extrap');
            else
                data.pva = []; data.posstd = [];
            end
            
        elseif sum(valid_mask) <= 1
            data.pva = []; data.posstd = [];
        end
    end
    
    % ==============================================================
    % 2. GPGGA (GPS)
    % ==============================================================
    if nargin >= 3 && ~isempty(gpgga)
        [~, base_sec] = bjt_to_utc_week_seconds(data_date(1), data_date(2), data_date(3), gpgga(1,1) + 8, gpgga(2, 1), gpgga(3, 1), 0);
        total_s = gpgga(1,:)*3600 + gpgga(2,:)*60 + gpgga(3,:);
        utc_sec = (total_s - total_s(1)) + base_sec + cfg.leapSec;
        gps_mask = (utc_sec >= t_start) & (utc_sec <= t_end) & (gpgga(10,:) == 4);
        gps_sub = gpgga(:, gps_mask)';
        if ~isempty(gps_sub)
            lat = gps_sub(:,4) + gps_sub(:,5)/60;
            lon = gps_sub(:,7) + gps_sub(:,8)/60;
            alt = gps_sub(:,13) + gps_sub(:,15);
            data.gps = [utc_sec(gps_mask)', utc_sec(gps_mask)' - utc_sec(1), lat, lon, alt, gps_sub(:,10)];
        end
    end
    
    % ==============================================================
    % 3. IMU (RFU -> FRD)
    % ==============================================================
    imu_mask = (imu_raw(:,7) >= t_start) & (imu_raw(:,7) <= t_end);
    imu_sub = imu_raw(imu_mask, :);
    if ~isempty(imu_sub)
        imu_rfu = zeros(size(imu_sub, 1), 7);
        imu_rfu(:,1:3) = imu_sub(:,1:3) * (pi/180);
        imu_rfu(:,4:6) = imu_sub(:,4:6) * 9.80665;
        imu_rfu(:,[2,5]) = -imu_rfu(:,[2,5]);
        imu_rfu(:,7) = imu_sub(:,7);
        data.imu = imuRFU2FRD(imu_rfu);
    end
end