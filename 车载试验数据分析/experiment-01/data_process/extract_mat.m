%% =========================================================================
% 多传感器数据读取批处理脚本 (按 Round 自动分类输出)
% =========================================================================
clear; clc;

% ==================== 1. 全局配置 ====================
% 选择你要处理的数据组别
target_datasets = 8; 

% 基础输入/输出路径
cfg.inDir  = 'D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-01\'; 


% 120设备的帧头
frameHeader = [235, 144, 32]; 

% ==================== 2. 批处理主循环 ====================
for ds_idx = target_datasets
    fprintf('\n======================================================\n');
    fprintf('[Extract Mat]开始处理第 %d 组数据 (Round %d)...\n', ds_idx, ds_idx);
    fprintf('======================================================\n');
    
    % --- 2.1 清空并初始化当次循环的变量 ---
    file_imu_file = ''; file_imu_file1 = ''; file_imu_rs = '';
    file_auax_file = ''; file_auax_rs = '';
    file_430_imu = ''; file_430_gpchcx = ''; file_430_gpgga = '';
    file_830_imu = ''; file_830_gpchcx = ''; file_830_gpgga = '';
    
    IMU_FUR_file = []; IMU_FUR_file1 = []; IMU_FUR_RS = []; IMU_FUR_file_inte = [];
    AUAX_file = []; AUAX_RS = [];
    
    % 输出路径
    outDir = fullfile(cfg.inDir, sprintf('case-0%d', ds_idx),'mat');
    if ~exist(outDir, 'dir'), mkdir(outDir); end
    subDir = fullfile(sprintf('case-0%d', ds_idx),'raw');
    % --- 2.2 设定子文件夹与文件名 (保持纯净) ---
    switch ds_idx
    case 1
        file_imu_file  = 'STDIMU_0928_1706~1901.dat';
        file_auax_file = 'AUAX_0928_1706~1746(有乱码).dat';

        file_430_imu    = '0928_1700~1903_430_binary.dat';
        file_430_gpchcx = '0928_1700~1903_430_GPCHCX.txt';
        file_430_gpgga  = '0928_1700~1903_430_GPGGA.txt';

        file_830_imu    = '0928_1700~1903_830_binary.dat';
        file_830_gpchcx = '0928_1700~1903_830_GPCHCX.txt';
        file_830_gpgga  = '0928_1700~1903_830_GPGGA.txt';

    case 2
        file_imu_file  = 'STDIMU_0928_1913~2006.dat';
        file_auax_file = 'AUAX_0928_1913~1936(有乱码).dat';

        file_430_imu    = '0928_1904~2006_430_binary.dat';
        file_430_gpchcx = '0928_1904~2006_430_GPCHCX.txt';
        file_430_gpgga  = '0928_1904~2006_430_GPGGA.txt';

        file_830_imu    = '0928_1904~2006_830_binary.dat';
        file_830_gpchcx = '0928_1904~2006_830_GPCHCX.txt';
        file_830_gpgga  = '0928_1904~2006_830_GPGGA.txt';

    case 3
        file_imu_file  = 'STDIMU_0929_0951~1035.dat';
        file_auax_file = 'AUAX_0929_0951~1035.dat';

        file_430_imu    = '0929_0950~1034_430_binary.dat';
        file_430_gpchcx = '0929_0950~1034_430_GPCHCX.txt';
        file_430_gpgga  = '0929_0950~1034_430_GPGGA.txt';

        file_830_imu    = '0929_0950~1034_830_binary.dat';
        file_830_gpchcx = '0929_0950~1034_830_GPCHCX.txt';
        file_830_gpgga  = '0929_0950~1034_830_GPGGA.txt';

    case 4
        file_imu_file  = 'STDIMU_0929_1050~1138.dat';
        file_auax_file = 'AUAX_0929_1050~1138.dat';

        file_430_imu    = '0929_1050~1138_430_binary.dat';
        file_430_gpchcx = '0929_1050~1138_430_GPCHCX.txt';
        file_430_gpgga  = '0929_1050~1138_430_GPGGA.txt';

        file_830_imu    = '0929_1050~1138_830_binary.dat';
        file_830_gpchcx = '0929_1050~1138_830_GPCHCX.txt';
        file_830_gpgga  = '0929_1050~1138_830_GPGGA.txt';

    case 5
        file_imu_rs  = 'STDIMU_232_0929_1527~1658.dat';
        file_auax_rs = 'AUAX_232_0929_1526~1658.XDat';

        file_430_imu    = '0929_1504~~_430_binary.dat';
        file_430_gpchcx = '0929_1504~~_430_GPCHCX.txt';
        file_430_gpgga  = '0929_1504~~_430_GPGGA.txt';

        file_830_imu    = '0929_1504~~_830_binary.dat';
        file_830_gpchcx = '0929_1504~~_830_GPCHCX.txt';
        file_830_gpgga  = '0929_1504~~_830_GPGGA.txt';

    case 6
        file_imu_file  = 'STDIMU_232_0929_1527~1925(合).DAT';
        file_imu_file1 = 'STDIMU_0929_1749~1927.dat';
        file_auax_file = 'AUAX_232_0929_1750~1850_SEL.XDat';

        file_430_imu    = '0929_1715~~_430_binary.dat';
        file_430_gpchcx = '0929_1715~~_430_GPCHCX.txt';
        file_430_gpgga  = '0929_1715~~_430_GPGGA.txt';

        file_830_imu    = '0929_1716~~_830_binary.dat';
        file_830_gpchcx = '0929_1716~~_830_GPCHCX.txt';
        file_830_gpgga  = '0929_1716~~_830_GPGGA.txt';

    case 7
        file_imu_file  = 'STDIMU_232_0930_1539~1733.dat';
        file_imu_file1 = 'STDIMU_0930_1531~1704(缺失).dat';
        file_auax_file = 'AUAX_0930_1531~1704.dat';

        file_430_imu    = '0930_1533~1705_430_binary.dat';
        file_430_gpchcx = '0930_1533~1705_430_GPCHCX.txt';
        file_430_gpgga  = '0930_1533~1705_430_GPGGA.txt';

        file_830_imu    = '0930_1502~1804_830_binary.dat';
        file_830_gpchcx = '0930_1502~1804_830_GPCHCX.txt';
        file_830_gpgga  = '0930_1502~1804_830_GPGGA.txt';

    case 8
        file_imu_file  = 'STDIMU_232_0930_1818~1918.dat';
        file_imu_file1 = 'STDIMU_0930_1804~1918(缺失).dat';
        file_auax_rs   = 'AUAX_232_0930_1816~1918.XDat';
        file_auax_file = 'AUAX_0930_1804~1918.dat';

        file_430_imu    = '0930_1817~1920_430_binary.dat';
        file_430_gpchcx = '0930_1817~1920_430_GPCHCX.txt';
        file_430_gpgga  = '0930_1817~1920_430_GPGGA.txt';

        file_830_imu    = '0930_1817~1920_830_binary.dat';
        file_830_gpchcx = '0930_1817~1920_830_GPCHCX.txt';
        file_830_gpgga  = '0930_1817~1920_830_GPGGA.txt';

    otherwise
        warning('未定义 Round %d 的配置，跳过...', ds_idx);
        continue;
end
    
    % --- 2.3 执行数据读取操作 ---
    % 【优化】在读取时，统一使用 fullfile(cfg.inDir, subDir, filename)
    
    % 读 120 IMU
    if ~isempty(file_imu_file)
        target_file = fullfile(cfg.inDir, subDir, file_imu_file);
        tic; pos = find_header_positions(target_file, frameHeader);
        IMU_FUR_file = read_stdimu_120(target_file, pos);
        fprintf('[120] 读取 IMU_file 完成，耗时: %.2fs\n', toc);
    end
    if ~isempty(file_imu_file1)
        target_file = fullfile(cfg.inDir, subDir, file_imu_file1);
        tic; pos = find_header_positions(target_file, frameHeader);
        IMU_FUR_file1 = read_stdimu_120(target_file, pos);
        fprintf('[120] 读取 IMU_file1 完成，耗时: %.2fs\n', toc);
    end
    if ~isempty(file_imu_rs)
        target_file = fullfile(cfg.inDir, subDir, file_imu_rs);
        tic; pos = find_header_positions(target_file, frameHeader);
        IMU_FUR_RS = read_stdimu_120(target_file, pos);
        fprintf('[120] 读取 IMU_RS 完成，耗时: %.2fs\n', toc);
    end
    
    % 读 120 AUAX
    if ~isempty(file_auax_file)
        tic; AUAX_file = read_auax_120(fullfile(cfg.inDir, subDir, file_auax_file)); 
        fprintf('[120] 读取 AUAX_file 完成，耗时: %.2fs\n', toc);
    end
    if ~isempty(file_auax_rs)
        tic; AUAX_RS = read_auax_120(fullfile(cfg.inDir, subDir, file_auax_rs)); 
        fprintf('[120] 读取 AUAX_RS 完成，耗时: %.2fs\n', toc);
    end
    % 读 430 / 830
    if ~isempty(file_430_imu)
        tic; 
        fprintf('[430] 开始读取\n')
        [IMU_DATA_430, GPCHCX_430, GPGGA_430] = read_mems_ins(fullfile(cfg.inDir, subDir, file_430_imu), fullfile(cfg.inDir, subDir, file_430_gpchcx), fullfile(cfg.inDir, subDir, file_430_gpgga));
        fprintf('[430] 读取完成，耗时: %.2fs\n', toc);
    end
    if ~isempty(file_830_imu)
        tic; 
        fprintf('[830] 开始读取\n')
        [IMU_DATA_830, GPCHCX_830, GPGGA_830] = read_mems_ins(fullfile(cfg.inDir, subDir, file_830_imu), fullfile(cfg.inDir, subDir, file_830_gpchcx), fullfile(cfg.inDir, subDir, file_830_gpgga));
        fprintf('[830] 读取完成，耗时: %.2fs\n', toc);
    end
    
    % --- 2.4 缺失段拼接逻辑 ---
    if ismember(ds_idx, [6, 7]) && ~isempty(IMU_FUR_file) && ~isempty(IMU_FUR_file1)
        idstart = find(abs(IMU_FUR_file1(:,1)-IMU_FUR_file(8,1))<0.0001 & abs(IMU_FUR_file1(:,2)-IMU_FUR_file(8,2))<0.001 & abs(IMU_FUR_file1(:,3)-IMU_FUR_file(8,3))<0.001);
        if ~isempty(idstart), IMU_FUR_file_inte = [IMU_FUR_file1(1:idstart(1),:); IMU_FUR_file(9:end,:)]; end
    end
    
    if ds_idx == 8 && ~isempty(IMU_FUR_file) && ~isempty(IMU_FUR_file1)
        idstart = find(abs(IMU_FUR_file1(:,1)-IMU_FUR_file(8,1))<0.00001 & abs(IMU_FUR_file1(:,2)-IMU_FUR_file(8,2))<0.0001 & abs(IMU_FUR_file1(:,3)-IMU_FUR_file(8,3))<0.0001);
        if ~isempty(idstart), IMU_FUR_file_inte = [IMU_FUR_file1(61109:idstart(1),:); IMU_FUR_file(9:end,:)]; end
    end
    
    % --- 2.5 匹配各组原始脚本的保存规则 ---
    fprintf('正在保存 Round %d 至 %s ...\n', ds_idx, outDir);
    
    % 【修复】加入了 ds_idx == 6 的保存逻辑（和 7 组一致）
    if ds_idx >= 1 && ds_idx <= 4
        save(fullfile(outDir, 'stdimu.mat'), 'IMU_FUR_file');
        save(fullfile(outDir, 'auax.mat'), 'AUAX_file');
    elseif ds_idx == 5
        save(fullfile(outDir, 'stdimu.mat'), 'IMU_FUR_RS');
        save(fullfile(outDir, 'auax.mat'), 'AUAX_RS');
    elseif ismember(ds_idx, [6, 7])
        save(fullfile(outDir, 'stdimu.mat'), 'IMU_FUR_file', 'IMU_FUR_file_inte');
        save(fullfile(outDir, 'auax.mat'), 'AUAX_file');
    elseif ds_idx == 8
        save(fullfile(outDir, 'stdimu.mat'), 'IMU_FUR_file', 'IMU_FUR_file_inte');
        save(fullfile(outDir, 'auax.mat'), 'AUAX_file', 'AUAX_RS');
    end
    
    % 430 和 830 统一保存
    save(fullfile(outDir, '430.mat'), 'GPGGA_430', 'GPCHCX_430', 'IMU_DATA_430');
    save(fullfile(outDir, '830.mat'), 'GPGGA_830', 'GPCHCX_830', 'IMU_DATA_830');
    
    fprintf('Round %d 处理完毕！\n', ds_idx);
end
fprintf('\n[Extract Mat]所有选定的数据集已全部处理并保存完毕！\n');

%% =========================================================================
% 内部辅助函数
% =========================================================================
function headerPositions = find_header_positions(filepath, header_bytes)
    fid = fopen(filepath, 'rb');
    if fid == -1, error('无法打开文件: %s', filepath); end
    allData = fread(fid, inf, 'uint8');
    fclose(fid);
    n = length(header_bytes);
    len = length(allData);
    if len < n, headerPositions = []; return; end
    
    match = true(1, len - n + 1);
    for i = 1:n, match = match & (allData(i : len - n + i) == header_bytes(i))'; end
    headerPositions = find(match);
end