%% =========================================================================
% 8组数据轨迹多子图对比 & 数据时长完整统计表 (含 IMU)
% 核心逻辑：强制以每组的 gps_830 第一个有效点作为本地坐标原点，并统计所有文件时长
% =========================================================================
clear; clc; close all;

% ==================== 1. 全局配置 ====================
base_dir = 'D:\\Github\\KF-GINS-Matlab\\data\\experiment-data\\experiment-01\\case-0%d\\input';
total_rounds = 1:8;

% 创建图窗
% figure('Name', '8组实验数据轨迹详细对比', 'Position', [50, 50, 1600, 800], 'Color', 'w');
fig=myfigurestartup(12,7,'paper');
color_gps830 = [0.6 0.6 0.6]; 
color_120    = 'k';           
color_430    = '#D95319';     
color_830    = '#0072BD';     

% 用于存放时长的矩阵：8行 x 8列
% 列定义：[120_PVA, 430_PVA, 830_PVA, 430_GPS, 830_GPS, 120_IMU, 430_IMU, 830_IMU] （分钟）
durations = zeros(length(total_rounds), 8); 

fprintf('正在加载 %d 组数据，绘制轨迹并统计时长(含高频IMU)，请稍候...\n', length(total_rounds));

% ==================== 2. 循环加载、画图与统计 ====================
for r = total_rounds
    current_dir = sprintf(base_dir, r);
    
    % 初始化空矩阵
    pva_120 = []; pva_430 = []; pva_830 = [];
    gps_430 = []; gps_830 = [];
    imu_120 = []; imu_430 = []; imu_830 = [];
    
    % --- 2.1 安全加载数据 ---
    if exist(fullfile(current_dir, 'pva_file.txt'), 'file'), pva_120 = readmatrix(fullfile(current_dir, 'pva_file.txt')); end
    if exist(fullfile(current_dir, 'pva_430.txt'), 'file'),  pva_430 = readmatrix(fullfile(current_dir, 'pva_430.txt'));  end
    if exist(fullfile(current_dir, 'pva_830.txt'), 'file'),  pva_830 = readmatrix(fullfile(current_dir, 'pva_830.txt'));  end
    
    if exist(fullfile(current_dir, 'gps_430.txt'), 'file'),  gps_430 = readmatrix(fullfile(current_dir, 'gps_430.txt'));  end
    if exist(fullfile(current_dir, 'gps_830.txt'), 'file'),  gps_830 = readmatrix(fullfile(current_dir, 'gps_830.txt'));  end
    
    if exist(fullfile(current_dir, 'IMU_120.txt'), 'file'),  imu_120 = readmatrix(fullfile(current_dir, 'IMU_120.txt'));  end
    if exist(fullfile(current_dir, 'imu_430.txt'), 'file'),  imu_430 = readmatrix(fullfile(current_dir, 'imu_430.txt'));  end
    if exist(fullfile(current_dir, 'imu_830.txt'), 'file'),  imu_830 = readmatrix(fullfile(current_dir, 'imu_830.txt'));  end

    % --- 2.2 统计时长 (换算为分钟) ---
    % PVA 时间在第 2 列
    if ~isempty(pva_120), durations(r, 1) = (pva_120(end,2) - pva_120(1,2)) / 60; end
    if ~isempty(pva_430), durations(r, 2) = (pva_430(end,2) - pva_430(1,2)) / 60; end
    if ~isempty(pva_830), durations(r, 3) = (pva_830(end,2) - pva_830(1,2)) / 60; end
    % GPS 时间在第 1 列
    if ~isempty(gps_430), durations(r, 4) = (gps_430(end,1) - gps_430(1,1)) / 60; end
    if ~isempty(gps_830), durations(r, 5) = (gps_830(end,1) - gps_830(1,1)) / 60; end
    % IMU 时间在第 1 列 (根据您之前的 FRD 导出格式)
    if ~isempty(imu_120), durations(r, 6) = (imu_120(end,1) - imu_120(1,1)) / 60; end
    if ~isempty(imu_430), durations(r, 7) = (imu_430(end,1) - imu_430(1,1)) / 60; end
    if ~isempty(imu_830), durations(r, 8) = (imu_830(end,1) - imu_830(1,1)) / 60; end

    % --- 2.3 异常点剔除 (针对 830 飞点) ---
    if ~isempty(pva_830)
        bad_idx = find(pva_830(:,3) > 37 | pva_830(:,3) < 36);
        pva_830(bad_idx, :) = [];
    end

    % --- 2.4 子图绘制 ---
    subplot(2, 4, r);
    hold on; grid on; 
    axis equal;
    
    % 强制以 gps_830 为参考原点
    ref_lat = NaN; ref_lon = NaN;
    if ~isempty(gps_830)
        ref_lat = gps_830(1,3); ref_lon = gps_830(1,4);
    else
        if ~isempty(pva_830), ref_lat = pva_830(1,3); ref_lon = pva_830(1,4);
        elseif ~isempty(pva_120), ref_lat = pva_120(1,3); ref_lon = pva_120(1,4);
        end
    end
    
    if isnan(ref_lat)
        title(sprintf('Round %d (无数据)', r));
        continue;
    end

    % % 画底层 GPS
    % if ~isempty(gps_830)
    %     plot((gps_830(:,4)-ref_lon)*111319.9.*cosd(ref_lat), (gps_830(:,3)-ref_lat)*111319.9, ...
    %         '.', 'Color', color_gps830, 'MarkerSize', 5, 'DisplayName', '830 GPS');
    % end
    % % 画 430
    % if ~isempty(pva_430)
    %     plot((pva_430(:,4)-ref_lon)*111319.9.*cosd(ref_lat), (pva_430(:,3)-ref_lat)*111319.9, ...
    %         '-', 'Color', color_430, 'LineWidth', 1.2, 'DisplayName', '430 PVA');
    % end
    % 画 830
    if ~isempty(pva_830)
        plot((pva_830(:,4)-ref_lon)*111319.9.*cosd(ref_lat), (pva_830(:,3)-ref_lat)*111319.9, ...
            '-', 'Color', color_830, 'LineWidth', 1.2, 'DisplayName', '830 PVA');
    end
    % % 画 120 (压轴)
    % if ~isempty(pva_120)
    %     plot((pva_120(:,4)-ref_lon)*111319.9.*cosd(ref_lat), (pva_120(:,3)-ref_lat)*111319.9, ...
    %         '-', 'Color', color_120, 'LineWidth', 1.5, 'DisplayName', '120 PVA (Ref)');
    % end

    title(sprintf('Round %d', r), 'FontSize', 13, 'FontWeight', 'bold');
    xlabel('East (m)'); ylabel('North (m)');
    if r == 1, legend('Location', 'best', 'FontSize', 9); end
end

sgtitle('实验数据 1~8 组 2D 轨迹详细对比 (坐标原点: 各组 GPS 830 起点)', 'FontSize', 16, 'FontWeight', 'bold');

% ==================== 3. 打印统计表格 ====================
fprintf('\n');
fprintf('=========================================================================================================\n');
fprintf('                               🚀 实验数据各设备有效时长统计表 (单位：分钟)                               \n');
fprintf('=========================================================================================================\n');
fprintf(' 组别(Round) | 120_PVA | 430_PVA | 830_PVA | 430_GPS | 830_GPS | 120_IMU | 430_IMU | 830_IMU \n');
fprintf('---------------------------------------------------------------------------------------------------------\n');
for r = total_rounds
    fprintf('   Round %d   | %7.2f | %7.2f | %7.2f | %7.2f | %7.2f | %7.2f | %7.2f | %7.2f \n', ...
        r, durations(r,1), durations(r,2), durations(r,3), durations(r,4), durations(r,5), durations(r,6), durations(r,7), durations(r,8));
end
fprintf('=========================================================================================================\n');

% ==================== 4. 生成 MATLAB Table (方便复制到 Excel) ====================
Round_Number = total_rounds';
PVA_120_min  = round(durations(:,1), 2);
PVA_430_min  = round(durations(:,2), 2);
PVA_830_min  = round(durations(:,3), 2);
GPS_430_min  = round(durations(:,4), 2);
GPS_830_min  = round(durations(:,5), 2);
IMU_120_min  = round(durations(:,6), 2);
IMU_430_min  = round(durations(:,7), 2);
IMU_830_min  = round(durations(:,8), 2);

Duration_Summary_Table = table(Round_Number, PVA_120_min, PVA_430_min, PVA_830_min, ...
    GPS_430_min, GPS_830_min, IMU_120_min, IMU_430_min, IMU_830_min);
outdir = 'D:\\Github\\KF-GINS-Matlab\\data\\experiment-data\\experiment-01\\summary';
exportgraphics(fig,fullfile(outdir,'eight-case-trajectory-compare.png'),'Resolution', 600)
fprintf('\n✅ 提示：已在工作区生成变量 Duration_Summary_Table，双击打开可直接复制到 Excel。\n');