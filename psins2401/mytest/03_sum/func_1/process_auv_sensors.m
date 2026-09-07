function [compass, octans, depther, height, vxy, VXYZ_raw] = process_auv_sensors(DistData, tt_lbl, cfg, isfig)
% PROCESS_AUV_SENSORS 处理 AUV 本体传感器数据 (姿态、深度、DVL 降噪与声速补偿)
%
% 输入:
%   DistData : 经过时间同步后的 AUV 综合数据矩阵
%   tt_lbl   : AUV 相对时间轴 (用于绘图平滑等)
%   cfg      : 配置参数结构体 (包含深度补偿 cfg.AdepC 等)
%   isfig    : 是否绘制处理前后的对比图 (1/0)
%
% 输出:
%   compass  : 罗盘姿态数据 [pitch, roll, yaw] (单位: rad, N*3 矩阵)
%   octans   : 高精度陀螺姿态 [pitch, roll, yaw] (单位: rad, N*3 矩阵)
%   depther  : 修正后的深度计数据 (单位: m, N*1 向量)
%   height   : DVL 高度计数据 (单位: m, N*1 向量)
%   vxy      : 经过声速修正与野值剔除后的最终水平速度 [vx, vy] (N*2 矩阵)
%   VXYZ_raw : DVL 原始速度 (N*2 矩阵, 供对比查看)

    if nargin < 4, isfig = 0; end
    fprintf('开始处理 AUV 本体传感器 (姿态、深度、DVL声速补偿)...\n');

    %% 1. 姿态数据提取与坐标系转换 (Pitch, Roll, Yaw)
    % 内部定义了一个匿名函数(转换逻辑)，避免重复写两次
    format_heading = @(att) wrap_heading(att); 
    
    compass_raw = DistData(1:3, :)';
    compass = format_heading(compass_raw);
    compass = d2r(compass); % 转为弧度 (调用你原有的 d2r)

    octans_raw = DistData(4:6, :)';
    octans = format_heading(octans_raw);
    octans = d2r(octans);   % 转为弧度

    %% 2. 深度与高度数据
    depther = DistData(17, :)' + cfg.AdepC; % 深度计加上安装/吃水补偿
    height  = DistData(21, :)';             % DVL 高度计数据

    %% 3. DVL 速度数据清洗 (剔除零点飞点)
    VXYZ_raw = DistData(7:8, :)';
    
    % 调用底部内嵌的清洗子函数 (整合了你原有的 findzerosfrc 逻辑)
    VXYZ_n(:, 1) = clean_dvl_axis(setvals(VXYZ_raw(:, 1)), 50);
    VXYZ_n(:, 2) = clean_dvl_axis(setvals(VXYZ_raw(:, 2)), 50);

    %% 4. 温盐深数据降噪与声速(SSP)比例修正
    % 提取并降噪 (调用你原有的 denoise 函数)
    salinity = denoise(DistData(15, :), 2, 0.2)'; 
    T_water  = denoise(DistData(16, :), 2, 0.2)'; 
    close all
    % [核心优化]: 彻底向量化声速经验公式(Mackenzie)，告别缓慢的 for 循环
    % 直接对整个向量进行加减乘除，计算速度提升百倍
    ss = 1449.2 + 4.6 .* T_water - 0.055 .* (T_water.^2) ...
         + 0.00029 .* (T_water.^3) ...
         + (1.34 - 0.01 .* T_water) .* (salinity .* 10 - 35) ...
         + 0.016 .* depther;
     
    Creal_dvl = ss ./ 1500; % DVL 默认硬件声速通常为 1500m/s
    
    % 根据实际声速比例修正 DVL 速度
    vxy = [VXYZ_n(:, 1) .* Creal_dvl, VXYZ_n(:, 2) .* Creal_dvl];

    fprintf('AUV 传感器处理与声速补偿完毕！\n\n');

    %% 5. 可视化对比 (如果 isfig == 1)
    if isfig
        % --- 姿态对比图 ---
        myfigurestartup(5,2,'paper');
        subplot(1, 2, 1);
        plot(tt_lbl, DistData(3, :));
        xygo('t/s', 'phi/deg'); title('Raw Yaw (Compass)');
        
        subplot(1, 2, 2);
        plot(tt_lbl, compass(:, 3));
        xygo('t/s', 'phi/rad'); title('Reversed & Wrapped Yaw (rad)');

        % --- DVL 清洗与补偿对比图 ---
        myfigurestartup(5,2,'zxy');
        % set(0, 'defaultLineMarkerSize', 6);
        
        subplot(1, 2, 1);
        plot(tt_lbl, VXYZ_raw(:, 1), '.', 'DisplayName', 'Raw'); hold on;
        plot(tt_lbl, vxy(:, 1), '.', 'DisplayName', 'Corrected');
        xygo('t/s', 'velocity-x (m/s)'); xlim([0 8800]); 
        legend('Location', 'best'); DeciPoin(0,2);
        
        subplot(1, 2, 2);
        plot(tt_lbl, VXYZ_raw(:, 2), '.','Color',[0.651, 0.102, 0.153], 'DisplayName', 'Raw'); hold on;
        plot(tt_lbl, vxy(:, 2), '.','Color',[0.051, 0.251, 0.502], 'DisplayName', 'Corrected');
        xygo('t/s', 'velocity-y (m/s)'); xlim([0 8800]); 
        legend('Location', 'best'); DeciPoin(0,1);

        path = 'D:\WPS云盘\469639050\WPS云盘\成果\1_DR_RANGE\fig\';
        exportpngandpdf(gca, [path,'DVL-vy'])
    end
end

%% ================== 局部辅助函数 ==================

function att_out = wrap_heading(att_in)
% 统一处理偏航角反向及跨越 180 度的折叠逻辑
    yaw = -att_in(:, 3) + 360;
    yaw(yaw > 180) = yaw(yaw > 180) - 360;
    att_out = att_in;
    att_out(:, 3) = yaw;
end

function v_axis_clean = clean_dvl_axis(v_axis, zero_threshold)
% 针对单轴速度进行连续零点异常修复 (调用外部 findzerosfrc)
    v_axis_clean = v_axis;
    [~, indice] = findzerosfrc(v_axis_clean, zero_threshold);
    if ~isempty(indice)
        for i = 1:size(indice, 2)
            idx_start = indice(1, i);
            idx_end   = indice(2, i);
            % 防止索引溢出
            calc_end = min(idx_end + 2, length(v_axis_clean));
            valid_vals = nonzeros(v_axis_clean(idx_start : calc_end));
            if ~isempty(valid_vals)
                v_axis_clean(idx_start : idx_end) = mean(valid_vals);
            else
                if idx_start > 1
                    v_axis_clean(idx_start : idx_end) = v_axis_clean(idx_start-1);
                end
            end
        end
    end
end