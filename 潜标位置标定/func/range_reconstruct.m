function range_reconstruct(path, pathpos, outputfolder, id, S_est_xyz,filenn)
% -------------------------------------------------------------------------
% 功能：利用标定成果算出来的估计位置 S_est_xyz 以及数据发生器的真实位置 S_true_xyz，
%       对阶段二（长距离直行巡航）的时序测距文件进行静态代偿重构（保持原文件名加后缀）
% 
% 输入参数：
%   path         - 阶段二原始测距文件的路径元胞数组 {path1, path2, path3}
%   pathpos      - 数据发生器生成的 beacon_pos.mat 静态真值地图路径
%   outputfolder - 标定成果与重构数据的落地目标文件夹路径
%   S_est_xyz    - 阶段一滤波器最终输出的潜标水下发声点估计直角坐标 (3x3)
% -------------------------------------------------------------------------
    % A. 动态加载静态地图文件，提取 pos0_geo(青岛原点) 与 S_true_xyz(潜标实际发声点直角坐标真值)
    load(pathpos);
    
    if ~exist(outputfolder, 'dir'), mkdir(outputfolder); end
    fprintf('\n==================== 正在执行阶段二数据流双重静态代偿重构 ====================\n');
    
    % B. 循环处理 3 个潜标的时序测距文本
    for i = 1:3
        % 1. 动态解析原文件路径，获取文件名(name)和扩展名(ext)
        [~, filename, fileext] = fileparts(path{i});
        if isempty(fileext)
            fileext = '.txt'; % 如果原文件没写后缀，默认设为 .txt
        end
        
        % 2. 重新加载最原始的阶段二测距文本，确保前3列 [时间, 带噪斜距, 带噪水平距离] 完好不丢失
        raw_range = load(path{i});
        
        % 3. 预分配两套独立的目标代偿时序矩阵
        range_calib = raw_range; % 方案 A：灌入估计坐标（标定算法反演成果）
        range_true  = raw_range; % 方案 B：灌入真实坐标（绝对零误差理想基准）
        
        %% 【数据流一：重构标定估计位置地理经纬度】
        % 调用 PSINS 工具箱将估计直角坐标转回大圆弧地理坐标系 [纬度(rad), 经度(rad), 高度(m)]
        S_calib_geo = dxyz2pos(S_est_xyz(i, :), pos0_geo); 
        % 核心修正：加单引号转置为行向量，利用 repmat 完美平铺复制到时序矩阵的第 4、5、6 列
        range_calib(:, 4:6) = repmat(S_calib_geo, size(range_calib, 1), 1);
        
        %% 【数据流二：重构真实物理位置地理经纬度（新增功能）】
        % 将发生器自带的绝对真实直角坐标 S_true_xyz 转回大圆弧地理坐标系
        S_real_geo = dxyz2pos(S_true_xyz(i, :), pos0_geo); 
        % 同样转置为行向量，平铺覆盖第 4、5、6 列
        range_true(:, 4:6) = repmat(S_real_geo, size(range_true, 1), 1);
        
        %% C. 分层双精密落地保存文本（直接在原名后追加后缀）
        % 落地一：原文件名 + _calib
        % file_c = [filenn, fileext];
        % save(file_c, 'range_calib', '-ascii', '-double'); 
        file_c = fullfile('D:\Github\KF-GINS-Matlab\潜标位置标定\Non-idealization\rangeconstruct\', [filename, '_calib_', num2str(id), filenn, fileext]);
        save(file_c, 'range_calib', '-ascii', '-double');
        
        % % 落地二：原文件名 + _true
        % file_t = fullfile(outputfolder, [filename, '_true',fileext]);
        % save(file_t, 'range_true', '-ascii', '-double');
        
        fprintf('【潜标 #%d 导出成功】\n', i);
        fprintf('  └─ 估计位置代偿流 -> %s\n', file_c);
        % fprintf('  └─ 真实点位代偿流 -> %s\n', file_t);
    end
    fprintf('================================================================================\n\n');
end