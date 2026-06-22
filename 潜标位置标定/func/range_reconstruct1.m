function outputFiles = range_reconstruct1(path, pathpos, outputfolder, id, S_est_xyz, filenn, exportTrue)
% -------------------------------------------------------------------------
% 功能：
%   根据原始距离文件 + 潜标参考位置，将阶段二测距文件的第4~6列
%   替换为标定后的潜标估计位置，并按指定输出路径导出。
%
% 输入：
%   path         : 原始测距文件路径，可以是 cell / char / string
%   pathpos      : beacon_pos.mat 路径，里面需要包含 pos0_geo
%   outputfolder : 输出文件夹
%   id           : 当前迭代次数
%   S_est_xyz    : 标定后的潜标估计直角坐标，N x 3
%   filenn       : 输出文件附加后缀，例如 '_stage2'
%   exportTrue   : 是否额外导出真实位置代偿结果，默认 false
%
% 输出：
%   outputFiles.calib : 标定位置代偿文件路径
%   outputFiles.true  : 真实位置代偿文件路径，可选
% -------------------------------------------------------------------------

    %% ==================== 0. 输入整理 ====================

    if nargin < 7 || isempty(exportTrue)
        exportTrue = false;
    end

    if nargin < 6 || isempty(filenn)
        filenn = '';
    end

    % path 统一整理成 cell
    if ischar(path) || isstring(path)
        path = cellstr(path);
    end

    numBeacon = numel(path);

    if size(S_est_xyz, 1) < numBeacon || size(S_est_xyz, 2) ~= 3
        error('S_est_xyz 尺寸错误：应至少为 %d x 3。', numBeacon);
    end

    % 输出目录转为 char，避免 save 报“参数必须为文本标量”
    outputfolder = local_path_to_char(outputfolder);

    if isempty(outputfolder)
        outputfolder = pwd;
    end

    if ~exist(outputfolder, 'dir')
        mkdir(outputfolder);
    end

    iterStr   = local_to_char(id);
    suffixStr = local_format_suffix(filenn);

    %% ==================== 1. 加载静态地图 ====================

    pathpos = local_path_to_char(pathpos);
    mapData = load(pathpos);

    if ~isfield(mapData, 'pos0_geo')
        error('静态地图文件中缺少 pos0_geo。');
    end

    pos0_geo = mapData.pos0_geo;

    if exportTrue
        if ~isfield(mapData, 'S_true_xyz')
            error('exportTrue=true，但静态地图文件中缺少 S_true_xyz。');
        end

        S_true_xyz = mapData.S_true_xyz;

        if size(S_true_xyz, 1) < numBeacon || size(S_true_xyz, 2) ~= 3
            error('S_true_xyz 尺寸错误：应至少为 %d x 3。', numBeacon);
        end
    end

    %% ==================== 2. 初始化输出路径 ====================

    outputFiles.calib = cell(numBeacon, 1);
    outputFiles.true  = cell(numBeacon, 1);

    fprintf('\n==================== 阶段二静态代偿重构开始 ====================\n');
    fprintf('当前迭代次数：%s\n', iterStr);
    fprintf('输出目录：%s\n', outputfolder);

    %% ==================== 3. 循环处理每个潜标文件 ====================

    for i = 1:numBeacon

        % 关键修复：无论 path{i} 是 char、string、还是 cell，都转成 char
        inputFile = local_path_to_char(path{i});

        if ~exist(inputFile, 'file')
            error('第 %d 个测距文件不存在：%s', i, inputFile);
        end

        [~, filename, fileext] = fileparts(inputFile);

        if isempty(fileext)
            fileext = '.txt';
        end

        % 读取原始测距文件
        raw_range = load(inputFile);

        if size(raw_range, 2) < 3
            error('第 %d 个测距文件列数不足，至少应包含 [时间, 斜距, 水平距离] 三列。', i);
        end

        %% -------- A. 估计位置代偿 --------

        range_calib = raw_range;

        % 将估计直角坐标转回地理坐标
        S_calib_geo = dxyz2pos(S_est_xyz(i, :), pos0_geo);
        S_calib_geo = S_calib_geo(:).';

        % 覆盖第 4~6 列：[纬度(rad), 经度(rad), 高度(m)]
        range_calib(:, 4:6) = repmat(S_calib_geo, size(range_calib, 1), 1);

        % 输出文件名：原文件名_calib_迭代次数_自定义后缀.txt
        file_calib = fullfile( ...
            outputfolder, ...
            [filename, '_calib_', iterStr, suffixStr, fileext]);

        % 关键修复：强制转成 char 文本标量
        file_calib = local_path_to_char(file_calib);

        save(file_calib, 'range_calib', '-ascii', '-double');

        outputFiles.calib{i} = file_calib;

        %% -------- B. 真实位置代偿，可选 --------

        if exportTrue

            range_true = raw_range;

            S_real_geo = dxyz2pos(S_true_xyz(i, :), pos0_geo);
            S_real_geo = S_real_geo(:).';

            range_true(:, 4:6) = repmat(S_real_geo, size(range_true, 1), 1);

            file_true = fullfile( ...
                outputfolder, ...
                [filename, '_true_', iterStr, suffixStr, fileext]);

            file_true = local_path_to_char(file_true);

            save(file_true, 'range_true', '-ascii', '-double');

            outputFiles.true{i} = file_true;
        end

        fprintf('\n【潜标 #%d 导出成功】\n', i);
        fprintf('  └─ 估计位置代偿流 -> %s\n', file_calib);

        if exportTrue
            fprintf('  └─ 真实位置代偿流 -> %s\n', file_true);
        end
    end

    fprintf('\n==================== 阶段二静态代偿重构完成 ====================\n\n');

end

%% ==================== 本地辅助函数 ====================

function str = local_to_char(x)

    if isnumeric(x)
        str = num2str(x);
    elseif isstring(x)
        str = char(x);
    elseif ischar(x)
        str = x;
    elseif iscell(x)
        str = local_to_char(x{1});
    else
        error('输入只能是 numeric、string、char 或 cell。');
    end

end

function pathStr = local_path_to_char(x)

    % 处理 cell 嵌套路径，例如 {{'D:\xxx.txt'}} 或 {'D:\xxx.txt'}
    while iscell(x)
        if isempty(x)
            error('路径 cell 为空。');
        end
        x = x{1};
    end

    if isstring(x)
        if numel(x) ~= 1
            error('路径必须是单个 string，不能是 string 数组。');
        end
        pathStr = char(x);
    elseif ischar(x)
        pathStr = x;
    else
        error('路径必须是 char、string 或 cell 包裹的路径。');
    end

end

function suffixStr = local_format_suffix(filenn)

    if isempty(filenn)
        suffixStr = '';
        return;
    end

    suffixStr = local_to_char(filenn);

    if ~isempty(suffixStr) && suffixStr(1) ~= '_' && suffixStr(1) ~= '-'
        suffixStr = ['_', suffixStr];
    end

end