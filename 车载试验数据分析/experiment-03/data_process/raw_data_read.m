function raw_data_read(dataset_ids)
%RAW_DATA_READ 读取第三次车载试验原始数据并保存统一 MAT 文件。
%
%   raw_data_read()      依次处理三个时间批次。
%   raw_data_read(2)     只处理 run-0818。
%   raw_data_read(1:3)   显式处理全部批次。
%
% 830 数据使用 read_mems_ins 解析。脚本会自动发现 raw 目录下所有
% “主文件.dat”，并匹配同名的 _GPCHCX.dat 和 _GPGGA.dat。因此普通
% 批次的一段 830 文件与 run-0818-noon 的多段文件使用同一套流程。
% 120 IMU、测距、深度和 AUXA 已是文本数据，直接读取并保存。

    if nargin < 1 || isempty(dataset_ids)
        dataset_ids = 1:3;
    end

    script_dir = fileparts(mfilename('fullpath'));
    topic_dir = fileparts(script_dir);
    addpath(topic_dir, '-begin');

    for dataset_id = dataset_ids
        paths = setup_all_real_data_preprocessing(dataset_id, 'raw');
        fprintf('\n============================================================\n');
        fprintf('读取第三次车载试验原始数据：%s\n', paths.dataset_name);
        fprintf('============================================================\n');

        [IMU_DATA_830, GPCHCX_830, GPGGA_830, source_830_files] = ...
            read_all_830_segments(paths.raw);

        IMU_raw = read_numeric_text(fullfile(paths.raw, ...
            'imu_raw_data.txt'), '120 IMU');
        range_raw = read_numeric_text(fullfile(paths.raw, ...
            'range_data.txt'), '测距');
        depth_raw = read_numeric_text(fullfile(paths.raw, ...
            'depth_data.txt'), '深度');
        [auxa_raw, auxa_state] = read_auxa_text(fullfile(paths.raw, ...
            'auxa_fields.txt'));

        validate_raw_data(IMU_raw, range_raw, depth_raw, auxa_raw);

        save(paths.raw_mat_file, ...
            'IMU_DATA_830', 'GPCHCX_830', 'GPGGA_830', ...
            'IMU_raw', 'range_raw', 'depth_raw', ...
            'auxa_raw', 'auxa_state', 'source_830_files', '-v7.3');

        fprintf('830 文件段数：%d\n', numel(source_830_files));
        fprintf('120 IMU：%d 行；测距：%d 行；深度：%d 行；AUXA：%d 行。\n', ...
            size(IMU_raw, 1), size(range_raw, 1), ...
            size(depth_raw, 1), size(auxa_raw, 1));
        fprintf('统一原始数据已保存：\n%s\n', paths.raw_mat_file);

        clear IMU_DATA_830 GPCHCX_830 GPGGA_830 ...
            IMU_raw range_raw depth_raw auxa_raw auxa_state;
    end
end

function [imu_all, gpchcx_all, gpgga_all, source_files] = ...
        read_all_830_segments(raw_dir)
%READ_ALL_830_SEGMENTS 自动发现、解析并合并一段或多段 830 数据。

    candidates = dir(fullfile(raw_dir, '*.dat'));
    is_companion = endsWith({candidates.name}, '_GPCHCX.dat', ...
        'IgnoreCase', true) | endsWith({candidates.name}, ...
        '_GPGGA.dat', 'IgnoreCase', true);
    primary_files = candidates(~is_companion);
    if isempty(primary_files)
        error('没有在 raw 目录发现 830 主文件：%s', raw_dir);
    end

    [~, order] = sort(lower(string({primary_files.name})));
    primary_files = primary_files(order);

    imu_parts = cell(numel(primary_files), 1);
    gpchcx_parts = cell(numel(primary_files), 1);
    gpgga_parts = cell(numel(primary_files), 1);
    source_files = strings(numel(primary_files), 1);

    for segment_index = 1:numel(primary_files)
        primary_name = primary_files(segment_index).name;
        [~, base_name] = fileparts(primary_name);
        imu_file = fullfile(raw_dir, primary_name);
        gpchcx_file = fullfile(raw_dir, [base_name, '_GPCHCX.dat']);
        gpgga_file = fullfile(raw_dir, [base_name, '_GPGGA.dat']);
        assert_files_exist({imu_file, gpchcx_file, gpgga_file});

        fprintf('  解析 830 第 %d/%d 段：%s\n', ...
            segment_index, numel(primary_files), base_name);
        segment_timer = tic;
        [imu_parts{segment_index}, gpchcx_parts{segment_index}, ...
            gpgga_parts{segment_index}] = read_mems_ins( ...
            imu_file, gpchcx_file, gpgga_file);
        fprintf('  完成，用时 %.2f s。\n', toc(segment_timer));
        source_files(segment_index) = string(base_name);
    end

    imu_all = vertcat(imu_parts{:});
    gpchcx_all = horzcat(gpchcx_parts{:});
    gpgga_all = horzcat(gpgga_parts{:});

    imu_all = sort_unique_rows(imu_all, size(imu_all, 2));
    gpchcx_all = sort_unique_columns(gpchcx_all, 2);
    gpgga_all = sort_gpgga_columns(gpgga_all);
end

function data = read_numeric_text(file_path, data_name)
%READ_NUMERIC_TEXT 读取可能带注释表头的数值文本文件。

    assert_files_exist({file_path});
    imported = importdata(file_path);
    if isstruct(imported)
        data = imported.data;
    else
        data = imported;
    end
    if isempty(data) || ~isnumeric(data)
        error('%s 文件没有可用数值数据：%s', data_name, file_path);
    end
end

function [auxa_raw, auxa_state] = read_auxa_text(file_path)
%READ_AUXA_TEXT 读取 AUXA 的十个数值字段和一个十六进制状态字。

    assert_files_exist({file_path});
    file_id = fopen(file_path, 'r');
    if file_id < 0
        error('无法打开 AUXA 文件：%s', file_path);
    end
    cleanup = onCleanup(@() fclose(file_id));
    fields = textscan(file_id, ...
        '%f %f %f %f %f %f %f %f %f %f %x', ...
        'CommentStyle', '#', 'CollectOutput', false);
    clear cleanup;

    auxa_raw = [fields{1:10}];
    auxa_state = fields{11};
    if isempty(auxa_raw)
        error('AUXA 文件没有可用数据：%s', file_path);
    end
end

function validate_raw_data(imu_raw, range_raw, depth_raw, auxa_raw)
%VALIDATE_RAW_DATA 检查后续导出流程依赖的最小列数。

    if size(imu_raw, 2) < 9
        error(['imu_raw_data.txt 至少需要 9 列：本地时间、六轴数据、', ...
            '温度和 UTC 时间。当前只有 %d 列。'], size(imu_raw, 2));
    end
    if size(range_raw, 2) < 6
        error('range_data.txt 至少需要 6 列，当前只有 %d 列。', ...
            size(range_raw, 2));
    end
    if size(depth_raw, 2) < 2
        error('depth_data.txt 至少需要 2 列，当前只有 %d 列。', ...
            size(depth_raw, 2));
    end
    if size(auxa_raw, 2) < 10
        error('auxa_fields.txt 至少需要 10 个数值字段。');
    end
end

function data = sort_unique_rows(data, time_column)
%SORT_UNIQUE_ROWS 按时间排序并去除分段交界处的重复历元。

    if isempty(data)
        return;
    end
    data = sortrows(data, time_column);
    [~, unique_indices] = unique(data(:, time_column), 'stable');
    data = data(unique_indices, :);
end

function data = sort_unique_columns(data, time_row)
%SORT_UNIQUE_COLUMNS 按时间排序并去除重复列。

    if isempty(data)
        return;
    end
    [~, order] = sort(data(time_row, :));
    data = data(:, order);
    [~, unique_indices] = unique(data(time_row, :), 'stable');
    data = data(:, unique_indices);
end

function data = sort_gpgga_columns(data)
%SORT_GPGGA_COLUMNS 按时分秒排序 GPGGA 多段数据。

    if isempty(data) || size(data, 1) < 3
        return;
    end
    seconds_of_day = data(1, :) * 3600 + data(2, :) * 60 + data(3, :);
    [~, order] = sort(seconds_of_day);
    data = data(:, order);
    [~, unique_indices] = unique(seconds_of_day(order), 'stable');
    data = data(:, unique_indices);
end

function assert_files_exist(file_paths)
%ASSERT_FILES_EXIST 对缺失输入给出完整路径。

    for file_index = 1:numel(file_paths)
        if ~isfile(file_paths{file_index})
            error('缺少原始输入文件：%s', file_paths{file_index});
        end
    end
end
