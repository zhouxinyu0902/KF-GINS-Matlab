function process_data_1(dataset_ids, show_figures)
%PROCESS_DATA_1 检查、可视化并导出第三次车载试验导航输入。
%
%   process_data_1()          处理三个时间批次并显示图形。
%   process_data_1(2)         只处理 run-0818。
%   process_data_1(1:3,false) 批处理全部数据，图形离屏生成。
%
% 本函数只负责二级处理：读取 raw_data_read 生成的 MAT，检查时间轴和
% 观测质量，生成诊断图，并把导航需要的标准输入写入 input 目录。
% 原始文件解析只在 raw_data_read 中进行。

    if nargin < 1 || isempty(dataset_ids)
        dataset_ids = 1:3;
    end
    if nargin < 2 || isempty(show_figures)
        show_figures = true;
    end

    script_dir = fileparts(mfilename('fullpath'));
    topic_dir = fileparts(script_dir);
    addpath(topic_dir, '-begin');

    original_visibility = get(groot, 'DefaultFigureVisible');
    visibility_cleanup = onCleanup(@() set(groot, ...
        'DefaultFigureVisible', original_visibility));
    if ~show_figures
        set(groot, 'DefaultFigureVisible', 'off');
    end

    for dataset_id = dataset_ids
        process_one_dataset(dataset_id, show_figures);
    end
    clear visibility_cleanup;
end

function process_one_dataset(dataset_id, show_figures)
%PROCESS_ONE_DATASET 处理单个时间批次。

    paths = setup_all_real_data_preprocessing(dataset_id, 'processing');
    if ~isfile(paths.raw_mat_file)
        error('缺少统一原始数据，请先运行 raw_data_read：%s', ...
            paths.raw_mat_file);
    end

    raw = load(paths.raw_mat_file, 'GPCHCX_830', 'IMU_raw', ...
        'range_raw', 'depth_raw', 'auxa_raw');
    require_fields(raw, {'GPCHCX_830', 'IMU_raw', 'range_raw', ...
        'depth_raw', 'auxa_raw'}, paths.raw_mat_file);

    initial_state = yaml.ReadYaml(paths.initial_state_file);
    capture_time = double(initial_state.capture_time);
    initial_imu_time = double(initial_state.init_imu_time);

    fprintf('\n============================================================\n');
    fprintf('检查并导出导航输入：%s\n', paths.dataset_name);
    fprintf('YAML 起始时刻：%.6f s\n', capture_time);
    fprintf('============================================================\n');

    [imu_time, start_index] = build_precise_imu_time( ...
        raw.IMU_raw, capture_time, initial_imu_time);
    process_start = imu_time(start_index);
    process_end = imu_time(end);

    nav830 = prepare_830_navigation(raw.GPCHCX_830, ...
        process_start, process_end);
    figures_before = findall(groot, 'Type', 'figure');
    [~, pva830, std830] = visualize_gpchcx_navigation_results(nav830);
    figures_after = findall(groot, 'Type', 'figure');
    figures_830 = setdiff(figures_after, figures_before);
    export_830_figures(figures_830, paths.artifacts);

    [pva830_interp, std830_interp] = ...
        interpolate_830_outputs(pva830, std830);

    auxa = crop_rows_by_time(raw.auxa_raw, 1, ...
        process_start, process_end);
    if isempty(auxa)
        error('%s 的 AUXA 数据与处理时间段没有交集。', ...
            paths.dataset_name);
    end
    pva120 = auxa(:, [2, 1, 3:size(auxa, 2)]);

    glvs;
    [range830, range830_stats, range830_figure] = ...
        build_range_reference(raw.range_raw, pva830_interp, '830');
    [range120, range120_stats, range120_figure] = ...
        build_range_reference(raw.range_raw, pva120, '120');
    export_figure(range830_figure, paths.artifacts, ...
        'range-error-830');
    export_figure(range120_figure, paths.artifacts, ...
        'range-error-120');

    [depth_stats, depth_figures] = analyze_depth( ...
        raw.depth_raw, auxa, pva830_interp);
    export_figure(depth_figures(1), paths.artifacts, ...
        'depth-comparison');
    export_figure(depth_figures(2), paths.artifacts, ...
        'depth-error');

    imu120_raw = build_imu_input(raw.IMU_raw, imu_time, start_index);
    check_regular_time_axis(imu120_raw(:, 1), 0.01, ...
        sprintf('%s 原始IMU', paths.dataset_name));
    [imu120, imu_repair_table, is_interpolated_imu] = ...
        regularize_experiment03_imu(imu120_raw, 0.01, 0.5);
    check_regular_time_axis(imu120(:, 1), 0.01, ...
        sprintf('%s 规则化IMU', paths.dataset_name));

    write_matrix_safe(pva830_interp, fullfile(paths.input, ...
        'pva_830.txt'), '830 导航数据');
    write_matrix_safe(std830_interp, fullfile(paths.input, ...
        'std_830.txt'), '830 位置/速度标准差');
    write_matrix_safe(imu120_raw, paths.imu_raw_file, ...
        '120 IMU原始导航格式');
    write_matrix_safe(imu120, paths.imu_120_file, ...
        '120 IMU规则化100 Hz');
    write_matrix_safe(raw.range_raw, fullfile(paths.input, ...
        'range.txt'), '实测距离');
    write_matrix_safe(range120, fullfile(paths.input, ...
        'range_120.txt'), '120 位置构造距离');
    write_matrix_safe(range830, fullfile(paths.input, ...
        'range_830.txt'), '830 位置构造距离');
    write_matrix_safe(raw.depth_raw, fullfile(paths.input, ...
        'depth_raw.txt'), '原始深度');
    write_matrix_safe(pva120, fullfile(paths.input, ...
        'pva_120.txt'), '120 PVA');

    gnss_one_hz = select_one_hz_rows(auxa);
    write_matrix_safe(gnss_one_hz(:, [1, 3:8]), ...
        fullfile(paths.input, 'GNSS_1s.txt'), '120 一秒 GNSS');

    range_statistics = [range830_stats; range120_stats];
    writetable(range_statistics, fullfile(paths.artifacts, ...
        'range-reference-statistics.csv'));
    writetable(depth_stats, fullfile(paths.artifacts, ...
        'depth-reference-statistics.csv'));
    writetable(imu_repair_table, fullfile(paths.artifacts, ...
        'imu-time-repair.csv'));

    fprintf('%s 导出完成：\n', paths.dataset_name);
    fprintf(['  pva_830：%d 行；std_830：%d 行；imu_raw：%d 行；' ...
        'imu_120：%d 行。\n'], ...
        size(pva830_interp, 1), size(std830_interp, 1), ...
        size(imu120_raw, 1), size(imu120, 1));
    fprintf('  IMU补帧：%d处，共%d个历元。\n', ...
        height(imu_repair_table), sum(is_interpolated_imu));
    fprintf('  输入目录：%s\n', paths.input);
    fprintf('  诊断目录：%s\n', paths.artifacts);

    if ~show_figures
        close([figures_830(:); range830_figure; range120_figure; ...
            depth_figures(:)]);
    end
end

function [imu_time, start_index] = build_precise_imu_time( ...
        imu_raw, capture_time, initial_imu_time)
%BUILD_PRECISE_IMU_TIME 从上机计时与 YAML 锚点构造连续 UTC 时间。
%
% 第1列原本是高分辨率上机时间，部分文件在 YAML 起点后切换成了
% 绝对秒；个别文件在起点前还会清零。最后一列只用于识别这种切换，
% 不能直接作为 100 Hz 导航时间。真正处理起点由 YAML 中的
% init_imu_time 与 capture_time 配对确定。

    if size(imu_raw, 2) < 9
        error('IMU 原始数据至少需要 9 列。');
    end
    local_time = double(imu_raw(:, 1));
    coarse_utc = double(imu_raw(:, end));

    anchor_tolerance = 0.02;
    candidates = find(isfinite(local_time) & ...
        abs(local_time - initial_imu_time) <= anchor_tolerance);
    if isempty(candidates)
        error(['IMU 中找不到 YAML init_imu_time=%.6f s 对应的样本，' ...
            '请检查原始文件与 YAML 是否匹配。'], initial_imu_time);
    end

    finite_candidates = candidates(isfinite(coarse_utc(candidates)));
    if isempty(finite_candidates)
        start_index = candidates(end);
    else
        [~, best_candidate] = min(abs( ...
            coarse_utc(finite_candidates) - capture_time));
        start_index = finite_candidates(best_candidate);
    end

    expected_step = 0.01;
    maximum_local_step = 1.0;
    maximum_coarse_alignment = 60.0;
    imu_time = nan(size(local_time));
    imu_time(start_index) = capture_time;
    repaired_count = 0;

    for sample_index = start_index + 1:numel(local_time)
        local_step = local_time(sample_index) - ...
            local_time(sample_index - 1);
        if isfinite(local_step) && local_step > 0 && ...
                local_step <= maximum_local_step
            imu_time(sample_index) = ...
                imu_time(sample_index - 1) + local_step;
            continue;
        end

        coarse_target = coarse_utc(sample_index);
        coarse_offset = coarse_target - imu_time(sample_index - 1);
        if isfinite(coarse_offset) && coarse_offset > 0 && ...
                coarse_offset <= maximum_coarse_alignment
            imu_time(sample_index) = coarse_target;
        else
            imu_time(sample_index) = ...
                imu_time(sample_index - 1) + expected_step;
            repaired_count = repaired_count + 1;
        end
    end

    if any(diff(imu_time(start_index:end)) <= 0)
        error('构造后的 IMU UTC 时间不是严格递增序列。');
    end
    if repaired_count > 0
        warning(['IMU 起点后有 %d 处上机时间异常且无法用粗 UTC 对齐，' ...
            '已按 %.3f s 步长补齐。'], repaired_count, expected_step);
    end
    fprintf(['IMU 时间锚定：原始第 %d 行 %.6f s -> UTC %.6f s，' ...
        '共导出 %d 个历元。\n'], start_index, ...
        local_time(start_index), capture_time, ...
        numel(local_time) - start_index + 1);
end

function nav830 = prepare_830_navigation( ...
        gpchcx830, process_start, process_end)
%PREPARE_830_NAVIGATION 清理状态、转换时间并截取 830 数据。

    if size(gpchcx830, 1) < 32
        error('GPCHCX 数据字段不足，当前只有 %d 行。', ...
            size(gpchcx830, 1));
    end
    valid_status = gpchcx830(21, :) ~= 0;
    nav830 = gpchcx830(:, valid_status);
    nav830(2, :) = nav830(2, :) - 18;

    [~, order] = sort(nav830(2, :));
    nav830 = nav830(:, order);
    [~, unique_indices] = unique(nav830(2, :), 'stable');
    nav830 = nav830(:, unique_indices);

    time_mask = nav830(2, :) >= process_start & ...
        nav830(2, :) <= process_end;
    nav830 = nav830(:, time_mask);
    if size(nav830, 2) < 2
        error('830 数据与 IMU 处理时段没有足够交集。');
    end
end

function [pva_interp, std_interp] = ...
        interpolate_830_outputs(pva, position_std)
%INTERPOLATE_830_OUTPUTS 将 830 PVA 与标准差同步插值到 100 Hz。

    [pva_time, pva_unique] = unique(pva(:, 2), 'stable');
    pva = pva(pva_unique, :);
    [std_time, std_unique] = unique(position_std(:, 1), 'stable');
    position_std = position_std(std_unique, :);

    grid_start = ceil(max(pva_time(1), std_time(1)) * 100) / 100;
    grid_end = floor(min(pva_time(end), std_time(end)) * 100) / 100;
    time_grid = (grid_start:0.01:grid_end)';
    if numel(time_grid) < 2
        error('830 数据无法形成有效的 100 Hz 插值时间轴。');
    end

    pva_interp = zeros(numel(time_grid), size(pva, 2));
    pva_interp(:, 1) = interp1(pva_time, pva(:, 1), ...
        time_grid, 'nearest');
    pva_interp(:, 2) = time_grid;
    pva_interp(:, 3:10) = interp1(pva_time, pva(:, 3:10), ...
        time_grid, 'linear');
    heading_unwrapped = unwrap(deg2rad(pva(:, 11)));
    heading_interp = interp1(pva_time, heading_unwrapped, ...
        time_grid, 'linear');
    pva_interp(:, 11) = mod(rad2deg(heading_interp) + 180, 360) - 180;

    std_interp = zeros(numel(time_grid), size(position_std, 2));
    std_interp(:, 1) = time_grid;
    std_interp(:, 2:end) = interp1(std_time, ...
        position_std(:, 2:end), time_grid, 'linear');
end

function imu120 = build_imu_input(imu_raw, imu_time, start_index)
%BUILD_IMU_INPUT 转为 KF-GINS 使用的 FRD 增量格式。

    sensor_data = imu_raw(start_index:end, 2:7);
    absolute_time = imu_time(start_index:end);
    fur_input = [sensor_data, absolute_time, absolute_time];
    imu120 = imuFUR2FRD(fur_input);
end

function [range_reference, statistics, error_figure] = ...
        build_range_reference(range_data, reference_pva, source_name)
%BUILD_RANGE_REFERENCE 使用原测距记录的时刻和信标位置计算理论距离。

    [reference_time, unique_indices] = unique( ...
        reference_pva(:, 2), 'stable');
    reference_position_deg = reference_pva(unique_indices, 3:5);
    interpolated_position_deg = interp1(reference_time, ...
        reference_position_deg, range_data(:, 1), 'linear', NaN);
    valid = all(isfinite(interpolated_position_deg), 2);

    theoretical_range = nan(size(range_data, 1), 1);
    if any(valid)
        vehicle_position_rad = interpolated_position_deg(valid, :);
        vehicle_position_rad(:, 1:2) = ...
            deg2rad(vehicle_position_rad(:, 1:2));
        beacon_position_rad = range_data(valid, 4:6);
        theoretical_range(valid) = RCompu( ...
            beacon_position_rad, vehicle_position_rad);
    end

    range_reference = range_data;
    range_reference(:, 3) = theoretical_range;
    residual = range_data(valid, 3) - theoretical_range(valid);
    if isempty(residual)
        warning('%s 参考轨迹没有覆盖任何测距时刻。', source_name);
        mean_error = NaN;
        rmse_error = NaN;
        max_abs_error = NaN;
    else
        mean_error = mean(residual);
        rmse_error = sqrt(mean(residual .^ 2));
        max_abs_error = max(abs(residual));
    end

    statistics = table(string(source_name), sum(valid), mean_error, ...
        rmse_error, max_abs_error, ...
        'VariableNames', {'Reference', 'ValidCount', 'Mean_m', ...
        'RMSE_m', 'MaxAbs_m'});
    fprintf('%s 距离参考：%d/%d 点有效，Mean %.3f m，', ...
        source_name, sum(valid), size(range_data, 1), mean_error);
    fprintf('RMSE %.3f m，MaxAbs %.3f m。\n', ...
        rmse_error, max_abs_error);

    error_figure = figure('Name', sprintf('%s range error', source_name), ...
        'Color', 'w');
    plot(find(valid), residual, 'LineWidth', 1.2);
    grid on;
    xlabel('测距序号');
    ylabel('测距误差（m）');
    title(sprintf('实测距离 - %s位置构造距离', source_name));
end

function [statistics, figures] = analyze_depth(depth, auxa, pva830)
%ANALYZE_DEPTH 对比深度传感器、120 与 830 高程。

    time = depth(:, 1);
    sensor_depth = depth(:, 2);
    depth120 = interp1(auxa(:, 1), auxa(:, 5), ...
        time, 'linear', NaN);
    depth830 = interp1(pva830(:, 2), pva830(:, 5), ...
        time, 'linear', NaN);
    valid = isfinite(sensor_depth) & isfinite(depth120) & ...
        isfinite(depth830);
    if ~any(valid)
        error('深度、120 和 830 数据没有共同有效时段。');
    end

    time_valid = time(valid);
    sensor_valid = sensor_depth(valid);
    depth120_valid = depth120(valid);
    depth830_valid = depth830(valid);
    error120 = depth120_valid - sensor_valid;
    error830 = depth830_valid - sensor_valid;

    statistics = table(["120"; "830"], ...
        [sqrt(mean(error120 .^ 2)); sqrt(mean(error830 .^ 2))], ...
        [mean(error120); mean(error830)], ...
        [max(abs(error120)); max(abs(error830))], ...
        'VariableNames', {'Reference', 'RMSE_m', 'Mean_m', 'MaxAbs_m'});

    figures = gobjects(2, 1);
    figures(1) = figure('Name', 'Depth comparison', 'Color', 'w');
    plot(time_valid - time_valid(1), sensor_valid, 'LineWidth', 1.2);
    hold on;
    plot(time_valid - time_valid(1), depth120_valid, 'LineWidth', 1.2);
    plot(time_valid - time_valid(1), depth830_valid, 'LineWidth', 1.2);
    grid on;
    legend('Depth sensor', '120', '830', 'Location', 'best');
    xlabel('时间（s）');
    ylabel('深度/高程（m）');
    title('深度结果对比');

    figures(2) = figure('Name', 'Depth error', 'Color', 'w');
    plot(time_valid - time_valid(1), error120, 'LineWidth', 1.2);
    hold on;
    plot(time_valid - time_valid(1), error830, 'LineWidth', 1.2);
    grid on;
    legend('120-Depth', '830-Depth', 'Location', 'best');
    xlabel('时间（s）');
    ylabel('误差（m）');
    title('深度误差');
end

function data = crop_rows_by_time(data, time_column, start_time, end_time)
%CROP_ROWS_BY_TIME 按闭区间截取并去除重复时间。

    mask = data(:, time_column) >= start_time & ...
        data(:, time_column) <= end_time;
    data = data(mask, :);
    if isempty(data)
        return;
    end
    data = sortrows(data, time_column);
    [~, unique_indices] = unique(data(:, time_column), 'stable');
    data = data(unique_indices, :);
end

function selected = select_one_hz_rows(auxa)
%SELECT_ONE_HZ_ROWS 每个一秒区间保留第一个 AUXA 历元。

    elapsed_seconds = floor(auxa(:, 1) - auxa(1, 1));
    [~, selected_indices] = unique(elapsed_seconds, 'stable');
    selected = auxa(selected_indices, :);
end

function check_regular_time_axis(time, expected_step, data_name)
%CHECK_REGULAR_TIME_AXIS 汇报 100 Hz 时间轴中的间断。

    time_difference = diff(time);
    gap_indices = find(time_difference > expected_step * 1.5);
    if isempty(gap_indices)
        fprintf('%s 时间轴连续，步长约 %.3f s。\n', ...
            data_name, median(time_difference));
        return;
    end
    missing_count = sum(max(round( ...
        time_difference(gap_indices) / expected_step) - 1, 0));
    warning('%s 存在 %d 处时间间断，估计缺失 %d 个历元。', ...
        data_name, numel(gap_indices), missing_count);
end

function export_830_figures(figures, output_dir)
%EXPORT_830_FIGURES 导出可视化函数创建的两幅 830 诊断图。

    if isempty(figures)
        return;
    end
    [~, order] = sort([figures.Number]);
    figures = figures(order);
    names = {'pva-830-overview', 'std-830-overview'};
    for figure_index = 1:numel(figures)
        if figure_index <= numel(names)
            base_name = names{figure_index};
        else
            base_name = sprintf('pva-830-diagnostic-%02d', figure_index);
        end
        export_figure(figures(figure_index), output_dir, base_name);
    end
end

function export_figure(figure_handle, output_dir, base_name)
%EXPORT_FIGURE 同时保存 FIG 与 PNG。

    savefig(figure_handle, fullfile(output_dir, [base_name, '.fig']));
    exportgraphics(figure_handle, fullfile(output_dir, ...
        [base_name, '.png']), 'Resolution', 300);
end

function write_matrix_safe(data, output_file, data_name)
%WRITE_MATRIX_SAFE 统一写出空格分隔文本。

    if isempty(data)
        error('%s 为空，拒绝写入：%s', data_name, output_file);
    end
    writematrix(data, output_file, 'Delimiter', ' ');
    fprintf('  [OK] %s：%s\n', data_name, output_file);
end

function require_fields(data_struct, field_names, source_file)
%REQUIRE_FIELDS 检查 MAT 是否包含二级处理需要的变量。

    missing = field_names(~isfield(data_struct, field_names));
    if ~isempty(missing)
        error('MAT 文件缺少变量 %s：%s', ...
            strjoin(missing, ', '), source_file);
    end
end
