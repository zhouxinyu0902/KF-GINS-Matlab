function results = step3_build_dataset_range_03(dataset_ids, show_figures)
%STEP3_BUILD_DATASET_RANGE_03 构造120/830位置参考距离。
%
% 保留input/range.txt中的原始测距、时刻和信标位置，仅用120或830
% 位置重新计算第3列距离，分别输出range_120.txt和range_830.txt。

    if nargin < 1 || isempty(dataset_ids)
        dataset_ids = 1:3;
    end
    if nargin < 2 || isempty(show_figures)
        show_figures = false;
    end

    original_visibility = get(groot, 'DefaultFigureVisible');
    visibility_cleanup = onCleanup(@() set(groot, ...
        'DefaultFigureVisible', original_visibility));
    if ~show_figures
        set(groot, 'DefaultFigureVisible', 'off');
    end

    dataset_count = count_datasets(dataset_ids);
    result_template = struct('dataset_name', '', ...
        'range_120_file', '', 'range_830_file', '', ...
        'statistics_file', '', 'statistics', table());
    results = repmat(result_template, dataset_count, 1);

    for dataset_index = 1:dataset_count
        dataset_id = select_dataset(dataset_ids, dataset_index);
        paths = setup_experiment03(dataset_id, 'range');
        fprintf('\n============================================================\n');
        fprintf('Step 3 构造距离：%s\n', paths.dataset_name);
        fprintf('============================================================\n');

        pva120_file = fullfile(paths.input, 'pva_120.txt');
        required_files = {paths.range_file, pva120_file, ...
            paths.pva_830_file};
        missing = required_files(~cellfun(@isfile, required_files));
        if ~isempty(missing)
            error('%s缺少距离构造输入：%s', paths.dataset_name, ...
                strjoin(missing, ', '));
        end

        range_data = readmatrix(paths.range_file, 'FileType', 'text');
        pva120 = readmatrix(pva120_file, 'FileType', 'text');
        pva830 = readmatrix(paths.pva_830_file, 'FileType', 'text');
        validate_inputs(range_data, pva120, pva830, paths.dataset_name);

        glvs;
        [range120, statistics120, figure120] = ...
            build_range_reference(range_data, pva120, '120');
        [range830, statistics830, figure830] = ...
            build_range_reference(range_data, pva830, '830');

        range120_file = fullfile(paths.input, 'range_120.txt');
        range830_file = fullfile(paths.input, 'range_830.txt');
        writematrix(range120, range120_file, 'FileType', 'text', ...
            'Delimiter', ' ');
        writematrix(range830, range830_file, 'FileType', 'text', ...
            'Delimiter', ' ');

        export_figure(figure120, paths.artifacts, 'range-error-120');
        export_figure(figure830, paths.artifacts, 'range-error-830');
        statistics = [statistics830; statistics120];
        statistics_file = fullfile(paths.artifacts, ...
            'range-reference-statistics.csv');
        writetable(statistics, statistics_file);

        results(dataset_index) = struct( ...
            'dataset_name', paths.dataset_name, ...
            'range_120_file', range120_file, ...
            'range_830_file', range830_file, ...
            'statistics_file', statistics_file, ...
            'statistics', statistics);
        fprintf('%s距离构造完成：range_120.txt、range_830.txt。\n', ...
            paths.dataset_name);

        if ~show_figures
            close([figure120, figure830]);
        end
    end
    clear visibility_cleanup;
end

function [range_reference, statistics, error_figure] = ...
        build_range_reference(range_data, reference_pva, source_name)
%BUILD_RANGE_REFERENCE 在原始测距时刻计算参考轨迹至信标的距离。

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
        warning('%s参考轨迹没有覆盖任何测距时刻。', source_name);
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
    fprintf('%s距离参考：%d/%d点有效，Mean %.3f m，', ...
        source_name, sum(valid), size(range_data, 1), mean_error);
    fprintf('RMSE %.3f m，MaxAbs %.3f m。\n', ...
        rmse_error, max_abs_error);

    error_figure = figure('Name', ...
        sprintf('%s range error', source_name), 'Color', 'w');
    plot(find(valid), residual, 'LineWidth', 1.2);
    grid on;
    xlabel('测距序号');
    ylabel('测距误差（m）');
    title(sprintf('实测距离 - %s位置构造距离', source_name));
end

function validate_inputs(range_data, pva120, pva830, dataset_name)
    if isempty(range_data) || size(range_data, 2) < 6
        error('%s range.txt必须至少包含6列。', dataset_name);
    end
    if isempty(pva120) || size(pva120, 2) < 5
        error('%s pva_120.txt必须至少包含5列。', dataset_name);
    end
    if isempty(pva830) || size(pva830, 2) < 5
        error('%s pva_830.txt必须至少包含5列。', dataset_name);
    end
    if any(diff(range_data(:, 1)) <= 0)
        error('%s range.txt时间必须严格递增。', dataset_name);
    end
end

function export_figure(figure_handle, output_dir, base_name)
    if ~isfolder(output_dir), mkdir(output_dir); end
    exportgraphics(figure_handle, fullfile(output_dir, ...
        [base_name, '.png']), 'Resolution', 300);
    savefig(figure_handle, fullfile(output_dir, [base_name, '.fig']));
end

function count = count_datasets(dataset_ids)
    if ischar(dataset_ids), count = 1; else, count = numel(dataset_ids); end
end

function dataset_id = select_dataset(dataset_ids, index)
    if ischar(dataset_ids)
        dataset_id = dataset_ids;
    elseif iscell(dataset_ids)
        dataset_id = dataset_ids{index};
    else
        dataset_id = dataset_ids(index);
    end
end
