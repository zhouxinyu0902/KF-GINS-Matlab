function [first_hour_table, full_duration_table] = ...
        plot_all_pureins_03(dataset_ids, unit_types)
%PLOT_ALL_PUREINS_03 汇总三批纯惯导前3600秒和完整时段误差。

    if nargin < 1 || isempty(dataset_ids), dataset_ids = 1:3; end
    if nargin < 2 || isempty(unit_types), unit_types = ["rad", "m"]; end
    unit_types = lower(string(unit_types));
    evaluation_duration_s = 3600;

    first_rows = cell(0, 10);
    full_rows = cell(0, 10);
    plot_data = struct();
    for dataset_index = 1:numel(dataset_ids)
        paths = setup_all_real_data_preprocessing( ...
            dataset_ids(dataset_index), 'navigation');
        for unit_index = 1:numel(unit_types)
            unit = unit_types(unit_index);
            nav_file = fullfile(paths.output, char(unit), ...
                sprintf('PureIns-%s.nav', unit));
            if ~isfile(nav_file)
                warning('%s缺少PureIns结果：%s', ...
                    paths.dataset_name, nav_file);
                continue;
            end
            first_result = evaluate_experiment03_nav_file( ...
                paths.truth_file, nav_file, evaluation_duration_s);
            full_result = evaluate_experiment03_nav_file( ...
                paths.truth_file, nav_file, paths.duration_s);
            first_rows(end + 1, :) = result_row( ...
                paths.dataset_name, unit, first_result); %#ok<AGROW>
            full_rows(end + 1, :) = result_row( ...
                paths.dataset_name, unit, full_result); %#ok<AGROW>
            key = matlab.lang.makeValidName(sprintf('%s_%s', ...
                paths.dataset_name, unit));
            plot_data.(key) = first_result;
        end
    end

    variable_names = {'Dataset', 'Unit', 'Duration_s', 'Complete3600s', ...
        'RMSE_m', 'Mean_m', 'Median_m', 'P95_m', 'Maximum_m', 'Final_m'};
    first_hour_table = cell2table(first_rows, ...
        'VariableNames', variable_names);
    if ~isempty(first_hour_table)
        first_hour_table.Complete3600s = ...
            first_hour_table.Duration_s >= evaluation_duration_s - 1;
    end
    full_duration_table = cell2table(full_rows, ...
        'VariableNames', variable_names);
    if ~isempty(full_duration_table)
        full_duration_table.Complete3600s = ...
            full_duration_table.Duration_s >= evaluation_duration_s - 1;
    end

    first_paths = experiment03_dataset_paths(dataset_ids(1));
    summary_dir = first_paths.summary;
    if ~isfolder(summary_dir), mkdir(summary_dir); end
    writetable(first_hour_table, fullfile(summary_dir, ...
        'pureins-first3600s-statistics.csv'));
    writetable(first_hour_table, fullfile(summary_dir, ...
        'pureins-first3600s-statistics.xlsx'));
    writetable(full_duration_table, fullfile(summary_dir, ...
        'pureins-full-duration-statistics.csv'));

    for unit_index = 1:numel(unit_types)
        unit = unit_types(unit_index);
        figure_handle = figure('Color', 'w', ...
            'Name', sprintf('experiment-03-%s-pureins', unit));
        layout = tiledlayout(1, numel(dataset_ids), ...
            'TileSpacing', 'compact', 'Padding', 'compact');
        for dataset_index = 1:numel(dataset_ids)
            paths = experiment03_dataset_paths(dataset_ids(dataset_index));
            key = matlab.lang.makeValidName(sprintf('%s_%s', ...
                paths.dataset_name, unit));
            axis_handle = nexttile(layout);
            if isfield(plot_data, key)
                result = plot_data.(key);
                plot(axis_handle, result.elapsed_time, ...
                    result.radial_error_m, 'LineWidth', 1.1);
            end
            grid(axis_handle, 'on');
            xlabel(axis_handle, '时间（s）');
            ylabel(axis_handle, '水平径向误差（m）');
            title(axis_handle, paths.dataset_name, 'Interpreter', 'none');
        end
        title(layout, sprintf('Pure INS前3600秒（%s）', unit));
        exportgraphics(figure_handle, fullfile(summary_dir, ...
            sprintf('pureins-first3600s-%s.png', unit)), ...
            'Resolution', 300);
        savefig(figure_handle, fullfile(summary_dir, ...
            sprintf('pureins-first3600s-%s.fig', unit)));
    end
end

function row = result_row(dataset_name, unit, result)
    row = {string(dataset_name), string(unit), result.duration_s, false, ...
        result.rmse_m, result.mean_m, result.median_m, result.p95_m, ...
        result.maximum_m, result.final_m};
end
