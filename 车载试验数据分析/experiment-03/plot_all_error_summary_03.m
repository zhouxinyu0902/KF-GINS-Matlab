function summary_table = plot_all_error_summary_03(dataset_ids, unit_types)
%PLOT_ALL_ERROR_SUMMARY_03 汇总三批EKF、一次RTS和二次RTS结果。

    if nargin < 1 || isempty(dataset_ids), dataset_ids = 1:3; end
    if nargin < 2 || isempty(unit_types), unit_types = ["rad", "m"]; end
    unit_types = lower(string(unit_types));
    methods = ["ekf", "rts-once", "rts-twice"];
    method_labels = ["前向EKF", "一次RTS", "二次RTS"];

    rows = cell(0, 10);
    plot_data = struct();
    for dataset_index = 1:numel(dataset_ids)
        paths = setup_all_real_data_preprocessing( ...
            dataset_ids(dataset_index), 'navigation');
        for unit_index = 1:numel(unit_types)
            unit = unit_types(unit_index);
            for method_index = 1:numel(methods)
                nav_file = fullfile(paths.output, char(unit), ...
                    char(methods(method_index) + ".nav"));
                if ~isfile(nav_file)
                    warning('%s缺少结果：%s', paths.dataset_name, nav_file);
                    continue;
                end
                result = evaluate_experiment03_nav_file( ...
                    paths.truth_file, nav_file, paths.duration_s);
                rows(end + 1, :) = {string(paths.dataset_name), unit, ...
                    method_labels(method_index), result.duration_s, ...
                    result.rmse_m, result.mean_m, result.median_m, ...
                    result.p95_m, result.maximum_m, result.final_m}; %#ok<AGROW>
                key = matlab.lang.makeValidName(sprintf('%s_%s_%s', ...
                    paths.dataset_name, unit, methods(method_index)));
                plot_data.(key) = result;
            end
        end
    end

    summary_table = cell2table(rows, 'VariableNames', ...
        {'Dataset', 'Unit', 'Method', 'Duration_s', 'RMSE_m', ...
        'Mean_m', 'Median_m', 'P95_m', 'Maximum_m', 'Final_m'});
    first_paths = experiment03_dataset_paths(dataset_ids(1));
    summary_dir = first_paths.summary;
    if ~isfolder(summary_dir), mkdir(summary_dir); end
    writetable(summary_table, fullfile(summary_dir, ...
        'navigation-error-summary.csv'));
    writetable(summary_table, fullfile(summary_dir, ...
        'navigation-error-summary.xlsx'));

    for unit_index = 1:numel(unit_types)
        unit = unit_types(unit_index);
        figure_handle = figure('Color', 'w', ...
            'Name', sprintf('experiment-03-%s-navigation-summary', unit));
        layout = tiledlayout(1, numel(dataset_ids), ...
            'TileSpacing', 'compact', 'Padding', 'compact');
        for dataset_index = 1:numel(dataset_ids)
            paths = experiment03_dataset_paths(dataset_ids(dataset_index));
            axis_handle = nexttile(layout);
            hold(axis_handle, 'on');
            for method_index = 1:numel(methods)
                key = matlab.lang.makeValidName(sprintf('%s_%s_%s', ...
                    paths.dataset_name, unit, methods(method_index)));
                if isfield(plot_data, key)
                    result = plot_data.(key);
                    plot(axis_handle, result.elapsed_time, ...
                        result.radial_error_m, 'LineWidth', 1.0, ...
                        'DisplayName', method_labels(method_index));
                end
            end
            grid(axis_handle, 'on');
            xlabel(axis_handle, '时间（s）');
            ylabel(axis_handle, '水平径向误差（m）');
            title(axis_handle, paths.dataset_name, 'Interpreter', 'none');
            if dataset_index == 1
                legend(axis_handle, 'Location', 'best');
            end
        end
        title(layout, sprintf('第三次车载试验导航结果（%s）', unit));
        exportgraphics(figure_handle, fullfile(summary_dir, ...
            sprintf('navigation-error-summary-%s.png', unit)), ...
            'Resolution', 300);
        savefig(figure_handle, fullfile(summary_dir, ...
            sprintf('navigation-error-summary-%s.fig', unit)));
    end
end
