function comparison = compare_experiment_state_units( ...
        meter_statistics_path, rad_statistics_path, output_dir, options)
%COMPARE_EXPERIMENT_STATE_UNITS 对比 m 链与 rad 链的四方法 RMSE。
    arguments
        meter_statistics_path (1, :) char
        rad_statistics_path (1, :) char
        output_dir (1, :) char
        options.create_figure (1, 1) logical = true
    end

    if ~isfile(meter_statistics_path) || ~isfile(rad_statistics_path)
        error('m 链或 rad 链统计文件不存在。');
    end
    meter_statistics = readtable(meter_statistics_path, 'TextType', 'string');
    rad_statistics = readtable(rad_statistics_path, 'TextType', 'string');
    if height(meter_statistics) ~= height(rad_statistics) || ...
            any(meter_statistics.Method ~= rad_statistics.Method)
        error('m 链与 rad 链的统计方法或顺序不一致。');
    end

    comparison = table(rad_statistics.Method, meter_statistics.RMSE_m, ...
        rad_statistics.RMSE_m, ...
        rad_statistics.RMSE_m - meter_statistics.RMSE_m, ...
        'VariableNames', {'Method', 'MeterState_RMSE_m', ...
        'RadState_RMSE_m', 'RadMinusMeter_m'});
    if ~isfolder(output_dir)
        mkdir(output_dir);
    end
    writetable(comparison, fullfile(output_dir, ...
        'rad-vs-meter-rmse-comparison.csv'));

    if ~options.create_figure
        return;
    end

    comparison_figure = myfigurestartup(7, 5, 'prese');
    method_axis = categorical(comparison.Method, ...
        comparison.Method, 'Ordinal', true);
    bar(method_axis, [comparison.MeterState_RMSE_m, ...
        comparison.RadState_RMSE_m], 'grouped');
    grid on; box on;
    ylabel('水平径向误差 RMSE（m）');
    title('实测数据：m 链与 rad 链四方法 RMSE 对比', ...
        'FontSize', 15);
    legend('m 链', 'rad 链', 'Location', 'northwest');
    xtickangle(15);
    set(findall(comparison_figure, '-property', 'FontName'), ...
        'FontName', 'TimesSimSun');
    exportgraphics(comparison_figure, fullfile(output_dir, ...
        'rad-vs-meter-rmse-comparison.png'), 'Resolution', 600);
    savefig(comparison_figure, fullfile(output_dir, ...
        'rad-vs-meter-rmse-comparison.fig'));
end
