function statistics = evaluate_range_delay_context(context_path)
%EVALUATE_RANGE_DELAY_CONTEXT 统一评价仿真或实测测距延迟专题。
    statistics = evaluate_engineering_sensitivity_context(context_path);
    loaded = load(context_path, 'study_context');
    context = loaded.study_context;
    study_id = string(context.study_id);
    study_title = char(string(context.study_title));
    if ~isfield(context, 'measurement_table') || ...
            isempty(context.measurement_table)
        return;
    end
    data = context.measurement_table;
    writetable(data, fullfile(context.artifact_root, ...
        'range-input-comparison.csv'));
    figure_handle = myfigurestartup(9, 5, 'prese');
    layout = tiledlayout(figure_handle, 2, 1, ...
        'TileSpacing', 'compact', 'Padding', 'compact');
    title(layout, sprintf('%s：测距输入', study_title));
    nexttile;
    plot(data.ArrivalTime_s, data.CurrentInputRange_m, '-o', ...
        'LineWidth', 1.1, 'DisplayName', '当前时刻距离');
    hold on;
    plot(data.ArrivalTime_s, data.DelayedInputRange_m, '--s', ...
        'LineWidth', 1.1, 'DisplayName', '延迟距离');
    grid on; box on; ylabel('距离（m）'); legend('Location', 'best');
    nexttile;
    stem(data.ArrivalTime_s, data.StaleMinusCurrent_m, 'filled', ...
        'LineWidth', 1.0);
    grid on; box on; xlabel('测距到达时刻（s）');
    ylabel('陈旧值-当前值（m）');
    set(findall(figure_handle, '-property', 'FontName'), ...
        'FontName', 'TimesSimSun');
    exportgraphics(figure_handle, fullfile(context.artifact_root, ...
        study_id+"-range-input.png"), 'Resolution', 600);
    savefig(figure_handle, fullfile(context.artifact_root, ...
        study_id+"-range-input.fig"));
end
