clear;
close all;
clc;

%% 固定测距延迟敏感性统一评价
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(fileparts(script_dir))));
common_dir = fullfile(fileparts(script_dir), 'common');
addpath(topic_dir);
addpath(common_dir);
paths = setup_inertial_experiment();

result_root = fullfile(paths.experiment_navigation, ...
    'ekf-double-rts-range-delay-rad');
context_path = fullfile(result_root, 'study-context.mat');
evaluate_engineering_sensitivity_context(context_path);

loaded = load(context_path, 'study_context');
context = loaded.study_context;
writetable(context.measurement_table, fullfile(context.artifact_root, ...
    'range-input-comparison.csv'));
plot_range_input(context.measurement_table, context.artifact_root, ...
    context.range_interval_s, context.delay_s);

function plot_range_input(data, output_dir, interval_s, delay_s)
%PLOT_RANGE_INPUT 绘制当前距离、陈旧距离及二者差值。
    figure_handle = myfigurestartup(9, 5, 'prese');
    layout = tiledlayout(figure_handle, 2, 1, ...
        'TileSpacing', 'compact', 'Padding', 'compact');
    title(layout, sprintf('测距间隔 %.0f s，固定延迟 %.0f s', ...
        interval_s, delay_s));
    nexttile;
    plot(data.UpdateTime_s, data.CurrentInputRange_m, '-o', ...
        'LineWidth', 1.1, 'DisplayName', '当前时刻距离');
    hold on;
    plot(data.UpdateTime_s, data.DelayedInputRange_m, '--s', ...
        'LineWidth', 1.1, 'DisplayName', '4 s前陈旧距离');
    grid on; box on; ylabel('距离（m）'); legend('Location', 'best');
    nexttile;
    stem(data.UpdateTime_s, data.StaleMinusCurrent_m, 'filled', ...
        'LineWidth', 1.0);
    grid on; box on; xlabel('测距更新时间（s）');
    ylabel('陈旧值-当前值（m）');
    set(findall(figure_handle, '-property', 'FontName'), ...
        'FontName', 'TimesSimSun');
    exportgraphics(figure_handle, fullfile(output_dir, ...
        'range-delay-input-comparison.png'), 'Resolution', 600);
    savefig(figure_handle, fullfile(output_dir, ...
        'range-delay-input-comparison.fig'));
end
