clear;
close all;
clc;
%% 汇总56 min窗口、42 min触发与2RTS在case-06～09上的结果

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
paths = setup_inertial_experiment();
case_names = ["case-06"; "case-07"; "case-08"; "case-09"];
result_folder_name = 'windowed-periodic-global-rts-56min-every-42min-m';
summary_dir = fullfile(paths.experiment, ...
    'summary', 'windowed-periodic-global-rts-56min-every42min');
if ~exist(summary_dir, 'dir')
    mkdir(summary_dir);
end

all_rows = table();
summary_rows = repmat(empty_summary_row(), numel(case_names), 1);
for case_index = 1:numel(case_names)
    case_id = str2double(extractAfter(case_names(case_index), 'case-'));
    cfg = load_algorithm_exploration_config( ...
        "experiment", "m", paths.experiment_input(case_id));
    result_dir = fullfile(cfg.outputfolder, result_folder_name);
    comparison_path = fullfile(result_dir, ...
        'windowed-periodic-global-rts-comparison.csv');
    diagnostic_path = fullfile(cfg.outputfolder, '..', 'figures-tables', ...
        result_folder_name, 'range-rts-standard-global-events.csv');
    if ~isfile(comparison_path) || ~isfile(diagnostic_path)
        error('缺少%s的窗口RTS结果，请先运行窗口RTS脚本。', ...
            case_names(case_index));
    end
    comparison = readtable(comparison_path);
    comparison.case_name = repmat(case_names(case_index), height(comparison), 1);
    all_rows = [all_rows; comparison]; %#ok<AGROW>

    strict = comparison(strcmp(comparison.scope, '严格已覆盖区间'), :);
    full = comparison(strcmp(comparison.scope, '完整运行区间'), :);
    window_row = strict(strcmp(strict.method, '周期窗口RTS'), :);
    double_row = strict(strcmp(strict.method, '二次RTS'), :);
    hybrid_row = full(strcmp(full.method, '2RTS+周期窗口RTS'), :);
    full_double_row = full(strcmp(full.method, '二次RTS'), :);
    diagnostics = readtable(diagnostic_path);
    boundary_mask = isfinite(diagnostics.boundary_position_step_m);
    boundary_position_max = nan;
    boundary_velocity_max = nan;
    if any(boundary_mask)
        boundary_position_max = max( ...
            diagnostics.boundary_position_step_m(boundary_mask));
        boundary_velocity_max = max( ...
            diagnostics.boundary_velocity_step_mps(boundary_mask));
    end
    row = empty_summary_row();
    row.case_name = case_names(case_index);
    row.trigger_count = height(diagnostics);
    row.strict_window_rmse_m = window_row.horizontal_rmse_m;
    row.strict_double_rmse_m = double_row.horizontal_rmse_m;
    row.strict_forward_rmse_m = strict(strcmp(strict.method, '前向EKF'), :).horizontal_rmse_m;
    row.strict_window_improvement_pct = 100 * ...
        (row.strict_double_rmse_m - row.strict_window_rmse_m) / ...
        row.strict_double_rmse_m;
    row.full_hybrid_rmse_m = hybrid_row.horizontal_rmse_m;
    row.full_double_rmse_m = full_double_row.horizontal_rmse_m;
    row.full_forward_rmse_m = full(strcmp(full.method, '前向EKF'), :).horizontal_rmse_m;
    row.full_hybrid_improvement_pct = 100 * ...
        (row.full_double_rmse_m - row.full_hybrid_rmse_m) / ...
        row.full_double_rmse_m;
    row.boundary_position_max_m = boundary_position_max;
    row.boundary_velocity_max_mps = boundary_velocity_max;
    summary_rows(case_index) = row;
end

summary = struct2table(summary_rows);
summary_path = fullfile(summary_dir, ...
    'windowed-periodic-global-rts-case-summary.csv');
all_rows_path = fullfile(summary_dir, ...
    'windowed-periodic-global-rts-all-methods.csv');
writetable(summary, summary_path);
writetable(all_rows, all_rows_path);
disp(summary);

figure_path = fullfile(summary_dir, ...
    'windowed-periodic-global-rts-case-comparison.png');
fig = figure('Color', 'w', 'Visible', 'off', ...
    'Position', [80, 80, 1400, 820]);
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', ...
    'Padding', 'compact');
title(layout, '56 min窗口RTS、2RTS与前向EKF：case-06～09');

nexttile;
bar(categorical(summary.case_name), [summary.strict_window_rmse_m, ...
    summary.strict_double_rmse_m, summary.strict_forward_rmse_m]);
grid on;
ylabel('RMSE / m');
title('严格已覆盖区间');
legend({'窗口RTS', '2RTS', '前向EKF'}, 'Location', 'northwest');

nexttile;
bar(categorical(summary.case_name), [summary.full_hybrid_rmse_m, ...
    summary.full_double_rmse_m, summary.full_forward_rmse_m]);
grid on;
ylabel('RMSE / m');
title('完整运行区间（尾段暂用2RTS）');
legend({'窗口RTS+2RTS', '2RTS', '前向EKF'}, 'Location', 'northwest');

nexttile;
bar(categorical(summary.case_name), [summary.strict_window_improvement_pct, ...
    summary.full_hybrid_improvement_pct]);
yline(0, 'k:');
grid on;
ylabel('相对2RTS改善 / %');
title('窗口方法相对2RTS的RMSE改善');
legend({'已覆盖区间', '完整运行区间'}, 'Location', 'best');

nexttile;
bar(categorical(summary.case_name), [summary.boundary_position_max_m, ...
    summary.boundary_velocity_max_mps]);
grid on;
ylabel('数值大小');
title('最大窗口接缝：位置(m) / 速度(m/s)');
legend({'位置跳变 / m', '速度跳变 / m/s'}, 'Location', 'best');

exportgraphics(fig, figure_path, 'Resolution', 220);
close(fig);
fprintf('汇总表：%s\n', summary_path);
fprintf('全方法明细：%s\n', all_rows_path);
fprintf('对比图：%s\n', figure_path);

function row = empty_summary_row()
    row = struct('case_name', "", 'trigger_count', nan, ...
        'strict_window_rmse_m', nan, 'strict_double_rmse_m', nan, ...
        'strict_forward_rmse_m', nan, ...
        'strict_window_improvement_pct', nan, ...
        'full_hybrid_rmse_m', nan, 'full_double_rmse_m', nan, ...
        'full_forward_rmse_m', nan, ...
        'full_hybrid_improvement_pct', nan, ...
        'boundary_position_max_m', nan, ...
        'boundary_velocity_max_mps', nan);
end
