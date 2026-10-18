clear;
close all;
clc;
%% 掉帧影响与测向改善的统一分析
% 对比对象：
%   1) 无掉帧仅测距；
%   2) 掉帧仅测距；
%   3) 无掉帧测距+测向（相位差噪声 1/3/5/10 deg）；
%   4) 同一掉帧编号下的测距+测向。
% 输出两类相对指标：
%   - 测向改善率：掉帧测距+测向相对掉帧仅测距的误差降低百分比；
%   - 掉帧影响率：掉帧结果相对同方法无掉帧结果的误差变化百分比。

data_source = "experiment"; 
case_name = 'case-06';
% data_source = "simulation"; 
% case_name = 'case-00';
phase_std_degs = [1, 3, 5, 10];
stage_ids = {'forward', 'rts-single', 'rts-double'};
stage_labels = {'前向 EKF', '一次 RTS', '二次 RTS'};
range_only_baseline_names = {
    'simple-forward-ekf-rad.nav'
    'simple-rts-single-rad.nav'
    'simple-rts-double-rad.nav'};
range_azimuth_baseline_names = {
    'range-phase-azimuth-forward.nav'
    'range-phase-azimuth-rts-single.nav'
    'range-phase-azimuth-rts-double.nav'};
drop_indices_to_analyze = [];  % 空数组表示自动分析全部已有 dropN。
apply_grubbs = false;
show_figures = false;
generate_radial_error_figures = false;
generate_summary_figures = true;
export_resolution_dpi = 600;

%% 路径
script_dir = fileparts(mfilename('fullpath'));
algorithm_root = fileparts(fileparts(fileparts(fileparts( ...
    fileparts(script_dir)))));
addpath(algorithm_root);
paths = setup_inertial_experiment();

case_id = sscanf(case_name, 'case-%d', 1);
if isempty(case_id) || ~isscalar(case_id) || ...
        case_id < 0 || case_id ~= fix(case_id)
    error('case_name 必须采用 case-XX 格式，例如 case-00。');
end
if data_source == "experiment"
    input_dir = paths.experiment_input(case_id);
    navigation_dir = paths.experiment_navigation(case_id);
    artifact_root = paths.experiment_artifacts(case_id);
    cfg = load_algorithm_exploration_config('experiment','rad',input_dir);
elseif data_source == "simulation"
    input_dir = paths.simulation_input(case_id);
    navigation_dir = paths.simulation_navigation(case_id);
    artifact_root = paths.simulation_artifacts(case_id);
    cfg = load_algorithm_exploration_config('simulation','rad',input_dir);
end
truth_path = cfg.truthpath;
navigation_root = fullfile(navigation_dir, 'navigation-results');
range_baseline_dir = fullfile(navigation_root, 'simple-ekf-rts-rad');
range_dropout_dir = fullfile(navigation_root, 'drop-noazi-2RTS');
analysis_dir = fullfile(artifact_root, ...
    'dropout-range-azimuth-analysis');
radial_figure_dir = fullfile(analysis_dir, 'figures', 'radial-error');
summary_figure_dir = fullfile(analysis_dir, 'figures', 'summary');
ensure_directory(analysis_dir);
if generate_radial_error_figures
    ensure_directory(radial_figure_dir);
end
if generate_summary_figures
    ensure_directory(summary_figure_dir);
end

gjb_function_dir = fullfile(paths.project, ...
    'function_zxy', 'plot-function');
if isfolder(gjb_function_dir)
    addpath(gjb_function_dir);
end
if exist('calc_radial_error_gjb', 'file') ~= 2
    error('未找到已有分析函数 calc_radial_error_gjb.m。');
end
if ~isfile(truth_path)
    error('真值文件不存在：%s', truth_path);
end
if ~isfolder(range_baseline_dir)
    error('无掉帧仅测距结果目录不存在：%s', range_baseline_dir);
end
if ~isfolder(range_dropout_dir)
    error('掉帧仅测距结果目录不存在：%s', range_dropout_dir);
end

drop_indices = discover_drop_indices(range_dropout_dir);
if ~isempty(drop_indices_to_analyze)
    missing_requested = setdiff(drop_indices_to_analyze, drop_indices);
    if ~isempty(missing_requested)
        error('指定的掉帧编号不存在：%s', num2str(missing_requested));
    end
    drop_indices = intersect(drop_indices, drop_indices_to_analyze, ...
        'stable');
end
fprintf('检测到掉帧编号：%s\n', num2str(drop_indices));

method_labels = [{'仅测距'}; arrayfun(@(value) sprintf( ...
    '\\sigma_{\\phi}=%g^\\circ', value), phase_std_degs(:), ...
    'UniformOutput', false)];
method_count = numel(method_labels);

%% 构造并检查全部输入文件
baseline_paths = cell(numel(stage_ids), method_count);
for stage_index = 1:numel(stage_ids)
    baseline_paths{stage_index, 1} = fullfile(range_baseline_dir, ...
        range_only_baseline_names{stage_index});
    for phase_index = 1:numel(phase_std_degs)
        phase_dir = fullfile(navigation_root, sprintf( ...
            'double-hydrophone-phasestd%gdeg', ...
            phase_std_degs(phase_index)));
        baseline_paths{stage_index, phase_index + 1} = ...
            fullfile(phase_dir, ...
            range_azimuth_baseline_names{stage_index});
    end
end

baseline_path_count = numel(baseline_paths);
required_path_count = baseline_path_count + numel(drop_indices) * ...
    numel(stage_ids) * method_count;
all_required_paths = cell(required_path_count, 1);
all_required_paths(1:baseline_path_count) = baseline_paths(:);
required_path_index = baseline_path_count;
for drop_case_index = 1:numel(drop_indices)
    drop_index = drop_indices(drop_case_index);
    for stage_index = 1:numel(stage_ids)
        required_path_index = required_path_index + 1;
        all_required_paths{required_path_index} = fullfile( ...
            range_dropout_dir, sprintf('range-noazi-%s-drop%d.nav', ...
            stage_ids{stage_index}, drop_index));
        for phase_index = 1:numel(phase_std_degs)
            phase_dropout_dir = fullfile(navigation_root, sprintf( ...
                'double-hydrophone-phasestd%gdeg', ...
                phase_std_degs(phase_index)), 'drop-hasazi-2RTS');
            required_path_index = required_path_index + 1;
            all_required_paths{required_path_index} = fullfile( ...
                phase_dropout_dir, sprintf( ...
                'range-azimuth-%s-drop%d.nav', ...
                stage_ids{stage_index}, drop_index));
        end
    end
end
missing_mask = ~cellfun(@isfile, all_required_paths);
if any(missing_mask)
    error('以下导航结果不存在：\n%s', ...
        strjoin(all_required_paths(missing_mask), newline));
end

%% 输出表初始化
summary_csv_path = fullfile(analysis_dir, ...
    'dropout-range-azimuth-summary.csv');
summary_xlsx_path = fullfile(analysis_dir, ...
    'dropout-range-azimuth-summary.xlsx');
if isfile(summary_xlsx_path)
    delete(summary_xlsx_path);
end

no_dropout_header = {
    '处理阶段', '方法', '是否使用测向', '相位差噪声标准差 (deg)', ...
    '导航文件', 'RMS (m)', 'Max (m)', 'RER (n mile/h)', 'Median (m)'};
no_dropout_body = cell(numel(stage_ids) * method_count, ...
    numel(no_dropout_header));
no_dropout_row = 0;
no_dropout_numeric = nan(numel(stage_ids), method_count, 4);

summary_header = {
    '掉帧编号', '处理阶段', '方法', '是否使用测向', ...
    '相位差噪声标准差 (deg)', '掉帧导航文件', '无掉帧基准文件', ...
    '掉帧 RMS (m)', '掉帧 Max (m)', '掉帧 RER (n mile/h)', ...
    '掉帧 Median (m)', '无掉帧 RMS (m)', '无掉帧 Max (m)', ...
    '无掉帧 RER (n mile/h)', '无掉帧 Median (m)', ...
    'RMS 掉帧影响 (%)', 'Max 掉帧影响 (%)', ...
    'RER 掉帧影响 (%)', 'Median 掉帧影响 (%)', ...
    'RMS 测向改善 (%)', 'Max 测向改善 (%)', ...
    'RER 测向改善 (%)', 'Median 测向改善 (%)'};
summary_body = cell(numel(drop_indices) * numel(stage_ids) * ...
    method_count, numel(summary_header));
summary_row = 0;

%% 无掉帧基准统计
for stage_index = 1:numel(stage_ids)
    stage_label = stage_labels{stage_index};
    paths_this_stage = baseline_paths(stage_index, :)';
    fprintf('\n无掉帧基准：%s，共 %d 个结果。\n', ...
        stage_label, method_count);
    [comparison_figure, statistics_cell] = calc_radial_error_gjb( ...
        truth_path, paths_this_stage{:}, apply_grubbs);
    numeric_statistics = extract_gjb_metrics(statistics_cell, ...
        method_count, sprintf('无掉帧/%s', stage_label));
    statistics_cell(2:end, 1) = method_labels;
    if generate_radial_error_figures
        format_gjb_figure(comparison_figure, method_labels, ...
            sprintf('无掉帧：%s', stage_label), show_figures);
        output_stem = sprintf('baseline-%s-radial-error', ...
            stage_ids{stage_index});
        save_figure_pair(comparison_figure, radial_figure_dir, ...
            output_stem, export_resolution_dpi);
        close_hidden_figure(comparison_figure, show_figures);
    elseif isgraphics(comparison_figure)
        close(comparison_figure);
    end
    writecell(statistics_cell, summary_xlsx_path, ...
        'Sheet', sprintf('base-%s', stage_ids{stage_index}));

    no_dropout_numeric(stage_index, :, :) = reshape( ...
        numeric_statistics, 1, method_count, 4);
    for method_index = 1:method_count
        no_dropout_row = no_dropout_row + 1;
        no_dropout_body(no_dropout_row, :) = {
            stage_label, method_labels{method_index}, method_index > 1, ...
            phase_std_for_method(method_index, phase_std_degs), ...
            paths_this_stage{method_index}, ...
            numeric_statistics(method_index, 1), ...
            numeric_statistics(method_index, 2), ...
            numeric_statistics(method_index, 3), ...
            numeric_statistics(method_index, 4)};
    end
end

%% 掉帧统计：无掉帧仅测距 + 掉帧仅测距 + 四种掉帧测距+测向
for drop_case_index = 1:numel(drop_indices)
    drop_index = drop_indices(drop_case_index);
    for stage_index = 1:numel(stage_ids)
        stage_id = stage_ids{stage_index};
        stage_label = stage_labels{stage_index};
        dropout_paths = cell(method_count, 1);
        dropout_paths{1} = fullfile(range_dropout_dir, sprintf( ...
            'range-noazi-%s-drop%d.nav', stage_id, drop_index));
        for phase_index = 1:numel(phase_std_degs)
            phase_dropout_dir = fullfile(navigation_root, sprintf( ...
                'double-hydrophone-phasestd%gdeg', ...
                phase_std_degs(phase_index)), 'drop-hasazi-2RTS');
            dropout_paths{phase_index + 1} = fullfile( ...
                phase_dropout_dir, sprintf( ...
                'range-azimuth-%s-drop%d.nav', stage_id, drop_index));
        end

        gjb_paths = [baseline_paths(stage_index, 1); dropout_paths];
        gjb_labels = [{sprintf('无掉帧仅测距，%s', stage_label)}; ...
            cellfun(@(label) sprintf('drop%d，%s', drop_index, label), ...
            method_labels, 'UniformOutput', false)];
        fprintf('\n掉帧 drop%d / %s，共 %d 个结果。\n', ...
            drop_index, stage_label, numel(gjb_paths));
        [comparison_figure, statistics_cell] = calc_radial_error_gjb( ...
            truth_path, gjb_paths{:}, apply_grubbs);
        numeric_statistics = extract_gjb_metrics(statistics_cell, ...
            numel(gjb_paths), sprintf('drop%d/%s', drop_index, stage_label));
        statistics_cell(2:end, 1) = gjb_labels;
        if generate_radial_error_figures
            format_gjb_figure(comparison_figure, gjb_labels, sprintf( ...
                '丢第 %d 帧：%s（含无掉帧仅测距基准）', ...
                drop_index, stage_label), show_figures);
            output_stem = sprintf('drop%02d-%s-radial-error', ...
                drop_index, stage_id);
            save_figure_pair(comparison_figure, radial_figure_dir, ...
                output_stem, export_resolution_dpi);
            close_hidden_figure(comparison_figure, show_figures);
        elseif isgraphics(comparison_figure)
            close(comparison_figure);
        end
        writecell(statistics_cell, summary_xlsx_path, ...
            'Sheet', sprintf('d%02d-%s', drop_index, stage_id));

        matched_range_baseline = numeric_statistics(1, :);
        dropout_range_statistics = numeric_statistics(2, :);
        for method_index = 1:method_count
            dropout_statistics = numeric_statistics(method_index + 1, :);
            if method_index == 1
                baseline_statistics = matched_range_baseline;
            else
                baseline_statistics = reshape(no_dropout_numeric( ...
                    stage_index, method_index, :), 1, 4);
            end

            summary_row = summary_row + 1;
            summary_body(summary_row, :) = {
                drop_index, stage_label, method_labels{method_index}, ...
                method_index > 1, ...
                phase_std_for_method(method_index, phase_std_degs), ...
                dropout_paths{method_index}, ...
                baseline_paths{stage_index, method_index}, ...
                dropout_statistics(1), dropout_statistics(2), ...
                dropout_statistics(3), dropout_statistics(4), ...
                baseline_statistics(1), baseline_statistics(2), ...
                baseline_statistics(3), baseline_statistics(4), ...
                relative_change_percent(dropout_statistics(1), ...
                baseline_statistics(1)), ...
                relative_change_percent(dropout_statistics(2), ...
                baseline_statistics(2)), ...
                relative_change_percent(dropout_statistics(3), ...
                baseline_statistics(3)), ...
                relative_change_percent(dropout_statistics(4), ...
                baseline_statistics(4)), ...
                relative_reduction_percent(dropout_statistics(1), ...
                dropout_range_statistics(1)), ...
                relative_reduction_percent(dropout_statistics(2), ...
                dropout_range_statistics(2)), ...
                relative_reduction_percent(dropout_statistics(3), ...
                dropout_range_statistics(3)), ...
                relative_reduction_percent(dropout_statistics(4), ...
                dropout_range_statistics(4))};
        end
    end
end

%% 保存表格
no_dropout_body = no_dropout_body(1:no_dropout_row, :);
summary_body = summary_body(1:summary_row, :);
writecell([summary_header; summary_body], summary_csv_path);
writecell([summary_header; summary_body], summary_xlsx_path, ...
    'Sheet', 'summary');
writecell([no_dropout_header; no_dropout_body], summary_xlsx_path, ...
    'Sheet', 'no-dropout');

mean_effects = build_mean_effects(summary_body, stage_labels, ...
    method_labels);
writecell(mean_effects, summary_xlsx_path, 'Sheet', 'mean-effects');

metric_definitions = {
    '项目', '定义或说明'
    '水平径向误差', 'r(t)=sqrt(e_N(t)^2+e_E(t)^2)，单位 m'
    'RMS', 'sqrt(mean(r(t)^2))，单位 m'
    'Max', 'max(r(t))，单位 m'
    'RER', '末时刻水平径向误差（海里）/累计时长（小时），单位 n mile/h'
    'RER 窗口', '评价时长不超过 3600 s；采用窗口末端误差'
    'Median', 'median(r(t))，单位 m'
    '测向改善率', ...
    '100*(同掉帧仅测距指标-同掉帧测距+测向指标)/同掉帧仅测距指标；正值表示改善'
    '掉帧影响率', ...
    '100*(掉帧指标-同方法无掉帧指标)/同方法无掉帧指标；正值表示恶化'
    '仅测距掉帧影响', ...
    '无掉帧仅测距被纳入对应 dropN 的 GJB 组，使用完全相同的公共时间段'
    '测距+测向掉帧影响', ...
    '无掉帧测距+测向取 no-dropout 组统计；若有效时长不同，完整航程指标可能包含窗口差异'
    '测距+测向掉帧目录', ...
    'double-hydrophone-phasestdXdeg/drop-hasazi-2RTS'
    'Grubbs', sprintf('apply_grubbs=%d', apply_grubbs)};
writecell(metric_definitions, summary_xlsx_path, 'Sheet', '指标说明');

%% 汇总图：绝对误差、测向改善、掉帧影响和 RTS 改善
if generate_summary_figures
delete_obsolete_absolute_metric_figures(summary_figure_dir, stage_ids);
for stage_index = 1:numel(stage_labels)
    create_absolute_metric_group_figure(summary_body, drop_indices, ...
        stage_labels{stage_index}, method_labels, [8, 9], [12, 13], ...
        {'RMS (m)', 'Max (m)'}, 1, 2, 11, 5.4, ...
        '核心绝对误差', sprintf('absolute-primary-rms-max-%s', ...
        stage_ids{stage_index}), summary_figure_dir, show_figures, ...
        export_resolution_dpi);
    create_absolute_metric_group_figure(summary_body, drop_indices, ...
        stage_labels{stage_index}, method_labels, ...
        [10, 11], [14, 15], {'RER (n mile/h)', 'Median (m)'}, ...
        1, 2, 11, 5.4, ...
        '补充绝对误差', sprintf('absolute-supplementary-metrics-%s', ...
        stage_ids{stage_index}), summary_figure_dir, show_figures, ...
        export_resolution_dpi);
end
delete_obsolete_combined_heatmaps(summary_figure_dir);

% 每个指标单独成图；三列依次为前向 EKF、一次 RTS 和二次 RTS。
heatmap_metrics = {
    'rms',    'RMS',     8, 12
    'max',    'MAX',     9, 13
    'rer',    'RER',    10, 14
    'median', 'Median', 11, 15};
for metric_index = 1:size(heatmap_metrics, 1)
    metric_id = heatmap_metrics{metric_index, 1};
    metric_title = heatmap_metrics{metric_index, 2};
    dropout_metric_column = heatmap_metrics{metric_index, 3};
    baseline_metric_column = heatmap_metrics{metric_index, 4};

    create_single_metric_effect_heatmap(summary_body, drop_indices, ...
        stage_labels, method_labels(2:end), dropout_metric_column, ...
        baseline_metric_column, metric_title, 'azimuth-improvement', ...
        sprintf('azimuth-improvement-%s-heatmap', metric_id), ...
        summary_figure_dir, show_figures, export_resolution_dpi);
    create_single_metric_effect_heatmap(summary_body, drop_indices, ...
        stage_labels, method_labels, dropout_metric_column, ...
        baseline_metric_column, metric_title, 'dropout-impact', ...
        sprintf('dropout-impact-%s-heatmap', metric_id), ...
        summary_figure_dir, show_figures, export_resolution_dpi);
end
for method_index = 1:method_count
    create_rts_improvement_figure(summary_body, drop_indices, ...
        stage_labels, method_labels{method_index}, summary_figure_dir, ...
        show_figures, export_resolution_dpi);
end
end

fprintf('\n统一分析完成。\n');
fprintf('汇总 CSV：%s\n', summary_csv_path);
fprintf('多工作表 Excel：%s\n', summary_xlsx_path);
if generate_radial_error_figures
    fprintf('径向误差图：%s\n', radial_figure_dir);
else
    fprintf('径向误差图：未生成（generate_radial_error_figures=false）\n');
end
if generate_summary_figures
    fprintf('汇总图：%s\n', summary_figure_dir);
else
    fprintf('汇总图：未生成（generate_summary_figures=false）\n');
end

%% 局部函数
function ensure_directory(directory_path)
if ~isfolder(directory_path)
    mkdir(directory_path);
end
end

function delete_obsolete_combined_heatmaps(output_dir)
obsolete_stems = {'azimuth-improvement-heatmaps', ...
    'dropout-impact-heatmaps', ...
    'azimuth-improvement-cep50-heatmap', ...
    'azimuth-improvement-p95-heatmap', ...
    'azimuth-improvement-mean-heatmap', ...
    'azimuth-improvement-position-error-rate-heatmap', ...
    'dropout-impact-cep50-heatmap', ...
    'dropout-impact-p95-heatmap', ...
    'dropout-impact-mean-heatmap', ...
    'dropout-impact-position-error-rate-heatmap'};
extensions = {'.png', '.fig'};
for stem_index = 1:numel(obsolete_stems)
    for extension_index = 1:numel(extensions)
        obsolete_path = fullfile(output_dir, ...
            [obsolete_stems{stem_index}, extensions{extension_index}]);
        if isfile(obsolete_path)
            delete(obsolete_path);
        end
    end
end
end

function delete_obsolete_absolute_metric_figures(output_dir, stage_ids)
extensions = {'.png', '.fig'};
for stage_index = 1:numel(stage_ids)
    obsolete_stem = sprintf('absolute-metrics-%s', ...
        stage_ids{stage_index});
    for extension_index = 1:numel(extensions)
        obsolete_path = fullfile(output_dir, ...
            [obsolete_stem, extensions{extension_index}]);
        if isfile(obsolete_path)
            delete(obsolete_path);
        end
    end
end
end

function drop_indices = discover_drop_indices(range_dropout_dir)
files = dir(fullfile(range_dropout_dir, ...
    'range-noazi-rts-double-drop*.nav'));
drop_indices = zeros(numel(files), 1);
valid_count = 0;
for file_index = 1:numel(files)
    token = regexp(files(file_index).name, ...
        'drop([0-9]+)\.nav$', 'tokens', 'once');
    if isempty(token)
        continue;
    end
    valid_count = valid_count + 1;
    drop_indices(valid_count) = str2double(token{1});
end
drop_indices = sort(unique(drop_indices(1:valid_count)))';
if isempty(drop_indices)
    error('没有发现掉帧仅测距导航结果：%s', range_dropout_dir);
end
end

function phase_std_deg = phase_std_for_method(method_index, phase_std_degs)
if method_index == 1
    phase_std_deg = nan;
else
    phase_std_deg = phase_std_degs(method_index - 1);
end
end

function numeric_metrics = extract_gjb_metrics(statistics_cell, ...
        expected_count, context)
required_headers = {'RMS (m)', 'Max (m)', 'RER (n mile/h)', 'Median (m)'};
if size(statistics_cell, 1) ~= expected_count + 1
    error('%s 的 GJB 统计表行数异常。', context);
end
headers = string(statistics_cell(1, :));
metric_columns = zeros(1, numel(required_headers));
for metric_index = 1:numel(required_headers)
    column_index = find(headers == required_headers{metric_index}, 1);
    if isempty(column_index)
        error('%s 缺少评价指标列：%s。', ...
            context, required_headers{metric_index});
    end
    metric_columns(metric_index) = column_index;
end
numeric_metrics = cell2mat(statistics_cell(2:end, metric_columns));
end

function format_gjb_figure(figure_handle, labels, title_text, show_figures)
if ~isgraphics(figure_handle)
    return;
end
if ~show_figures
    set(figure_handle, 'Visible', 'off');
end
axes_handles = findobj(figure_handle, 'Type', 'Axes');
if isempty(axes_handles)
    return;
end
axes_handle = axes_handles(1);
line_handles = flipud(findobj(axes_handle, 'Type', 'Line'));
line_count = min(numel(line_handles), numel(labels));
for line_index = 1:line_count
    set(line_handles(line_index), 'DisplayName', labels{line_index}, ...
        'LineWidth', 1.1);
end
title(axes_handle, title_text, 'Interpreter', 'none');
xlabel(axes_handle, '时间 (s)');
ylabel(axes_handle, '水平径向误差 (m)');
legend(axes_handle, 'show', 'Location', 'best', 'Interpreter', 'tex');
grid(axes_handle, 'on');
box(axes_handle, 'on');
set(findall(figure_handle, '-property', 'FontName'), ...
    'FontName', 'TimesSimSun');
end

function save_figure_pair(figure_handle, output_dir, output_stem, dpi)
exportgraphics(figure_handle, fullfile(output_dir, ...
    sprintf('%s.png', output_stem)), 'Resolution', dpi);
savefig(figure_handle, fullfile(output_dir, ...
    sprintf('%s.fig', output_stem)));
end

function close_hidden_figure(figure_handle, show_figures)
if ~show_figures && isgraphics(figure_handle)
    close(figure_handle);
end
end

function change_percent = relative_change_percent(value, baseline)
if isfinite(value) && isfinite(baseline) && baseline ~= 0
    change_percent = 100 * (value - baseline) / baseline;
else
    change_percent = nan;
end
end

function reduction_percent = relative_reduction_percent(value, baseline)
if isfinite(value) && isfinite(baseline) && baseline ~= 0
    reduction_percent = 100 * (baseline - value) / baseline;
else
    reduction_percent = nan;
end
end

function mean_effects = build_mean_effects(summary_body, ...
        stage_labels, method_labels)
header = {'处理阶段', '方法', '平均 RMS 掉帧影响 (%)', ...
    '平均 Max 掉帧影响 (%)', '平均 RER 掉帧影响 (%)', ...
    '平均 Median 掉帧影响 (%)', '平均 RMS 测向改善 (%)', ...
    '平均 Max 测向改善 (%)', '平均 RER 测向改善 (%)', ...
    '平均 Median 测向改善 (%)'};
body = cell(numel(stage_labels) * numel(method_labels), numel(header));
row_index = 0;
for stage_index = 1:numel(stage_labels)
    for method_index = 1:numel(method_labels)
        mask = strcmp(summary_body(:, 2), stage_labels{stage_index}) & ...
            strcmp(summary_body(:, 3), method_labels{method_index});
        row_index = row_index + 1;
        body(row_index, :) = {stage_labels{stage_index}, ...
            method_labels{method_index}, ...
            mean(cell2mat(summary_body(mask, 16)), 'omitnan'), ...
            mean(cell2mat(summary_body(mask, 17)), 'omitnan'), ...
            mean(cell2mat(summary_body(mask, 18)), 'omitnan'), ...
            mean(cell2mat(summary_body(mask, 19)), 'omitnan'), ...
            mean(cell2mat(summary_body(mask, 20)), 'omitnan'), ...
            mean(cell2mat(summary_body(mask, 21)), 'omitnan'), ...
            mean(cell2mat(summary_body(mask, 22)), 'omitnan'), ...
            mean(cell2mat(summary_body(mask, 23)), 'omitnan')};
    end
end
mean_effects = [header; body];
end

function create_absolute_metric_group_figure(summary_body, drop_indices, ...
        stage_label, method_labels, drop_metric_columns, ...
        baseline_metric_columns, metric_labels, layout_rows, ...
        layout_columns, figure_width, figure_height, group_title, ...
        output_stem, output_dir, show_figures, dpi)
figure_handle = myfigurestartup(figure_width, figure_height, 'prese');
if ~show_figures
    set(figure_handle, 'Visible', 'off');
end
layout = tiledlayout(figure_handle, layout_rows, layout_columns, ...
    'TileSpacing', 'loose', 'Padding', 'compact');
colors = lines(numel(method_labels));
markers = {'o', 's', '^', 'd', 'v'};
legend_handles = gobjects(numel(method_labels), 1);

for metric_index = 1:numel(drop_metric_columns)
    axes_handle = nexttile(layout);
    hold(axes_handle, 'on');
    for method_index = 1:numel(method_labels)
        mask = strcmp(summary_body(:, 2), stage_label) & ...
            strcmp(summary_body(:, 3), method_labels{method_index});
        x = cell2mat(summary_body(mask, 1));
        drop_values = cell2mat(summary_body(mask, ...
            drop_metric_columns(metric_index)));
        baseline_values = cell2mat(summary_body(mask, ...
            baseline_metric_columns(metric_index)));
        [x, order] = sort(x);
        drop_values = drop_values(order);
        baseline_values = baseline_values(order);
        line_handle = plot(axes_handle, x, drop_values, ...
            ['-' markers{method_index}], ...
            'Color', colors(method_index, :), 'LineWidth', 1.3, ...
            'MarkerSize', 6, 'DisplayName', method_labels{method_index});
        if metric_index == 1
            legend_handles(method_index) = line_handle;
        end
        plot(axes_handle, x, baseline_values, '--', ...
            'Color', colors(method_index, :), 'LineWidth', 0.9, ...
            'HandleVisibility', 'off');
    end
    xticks(axes_handle, drop_indices);
    xtickangle(axes_handle, 0);
    xlabel(axes_handle, '掉帧编号');
    ylabel(axes_handle, metric_labels{metric_index});
    title(axes_handle, metric_labels{metric_index});
    grid(axes_handle, 'on');
    box(axes_handle, 'on');
end
legend_handle = legend(legend_handles, method_labels, ...
    'Orientation', 'horizontal', 'Interpreter', 'tex', ...
    'NumColumns', numel(method_labels));
legend_handle.Layout.Tile = 'north';
title(layout, sprintf('%s：%s（实线为掉帧，虚线为同方法无掉帧）', ...
    stage_label, group_title), 'Interpreter', 'none');
set(findall(figure_handle, '-property', 'FontName'), ...
    'FontName', 'TimesSimSun');
save_figure_pair(figure_handle, output_dir, output_stem, dpi);
close_hidden_figure(figure_handle, show_figures);
end

function create_single_metric_effect_heatmap(summary_body, drop_indices, ...
        stage_labels, displayed_methods, dropout_metric_column, ...
        baseline_metric_column, metric_title, effect_kind, output_stem, ...
        output_dir, show_figures, dpi)
is_improvement = strcmp(effect_kind, 'azimuth-improvement');
if ~is_improvement && ~strcmp(effect_kind, 'dropout-impact')
    error('未知热力图类型：%s', effect_kind);
end

matrices = cell(numel(stage_labels), 1);
all_values = [];
drop_number_column = cell2mat(summary_body(:, 1));
for stage_index = 1:numel(stage_labels)
    values = nan(numel(displayed_methods), numel(drop_indices));
    for drop_case_index = 1:numel(drop_indices)
        range_mask = strcmp(summary_body(:, 2), ...
            stage_labels{stage_index}) & ...
            strcmp(summary_body(:, 3), '仅测距') & ...
            drop_number_column == drop_indices(drop_case_index);
        if is_improvement && any(range_mask)
            range_value = summary_body{find(range_mask, 1), ...
                dropout_metric_column};
        else
            range_value = nan;
        end

        for method_index = 1:numel(displayed_methods)
            mask = strcmp(summary_body(:, 2), ...
                stage_labels{stage_index}) & ...
                strcmp(summary_body(:, 3), ...
                displayed_methods{method_index}) & ...
                drop_number_column == drop_indices(drop_case_index);
            if ~any(mask)
                continue;
            end
            row_index = find(mask, 1);
            dropout_value = summary_body{row_index, ...
                dropout_metric_column};
            if is_improvement
                values(method_index, drop_case_index) = ...
                    relative_reduction_percent(dropout_value, range_value);
            else
                baseline_value = summary_body{row_index, ...
                    baseline_metric_column};
                values(method_index, drop_case_index) = ...
                    relative_change_percent(dropout_value, baseline_value);
            end
        end
    end
    matrices{stage_index} = values;
    all_values = [all_values; values(:)]; %#ok<AGROW>
end

finite_values = all_values(isfinite(all_values));
if isempty(finite_values)
    color_limit = 10;
else
    color_limit = max(10, ceil(max(abs(finite_values)) / 10) * 10);
end

% azimuth-improvement-*-heatmap 使用固定 10:3 画布比例；
% dropout-impact 当前沿用相同尺寸以保持两组热力图版式一致。
figure_handle = myfigurestartup(10, 3, 'prese');
if ~show_figures
    set(figure_handle, 'Visible', 'off');
end
layout = tiledlayout(figure_handle, 1, numel(stage_labels), ...
    'TileSpacing', 'compact', 'Padding', 'compact');
for stage_index = 1:numel(stage_labels)
    axes_handle = nexttile(layout);
    values = matrices{stage_index};
    imagesc(axes_handle, values);
    if is_improvement
        % 正值表示加入测向后误差降低，对应参考图中的蓝色。
        colormap(axes_handle, red_white_blue_colormap(128));
    else
        % 正值表示掉帧后误差增加，对应参考图中的红色。
        colormap(axes_handle, blue_white_red_colormap(128));
    end
    clim(axes_handle, [-color_limit, color_limit]);
    xlim(axes_handle, [0.5, numel(drop_indices) + 0.5]);
    ylim(axes_handle, [0.5, numel(displayed_methods) + 0.5]);
    xticks(axes_handle, 1:numel(drop_indices));
    xticklabels(axes_handle, string(drop_indices));
    yticks(axes_handle, 1:numel(displayed_methods));
    yticklabels(axes_handle, displayed_methods);
    set(axes_handle, 'TickLabelInterpreter', 'tex', 'FontSize', 9, ...
        'LineWidth', 0.8, 'Box', 'on', 'Layer', 'top');
    xlabel(axes_handle, '掉帧编号');
    title(axes_handle, sprintf('%s / %s', ...
        stage_labels{stage_index}, metric_title), ...
        'Interpreter', 'none', 'FontWeight', 'normal');
    colorbar_handle = colorbar(axes_handle);
    colorbar_handle.Ticks = linspace(-color_limit, color_limit, 5);
    colorbar_handle.Label.String = '变化 (%)';
    colorbar_handle.FontSize = 8;

    for row_index = 1:size(values, 1)
        for column_index = 1:size(values, 2)
            if ~isfinite(values(row_index, column_index))
                label_text = '--';
            else
                label_text = sprintf('%+.1f%%', ...
                    values(row_index, column_index));
            end
            text(axes_handle, column_index, row_index, label_text, ...
                'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'middle', 'FontSize', 7, ...
                'FontWeight', 'bold', 'Interpreter', 'none', ...
                'Color', [0.08, 0.08, 0.08]);
        end
    end
end
set(findall(figure_handle, '-property', 'FontName'), ...
    'FontName', 'TimesSimSun');
save_figure_pair(figure_handle, output_dir, output_stem, dpi);
close_hidden_figure(figure_handle, show_figures);
end

function create_rts_improvement_figure(summary_body, drop_indices, ...
        stage_labels, method_label, output_dir, show_figures, dpi)
metric_columns = [8, 9, 10, 11];
metric_labels = {'RMS (m)', 'Max (m)', 'RER (n mile/h)', 'Median (m)'};
figure_handle = myfigurestartup(11, 8.2, 'prese');
if ~show_figures
    set(figure_handle, 'Visible', 'off');
end
layout = tiledlayout(figure_handle, 2, 2, ...
    'TileSpacing', 'loose', 'Padding', 'compact');
legend_handles = gobjects(2, 1);

for metric_index = 1:numel(metric_columns)
    values = nan(numel(drop_indices), numel(stage_labels));
    for stage_index = 1:numel(stage_labels)
        mask = strcmp(summary_body(:, 2), stage_labels{stage_index}) & ...
            strcmp(summary_body(:, 3), method_label);
        x = cell2mat(summary_body(mask, 1));
        y = cell2mat(summary_body(mask, metric_columns(metric_index)));
        [x, order] = sort(x);
        [~, locations] = ismember(x, drop_indices);
        values(locations, stage_index) = y(order);
    end
    forward_values = values(:, 1);
    gains = [
        100 * (forward_values - values(:, 2)) ./ forward_values, ...
        100 * (forward_values - values(:, 3)) ./ forward_values];
    axes_handle = nexttile(layout);
    bars = bar(axes_handle, drop_indices, gains, 'grouped');
    bars(1).FaceColor = [0.93, 0.56, 0.25];
    bars(2).FaceColor = [0.22, 0.47, 0.76];
    if metric_index == 1
        legend_handles = bars(:);
    end
    xlabel(axes_handle, '掉帧编号');
    ylabel(axes_handle, '相对前向 EKF 误差降低 (%)');
    title(axes_handle, metric_labels{metric_index});
    xticks(axes_handle, drop_indices);
    grid(axes_handle, 'on');
    box(axes_handle, 'on');
end
legend_handle = legend(legend_handles, {'一次 RTS', '二次 RTS'}, ...
    'Orientation', 'horizontal', 'Interpreter', 'tex', ...
    'NumColumns', 2);
legend_handle.Layout.Tile = 'north';
title(layout, sprintf('%s：RTS 相对前向 EKF 的改善', method_label), ...
    'Interpreter', 'tex');
set(findall(figure_handle, '-property', 'FontName'), ...
    'FontName', 'TimesSimSun');
output_stem = sprintf('rts-improvement-%s', ...
    matlab.lang.makeValidName(method_label));
save_figure_pair(figure_handle, output_dir, output_stem, dpi);
close_hidden_figure(figure_handle, show_figures);
end

function cmap = red_white_blue_colormap(half_count)
red = [0.85, 0.30, 0.25];
blue = [0.25, 0.45, 0.80];
cmap = [
    linspace(red(1), 1, half_count)', ...
    linspace(red(2), 1, half_count)', ...
    linspace(red(3), 1, half_count)'
    linspace(1, blue(1), half_count)', ...
    linspace(1, blue(2), half_count)', ...
    linspace(1, blue(3), half_count)'];
end

function cmap = blue_white_red_colormap(half_count)
blue = [0.25, 0.45, 0.80];
red = [0.85, 0.30, 0.25];
cmap = [
    linspace(blue(1), 1, half_count)', ...
    linspace(blue(2), 1, half_count)', ...
    linspace(blue(3), 1, half_count)'
    linspace(1, red(1), half_count)', ...
    linspace(1, red(2), half_count)', ...
    linspace(1, red(3), half_count)'];
end
