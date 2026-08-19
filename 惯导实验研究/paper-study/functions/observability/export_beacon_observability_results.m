function export_files = export_beacon_observability_results(results, output_dir, varargin)
%EXPORT_BEACON_OBSERVABILITY_RESULTS 在函数外选择性导出可观测性分析结果。
%
%   export_files = export_beacon_observability_results(results, output_dir)
%   export_files = export_beacon_observability_results(results, output_dir, Name, Value, ...)
%
% 输入：
%   results     analyze_beacon_observability_v2 返回的结果结构体。
%   output_dir  输出目录。若目录不存在，则自动创建。
%
% 可选 Name-Value 参数：
%   'Obs'                    原始 obs 结构体。若导出 MAT 文件且提供该参数，
%                            MAT 文件中同时保存 results 和 obs；否则只保存 results。
%   'ExportSummaryCSV'       是否导出汇总 CSV，默认 true。
%   'ExportWindowCSV'        是否导出逐窗口 CSV，默认 true。
%   'ExportMat'              是否导出 MAT 文件，默认 true。
%   'ExportComparisonFigure' 是否导出时间序列对比图，默认 true。
%   'ExportBoxFigure'        是否导出箱线图，默认 true。
%   'FilePrefix'             输出文件名前缀，默认 'beacon_observability'。
%   'Resolution'             图片分辨率，默认 600 dpi。
%   'CloseFigure'            导出后是否关闭图窗，默认 true。
%
% 示例 1：只计算，不导出
%   results = analyze_beacon_observability_v2(obs, 18);
%
% 示例 2：只导出 CSV
%   export_beacon_observability_results(results, './output', ...
%       'ExportMat', false, ...
%       'ExportComparisonFigure', false, ...
%       'ExportBoxFigure', false);
%
% 示例 3：导出全部结果，并在 MAT 文件中保存 obs
%   export_beacon_observability_results(results, './output', 'Obs', obs);

    % ---------- 解析导出选项 ----------
    p = inputParser;
    p.FunctionName = mfilename;
    addRequired(p, 'results', @isstruct);
    addRequired(p, 'output_dir', @(x) ischar(x) || isstring(x));
    addParameter(p, 'Obs', [], @(x) isstruct(x) || isempty(x));
    addParameter(p, 'ExportSummaryCSV', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'ExportWindowCSV', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'ExportMat', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'ExportComparisonFigure', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'ExportBoxFigure', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'FilePrefix', 'beacon_observability', @(x) ischar(x) || isstring(x));
    addParameter(p, 'Resolution', 600, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(p, 'CloseFigure', true, @(x) islogical(x) || isnumeric(x));
    parse(p, results, output_dir, varargin{:});

    opts = p.Results;
    output_dir = char(output_dir);
    file_prefix = char(opts.FilePrefix);

    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    % ---------- 检查必要结果字段 ----------
    required_fields = {'summary', 'window_table', 'time', 'scenario_names', ...
        'horizontal_lambda_min', 'horizontal_condition', 'horizontal_logdet'};
    for i = 1:numel(required_fields)
        if ~isfield(results, required_fields{i})
            error('results 缺少必要字段：%s。请确认其由 analyze_beacon_observability_v2 生成。', ...
                required_fields{i});
        end
    end

    export_files = struct();

    % ---------- 导出汇总表 ----------
    if logical(opts.ExportSummaryCSV)
        export_files.summary_csv = fullfile(output_dir, [file_prefix '_summary.csv']);
        writetable(results.summary, export_files.summary_csv);
    end

    % ---------- 导出逐窗口明细表 ----------
    if logical(opts.ExportWindowCSV)
        export_files.window_csv = fullfile(output_dir, [file_prefix '_windows.csv']);
        writetable(results.window_table, export_files.window_csv);
    end

    % ---------- 导出 MAT 数据 ----------
    if logical(opts.ExportMat)
        export_files.mat = fullfile(output_dir, [file_prefix '_results.mat']);
        if isempty(opts.Obs)
            save(export_files.mat, 'results');
        else
            obs = opts.Obs; %#ok<NASGU>
            save(export_files.mat, 'results', 'obs');
        end
    end

    % ---------- 导出时间序列对比图 ----------
    if logical(opts.ExportComparisonFigure)
        export_files.comparison_png = fullfile(output_dir, [file_prefix '_comparison.png']);
        fig = plot_comparison_figure(results);
        exportgraphics(fig, export_files.comparison_png, 'Resolution', opts.Resolution);
        if logical(opts.CloseFigure)
            close(fig);
        end
    end

    % ---------- 导出箱线图 ----------
    if logical(opts.ExportBoxFigure)
        export_files.box_png = fullfile(output_dir, [file_prefix '_boxplots.png']);
        export_files.box_pdf = fullfile(output_dir, [file_prefix '_boxplots.pdf']);
        fig = plot_box_figure(results);
        exportgraphics(fig, export_files.box_png, 'Resolution', opts.Resolution);
        exportgraphics(fig, export_files.box_pdf, "ContentType", "vector");
        if logical(opts.CloseFigure)
            close(fig);
        end
    end

    fprintf('可观测性结果导出完成：%s\n', output_dir);
end

function fig = plot_comparison_figure(results)
%PLOT_COMPARISON_FIGURE 生成滑动窗口指标随时间变化的对比图。

    num_scenarios = numel(results.scenario_names);
    colors = lines(num_scenarios);

    fig = create_observability_figure(9, 3);
    t = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile(t, 1);
    line_handles = plot_metric(ax1, results.time, results.horizontal_lambda_min, colors, false);
    ylabel(ax1, '\lambda_{min}(W_{horizontal})');
    xlabel(ax1, 'Time (s)');

    ax = nexttile(t, 2);
    plot_metric(ax, results.time, results.horizontal_condition, colors, true);
    ylabel(ax, 'Condition number');
    xlabel(ax, 'Time (s)');

    ax = nexttile(t, 3);
    plot_metric(ax, results.time, results.horizontal_logdet, colors, false);
    ylabel(ax, 'log_{10} det(W_{horizontal})');
    xlabel(ax, 'Time (s)');

    valid_idx = isgraphics(line_handles);
    valid_handles = line_handles(valid_idx);
    if ~isempty(valid_handles)
        legend(ax1, valid_handles, results.scenario_names(valid_idx), 'Location', 'best');
    end
end

function fig = plot_box_figure(results)
%PLOT_BOX_FIGURE 生成不同信标工况下滑动窗口指标的分布箱线图。

    num_scenarios = numel(results.scenario_names);
    num_windows = size(results.horizontal_lambda_min, 1);
    colors = lines(num_scenarios);
    short_labels = make_short_labels(results.scenario_names);

    fig = create_observability_figure(9, 3);
    t = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

    % 注意：(:) 按列展开，因此这里使用 kron 与指标矩阵展开顺序保持一致。
    scenario_idx = kron((1:num_scenarios)', ones(num_windows, 1));
    scenario_cat = categorical(scenario_idx, 1:num_scenarios, short_labels, 'Ordinal', true);

    ax = nexttile(t, 1);
    plot_box_metric(ax, scenario_cat, results.horizontal_lambda_min(:), colors, ...
        '\lambda_{min}(W_{horizontal})');

    ax = nexttile(t, 2);
    plot_box_metric(ax, scenario_cat, results.horizontal_condition(:), colors, ...
        'Condition number');
    set(ax, 'YScale', 'log');

    ax = nexttile(t, 3);
    plot_box_metric(ax, scenario_cat, results.horizontal_logdet(:), colors, ...
        'log_{10} det(W_{horizontal})');
end

function fig = create_observability_figure(width_cm, height_cm)
%CREATE_OBSERVABILITY_FIGURE 创建图窗。若存在 myfigurestartup，则优先使用原工程风格。

    if exist('myfigurestartup', 'file') == 2
        fig = myfigurestartup(width_cm, height_cm, 'zxy');
    else
        fig = figure('Color', 'w', 'Units', 'centimeters', ...
            'Position', [2, 2, width_cm, height_cm]);
    end
end

function line_handles = plot_metric(ax, time, values, colors, use_log_scale)
%PLOT_METRIC 绘制单个指标的多工况时间序列。

    hold(ax, 'on');
    line_handles = gobjects(1, size(values, 2));

    for scenario_id = 1:size(values, 2)
        valid = isfinite(time) & isfinite(values(:, scenario_id));
        if any(valid)
            line_handles(scenario_id) = plot(ax, time(valid), values(valid, scenario_id), ...
                'LineWidth', 1.4, ...
                'Color', colors(scenario_id, :));
        end
    end

    if use_log_scale
        set(ax, 'YScale', 'log');
    end

    grid(ax, 'on');
    box(ax, 'on');
    if any(isfinite(time))
        xlim(ax, [min(time), max(time)]);
    end
    set(ax, 'FontSize', 10, 'LineWidth', 1.0);
end

function plot_box_metric(ax, scenario_cat, values, colors, y_label_text)
%PLOT_BOX_METRIC 绘制单个指标的多工况箱线图。

    hold(ax, 'on');
    cats = categories(scenario_cat);

    for scenario_id = 1:numel(cats)
        idx = (scenario_cat == cats{scenario_id}) & isfinite(values);
        if any(idx)
            boxchart(ax, scenario_cat(idx), values(idx), ...
                'BoxFaceColor', colors(scenario_id, :), ...
                'BoxFaceAlpha', 0.75, ...
                'MarkerStyle', '.', ...
                'MarkerColor', [0.35 0.35 0.35], ...
                'LineWidth', 1.0);
        end
    end

    ylabel(ax, y_label_text);
    grid(ax, 'on');
    box(ax, 'on');
    set(ax, 'FontSize', 10, 'LineWidth', 1.0);
end

function short_labels = make_short_labels(scenario_names)
%MAKE_SHORT_LABELS 生成适合箱线图横轴显示的短标签。

    num_scenarios = numel(scenario_names);
    short_labels = cell(1, num_scenarios);
    for k = 1:num_scenarios
        name = scenario_names{k};
        if strcmp(name, 'Alternating')
            short_labels{k} = 'Alt';
        else
            tok = regexp(name, 'Fixed-B(\d+)', 'tokens', 'once');
            if ~isempty(tok)
                short_labels{k} = ['B' tok{1}];
            else
                short_labels{k} = name;
            end
        end
    end
end
