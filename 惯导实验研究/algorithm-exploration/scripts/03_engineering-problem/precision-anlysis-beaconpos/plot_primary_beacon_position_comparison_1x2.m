function output_paths = plot_primary_beacon_position_comparison_1x2(varargin)
%PLOT_PRIMARY_BEACON_POSITION_COMPARISON_1X2 Combine EKF and single RTS plots.
%   The function reads the existing navigation-error-forward-ekf.fig and
%   navigation-error-rts-single.fig files, keeps only the uncompensated,
%   position-uncertainty compensation, and true-beacon-position curves,
%   and exports a clean 1-by-2 comparison figure.
%
%   Example:
%     plot_primary_beacon_position_comparison_1x2
%
%     plot_primary_beacon_position_comparison_1x2( ...
%         'DataSource', "simulation", ...
%         'DatasetId', "case-06", ...
%         'StudyId', ...
%         "beacon-position-time-varying-circle-61m-initial10m")

parser = inputParser;
parser.addParameter('DataSource', "simulation", ...
    @(x) isstring(x) || ischar(x));
parser.addParameter('DatasetId', "case-06", ...
    @(x) isstring(x) || ischar(x));
parser.addParameter('StudyId', ...
    "beacon-position-time-varying-circle-61m-initial10m", ...
    @(x) isstring(x) || ischar(x));
parser.addParameter('FigureTitle', "24 h仿真潜标位置处理方法对比", ...
    @(x) isstring(x) || ischar(x));
parser.addParameter('OutputStem', ...
    "navigation-error-forward-rts-single-primary-comparison-1x2", ...
    @(x) isstring(x) || ischar(x));
parser.addParameter('CloseFigure', true, ...
    @(x) islogical(x) && isscalar(x));
parser.parse(varargin{:});
options = parser.Results;

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));
addpath(topic_dir);
paths = setup_inertial_experiment();

artifact_root = fullfile(paths.data, string(options.DataSource), ...
    string(options.DatasetId), 'output', 'figures-tables', ...
    string(options.StudyId));
if ~isfolder(artifact_root)
    error('结果目录不存在：%s', artifact_root);
end

panels = struct( ...
    'source_stem', {"navigation-error-forward-ekf", ...
                    "navigation-error-rts-single"}, ...
    'title', {"(a) 前向EKF", "(b) 一次RTS"});

source_name_groups = { ...
    ["未补偿（固定潜标位置）", "未补偿"], ...
    ["等效量测噪声补偿", "位置不确定度补偿", ...
     "本方法（位置不确定度补偿）"], ...
    "真实潜标位置"};
display_names = ["未补偿（固定测距方差）", ...
    "本方法（位置不确定度补偿）", "真实潜标位置"];
colors = [0.00 0.00 0.00; 0.85 0.15 0.15; 0.10 0.35 0.70];

panel_data = cell(numel(panels), numel(display_names));
maximum_time_h = 0;
maximum_error_m = 0;
for panel_index = 1:numel(panels)
    source_path = fullfile(artifact_root, ...
        panels(panel_index).source_stem + ".fig");
    if ~isfile(source_path)
        error('缺少源FIG：%s', source_path);
    end
    source_figure = openfig(source_path, 'invisible');
    cleanup = onCleanup(@() close_valid_figure(source_figure));
    source_axes = select_main_axes(source_figure);
    source_lines = findobj(source_axes, 'Type', 'line');

    for method_index = 1:numel(display_names)
        matched_line = find_named_line(source_lines, ...
            source_name_groups{method_index});
        time_h = double(get(matched_line, 'XData')) / 3600;
        error_m = double(get(matched_line, 'YData'));
        panel_data{panel_index, method_index} = struct( ...
            'time_h', time_h, 'error_m', error_m);
        maximum_time_h = max(maximum_time_h, max(time_h, [], 'all'));
        maximum_error_m = max(maximum_error_m, max(error_m, [], 'all'));
    end
    clear cleanup;
end

% figure_handle = figure('Color', 'w', 'Units', 'centimeters', ...
%     'Position', [2 2 32 12], 'Visible', 'off');
figure_handle = myfigurestartup(10,3,'paper');
layout = tiledlayout(figure_handle, 1, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
axes_handles = gobjects(numel(panels),1);
line_handles = gobjects(numel(panels),numel(display_names));

shared_upper_limit_m = max(100, ceil(maximum_error_m/500)*500);
shared_upper_time_h = max(1, ceil(maximum_time_h));
for panel_index = 1:numel(panels)
    axes_handles(panel_index) = nexttile(layout, panel_index);
    hold(axes_handles(panel_index), 'on');
    for method_index = 1:numel(display_names)
        data = panel_data{panel_index, method_index};
        line_handles(panel_index,method_index) = plot( ...
            axes_handles(panel_index), data.time_h, data.error_m, ...
            'Color', colors(method_index,:), 'LineWidth', 1.0, ...
            'DisplayName', display_names(method_index));
    end
    grid(axes_handles(panel_index), 'on');
    box(axes_handles(panel_index), 'on');
    xlabel(axes_handles(panel_index), '时间 (h)');
    title(axes_handles(panel_index), panels(panel_index).title, ...
        'Interpreter', 'none');
    xlim(axes_handles(panel_index), [0 shared_upper_time_h]);
    ylim(axes_handles(panel_index), [0 shared_upper_limit_m]);
end
ylabel(axes_handles(1), '水平径向误差 (m)');
legend(axes_handles(1), line_handles(1,:), cellstr(display_names), ...
    'Location', 'northwest', 'Box', 'on');
% title(layout, string(options.FigureTitle), 'Interpreter', 'none');

set(axes_handles, 'FontName', 'TimesSimSun', 'FontSize', 10);
set(findall(figure_handle, '-property', 'FontName'), ...
    'FontName', 'TimesSimSun');

output_stem = string(options.OutputStem);
output_paths = struct( ...
    'png', fullfile(artifact_root, output_stem + ".png"), ...
    'fig', fullfile(artifact_root, output_stem + ".fig"));
savefig(figure_handle, output_paths.fig);
exportgraphics(figure_handle, output_paths.png, 'Resolution', 300);

if options.CloseFigure
    close(figure_handle);
else
    set(figure_handle, 'Visible', 'on');
end

fprintf('1x2 comparison figure saved:\n%s\n%s\n', ...
    output_paths.png, output_paths.fig);
end

function axes_handle = select_main_axes(figure_handle)
axes_list = findobj(figure_handle, 'Type', 'axes');
if isempty(axes_list)
    error('源FIG中未找到坐标轴。');
end
line_counts = arrayfun(@(item) ...
    numel(findobj(item, 'Type', 'line')), axes_list);
[~, selected] = max(line_counts);
axes_handle = axes_list(selected);
end

function matched_line = find_named_line(lines, candidate_names)
matched_line = gobjects(0);
for line_index = 1:numel(lines)
    display_name = string(get(lines(line_index), 'DisplayName'));
    if ismember(display_name, candidate_names)
        matched_line(end+1,1) = lines(line_index); %#ok<AGROW>
    end
end
if numel(matched_line) ~= 1
    error('无法唯一匹配曲线：%s', strjoin(candidate_names, ' / '));
end
end

function close_valid_figure(figure_handle)
if isgraphics(figure_handle)
    close(figure_handle);
end
end
