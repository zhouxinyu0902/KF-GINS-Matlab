clear; clc; close all;
%RANGE_DATAGET 为第一次车载试验的 8 组轨迹生成三信标测距数据。
%
% 处理流程：
%   1. 读取各 case/input/truth.nav；
%   2. 调用公共 rangedataget 函数生成 range1.txt～range3.txt；
%   3. 绘制 2×4 的轨迹与信标分布对比图。
%
% align_modes 可逐组改为 "right" 或 "bottom"。当前 8 组均采用默认
% "right" 模式，后续只需修改下面这一行，无需改动后续处理代码。

%% ===================== 用户配置 =====================
case_ids = 1:8;
align_modes = repmat("bottom", size(case_ids));
align_modes(1,[4,6])=repmat("right",[1,2]);
show_figure = true;
figure_resolution_dpi = 300;

if numel(align_modes) ~= numel(case_ids)
    error('align_modes 的数量必须与 case_ids 一致。');
end

%% ===================== 路径与依赖 =====================
script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);
paths = setup_experiment_01();

if exist('rangedataget', 'file') ~= 2
    error('没有找到公共函数 rangedataget.m，请检查 function_zxy 路径。');
end

case_plot_data = repmat(struct( ...
    'case_id', [], ...
    'align_mode', "", ...
    'trajectory_en_m', [], ...
    'beacon_en_m', []), numel(case_ids), 1);

%% ===================== 批量生成距离数据 =====================
for case_index = 1:numel(case_ids)
    case_id = case_ids(case_index);
    align_mode = lower(string(align_modes(case_index)));

    if ~ismember(align_mode, ["right", "bottom"])
        error('case-%02d 的模式无效：%s。只能使用 right 或 bottom。', ...
            case_id, align_mode);
    end

    input_dir = paths.case_input(case_id);
    truth_path = fullfile(input_dir, 'truth.nav');
    if ~isfile(truth_path)
        error('case-%02d 缺少 truth.nav：%s', case_id, truth_path);
    end

    fprintf('\n[%d/%d] 正在处理 case-%02d，信标模式：%s\n', ...
        case_index, numel(case_ids), case_id, align_mode);

    % 公共函数直接将 range1.txt～range3.txt 写入当前 case 的 input。
    % 此处关闭公共函数的单组绘图，由本脚本统一生成 2×4 对比图。
    rangedataget(input_dir, char(align_mode), 0, 0, 0, 0);

    verify_range_files(input_dir, case_id);
    [trajectory_en_m, beacon_en_m] = load_case_plot_data(input_dir);

    case_plot_data(case_index).case_id = case_id;
    case_plot_data(case_index).align_mode = align_mode;
    case_plot_data(case_index).trajectory_en_m = trajectory_en_m;
    case_plot_data(case_index).beacon_en_m = beacon_en_m;
end

%% ===================== 2×4 轨迹—信标对比图 =====================
if show_figure
    figure_visibility = 'on';
else
    figure_visibility = 'off';
end

fig = figure( ...
    'Name', 'Experiment-01 trajectory and beacon comparison', ...
    'Color', 'w', ...
    'Visible', figure_visibility, ...
    'Position', [80, 80, 1600, 820]);

layout = tiledlayout(fig, 2, 4, ...
    'TileSpacing', 'compact', ...
    'Padding', 'compact');

trajectory_color = [0.0000, 0.4470, 0.7410];
beacon_colors = lines(3);

for case_index = 1:numel(case_plot_data)
    data = case_plot_data(case_index);
    ax = nexttile(layout);
    hold(ax, 'on');
    grid(ax, 'on');
    box(ax, 'on');

    trajectory_km = data.trajectory_en_m(:, 1:2) / 1000;
    beacon_km = data.beacon_en_m(:, 1:2) / 1000;

    plot(ax, trajectory_km(:, 1), trajectory_km(:, 2), '-', ...
        'Color', trajectory_color, ...
        'LineWidth', 1.35, ...
        'DisplayName', '车辆轨迹');
    scatter(ax, trajectory_km(1, 1), trajectory_km(1, 2), ...
        38, '^', 'filled', ...
        'MarkerFaceColor', [0.15, 0.15, 0.15], ...
        'DisplayName', '起点');

    for beacon_index = 1:size(beacon_km, 1)
        scatter(ax, beacon_km(beacon_index, 1), ...
            beacon_km(beacon_index, 2), 58, 'p', 'filled', ...
            'MarkerFaceColor', beacon_colors(beacon_index, :), ...
            'MarkerEdgeColor', [0.1, 0.1, 0.1], ...
            'DisplayName', sprintf('信标 B%d', beacon_index));
        text(ax, beacon_km(beacon_index, 1), ...
            beacon_km(beacon_index, 2), sprintf('  B%d', beacon_index), ...
            'FontSize', 8, ...
            'FontWeight', 'bold', ...
            'VerticalAlignment', 'bottom');
    end

    axis(ax, 'equal');
    xlabel(ax, '东向 / km');
    ylabel(ax, '北向 / km');
    title(ax, sprintf('case-%02d（%s）', ...
        data.case_id, data.align_mode), 'FontWeight', 'normal');

    if case_index == 1
        legend(ax, 'Location', 'best', 'FontSize', 8);
    end
end

title(layout, '第一次车载试验：8 组轨迹与三信标分布', ...
    'FontWeight', 'bold');

output_png = fullfile(paths.summary_artifacts, ...
    'eight-case-trajectory-beacon-comparison.png');
output_fig = fullfile(paths.summary_artifacts, ...
    'eight-case-trajectory-beacon-comparison.fig');

exportgraphics(fig, output_png, 'Resolution', figure_resolution_dpi);
savefig(fig, output_fig);

fprintf('\n8 组测距数据生成完毕。\n');
fprintf('对比图 PNG：%s\n', output_png);
fprintf('对比图 FIG：%s\n', output_fig);

%% ===================== 局部函数 =====================
function verify_range_files(input_dir, case_id)
%VERIFY_RANGE_FILES 检查公共函数生成的三个测距文件。

    for beacon_index = 1:3
        range_path = fullfile(input_dir, ...
            sprintf('range%d.txt', beacon_index));
        if ~isfile(range_path)
            error('case-%02d 未生成 range%d.txt：%s', ...
                case_id, beacon_index, range_path);
        end

        range_data = readmatrix(range_path, 'FileType', 'text');
        if size(range_data, 2) < 6 || isempty(range_data)
            error('case-%02d 的 range%d.txt 格式无效，应至少包含 6 列。', ...
                case_id, beacon_index);
        end
        if any(~isfinite(range_data(:, 1:6)), 'all')
            error('case-%02d 的 range%d.txt 含有非有限数值。', ...
                case_id, beacon_index);
        end
    end
end

function [trajectory_en_m, beacon_en_m] = load_case_plot_data(input_dir)
%LOAD_CASE_PLOT_DATA 将真值轨迹和信标统一转换到局部东—北坐标系。

    truth = readmatrix(fullfile(input_dir, 'truth.nav'), ...
        'FileType', 'text');
    if size(truth, 2) < 5
        error('truth.nav 至少需要 5 列：[周, 时间, 纬度, 经度, 高度]。');
    end

    valid_truth = all(isfinite(truth(:, 2:5)), 2);
    truth = truth(valid_truth, :);
    if isempty(truth)
        error('truth.nav 中没有有效轨迹。');
    end

    % 与 rangedataget 保持一致，从 100 Hz 真值中每 100 行取一点。
    sample_index = 1:100:size(truth, 1);
    trajectory_lla = [deg2rad(truth(sample_index, 3:4)), ...
        truth(sample_index, 5)];

    beacon_lla = zeros(3, 3);
    for beacon_index = 1:3
        range_data = readmatrix(fullfile(input_dir, ...
            sprintf('range%d.txt', beacon_index)), 'FileType', 'text');
        beacon_lla(beacon_index, :) = range_data(1, 4:6);
    end

    % 绘图只关注水平分布，高度原点统一取 0 m。
    local_origin = [trajectory_lla(1, 1:2), 0]';
    trajectory_en_m = pos2dxyz(trajectory_lla, local_origin);
    beacon_en_m = pos2dxyz(beacon_lla, local_origin);
end
