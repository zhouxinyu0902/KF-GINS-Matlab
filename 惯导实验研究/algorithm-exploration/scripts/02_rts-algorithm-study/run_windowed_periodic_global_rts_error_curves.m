clear;
close all;
clc;
%% case-06～09：2RTS、56 min窗口RTS与前向EKF水平误差曲线

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
paths = setup_inertial_experiment();
case_names = ["case-06"; "case-07"; "case-08"; "case-09"];
window_folder = 'windowed-periodic-global-rts-56min-every-42min-m';
figure_dir = fullfile(paths.experiment, 'summary', ...
    'windowed-periodic-global-rts-56min-every42min');
if ~exist(figure_dir, 'dir')
    mkdir(figure_dir);
end

fig = figure('Color', 'w', 'Visible', 'off', ...
    'Position', [80, 80, 1500, 900]);
layout = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', ...
    'Padding', 'compact');
title(layout, '各工况水平位置误差：2RTS、56 min窗口RTS与前向EKF');

for case_index = 1:numel(case_names)
    case_id = str2double(extractAfter(case_names(case_index), 'case-'));
    cfg = load_algorithm_exploration_config( ...
        "experiment", "m", paths.experiment_input(case_id));
    result_dir = fullfile(cfg.outputfolder, window_folder);
    window_nav = readmatrix(fullfile(result_dir, ...
        'range-ins-rts-standard-global.nav'), 'FileType', 'text');
    hybrid_nav = readmatrix(fullfile(result_dir, ...
        'range-ins-2rts-windowed-global-hybrid.nav'), 'FileType', 'text');
    forward_nav = readmatrix(fullfile(result_dir, ...
        'range-ins-forward.nav'), 'FileType', 'text');
    double_nav = readmatrix(fullfile(cfg.outputfolder, ...
        'simple-ekf-rts-rad', 'simple-rts-double-rad.nav'), ...
        'FileType', 'text');
    truth = readmatrix(cfg.truthpath, 'FileType', 'text');

    double_aligned = align_nav_to_time(double_nav, hybrid_nav(:, 2));
    forward_aligned = align_nav_to_time(forward_nav, hybrid_nav(:, 2));
    double_error = horizontal_error_m(double_aligned, truth);
    forward_error = horizontal_error_m(forward_aligned, truth);
    hybrid_error = horizontal_error_m(hybrid_nav, truth);
    window_error = horizontal_error_m(window_nav, truth);
    elapsed_min = (hybrid_nav(:, 2) - hybrid_nav(1, 2)) / 60;
    window_elapsed_min = (window_nav(:, 2) - hybrid_nav(1, 2)) / 60;

    nexttile;
    plot(elapsed_min, double_error, '--', 'LineWidth', 1.0, ...
        'DisplayName', '2RTS基线');
    hold on;
    plot(elapsed_min, forward_error, ':', 'LineWidth', 1.0, ...
        'DisplayName', '前向EKF');
    plot(elapsed_min, hybrid_error, '-', 'LineWidth', 1.2, ...
        'DisplayName', '窗口RTS+尾段2RTS');
    plot(window_elapsed_min, window_error, '-', 'LineWidth', 1.2, ...
        'DisplayName', '窗口RTS已覆盖区间');
    grid on;
    xlabel('相对时间 / min');
    ylabel('水平误差 / m');
    title(char(case_names(case_index)));
    legend('Location', 'northwest');
    xlim([0, max(elapsed_min)]);

    diagnostics = readtable(fullfile(cfg.outputfolder, '..', ...
        'figures-tables', window_folder, ...
        'range-rts-standard-global-events.csv'));
    trigger_times = (diagnostics.time_s - hybrid_nav(1, 2)) / 60;
    for trigger_index = 1:numel(trigger_times)
        xline(trigger_times(trigger_index), 'k:', ...
            'HandleVisibility', 'off');
    end
end

figure_path = fullfile(figure_dir, ...
    'windowed-periodic-global-rts-error-curves-by-case.png');
exportgraphics(fig, figure_path, 'Resolution', 220);
close(fig);
fprintf('四工况误差曲线：%s\n', figure_path);

function aligned = align_nav_to_time(nav, target_time)
    [unique_time, unique_indices] = unique(nav(:, 2), 'stable');
    unique_nav = nav(unique_indices, :);
    aligned = nan(numel(target_time), size(nav, 2));
    aligned(:, 2) = target_time;
    aligned(:, 1) = interp1(unique_time, unique_nav(:, 1), ...
        target_time, 'nearest', nan);
    % 个别工况的IMU末端比历史2RTS结果多约2 s；仅对绘图轴做线性外推，
    % 不参与前面的数值精度统计。
    aligned(:, 3:end) = interp1(unique_time, unique_nav(:, 3:end), ...
        target_time, 'linear', 'extrap');
end

function horizontal_error = horizontal_error_m(nav, truth)
    param = Param();
    truth_position = interp1(truth(:, 2), truth(:, 3:5), ...
        nav(:, 2), 'linear', nan);
    if any(~isfinite(truth_position), 'all')
        error('误差评价区间内存在无法插值的真值。');
    end
    latitude = mean(truth_position(:, 1)) * param.D2R;
    height = mean(truth_position(:, 3));
    [rm, rn] = getRmRn(latitude, param);
    north = (nav(:, 3) - truth_position(:, 1)) * ...
        param.D2R * (rm + height);
    east = (nav(:, 4) - truth_position(:, 2)) * ...
        param.D2R * (rn + height) * cos(latitude);
    horizontal_error = hypot(north, east);
end
