function outputs = evaluate_range_azimuth_aiding(case_name)
%EVALUATE_RANGE_AZIMUTH_AIDING 对比距离辅助和距离+方位角辅助的有效性。
%   分别比较前向 EKF 和二次 RTS，输出水平径向误差统计、误差曲线和轨迹图。

    if nargin < 1 || isempty(case_name)
        case_name = 'case-00';
    end
    case_name = char(string(case_name));

    script_dir = fileparts(mfilename('fullpath'));
    topic_dir = fileparts(fileparts(script_dir));
    addpath(topic_dir);
    paths = setup_inertial_experiment();

    navigation_root = fullfile(paths.simulation_navigation, case_name, ...
        'range-azimuth-aided');
    artifact_dir = fullfile(paths.simulation_artifacts, case_name, ...
        'range-azimuth-aided');
    if ~isfolder(artifact_dir)
        mkdir(artifact_dir);
    end

    result_paths = { ...
        fullfile(navigation_root, 'range-only', 'range-ins-forward.nav'), ...
        fullfile(navigation_root, 'range-azimuth', 'range-ins-forward.nav'), ...
        fullfile(navigation_root, 'range-only', 'range-ins-rts-double.nav'), ...
        fullfile(navigation_root, 'range-azimuth', 'range-ins-rts-double.nav')};
    method_names = ["前向EKF-距离", "前向EKF-距离+方位角", ...
        "二次RTS-距离", "二次RTS-距离+方位角"];
    missing_mask = ~cellfun(@isfile, result_paths);
    if any(missing_mask)
        error('导航结果不完整，请先运行主脚本。缺失：%s', ...
            strjoin(result_paths(missing_mask), ', '));
    end

    nav_results = cellfun(@(path) readmatrix(path, 'FileType', 'text'), ...
        result_paths, 'UniformOutput', false);
    row_counts = cellfun(@(data) size(data, 1), nav_results);
    if numel(unique(row_counts)) ~= 1
        error('四组导航结果长度不一致。');
    end
    time = nav_results{1}(:, 2);
    for result_index = 2:numel(nav_results)
        if any(abs(nav_results{result_index}(:, 2) - time) > 1e-6)
            error('第 %d 组导航结果与基准时间轴不一致。', result_index);
        end
    end

    truth_path = fullfile(paths.simulation_input, case_name, 'truth.txt');
    truth_all = readmatrix(truth_path, 'FileType', 'text');
    truth_position = [ ...
        interp1(truth_all(:, 2), truth_all(:, 3), time, 'linear'), ...
        interp1(truth_all(:, 2), truth_all(:, 4), time, 'linear'), ...
        interp1(truth_all(:, 2), truth_all(:, 5), time, 'linear')];

    radial_error = nan(numel(time), numel(nav_results));
    for result_index = 1:numel(nav_results)
        radial_error(:, result_index) = horizontal_radial_error( ...
            nav_results{result_index}(:, 3:5), truth_position);
    end

    % 最后一个 420 s 区间只有一次 RTS，统一截到最后一个完整二次 RTS 终点。
    range_interval_s = 420;
    complete_double_rts_end = floor((max(time) - range_interval_s) / ...
        range_interval_s) * range_interval_s;
    evaluation_mask = time <= complete_double_rts_end & ...
        all(isfinite(truth_position), 2) & all(isfinite(radial_error), 2);
    if ~any(evaluation_mask)
        error('没有可用于评价的完整二次 RTS 区间。');
    end

    statistics = table('Size', [numel(method_names), 7], ...
        'VariableTypes', {'string', 'double', 'double', 'double', ...
        'double', 'double', 'double'}, ...
        'VariableNames', {'Method', 'StartTime_s', 'EndTime_s', ...
        'RMSE_m', 'Mean_m', 'Median_m', 'Max_m'});
    for method_index = 1:numel(method_names)
        values = radial_error(evaluation_mask, method_index);
        statistics.Method(method_index) = method_names(method_index);
        statistics.StartTime_s(method_index) = time(find(evaluation_mask, 1));
        statistics.EndTime_s(method_index) = time(find(evaluation_mask, 1, 'last'));
        statistics.RMSE_m(method_index) = sqrt(mean(values .^ 2));
        statistics.Mean_m(method_index) = mean(values);
        statistics.Median_m(method_index) = median(values);
        statistics.Max_m(method_index) = max(values);
    end
    statistics_path = fullfile(artifact_dir, ...
        'range-azimuth-aiding-statistics.csv');
    writetable(statistics, statistics_path);

    error_figure = create_figure(10, 5);
    plot(time(evaluation_mask), radial_error(evaluation_mask, 1), ...
        'Color', [0.00, 0.45, 0.74], 'LineWidth', 1.0);
    hold on;
    plot(time(evaluation_mask), radial_error(evaluation_mask, 2), '--', ...
        'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.2);
    plot(time(evaluation_mask), radial_error(evaluation_mask, 3), ...
        'Color', [0.47, 0.67, 0.19], 'LineWidth', 1.0);
    plot(time(evaluation_mask), radial_error(evaluation_mask, 4), '--', ...
        'Color', [0.49, 0.18, 0.56], 'LineWidth', 1.2);
    grid on;
    xlim([time(find(evaluation_mask, 1)), ...
        time(find(evaluation_mask, 1, 'last'))]);
    xlabel('时间（s）');
    ylabel('水平径向误差（m）');
    title(sprintf('%s：距离与距离+方位角辅助误差对比', case_name));
    legend(cellstr(method_names), 'Location', 'best');
    error_png = fullfile(artifact_dir, 'horizontal-radial-error.png');
    error_fig = fullfile(artifact_dir, 'horizontal-radial-error.fig');
    exportgraphics(error_figure, error_png, 'Resolution', 300);
    savefig(error_figure, error_fig);

    trajectory_figure = create_figure(10, 5);
    tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    plot_trajectory_panel(nexttile, truth_position, nav_results(1:2), ...
        evaluation_mask, {'距离', '距离+方位角'}, '前向 EKF');
    plot_trajectory_panel(nexttile, truth_position, nav_results(3:4), ...
        evaluation_mask, {'距离', '距离+方位角'}, '二次 RTS');
    trajectory_png = fullfile(artifact_dir, 'trajectory-comparison.png');
    trajectory_fig = fullfile(artifact_dir, 'trajectory-comparison.fig');
    exportgraphics(trajectory_figure, trajectory_png, 'Resolution', 300);
    savefig(trajectory_figure, trajectory_fig);

    outputs = struct('case_name', case_name, 'statistics', statistics, ...
        'statistics_path', statistics_path, 'error_png', error_png, ...
        'trajectory_png', trajectory_png, ...
        'evaluation_end_time_s', complete_double_rts_end);
    disp(statistics);
    fprintf('距离+方位角评价结果：%s\n', artifact_dir);
end

function radial_error = horizontal_radial_error(estimate, truth)
    latitude_rad = deg2rad(truth(:, 1));
    [rm, rn] = wgs84_radii(latitude_rad);
    north_error = deg2rad(estimate(:, 1) - truth(:, 1)) .* ...
        (rm + truth(:, 3));
    east_error = deg2rad(estimate(:, 2) - truth(:, 2)) .* ...
        (rn + truth(:, 3)) .* cos(latitude_rad);
    radial_error = hypot(north_error, east_error);
end

function plot_trajectory_panel(axis_handle, truth, nav_results, mask, ...
        method_names, panel_title)
    reference = truth(find(mask, 1), :);
    truth_en = geodetic_to_local_en(truth(mask, :), reference);
    first_en = geodetic_to_local_en(nav_results{1}(mask, 3:5), reference);
    second_en = geodetic_to_local_en(nav_results{2}(mask, 3:5), reference);
    plot(axis_handle, truth_en(:, 1), truth_en(:, 2), 'k-', ...
        'LineWidth', 1.5);
    hold(axis_handle, 'on');
    plot(axis_handle, first_en(:, 1), first_en(:, 2), ...
        'Color', [0.00, 0.45, 0.74], 'LineWidth', 1.0);
    plot(axis_handle, second_en(:, 1), second_en(:, 2), '--', ...
        'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.2);
    axis(axis_handle, 'equal');
    grid(axis_handle, 'on');
    xlabel(axis_handle, '东向（m）');
    ylabel(axis_handle, '北向（m）');
    title(axis_handle, panel_title);
    legend(axis_handle, [{'真值'}, method_names], 'Location', 'best');
end

function en = geodetic_to_local_en(position_deg, reference_deg)
    reference_lat = deg2rad(reference_deg(1));
    [rm, rn] = wgs84_radii(reference_lat);
    north = deg2rad(position_deg(:, 1) - reference_deg(1)) .* ...
        (rm + reference_deg(3));
    east = deg2rad(position_deg(:, 2) - reference_deg(2)) .* ...
        (rn + reference_deg(3)) .* cos(reference_lat);
    en = [east, north];
end

function [rm, rn] = wgs84_radii(latitude_rad)
    semi_major_axis = 6378137.0;
    eccentricity_squared = 6.6943799901413165e-3;
    denominator = sqrt(1 - eccentricity_squared .* sin(latitude_rad) .^ 2);
    rn = semi_major_axis ./ denominator;
    rm = semi_major_axis * (1 - eccentricity_squared) ./ denominator .^ 3;
end

function figure_handle = create_figure(width_in, height_in)
    if exist('myfigurestartup', 'file') == 2
        figure_handle = myfigurestartup(width_in, height_in, 'prese');
    else
        figure_handle = figure('Color', 'w', 'Units', 'inches', ...
            'Position', [1, 1, width_in, height_in]);
    end
end
