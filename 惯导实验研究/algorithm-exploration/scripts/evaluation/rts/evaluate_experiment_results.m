clear;
close all;
clc;
% 实测结果评价总入口，依次评价 m、rad，最后比较两套单位
%% 实测四方法结果统一评价入口
% 本脚本不运行核心算法，只读取已有 m/rad 导航结果并生成统计和图片。
% 如只想评价其中一条状态链，可修改 state_modes。

state_modes = ["m", "rad"];

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));
addpath(topic_dir);
paths = setup_inertial_experiment();
truth_path = fullfile(paths.experiment_reference, 'truth.nav');

evaluations = struct();
for state_mode = state_modes
    if state_mode == "m"
        case_name = 'experiment-m-state';
        result_dir = fullfile(paths.experiment_navigation, ...
            'four-method-comparison');
    elseif state_mode == "rad"
        case_name = 'experiment-rad-state';
        result_dir = fullfile(paths.experiment_navigation, ...
            'four-method-comparison-rad');
    else
        error('不支持的状态模式：%s', state_mode);
    end

    if ~isfolder(result_dir)
        warning('结果目录不存在，跳过 %s：%s', state_mode, result_dir);
        continue;
    end
    evaluations.(char(state_mode)) = evaluate_experiment_four_methods( ...
        result_dir, truth_path=truth_path, case_name=case_name, ...
        create_figure=true);
    fprintf('\n%s 链四方法统计：\n', state_mode);
    disp(evaluations.(char(state_mode)).statistics);
end

if isfield(evaluations, 'm') && isfield(evaluations, 'rad')
    output_dir = fullfile(paths.experiment_artifacts, ...
        'four-method-comparison-rad');
    unit_comparison = compare_experiment_state_units( ...
        evaluations.m.statistics_path, evaluations.rad.statistics_path, ...
        output_dir, create_figure=true);
    fprintf('\nm 链与 rad 链 RMSE 对比：\n');
    disp(unit_comparison);
end
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
function evaluation = evaluate_experiment_four_methods(result_dir, options)
%EVALUATE_EXPERIMENT_FOUR_METHODS 统一评价实测四方法导航结果。
%   本函数只读取核心算法已经生成的导航文件，不重新运行滤波与平滑。
%   options.truth_path    : truth.nav 路径（必填）
%   options.case_name     : 图表标题中的工况名称
%   options.create_figure : 是否生成 PNG/FIG，默认 true

    arguments
        result_dir (1, :) char
        options.truth_path (1, :) char
        options.case_name (1, :) char = 'experiment'
        options.create_figure (1, 1) logical = true
    end

    method_names = {'前向 EKF', '二次 RTS', '2RTS+旋转收缩', ...
        '2RTS+位置速度约束（固定滞后）'};
    result_paths = { ...
        fullfile(result_dir, 'range-ins-forward.nav'), ...
        fullfile(result_dir, 'range-ins-rts-double.nav'), ...
        fullfile(result_dir, 'range-ins-rts-double-bridge-rotation.nav'), ...
        fullfile(result_dir, ...
        'range-ins-double-rts-position-velocity-fixed-lag-replay.nav')};

    required_files = [{options.truth_path}, result_paths];
    for file_index = 1:numel(required_files)
        if ~isfile(required_files{file_index})
            error('缺少四方法评价文件：%s', required_files{file_index});
        end
    end

    truth = readmatrix(options.truth_path, 'FileType', 'text');
    nav_results = cell(size(result_paths));
    for result_index = 1:numel(result_paths)
        nav_results{result_index} = readmatrix(result_paths{result_index}, ...
            'FileType', 'text');
    end

    [time, effective_mask] = validate_navigation_results(nav_results);
    truth_position = interp1(truth(:, 2), truth(:, 3:5), time, ...
        'linear', 'extrap');
    result_count = numel(nav_results);
    radial_error = nan(numel(time), result_count);
    rmse_m = zeros(result_count, 1);
    mean_m = zeros(result_count, 1);
    median_m = zeros(result_count, 1);
    p95_m = zeros(result_count, 1);
    maximum_m = zeros(result_count, 1);

    for result_index = 1:result_count
        radial_error(:, result_index) = calculate_horizontal_radial_error( ...
            nav_results{result_index}(:, 3:5), truth_position);
        values = radial_error(effective_mask, result_index);
        rmse_m(result_index) = sqrt(mean(values .^ 2));
        mean_m(result_index) = mean(values);
        median_m(result_index) = median(values);
        p95_m(result_index) = prctile(values, 95);
        maximum_m(result_index) = max(values);
    end

    method = string(method_names(:));
    statistics = table(method, rmse_m, mean_m, median_m, p95_m, ...
        maximum_m, 'VariableNames', {'Method', 'RMSE_m', 'Mean_m', ...
        'Median_m', 'P95_m', 'Maximum_m'});

    artifact_dir = exploration_artifact_dir(result_dir);
    if ~isfolder(artifact_dir)
        mkdir(artifact_dir);
    end
    statistics_path = fullfile(artifact_dir, ...
        'fixed-lag-four-method-statistics.csv');
    writetable(statistics, statistics_path);

    figure_path = '';
    figure_source_path = '';
    if options.create_figure
        [figure_path, figure_source_path] = plot_four_method_comparison( ...
            time, truth_position, nav_results, radial_error, ...
            effective_mask, method_names, artifact_dir, options.case_name);
    end

    evaluation = struct( ...
        'statistics', statistics, ...
        'statistics_path', statistics_path, ...
        'figure_path', figure_path, ...
        'figure_source_path', figure_source_path, ...
        'effective_mask', effective_mask, ...
        'effective_time_s', time(effective_mask), ...
        'radial_error_m', radial_error, ...
        'result_paths', {result_paths});
end

function [time, effective_mask] = validate_navigation_results(nav_results)
%VALIDATE_NAVIGATION_RESULTS 检查四种结果的长度、时间轴和有效区间。
    row_count = size(nav_results{1}, 1);
    time = nav_results{1}(:, 2);
    effective_mask = true(row_count, 1);
    for result_index = 1:numel(nav_results)
        nav = nav_results{result_index};
        if size(nav, 1) ~= row_count || size(nav, 2) < 11
            error('四种导航结果的长度或列数不一致。');
        end
        if any(abs(nav(:, 2) - time) > 1e-6)
            error('四种导航结果的时间轴不一致。');
        end
        effective_mask = effective_mask & all(isfinite(nav(:, 2:11)), 2);
    end
    if ~any(effective_mask)
        error('未找到四种方法共同有效的固定滞后区间。');
    end
end

function [figure_path, figure_source_path] = plot_four_method_comparison( ...
        time, truth_position, nav_results, radial_error, effective_mask, ...
        method_names, artifact_dir, case_name)
%PLOT_FOUR_METHOD_COMPARISON 绘制轨迹与水平径向误差双图。
    elapsed_time = time - time(1);
    display_indices = find(effective_mask);
    display_indices = display_indices(unique(round(linspace(1, ...
        numel(display_indices), min(20000, numel(display_indices))))));
    origin = truth_position(display_indices(1), :);
    [truth_east, truth_north] = position_to_local_plane( ...
        truth_position(display_indices, :), origin);
    colors = [0.10, 0.35, 0.75; 0.05, 0.55, 0.55; ...
        0.78, 0.16, 0.52; 0.20, 0.65, 0.35];
    line_styles = {'-', '--', ':', '-.'};

    comparison_figure = myfigurestartup(10, 5, 'prese');
    layout = tiledlayout(comparison_figure, 1, 2, ...
        'TileSpacing', 'compact', 'Padding', 'compact');
    title(layout, sprintf('%s：事件驱动固定滞后四方法对比', ...
        case_name), 'FontSize', 15, 'FontWeight', 'bold');

    nexttile(layout, 1);
    plot(truth_east / 1000, truth_north / 1000, 'k-', ...
        'LineWidth', 1.7, 'DisplayName', '真值');
    hold on;
    for result_index = 1:numel(nav_results)
        [east, north] = position_to_local_plane( ...
            nav_results{result_index}(display_indices, 3:5), origin);
        plot(east / 1000, north / 1000, ...
            'Color', colors(result_index, :), ...
            'LineStyle', line_styles{result_index}, ...
            'LineWidth', 1.25, ...
            'DisplayName', method_names{result_index});
    end
    grid on; box on; axis tight;
    xlabel('东向位置（km）');
    ylabel('北向位置（km）');
    title('固定滞后有效区间轨迹');
    legend('Location', 'best', 'Interpreter', 'none');

    nexttile(layout, 2);
    hold on;
    for result_index = 1:numel(nav_results)
        plot(elapsed_time(display_indices), ...
            radial_error(display_indices, result_index), ...
            'Color', colors(result_index, :), ...
            'LineStyle', line_styles{result_index}, ...
            'LineWidth', 1.2, ...
            'DisplayName', method_names{result_index});
    end
    grid on; box on; axis tight;
    xlabel('时间（s）');
    ylabel('水平径向误差（m）');
    title('固定滞后有效区间水平径向误差');
    legend('Location', 'best', 'Interpreter', 'none');
    xlim([0, elapsed_time(end)]);

    set(findall(comparison_figure, '-property', 'FontName'), ...
        'FontName', 'TimesSimSun');
    figure_path = fullfile(artifact_dir, ...
        'fixed-lag-four-method-comparison.png');
    figure_source_path = fullfile(artifact_dir, ...
        'fixed-lag-four-method-comparison.fig');
    exportgraphics(comparison_figure, figure_path, 'Resolution', 600);
    savefig(comparison_figure, figure_source_path);
end

function radial_error = calculate_horizontal_radial_error( ...
        estimated_position, truth_position)
%CALCULATE_HORIZONTAL_RADIAL_ERROR 按 WGS-84 曲率半径计算水平误差。
    latitude = deg2rad(truth_position(:, 1));
    height = truth_position(:, 3);
    [meridian_radius, prime_vertical_radius] = wgs84_radii(latitude);
    north_error = deg2rad(estimated_position(:, 1) - ...
        truth_position(:, 1)) .* (meridian_radius + height);
    east_error = deg2rad(estimated_position(:, 2) - ...
        truth_position(:, 2)) .* (prime_vertical_radius + height) ...
        .* cos(latitude);
    radial_error = hypot(north_error, east_error);
end

function [east, north] = position_to_local_plane(position, origin)
%POSITION_TO_LOCAL_PLANE 将经纬度转换到以 origin 为原点的局部平面。
    latitude = deg2rad(origin(1));
    [meridian_radius, prime_vertical_radius] = wgs84_radii(latitude);
    north = deg2rad(position(:, 1) - origin(1)) ...
        * (meridian_radius + origin(3));
    east = deg2rad(position(:, 2) - origin(2)) ...
        * (prime_vertical_radius + origin(3)) * cos(latitude);
end

function [meridian_radius, prime_vertical_radius] = wgs84_radii(latitude)
%WGS84_RADII 计算 WGS-84 子午圈和卯酉圈曲率半径。
    semi_major_axis = 6378137.0;
    flattening = 1 / 298.257223563;
    eccentricity_squared = flattening * (2 - flattening);
    denominator = sqrt(1 - eccentricity_squared .* sin(latitude) .^ 2);
    prime_vertical_radius = semi_major_axis ./ denominator;
    meridian_radius = semi_major_axis * (1 - eccentricity_squared) ...
        ./ denominator .^ 3;
end
