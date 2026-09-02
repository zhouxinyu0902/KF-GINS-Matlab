clear;
close all;
clc;

%% 绘制固定4 s测距延迟补偿对比（rad位置误差状态）
% 本脚本只读取已有导航结果，不运行组合导航核心。
% 请先运行 scripts/range-delay-study/run_range_delay_compensation_rad.m。

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(fileparts(script_dir))));
addpath(topic_dir);
paths = setup_inertial_experiment();

result_root = fullfile(paths.experiment_navigation, ...
    'range-delay-compensation-rad');
artifact_root = exploration_artifact_dir(result_root);
context_path = fullfile(artifact_root, ...
    'range-delay-compensation-evaluation-context.mat');
if ~isfile(context_path)
    error(['缺少绘图上下文：%s\n请先运行 range-delay-study/', ...
        'run_range_delay_compensation_rad.m。'], context_path);
end

loaded = load(context_path, 'evaluation_context');
context = loaded.evaluation_context;
validate_context(context);
truth = readmatrix(context.truth_path, 'FileType', 'text');

ekf_nav = read_condition_results(context.condition_result_dirs, ...
    'range-ins-forward.nav');
double_rts_nav = read_condition_results(context.condition_result_dirs, ...
    'range-ins-rts-double.nav');
rotation_nav = read_condition_results(context.condition_result_dirs, ...
    'range-ins-rts-double-bridge-rotation.nav');
validate_navigation_time_axes(ekf_nav, double_rts_nav, rotation_nav);

condition_names = string(context.condition_names);
plot_algorithm_comparison(truth, ekf_nav, context.ekf_mask, ...
    condition_names, '前向EKF', 'ekf', context.delay_s, artifact_root);
plot_algorithm_comparison(truth, double_rts_nav, context.double_rts_mask, ...
    condition_names, '二次RTS', 'double-rts', context.delay_s, ...
    artifact_root);
plot_algorithm_comparison(truth, rotation_nav, context.rotation_mask, ...
    condition_names, '2RTS+旋转收缩', 'double-rts-rotation', ...
    context.delay_s, artifact_root);

fprintf('延迟补偿对比图片已保存：%s\n', artifact_root);

%% 局部函数
function validate_context(context)
%VALIDATE_CONTEXT 检查实验脚本保存的绘图上下文。
    required_fields = {'delay_s', 'truth_path', 'condition_names', ...
        'condition_result_dirs', 'ekf_mask', 'double_rts_mask', ...
        'rotation_mask'};
    for index = 1:numel(required_fields)
        if ~isfield(context, required_fields{index})
            error('绘图上下文缺少字段：%s', required_fields{index});
        end
    end
    if numel(context.condition_result_dirs) ~= 3 || ...
            numel(context.condition_names) ~= 3
        error('绘图上下文应包含3种延迟工况。');
    end
    if ~isfile(context.truth_path)
        error('真值文件不存在：%s', context.truth_path);
    end
end

function nav_results = read_condition_results(result_dirs, file_name)
%READ_CONDITION_RESULTS 读取三个工况下同一种算法的导航结果。
    nav_results = cell(3, 1);
    for index = 1:3
        file_path = fullfile(result_dirs{index}, file_name);
        if ~isfile(file_path)
            error('导航结果不存在：%s', file_path);
        end
        nav_results{index} = readmatrix(file_path, 'FileType', 'text');
    end
end

function validate_navigation_time_axes(ekf_nav, double_rts_nav, rotation_nav)
%VALIDATE_NAVIGATION_TIME_AXES 保证九份导航结果可以逐历元比较。
    reference_time = ekf_nav{1}(:, 2);
    groups = {ekf_nav, double_rts_nav, rotation_nav};
    for group_index = 1:numel(groups)
        for condition_index = 1:3
            nav = groups{group_index}{condition_index};
            if size(nav, 1) ~= numel(reference_time) || ...
                    any(abs(nav(:, 2)-reference_time) > 1e-8)
                error('导航结果长度或时间轴不一致。');
            end
        end
    end
end

function plot_algorithm_comparison(truth, nav_results, evaluation_mask, ...
        condition_names, algorithm_name, file_prefix, delay_s, output_dir)
%PLOT_ALGORITHM_COMPARISON 绘制一种算法的轨迹和水平径向误差。
    time = nav_results{1}(:, 2);
    evaluation_mask = logical(evaluation_mask(:));
    if numel(evaluation_mask) ~= numel(time) || ~any(evaluation_mask)
        error('%s的评价掩码无效。', algorithm_name);
    end

    truth_position = interp1(truth(:, 2), truth(:, 3:5), time, ...
        'linear', 'extrap');
    radial_error = nan(numel(time), 3);
    for index = 1:3
        radial_error(:, index) = horizontal_radial_error( ...
            nav_results{index}(:, 3:5), truth_position);
    end

    indices = find(evaluation_mask);
    plot_indices = indices(unique(round(linspace(1, numel(indices), ...
        min(20000, numel(indices))))));
    origin = truth_position(plot_indices(1), :);
    [truth_east, truth_north] = position_to_local_plane( ...
        truth_position(plot_indices, :), origin);
    elapsed_time = time-time(1);
    colors = [0.10, 0.35, 0.78; 0.82, 0.25, 0.18; 0.05, 0.58, 0.42];
    styles = {'-', '--', '-.'};

    figure_handle = myfigurestartup(10, 5, 'prese');
    layout = tiledlayout(figure_handle, 1, 2, ...
        'TileSpacing', 'compact', 'Padding', 'compact');
    title(layout, sprintf('rad链%s：固定%.0f s测距延迟补偿', ...
        algorithm_name, delay_s));

    nexttile;
    plot(truth_east/1000, truth_north/1000, 'k-', ...
        'LineWidth', 1.7, 'DisplayName', '真值');
    hold on;
    for index = 1:3
        [east, north] = position_to_local_plane( ...
            nav_results{index}(plot_indices, 3:5), origin);
        plot(east/1000, north/1000, 'Color', colors(index, :), ...
            'LineStyle', styles{index}, 'LineWidth', 1.25, ...
            'DisplayName', condition_names(index));
    end
    grid on;
    box on;
    axis equal;
    xlabel('东向位置（km）');
    ylabel('北向位置（km）');
    title('严格共同评价区间的轨迹');
    legend('Location', 'best', 'Interpreter', 'none');

    nexttile;
    hold on;
    for index = 1:3
        plot(elapsed_time(plot_indices), ...
            radial_error(plot_indices, index), ...
            'Color', colors(index, :), 'LineStyle', styles{index}, ...
            'LineWidth', 1.15, 'DisplayName', condition_names(index));
    end
    grid on;
    box on;
    xlabel('相对导航起点的时间（s）');
    ylabel('水平径向误差（m）');
    title('水平径向误差');
    xlim([elapsed_time(indices(1)), elapsed_time(indices(end))]);
    legend('Location', 'best', 'Interpreter', 'none');

    set(findall(figure_handle, '-property', 'FontName'), ...
        'FontName', 'TimesSimSun');
    exportgraphics(figure_handle, fullfile(output_dir, sprintf( ...
        '%s-delay-compensation-comparison.png', file_prefix)), ...
        'Resolution', 600);
    savefig(figure_handle, fullfile(output_dir, sprintf( ...
        '%s-delay-compensation-comparison.fig', file_prefix)));
end

function radial_error = horizontal_radial_error(estimate, truth)
%HORIZONTAL_RADIAL_ERROR 按WGS-84曲率半径计算水平径向误差。
    latitude = deg2rad(truth(:, 1));
    height = truth(:, 3);
    [rm, rn] = wgs84_radii(latitude);
    north_error = deg2rad(estimate(:, 1)-truth(:, 1)).*(rm+height);
    east_error = deg2rad(estimate(:, 2)-truth(:, 2)).* ...
        (rn+height).*cos(latitude);
    radial_error = hypot(north_error, east_error);
end

function [east, north] = position_to_local_plane(position, origin)
%POSITION_TO_LOCAL_PLANE 将经纬度转换为局部平面坐标。
    latitude = deg2rad(origin(1));
    [rm, rn] = wgs84_radii(latitude);
    north = deg2rad(position(:, 1)-origin(1))*(rm+origin(3));
    east = deg2rad(position(:, 2)-origin(2))* ...
        (rn+origin(3))*cos(latitude);
end

function [rm, rn] = wgs84_radii(latitude)
%WGS84_RADII 计算WGS-84曲率半径。
    semi_major_axis = 6378137.0;
    flattening = 1/298.257223563;
    eccentricity_squared = flattening*(2-flattening);
    denominator = sqrt(1-eccentricity_squared.*sin(latitude).^2);
    rn = semi_major_axis./denominator;
    rm = semi_major_axis*(1-eccentricity_squared)./denominator.^3;
end
