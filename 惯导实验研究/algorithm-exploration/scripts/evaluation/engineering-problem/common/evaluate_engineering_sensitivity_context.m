function statistics = evaluate_engineering_sensitivity_context(context_path)
%EVALUATE_ENGINEERING_SENSITIVITY_CONTEXT 统一评价工程误差敏感性工况。
%   每种算法在所有工况共同有效的严格区间内评价，输出水平径向误差
%   统计、轨迹/误差图和“误差参数—RMSE”响应曲线。

if nargin < 1 || ~isfile(context_path)
    error('找不到专题评价上下文：%s', string(context_path));
end
loaded = load(context_path, 'study_context');
if ~isfield(loaded, 'study_context')
    error('上下文文件中缺少 study_context：%s', context_path);
end
context = loaded.study_context;
validate_context(context);
% 兼容 compose 在不同 MATLAB 版本中返回 cellstr 或 string 的差异。
context.study_id = string(context.study_id);
context.study_title = string(context.study_title);
context.scenario_ids = string(context.scenario_ids(:));
context.scenario_names = string(context.scenario_names(:));
context.output_dirs = string(context.output_dirs(:));
context.method_ids = string(context.method_ids(:));
context.method_names = string(context.method_names(:));
context.method_files = string(context.method_files(:));
context.parameter_name = string(context.parameter_name);
context.parameter_label = string(context.parameter_label);

if ~isfolder(context.artifact_root)
    mkdir(context.artifact_root);
end
truth = readmatrix(context.truth_path, 'FileType', 'text');
scenario_count = numel(context.scenario_ids);
method_count = numel(context.method_ids);
navigation = cell(scenario_count, method_count);

reference_time = [];
for scenario_index = 1:scenario_count
    for method_index = 1:method_count
        result_path = fullfile(context.output_dirs(scenario_index), ...
            context.method_files(method_index));
        if ~isfile(result_path)
            error('缺少导航结果：%s', result_path);
        end
        navigation{scenario_index, method_index} = readmatrix( ...
            result_path, 'FileType', 'text');
        time = navigation{scenario_index, method_index}(:, 2);
        if isempty(reference_time)
            reference_time = time;
        elseif numel(time) ~= numel(reference_time) || ...
                any(abs(time-reference_time) > 1e-8)
            error('各工况或算法的导航结果时间轴不一致。');
        end
    end
end

truth_position = interp1(truth(:, 2), truth(:, 3:5), reference_time, ...
    'linear', 'extrap');
truth_time_mask = reference_time >= truth(1, 2) & ...
    reference_time <= truth(end, 2);

row_count = scenario_count*method_count;
method = strings(row_count, 1);
scenario = strings(row_count, 1);
parameter_value = zeros(row_count, 1);
sample_count = zeros(row_count, 1);
start_time_s = zeros(row_count, 1);
end_time_s = zeros(row_count, 1);
rmse_m = zeros(row_count, 1);
mean_m = zeros(row_count, 1);
median_m = zeros(row_count, 1);
p95_m = zeros(row_count, 1);
maximum_m = zeros(row_count, 1);
radial_errors = cell(scenario_count, method_count);

row_index = 0;
for method_index = 1:method_count
    common_mask = truth_time_mask;
    for scenario_index = 1:scenario_count
        nav = navigation{scenario_index, method_index};
        common_mask = common_mask & all(isfinite(nav(:, 2:5)), 2);
        stored_mask = logical(context.valid_masks{scenario_index, method_index}(:));
        if numel(stored_mask) ~= numel(common_mask)
            error('工况 %s 的有效区间掩码长度不一致。', ...
                context.scenario_ids(scenario_index));
        end
        common_mask = common_mask & stored_mask;
    end
    if ~any(common_mask)
        error('%s没有找到所有工况共同有效的评价区间。', ...
            context.method_names(method_index));
    end

    for scenario_index = 1:scenario_count
        row_index = row_index+1;
        nav = navigation{scenario_index, method_index};
        radial_error = horizontal_radial_error(nav(:, 3:5), truth_position);
        radial_errors{scenario_index, method_index} = radial_error;
        values = radial_error(common_mask);
        method(row_index) = context.method_names(method_index);
        scenario(row_index) = context.scenario_names(scenario_index);
        parameter_value(row_index) = context.parameter_values(scenario_index);
        sample_count(row_index) = nnz(common_mask);
        start_time_s(row_index) = reference_time(find(common_mask, 1));
        end_time_s(row_index) = reference_time(find(common_mask, 1, 'last'));
        rmse_m(row_index) = sqrt(mean(values.^2));
        mean_m(row_index) = mean(values);
        median_m(row_index) = median(values);
        p95_m(row_index) = prctile(values, 95);
        maximum_m(row_index) = max(values);
    end

    plot_method_sensitivity(context, truth_position, navigation(:, method_index), ...
        radial_errors(:, method_index), common_mask, method_index, ...
        reference_time);
end

statistics = table(method, scenario, parameter_value, sample_count, ...
    start_time_s, end_time_s, rmse_m, mean_m, median_m, p95_m, maximum_m, ...
    'VariableNames', {'Method', 'Scenario', 'ParameterValue', ...
    'SampleCount', 'StartTime_s', 'EndTime_s', 'RMSE_m', 'Mean_m', ...
    'Median_m', 'P95_m', 'Maximum_m'});
statistics.Properties.VariableNames{'ParameterValue'} = ...
    char(context.parameter_name);
writetable(statistics, fullfile(context.artifact_root, ...
    context.study_id + "-statistics.csv"));
if ~isfield(context, 'plot_response') || context.plot_response
    plot_rmse_response(context, statistics);
end

fprintf('\n%s评价结果：\n', context.study_title);
disp(statistics);
fprintf('图表与统计已保存：%s\n', context.artifact_root);
end

function validate_context(context)
%VALIDATE_CONTEXT 检查运行脚本与评价脚本之间的数据契约。
    required = {'study_id', 'study_title', 'scenario_ids', ...
        'scenario_names', 'parameter_values', 'parameter_name', ...
        'parameter_label', 'output_dirs', 'truth_path', 'method_ids', ...
        'method_names', 'method_files', 'valid_masks', 'artifact_root'};
    for index = 1:numel(required)
        if ~isfield(context, required{index})
            error('专题评价上下文缺少字段：%s', required{index});
        end
    end
    scenario_count = numel(context.scenario_ids);
    method_count = numel(context.method_ids);
    if scenario_count < 2 || method_count < 1 || ...
            numel(context.scenario_names) ~= scenario_count || ...
            numel(context.parameter_values) ~= scenario_count || ...
            numel(context.output_dirs) ~= scenario_count || ...
            numel(context.method_names) ~= method_count || ...
            numel(context.method_files) ~= method_count || ...
            ~isequal(size(context.valid_masks), [scenario_count, method_count])
        error('专题评价上下文中的工况或算法维度不一致。');
    end
    if ~isfile(context.truth_path)
        error('真值文件不存在：%s', context.truth_path);
    end
end

function plot_method_sensitivity(context, truth_position, navigation, ...
        radial_errors, evaluation_mask, method_index, time)
%PLOT_METHOD_SENSITIVITY 绘制一种算法在全部误差工况下的结果。
    indices = find(evaluation_mask);
    plot_indices = indices(unique(round(linspace(1, numel(indices), ...
        min(20000, numel(indices))))));
    origin = truth_position(plot_indices(1), :);
    [truth_east, truth_north] = position_to_local_plane( ...
        truth_position(plot_indices, :), origin);
    colors = lines(numel(context.scenario_ids));

    figure_handle = myfigurestartup(10, 5, 'prese');
    layout = tiledlayout(figure_handle, 1, 2, ...
        'TileSpacing', 'compact', 'Padding', 'compact');
    title(layout, sprintf('%s：%s', context.study_title, ...
        context.method_names(method_index)));

    nexttile;
    plot(truth_east/1000, truth_north/1000, 'k-', ...
        'LineWidth', 1.7, 'DisplayName', '真值');
    hold on;
    for scenario_index = 1:numel(navigation)
        [east, north] = position_to_local_plane( ...
            navigation{scenario_index}(plot_indices, 3:5), origin);
        plot(east/1000, north/1000, 'Color', colors(scenario_index, :), ...
            'LineWidth', 1.1, ...
            'DisplayName', context.scenario_names(scenario_index));
    end
    grid on; box on; axis equal;
    xlabel('东向位置（km）'); ylabel('北向位置（km）');
    title('共同评价区间轨迹');
    legend('Location', 'best', 'Interpreter', 'none');

    nexttile;
    hold on;
    elapsed_time = time-time(1);
    for scenario_index = 1:numel(radial_errors)
        plot(elapsed_time(plot_indices), ...
            radial_errors{scenario_index}(plot_indices), ...
            'Color', colors(scenario_index, :), 'LineWidth', 1.05, ...
            'DisplayName', context.scenario_names(scenario_index));
    end
    grid on; box on;
    xlabel('相对导航起点的时间（s）'); ylabel('水平径向误差（m）');
    title('水平径向误差');
    xlim([elapsed_time(indices(1)), elapsed_time(indices(end))]);
    legend('Location', 'best', 'Interpreter', 'none');

    set(findall(figure_handle, '-property', 'FontName'), ...
        'FontName', 'TimesSimSun');
    file_prefix = context.study_id + "-" + context.method_ids(method_index);
    exportgraphics(figure_handle, fullfile(context.artifact_root, ...
        file_prefix + "-comparison.png"), 'Resolution', 600);
    savefig(figure_handle, fullfile(context.artifact_root, ...
        file_prefix + "-comparison.fig"));
end

function plot_rmse_response(context, statistics)
%PLOT_RMSE_RESPONSE 绘制工程误差参数与导航RMSE的响应关系。
    figure_handle = myfigurestartup(6.5, 5, 'prese');
    hold on;
    colors = lines(numel(context.method_ids));
    parameter_column = statistics.(char(context.parameter_name));
    for method_index = 1:numel(context.method_ids)
        mask = statistics.Method == context.method_names(method_index);
        [x, order] = sort(parameter_column(mask));
        y = statistics.RMSE_m(mask);
        plot(x, y(order), '-o', 'Color', colors(method_index, :), ...
            'LineWidth', 1.35, 'MarkerSize', 5, ...
            'DisplayName', context.method_names(method_index));
    end
    grid on; box on;
    xlabel(context.parameter_label); ylabel('水平位置 RMSE（m）');
    title(context.study_title + "：误差—精度响应");
    legend('Location', 'best');
    set(findall(figure_handle, '-property', 'FontName'), ...
        'FontName', 'TimesSimSun');
    exportgraphics(figure_handle, fullfile(context.artifact_root, ...
        context.study_id + "-rmse-response.png"), 'Resolution', 600);
    savefig(figure_handle, fullfile(context.artifact_root, ...
        context.study_id + "-rmse-response.fig"));
end

function radial_error = horizontal_radial_error(estimate, truth)
%HORIZONTAL_RADIAL_ERROR 按WGS-84局部曲率计算水平位置误差。
    latitude = deg2rad(truth(:, 1));
    height = truth(:, 3);
    [rm, rn] = wgs84_radii(latitude);
    north_error = deg2rad(estimate(:, 1)-truth(:, 1)).*(rm+height);
    east_error = deg2rad(estimate(:, 2)-truth(:, 2)).* ...
        (rn+height).*cos(latitude);
    radial_error = hypot(north_error, east_error);
end

function [east, north] = position_to_local_plane(position, origin)
%POSITION_TO_LOCAL_PLANE 将经纬高转换为相对原点的局部水平坐标。
    latitude = deg2rad(origin(1));
    [rm, rn] = wgs84_radii(latitude);
    north = deg2rad(position(:, 1)-origin(1))*(rm+origin(3));
    east = deg2rad(position(:, 2)-origin(2))* ...
        (rn+origin(3))*cos(latitude);
end

function [rm, rn] = wgs84_radii(latitude)
%WGS84_RADII 计算WGS-84子午圈和卯酉圈曲率半径。
    a = 6378137.0;
    e2 = 6.69437999014e-3;
    denominator = sqrt(1-e2*sin(latitude).^2);
    rn = a./denominator;
    rm = a*(1-e2)./denominator.^3;
end
