clear;
close all;
clc;
%% 四数据集固定滞后 RTS 与标准全局 RTS 精度/计算量对比
% 本脚本只读取已有结果，不重复运行导航算法。
% 精度在所有算法均达到完整滞后的严格共同区间内评价。
% 相对计算量按相同 RTS 节点密度下的反向区间处理次数估算，lag1=1。

%% 1. 用户配置
data_source = "experiment";
case_names = ["case-06", "case-07", "case-08", "case-09"];
position_error_unit = "m";
lag_segments = 1:4;
maximum_plot_points = 8000;

%% 2. 路径与输出
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
paths = setup_inertial_experiment();
param = Param();

if data_source ~= "experiment"
    error('当前四数据集批量分析固定为experiment/case-06至case-09。');
end
method_names = [compose("lag%d", lag_segments), "标准全局RTS"];
method_count = numel(method_names);
case_count = numel(case_names);
output_dir = fullfile(paths.data, 'summary', ...
    'all-lags-global-rts-comparison');
if ~isfolder(output_dir)
    mkdir(output_dir);
end

accuracy_row_count = case_count * method_count;
accuracy_dataset = strings(accuracy_row_count, 1);
accuracy_method = strings(accuracy_row_count, 1);
accuracy_start_time = zeros(accuracy_row_count, 1);
accuracy_end_time = zeros(accuracy_row_count, 1);
accuracy_duration = zeros(accuracy_row_count, 1);
accuracy_sample_count = zeros(accuracy_row_count, 1);
accuracy_rmse = zeros(accuracy_row_count, 1);
accuracy_mean = zeros(accuracy_row_count, 1);
accuracy_p95 = zeros(accuracy_row_count, 1);
accuracy_max = zeros(accuracy_row_count, 1);

work_dataset = strings(accuracy_row_count, 1);
work_method = strings(accuracy_row_count, 1);
work_event_count = zeros(accuracy_row_count, 1);
work_segment_passes = zeros(accuracy_row_count, 1);
work_relative = zeros(accuracy_row_count, 1);
work_peak_history = zeros(accuracy_row_count, 1);
work_complexity = strings(accuracy_row_count, 1);

rmse_matrix = nan(case_count, method_count);
p95_matrix = nan(case_count, method_count);
relative_work_matrix = nan(case_count, method_count);
curve_data = repmat(struct('time_h', {{}}, 'error_m', {{}}, ...
    'common_start_time', nan, 'common_end_time', nan), case_count, 1);

%% 3. 逐数据集读取与评价
for case_index = 1:case_count
    case_name = char(case_names(case_index));
    case_id = str2double(case_name(end));
    input_dir = paths.experiment_input(case_id);
    cfg = load_algorithm_exploration_config( ...
        data_source, position_error_unit, input_dir);
    fixed_lag_dir = fullfile(cfg.outputfolder, ...
        sprintf('fixed-lag-rts-%s', position_error_unit));
    global_dir = fullfile(cfg.outputfolder, ...
        sprintf('standard-global-rts-%s', position_error_unit));
    global_path = fullfile(global_dir, ...
        'range-ins-rts-standard-global.nav');
    global_diagnostic_path = fullfile(fileparts(cfg.figurefolder), ...
        'figures-tables', sprintf('standard-global-rts-%s', ...
        position_error_unit), 'range-rts-standard-global-events.csv');
    lag_paths = arrayfun(@(lag) fullfile(fixed_lag_dir, sprintf( ...
        'simple-rts-lag%d-%s.nav', lag, position_error_unit)), ...
        lag_segments, 'UniformOutput', false);
    required_paths = [lag_paths, {global_path, global_diagnostic_path, ...
        cfg.truthpath}];
    if any(~cellfun(@isfile, required_paths))
        missing_paths = required_paths(~cellfun(@isfile, required_paths));
        error('数据集%s缺少结果文件：\n%s', case_name, ...
            strjoin(missing_paths, newline));
    end

    fprintf('\n读取并分析 %s ...\n', case_name);
    global_diagnostic = readtable(global_diagnostic_path, ...
        'VariableNamingRule', 'preserve');
    event_count = height(global_diagnostic);
    if event_count < max(lag_segments)
        error('%s只有%d个全局RTS事件，无法评价lag%d。', ...
            case_name, event_count, max(lag_segments));
    end
    strict_segment_cutoff = event_count - max(lag_segments) + 1;

    global_nav = readmatrix(global_path, 'FileType', 'text');
    lag1_nav = readmatrix(lag_paths{1}, 'FileType', 'text');
    validate_nav(global_nav, sprintf('%s标准全局RTS', case_name));
    validate_nav(lag1_nav, sprintf('%s lag1', case_name));
    cutoff_mask = lag1_nav(:, 1) <= strict_segment_cutoff;
    if ~any(cutoff_mask)
        error('%s找不到严格共同区间。', case_name);
    end
    common_start_time = max(global_nav(1, 2), lag1_nav(1, 2));
    common_end_time = min([max(lag1_nav(cutoff_mask, 2)), ...
        global_nav(end, 2), global_diagnostic.time_s(end)]);
    if common_end_time <= common_start_time
        error('%s的严格共同评价区间无效。', case_name);
    end
    truth = readmatrix(cfg.truthpath, 'FileType', 'text');
    curve_data(case_index).time_h = cell(method_count, 1);
    curve_data(case_index).error_m = cell(method_count, 1);
    curve_data(case_index).common_start_time = common_start_time;
    curve_data(case_index).common_end_time = common_end_time;

    for method_index = 1:method_count
        if method_index <= numel(lag_segments)
            if method_index == 1
                nav = lag1_nav;
            else
                nav = readmatrix(lag_paths{method_index}, ...
                    'FileType', 'text');
            end
        else
            nav = global_nav;
        end
        validate_nav(nav, sprintf('%s %s', case_name, ...
            method_names(method_index)));
        evaluation_mask = nav(:, 2) >= common_start_time & ...
            nav(:, 2) <= common_end_time;
        evaluation_nav = nav(evaluation_mask, :);
        horizontal_error = calculate_horizontal_error_m( ...
            evaluation_nav, truth, param);

        row_index = (case_index - 1) * method_count + method_index;
        accuracy_dataset(row_index) = case_names(case_index);
        accuracy_method(row_index) = method_names(method_index);
        accuracy_start_time(row_index) = common_start_time;
        accuracy_end_time(row_index) = common_end_time;
        accuracy_duration(row_index) = common_end_time - common_start_time;
        accuracy_sample_count(row_index) = numel(horizontal_error);
        accuracy_rmse(row_index) = sqrt(mean(horizontal_error .^ 2));
        accuracy_mean(row_index) = mean(horizontal_error);
        accuracy_p95(row_index) = prctile(horizontal_error, 95);
        accuracy_max(row_index) = max(horizontal_error);
        rmse_matrix(case_index, method_index) = accuracy_rmse(row_index);
        p95_matrix(case_index, method_index) = accuracy_p95(row_index);

        plot_indices = select_plot_indices( ...
            numel(horizontal_error), maximum_plot_points);
        curve_data(case_index).time_h{method_index} = ...
            (evaluation_nav(plot_indices, 2) - common_start_time) / 3600;
        curve_data(case_index).error_m{method_index} = ...
            horizontal_error(plot_indices);
        if method_index > 1 && method_index < method_count
            clear nav;
        end
    end

    % 相同节点密度下的算法工作量：每处理一个历史区间记一次单位工作。
    lag1_work = event_count;
    for method_index = 1:method_count
        row_index = (case_index - 1) * method_count + method_index;
        if method_index <= numel(lag_segments)
            lag = lag_segments(method_index);
            segment_passes = sum(min((1:event_count)', lag));
            peak_history = lag;
            complexity = sprintf('O(M*%d*S)', lag);
        else
            segment_passes = event_count * (event_count + 1) / 2;
            peak_history = event_count;
            complexity = 'O(M^2*S)';
        end
        work_dataset(row_index) = case_names(case_index);
        work_method(row_index) = method_names(method_index);
        work_event_count(row_index) = event_count;
        work_segment_passes(row_index) = segment_passes;
        work_relative(row_index) = segment_passes / lag1_work;
        work_peak_history(row_index) = peak_history;
        work_complexity(row_index) = complexity;
        relative_work_matrix(case_index, method_index) = ...
            work_relative(row_index);
    end
    fprintf(['%s：全局RTS事件=%d，严格评价到第%d段，' ...
        '共同区间 %.2f--%.2f s。\n'], case_name, event_count, ...
        strict_segment_cutoff, common_start_time, common_end_time);
    clear truth lag1_nav global_nav;
end

%% 4. 保存统计表
accuracy_table = table(accuracy_dataset, accuracy_method, ...
    accuracy_start_time, accuracy_end_time, accuracy_duration, ...
    accuracy_sample_count, accuracy_rmse, accuracy_mean, accuracy_p95, ...
    accuracy_max, 'VariableNames', {'dataset', 'method', 'start_time_s', ...
    'end_time_s', 'duration_s', 'sample_count', 'horizontal_rmse_m', ...
    'horizontal_mean_m', 'horizontal_p95_m', 'horizontal_max_m'});
work_table = table(work_dataset, work_method, work_event_count, ...
    work_segment_passes, work_relative, work_peak_history, ...
    work_complexity, 'VariableNames', {'dataset', 'method', ...
    'range_segment_count_M', 'total_segment_passes', ...
    'relative_compute_lag1', 'relative_peak_history_segments', ...
    'asymptotic_complexity'});

accuracy_path = fullfile(output_dir, ...
    'all-lags-global-rts-accuracy.csv');
work_path = fullfile(output_dir, ...
    'all-lags-global-rts-relative-compute.csv');
writetable(accuracy_table, accuracy_path);
writetable(work_table, work_path);

normalized_rmse = rmse_matrix ./ rmse_matrix(:, 1);
method_summary = table(method_names(:), ...
    mean(normalized_rmse, 1)', mean(relative_work_matrix, 1)', ...
    mean(rmse_matrix, 1)', mean(p95_matrix, 1)', ...
    'VariableNames', {'method', 'mean_normalized_rmse_vs_lag1', ...
    'mean_relative_compute_vs_lag1', 'mean_horizontal_rmse_m', ...
    'mean_horizontal_p95_m'});
summary_path = fullfile(output_dir, ...
    'all-lags-global-rts-method-summary.csv');
writetable(method_summary, summary_path);

%% 5. 四数据集水平误差曲线
colors = lines(method_count);
curve_figure = figure('Color', 'w', 'Visible', 'off', ...
    'Position', [50, 50, 1500, 900]);
curve_layout = tiledlayout(curve_figure, 2, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
for case_index = 1:case_count
    nexttile;
    hold on;
    for method_index = 1:method_count
        plot(curve_data(case_index).time_h{method_index}, ...
            curve_data(case_index).error_m{method_index}, ...
            'LineWidth', 0.9, 'Color', colors(method_index, :));
    end
    grid on;
    xlabel('相对时间 / h');
    ylabel('水平误差 / m');
    title(case_names(case_index), 'Interpreter', 'none');
end
legend(curve_layout.Children(end), method_names, ...
    'Location', 'northoutside', 'Orientation', 'horizontal');
title(curve_layout, '固定滞后 lag1--lag4 与标准全局 RTS 水平误差');
curve_figure_path = fullfile(output_dir, ...
    'all-lags-global-rts-horizontal-error-curves.png');
exportgraphics(curve_figure, curve_figure_path, 'Resolution', 200);
close(curve_figure);

%% 6. 精度、计算量和折中关系
accuracy_figure = figure('Color', 'w', 'Visible', 'off', ...
    'Position', [100, 100, 1250, 760]);
tiledlayout(accuracy_figure, 2, 1, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
nexttile;
bar(categorical(case_names), rmse_matrix, 'grouped');
grid on;
ylabel('水平 RMSE / m');
title('严格共同区间水平 RMSE');
legend(method_names, 'Location', 'northoutside', ...
    'Orientation', 'horizontal');
nexttile;
bar(categorical(case_names), p95_matrix, 'grouped');
grid on;
xlabel('数据集');
ylabel('水平误差 P95 / m');
title('严格共同区间水平误差 P95');
accuracy_figure_path = fullfile(output_dir, ...
    'all-lags-global-rts-accuracy-summary.png');
exportgraphics(accuracy_figure, accuracy_figure_path, 'Resolution', 200);
close(accuracy_figure);

work_figure = figure('Color', 'w', 'Visible', 'off', ...
    'Position', [100, 100, 1200, 600]);
bar(categorical(case_names), relative_work_matrix, 'grouped');
grid on;
xlabel('数据集');
ylabel('相对计算量（lag1 = 1）');
title('相同 RTS 节点密度下的累计反向递推计算量');
legend(method_names, 'Location', 'northoutside', ...
    'Orientation', 'horizontal');
work_figure_path = fullfile(output_dir, ...
    'all-lags-global-rts-relative-compute.png');
exportgraphics(work_figure, work_figure_path, 'Resolution', 200);
close(work_figure);

tradeoff_figure = figure('Color', 'w', 'Visible', 'off', ...
    'Position', [100, 100, 900, 620]);
mean_relative_work = mean(relative_work_matrix, 1);
mean_normalized_rmse = mean(normalized_rmse, 1);
scatter(mean_relative_work, mean_normalized_rmse, 80, colors, 'filled');
grid on;
xlabel('四数据集平均相对计算量（lag1 = 1）');
ylabel('四数据集平均归一化 RMSE（lag1 = 1）');
title('RTS 精度—计算量折中（越靠左下越优）');
for method_index = 1:method_count
    text(mean_relative_work(method_index), ...
        mean_normalized_rmse(method_index), ...
        "  " + method_names(method_index), 'VerticalAlignment', 'middle');
end
tradeoff_figure_path = fullfile(output_dir, ...
    'all-lags-global-rts-accuracy-compute-tradeoff.png');
exportgraphics(tradeoff_figure, tradeoff_figure_path, 'Resolution', 200);
close(tradeoff_figure);

%% 7. 控制台摘要
fprintf('\n四数据集方法汇总（RMSE先对每个数据集以lag1归一化）：\n');
disp(method_summary);
fprintf('精度明细：%s\n', accuracy_path);
fprintf('相对计算量：%s\n', work_path);
fprintf('方法汇总：%s\n', summary_path);
fprintf('误差曲线：%s\n', curve_figure_path);
fprintf('精度汇总图：%s\n', accuracy_figure_path);
fprintf('计算量图：%s\n', work_figure_path);
fprintf('精度—计算量折中图：%s\n', tradeoff_figure_path);

function validate_nav(nav, label)
    if isempty(nav) || size(nav, 2) < 11 || ...
            any(~isfinite(nav(:, 1:8)), 'all') || ...
            any(diff(nav(:, 2)) < 0)
        error('%s不是有效的导航结果。', label);
    end
end

function horizontal_error = calculate_horizontal_error_m(nav, truth, param)
    truth_position = interp1(truth(:, 2), truth(:, 3:5), ...
        nav(:, 2), 'linear', nan);
    if any(~isfinite(truth_position), 'all')
        error('评价区间内存在无法插值的真值。');
    end
    latitude = truth_position(:, 1) * param.D2R;
    reference_latitude = mean(latitude);
    reference_height = mean(truth_position(:, 3));
    [rm, rn] = getRmRn(reference_latitude, param);
    north_error = (nav(:, 3) - truth_position(:, 1)) ...
        * param.D2R * (rm + reference_height);
    east_error = (nav(:, 4) - truth_position(:, 2)) ...
        * param.D2R * (rn + reference_height) * cos(reference_latitude);
    horizontal_error = hypot(north_error, east_error);
end

function indices = select_plot_indices(sample_count, maximum_plot_points)
    if sample_count <= maximum_plot_points
        indices = (1:sample_count)';
    else
        indices = unique(round(linspace( ...
            1, sample_count, maximum_plot_points)))';
    end
end
