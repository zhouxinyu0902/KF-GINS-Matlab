function results = analyze_beacon_observability_v2(obs, window_measurements)
%ANALYZE_BEACON_OBSERVABILITY_V2 计算轮换信标与固定信标的有限窗口可观测性指标。
%
%   results = analyze_beacon_observability_v2(obs)
%   results = analyze_beacon_observability_v2(obs, window_measurements)
%
% 输入：
%   obs                 观测分析结构体，至少应包含以下字段：
%                       obs.events                测距事件序列
%                       obs.events(k).time        第 k 次测距时刻
%                       obs.events(k).actual_beacon_id
%                                               轮换信标工况下第 k 次实际调用的信标编号
%                       obs.events(k).phi_from_previous
%                                               从上一测距时刻到当前测距时刻的状态转移矩阵
%                       obs.events(k).H_candidates
%                                               各候选信标对应的测距观测矩阵
%                       obs.beacon_positions      信标位置矩阵，每行对应一个信标
%                       obs.rank                  误差状态维数
%                       obs.state_scale           状态尺度归一化矩阵
%                       obs.range_std             测距标准差
%   window_measurements 滑动窗口内包含的测距次数，默认值为 18。
%
% 输出：
%   results             可观测性分析结果结构体。该函数只负责计算并返回结果，
%                       不在函数内部写入 CSV、MAT 或图片文件。
%
% 说明：
%   本函数构造有限窗口归一化可观测矩阵：
%
%       O = [R^(-1/2) H_s S;
%            R^(-1/2) H_(s+1) Phi_(s+1,s) S;
%            ...]
%
%   其中 x = S*x_bar。水平位置子空间指标用于直接评价测距几何对水平定位
%   信息的改善效果；全状态指标同时给出，但短窗口内全状态矩阵可能保持秩亏。
%
%   如需导出结果，请在函数外调用：
%       export_beacon_observability_results(results, output_dir, ...)
%
% 示例：
%   results = analyze_beacon_observability_v2(obs, 18);
%   export_beacon_observability_results(results, './output', ...
%       'Obs', obs, ...
%       'ExportSummaryCSV', true, ...
%       'ExportWindowCSV', true, ...
%       'ExportMat', true, ...
%       'ExportComparisonFigure', true, ...
%       'ExportBoxFigure', true);

    % ---------- 输入参数默认值与合法性检查 ----------
    if nargin < 2 || isempty(window_measurements)
        window_measurements = 18;
    end

    validateattributes(window_measurements, {'numeric'}, ...
        {'scalar', 'integer', '>=', 2}, mfilename, 'window_measurements');

    required_fields = {'events', 'beacon_positions', 'rank', 'state_scale', 'range_std'};
    for i = 1:numel(required_fields)
        if ~isfield(obs, required_fields{i})
            error('obs 缺少必要字段：%s。', required_fields{i});
        end
    end

    num_events = numel(obs.events);
    if num_events < 2
        error('至少需要两个声学测距事件才能进行有限窗口可观测性分析。');
    end

    if window_measurements > num_events
        warning('窗口长度由 %d 缩短为可用测距次数 %d。', ...
            window_measurements, num_events);
        window_measurements = num_events;
    end

    % ---------- 工况定义 ----------
    % 第 1 个工况为轮换信标；其余工况为固定调用单个信标。
    num_beacons = size(obs.beacon_positions, 1);
    scenario_names = cell(1, num_beacons + 1);
    scenario_names{1} = 'Alternating';
    for beacon_id = 1:num_beacons
        scenario_names{beacon_id + 1} = sprintf('Fixed-B%d', beacon_id);
    end

    num_windows = num_events - window_measurements + 1;
    num_scenarios = numel(scenario_names);

    % ---------- 提取实际轮换信标编号 ----------
    event_time = zeros(num_windows, 1);
    actual_beacon_id = zeros(num_events, 1);
    for event_id = 1:num_events
        actual_beacon_id(event_id) = obs.events(event_id).actual_beacon_id;
    end

    % ---------- 预分配指标矩阵 ----------
    % 每行对应一个滑动窗口，每列对应一个信标工况。
    horizontal_lambda_min = nan(num_windows, num_scenarios);   % 水平信息矩阵最小特征值
    horizontal_condition = nan(num_windows, num_scenarios);    % 水平信息矩阵条件数
    horizontal_logdet = nan(num_windows, num_scenarios);       % 水平信息矩阵 log10(det)
    full_rank = nan(num_windows, num_scenarios);               % 全状态可观测矩阵数值秩
    full_sigma_weak = nan(num_windows, num_scenarios);         % 全状态最弱可观奇异值
    full_log_pdet = nan(num_windows, num_scenarios);           % 全状态伪行列式对数指标

    % ---------- 滑动窗口可观测性计算 ----------
    for window_id = 1:num_windows
        first_event = window_id;
        last_event = window_id + window_measurements - 1;
        event_time(window_id) = obs.events(last_event).time;

        for scenario_id = 1:num_scenarios
            O = build_window_matrix_v2(obs, first_event, last_event, ...
                scenario_id, actual_beacon_id);

            % 水平位置子空间：默认取状态向量前两维为水平位置误差。
            O_horizontal = O(:, 1:2);
            W_horizontal = O_horizontal' * O_horizontal;
            W_horizontal = 0.5 * (W_horizontal + W_horizontal');
            lambda = sort(max(real(eig(W_horizontal)), 0), 'descend');

            horizontal_lambda_min(window_id, scenario_id) = lambda(end);
            horizontal_condition(window_id, scenario_id) = ...
                lambda(1) / max(lambda(end), eps);
            horizontal_logdet(window_id, scenario_id) = ...
                sum(log10(lambda + 1e-12));

            % 全状态矩阵的数值秩与弱可观方向指标。
            singular_values = svd(O);
            if isempty(singular_values)
                continue;
            end

            tolerance = max(size(O)) * eps(max(singular_values)) * 1e3;
            observable = singular_values > tolerance;
            full_rank(window_id, scenario_id) = nnz(observable);

            if any(observable)
                retained = singular_values(observable);
                full_sigma_weak(window_id, scenario_id) = retained(end);
                full_log_pdet(window_id, scenario_id) = ...
                    2 * sum(log10(retained));
            end
        end
    end

    % ---------- 汇总结果 ----------
    results.window_measurements = window_measurements;
    results.scenario_names = scenario_names;
    results.time = event_time;
    results.actual_beacon_id = actual_beacon_id;
    results.horizontal_lambda_min = horizontal_lambda_min;
    results.horizontal_condition = horizontal_condition;
    results.horizontal_logdet = horizontal_logdet;
    results.full_rank = full_rank;
    results.full_sigma_weak = full_sigma_weak;
    results.full_log_pdet = full_log_pdet;

    results.summary = make_summary_table_v2(results);
    results.window_table = make_window_table_v2(results);

    fprintf(['可观测性分析完成：共 %d 次测距，窗口长度为 %d 次测距，', ...
        '得到 %d 个滑动窗口。\n'], ...
        num_events, window_measurements, num_windows);
end

function O = build_window_matrix_v2(obs, first_event, last_event, ...
        scenario_id, actual_beacon_id)
%BUILD_WINDOW_MATRIX_V2 构造指定窗口和指定工况下的归一化可观测矩阵。

    num_rows = last_event - first_event + 1;
    O = zeros(num_rows, obs.rank);
    transition = eye(obs.rank);

    for event_id = first_event:last_event
        % 累乘窗口起点到当前测距时刻的状态转移矩阵。
        if event_id > first_event
            transition = obs.events(event_id).phi_from_previous * transition;
        end

        % 轮换工况使用实际调用信标；固定工况强制使用对应编号信标。
        if scenario_id == 1
            beacon_id = actual_beacon_id(event_id);
        else
            beacon_id = scenario_id - 1;
        end

        H = obs.events(event_id).H_candidates(beacon_id, :);
        row_id = event_id - first_event + 1;

        % R^(-1/2) H Phi S。这里 R = range_std^2。
        O(row_id, :) = (H * transition * obs.state_scale) / obs.range_std;
    end
end

function summary = make_summary_table_v2(results)
%MAKE_SUMMARY_TABLE_V2 生成不同信标工况的窗口平均统计表。

    names = string(results.scenario_names(:));
    summary = table( ...
        names, ...
        mean(results.horizontal_lambda_min, 1, 'omitnan')', ...
        median(results.horizontal_lambda_min, 1, 'omitnan')', ...
        mean(results.horizontal_condition, 1, 'omitnan')', ...
        median(results.horizontal_condition, 1, 'omitnan')', ...
        mean(results.horizontal_logdet, 1, 'omitnan')', ...
        mean(results.full_rank, 1, 'omitnan')', ...
        mean(results.full_sigma_weak, 1, 'omitnan')', ...
        mean(results.full_log_pdet, 1, 'omitnan')', ...
        'VariableNames', { ...
            'Scenario', ...
            'MeanHorizontalLambdaMin', ...
            'MedianHorizontalLambdaMin', ...
            'MeanHorizontalCondition', ...
            'MedianHorizontalCondition', ...
            'MeanHorizontalLogDet', ...
            'MeanFullRank', ...
            'MeanFullWeakSingularValue', ...
            'MeanFullLogPseudoDet'});
end

function window_table = make_window_table_v2(results)
%MAKE_WINDOW_TABLE_V2 生成逐窗口、逐工况的长表，便于后处理或绘图。

    num_windows = numel(results.time);
    num_scenarios = numel(results.scenario_names);
    num_rows = num_windows * num_scenarios;

    WindowID = zeros(num_rows, 1);
    WindowEndTime = zeros(num_rows, 1);
    Scenario = strings(num_rows, 1);
    HorizontalLambdaMin = zeros(num_rows, 1);
    HorizontalCondition = zeros(num_rows, 1);
    HorizontalLogDet = zeros(num_rows, 1);
    FullRank = zeros(num_rows, 1);
    FullWeakSingularValue = zeros(num_rows, 1);
    FullLogPseudoDet = zeros(num_rows, 1);

    row = 0;
    for window_id = 1:num_windows
        for scenario_id = 1:num_scenarios
            row = row + 1;
            WindowID(row) = window_id;
            WindowEndTime(row) = results.time(window_id);
            Scenario(row) = string(results.scenario_names{scenario_id});
            HorizontalLambdaMin(row) = ...
                results.horizontal_lambda_min(window_id, scenario_id);
            HorizontalCondition(row) = ...
                results.horizontal_condition(window_id, scenario_id);
            HorizontalLogDet(row) = ...
                results.horizontal_logdet(window_id, scenario_id);
            FullRank(row) = results.full_rank(window_id, scenario_id);
            FullWeakSingularValue(row) = ...
                results.full_sigma_weak(window_id, scenario_id);
            FullLogPseudoDet(row) = ...
                results.full_log_pdet(window_id, scenario_id);
        end
    end

    window_table = table(WindowID, WindowEndTime, Scenario, ...
        HorizontalLambdaMin, HorizontalCondition, HorizontalLogDet, ...
        FullRank, FullWeakSingularValue, FullLogPseudoDet);
end
