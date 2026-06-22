function results = analyze_beacon_observability(obs, window_measurements, output_dir)
%ANALYZE_BEACON_OBSERVABILITY Compare alternating and fixed beacons.
%
% results = analyze_beacon_observability(obs, window_measurements, output_dir)
%
% The finite-horizon normalized observability matrix is
%
%   O = [R^(-1/2) H_s S;
%        R^(-1/2) H_(s+1) Phi_(s+1,s) S;
%        ...],
%
% where x = S*x_bar. Metrics for the horizontal-position subspace are the
% most direct indicators of measurement-geometry improvement. Full-state
% metrics are also reported but can remain rank deficient for short windows.

    if nargin < 2 || isempty(window_measurements)
        window_measurements = 18;
    end
    if nargin < 3 || isempty(output_dir)
        output_dir = pwd;
    end

    validateattributes(window_measurements, {'numeric'}, ...
        {'scalar', 'integer', '>=', 2});
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    num_events = numel(obs.events);
    if num_events < 2
        error('At least two acoustic measurement events are required.');
    end
    if window_measurements > num_events
        warning('Window shortened from %d to %d available measurements.', ...
            window_measurements, num_events);
        window_measurements = num_events;
    end

    num_beacons = size(obs.beacon_positions, 1);
    scenario_names = cell(1, num_beacons + 1);
    scenario_names{1} = 'Alternating';
    for beacon_id = 1:num_beacons
        scenario_names{beacon_id + 1} = sprintf('Fixed-B%d', beacon_id);
    end

    num_windows = num_events - window_measurements + 1;
    num_scenarios = numel(scenario_names);
    event_time = zeros(num_windows, 1);
    actual_beacon_id = zeros(num_events, 1);
    for event_id = 1:num_events
        actual_beacon_id(event_id) = obs.events(event_id).actual_beacon_id;
    end

    horizontal_lambda_min = nan(num_windows, num_scenarios);
    horizontal_condition = nan(num_windows, num_scenarios);
    horizontal_logdet = nan(num_windows, num_scenarios);
    full_rank = nan(num_windows, num_scenarios);
    full_sigma_weak = nan(num_windows, num_scenarios);
    full_log_pdet = nan(num_windows, num_scenarios);

    for window_id = 1:num_windows
        first_event = window_id;
        last_event = window_id + window_measurements - 1;
        event_time(window_id) = obs.events(last_event).time;

        for scenario_id = 1:num_scenarios
            O = build_window_matrix(obs, first_event, last_event, ...
                scenario_id, actual_beacon_id);

            O_horizontal = O(:, 1:2);
            W_horizontal = O_horizontal' * O_horizontal;
            W_horizontal = 0.5 * (W_horizontal + W_horizontal');
            lambda = sort(max(real(eig(W_horizontal)), 0), 'descend');

            horizontal_lambda_min(window_id, scenario_id) = lambda(end);
            horizontal_condition(window_id, scenario_id) = ...
                lambda(1) / max(lambda(end), eps);
            horizontal_logdet(window_id, scenario_id) = ...
                sum(log10(lambda + 1e-12));

            singular_values = svd(O);
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

    results.summary = make_summary_table(results);
    writetable(results.summary, fullfile(output_dir, ...
        'beacon_observability_summary.csv'));
    results.window_table = make_window_table(results);
    writetable(results.window_table, fullfile(output_dir, ...
        'beacon_observability_windows.csv'));
    save(fullfile(output_dir, 'beacon_observability_results.mat'), ...
        'results', 'obs');
    try
        plot_results(results, output_dir);
    catch ME
        warning('Failed to generate comparison plot: %s', ME.message);
    end
    try
        plot_boxcharts(results, output_dir);
    catch ME
        warning('Failed to generate boxchart plot: %s', ME.message);
    end

    fprintf(['Observability analysis completed: %d measurements, ', ...
        '%d-measurement window, %d sliding windows.\n'], ...
        num_events, window_measurements, num_windows);
end

function O = build_window_matrix(obs, first_event, last_event, ...
        scenario_id, actual_beacon_id)
    num_rows = last_event - first_event + 1;
    O = zeros(num_rows, obs.rank);
    transition = eye(obs.rank);

    for event_id = first_event:last_event
        if event_id > first_event
            transition = obs.events(event_id).phi_from_previous * transition;
        end

        if scenario_id == 1
            beacon_id = actual_beacon_id(event_id);
        else
            beacon_id = scenario_id - 1;
        end

        H = obs.events(event_id).H_candidates(beacon_id, :);
        row_id = event_id - first_event + 1;
        O(row_id, :) = (H * transition * obs.state_scale) / obs.range_std;
    end
end

function summary = make_summary_table(results)
    names = string(results.scenario_names(:));
    summary = table( ...
        names, ...
        mean(results.horizontal_lambda_min, 1, 'omitnan')', ...
        median(results.horizontal_lambda_min, 1, 'omitnan')', ...
        mean(results.horizontal_condition, 1, 'omitnan')', ...
        mean(results.horizontal_logdet, 1, 'omitnan')', ...
        mean(results.full_rank, 1, 'omitnan')', ...
        mean(results.full_sigma_weak, 1, 'omitnan')', ...
        'VariableNames', { ...
            'Scenario', ...
            'MeanHorizontalLambdaMin', ...
            'MedianHorizontalLambdaMin', ...
            'MeanHorizontalCondition', ...
            'MeanHorizontalLogDet', ...
            'MeanFullRank', ...
            'MeanFullWeakSingularValue'});
end

function window_table = make_window_table(results)
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
        end
    end

    window_table = table(WindowID, WindowEndTime, Scenario, ...
        HorizontalLambdaMin, HorizontalCondition, HorizontalLogDet, ...
        FullRank, FullWeakSingularValue);
end

function plot_results(results, output_dir)
    colors = lines(numel(results.scenario_names));
    fig = figure('Color', 'w', 'Name', 'Beacon observability comparison', ...
        'Position', [100, 100, 1200, 720]);
    t = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot_metric(results.time, results.horizontal_lambda_min, colors);
    ylabel('\lambda_{min}(W_{horizontal})');
    title('Weakest horizontal information');

    nexttile;
    plot_metric(results.time, results.horizontal_condition, colors);
    set(gca, 'YScale', 'log');
    ylabel('Condition number');
    title('Horizontal information conditioning');

    nexttile;
    plot_metric(results.time, results.horizontal_logdet, colors);
    ylabel('log_{10} det(W_{horizontal})');
    xlabel('Time (s)');
    title('Horizontal information volume');

    nexttile;
    plot_metric(results.time, results.full_rank, colors);
    ylabel('Effective rank');
    xlabel('Time (s)');
    title('Full 15-state observability rank');

    % Shared legend at the bottom of the tiled layout
    lg = legend(t, results.scenario_names, 'Location', 'south', ...
        'Orientation', 'horizontal', 'NumColumns', numel(results.scenario_names));
    lg.Layout.Tile = 'south';

    exportgraphics(fig, fullfile(output_dir, 'beacon_observability_comparison.png'), ...
        'Resolution', 300);
    exportgraphics(fig, fullfile(output_dir, 'beacon_observability_comparison.svg'));
    close(fig);
end

function plot_boxcharts(results, output_dir)
    num_scenarios = numel(results.scenario_names);
    num_windows = size(results.horizontal_lambda_min, 1);
    group = repmat(1:num_scenarios, num_windows, 1);
    group = group(:);
    short_labels = {'Alt', 'B1', 'B2', 'B3'};
    short_labels = short_labels(1:num_scenarios);

    fig = figure('Color', 'w', 'Name', ...
        'Sliding-window observability distributions', ...
        'Position', [100, 100, 1500, 520]);
    tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

    % Suppress boxplot warning about labels when data contains NaN
    warning('off', 'MATLAB:boxplot:EmptyGroup');

    nexttile;
    boxplot(results.horizontal_lambda_min(:), group, ...
        'Labels', short_labels);
    ylabel('\lambda_{min}(W_{horizontal})');
    title('Weakest horizontal information');
    grid on;

    nexttile;
    boxplot(results.horizontal_condition(:), group, ...
        'Labels', short_labels);
    set(gca, 'YScale', 'log');
    ylabel('Condition number');
    title('Horizontal conditioning');
    grid on;

    nexttile;
    boxplot(results.horizontal_logdet(:), group, ...
        'Labels', short_labels);
    ylabel('log_{10} det(W_{horizontal})');
    title('Horizontal information volume');
    grid on;

    warning('on', 'MATLAB:boxplot:EmptyGroup');

    exportgraphics(fig, fullfile(output_dir, ...
        'beacon_observability_boxplots.png'), 'Resolution', 300);
    exportgraphics(fig, fullfile(output_dir, ...
        'beacon_observability_boxplots.svg'));
    close(fig);
end

function plot_metric(time, values, colors)
    hold on;
    for scenario_id = 1:size(values, 2)
        valid = isfinite(values(:, scenario_id));
        if any(valid)
            plot(time(valid), values(valid, scenario_id), 'LineWidth', 1.3, ...
                'Color', colors(scenario_id, :));
        end
    end
    grid on;
end
