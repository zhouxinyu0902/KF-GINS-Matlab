clear;
close all;
clc;
%% 双水听器结果统一分析
% 自动扫描仿真/实测案例中现有的双水听器导航结果，不要求所有组合齐全。
% 输出：
%   1) navigation-summary.csv                导航误差汇总；
%   2) measurement-summary.csv               测角/测距精度汇总；
%   3) azimuth-diagnostic-summary.csv         先验角与选择角差异汇总；
%   4) 非掉帧误差、二次 RTS 轨迹及掉帧 RMSE 对比图。
%
% 说明：
%   - 空 case_filter 表示自动分析所有存在相关结果的案例；
%   - 导航误差默认按 1 Hz 等时间间隔计算，避免 100 Hz 文件过大；
%   - Full* 指各文件全部有效时段，Common* 指同案例所有结果的公共时段；
%   - 掉最后一帧时导航文件可能提前结束，掉帧横向比较应优先看 Common*。

options = struct( ...
    'data_sources', ["simulation", "experiment"], ...
    'case_filter', strings(0, 1), ... % 例如 "case-00"；空数组表示自动扫描
    'sample_interval_s', 1, ...
    'include_range_only_baseline', true, ...
    'include_range_azimuth_dropout', true, ...
    'include_range_only_dropout', true, ...
    'save_figures', true, ...
    'show_figures', false);

%% 路径
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));
addpath(topic_dir);
paths = setup_inertial_experiment();

analysis_root = fullfile(paths.data, 'double-hydrophone-analysis');
if ~isfolder(analysis_root)
    mkdir(analysis_root);
end

all_navigation_rows = struct([]);
all_measurement_rows = struct([]);
all_diagnostic_rows = struct([]);
analyzed_case_count = 0;

for source_index = 1:numel(options.data_sources)
    data_source = lower(options.data_sources(source_index));
    if data_source == "simulation"
        source_root = paths.simulation;
        truth_name = 'truth.txt';
    elseif data_source == "experiment"
        source_root = paths.experiment;
        truth_name = 'truth.nav';
    else
        warning('跳过未知数据源：%s。', data_source);
        continue;
    end

    case_dirs = dir(fullfile(source_root, 'case-*'));
    case_dirs = case_dirs([case_dirs.isdir]);
    for case_index = 1:numel(case_dirs)
        case_name = string(case_dirs(case_index).name);
        if ~isempty(options.case_filter) && ...
                ~any(case_name == string(options.case_filter))
            continue;
        end

        case_dir = fullfile(case_dirs(case_index).folder, ...
            case_dirs(case_index).name);
        input_dir = fullfile(case_dir, 'input');
        navigation_root = fullfile(case_dir, 'output', ...
            'navigation-results');
        truth_path = fullfile(input_dir, truth_name);
        if ~isfolder(navigation_root) || ~isfile(truth_path)
            continue;
        end

        nav_files = collect_navigation_files(navigation_root, options);
        if isempty(nav_files)
            continue;
        end

        analyzed_case_count = analyzed_case_count + 1;
        fprintf('\n[%s/%s] 找到 %d 个导航结果。\n', ...
            data_source, case_name, numel(nav_files));

        truth = readmatrix(truth_path, 'FileType', 'text');
        truth = truth(:, 1:min(size(truth, 2), 11));
        truth = truth(all(isfinite(truth(:, [2, 3, 4, 5])), 2), :);
        [truth_time, truth_unique_index] = unique(truth(:, 2), 'stable');
        truth = truth(truth_unique_index, :);
        if size(truth, 2) < 11 || numel(truth_time) < 2
            warning('%s 的真值文件不足 11 列或有效时间点不足，跳过。', ...
                case_name);
            continue;
        end

        time_bounds = nan(numel(nav_files), 2);
        for file_index = 1:numel(nav_files)
            [time_bounds(file_index, 1), time_bounds(file_index, 2)] = ...
                read_nav_time_bounds(nav_files(file_index).fullpath);
        end
        valid_bounds = all(isfinite(time_bounds), 2);
        nav_files = nav_files(valid_bounds);
        time_bounds = time_bounds(valid_bounds, :);
        if isempty(nav_files)
            warning('%s 没有可读取时间范围的导航文件。', case_name);
            continue;
        end

        common_start_time = max([truth_time(1); time_bounds(:, 1)]);
        common_end_time = min([truth_time(end); time_bounds(:, 2)]);
        if common_end_time <= common_start_time
            warning('%s 的结果不存在公共时间段，仅计算各文件全时段指标。', ...
                case_name);
            common_start_time = nan;
            common_end_time = nan;
        end

        no_drop_mask = false(numel(nav_files), 1);
        for file_index = 1:numel(nav_files)
            file_metadata = parse_navigation_metadata( ...
                nav_files(file_index).fullpath);
            no_drop_mask(file_index) = ~isfinite(file_metadata.DropIndex);
        end
        if any(no_drop_mask)
            no_drop_common_start_time = max([truth_time(1); ...
                time_bounds(no_drop_mask, 1)]);
            no_drop_common_end_time = min([truth_time(end); ...
                time_bounds(no_drop_mask, 2)]);
        else
            no_drop_common_start_time = nan;
            no_drop_common_end_time = nan;
        end

        reference_time = max(truth_time(1), min(time_bounds(:, 1)));
        reference_position = interp1(truth_time, truth(:, 3:5), ...
            reference_time, 'linear');

        case_rows = struct([]);
        traces = struct([]);
        for file_index = 1:numel(nav_files)
            metadata = parse_navigation_metadata( ...
                nav_files(file_index).fullpath);
            [row, trace] = analyze_navigation_file( ...
                nav_files(file_index).fullpath, truth, metadata, ...
                data_source, case_name, options.sample_interval_s, ...
                common_start_time, common_end_time, reference_position);
            if isempty(row)
                continue;
            end
            if ~isempty(row) && ~isfinite(metadata.DropIndex)
                [row, trace] = add_no_drop_common_metrics( ...
                    row, trace, no_drop_common_start_time, ...
                    no_drop_common_end_time);
            else
                row.NoDropCommonStartTimeS = nan;
                row.NoDropCommonEndTimeS = nan;
                row.NoDropCommonSampleCount = 0;
                row.NoDropCommonHorizontalRMSEM = nan;
                row.NoDropCommonHorizontalMeanM = nan;
                row.NoDropCommonHorizontalP95M = nan;
                row.NoDropCommonHorizontalMaxM = nan;
                trace.NoDropCommonHorizontalRMSEM = nan;
            end
            case_rows = append_struct_rows(case_rows, row);
            traces = append_struct_rows(traces, trace);
            fprintf('  %-62s Full RMSE = %8.3f m, Common RMSE = %8.3f m\n', ...
                char(row.ResultLabel), row.FullHorizontalRMSEM, ...
                row.CommonHorizontalRMSEM);
        end
        if isempty(case_rows)
            continue;
        end

        case_table = struct2table(case_rows);
        case_table = sortrows(case_table, ...
            {'DropIndex', 'Stage', 'MeasurementMode', ...
            'PhaseStdDeg', 'WeightMode'}, ...
            {'ascend', 'ascend', 'ascend', 'ascend', 'ascend'}, ...
            'MissingPlacement', 'first');

        case_output_dir = fullfile(analysis_root, ...
            sprintf('%s-%s', data_source, case_name));
        if ~isfolder(case_output_dir)
            mkdir(case_output_dir);
        end
        writetable(case_table, fullfile(case_output_dir, ...
            'navigation-summary.csv'));

        measurement_rows = collect_measurement_summaries( ...
            navigation_root, data_source, case_name);
        if ~isempty(measurement_rows)
            measurement_table = struct2table(measurement_rows);
            measurement_table = sortrows(measurement_table, ...
                {'PhaseStdDeg', 'WeightMode'});
            writetable(measurement_table, fullfile(case_output_dir, ...
                'measurement-summary.csv'));
            all_measurement_rows = append_struct_rows( ...
                all_measurement_rows, measurement_rows);
        end

        diagnostic_rows = collect_diagnostic_summaries( ...
            navigation_root, data_source, case_name);
        if ~isempty(diagnostic_rows)
            diagnostic_table = struct2table(diagnostic_rows);
            diagnostic_table = sortrows(diagnostic_table, ...
                {'PhaseStdDeg', 'WeightMode'});
            writetable(diagnostic_table, fullfile(case_output_dir, ...
                'azimuth-diagnostic-summary.csv'));
            all_diagnostic_rows = append_struct_rows( ...
                all_diagnostic_rows, diagnostic_rows);
        end

        create_case_figures(case_table, traces, case_output_dir, ...
            data_source, case_name, options);
        all_navigation_rows = append_struct_rows( ...
            all_navigation_rows, case_rows);
    end
end
%% 全部案例总表
if isempty(all_navigation_rows)
    warning('没有发现可分析的双水听器导航结果。');
else
    navigation_table = struct2table(all_navigation_rows);
    writetable(navigation_table, fullfile(analysis_root, ...
        'all-navigation-summary.csv'));
end
if ~isempty(all_measurement_rows)
    writetable(struct2table(all_measurement_rows), ...
        fullfile(analysis_root, 'all-measurement-summary.csv'));
end
if ~isempty(all_diagnostic_rows)
    writetable(struct2table(all_diagnostic_rows), ...
        fullfile(analysis_root, 'all-azimuth-diagnostic-summary.csv'));
end

fprintf('\n分析完成：%d 个案例。\n输出目录：%s\n', ...
    analyzed_case_count, analysis_root);

%% 局部函数
function [row, trace] = add_no_drop_common_metrics( ...
        row, trace, start_time, end_time)
    if ~isfinite(start_time) || ~isfinite(end_time) || end_time <= start_time
        row.NoDropCommonStartTimeS = nan;
        row.NoDropCommonEndTimeS = nan;
        row.NoDropCommonSampleCount = 0;
        row.NoDropCommonHorizontalRMSEM = nan;
        row.NoDropCommonHorizontalMeanM = nan;
        row.NoDropCommonHorizontalP95M = nan;
        row.NoDropCommonHorizontalMaxM = nan;
        return;
    end
    mask = trace.SampleTimeS >= start_time & trace.SampleTimeS <= end_time;
    horizontal_error_m = trace.HorizontalErrorM(mask);
    sample_time = trace.SampleTimeS(mask);
    row.NoDropCommonStartTimeS = sample_time(1);
    row.NoDropCommonEndTimeS = sample_time(end);
    row.NoDropCommonSampleCount = numel(sample_time);
    row.NoDropCommonHorizontalRMSEM = root_mean_square(horizontal_error_m);
    row.NoDropCommonHorizontalMeanM = mean(horizontal_error_m);
    row.NoDropCommonHorizontalP95M = percentile_value(horizontal_error_m, 95);
    row.NoDropCommonHorizontalMaxM = max(horizontal_error_m);
    trace.NoDropCommonHorizontalRMSEM = row.NoDropCommonHorizontalRMSEM;
end

function nav_files = collect_navigation_files(navigation_root, options)
    nav_files = struct('name', {}, 'folder', {}, 'fullpath', {});
    phase_dirs = dir(fullfile(navigation_root, ...
        'double-hydrophone-phasestd*deg'));
    phase_dirs = phase_dirs([phase_dirs.isdir]);
    range_dropout_files = dir(fullfile(navigation_root, ...
        'drop-noazi-2RTS', '*.nav'));
    if isempty(phase_dirs) && isempty(range_dropout_files)
        return;
    end
    if options.include_range_only_baseline
        baseline_files = dir(fullfile(navigation_root, ...
            'simple-ekf-rts-rad', 'simple-*-rad.nav'));
        nav_files = add_file_entries(nav_files, baseline_files);
    end
    for directory_index = 1:numel(phase_dirs)
        phase_dir = fullfile(phase_dirs(directory_index).folder, ...
            phase_dirs(directory_index).name);
        root_files = dir(fullfile(phase_dir, '*.nav'));
        nav_files = add_file_entries(nav_files, root_files);
        if options.include_range_azimuth_dropout
            dropout_files = dir(fullfile(phase_dir, ...
                'drop-hasazi-2RTS', '*.nav'));
            nav_files = add_file_entries(nav_files, dropout_files);
        end
    end
    if options.include_range_only_dropout
        nav_files = add_file_entries(nav_files, range_dropout_files);
    end
end

function files = add_file_entries(files, new_files)
    for index = 1:numel(new_files)
        entry.name = new_files(index).name;
        entry.folder = new_files(index).folder;
        entry.fullpath = fullfile(new_files(index).folder, ...
            new_files(index).name);
        files(end + 1) = entry; %#ok<AGROW>
    end
end

function metadata = parse_navigation_metadata(file_path)
    file_string = string(file_path);
    [~, file_name, extension] = fileparts(file_path);
    file_name = string(file_name) + string(extension);

    phase_token = regexp(char(file_string), ...
        'double-hydrophone-phasestd([0-9.]+)deg', ...
        'tokens', 'once');
    if isempty(phase_token)
        phase_std_deg = nan;
    else
        phase_std_deg = str2double(phase_token{1});
    end

    drop_token = regexp(char(file_name), 'drop([0-9]+)\.nav$', ...
        'tokens', 'once');
    if isempty(drop_token)
        drop_index = nan;
    else
        drop_index = str2double(drop_token{1});
    end

    if contains(file_name, 'range-only') || ...
            contains(file_name, 'range-noazi') || ...
            startsWith(file_name, 'simple-')
        measurement_mode = "range-only";
    else
        measurement_mode = "range+azimuth";
    end
    if contains(file_name, 'adaptive')
        weight_mode = "adaptive";
    elseif measurement_mode == "range+azimuth"
        weight_mode = "fixed";
    else
        weight_mode = "none";
    end
    if contains(file_name, 'forward')
        stage = "forward";
    elseif contains(file_name, 'rts-single')
        stage = "rts-single";
    elseif contains(file_name, 'rts-double')
        stage = "rts-double";
    else
        stage = "unknown";
    end

    phase_label = "phase-na";
    if isfinite(phase_std_deg)
        phase_label = "phase-" + string(phase_std_deg) + "deg";
    end
    drop_label = "no-drop";
    if isfinite(drop_index)
        drop_label = "drop-" + string(drop_index);
    end
    metadata.PhaseStdDeg = phase_std_deg;
    metadata.MeasurementMode = measurement_mode;
    metadata.WeightMode = weight_mode;
    metadata.Stage = stage;
    metadata.DropIndex = drop_index;
    metadata.ResultLabel = strjoin([measurement_mode, weight_mode, ...
        phase_label, stage, drop_label], " | ");
end

function [start_time, end_time] = read_nav_time_bounds(file_path)
    start_time = nan;
    end_time = nan;
    file_id = fopen(file_path, 'r');
    if file_id < 0
        return;
    end
    cleanup = onCleanup(@() fclose(file_id));

    while ~feof(file_id)
        line = fgetl(file_id);
        if ~ischar(line)
            break;
        end
        values = sscanf(line, '%f');
        if numel(values) >= 2
            start_time = values(2);
            break;
        end
    end

    file_info = dir(file_path);
    tail_byte_count = min(file_info.bytes, 65536);
    fseek(file_id, -tail_byte_count, 'eof');
    tail_text = fread(file_id, tail_byte_count, '*char')';
    tail_lines = regexp(tail_text, '\r\n|\n|\r', 'split');
    for line_index = numel(tail_lines):-1:1
        values = sscanf(tail_lines{line_index}, '%f');
        if numel(values) >= 2
            end_time = values(2);
            break;
        end
    end
end

function [row, trace] = analyze_navigation_file(file_path, truth, ...
        metadata, data_source, case_name, sample_interval_s, ...
        common_start_time, common_end_time, reference_position)
    row = [];
    trace = [];
    nav = readmatrix(file_path, 'FileType', 'text');
    if size(nav, 2) < 11
        warning('导航文件不足 11 列，跳过：%s', file_path);
        return;
    end
    nav = nav(all(isfinite(nav(:, [2, 3, 4, 5])), 2), :);
    [nav_time, unique_index] = unique(nav(:, 2), 'stable');
    nav = nav(unique_index, :);
    truth_time = truth(:, 2);

    start_time = max(nav_time(1), truth_time(1));
    end_time = min(nav_time(end), truth_time(end));
    sample_start = ceil(start_time / sample_interval_s) * ...
        sample_interval_s;
    sample_end = floor(end_time / sample_interval_s) * ...
        sample_interval_s;
    sample_time = (sample_start:sample_interval_s:sample_end)';
    if numel(sample_time) < 2
        warning('导航文件有效重叠时间不足，跳过：%s', file_path);
        return;
    end

    nav_value = interp1(nav_time, nav(:, 3:11), sample_time, 'linear');
    truth_value = interp1(truth_time, truth(:, 3:11), ...
        sample_time, 'linear');
    valid = all(isfinite(nav_value), 2) & all(isfinite(truth_value), 2);
    sample_time = sample_time(valid);
    nav_value = nav_value(valid, :);
    truth_value = truth_value(valid, :);
    if isempty(sample_time)
        return;
    end

    latitude_rad = deg2rad(truth_value(:, 1));
    height_m = truth_value(:, 3);
    earth_a = 6378137;
    earth_e2 = 6.69437999014e-3;
    denominator = sqrt(1 - earth_e2 .* sin(latitude_rad) .^ 2);
    rn = earth_a ./ denominator;
    rm = earth_a * (1 - earth_e2) ./ denominator .^ 3;
    north_error_m = deg2rad(nav_value(:, 1) - truth_value(:, 1)) .* ...
        (rm + height_m);
    east_error_m = deg2rad(nav_value(:, 2) - truth_value(:, 2)) .* ...
        (rn + height_m) .* cos(latitude_rad);
    height_error_m = nav_value(:, 3) - truth_value(:, 3);
    horizontal_error_m = hypot(north_error_m, east_error_m);
    velocity_error_mps = nav_value(:, 4:6) - truth_value(:, 4:6);
    speed_error_mps = sqrt(sum(velocity_error_mps .^ 2, 2));
    yaw_error_deg = wrap_to_180(nav_value(:, 9) - truth_value(:, 9));

    common_mask = false(size(sample_time));
    if isfinite(common_start_time) && isfinite(common_end_time)
        common_mask = sample_time >= common_start_time & ...
            sample_time <= common_end_time;
    end

    [~, file_name, extension] = fileparts(file_path);
    row.DataSource = string(data_source);
    row.CaseName = string(case_name);
    row.PhaseStdDeg = metadata.PhaseStdDeg;
    row.MeasurementMode = metadata.MeasurementMode;
    row.WeightMode = metadata.WeightMode;
    row.Stage = metadata.Stage;
    row.DropIndex = metadata.DropIndex;
    row.ResultLabel = metadata.ResultLabel;
    row.FileName = string(file_name) + string(extension);
    row.FilePath = string(file_path);
    row.StartTimeS = sample_time(1);
    row.EndTimeS = sample_time(end);
    row.DurationS = sample_time(end) - sample_time(1);
    row.SampleCount = numel(sample_time);
    row.FullNorthRMSEM = root_mean_square(north_error_m);
    row.FullEastRMSEM = root_mean_square(east_error_m);
    row.FullHorizontalRMSEM = root_mean_square(horizontal_error_m);
    row.FullHorizontalMeanM = mean(horizontal_error_m);
    row.FullHorizontalP95M = percentile_value(horizontal_error_m, 95);
    row.FullHorizontalMaxM = max(horizontal_error_m);
    row.FullHorizontalFinalM = horizontal_error_m(end);
    row.FullHeightRMSEM = root_mean_square(height_error_m);
    row.FullSpeedRMSEMps = root_mean_square(speed_error_mps);
    row.FullYawRMSEDeg = root_mean_square(yaw_error_deg);
    row.FullYawMAEDeg = mean(abs(yaw_error_deg));
    row.FullYawFinalDeg = yaw_error_deg(end);
    if any(common_mask)
        row.CommonStartTimeS = sample_time(find(common_mask, 1, 'first'));
        row.CommonEndTimeS = sample_time(find(common_mask, 1, 'last'));
        row.CommonSampleCount = nnz(common_mask);
        row.CommonHorizontalRMSEM = ...
            root_mean_square(horizontal_error_m(common_mask));
        row.CommonHorizontalMeanM = mean(horizontal_error_m(common_mask));
        row.CommonHorizontalP95M = ...
            percentile_value(horizontal_error_m(common_mask), 95);
        row.CommonHorizontalMaxM = max(horizontal_error_m(common_mask));
        row.CommonHeightRMSEM = ...
            root_mean_square(height_error_m(common_mask));
        row.CommonSpeedRMSEMps = ...
            root_mean_square(speed_error_mps(common_mask));
        row.CommonYawRMSEDeg = ...
            root_mean_square(yaw_error_deg(common_mask));
    else
        row.CommonStartTimeS = nan;
        row.CommonEndTimeS = nan;
        row.CommonSampleCount = 0;
        row.CommonHorizontalRMSEM = nan;
        row.CommonHorizontalMeanM = nan;
        row.CommonHorizontalP95M = nan;
        row.CommonHorizontalMaxM = nan;
        row.CommonHeightRMSEM = nan;
        row.CommonSpeedRMSEMps = nan;
        row.CommonYawRMSEDeg = nan;
    end

    reference_lat_rad = deg2rad(reference_position(1));
    reference_height_m = reference_position(3);
    reference_denominator = sqrt(1 - earth_e2 * ...
        sin(reference_lat_rad) ^ 2);
    reference_rn = earth_a / reference_denominator;
    reference_rm = earth_a * (1 - earth_e2) / ...
        reference_denominator ^ 3;
    nav_north_m = deg2rad(nav_value(:, 1) - reference_position(1)) * ...
        (reference_rm + reference_height_m);
    nav_east_m = deg2rad(nav_value(:, 2) - reference_position(2)) * ...
        (reference_rn + reference_height_m) * cos(reference_lat_rad);
    truth_north_m = deg2rad(truth_value(:, 1) - reference_position(1)) * ...
        (reference_rm + reference_height_m);
    truth_east_m = deg2rad(truth_value(:, 2) - reference_position(2)) * ...
        (reference_rn + reference_height_m) * cos(reference_lat_rad);

    trace.ResultLabel = metadata.ResultLabel;
    trace.MeasurementMode = metadata.MeasurementMode;
    trace.WeightMode = metadata.WeightMode;
    trace.Stage = metadata.Stage;
    trace.DropIndex = metadata.DropIndex;
    trace.PhaseStdDeg = metadata.PhaseStdDeg;
    trace.SampleTimeS = sample_time;
    trace.TimeHours = (sample_time - sample_time(1)) / 3600;
    trace.HorizontalErrorM = horizontal_error_m;
    trace.NavNorthM = nav_north_m;
    trace.NavEastM = nav_east_m;
    trace.TruthNorthM = truth_north_m;
    trace.TruthEastM = truth_east_m;
end

function rows = collect_measurement_summaries(navigation_root, ...
        data_source, case_name)
    rows = struct([]);
    files = dir(fullfile(navigation_root, ...
        'double-hydrophone-phasestd*deg', ...
        'range-phase-azimuth-measurement-accuracy*.txt'));
    for index = 1:numel(files)
        row = struct();
        file_path = fullfile(files(index).folder, files(index).name);
        phase_token = regexp(files(index).folder, ...
            'double-hydrophone-phasestd([0-9.]+)deg', ...
            'tokens', 'once');
        summary_values = read_all_summary_line(file_path);
        if numel(summary_values) < 14 || isempty(phase_token)
            warning('无法读取量测汇总：%s', file_path);
            continue;
        end
        row.DataSource = string(data_source);
        row.CaseName = string(case_name);
        row.PhaseStdDeg = str2double(phase_token{1});
        if contains(files(index).name, 'adaptive')
            row.WeightMode = "adaptive";
        else
            row.WeightMode = "fixed";
        end
        row.EventCount = summary_values(1);
        row.CandidateCorrectPct = summary_values(2);
        row.AzimuthBiasDeg = summary_values(3);
        row.AzimuthStdDeg = summary_values(4);
        row.AzimuthMAEDeg = summary_values(5);
        row.AzimuthRMSEDeg = summary_values(6);
        row.AzimuthP95AbsDeg = summary_values(7);
        row.AzimuthMaxAbsDeg = summary_values(8);
        row.RangeBiasM = summary_values(9);
        row.RangeStdM = summary_values(10);
        row.RangeMAEM = summary_values(11);
        row.RangeRMSEM = summary_values(12);
        row.RangeP95AbsM = summary_values(13);
        row.RangeMaxAbsM = summary_values(14);
        row.FilePath = string(file_path);
        rows = append_struct_rows(rows, row);
    end
end

function values = read_all_summary_line(file_path)
    values = [];
    file_id = fopen(file_path, 'r');
    if file_id < 0
        return;
    end
    cleanup = onCleanup(@() fclose(file_id));
    while ~feof(file_id)
        line = strtrim(fgetl(file_id));
        if startsWith(line, 'ALL')
            values = sscanf(line(4:end), '%f')';
            return;
        end
    end
end

function rows = collect_diagnostic_summaries(navigation_root, ...
        data_source, case_name)
    rows = struct([]);
    files = dir(fullfile(navigation_root, ...
        'double-hydrophone-phasestd*deg', ...
        'range-phase-azimuth-diagnostic*.txt'));
    for index = 1:numel(files)
        row = struct();
        file_path = fullfile(files(index).folder, files(index).name);
        phase_token = regexp(files(index).folder, ...
            'double-hydrophone-phasestd([0-9.]+)deg', ...
            'tokens', 'once');
        data = readmatrix(file_path, 'FileType', 'text', ...
            'NumHeaderLines', 1);
        if isempty(data) || size(data, 2) < 5 || isempty(phase_token)
            continue;
        end
        difference_deg = data(:, 5);
        difference_deg = difference_deg(isfinite(difference_deg));
        row.DataSource = string(data_source);
        row.CaseName = string(case_name);
        row.PhaseStdDeg = str2double(phase_token{1});
        if contains(files(index).name, 'adaptive')
            row.WeightMode = "adaptive";
        else
            row.WeightMode = "fixed";
        end
        row.EventCount = numel(difference_deg);
        row.PriorSelectedMeanAbsDeg = mean(abs(difference_deg));
        row.PriorSelectedRMSEDeg = root_mean_square(difference_deg);
        row.PriorSelectedP95AbsDeg = ...
            percentile_value(abs(difference_deg), 95);
        row.PriorSelectedMaxAbsDeg = max(abs(difference_deg));
        if size(data, 2) >= 11
            prior_std_deg = data(:, 6);
            normalized_q = data(:, 7);
            geometry_r_ratio = data(:, 8);
            activation_weight = data(:, 9);
            effective_r_ratio = data(:, 10);
            adaptive_std_deg = data(:, 11);
            row.PriorStdMeanDeg = mean(prior_std_deg, 'omitnan');
            row.NormalizedInnovationMean = mean(normalized_q, 'omitnan');
            row.NormalizedInnovationMax = max(normalized_q, [], 'omitnan');
            row.GeometryRRatioMean = mean(geometry_r_ratio, 'omitnan');
            row.ActivationWeightMean = mean(activation_weight, 'omitnan');
            row.EffectiveRRatioMean = mean(effective_r_ratio, 'omitnan');
            row.EffectiveRRatioMax = max(effective_r_ratio, [], 'omitnan');
        elseif size(data, 2) >= 6
            adaptive_std_deg = data(:, 6);
            row.PriorStdMeanDeg = nan;
            row.NormalizedInnovationMean = nan;
            row.NormalizedInnovationMax = nan;
            row.GeometryRRatioMean = nan;
            row.ActivationWeightMean = nan;
            row.EffectiveRRatioMean = nan;
            row.EffectiveRRatioMax = nan;
        else
            adaptive_std_deg = [];
            row.PriorStdMeanDeg = nan;
            row.NormalizedInnovationMean = nan;
            row.NormalizedInnovationMax = nan;
            row.GeometryRRatioMean = nan;
            row.ActivationWeightMean = nan;
            row.EffectiveRRatioMean = nan;
            row.EffectiveRRatioMax = nan;
        end
        if ~isempty(adaptive_std_deg)
            adaptive_std_deg = adaptive_std_deg(isfinite(adaptive_std_deg));
            row.AdaptiveStdMeanDeg = mean(adaptive_std_deg);
            row.AdaptiveStdMinDeg = min(adaptive_std_deg);
            row.AdaptiveStdMaxDeg = max(adaptive_std_deg);
        else
            row.AdaptiveStdMeanDeg = nan;
            row.AdaptiveStdMinDeg = nan;
            row.AdaptiveStdMaxDeg = nan;
        end
        row.FilePath = string(file_path);
        rows = append_struct_rows(rows, row);
    end
end

function create_case_figures(case_table, traces, output_dir, ...
        data_source, case_name, options)
    visibility = 'off';
    if options.show_figures
        visibility = 'on';
    end
    title_prefix = sprintf('%s / %s', data_source, case_name);

    non_dropout = find(~isfinite([traces.DropIndex]));
    if ~isempty(non_dropout)
        stage_order = ["forward", "rts-single", "rts-double"];
        present_stage = false(size(stage_order));
        for index = 1:numel(stage_order)
            present_stage(index) = any(arrayfun(@(item) ...
                item.Stage == stage_order(index), traces(non_dropout)));
        end
        stage_order = stage_order(present_stage);
        if ~isempty(stage_order)
            figure_handle = figure('Visible', visibility, ...
                'Color', 'w', 'Position', [100, 100, 1200, 350 * numel(stage_order)]);
            layout = tiledlayout(numel(stage_order), 1, ...
                'TileSpacing', 'compact', 'Padding', 'compact');
            for stage_index = 1:numel(stage_order)
                nexttile;
                hold on;
                selected = non_dropout(arrayfun(@(item) ...
                    item.Stage == stage_order(stage_index), ...
                    traces(non_dropout)));
                colors = lines(max(1, numel(selected)));
                for trace_index = 1:numel(selected)
                    item = traces(selected(trace_index));
                    plot(item.TimeHours, item.HorizontalErrorM, ...
                        'LineWidth', 1.1, 'Color', colors(trace_index, :), ...
                        'DisplayName', char(short_trace_label(item)));
                end
                grid on;
                xlabel('Time (h)');
                ylabel('Horizontal error (m)');
                title(char(stage_order(stage_index)), 'Interpreter', 'none');
                legend('Location', 'best', 'Interpreter', 'none');
            end
            title(layout, sprintf('%s: non-drop horizontal error', ...
                title_prefix), 'Interpreter', 'none');
            save_and_close_figure(figure_handle, fullfile(output_dir, ...
                'non-drop-horizontal-error.png'), options);
        end

        selected = non_dropout(arrayfun(@(item) ...
            item.Stage == "rts-double", traces(non_dropout)));
        if ~isempty(selected)
            figure_handle = figure('Visible', visibility, ...
                'Color', 'w', 'Position', [100, 100, 900, 700]);
            hold on;
            reference_trace = traces(selected(1));
            plot(reference_trace.TruthEastM, reference_trace.TruthNorthM, ...
                'k-', 'LineWidth', 2.0, 'DisplayName', 'truth');
            colors = lines(numel(selected));
            for trace_index = 1:numel(selected)
                item = traces(selected(trace_index));
                plot(item.NavEastM, item.NavNorthM, 'LineWidth', 1.1, ...
                    'Color', colors(trace_index, :), ...
                    'DisplayName', char(short_trace_label(item)));
            end
            axis equal;
            grid on;
            xlabel('East (m)');
            ylabel('North (m)');
            title(sprintf('%s: double RTS trajectory', title_prefix), ...
                'Interpreter', 'none');
            legend('Location', 'best', 'Interpreter', 'none');
            save_and_close_figure(figure_handle, fullfile(output_dir, ...
                'non-drop-double-rts-trajectory.png'), options);
        end

        row_mask = ~isfinite(case_table.DropIndex);
        if any(row_mask)
            plot_table = case_table(row_mask, :);
            figure_handle = figure('Visible', visibility, ...
                'Color', 'w', 'Position', [100, 100, 1100, 650]);
            values = plot_table.NoDropCommonHorizontalRMSEM;
            if all(~isfinite(values))
                values = plot_table.FullHorizontalRMSEM;
            end
            labels = plot_table.ResultLabel;
            bar(1:numel(values), values);
            grid on;
            ylabel('Horizontal RMSE (m)');
            title(sprintf('%s: non-drop navigation RMSE', title_prefix), ...
                'Interpreter', 'none');
            set(gca, 'TickLabelInterpreter', 'none');
            xticks(1:numel(values));
            xticklabels(labels);
            xtickangle(30);
            save_and_close_figure(figure_handle, fullfile(output_dir, ...
                'non-drop-navigation-rmse.png'), options);
        end
    end

    dropout = find(isfinite([traces.DropIndex]));
    if ~isempty(dropout)
        group_labels = strings(numel(dropout), 1);
        for index = 1:numel(dropout)
            item = traces(dropout(index));
            group_labels(index) = short_dropout_group_label(item);
        end
        unique_groups = unique(group_labels, 'stable');
        figure_handle = figure('Visible', visibility, ...
            'Color', 'w', 'Position', [100, 100, 1000, 650]);
        hold on;
        colors = lines(numel(unique_groups));
        for group_index = 1:numel(unique_groups)
            selected = dropout(group_labels == unique_groups(group_index));
            drop_index = [traces(selected).DropIndex]';
            table_rows = nan(size(drop_index));
            for item_index = 1:numel(selected)
                label = traces(selected(item_index)).ResultLabel;
                match = case_table.ResultLabel == label;
                metric = case_table.CommonHorizontalRMSEM(match);
                if isempty(metric) || ~isfinite(metric(1))
                    metric = case_table.FullHorizontalRMSEM(match);
                end
                table_rows(item_index) = metric(1);
            end
            [drop_index, order] = sort(drop_index);
            table_rows = table_rows(order);
            plot(drop_index, table_rows, '-o', 'LineWidth', 1.4, ...
                'MarkerSize', 6, 'Color', colors(group_index, :), ...
                'DisplayName', char(unique_groups(group_index)));
        end
        grid on;
        xlabel('Dropped measurement index');
        ylabel('Common-window horizontal RMSE (m)');
        title(sprintf('%s: dropout comparison', title_prefix), ...
            'Interpreter', 'none');
        legend('Location', 'best', 'Interpreter', 'none');
        save_and_close_figure(figure_handle, fullfile(output_dir, ...
            'dropout-horizontal-rmse.png'), options);
    end
end

function label = short_trace_label(trace)
    if isfinite(trace.PhaseStdDeg)
        phase_label = string(trace.PhaseStdDeg) + "deg";
    else
        phase_label = "phase-na";
    end
    label = strjoin([trace.MeasurementMode, trace.WeightMode, ...
        phase_label], " | ");
end

function label = short_dropout_group_label(trace)
    if trace.MeasurementMode == "range-only"
        label = "range-only";
    else
        label = trace.MeasurementMode + " | " + trace.WeightMode + ...
            " | " + string(trace.PhaseStdDeg) + "deg";
    end
end

function save_and_close_figure(figure_handle, file_path, options)
    if options.save_figures
        exportgraphics(figure_handle, file_path, 'Resolution', 220);
    end
    if ~options.show_figures
        close(figure_handle);
    end
end

function output = append_struct_rows(output, input)
    if isempty(input)
        return;
    end
    if isempty(output)
        output = input;
    else
        input = orderfields(input, output(1));
        output(end + 1:end + numel(input)) = input;
    end
end

function value = root_mean_square(data)
    data = data(isfinite(data));
    if isempty(data)
        value = nan;
    else
        value = sqrt(mean(data .^ 2));
    end
end

function value = percentile_value(data, percentile)
    data = sort(data(isfinite(data)));
    if isempty(data)
        value = nan;
        return;
    end
    if numel(data) == 1
        value = data(1);
        return;
    end
    position = 1 + (numel(data) - 1) * percentile / 100;
    lower_index = floor(position);
    upper_index = ceil(position);
    fraction = position - lower_index;
    value = data(lower_index) * (1 - fraction) + ...
        data(upper_index) * fraction;
end

function angle_deg = wrap_to_180(angle_deg)
    angle_deg = mod(angle_deg + 180, 360) - 180;
end
