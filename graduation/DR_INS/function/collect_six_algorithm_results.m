function [selected, info] = collect_six_algorithm_results(result_root, ...
    target_seed, target_input_id, target_range_beacon_index)
%COLLECT_SIX_ALGORITHM_RESULTS 查找并整理六种DVL/INS实验结果。

% 六种结果固定为：三种组合方式，各自包含15维和17维。
required_type = ["INS_DVL"; "INS_DVL"; "INS_DVL_LBL"; ...
    "INS_DVL_LBL"; "INS_DVL_RANGE"; "INS_DVL_RANGE"];
required_dimension = [15; 17; 15; 17; 15; 17];

files = dir(fullfile(result_root, '**', 'performance_summary.csv'));
if isempty(files)
    error('没有在 %s 下找到 performance_summary.csv。', result_root);
end

rows = cell(0, 1);
for k = 1:numel(files)
    summary_path = fullfile(files(k).folder, files(k).name);
    if contains(lower(summary_path), [filesep, '_smoke'])
        continue;
    end

    opts = detectImportOptions(summary_path, 'VariableNamingRule', 'preserve');
    text_names = intersect({'combination_type', 'trajectory_tag', 'input_id'}, ...
        opts.VariableNames, 'stable');
    if ~isempty(text_names)
        opts = setvartype(opts, text_names, 'string');
    end
    one_row = readtable(summary_path, opts);
    if isempty(one_row)
        continue;
    end
    one_row = one_row(1, :);
    one_row.result_dir = string(files(k).folder);
    one_row.summary_mtime = files(k).datenum;
    rows{end + 1, 1} = one_row; %#ok<AGROW>
end

if isempty(rows)
    error('只找到了空结果或_smoke结果，没有可用于比较的数据。');
end
all_results = vertcat(rows{:});
all_results.combination_type = string(all_results.combination_type);
all_results.input_id = string(all_results.input_id);

if isempty(target_seed)
    seed_values = unique(all_results.measurement_seed);
    seed_coverage = zeros(numel(seed_values), 1);
    seed_newest = zeros(numel(seed_values), 1);
    for k = 1:numel(seed_values)
        mask = all_results.measurement_seed == seed_values(k);
        case_key = all_results.combination_type(mask) + "_" + ...
            string(all_results.dimension(mask));
        seed_coverage(k) = numel(unique(case_key));
        seed_newest(k) = max(all_results.summary_mtime(mask));
    end
    best_coverage = max(seed_coverage);
    candidates = find(seed_coverage == best_coverage);
    [~, newest_index] = max(seed_newest(candidates));
    target_seed = seed_values(candidates(newest_index));
end
all_results = all_results(all_results.measurement_seed == target_seed, :);
if isempty(all_results)
    error('没有找到 measurement_seed=%g 的正式实验结果。', target_seed);
end

if ~isempty(target_range_beacon_index)
    is_range = all_results.combination_type == "INS_DVL_RANGE";
    keep = ~is_range | ...
        all_results.beacon_index == target_range_beacon_index;
    all_results = all_results(keep, :);
end

target_input_id = string(target_input_id);
if strlength(target_input_id) == 0
    input_ids = unique(all_results.input_id);
    coverage = zeros(numel(input_ids), 1);
    newest = zeros(numel(input_ids), 1);
    for k = 1:numel(input_ids)
        mask = all_results.input_id == input_ids(k);
        case_key = all_results.combination_type(mask) + "_" + ...
            string(all_results.dimension(mask));
        coverage(k) = numel(unique(case_key));
        newest(k) = max(all_results.summary_mtime(mask));
    end
    best_coverage = max(coverage);
    candidates = find(coverage == best_coverage);
    [~, newest_index] = max(newest(candidates));
    target_input_id = input_ids(candidates(newest_index));
end
all_results = all_results(all_results.input_id == target_input_id, :);

selected = all_results([] , :);
missing = strings(0, 1);
for k = 1:numel(required_type)
    match = all_results.combination_type == required_type(k) & ...
        all_results.dimension == required_dimension(k);
    matched_rows = all_results(match, :);
    if isempty(matched_rows)
        missing(end + 1, 1) = required_type(k) + "-" + ...
            string(required_dimension(k)) + "维"; %#ok<AGROW>
        continue;
    end
    [~, newest_index] = max(matched_rows.summary_mtime);
    selected = [selected; matched_rows(newest_index, :)]; %#ok<AGROW>
end

if ~isempty(missing)
    error('输入数据 %s 缺少结果：%s。', target_input_id, ...
        strjoin(missing, '、'));
end

for k = 1:height(selected)
    time_series_path = fullfile(selected.result_dir(k), ...
        'navigation_error_timeseries.csv');
    if ~isfile(time_series_path)
        error('缺少误差时序文件：%s', time_series_path);
    end
end

info = struct();
info.measurement_seed = selected.measurement_seed(1);
info.input_id = target_input_id;
info.range_beacon_index = target_range_beacon_index;
info.result_root = result_root;
end
