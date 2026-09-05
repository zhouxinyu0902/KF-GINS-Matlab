function dataset = load_engineering_problem_dataset( ...
        data_source, simulation_case, range_interval_s, ...
        random_seed, simulation_base_range_noise_std_m)
%LOAD_ENGINEERING_PROBLEM_DATASET 统一读取仿真或实测工程问题输入。
%   输出的 current_range 均为已按测距时刻和 1→2→3 次序整理好的 N×6
%   数据，可直接交给仿真或实测导航核心。

if nargin < 1 || isempty(data_source), data_source = "simulation"; end
if nargin < 2 || isempty(simulation_case), simulation_case = 'case-00'; end
if nargin < 3 || isempty(range_interval_s), range_interval_s = 420; end
if nargin < 4 || isempty(random_seed), random_seed = 1; end
if nargin < 5 || isempty(simulation_base_range_noise_std_m)
    simulation_base_range_noise_std_m = 10;
end
data_source = lower(string(data_source));
if ~ismember(data_source, ["simulation", "experiment"])
    error('data_source 只能设置为 "simulation" 或 "experiment"。');
end

paths = setup_inertial_experiment();
simulation_case = char(string(simulation_case));
raw_range = cell(3, 1);
if data_source == "simulation"
    input_dir = fullfile(paths.simulation_input, simulation_case);
    cfg = load_algorithm_exploration_config( ...
        "simulation", "rad", input_dir);
    for beacon_index = 1:3
        raw_path = fullfile(input_dir, sprintf('range%d.txt', beacon_index));
        if ~isfile(raw_path), error('缺少仿真测距文件：%s', raw_path); end
        raw_range{beacon_index} = readmatrix(raw_path, 'FileType', 'text');
    end
    source_interval_s = median(diff(raw_range{1}(:, 1)));
    stride = round(range_interval_s/source_interval_s);
    if abs(stride*source_interval_s-range_interval_s) > 1e-6
        error('测距间隔 %.3f s 不是仿真源间隔 %.3f s 的整数倍。', ...
            range_interval_s, source_interval_s);
    end
    sampled = cellfun(@(data) data(stride:stride:end, :), raw_range, ...
        'UniformOutput', false);
    event_count = min(cellfun(@(data) size(data, 1), sampled));
    current_range = zeros(event_count, size(sampled{1}, 2));
    for event_index = 1:event_count
        beacon_index = mod(event_index-1, 3)+1;
        current_range(event_index, :) = sampled{beacon_index}(event_index, :);
    end
    rng(random_seed, 'twister');
    current_range(:, 3) = current_range(:, 3) + ...
        simulation_base_range_noise_std_m*randn(event_count, 1);
    dataset_id = string(simulation_case);
else
    input_dir = paths.experiment_input;
    cfg = load_algorithm_exploration_config("experiment", "rad", []);
    current_path = fullfile(input_dir, 'rangedata_noised.txt');
    if ~isfile(current_path), error('缺少实测测距文件：%s', current_path); end
    current_range = readmatrix(current_path, 'FileType', 'text');
    for beacon_index = 1:3
        raw_path = fullfile(input_dir, sprintf('range%d.txt', beacon_index));
        if ~isfile(raw_path), error('缺少实测原始距离文件：%s', raw_path); end
        raw_range{beacon_index} = readmatrix(raw_path, 'FileType', 'text');
    end
    if size(current_range, 1) > 1 && ...
            any(abs(diff(current_range(:, 1))-range_interval_s) > 1e-6)
        error('实测测距数据不是固定 %.0f s 间隔。', range_interval_s);
    end
    dataset_id = "experiment";
end

if isempty(current_range) || size(current_range, 2) < 6 || ...
        any(diff(current_range(:, 1)) <= 0)
    error('%s测距数据必须是时间严格递增的非空 N×6 矩阵。', data_source);
end
dataset = struct();
dataset.data_source = data_source;
dataset.dataset_id = dataset_id;
dataset.simulation_case = simulation_case;
dataset.input_dir = input_dir;
dataset.cfg = cfg;
dataset.truth_path = cfg.truthpath;
dataset.current_range = current_range;
dataset.raw_range = raw_range;
dataset.range_interval_s = range_interval_s;
end
