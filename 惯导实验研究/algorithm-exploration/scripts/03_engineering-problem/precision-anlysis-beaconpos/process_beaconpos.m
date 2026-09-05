function [manifest, generation_context] = process_beaconpos( ...
        data_source, simulation_case, error_enu_m, scenario_ids, ...
        range_interval_s, random_seed, range_noise_std_m, depth_noise_std_m)
%PROCESS_BEACONPOS 构造潜标位置误差专题所需的可复现实验输入。
%   该函数只负责生成数据，不执行导航。仿真和实测均从三个原始距离
%   文件抽取固定间隔量测，按 1->2->3 潜标轮换，并且所有误差工况
%   共用完全相同的距离噪声和深度噪声。

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));
addpath(topic_dir);
paths = setup_inertial_experiment();

if nargin < 1 || isempty(data_source), data_source = "experiment"; end
if nargin < 2 || isempty(simulation_case), simulation_case = 'case-00'; end
if nargin < 3 || isempty(error_enu_m)
    error_enu_m = [0,0,0; 10,10,0.4; 20,20,0.4; ...
        50,50,0.4; 100,100,0.4; 200,200,1.0];
end
if nargin < 5 || isempty(range_interval_s), range_interval_s = 420; end
if nargin < 6 || isempty(random_seed), random_seed = 1; end
if nargin < 7 || isempty(range_noise_std_m), range_noise_std_m = 6; end
if nargin < 8 || isempty(depth_noise_std_m), depth_noise_std_m = 0.4; end

data_source = lower(string(data_source));
simulation_case = char(string(simulation_case));
if ~ismember(data_source, ["simulation", "experiment"])
    error('data_source 只能设置为 "simulation" 或 "experiment"。');
end
if size(error_enu_m, 2) ~= 3 || any(~isfinite(error_enu_m), 'all')
    error('error_enu_m 必须是 N×3 的有限矩阵。');
end
if range_interval_s <= 0 || range_noise_std_m < 0 || depth_noise_std_m < 0
    error('采样间隔必须为正数，距离和深度噪声标准差不能为负数。');
end

if data_source == "simulation"
    case_id = parse_case_id(simulation_case);
    input_dir = paths.simulation_input(case_id);
    cfg = load_algorithm_exploration_config( ...
        "simulation", "rad", input_dir);
    dataset_id = string(simulation_case);
else
    cfg = load_algorithm_exploration_config("experiment", "rad", []);
    dataset_id = "experiment";
end

range_sources = {
    readmatrix(cfg.rangefile1path, 'FileType', 'text'), ...
    readmatrix(cfg.rangefile2path, 'FileType', 'text'), ...
    readmatrix(cfg.rangefile3path, 'FileType', 'text')};
source_interval_s = median(diff(range_sources{1}(:, 1)));
range_stride = round(range_interval_s/source_interval_s);
if range_stride < 1 || ...
        abs(range_stride*source_interval_s-range_interval_s) > 1e-6
    error('测距间隔 %.3f s 不是原始采样间隔 %.3f s 的整数倍。', ...
        range_interval_s, source_interval_s);
end
for source_index = 1:numel(range_sources)
    current_interval = median(diff(range_sources{source_index}(:, 1)));
    if abs(current_interval-source_interval_s) > 1e-8
        error('三个原始距离文件的采样间隔不一致。');
    end
    range_sources{source_index} = ...
        range_sources{source_index}(range_stride:range_stride:end, :);
end

event_count = min(cellfun(@(data) size(data, 1), range_sources));
base_range = zeros(event_count, size(range_sources{1}, 2));
beacon_order = [1, 2, 3];
for event_index = 1:event_count
    order_index = mod(event_index-1, numel(beacon_order))+1;
    source_index = beacon_order(order_index);
    base_range(event_index, :) = range_sources{source_index}(event_index, :);
end

% 先生成一次基础噪声，再复制到全部潜标误差工况，保证控制变量一致。
rng(random_seed, 'twister');
base_range(:, 3) = base_range(:, 3) + ...
    range_noise_std_m*randn(event_count, 1);
imudata = readmatrix(cfg.imufilepath, 'FileType', 'text');
truth = readmatrix(cfg.truthpath, 'FileType', 'text');
height_value = interp1(truth(:, 2), truth(:, 5), imudata(:, 1), ...
    'linear', 'extrap');
heightdata = [imudata(:, 1), height_value + ...
    depth_noise_std_m*randn(size(height_value))];

scenario_count = size(error_enu_m, 1);
if nargin < 4 || isempty(scenario_ids)
    scenario_ids = strings(scenario_count, 1);
    for index = 1:scenario_count
        scenario_ids(index) = format_scenario_id(error_enu_m(index, :));
    end
else
    scenario_ids = string(scenario_ids(:));
end
if numel(scenario_ids) ~= scenario_count || ...
        numel(unique(scenario_ids)) ~= scenario_count
    error('scenario_ids 必须与误差工况数一致且不能重复。');
end

directories = engineering_problem_directories(paths, data_source, ...
    simulation_case, 'beacon-position-error', 'rad');
height_file_path = fullfile(directories.input_root, 'heightdata.txt');
writematrix(heightdata, height_file_path, 'Delimiter', ' ');

range_file_path = strings(scenario_count, 1);
for index = 1:scenario_count
    scenario_dir = fullfile(directories.input_root, scenario_ids(index));
    if ~isfolder(scenario_dir), mkdir(scenario_dir); end
    rangedata = apply_beacon_position_offset( ...
        base_range, error_enu_m(index, :));
    range_file_path(index) = fullfile(scenario_dir, 'rangedata.txt');
    writematrix(rangedata, range_file_path(index), 'Delimiter', ' ');
end

manifest = table(repmat(data_source, scenario_count, 1), ...
    repmat(dataset_id, scenario_count, 1), scenario_ids, ...
    error_enu_m(:, 1), error_enu_m(:, 2), error_enu_m(:, 3), ...
    vecnorm(error_enu_m(:, 1:2), 2, 2), range_file_path, ...
    repmat(string(height_file_path), scenario_count, 1), ...
    'VariableNames', {'DataSource', 'DatasetId', 'ScenarioId', ...
    'EastError_m', 'NorthError_m', 'UpError_m', ...
    'HorizontalError_m', 'RangeFilePath', 'HeightFilePath'});
manifest_path = fullfile(directories.input_root, 'input-manifest.csv');
writetable(manifest, manifest_path, 'Delimiter', ',');

generation_context = struct();
generation_context.version = 2;
generation_context.data_source = char(data_source);
generation_context.dataset_id = char(dataset_id);
generation_context.simulation_case = simulation_case;
generation_context.range_interval_s = range_interval_s;
generation_context.beacon_order = beacon_order;
generation_context.random_seed = random_seed;
generation_context.range_noise_std_m = range_noise_std_m;
generation_context.depth_noise_std_m = depth_noise_std_m;
generation_context.manifest_path = manifest_path;
generation_context.height_file_path = height_file_path;
generation_context.truth_path = cfg.truthpath;
generation_context.imu_path = cfg.imufilepath;
save(fullfile(directories.input_root, 'generation-context.mat'), ...
    'generation_context');

fprintf('已生成 %s/%s 的 %d 组潜标位置误差输入：%s\n', ...
    data_source, dataset_id, scenario_count, directories.input_root);
end

function scenario_id = format_scenario_id(offset)
%FORMAT_SCENARIO_ID 将 ENU 误差编码为稳定目录名。
    if all(abs(offset) < 1e-12)
        scenario_id = "nominal";
        return;
    end
    values = string(compose('%+g', offset));
    values = replace(values, '+', 'p');
    values = replace(values, '-', 'm');
    values = replace(values, '.', 'p');
    scenario_id = "e"+values(1)+"-n"+values(2)+"-u"+values(3);
end

function case_id = parse_case_id(case_name)
%PARSE_CASE_ID 从 case-00 一类名称中提取数据编号。
    token = regexp(char(case_name), '^case-(\d+)$', 'tokens', 'once');
    if isempty(token)
        error('simulation_case 必须采用 case-00 一类格式。');
    end
    case_id = str2double(token{1});
end
