function manifest = process_beaconpos(data_source, simulation_case, ...
        error_enu_m, scenario_ids, range_interval_s, random_seed, ...
        simulation_range_noise_std_m)
%PROCESS_BEACONPOS 为仿真或实测数据批量构造潜标位置误差工况。
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));
addpath(topic_dir);
setup_inertial_experiment();
if nargin < 1 || isempty(data_source), data_source = "simulation"; end
if nargin < 2 || isempty(simulation_case), simulation_case = 'case-00'; end
if nargin < 3 || isempty(error_enu_m)
    error_enu_m = [0,0,0; 10,10,0.4; 20,20,0.4; ...
        50,50,0.4; 100,100,0.4; 200,200,1.0];
end
if nargin < 5 || isempty(range_interval_s), range_interval_s = 420; end
if nargin < 6 || isempty(random_seed), random_seed = 1; end
if nargin < 7 || isempty(simulation_range_noise_std_m)
    simulation_range_noise_std_m = 10;
end
if size(error_enu_m, 2) ~= 3 || any(~isfinite(error_enu_m), 'all')
    error('error_enu_m 必须是 N×3 的有限矩阵。');
end

dataset = load_engineering_problem_dataset(data_source, simulation_case, ...
    range_interval_s, random_seed, simulation_range_noise_std_m);
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

paths = setup_inertial_experiment();
directories = engineering_problem_directories(paths, data_source, ...
    simulation_case, 'beacon-position-error', 'rad');
range_file_path = strings(scenario_count, 1);
for index = 1:scenario_count
    scenario_dir = fullfile(directories.input_root, scenario_ids(index));
    if ~isfolder(scenario_dir), mkdir(scenario_dir); end
    rangedata = apply_beacon_position_offset( ...
        dataset.current_range, error_enu_m(index, :));
    range_file_path(index) = fullfile(scenario_dir, 'rangedata.txt');
    writematrix(rangedata, range_file_path(index), 'Delimiter', ' ');
end
manifest = table(repmat(dataset.data_source, scenario_count, 1), ...
    repmat(dataset.dataset_id, scenario_count, 1), scenario_ids, ...
    error_enu_m(:, 1), error_enu_m(:, 2), error_enu_m(:, 3), ...
    vecnorm(error_enu_m(:, 1:2), 2, 2), range_file_path, ...
    'VariableNames', {'DataSource', 'DatasetId', 'ScenarioId', ...
    'EastError_m', 'NorthError_m', 'UpError_m', ...
    'HorizontalError_m', 'RangeFilePath'});
writetable(manifest, fullfile(directories.input_root, 'input-manifest.csv'));
fprintf('已生成 %s/%s 的 %d 组潜标误差输入：%s\n', ...
    dataset.data_source, dataset.dataset_id, scenario_count, ...
    directories.input_root);
end

function scenario_id = format_scenario_id(offset)
%FORMAT_SCENARIO_ID 将ENU误差编码为稳定目录名。
    if all(abs(offset) < 1e-12), scenario_id = "nominal"; return; end
    values = string(compose('%+g', offset));
    values = replace(values, '+', 'p');
    values = replace(values, '-', 'm');
    values = replace(values, '.', 'p');
    scenario_id = "e"+values(1)+"-n"+values(2)+"-u"+values(3);
end
