clear;
clc;

%% 测距误差对导航的影响：仿真/实测统一入口
data_source = "simulation";         % "simulation" 或 "experiment"
simulation_case = 'case-00';
position_error_unit = "rad";        % 实测当前使用 rad
injected_range_noise_std_m = [0; 5; 10; 20; 50];
range_interval_s = 420;
duration_s = 4621;
random_seed = 1;

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));
addpath(topic_dir);
paths = setup_inertial_experiment();
data_source = lower(string(data_source));
position_error_unit = lower(string(position_error_unit));
% 仿真从无附加距离噪声开始；实测以预处理后的实际距离为基准。
dataset = load_engineering_problem_dataset(data_source, simulation_case, ...
    range_interval_s, random_seed, 0);
directories = engineering_problem_directories(paths, data_source, ...
    simulation_case, 'range-measurement-error', position_error_unit);
filter_range_std_m = 10;
if data_source == "experiment", filter_range_std_m = 6; end

rng(random_seed, 'twister');
shared_standard_noise = randn(size(dataset.current_range, 1), 1);
scenario_count = numel(injected_range_noise_std_m);
scenario_ids = string(compose('sigma-%gm', injected_range_noise_std_m));
scenario_ids = replace(scenario_ids, '.', 'p');
output_dirs = strings(scenario_count, 1);
valid_masks = cell(scenario_count, 3);
for index = 1:scenario_count
    rangedata = dataset.current_range;
    rangedata(:, 3) = rangedata(:, 3) + ...
        injected_range_noise_std_m(index)*shared_standard_noise;
    input_dir = fullfile(directories.input_root, scenario_ids(index));
    if ~isfolder(input_dir), mkdir(input_dir); end
    writematrix(rangedata, fullfile(input_dir, 'rangedata.txt'), ...
        'Delimiter', ' ');
    output_dirs(index) = fullfile(directories.result_root, scenario_ids(index));
    options = struct('position_error_unit', position_error_unit, ...
        'duration_s', duration_s, 'filter_range_std_m', ...
        filter_range_std_m, 'random_seed', random_seed, ...
        'case_name', char("range-error-"+scenario_ids(index)));
    outputs = run_engineering_navigation_case( ...
        dataset, output_dirs(index), rangedata, options);
    valid_masks{index, 1} = outputs.forward_mask;
    valid_masks{index, 2} = outputs.single_rts_mask;
    valid_masks{index, 3} = outputs.double_rts_mask;
end

study_context = struct();
study_context.version = 2;
study_context.data_source = char(data_source);
study_context.dataset_id = char(dataset.dataset_id);
study_context.study_id = "range-measurement-error";
study_context.study_title = data_source_name(data_source)+"测距误差敏感性";
study_context.position_error_unit = char(position_error_unit);
study_context.scenario_ids = scenario_ids;
study_context.scenario_names = compose('附加测距误差标准差 %.1f m', ...
    injected_range_noise_std_m);
study_context.parameter_values = injected_range_noise_std_m;
study_context.parameter_name = "InjectedRangeNoiseStd_m";
study_context.parameter_label = "附加测距误差标准差（m）";
study_context.output_dirs = output_dirs;
study_context.truth_path = dataset.truth_path;
study_context.method_ids = ["ekf"; "single-rts"; "double-rts"];
study_context.method_names = ["前向EKF"; "一次RTS"; "二次RTS"];
study_context.method_files = ["range-ins-forward.nav"; ...
    "range-ins-rts-single.nav"; "range-ins-rts-double.nav"];
study_context.valid_masks = valid_masks;
study_context.artifact_root = directories.artifact_root;
save(fullfile(directories.result_root, 'study-context.mat'), 'study_context');
fprintf('测距误差专题完成：%s\n', directories.result_root);

function name = data_source_name(source)
    if source == "simulation", name = "仿真数据："; else, name = "实测数据："; end
end
