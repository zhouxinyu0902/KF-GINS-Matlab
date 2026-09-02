clear;
clc;

%% 固定测距延迟敏感性：仿真/实测统一入口
data_source = "simulation";         % "simulation" 或 "experiment"
simulation_case = 'case-00';
position_error_unit = "rad";        % 实测当前使用 rad
delay_s = 4;
range_interval_s = 420;
duration_s = 4621;
random_seed = 1;
simulation_range_noise_std_m = 10;

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));
addpath(topic_dir);
paths = setup_inertial_experiment();
data_source = lower(string(data_source));
position_error_unit = lower(string(position_error_unit));
dataset = load_engineering_problem_dataset(data_source, simulation_case, ...
    range_interval_s, random_seed, simulation_range_noise_std_m);
directories = engineering_problem_directories(paths, data_source, ...
    simulation_case, 'range-delay-sensitivity', position_error_unit);
filter_range_std_m = 10;
if data_source == "experiment", filter_range_std_m = 6; end

[delayed_range, measurement_table] = build_delayed_range_input( ...
    dataset.current_range, dataset.raw_range, delay_s);
range_inputs = {dataset.current_range, delayed_range};
scenario_ids = ["current"; "delayed-"+delay_s+"s"];
scenario_names = ["当前时刻距离"; delay_s+" s前陈旧距离"];
output_dirs = strings(2, 1);
valid_masks = cell(2, 2);
for index = 1:2
    input_dir = fullfile(directories.input_root, scenario_ids(index));
    if ~isfolder(input_dir), mkdir(input_dir); end
    writematrix(range_inputs{index}, fullfile(input_dir, 'rangedata.txt'), ...
        'Delimiter', ' ');
    output_dirs(index) = fullfile(directories.result_root, scenario_ids(index));
    options = struct('position_error_unit', position_error_unit, ...
        'duration_s', duration_s, 'filter_range_std_m', ...
        filter_range_std_m, 'random_seed', random_seed, ...
        'case_name', char("range-delay-"+scenario_ids(index)));
    outputs = run_engineering_navigation_case( ...
        dataset, output_dirs(index), range_inputs{index}, options);
    valid_masks{index, 1} = outputs.forward_mask;
    valid_masks{index, 2} = outputs.double_rts_mask;
end

study_context = struct();
study_context.version = 2;
study_context.data_source = char(data_source);
study_context.dataset_id = char(dataset.dataset_id);
study_context.study_id = "range-delay-sensitivity";
study_context.study_title = data_source_name(data_source)+ ...
    "固定"+delay_s+" s测距延迟敏感性";
study_context.position_error_unit = char(position_error_unit);
study_context.scenario_ids = scenario_ids;
study_context.scenario_names = scenario_names;
study_context.parameter_values = [0; delay_s];
study_context.parameter_name = "RangeDelay_s";
study_context.parameter_label = "测距延迟（s）";
study_context.output_dirs = output_dirs;
study_context.truth_path = dataset.truth_path;
study_context.method_ids = ["ekf"; "double-rts"];
study_context.method_names = ["前向EKF"; "二次RTS"];
study_context.method_files = ["range-ins-forward.nav"; ...
    "range-ins-rts-double.nav"];
study_context.valid_masks = valid_masks;
study_context.measurement_table = measurement_table;
study_context.range_interval_s = range_interval_s;
study_context.delay_s = delay_s;
study_context.artifact_root = directories.artifact_root;
save(fullfile(directories.result_root, 'study-context.mat'), 'study_context');
fprintf('测距延迟敏感性计算完成：%s\n', directories.result_root);

function name = data_source_name(source)
    if source == "simulation", name = "仿真数据："; else, name = "实测数据："; end
end
