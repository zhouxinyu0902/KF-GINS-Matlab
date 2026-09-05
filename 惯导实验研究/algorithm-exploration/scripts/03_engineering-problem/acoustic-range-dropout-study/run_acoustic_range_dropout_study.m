clear;
clc;

%% 声学测距掉帧敏感性：仿真/实测统一入口
% 在相同距离噪声与导航配置下，比较完整测距和指定声学包丢失后的
% 前向 EKF、一次 RTS 与二次 RTS。默认使用本专题实测 case-06。
data_source = "experiment";         % "simulation" 或 "experiment"
simulation_case = 'case-00';
position_error_unit = "rad";        % 实测当前使用 rad
drop_measurement_indices = [1, 3, 5, 7, 9, 11];
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
if size(dataset.current_range, 1) < max(drop_measurement_indices)
    error(['当前测距数据只有 %d 个观测，无法构造“丢失第 %d 个包”', ...
        '的对比场景。'], size(dataset.current_range, 1), ...
        max(drop_measurement_indices));
end

directories = engineering_problem_directories(paths, data_source, ...
    simulation_case, 'acoustic-range-dropout', position_error_unit);
filter_range_std_m = 10;
if data_source == "experiment"
    filter_range_std_m = 6;
end

scenario_ids = ["baseline"; ...
    compose("drop-%02d", drop_measurement_indices(:))];
scenario_names = ["完整声学测距"; ...
    compose("丢失第 %d 个声学测距包", drop_measurement_indices(:))];
parameter_values = [0; drop_measurement_indices(:)];
scenario_count = numel(scenario_ids);
output_dirs = strings(scenario_count, 1);
valid_masks = cell(scenario_count, 3);

for scenario_index = 1:scenario_count
    rangedata = dataset.current_range;
    if scenario_index > 1
        rangedata(drop_measurement_indices(scenario_index - 1), :) = [];
    end

    scenario_input_dir = fullfile( ...
        directories.input_root, scenario_ids(scenario_index));
    if ~isfolder(scenario_input_dir)
        mkdir(scenario_input_dir);
    end
    writematrix(rangedata, fullfile( ...
        scenario_input_dir, 'rangedata.txt'), 'Delimiter', ' ');

    output_dirs(scenario_index) = fullfile( ...
        directories.result_root, scenario_ids(scenario_index));
    options = struct( ...
        'position_error_unit', position_error_unit, ...
        'duration_s', duration_s, ...
        'filter_range_std_m', filter_range_std_m, ...
        'random_seed', random_seed, ...
        'case_name', char("acoustic-dropout-" + ...
            scenario_ids(scenario_index)));
    outputs = run_engineering_navigation_case(dataset, ...
        output_dirs(scenario_index), rangedata, options);
    valid_masks{scenario_index, 1} = outputs.forward_mask;
    valid_masks{scenario_index, 2} = outputs.single_rts_mask;
    valid_masks{scenario_index, 3} = outputs.double_rts_mask;
end

study_context = struct();
study_context.version = 2;
study_context.data_source = char(data_source);
study_context.dataset_id = char(dataset.dataset_id);
study_context.study_id = "acoustic-range-dropout";
study_context.study_title = data_source_name(data_source) + ...
    "声学测距掉帧敏感性";
study_context.position_error_unit = char(position_error_unit);
study_context.scenario_ids = scenario_ids;
study_context.scenario_names = scenario_names;
study_context.parameter_values = parameter_values;
study_context.parameter_name = "DroppedMeasurementIndex";
study_context.parameter_label = "丢失的声学测距包序号";
study_context.output_dirs = output_dirs;
study_context.truth_path = dataset.truth_path;
study_context.method_ids = ["ekf"; "single-rts"; "double-rts"];
study_context.method_names = ["前向EKF"; "一次RTS"; "二次RTS"];
study_context.method_files = ["range-ins-forward.nav"; ...
    "range-ins-rts-single.nav"; "range-ins-rts-double.nav"];
study_context.valid_masks = valid_masks;
study_context.artifact_root = directories.artifact_root;
save(fullfile(directories.result_root, ...
    'study-context.mat'), 'study_context');
fprintf('声学测距掉帧专题完成：%s\n', directories.result_root);

function name = data_source_name(source)
    if source == "simulation"
        name = "仿真数据：";
    else
        name = "实测数据：";
    end
end
