clear;
clc;

%% 潜标位置误差对导航的影响：仿真/实测统一入口
data_source = "experiment";         % "simulation" 或 "experiment"
simulation_case = 'case-00';        % 仅仿真使用
position_error_unit = "rad";        % 仿真可选 rad/m；实测当前使用 rad
error_enu_m = [0,0,0; 10,10,0.4; 20,20,0.4; ...
    50,50,0.4; 100,100,0.4; 200,200,1.0];
range_interval_s = 420;
duration_s = 4621;
random_seed = 1;
simulation_range_noise_std_m = 10;

%% 统一数据与输出目录
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));
addpath(topic_dir);
paths = setup_inertial_experiment();
data_source = lower(data_source);
position_error_unit = lower(position_error_unit);
filter_range_std_m = 10;
if data_source == "experiment", filter_range_std_m = 6; end

manifest = process_beaconpos(data_source, simulation_case, error_enu_m, ...
    [], range_interval_s, random_seed, simulation_range_noise_std_m);
dataset = load_engineering_problem_dataset(data_source, simulation_case, ...
    range_interval_s, random_seed, simulation_range_noise_std_m);
directories = engineering_problem_directories(paths, data_source, ...
    simulation_case, 'beacon-position-error', position_error_unit);

%% 批量运行前向EKF、一次RTS和二次RTS
scenario_count = height(manifest);
output_dirs = strings(scenario_count, 1);
valid_masks = cell(scenario_count, 3);
for index = 1:scenario_count
    output_dirs(index) = fullfile(directories.result_root, ...
        manifest.ScenarioId(index));
    rangedata = readmatrix(manifest.RangeFilePath(index), 'FileType', 'text');
    options = struct('position_error_unit', position_error_unit, ...
        'duration_s', duration_s, 'filter_range_std_m', ...
        filter_range_std_m, 'random_seed', random_seed, ...
        'case_name', char("beacon-error-"+manifest.ScenarioId(index)));
    outputs = run_engineering_navigation_case( ...
        dataset, output_dirs(index), rangedata, options);
    valid_masks{index, 1} = outputs.forward_mask;
    valid_masks{index, 2} = outputs.single_rts_mask;
    valid_masks{index, 3} = outputs.double_rts_mask;
end

%% 保存与数据来源无关的评价上下文
study_context = struct();
study_context.version = 2;
study_context.data_source = char(data_source);
study_context.dataset_id = char(dataset.dataset_id);
study_context.study_id = "beacon-position-error";
study_context.study_title = data_source_name(data_source)+"潜标位置误差敏感性";
study_context.position_error_unit = char(position_error_unit);
study_context.scenario_ids = manifest.ScenarioId;
study_context.scenario_names = compose('水平误差 %.1f m', ...
    manifest.HorizontalError_m);
study_context.parameter_values = manifest.HorizontalError_m;
study_context.parameter_name = "HorizontalBeaconPositionError_m";
study_context.parameter_label = "潜标水平位置误差（m）";
study_context.output_dirs = output_dirs;
study_context.truth_path = dataset.truth_path;
study_context.method_ids = ["ekf"; "single-rts"; "double-rts"];
study_context.method_names = ["前向EKF"; "一次RTS"; "二次RTS"];
study_context.method_files = ["range-ins-forward.nav"; ...
    "range-ins-rts-single.nav"; "range-ins-rts-double.nav"];
study_context.valid_masks = valid_masks;
study_context.artifact_root = directories.artifact_root;
save(fullfile(directories.result_root, 'study-context.mat'), 'study_context');
fprintf('潜标位置误差专题完成：%s\n', directories.result_root);

function name = data_source_name(source)
%DATA_SOURCE_NAME 返回图表使用的数据来源名称。
    if source == "simulation", name = "仿真数据："; else, name = "实测数据："; end
end
