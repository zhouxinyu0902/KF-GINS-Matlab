function directories = engineering_problem_directories( ...
        paths, data_source, simulation_case, study_id, position_error_unit)
%ENGINEERING_PROBLEM_DIRECTORIES 生成工程专题统一输入、结果和图表目录。
    data_source = lower(string(data_source));
    position_error_unit = lower(string(position_error_unit));
    study_id = char(string(study_id));
    if data_source == "simulation"
        case_name = char(string(simulation_case));
        case_input = fullfile(paths.simulation_input, case_name);
        case_result = fullfile(paths.simulation_navigation, case_name);
        case_artifact = fullfile(paths.simulation_artifacts, case_name);
    elseif data_source == "experiment"
        case_input = paths.experiment_input;
        case_result = paths.experiment_navigation;
        case_artifact = paths.experiment_artifacts;
    else
        error('未知数据来源：%s', data_source);
    end
    directories.input_root = fullfile(case_input, study_id);
    output_name = sprintf('%s-%s', study_id, char(position_error_unit));
    directories.result_root = fullfile(case_result, output_name);
    directories.artifact_root = fullfile(case_artifact, output_name);
    required = {directories.input_root, directories.result_root, ...
        directories.artifact_root};
    for index = 1:numel(required)
        if ~isfolder(required{index}), mkdir(required{index}); end
    end
end
