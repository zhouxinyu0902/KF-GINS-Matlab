function directories = engineering_problem_directories( ...
        paths, data_source, simulation_case, study_id, position_error_unit)
%ENGINEERING_PROBLEM_DIRECTORIES 生成工程专题统一输入、结果和图表目录。
    data_source = lower(string(data_source));
    position_error_unit = lower(string(position_error_unit));
    study_id = char(string(study_id));
    if data_source == "simulation"
        case_id = parse_case_id(simulation_case);
        case_name = sprintf('case-%02d', case_id);
        case_input = resolve_case_path( ...
            paths.simulation_input, case_id, case_name);
        case_result = resolve_case_path( ...
            paths.simulation_navigation, case_id, case_name);
        case_artifact = resolve_case_path( ...
            paths.simulation_artifacts, case_id, case_name);
    elseif data_source == "experiment"
        % 当前实测配置与 run_rts_navigation_study.m 一致，使用 case-06。
        case_id = 6;
        case_name = sprintf('case-%02d', case_id);
        case_input = resolve_case_path( ...
            paths.experiment_input, case_id, case_name);
        case_result = resolve_case_path( ...
            paths.experiment_navigation, case_id, case_name);
        case_artifact = resolve_case_path( ...
            paths.experiment_artifacts, case_id, case_name);
    else
        error('未知数据来源：%s', data_source);
    end
    directories.input_root = fullfile(case_input, study_id);
    output_name = sprintf('%s-%s', study_id, char(position_error_unit));
    directories.result_root = fullfile(case_result,'navigation-results', output_name);
    directories.artifact_root = fullfile(case_artifact,'figures-tables', output_name);
    required = {directories.input_root, directories.result_root, ...
        directories.artifact_root};
    for index = 1:numel(required)
        if ~isfolder(required{index}), mkdir(required{index}); end
    end
end

function resolved = resolve_case_path(path_source, case_id, case_name)
%RESOLVE_CASE_PATH 同时兼容路径根目录和按编号返回路径的函数句柄。
    if isa(path_source, 'function_handle')
        resolved = path_source(case_id);
    else
        resolved = fullfile(path_source, case_name);
    end
end

function case_id = parse_case_id(case_name)
%PARSE_CASE_ID 从 case-00 一类名称中提取数据编号。
    token = regexp(char(string(case_name)), '^case-(\d+)$', ...
        'tokens', 'once');
    if isempty(token)
        error('simulation_case 必须采用 case-00 一类格式。');
    end
    case_id = str2double(token{1});
end
