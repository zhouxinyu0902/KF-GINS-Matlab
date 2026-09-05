function paths = setup_experiment_02()
%SETUP_EXPERIMENT_01 配置第一次车载试验的代码、输入和输出路径。

    topic_root = fileparts(mfilename('fullpath'));
    vehicle_research_root = fileparts(topic_root);
    project_root = fileparts(vehicle_research_root);

    % 显式加入依赖，避免依赖用户已经保存的 MATLAB 搜索路径。
    addpath(project_root);
    addpath(fullfile(project_root, 'function'));
    addpath(fullfile(project_root, 'function_zxy'));
    addpath(fullfile(project_root, 'function_zxy', 'Update'));
    addpath(fullfile(project_root, 'function_zxy', 'ErrorFeedback'));
    addpath(fullfile(project_root, 'GINS-KF'));
    addpath(topic_root);
    % addpath(fullfile(topic_root, 'functions'));
    addpath(fullfile(topic_root, 'data-process'));

    paths.root = topic_root;
    paths.project_root = project_root;
    paths.data_root = fullfile(project_root, 'data', ...
        'experiment-data', 'experiment-02');
    paths.case_dir = @(case_id) fullfile(paths.data_root,'1207-longtime', ...
        sprintf('case-%02d', case_id));
    paths.case_input = @(case_id) fullfile(paths.case_dir(case_id), ...
        'input');
    paths.case_navigation = @(case_id) fullfile( ...
        paths.case_dir(case_id),'output', 'navigation-results');
    paths.case_artifacts = @(case_id) fullfile( ...
        paths.case_dir(case_id),'output', 'figures-tables');
    paths.summary_artifacts = fullfile(paths.data_root, 'summary');

    for case_id = 1:2
        if ~isfolder(paths.case_input(case_id))
            error('第 %d 组输入目录不存在：%s', ...
                case_id, paths.case_input(case_id));
        end
        required_dirs = {paths.case_navigation(case_id), ...
            paths.case_artifacts(case_id)};
        for index = 1:numel(required_dirs)
            if ~isfolder(required_dirs{index})
                mkdir(required_dirs{index});
            end
        end
    end
    if ~isfolder(paths.summary_artifacts)
        mkdir(paths.summary_artifacts);
    end

    fprintf('Experiment-02 configured: %s\n', topic_root);
end
