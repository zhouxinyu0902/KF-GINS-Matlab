function paths = setup_all_real_data_processing()
%SETUP_ALL_REAL_DATA_PROCESSING 配置全实测数据处理专题路径。

    topic_root = fileparts(mfilename('fullpath'));
    inertial_research_root = fileparts(topic_root);
    project_root = fileparts(inertial_research_root);

    % 显式加入依赖，避免运行结果依赖用户已有 MATLAB path。
    addpath(project_root);
    addpath(fullfile(project_root, 'function'));
    addpath(fullfile(project_root, 'function_zxy'));
    addpath(fullfile(project_root, 'function_zxy', 'Update'));
    addpath(fullfile(project_root, 'function_zxy', 'ErrorFeedback'));
    addpath(fullfile(project_root, 'GINS-KF'));
    addpath(fullfile(topic_root, 'config'));
    addpath(fullfile(topic_root, 'functions'));
    addpath(fullfile(topic_root, 'functions', 'navigation'));
    addpath(fullfile(topic_root, 'scripts'));
    addpath(fullfile(topic_root, 'scripts', 'analysis'));
    addpath(fullfile(topic_root, 'scripts', 'experiments'));
    addpath(fullfile(topic_root, 'scripts', 'preprocessing'));

    paths.root = topic_root;
    paths.project_root = project_root;
    paths.external_input_root = 'F:\2_Data\惯导试验\实验数据\All_data';
    paths.data_root = fullfile(project_root, 'data', ...
        'inertial-experiment', 'all-real-data-processing');
    paths.derived_input = fullfile(paths.data_root, 'derived-input');
    paths.navigation_results = fullfile(paths.data_root, 'navigation-results');
    paths.figures_tables = fullfile(paths.data_root, 'figures-tables');

    required_dirs = {paths.derived_input, paths.navigation_results, ...
        paths.figures_tables};
    for dataset_id = 1:8
        required_dirs{end + 1} = fullfile(paths.derived_input, ...
            sprintf('dataset%d', dataset_id)); %#ok<AGROW>
        required_dirs{end + 1} = fullfile(paths.navigation_results, ...
            sprintf('dataset%d', dataset_id)); %#ok<AGROW>
        required_dirs{end + 1} = fullfile(paths.figures_tables, ...
            sprintf('dataset%d', dataset_id)); %#ok<AGROW>
    end

    for index = 1:numel(required_dirs)
        if ~isfolder(required_dirs{index})
            mkdir(required_dirs{index});
        end
    end

    fprintf('All-real-data-processing topic configured: %s\n', topic_root);
end
