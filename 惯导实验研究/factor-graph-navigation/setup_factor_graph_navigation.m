function paths = setup_factor_graph_navigation()
%SETUP_FACTOR_GRAPH_NAVIGATION 配置因子图导航专题代码与数据路径。

    topic_root = fileparts(mfilename('fullpath'));
    research_root = fileparts(topic_root);
    project_root = fileparts(research_root);

    addpath(project_root);
    addpath(fullfile(project_root, 'function'));
    addpath(fullfile(project_root, 'function_zxy'));
    addpath(fullfile(project_root, 'function_zxy', 'Update'));
    addpath(fullfile(project_root, 'function_zxy', 'ErrorFeedback'));
    addpath(fullfile(project_root, 'GINS-KF'));
    addpath(fullfile(topic_root, 'config'));
    addpath(fullfile(topic_root, 'scripts', 'gnss-ins'));
    addpath(fullfile(topic_root, 'scripts', 'range-ins'));
    addpath(fullfile(topic_root, 'scripts', 'analysis'));

    paths.root = topic_root;
    paths.project_root = project_root;
    % 与 paper-study Dataset 1 哈希一致，直接复用，避免保留第二份输入。
    paths.input = fullfile(project_root, 'data', 'inertial-experiment', ...
        'paper-study', 'input', 'simulation', 'dataset1');
    paths.data_root = fullfile(project_root, 'data', ...
        'inertial-experiment', 'factor-graph-navigation');
    paths.derived_input = fullfile(paths.data_root, 'derived-input');
    paths.navigation_results = fullfile(paths.data_root, 'navigation-results');
    paths.figures_tables = fullfile(paths.data_root, 'figures-tables');

    required_dirs = {paths.derived_input, paths.navigation_results, ...
        paths.figures_tables};
    for index = 1:numel(required_dirs)
        if ~isfolder(required_dirs{index})
            mkdir(required_dirs{index});
        end
    end
end
