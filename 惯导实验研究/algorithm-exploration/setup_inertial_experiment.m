function paths = setup_inertial_experiment()
%SETUP_INERTIAL_EXPERIMENT 配置“算法探索”专题的代码、输入和输出路径。
    topic_dir = fileparts(mfilename('fullpath'));
    inertial_research_dir = fileparts(topic_dir);
    project_root = fileparts(inertial_research_dir);

    addpath(project_root);
    addpath(fullfile(project_root, 'function'));
    addpath(fullfile(project_root, 'function_zxy'));
    addpath(fullfile(project_root, 'GINS-KF'));
    addpath(fullfile(topic_dir, 'config', 'experiment'));
    addpath(fullfile(topic_dir, 'config', 'simulation'));
    addpath(fullfile(topic_dir, 'functions', 'experiment'));
    addpath(fullfile(topic_dir, 'functions', 'simulation'));

    paths.topic = topic_dir;
    paths.project = project_root;
    paths.data = fullfile(project_root, 'data', 'inertial-experiment', ...
        'algorithm-exploration');
    paths.input = fullfile(paths.data, 'input');
    paths.experiment_raw = fullfile(paths.input, 'experiment-raw');
    paths.experiment_preprocessed = fullfile(paths.input, ...
        'experiment-preprocessed');
    paths.experiment_reference = fullfile(paths.input, ...
        'experiment-reference');
    paths.simulation_input = fullfile(paths.input, 'simulation');
    paths.navigation_results = fullfile(paths.data, 'navigation-results');
    paths.experiment_navigation = fullfile(paths.navigation_results, ...
        'experiment');
    paths.simulation_navigation = fullfile(paths.navigation_results, ...
        'simulation');
    paths.figures_tables = fullfile(paths.data, 'figures-tables');
    paths.experiment_artifacts = fullfile(paths.figures_tables, 'experiment');
    paths.simulation_artifacts = fullfile(paths.figures_tables, 'simulation');

    required_dirs = {paths.experiment_navigation, ...
        paths.simulation_navigation, paths.experiment_artifacts, ...
        paths.simulation_artifacts, paths.experiment_preprocessed};
    for index = 1:numel(required_dirs)
        if ~isfolder(required_dirs{index})
            mkdir(required_dirs{index});
        end
    end

    fprintf('Algorithm-exploration topic configured: %s\n', topic_dir);
end
