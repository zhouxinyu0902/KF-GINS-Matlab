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
    addpath(fullfile(topic_dir, 'scripts', 'rts-algorithm-study'));
    addpath(genpath(fullfile(topic_dir, 'scripts', 'engineering-problem')));
    addpath(genpath(fullfile(topic_dir, 'scripts', 'evaluation')));
    addpath(genpath(fullfile(topic_dir, 'scripts', 'data-generation')));
    addpath(fullfile(topic_dir, 'scripts', 'data-preparation'));
    addpath(fullfile(topic_dir, 'scripts', 'gnss-ins-baseline'));
    addpath(fullfile(topic_dir, 'scripts', 'range-azimuth-aided'));

    paths.topic = topic_dir;
    paths.project = project_root;
    paths.data = fullfile(project_root, 'data', 'inertial-experiment', ...
        'algorithm-exploration');
    paths.input = fullfile(paths.data, 'input');
    paths.simulation_input = fullfile(paths.input, 'simulation');
    paths.experiment_case = 'case-06';
    paths.experiment_input = fullfile(paths.input, 'experiment', ...
        paths.experiment_case);

    % 兼容仍按“原始/预处理/参考”字段取路径的算法。三类文件现在均在
    % experiment/case-06 中，避免同一数据集被拆成多个顶层目录。
    paths.experiment_raw = paths.experiment_input;
    paths.experiment_preprocessed = paths.experiment_input;
    paths.experiment_reference = paths.experiment_input;
    paths.navigation_results = fullfile(paths.data, 'navigation-results');
    paths.experiment_navigation = fullfile(paths.navigation_results, ...
        'experiment', paths.experiment_case);
    paths.simulation_navigation = fullfile(paths.navigation_results, ...
        'simulation');
    paths.figures_tables = fullfile(paths.data, 'figures-tables');
    paths.experiment_artifacts = fullfile(paths.figures_tables, ...
        'experiment', paths.experiment_case);
    paths.simulation_artifacts = fullfile(paths.figures_tables, 'simulation');

    required_dirs = {paths.experiment_navigation, ...
        paths.simulation_navigation, paths.experiment_artifacts, ...
        paths.simulation_artifacts, paths.experiment_input, ...
        paths.simulation_input};
    for index = 1:numel(required_dirs)
        if ~isfolder(required_dirs{index})
            mkdir(required_dirs{index});
        end
    end

    fprintf('Algorithm-exploration topic configured: %s\n', topic_dir);
end
