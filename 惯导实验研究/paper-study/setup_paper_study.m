function paths = setup_paper_study()
%SETUP_PAPER_STUDY 配置论文实验专题的代码、输入与输出路径。
% 实测 Dataset 1/2 对应 F 盘 input5/input6；仿真输入和全部输出统一
% 保存到 data/inertial-experiment/paper-study。

    topic_root = fileparts(mfilename('fullpath'));
    inertial_topic_root = fileparts(topic_root);
    project_root = fileparts(inertial_topic_root);

    % 只加入本专题实际依赖的目录，避免依赖历史 MATLAB path。
    addpath(project_root);
    addpath(fullfile(project_root, 'function'));
    addpath(fullfile(project_root, 'function_zxy'));
    addpath(fullfile(project_root, 'function_zxy', 'Update'));
    addpath(fullfile(project_root, 'function_zxy', 'ErrorFeedback'));
    addpath(fullfile(project_root, 'GINS-KF'));
    addpath(fullfile(topic_root, 'config'));
    addpath(fullfile(topic_root, 'functions'));
    addpath(fullfile(topic_root, 'functions', 'navigation'));
    addpath(fullfile(topic_root, 'functions', 'observability'));
    addpath(fullfile(topic_root, 'functions', 'plotting'));
    addpath(fullfile(topic_root, 'scripts', 'experiment'));
    addpath(fullfile(topic_root, 'scripts', 'simulation'));
    addpath(fullfile(topic_root, 'scripts', 'analysis'));

    paths.root = topic_root;
    paths.project_root = project_root;
    paths.external_experiment_root = ...
        'D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-01';
    paths.data_root = fullfile(project_root, 'data', ...
        'inertial-experiment', 'paper-study');

    paths.simulation_dataset1 = fullfile(paths.data_root, ...
        'input', 'simulation', 'dataset1');
    paths.simulation_dataset2 = fullfile(paths.data_root, ...
        'input', 'simulation', 'dataset2');

    paths.output_experiment = fullfile(paths.data_root, ...
        'navigation-results', 'experiment');
    paths.output_experiment_dataset1 = fullfile( ...
        paths.output_experiment, 'dataset1');
    paths.output_experiment_dataset2 = fullfile( ...
        paths.output_experiment, 'dataset2');
    paths.output_simulation = fullfile(paths.data_root, ...
        'navigation-results', 'simulation');
    paths.output_simulation_dataset1 = fullfile( ...
        paths.output_simulation, 'dataset1');
    paths.output_simulation_dataset2 = fullfile( ...
        paths.output_simulation, 'dataset2');

    paths.figures_tables = fullfile(paths.data_root, 'figures-tables');
    paths.experiment_figures_tables = fullfile(paths.figures_tables, ...
        'experiment');
    paths.simulation_figures_tables = fullfile(paths.figures_tables, ...
        'simulation');
    % paths.paper_figures = fullfile(paths.figures_tables, ...
    %     'paper-artifacts', 'figures');
    paths.paper_figures = fullfile(paths.figures_tables, ...
        'paper-artifacts', 'figures-new');
    paths.paper_tables = fullfile(paths.figures_tables, ...
        'paper-artifacts', 'tables');

    required_dirs = {paths.output_experiment_dataset1, ...
        paths.output_experiment_dataset2, ...
        paths.output_simulation_dataset1, ...
        paths.output_simulation_dataset2, paths.paper_figures, ...
        paths.paper_tables};
    for index = 1:numel(required_dirs)
        if ~isfolder(required_dirs{index})
            mkdir(required_dirs{index});
        end
    end

    fprintf('Paper-study topic configured: %s\n', topic_root);
end
