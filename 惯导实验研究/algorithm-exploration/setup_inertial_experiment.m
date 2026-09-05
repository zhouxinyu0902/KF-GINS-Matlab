function paths = setup_inertial_experiment()
%SETUP_INERTIAL_EXPERIMENT 配置“算法探索”专题的代码、输入和输出路径。
    topic_dir = fileparts(mfilename('fullpath'));
    inertial_research_dir = fileparts(topic_dir);
    project_root = fileparts(inertial_research_dir);
    addpath("D:\Github\PSINS");
    addpath(project_root);
    addpath(fullfile(project_root, 'function'));
    addpath(fullfile(project_root, 'function_zxy'));
    addpath(fullfile(project_root, 'GINS-KF'));
    addpath(fullfile(topic_dir, 'config', 'experiment'));
    addpath(fullfile(topic_dir, 'config', 'simulation'));
    addpath(fullfile(topic_dir, 'functions', 'experiment'));
    addpath(fullfile(topic_dir, 'functions', 'simulation'));
    addpath(fullfile(topic_dir, 'scripts', '02_rts-algorithm-study'));
    addpath(genpath(fullfile(topic_dir, 'scripts', '03_engineering-problem')));
    addpath(genpath(fullfile(topic_dir, 'scripts', 'evaluation')));
    addpath(genpath(fullfile(topic_dir, 'scripts', '00_data-generation')));
    addpath(fullfile(topic_dir, 'scripts', '01_gnss-ins-baseline'));


    paths.topic = topic_dir;
    paths.project = project_root;
    paths.data = fullfile(project_root, 'data', 'inertial-experiment','algorithm-exploration');
    paths.simulation = fullfile(paths.data, 'simulation');
    paths.experiment = fullfile(paths.data, 'experiment');
    paths.simulation_input = @(case_id) fullfile(paths.simulation,sprintf('case-%02d', case_id),'input');
    paths.experiment_input = @(case_id) fullfile(paths.experiment,sprintf('case-%02d', case_id),'input');

    paths.experiment_raw = paths.experiment_input;
    paths.experiment_preprocessed = paths.experiment_input;
    paths.experiment_reference = paths.experiment_input;

    paths.experiment_navigation = @(case_id) fullfile(paths.experiment,sprintf('case-%02d', case_id),'output');
    paths.simulation_navigation = @(case_id) fullfile(paths.simulation,sprintf('case-%02d', case_id),'output');   
    paths.experiment_artifacts = @(case_id) fullfile(paths.experiment_navigation(case_id) , 'figures-tables');
    paths.simulation_artifacts = @(case_id) fullfile(paths.simulation_navigation(case_id) , 'figures-tables');

    % required_dirs = {paths.experiment_navigation, ...
    %     paths.simulation_navigation, paths.experiment_artifacts, ...
    %     paths.simulation_artifacts, paths.experiment_input, ...
    %     paths.simulation_input};
    % for index = 1:numel(required_dirs)
    %     if ~isfolder(required_dirs{index})
    %         mkdir(required_dirs{index});
    %     end
    % end

    fprintf('Algorithm-exploration topic configured: %s\n', topic_dir);
end
