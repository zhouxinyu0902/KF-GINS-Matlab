function paths = setup_mems_range_navigation()
%SETUP_MEMS_RANGE_NAVIGATION 配置MEMS/测距专题的代码、数据和输出路径。
    topic_dir = fileparts(mfilename('fullpath'));
    inertial_research_dir = fileparts(topic_dir);
    project_root = fileparts(inertial_research_dir);

    addpath(project_root);
    addpath(fullfile(project_root, 'function'));
    addpath(fullfile(project_root, 'function_zxy'));
    addpath(fullfile(project_root, 'GINS-KF'));
    addpath(fullfile(topic_dir, 'config', 'experiment'));
    addpath(fullfile(topic_dir, 'config', 'simulation'));
    addpath(fullfile(topic_dir, 'functions', 'analysis'));
    addpath(fullfile(topic_dir, 'functions', 'azimuth'));
    addpath(fullfile(topic_dir, 'functions', 'navigation'));
    addpath(fullfile(topic_dir, 'functions', 'optimization'));
    addpath(fullfile(topic_dir, 'scripts', 'azimuth'));
    addpath(fullfile(topic_dir, 'scripts', 'evaluation'));
    addpath(fullfile(topic_dir, 'scripts', 'experiment'));
    addpath(fullfile(topic_dir, 'scripts', 'simulation'));

    paths.topic = topic_dir;
    paths.project = project_root;
    paths.data = fullfile(project_root, 'data', 'inertial-experiment', ...
        'mems-range-navigation');
    paths.adis16545 = fullfile(paths.data, 'data_16545');
    paths.i300 = fullfile(paths.data, 'data_i300');
    paths.simulation = fullfile(paths.data, 'data_simu');
    for index = 1:6
        field_name = sprintf('dataset_830_430_%d', index);
        paths.(field_name) = fullfile(paths.data, ...
            sprintf('data_830_430_%d', index));
    end

    required_data_dirs = {paths.adis16545, paths.i300, paths.simulation, ...
        paths.dataset_830_430_1, paths.dataset_830_430_2, ...
        paths.dataset_830_430_3, paths.dataset_830_430_4, ...
        paths.dataset_830_430_5, paths.dataset_830_430_6};
    for index = 1:numel(required_data_dirs)
        if ~isfolder(required_data_dirs{index})
            error('MEMS/测距数据目录不存在：%s', required_data_dirs{index});
        end
    end

    output_roots = {paths.dataset_830_430_1, ...
        paths.dataset_830_430_2, paths.dataset_830_430_3, ...
        paths.dataset_830_430_4, paths.dataset_830_430_5, ...
        paths.dataset_830_430_6, paths.simulation};
    for index = 1:numel(output_roots)
        output_dir = fullfile(output_roots{index}, 'output');
        if ~isfolder(output_dir)
            mkdir(output_dir);
        end
    end
end
