function paths = setup_cpp_matlab_comparison()
%SETUP_CPP_MATLAB_COMPARISON 配置 MATLAB/C++ 导航结果对比专题。

    topic_root = fileparts(mfilename('fullpath'));
    research_root = fileparts(topic_root);
    project_root = fileparts(research_root);

    addpath(project_root);
    addpath(fullfile(project_root, 'function'));
    addpath(fullfile(project_root, 'function_zxy'));
    addpath(fullfile(topic_root, 'scripts'));

    paths.root = topic_root;
    paths.project_root = project_root;
    paths.external_input = ...
        'F:\2_Data\惯导试验\实验数据\All_data\input6';
    paths.external_cpp_dataset = ...
        'D:\Github\KF-GINS-main\dataset_exper6';
    paths.data_root = fullfile(project_root, 'data', ...
        'inertial-experiment', 'cpp-matlab-comparison');
    paths.derived_input = fullfile(paths.data_root, 'derived-input');
    paths.matlab_results = fullfile(paths.data_root, ...
        'navigation-results', 'matlab');
    paths.cpp_results = fullfile(paths.data_root, ...
        'navigation-results', 'cpp');
    paths.figures_tables = fullfile(paths.data_root, 'figures-tables');

    required_dirs = {paths.derived_input, paths.matlab_results, ...
        paths.cpp_results, paths.figures_tables};
    for index = 1:numel(required_dirs)
        if ~isfolder(required_dirs{index})
            mkdir(required_dirs{index});
        end
    end
end
