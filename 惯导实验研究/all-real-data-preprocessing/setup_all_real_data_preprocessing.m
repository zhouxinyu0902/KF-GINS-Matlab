function paths = setup_all_real_data_preprocessing()
%SETUP_ALL_REAL_DATA_PREPROCESSING 配置“全部实测数据预处理”专题。
% 所有输入、导航结果和图表统一放在 data/inertial-experiment/data-0817。

    topic_dir = fileparts(mfilename('fullpath'));
    inertial_research_dir = fileparts(topic_dir);
    project_root = fileparts(inertial_research_dir);
    github_root = fileparts(project_root);

    paths.topic = topic_dir;
    paths.project = project_root;
    paths.data = fullfile(project_root, 'data', 'inertial-experiment', ...
        'data-0817');
    paths.input = fullfile(paths.data, 'input');
    paths.output = fullfile(paths.data, 'output');
    paths.artifacts = fullfile(paths.output, 'artifacts');

    addpath(topic_dir, '-begin');
    addpath(genpath(fullfile(topic_dir, 'func')), '-begin');
    addpath(genpath(fullfile(project_root, 'function')), '-begin');
    addpath(genpath(fullfile(project_root, 'function_zxy')), '-begin');
    addpath(genpath(fullfile(project_root, 'GINS-KF')), '-begin');
    addpath(genpath(fullfile(inertial_research_dir, 'algorithm-exploration', ...
        'functions', 'experiment')), '-begin');

    % glvs 依赖 PSINS。优先使用与本项目同处 D:\Github 下的标准位置。
    psins_root = fullfile(github_root, 'PSINS', 'psins2401');
    if isfolder(psins_root)
        addpath(genpath(psins_root), '-begin');
    elseif isempty(which('glvs'))
        error(['未找到 PSINS。请将 PSINS 放在 %s，或在 MATLAB 中预先添加', ...
            '包含 glvs.m 的路径。'], psins_root);
    end

    required_functions = { ...
        'Param', 'InsMech', 'myInitialize_15state', ...
        'myInsPropagate_15state', 'myErrorFeedback_range', ...
        'myRangeUpdate', 'update_decoupled_height_rad', ...
        'bridge_error_horizontal_m', 'rotateAndScaleTrajectory', 'glvs'};
    missing_functions = required_functions(cellfun( ...
        @(name) isempty(which(name)), required_functions));
    if ~isempty(missing_functions)
        error('专题依赖未配置完整：%s', strjoin(missing_functions, ', '));
    end

    required_directories = {paths.input, paths.output, paths.artifacts};
    for directory_index = 1:numel(required_directories)
        if ~isfolder(required_directories{directory_index})
            mkdir(required_directories{directory_index});
        end
    end
end
