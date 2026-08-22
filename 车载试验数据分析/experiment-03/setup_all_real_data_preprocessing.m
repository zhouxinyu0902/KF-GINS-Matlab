function paths = setup_all_real_data_preprocessing(dataset_id)
%SETUP_ALL_REAL_DATA_PREPROCESSING 配置第三次车载试验的数据与依赖路径。

    if nargin < 1 || isempty(dataset_id)
        dataset_id = 'run-0817';
    end

    paths = experiment03_dataset_paths(dataset_id);
    github_root = fileparts(paths.project);

    if ~isfolder(paths.data)
        error('第三次试验数据目录不存在：%s', paths.data);
    end
    if ~isfolder(paths.raw)
        error('第三次试验原始数据目录不存在：%s', paths.raw);
    end

    addpath(paths.topic, '-begin');
    addpath(genpath(fullfile(paths.analysis, 'func')), '-begin');
    addpath(genpath(fullfile(paths.project, 'function')), '-begin');
    addpath(genpath(fullfile(paths.project, 'function_zxy')), '-begin');
    addpath(genpath(fullfile(paths.project, 'GINS-KF')), '-begin');
    addpath(genpath(fullfile(paths.project, '惯导实验研究', ...
        'algorithm-exploration', 'functions', 'experiment')), '-begin');

    % glvs 依赖 PSINS，默认使用与本项目同属 D:\Github 的标准位置。
    psins_root = fullfile(github_root, 'PSINS', 'psins2401');
    if isfolder(psins_root)
        addpath(genpath(psins_root), '-begin');
    elseif isempty(which('glvs'))
        error(['未找到 PSINS。请将 PSINS 放在 %s，或先在 MATLAB 中加入', ...
            '包含 glvs.m 的路径。'], psins_root);
    end

    required_functions = { ...
        'Param', 'InsMech', 'myInitialize_15state', ...
        'myInsPropagate_15state', 'myErrorFeedback_range', ...
        'myRangeUpdate', 'update_decoupled_height_rad', ...
        'myInsPropagate_15state_m', 'myErrorFeedback_range_m', ...
        'myRangeUpdate_m', 'update_decoupled_height_m', ...
        'bridge_error_horizontal_m', 'rotateAndScaleTrajectory', 'glvs'};
    missing_functions = required_functions(cellfun( ...
        @(name) isempty(which(name)), required_functions));
    if ~isempty(missing_functions)
        error('第三次试验依赖未配置完整：%s', ...
            strjoin(missing_functions, ', '));
    end

    required_directories = {paths.intermediate, paths.input, ...
        paths.output, paths.artifacts};
    for directory_index = 1:numel(required_directories)
        if ~isfolder(required_directories{directory_index})
            mkdir(required_directories{directory_index});
        end
    end
end
