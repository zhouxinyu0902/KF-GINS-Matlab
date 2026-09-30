function paths = setup_all_real_data_preprocessing(dataset_id, check_scope)
%SETUP_ALL_REAL_DATA_PREPROCESSING 配置第三次车载试验路径与依赖。
%
% check_scope:
%   'raw'        仅检查原始文件解析依赖；
%   'processing' 检查可视化、转换和输入导出依赖；
%   'navigation' 检查完整导航依赖（默认，兼容原调用方式）。

    if nargin < 1 || isempty(dataset_id)
        dataset_id = 'run-0817';
    end
    if nargin < 2 || isempty(check_scope)
        check_scope = 'navigation';
    end
    check_scope = lower(string(check_scope));
    if ~ismember(check_scope, ["raw", "processing", "navigation"])
        error('check_scope 只能取 raw、processing 或 navigation。');
    end

    paths = experiment03_dataset_paths(dataset_id);
    github_root = fileparts(paths.project);

    if ~isfolder(paths.data)
        error('第三次试验数据目录不存在：%s', paths.data);
    end
    if ~isfolder(paths.raw)
        error('第三次试验原始数据目录不存在：%s', paths.raw);
    end
    if ~isfile(paths.initial_state_file)
        error('第三次试验初始状态文件不存在：%s', ...
            paths.initial_state_file);
    end

    addpath(paths.topic, '-begin');
    addpath(paths.data_process, '-begin');
    addpath(fullfile(paths.topic, 'functions'), '-begin');
    addpath(genpath(fullfile(paths.analysis, 'func')), '-begin');
    addpath(genpath(fullfile(paths.project, 'function')), '-begin');
    addpath(genpath(fullfile(paths.project, 'function_zxy')), '-begin');
    addpath(genpath(fullfile(paths.project, 'GINS-KF')), '-begin');

    if check_scope == "navigation"
        addpath(genpath(fullfile(paths.project, '惯导实验研究', ...
            'algorithm-exploration', 'functions', 'experiment')), '-begin');
    end

    if check_scope ~= "raw"
        % glvs 依赖 PSINS，默认使用与本项目同属 D:\Github 的标准位置。
        psins_root = fullfile(github_root, 'PSINS', 'psins2401');
        if isfolder(psins_root)
            addpath(genpath(psins_root), '-begin');
        elseif isempty(which('glvs'))
            error(['未找到 PSINS。请将 PSINS 放在 %s，或先在 MATLAB 中', ...
                '加入包含 glvs.m 的路径。'], psins_root);
        end
    end

    switch check_scope
        case "raw"
            required_functions = {'read_mems_ins'};
        case "processing"
            required_functions = { ...
                'visualize_gpchcx_navigation_results', ...
                'imuFUR2FRD', 'regularize_experiment03_imu', ...
                'RCompu', 'glvs', 'yaml.ReadYaml'};
        otherwise
            required_functions = { ...
                'Param', 'InsMech', 'myInitialize_15state', ...
                'myInsPropagate_15state', 'myErrorFeedback_range', ...
                'myRangeUpdate', 'update_decoupled_height_rad', ...
                'myInsPropagate_15state_m', 'myErrorFeedback_range_m', ...
                'myRangeUpdate_m', 'update_decoupled_height_m', ...
                'bridge_error_horizontal_m', 'rotateAndScaleTrajectory', ...
                'glvs', 'yaml.ReadYaml'};
    end

    missing_functions = required_functions(cellfun( ...
        @(name) isempty(which(name)), required_functions));
    if ~isempty(missing_functions)
        error('第三次试验依赖未配置完整：%s', ...
            strjoin(missing_functions, ', '));
    end

    required_directories = {paths.intermediate, paths.input, ...
        paths.output, paths.artifacts, paths.summary};
    for directory_index = 1:numel(required_directories)
        if ~isfolder(required_directories{directory_index})
            mkdir(required_directories{directory_index});
        end
    end
end
