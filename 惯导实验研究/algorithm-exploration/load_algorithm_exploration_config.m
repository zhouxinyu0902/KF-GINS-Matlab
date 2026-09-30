function cfg = load_algorithm_exploration_config( ...
        data_source, position_error_unit, input_dir)
%LOAD_ALGORITHM_EXPLORATION_CONFIG 从本专题config目录加载唯一配置。
%   临时进入明确的配置目录，避免 MATLAB 当前目录中的同名旧函数遮蔽。
    data_source = lower(string(data_source));
    position_error_unit = lower(string(position_error_unit));
    topic_dir = fileparts(mfilename('fullpath'));
    previous_dir = pwd;
    restore_dir = onCleanup(@() cd(previous_dir)); 
    if data_source == "simulation"
        if nargin < 3 || isempty(input_dir)
            error('加载仿真配置时必须提供 input_dir。');
        end
        cd(fullfile(topic_dir, 'config', 'simulation'));
        if position_error_unit == "rad"
            cfg = ProcessConfigforSimu(input_dir);
        elseif position_error_unit == "m"
            cfg = ProcessConfigforSimu_m(input_dir);
        else
            error('未知位置误差单位：%s', position_error_unit);
        end
    elseif data_source == "experiment"
        if nargin < 3 || isempty(input_dir)
            project_root = fileparts(fileparts(topic_dir));
            input_dir = fullfile(project_root, 'data', ...
                'inertial-experiment', 'algorithm-exploration', ...
                'experiment', 'case-07', 'input');
        end
        cd(fullfile(topic_dir, 'config', 'experiment'));
        if position_error_unit == "rad"
            cfg = ProcessConfig_exper(input_dir);
        elseif position_error_unit == "m"
            cfg = ProcessConfig_exper_m(input_dir);
        else
            error('未知位置误差单位：%s', position_error_unit);
        end
    else
        error('未知数据来源：%s', data_source);
    end
end
