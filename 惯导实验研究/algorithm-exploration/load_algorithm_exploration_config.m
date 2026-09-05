function cfg = load_algorithm_exploration_config( ...
        data_source, position_error_unit, simulation_input_dir)
%LOAD_ALGORITHM_EXPLORATION_CONFIG 从本专题config目录加载唯一配置。
%   临时进入明确的配置目录，避免 MATLAB 当前目录中的同名旧函数遮蔽。
    data_source = lower(string(data_source));
    position_error_unit = lower(string(position_error_unit));
    topic_dir = fileparts(mfilename('fullpath'));
    previous_dir = pwd;
    restore_dir = onCleanup(@() cd(previous_dir)); 
    if data_source == "simulation"
        if nargin < 3 || isempty(simulation_input_dir)
            error('加载仿真配置时必须提供 simulation_input_dir。');
        end
        cd(fullfile(topic_dir, 'config', 'simulation'));
        if position_error_unit == "rad"
            cfg = ProcessConfigforSimu(simulation_input_dir);
        elseif position_error_unit == "m"
            cfg = ProcessConfigforSimu_m(simulation_input_dir);
        else
            error('未知位置误差单位：%s', position_error_unit);
        end
    elseif data_source == "experiment"
        cd(fullfile(topic_dir, 'config', 'experiment'));
        if position_error_unit == "rad"
            cfg = ProcessConfig_exper();
        elseif position_error_unit == "m"
            cfg = ProcessConfig_exper_m();
        else
            error('未知位置误差单位：%s', position_error_unit);
        end
    else
        error('未知数据来源：%s', data_source);
    end
end
