% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2023.3.3
% -------------------------------------------------------------------------

function cfg = ProcessConfig_exper(input_dir)
    param = Param();
    %% filepath
    config_dir = fileparts(mfilename('fullpath'));
    topic_dir = fileparts(fileparts(config_dir));
    inertial_research_dir = fileparts(topic_dir);
    project_root = fileparts(inertial_research_dir);
    cfg.dataroot = fullfile(project_root, 'data', 'inertial-experiment', ...
        'algorithm-exploration');
    if nargin < 1 || isempty(input_dir)
        input_dir = fullfile(cfg.dataroot, 'experiment', 'case-07', 'input');
    end
    cfg.inputfolder = char(string(input_dir));
    [case_root, input_name] = fileparts(cfg.inputfolder);
    if ~strcmpi(input_name, 'input')
        error('实测输入目录必须以 input 结尾：%s', cfg.inputfolder);
    end
    [~, cfg.case_name] = fileparts(case_root);
    cfg.preprocessedfolder = cfg.inputfolder;
    cfg.referencefolder = cfg.inputfolder;
    cfg.outputfolder = fullfile(case_root, 'output', 'navigation-results');
    cfg.figurefolder = fullfile(case_root, 'output', 'figures-tables');

    required_dirs = {cfg.figurefolder, ...
        cfg.outputfolder};
    for index = 1:numel(required_dirs)
        if ~isfolder(required_dirs{index})
            mkdir(required_dirs{index});
        end
    end
    cfg.imufilepath = first_existing_file(cfg.inputfolder, ...
        {'imu_120.txt', 'IMU_120.txt'});
    cfg.gnssfilepath = fullfile(cfg.inputfolder, 'pva_830.txt');
    cfg.heightfilepath = first_existing_file(cfg.inputfolder, ...
        {'height.txt', 'depth_raw.txt', 'height_noised.txt'});
    cfg.stdfilepath = fullfile(cfg.inputfolder, 'std_830.txt');
    cfg.rangefilepath = first_existing_file(cfg.inputfolder, ...
        {'range.txt', 'rangedata_noised.txt'});

    cfg.pureinsfilepath = fullfile(cfg.outputfolder, 'PureIns.nav');
    cfg.pureinsfilepath1 = cfg.pureinsfilepath;
    % cfg.odofilepath = '';
    cfg.rangefile1path = fullfile(cfg.inputfolder, 'range1.txt');
    cfg.rangefile2path = fullfile(cfg.inputfolder, 'range2.txt');
    cfg.rangefile3path = fullfile(cfg.inputfolder, 'range3.txt');
    cfg.truthpath = fullfile(cfg.referencefolder, 'truth.nav');
    %% configure
    cfg.usegnssvel = false;
    cfg.useodonhc = false;
    cfg.odoupdaterate = 1; % [Hz]

    %% initial information
    
    % 选择计算时间段
    initial_truth = read_first_numeric_row(cfg.truthpath, 11);
    cfg.starttime = initial_truth(2);
    cfg.endtime = inf;
    cfg.initpos = initial_truth(3:5)'; % [deg, deg, m]
    cfg.initvel = initial_truth(6:8)'; % [m/s]
    cfg.initatt = initial_truth(9:11)'; % [deg]

    cfg.initposstd = [0.005; 0.004; 0.008]; %[m]
    cfg.initvelstd = [0.003; 0.004; 0.004]; %[m/s]
    cfg.initattstd = [0.003; 0.003; 0.023]; %[deg]

    % cfg.initposstd = [0.005; 0.004; 0.008]; %[m]
    % cfg.initvelstd = [0.002; 0.002; 0.001]; %[m/s]
    % cfg.initattstd = [0.003; 0.003; 0.008]; %[deg]

    cfg.initgyrbias = [0; 0; 0]; % [deg/h]
    cfg.initaccbias = [0; 0; 0]; % [mGal]
    cfg.initgyrscale = [0; 0; 0]; % [ppm]
    cfg.initaccscale = [0; 0; 0]; % [ppm]

    cfg.initgyrbiasstd = [0.01; 0.01; 0.01]; % [deg/h]
    cfg.initaccbiasstd = [7; 7; 7]; % [mGal]
    cfg.initgyrscalestd = [10; 10; 10]; % [ppm]
    cfg.initaccscalestd = [10; 10; 10]; % [ppm]

    cfg.gyrarw = 0.0005; % [deg/sqrt(h)] 角度随机游走
    cfg.accvrw = 10e-6; % [m/s/sqrt(h)] 加速度计随机游走
    cfg.gyrbiasstd = 0.01; % [deg/h] 陀螺仪零偏标准差
    cfg.accbiasstd = 7; % [mGal] 加速度计零偏标准差
    cfg.gyrscalestd = 10; % [ppm] 刻度系数标准差
    cfg.accscalestd = 10; % [ppm] 
    cfg.corrtime = 1; % [h] 时间相关系数，衡量系统误差随时间相关程度的重要指标

    %% install parameters 安装参数
    % cfg.antlever = [0.65; 0.048;0.9]; % [m]
    % cfg.antlever = [0.136; -0.301; -0.184]; % [m]
    cfg.odolever = [0; 0; 0]; %[m]
    cfg.installangle = [0; 0; 0]; %[deg]

    %% ODO/NHC measurement noise 观测噪声
    cfg.odonhc_measnoise = [0.1; 0.1; 0.1]; % [m/s]
    %% convert unit to standard unit (单位转换)
    cfg.initpos(1) = cfg.initpos(1) * param.D2R;
    cfg.initpos(2) = cfg.initpos(2) * param.D2R;
    cfg.initatt = cfg.initatt * param.D2R;

    [rm, rn] = getRmRn(cfg.initpos(1) , param);
    DR = diag([rm + cfg.initpos(3), (rn + cfg.initpos(3))*cos(cfg.initpos(1)), -1]);
    cfg.initposstd = DR^-1*cfg.initposstd ;
    
    cfg.initattstd = cfg.initattstd * param.D2R;

    cfg.initgyrbias = cfg.initgyrbias * param.D2R / 3600;
    cfg.initaccbias = cfg.initaccbias * 1e-5;
    cfg.initgyrscale = cfg.initgyrscale * 1e-6;
    cfg.initaccscale = cfg.initaccscale * 1e-6;
    
    cfg.initgyrbiasstd = cfg.initgyrbiasstd * param.D2R / 3600;
    cfg.initaccbiasstd = cfg.initaccbiasstd * 1e-5;
    cfg.initgyrscalestd = cfg.initgyrscalestd * 1e-6;
    cfg.initaccscalestd = cfg.initaccscalestd * 1e-6;

    cfg.gyrarw = cfg.gyrarw * param.D2R / 60;
    cfg.accvrw = cfg.accvrw / 60;
    cfg.gyrbiasstd = cfg.gyrbiasstd * param.D2R / 3600;
    cfg.accbiasstd = cfg.accbiasstd * 1e-5;
    cfg.gyrscalestd = cfg.gyrscalestd * 1e-6;
    cfg.accscalestd = cfg.accscalestd * 1e-6;
    cfg.corrtime = cfg.corrtime * 3600;

    cfg.installangle = cfg.installangle * param.D2R;
    cfg.cbv = euler2dcm(cfg.installangle);

end

function path = first_existing_file(folder, candidates)
    path = fullfile(folder, candidates{1});
    for index = 1:numel(candidates)
        candidate = fullfile(folder, candidates{index});
        if isfile(candidate)
            path = candidate;
            return;
        end
    end
end

function row = read_first_numeric_row(path, minimum_columns)
    file_id = fopen(path, 'rt');
    if file_id < 0
        error('无法读取实测真值文件：%s', path);
    end
    cleanup = onCleanup(@() fclose(file_id));
    row = [];
    while ~feof(file_id) && isempty(row)
        line = strtrim(fgetl(file_id));
        if ~isempty(line)
            row = sscanf(line, '%f')';
        end
    end
    if numel(row) < minimum_columns
        error('实测真值首行至少需要 %d 列：%s', minimum_columns, path);
    end
end

