% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2023.3.3
% -------------------------------------------------------------------------

function cfg = Config(dataset_id, position_unit)
    if nargin < 1 || isempty(dataset_id)
        dataset_id = 'run-0817';
    end
    if nargin < 2 || isempty(position_unit)
        position_unit = "rad";
    end
    position_unit = lower(string(position_unit));
    if ~ismember(position_unit, ["rad", "m"])
        error('position_unit 只能取 "rad" 或 "m"。');
    end
    param = Param();
    %% filepath
    data_paths = experiment03_dataset_paths(dataset_id);
    cfg.datasetname = data_paths.dataset_name;
    cfg.positionunit = char(position_unit);
    cfg.dataroot = data_paths.data;
    cfg.inputfolder = data_paths.input;
    cfg.outputfolder = data_paths.output;
    cfg.figurefolder = data_paths.artifacts;

    cfg.imufilepath = fullfile(cfg.inputfolder, 'imu_120.txt');
    cfg.gnssfilepath = fullfile(cfg.inputfolder, 'pva_830.txt');
    cfg.heightfilepath = fullfile(cfg.inputfolder, 'depth_raw.txt');
    cfg.stdfilepath = fullfile(cfg.inputfolder, 'std_830.txt');

    cfg.pureinsfilepath = fullfile(cfg.outputfolder, 'PureIns.nav');
    cfg.pureinsfilepath1 = cfg.pureinsfilepath;
    % cfg.odofilepath = '';
    cfg.rangefile1path = fullfile(cfg.inputfolder, 'range1.txt');
    cfg.rangefile2path = fullfile(cfg.inputfolder, 'range2.txt');
    cfg.rangefile3path = fullfile(cfg.inputfolder, 'range3.txt');
    cfg.truthpath = fullfile(cfg.inputfolder, 'pva_830.txt');
    cfg.rangefilepath = fullfile(cfg.inputfolder, 'range.txt');
    data = yaml.ReadYaml(fullfile(cfg.dataroot, 'initial_state.yaml'));
    %% configure
    cfg.usegnssvel = false;
    cfg.useodonhc = false;
    cfg.odoupdaterate = 1; % [Hz]

    %% initial information
    cfg.initpos = cell2mat(data.initpos)';
    cfg.initvel = cell2mat(data.initvel)';
    cfg.initatt = cell2mat(data.initatt)';
    % 选择计算时间段
    cfg.starttime = data.capture_time;
    switch (dataset_id)
        case 'run-0817'
            lastingtime = 10000;
        case 'run-0818'
            lastingtime = 12000;
        case 'run-0818-noon'
            lastingtime = 17000;
    end
    % cfg.starttime = 103951.095;
    cfg.endtime = cfg.starttime + lastingtime;
    % cfg.initpos = [36.40042454, 120.68982814, 6.5900]'; % [deg, deg, m]
    % cfg.initvel = [0; 0; 0]; % [m/s]
    % cfg.initatt = [+0.787000, +1.413000, -42.216000]'; % [deg]

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

    if position_unit == "rad"
        [rm, rn] = getRmRn(cfg.initpos(1), param);
        DR = diag([rm + cfg.initpos(3), ...
            (rn + cfg.initpos(3))*cos(cfg.initpos(1)), -1]);
        cfg.initposstd = DR^-1*cfg.initposstd;
    end
    
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

