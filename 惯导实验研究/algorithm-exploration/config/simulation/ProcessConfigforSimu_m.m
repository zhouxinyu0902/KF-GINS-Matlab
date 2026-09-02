% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2023.3.3
% -------------------------------------------------------------------------

function cfg = ProcessConfigforSimu_m(filepath)
    param = Param();
    %% filepath
    config_dir = fileparts(mfilename('fullpath'));
    topic_dir = fileparts(fileparts(config_dir));
    inertial_research_dir = fileparts(topic_dir);
    project_root = fileparts(inertial_research_dir);
    cfg.dataroot = fullfile(project_root, 'data', 'inertial-experiment', ...
        'algorithm-exploration');
    cfg.simulationroot = fullfile(cfg.dataroot, 'input', 'simulation');
    [~, case_name] = fileparts(filepath);
    cfg.outputfolder = fullfile(cfg.dataroot, 'navigation-results', ...
        'simulation', case_name);
    cfg.figurefolder = fullfile(cfg.dataroot, 'figures-tables', ...
        'simulation', case_name);
    cfg.imufilepath = [filepath,'/IMU_120.txt'];
    cfg.rangefile1path = [filepath,'/range1.txt'];
    cfg.rangefile2path =[filepath,'/range2.txt'] ;
    cfg.rangefile3path =[filepath,'/range3.txt'] ;
    

    cfg.truthpath=[filepath,'/truth.txt'];
    %% configure
    cfg.usegnssvel = false;
    cfg.useodonhc = false;
    cfg.odoupdaterate = 1; % [Hz]

    %% initial information
    % 选择计算时间段
    cfg.starttime = 0;
    cfg.endtime = cfg.starttime+5000;
    % 初始状态
    cfg.initpos = [36.4004200;120.6898183;15.25]; % [deg, deg, m]
    cfg.initvel = [0; 0; 0]; % [m/s]
    if filepath(end)=='1'
        cfg.initatt = [0.003; 0.003; 270.03]; % [deg]
    elseif filepath(end)=='2'
        cfg.initatt = [0.003; 0.003; 240.03]; % [deg]
    elseif filepath(end)=='3'
        cfg.initatt = [0.003; 0.003; 180.03]; % [deg]
    elseif filepath(end)=='4'
        cfg.initatt = [0.003; 0.003; 255.03]; % [deg]
    else
        cfg.initatt = [0.003; 0.003; 323.03]; % [deg]
    end
    cfg.initgyrbias = [0; 0; 0]; % [deg/h]
    cfg.initaccbias = [0; 0; 0]; % [mGal]
    cfg.initgyrscale = [0; 0; 0]; % [ppm]
    cfg.initaccscale = [0; 0; 0]; % [ppm]

    % 初始协方差
    cfg.initposstd = [0.005; 0.004; 0.008]; %[m]
    cfg.initvelstd = [0.003; 0.004; 0.004]; %[m/s]
    cfg.initattstd = [0.008; 0.008; 0.075]; %[deg]
    % 参数设置
    eb=0.01;
    db=7;
    web=0.0005;
    wdb=10e-6;

    % eb=0.003;
    % db=7;
    % web=0.0003;
    % wdb=10e-7;

    % eb=0.05;
    % db=10;
    % web=0.003;
    % wdb=10e-5;
    cfg.initgyrbiasstd = [eb; eb; eb]; % [deg/h]
    cfg.initaccbiasstd = [db; db; db]; % [mGal]
    
    % 过程噪声
    cfg.gyrarw = web; % [deg/sqrt(h)] 角度随机游走
    cfg.accvrw = wdb; % [m/s/sqrt(h)] 加速度计随机游走
    cfg.gyrbiasstd = eb; % [deg/h] 陀螺仪零偏标准差
    cfg.accbiasstd = db; % [mGal] 加速度计零偏标准差

    % 不相关参数
    cfg.initgyrscalestd = [10; 10; 10]; % [ppm]
    cfg.initaccscalestd = [10; 10; 10]; % [ppm]

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

    % [rm, rn] = getRmRn(cfg.initpos(1) , param);
    % DR = diag([rm + cfg.initpos(3), (rn + cfg.initpos(3))*cos(cfg.initpos(1)), -1]);
    % cfg.initposstd = DR^-1*cfg.initposstd ;
    
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

