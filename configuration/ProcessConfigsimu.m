% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2023.3.3
% -------------------------------------------------------------------------

function cfg = ProcessConfigsimu()

    param = Param();

    %% filepath
    % cfg.imufilepath = 'dataset-simu\imu-simu.txt';
    % cfg.gnssfilepath = 'dataset-simu\gnss-2m.nav';
    % cfg.depthfilepath = 'dataset-simu\depth.txt';
    % cfg.odofilepath = '';
    % 
    % cfg.rangefilepath = 'dataset-simu\range-5.txt';
    % cfg.rangefilepath2 = 'dataset-simu\range-3.txt';
    % cfg.compassfilepath = 'dataset-simu\compass.nav';

    % cfg.outputfolder = 'dataset-simu\result';

    cfg.imufilepath = 'dataset-simu-circle\imu-10.txt';
    cfg.rangefilepath = 'dataset-simu-circle\range-10m-0.1.txt';
    cfg.depthfilepath = 'dataset-simu-circle\depth-10.txt';
    cfg.outputfolder = 'dataset-simu-circle\result';
    %% configure
    cfg.usegnssvel = false;
    cfg.useodonhc = false;
    cfg.odoupdaterate = 1; % [Hz]

    %% initial information
    cfg.starttime = 0.005;
    % cfg.endtime = inf;
    cfg.endtime = 1200;

    % 仿真设置
    cfg.trueinitpos = [15;115;-1200]; % [deg, deg, m]
    cfg.trueinitvel = [0; 0; 0]; % [m/s]
    cfg.trueinitatt = [0; 0; 0]; % [deg]

    [rm, rn] = getRmRn(cfg.trueinitpos(1)*param.D2R, param);
    DR = diag([rm + cfg.trueinitpos(3), (rn + cfg.trueinitpos(3))*cos(cfg.trueinitpos(1)*param.D2R), -1]);    
    cfg.initposstd = DR^-1*[0.005; 0.004; 0.008]; %[m] 转为弧度
    cfg.initvelstd = [0.003; 0.004; 0.004]; %[m/s]
    cfg.initattstd = [0.003; 0.003; 0.023]; %[deg]

    dll = cfg.initposstd(1:2)*param.R2D;
    dh = cfg.initposstd(3);
    cfg.initpos = cfg.trueinitpos+[dll;dh]; % [deg, deg, m]
    cfg.initvel = cfg.trueinitvel+cfg.initvelstd; % [m/s]
    cfg.initatt = cfg.trueinitatt+cfg.initattstd; % [deg]

    
    % imu初始偏差
    cfg.initgyrbias = [0; 0; 0]; % [deg/h]
    cfg.initaccbias = [0; 0; 0]; % [mGal]
    cfg.initgyrscale = [0; 0; 0]; % [ppm]
    cfg.initaccscale = [0; 0; 0]; % [ppm]
    
    % imu噪声参数
    cfg.initgyrbiasstd = [0.027; 0.027; 0.027]; % [deg/h]
    cfg.initaccbiasstd = [15; 15; 15]; % [mGal]
    cfg.initgyrscalestd = [300; 300; 300]; % [ppm]
    cfg.initaccscalestd = [300; 300; 300]; % [ppm]

    cfg.gyrarw = 0.003; % [deg/sqrt(h)]
    cfg.accvrw = 0.03; % [m/s/sqrt(h)]
    cfg.gyrbiasstd = 0.027; % [deg/h]
    cfg.accbiasstd = 15; % [mGal]
    cfg.gyrscalestd = 300; % [ppm]
    cfg.accscalestd = 300; % [ppm]
    cfg.corrtime = 4; % [h]

    %% install parameters 安装参数
    cfg.antlever = [0.136; -0.301; -0.184]; % [m]
    cfg.odolever = [0; 0; 0]; %[m]
    cfg.installangle = [0; 0; 0]; %[deg]

    %% ODO/NHC measurement noise 观测噪声
    cfg.odonhc_measnoise = [0.1; 0.1; 0.1]; % [m/s]


    %% convert unit to standard unit (单位转换)
    cfg.initpos(1) = cfg.initpos(1) * param.D2R;
    cfg.initpos(2) = cfg.initpos(2) * param.D2R;
    cfg.initatt = cfg.initatt * param.D2R;

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

