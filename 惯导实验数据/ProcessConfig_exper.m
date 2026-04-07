% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2023.3.3
% -------------------------------------------------------------------------

function cfg = ProcessConfig_exper()
    param = Param();
    %% filepath
    cfg.imufilepath = 'D:\GitHub\KF-GINS-Matlab\惯导实验数据/input/IMU_120.txt';
    cfg.gnssfilepath = 'D:\GitHub\KF-GINS-Matlab\惯导实验数据/input/pva_830.txt';
    cfg.heightfilepath = 'D:\GitHub\KF-GINS-Matlab\惯导实验数据/input/height.txt';
    cfg.stdfilepath = 'D:\GitHub\KF-GINS-Matlab\惯导实验数据/input/std_830.txt';

    
    cfg.pureinsfilepath = 'D:\GitHub\KF-GINS-Matlab\惯导实验数据/output/NavResult-pureins-height.nav';
    cfg.pureinsfilepath1 = 'D:\GitHub\KF-GINS-Matlab\惯导实验数据/output/NavResult-pureins.nav';
    % cfg.odofilepath = '';
    cfg.rangefile1path = 'D:\GitHub\KF-GINS-Matlab\惯导实验数据/input/range1.txt';
    cfg.rangefile2path = 'D:\GitHub\KF-GINS-Matlab\惯导实验数据/input/range2.txt';
    cfg.rangefile3path = 'D:\GitHub\KF-GINS-Matlab\惯导实验数据/input/range3.txt';
    cfg.outputfolder = 'D:\GitHub\KF-GINS-Matlab\惯导实验数据/output';
    % cfg.truthpath='D:\GitHub\KF-GINS-Matlab\惯导实验数据/input/truth_inte.txt';
    cfg.truthpath='D:\GitHub\KF-GINS-Matlab\惯导实验数据/output/truth.nav';
    %% configure
    cfg.usegnssvel = false;
    cfg.useodonhc = false;
    cfg.odoupdaterate = 1; % [Hz]

    %% initial information
    
    % 选择计算时间段
    cfg.starttime = 122235;
    cfg.endtime = cfg.starttime + 5000;
    cfg.initpos = [36.40042005;120.68981831;15.25]; % [deg, deg, m]
    cfg.initvel = [0; 0; 0]; % [m/s]
    cfg.initatt = [1.743;1.516;322.463]; % [deg]

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

