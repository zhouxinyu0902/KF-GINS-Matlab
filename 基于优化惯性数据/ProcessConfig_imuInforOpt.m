% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2023.3.3
% -------------------------------------------------------------------------

function cfg = ProcessConfig_imuInforOpt(filepath)
    param = Param();
    %% filepath
    cfg.truthpath = [filepath,'\input\truth.nav'];
    cfg.imufilepath = [filepath,'\input\imu.nav'];
    cfg.gnssfilepath = [filepath,'\input\gnss.txt'];
    cfg.depthfilepath = [filepath,'\input\height-markov-10Hz.txt'];
    cfg.LBLfilepath = [filepath,'\input\LBL.txt'];
    cfg.outputfolder = [filepath, '/output'];
    for i = 1:9
        fieldname = sprintf('rangefile%dpath', i);
        filename = sprintf('range_static_%d.txt', i);
        cfg.(fieldname) = fullfile(filepath, 'input', filename);
    end
    % cfg.odofilepath = '';
    %% configure
    cfg.usegnssvel = false;
    cfg.useodonhc = false;
    cfg.odoupdaterate = 1; % [Hz]

    %% initial information
    cfg.starttime = 0;
    cfg.endtime = 0+3600;
    pva = load(cfg.truthpath);% 真实pva0
    avp0 = pvaNED2ENU(pva(1,:)); % 真实avp0
    %%%%%%%%%%误差设置%%%%%%%%%%%%%%
    avperr = avperrset([0.003,0.003,0.023]*60,0.003,1);
    % avperr = avperrset([0.001,0.001,0.004]*60,0.003,1);
    % avperr = avperrset([0.001,0.001,0.002]*60,0.001,0.05);
    avp00 = avpadderr(avp0,avperr); % 误差avp00
    pva00 = avpENU2NED(avp00')';% 误差pva00

    cfg.initposstd = avperr(7:9)*1; %[r,r,m]
    cfg.initvelstd = avperr(4:6)*1; %[m/s]
    cfg.initattstd = avperr(1:3)*2; %[rad]

    cfg.initpos = avp00(7:9); %[r,r,m]
    cfg.initvel = pva00(6:8); %[m/s]
    cfg.initatt = pva00(9:11)*param.D2R; %[rad]
    % cfg.starttime = 122215.56;
    % cfg.endtime = cfg.starttime+5000;
    % cfg.initpos = [36.40042003;120.68981831;15.25]; % [deg, deg, m]
    % cfg.initvel = [0; 0; 0]; % [m/s]
    % cfg.initatt = [1.743;1.515;322.463]; % [deg]
    % % 选择计算时间段
    % cfg.starttime = 122235;
    % cfg.endtime = cfg.starttime+5000;
    % cfg.initpos = [36.40042005;120.68981831;15.25]; % [deg, deg, m]
    % cfg.initvel = [0; 0; 0]; % [m/s]
    % cfg.initatt = [1.743;1.516;322.463]; % [deg]
    % 
    % cfg.initposstd = [0.005; 0.004; 0.008]; %[m]
    % cfg.initvelstd = [0.003; 0.004; 0.004]; %[m/s]
    % cfg.initattstd = [0.003; 0.003; 0.023]; %[deg]
    % 
    % % cfg.initposstd = [0.005; 0.004; 0.008]; %[m]
    % % cfg.initvelstd = [0.002; 0.002; 0.001]; %[m/s]
    % % cfg.initattstd = [0.003; 0.003; 0.008]; %[deg]
    % 
    cfg.initgyrbias = [0; 0; 0]; % [deg/h]
    cfg.initaccbias = [0; 0; 0]; % [mGal]
    cfg.initgyrscale = [0; 0; 0]; % [ppm]
    cfg.initaccscale = [0; 0; 0]; % [ppm]

    cfg.initgyrbiasstd = [0.1; 0.1; 0.1]; % [deg/h]
    cfg.initaccbiasstd = [300; 300; 300]; % [mGal]
    cfg.initgyrscalestd = [10; 10; 10]; % [ppm]
    cfg.initaccscalestd = [10; 10; 10]; % [ppm]

    cfg.gyrarw = 0.01; % [deg/sqrt(h)] 角度随机游走
    cfg.accvrw = 1; % [m/s/sqrt(h)] 加速度计随机游走
    cfg.gyrbiasstd = 0.1; % [deg/h] 陀螺仪零偏标准差
    cfg.accbiasstd = 300; % [mGal] 加速度计零偏标准差
    cfg.gyrscalestd = 10; % [ppm] 刻度系数标准差
    cfg.accscalestd = 10; % [ppm] 
    cfg.corrtime = 0; % [h] 时间相关系数，衡量系统误差随时间相关程度的重要指标

    %% install parameters 安装参数
    % cfg.antlever = [0.65; 0.048;0.9]; % [m]
    % cfg.antlever = [0.136; -0.301; -0.184]; % [m]
    cfg.odolever = [0; 0; 0]; %[m]
    cfg.installangle = [0; 0; 0]; %[deg]

    %% ODO/NHC measurement noise 观测噪声
    cfg.odonhc_measnoise = [0.1; 0.1; 0.1]; % [m/s]
    %% convert unit to standard unit (单位转换)
    % cfg.initpos(1) = cfg.initpos(1) * param.D2R;
    % cfg.initpos(2) = cfg.initpos(2) * param.D2R;
    % cfg.initatt = cfg.initatt * param.D2R;

    % [rm, rn] = getRmRn(cfg.initpos(1) , param);
    % DR = diag([rm + cfg.initpos(3), (rn + cfg.initpos(3))*cos(cfg.initpos(1)), -1]);
    % cfg.initposstd = DR^-1*cfg.initposstd ;
    
    % cfg.initattstd = cfg.initattstd * param.D2R;

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

