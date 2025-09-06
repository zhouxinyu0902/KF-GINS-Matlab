% -------------------------------------------------------------------------
% KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
%
% Copyright (C) 2024, i2Nav Group, Wuhan University
%
%  Author : Liqiang Wang
% Contact : wlq@whu.edu.cn
%    Date : 2023.3.3
% -------------------------------------------------------------------------

function cfg = ProcessConfigsimu(filepath)

    param = Param();

    %% filepath
    cfg.truthpath = [filepath,'\input\truth.nav'];
    cfg.imufilepath = [filepath,'\input\imu.nav'];
    cfg.gnssfilepath = [filepath,'\input\gnss.txt'];
    cfg.depthfilepath = [filepath,'\input\height-1Hz.txt'];
    cfg.LBLfilepath = [filepath,'\input\LBL.txt'];
 
    cfg.outputfolder = [filepath, '\output'];
    cfg.rangefile1path = [filepath,'\input\range_static_1.txt'];
    cfg.rangefile2path = [filepath,'\input\range_static_2.txt'];
    cfg.rangefile3path = [filepath,'\input\range_static_3.txt'];
    cfg.rangefile4path = [filepath,'\input\range_static_4.txt'];
    cfg.rangefile5path = [filepath,'\input\range_moving.txt'];
    %% configure
    cfg.usegnss = false;
    cfg.userange = true;
    cfg.usepureins = false;
    %% 初始信息
    cfg.starttime = 0;
    cfg.endtime = 0+3600;
    pva0=load(cfg.truthpath);% 真实pva0
    avp0 = pvaNED2ENU(pva0(1,:)); % 真实avp0
    %%%%%%%%%%误差设置%%%%%%%%%%%%%%
    avperr = avperrset([0.005,0.005,0.03]*60,0.01,0.1);
    avp00 = avpadderr(avp0,avperr); % 误差avp00
    pva00 = avpENU2NED(avp00')';% 误差pva00

    cfg.initposstd = avperr(7:9); %[r,r,m]
    cfg.initvelstd = avperr(4:6); %[m/s]
    cfg.initattstd = avperr(1:3); %[rad]

    cfg.initpos = avp00(7:9); %[r,r,m]
    cfg.initvel = pva00(6:8) ; %[m/s]
    cfg.initatt = pva00(9:11)*param.D2R; %[rad]

    % % % 初始误差设置
    % cfg.initposstd = [1/param.WGS84_RA; 1/param.WGS84_RA; 1]; %[rad]
    % cfg.initvelstd = [0.01; 0.01; 0.01]; %[m/s]
    % cfg.initattstd = [0.008; 0.008; 0.06]; %[deg]
    % % 误差叠加
    % cfg.initpos = pva0(1,3:5)' +[cfg.initposstd(1:2)*param.R2D;cfg.initposstd(3)]; %[deg]
    % cfg.initvel = pva0(1,6:8)' +cfg.initvelstd; %[m/s]
    % cfg.initatt = pva0(1,9:11)'-[cfg.initattstd(1:2);-cfg.initattstd(3)]; %[deg]
    % cfg.initpos(1) = cfg.initpos(1) * param.D2R;
    % cfg.initpos(2) = cfg.initpos(2) * param.D2R;
    % cfg.initatt = cfg.initatt * param.D2R;
    % cfg.initattstd = cfg.initattstd * param.D2R;
    cfg.antlever = [0; 0; 0]; % [m]
    %% 滤波相关的参数
    cfg.initgyrbias = [0; 0; 0]; % [deg/h]
    cfg.initaccbias = [0; 0; 0]; % [mGal]
    cfg.initgyrscale = [0; 0; 0]; % [ppm]
    cfg.initaccscale = [0; 0; 0]; % [ppm]

    cfg.initgyrbiasstd = [0.003; 0.003; 0.003]; % [deg/h]
    cfg.initaccbiasstd = [7; 7; 7]; % [mGal]
    cfg.initgyrscalestd = [5; 5; 5]; % [ppm]
    cfg.initaccscalestd = [10; 10; 10]; % [ppm]
    
    cfg.gyrarw = 0.0003; % [deg/sqrt(h)] 角度随机游走
    cfg.accvrw = 1e-7; % [m/s/sqrt(h)] 加速度计随机游走
    cfg.gyrbiasstd = 0.003; % [deg/h] 陀螺仪零偏标准差
    cfg.accbiasstd = 7; % [mGal] 加速度计零偏标准差
    cfg.gyrscalestd = 5; % [ppm] 刻度系数标准差，即0.03%
    cfg.accscalestd = 10; % [ppm] 
    cfg.corrtime = 4; % [h] 时间相关系数，衡量系统误差随时间相关程度的重要指标
    %% install parameters 安装参数
    cfg.antlever = [0; 0; 0]; % [m]
    cfg.odolever = [0; 0; 0]; %[m]
    cfg.installangle = [0; 0; 0]; %[deg]
    %% convert unit to standard unit (单位转换)
      
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