function cfg = ProcessConfigPHINS()

    param = Param();

    %% filepath
    cfg.imufilepath = 'dataset-20230504/input/imu_NED.nav';
    cfg.LBLfilepath = 'dataset-20230504/input/LBL.nav';
    cfg.depthfilepath = 'dataset-20230504/input/depth-nav.nav';
    % cfg.rangefilepath = 'dataset-20230504/range_moving.txt';
    cfg.outputfolder = 'dataset-20230504/output';
    cfg.truthpath='dataset-20230504/input/truth.nav';
    %% initial information
    cfg.starttime = 1023;
    cfg.endtime = 1023+3600;
    
    % cfg.endtime = 4985;
    cfg.initpos = [15.8206477279060;115.147236896550;-4128.83740200000];
    cfg.initvel = [1.45142300000000;-0.355162000000000;-0.0135160000000000];
    cfg.initatt = [-2.41200000000000;-4.71900000000000;353.401001000000];
    

    cfg.initposstd = [0.005; 0.004; 0.008]; %[m]
    cfg.initvelstd = [0.003; 0.004; 0.004]; %[m/s]
    cfg.initattstd = [0.003; 0.003; 0.023]; %[deg]

    cfg.initgyrbias = [0; 0; 0]; % [deg/h]
    cfg.initaccbias = [0; 0; 0]; % [mGal]
    cfg.initgyrscale = [0; 0; 0]; % [ppm]
    cfg.initaccscale = [0; 0; 0]; % [ppm]

    cfg.initgyrbiasstd = [0.027; 0.027; 0.027]; % [deg/h]
    cfg.initaccbiasstd = [15; 15; 15]; % [mGal]
    cfg.initgyrscalestd = [300; 300; 300]; % [ppm]
    cfg.initaccscalestd = [300; 300; 300]; % [ppm]

    cfg.gyrarw = 0.003; % [deg/sqrt(h)] 角度随机游走
    cfg.accvrw = 0.03; % [m/s/sqrt(h)] 加速度计随机游走
    cfg.gyrbiasstd = 0.027; % [deg/h] 陀螺仪零偏标准差
    cfg.accbiasstd = 15; % [mGal] 加速度计零偏标准差
    cfg.gyrscalestd = 300; % [ppm] 刻度系数标准差，即0.03%
    cfg.accscalestd = 300; % [ppm] 
    cfg.corrtime = 4; % [h] 时间相关系数，衡量系统误差随时间相关程度的重要指标

    %% install parameters 安装参数
    cfg.antlever = [0; 0; 0]; % [m]
    cfg.odolever = [0; 0; 0]; %[m]
    cfg.installangle = [0; 0; 0]; %[deg]
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

