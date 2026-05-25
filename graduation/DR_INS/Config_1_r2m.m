function cfg = Config_1_r2m
    param = Param();
    %% filepath

    cfg.outputfolder = 'D:\Github\KF-GINS-Matlab\graduation\DR_INS\output';

    % 检查文件夹是否存在（'dir' 表示明确检查是否为文件夹）
    if ~exist(cfg.outputfolder, 'dir')
        mkdir(cfg.outputfolder); % 如果不存在，则递归创建该路径
        disp(['已成功创建输出文件夹：', cfg.outputfolder]);
    else
        disp(['输出文件夹已存在，无需创建：', cfg.outputfolder]);
    end
    filepath='D:\Github\KF-GINS-Matlab\graduation\DR_INS\input';
    cfg.imufilepath = [filepath,'/imu_data.txt'];
    cfg.truthpath=[filepath,'/truth.nav'];
    cfg.rangefile1path = [filepath,'/range1.txt'];
    cfg.rangefile2path = [filepath,'/range2.txt'];
    cfg.rangefile3path = [filepath,'/range3.txt'];
    %% configure
    cfg.usegnssvel = false;
    cfg.useodonhc = false;
    cfg.odoupdaterate = 1; % [Hz]

    %% initial information
    % 选择计算时间段
    glvs
    cfg.starttime = 0;
    cfg.endtime = cfg.starttime + 5000;
    % 初始状态
    all_lines = load(cfg.truthpath); 
    pva0 = all_lines(1,:);
    cfg.initpos = pva0(3:5)'+ d2r([0.005; 0.004; 0.008])/2/glv.Re; % [deg, deg, m]
    cfg.initvel = pva0(6:8)'+[0.002; 0.002; 0.002]; % [m/s]
    cfg.initatt = pva0(9:11)'+[0.008; 0.008; 0.075]/2; % [deg]

    cfg.initgyrbias = [0; 0; 0]; % [deg/h]
    cfg.initaccbias = [0; 0; 0]; % [mGal]
    cfg.initgyrscale = [0; 0; 0]; % [ppm]
    cfg.initaccscale = [0; 0; 0]; % [ppm]

    % 初始协方差
    cfg.initposstd = [0.005; 0.004; 0.008]; %[m]
    cfg.initvelstd = [0.003; 0.004; 0.004]; %[m/s]
    cfg.initattstd = [0.008; 0.008; 0.075]; %[deg]
    % 参数设置
    eb = 0.01;
    db  = 20;
    web = 0.0005;
    wdb = 10e-6;

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
