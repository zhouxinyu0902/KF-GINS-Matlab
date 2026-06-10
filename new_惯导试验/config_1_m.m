function cfg = config_1_m(in_dir)
    param = Param();
    
    %% filepath - 基于传入的 base_dir 动态拼接路径
    % in_dir  = fullfile(base_dir, 'input');
    % out_dir = fullfile(base_dir, 'output');
    
    cfg.imufilepath    = fullfile(in_dir, 'IMU_120.txt');
    cfg.gnssfilepath   = fullfile(in_dir, 'pva_830.txt');
    cfg.heightfilepath = fullfile(in_dir, 'height.txt');
    cfg.stdfilepath    = fullfile(in_dir, 'std_830.txt');
    
 
    % cfg.odofilepath = '';
    cfg.rangefile1path = fullfile(in_dir, 'range1.txt');
    cfg.rangefile2path = fullfile(in_dir, 'range2.txt');
    cfg.rangefile3path = fullfile(in_dir, 'range3.txt');
    
    % cfg.truthpath    = fullfile(in_dir, 'truth_inte.txt');
    cfg.truthpath      = fullfile(in_dir, 'truth.nav');
    
    %% configure
    cfg.usegnssvel = false;
    cfg.useodonhc = false;
    cfg.odoupdaterate = 1; % [Hz]

    
    %% initial information
    pva_830 = importdata(cfg.gnssfilepath);
    % pva_120 = importdata([path,'/pva_RS.txt']);
    pva_120 = importdata([in_dir,'/pva_file.txt']);
    % 6378137*(pva_830(1,3)-pva_120(1,3))/180*pi
    % (pva_830(1,2)-pva_120(1,2))/180*pi
    % 选择计算时间段
    id = 1;
    cfg.starttime = pva_830(id,2);
    cfg.endtime = pva_830(end,2);
    % cfg.endtime = cfg.starttime + 5000;
    % 初始状态

    cfg.initpos = pva_830(id,3:5)'; % [deg, deg, m]
    cfg.initvel = [0; 0; 0]; % [m/s]
    cfg.initatt = pva_120(id,9:11)'; % [deg]
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
    % wdb=10e-7*1e5/3600;

    % eb=0.05;
    % db=10;
    % web=0.003;
    % wdb=10e-5*1e5/3600;
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
    
    % [rm, rn] = getRmRn(cfg.initpos(1), param);
    % DR = diag([rm + cfg.initpos(3), (rn + cfg.initpos(3))*cos(cfg.initpos(1)), -1]);
    % cfg.initposstd = DR^-1 * cfg.initposstd;
    
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