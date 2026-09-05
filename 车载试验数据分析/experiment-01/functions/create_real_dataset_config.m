function cfg = create_real_dataset_config( ...
        input_dir, position_unit, end_time_override)
%CREATE_REAL_DATASET_CONFIG 构造全实测数据处理专题的统一配置。
% input_dir 指向 experiment-01/case-XX/input。
% position_unit 可取 "rad" 或 "m"，仅决定水平位置误差状态的单位。


if nargin < 2 || isempty(position_unit)
    position_unit = "rad";
end
if nargin < 3
    end_time_override = [];
end

position_unit = string(position_unit);
if ~ismember(position_unit, ["rad", "m"])
    error('position_unit 必须为 "rad" 或 "m"。');
end

param = Param();

%% 文件路径
cfg.inputfolder = input_dir;
cfg.imufilepath = fullfile(input_dir, 'IMU_120.txt');
cfg.gnssfilepath = fullfile(input_dir, 'pva_830.txt');
cfg.heightfilepath = fullfile(input_dir, 'height.txt');
cfg.stdfilepath = fullfile(input_dir, 'std_830.txt');
cfg.attitudefilepath = fullfile(input_dir, 'pva_file.txt');
cfg.rangefile1path = fullfile(input_dir, 'range1.txt');
cfg.rangefile2path = fullfile(input_dir, 'range2.txt');
cfg.rangefile3path = fullfile(input_dir, 'range3.txt');

cfg.truthpath = fullfile(input_dir, 'truth.nav');

%% 运行开关
cfg.usegnssvel = false;
cfg.useodonhc = false;
cfg.odoupdaterate = 1; % [Hz]

%% 初始状态与处理时段
pva_830 = importdata(cfg.gnssfilepath);
pva_120 = importdata(cfg.attitudefilepath);
initial_index = 1;
cfg.starttime = pva_830(initial_index, 2);
cfg.endtime = pva_830(end, 2);
if ~isempty(end_time_override)
    cfg.endtime = min(cfg.endtime, end_time_override);
end

cfg.initpos = pva_830(initial_index, 3:5)'; % [deg, deg, m]
cfg.initvel = [0; 0; 0];                   % [m/s]
cfg.initatt = pva_120(initial_index, 9:11)'; % [deg]
cfg.initgyrbias = zeros(3, 1);             % [deg/h]
cfg.initaccbias = zeros(3, 1);             % [mGal]
cfg.initgyrscale = zeros(3, 1);            % [ppm]
cfg.initaccscale = zeros(3, 1);            % [ppm]

%% 初始协方差与 IMU 随机模型
% cfg.initposstd = [0.005; 0.004; 0.008];    % [m]
% cfg.initvelstd = [0.003; 0.004; 0.004];    % [m/s]
% cfg.initattstd = [0.008; 0.008; 0.075];    % [deg]

cfg.initposstd = [0.005; 0.004; 0.008]; %[m]
cfg.initvelstd = [0.003; 0.004; 0.004]; %[m/s]
cfg.initattstd = [0.003; 0.003; 0.023]; %[deg]

gyro_bias_std = 0.01;
acc_bias_std = 7;
gyro_arw = 0.0005;
acc_vrw = 10e-6;

cfg.initgyrbiasstd = repmat(gyro_bias_std, 3, 1); % [deg/h]
cfg.initaccbiasstd = repmat(acc_bias_std, 3, 1);  % [mGal]
cfg.gyrarw = gyro_arw;                            % [deg/sqrt(h)]
cfg.accvrw = acc_vrw;                             % [m/s/sqrt(h)]
cfg.gyrbiasstd = gyro_bias_std;                   % [deg/h]
cfg.accbiasstd = acc_bias_std;                    % [mGal]
cfg.initgyrscalestd = 10 * ones(3, 1);            % [ppm]
cfg.initaccscalestd = 10 * ones(3, 1);            % [ppm]
cfg.gyrscalestd = 10;                             % [ppm]
cfg.accscalestd = 10;                             % [ppm]
cfg.corrtime = 1;                                 % [h]

%% 安装参数与观测噪声
cfg.odolever = zeros(3, 1);                       % [m]
cfg.installangle = zeros(3, 1);                   % [deg]
cfg.odonhc_measnoise = 0.1 * ones(3, 1);          % [m/s]

%% 转为内部标准单位
cfg.initpos(1:2) = cfg.initpos(1:2) * param.D2R;
cfg.initatt = cfg.initatt * param.D2R;

if position_unit == "rad"
    [rm, rn] = getRmRn(cfg.initpos(1), param);
    position_scale = diag([rm + cfg.initpos(3), ...
        (rn + cfg.initpos(3)) * cos(cfg.initpos(1)), -1]);
    cfg.initposstd = position_scale \ cfg.initposstd;
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
cfg.position_unit = char(position_unit);
end
