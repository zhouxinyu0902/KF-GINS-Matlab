function cfg = create_static_pureins_config(paths)
%CREATE_STATIC_PUREINS_CONFIG Build a pure INS configuration for one static batch.
%
% Initial position, velocity, and attitude all come from pva_initial.txt.
% Height is held at the initial value by run_static_pureins.

if nargin < 1 || isempty(paths)
    paths = setup_static_experiment();
end

param = Param();
pva_initial = readmatrix(paths.pva_initial_file, 'FileType', 'text');
if isempty(pva_initial) || size(pva_initial, 2) < 11
    error('create_static_pureins_config:InvalidInitialPVA', ...
        'pva_initial.txt must contain at least one row and 11 columns.');
end
pva_initial = pva_initial(1, :);
if any(~isfinite(pva_initial(1:11)))
    error('create_static_pureins_config:NonfiniteInitialPVA', ...
        'The first 11 values in pva_initial.txt must be finite.');
end

%% File paths
cfg.datasetname = paths.dataset_name;
cfg.inputfolder = paths.input;
cfg.outputfolder = paths.output;
cfg.imufilepath = paths.imu_file;
cfg.attitudefilepath = paths.pva_initial_file;
cfg.pureinsfilepath = paths.pureins_fixed_height_file;
cfg.pureins_fixed_height_filepath = paths.pureins_fixed_height_file;
cfg.pureins_zero_vel_fixed_height_filepath = ...
    paths.pureins_zero_vel_fixed_height_file;

%% Initial state and processing time
cfg.starttime = pva_initial(2);
cfg.endtime = inf;
cfg.initpos = pva_initial(3:5).';       % [deg, deg, m]
cfg.initvel = pva_initial(6:8).';       % [m/s]
cfg.initatt = pva_initial(9:11).';      % [deg]
cfg.fixedheight = cfg.initpos(3);       % [m]

cfg.initgyrbias = zeros(3, 1);         % [deg/h]
cfg.initaccbias = zeros(3, 1);         % [mGal]
cfg.initgyrscale = zeros(3, 1);        % [ppm]
cfg.initaccscale = zeros(3, 1);        % [ppm]

%% Initial covariance and IMU stochastic model (experiment-01 values)
cfg.initposstd = [0.005; 0.004; 0.008]; % [m]
cfg.initvelstd = [0.003; 0.004; 0.004]; % [m/s]
cfg.initattstd = [0.003; 0.003; 0.023]; % [deg]

cfg.initgyrbiasstd = 0.01 * ones(3, 1); % [deg/h]
cfg.initaccbiasstd = 7 * ones(3, 1);    % [mGal]
cfg.gyrarw = 0.0005;                    % [deg/sqrt(h)]
cfg.accvrw = 10e-6;                    % [m/s/sqrt(h)]
cfg.gyrbiasstd = 0.01;                  % [deg/h]
cfg.accbiasstd = 7;                     % [mGal]
cfg.initgyrscalestd = 10 * ones(3, 1); % [ppm]
cfg.initaccscalestd = 10 * ones(3, 1); % [ppm]
cfg.gyrscalestd = 10;                   % [ppm]
cfg.accscalestd = 10;                   % [ppm]
cfg.corrtime = 1;                       % [h]

%% Installation parameters retained for common initializer compatibility
cfg.odolever = zeros(3, 1);             % [m]
cfg.installangle = zeros(3, 1);         % [deg]
cfg.odonhc_measnoise = 0.1 * ones(3, 1); % [m/s]
cfg.usegnssvel = false;
cfg.useodonhc = false;
cfg.odoupdaterate = 1;
cfg.position_unit = 'rad';

%% Convert to the internal standard units
cfg.initpos(1:2) = cfg.initpos(1:2) * param.D2R;
cfg.initatt = cfg.initatt * param.D2R;

[rm, rn] = getRmRn(cfg.initpos(1), param);
position_scale = diag([rm + cfg.initpos(3), ...
    (rn + cfg.initpos(3)) * cos(cfg.initpos(1)), -1]);
cfg.initposstd = position_scale \ cfg.initposstd;

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
