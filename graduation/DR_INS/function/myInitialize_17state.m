function [kf, navstate] = myInitialize_17state(cfg)

%% Kalman filter
kf.RANK = 17;
kf.NOISE_RANK = 12;

kf.P = zeros(kf.RANK);
kf.Qc = zeros(kf.NOISE_RANK);
kf.x = zeros(kf.RANK,1);

%% Process noise
kf.Qc(1:3,1:3) = cfg.accvrw^2 * eye(3);
kf.Qc(4:6,4:6) = cfg.gyrarw^2 * eye(3);

kf.Qc(7:9,7:9) = ...
    2*cfg.gyrbiasstd^2/cfg.corrtime*eye(3);

kf.Qc(10:12,10:12) = ...
    2*cfg.accbiasstd^2/cfg.corrtime*eye(3);

%% Initial covariance
kf.P(1:3,1:3) = diag(cfg.initposstd.^2);
kf.P(4:6,4:6) = diag(cfg.initvelstd.^2);
kf.P(7:9,7:9) = diag(cfg.initattstd.^2);
kf.P(10:12,10:12) = diag(cfg.initgyrbiasstd.^2);
kf.P(13:15,13:15) = diag(cfg.initaccbiasstd.^2);

% DVL scale factor
kf.P(16,16) = cfg.initdvlscalestd^2;

% DVL yaw installation angle
kf.P(17,17) = cfg.initdvlyawstd^2;

kf.P0 = kf.P;

%% Navigation state
navstate.time = cfg.starttime;
navstate.pos = cfg.initpos;
navstate.vel = cfg.initvel;
navstate.att = cfg.initatt;

navstate.cbn = euler2dcm(cfg.initatt);
navstate.qbn = euler2quat(cfg.initatt);

navstate.gyrbias = cfg.initgyrbias;
navstate.accbias = cfg.initaccbias;
navstate.gyrscale = cfg.initgyrscale;
navstate.accscale = cfg.initaccscale;

%% DVL nominal calibration parameters
navstate.dvlscale = cfg.initdvlscale;
navstate.dvlyaw = cfg.initdvlyaw;

param = Param();
[navstate.Rm,navstate.Rn] = getRmRn(cfg.initpos(1),param);
navstate.gravity = getGravity(cfg.initpos);

end