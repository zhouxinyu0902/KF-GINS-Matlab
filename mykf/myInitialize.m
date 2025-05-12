function [kf, navstate] = myInitialize(cfg)
    
    % kalman parameters initialization
    kf.RANK = 15;
    kf.NOISE_RANK = 6;
    kf.P = zeros(kf.RANK, kf.RANK);
    kf.Qc = zeros(kf.NOISE_RANK, kf.NOISE_RANK);
    kf.x = zeros(kf.RANK, 1);

    % Qc
    kf.Qc(1:3, 1:3) = power(cfg.accvrw, 2) * eye(3, 3);
    kf.Qc(4:6, 4:6) = power(cfg.gyrarw, 2) * eye(3, 3);

    % P0
    kf.P(1:3, 1:3) = diag(power(cfg.initposstd, 2));
    kf.P(4:6, 4:6) = diag(power(cfg.initvelstd, 2));
    kf.P(7:9, 7:9) = diag(power(cfg.initattstd, 2));
    kf.P(10:12, 10:12) = diag(power(cfg.initgyrbiasstd, 2));
    kf.P(13:15, 13:15) = diag(power(cfg.initaccbiasstd, 2));


    % navigation state initialization
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
    param = Param();
    [navstate.Rm, navstate.Rn] = getRmRn(cfg.initpos(1), param);
    navstate.gravity = getGravity(cfg.initpos);
end