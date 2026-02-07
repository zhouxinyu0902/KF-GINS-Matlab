function [kf, navstate] = myErrorFeedback_range(kf, navstate)
    b=1;
    % b=1/kf.alpha;
    % position and velocity
    navstate.pos(1:2) = navstate.pos(1:2) - b * kf.x(1:2, 1);
    navstate.pos(3) = navstate.pos(3) - kf.x(3, 1);
    navstate.vel(1:2) = navstate.vel(1:2) - b * kf.x(4:5, 1);
    navstate.vel(3) = navstate.vel(3) - kf.x(6, 1);
    % attitude

    % kf.x(9) = 0;
    % kf.x(7:8) = 0.01*kf.x(7:8);
    % qpn = rotvec2quat(kf.x(7:9, 1));
    % navstate.qbn = quatProd(qpn, navstate.qbn);
    % navstate.cbn = quat2dcm(navstate.qbn);
    % navstate.att = dcm2euler(navstate.cbn);

    % imu error
    navstate.gyrbias = navstate.gyrbias + kf.x(10:12, 1);
    navstate.accbias = navstate.accbias + kf.x(13:15, 1);
    % navstate.gyrscale = navstate.gyrscale + kf.x(16:18, 1);
    % navstate.accscale = navstate.accscale + kf.x(19:21, 1);

    % update some parameters
    param = Param();
    [navstate.Rm, navstate.Rn] = getRmRn(navstate.pos(1), param);
    navstate.gravity = getGravity(navstate.pos);

    % reset state vector
    kf.x (1:6) = zeros(6,1);
    % kf.x ([3,6])= zeros(2, 1);
    kf.x (7:end)= zeros(length(kf.x (7:end)), 1);
    
    % kf.P = 0.5*kf.P;

end