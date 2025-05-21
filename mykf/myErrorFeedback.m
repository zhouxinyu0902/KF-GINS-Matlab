function [kf, navstate] = myErrorFeedback(kf, navstate)
    b=1;
    % position and velocity
    navstate.pos = navstate.pos - b * kf.x(1:3, 1);
    navstate.vel = navstate.vel - b * kf.x(4:6, 1);

    % attitude
    qpn = rotvec2quat(kf.x(7:9, 1));
    navstate.qbn = quatProd(qpn, navstate.qbn);
    navstate.cbn = quat2dcm(navstate.qbn);
    navstate.att = dcm2euler(navstate.cbn);

    % imu error
    navstate.gyrbias = navstate.gyrbias + kf.x(10:12, 1);
    navstate.accbias = navstate.accbias + kf.x(13:15, 1);

    % update some parameters
    param = Param();
    [navstate.Rm, navstate.Rn] = getRmRn(navstate.pos(1), param);
    navstate.gravity = getGravity(navstate.pos);

    % reset state vector
    kf.x = zeros(kf.RANK, 1);
    % kf.P = kf.P0;
end