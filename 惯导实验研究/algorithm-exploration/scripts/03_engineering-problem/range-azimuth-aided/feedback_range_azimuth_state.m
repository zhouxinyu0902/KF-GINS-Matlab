function [kf, navstate] = feedback_range_azimuth_state(kf, navstate)
%FEEDBACK_RANGE_AZIMUTH_STATE 反馈距离+方位角联合更新的 15 状态误差。
%   与 myErrorFeedback_range 的位置、速度和 IMU 零偏反馈保持一致，并额外
%   将 7:9 维小失准角反馈到姿态，确保相对方位角的航向约束不会被丢弃。

    navstate.pos = navstate.pos - kf.x(1:3);
    navstate.vel = navstate.vel - kf.x(4:6);

    attitude_error_quaternion = rotvec2quat(kf.x(7:9));
    navstate.qbn = quatProd(attitude_error_quaternion, navstate.qbn);
    navstate.cbn = quat2dcm(navstate.qbn);
    navstate.att = dcm2euler(navstate.cbn);

    navstate.gyrbias = navstate.gyrbias + kf.x(10:12);
    navstate.accbias = navstate.accbias + kf.x(13:15);

    param = Param();
    [navstate.Rm, navstate.Rn] = getRmRn(navstate.pos(1), param);
    navstate.gravity = getGravity(navstate.pos);
    kf.x(:) = 0;
end
