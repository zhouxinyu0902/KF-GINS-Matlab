function [kf, navstate] = feedback_range_azimuth_state( ...
        kf, navstate, feedback_imu_bias)
%FEEDBACK_RANGE_AZIMUTH_STATE 完整反馈距离+方位角联合更新的 15 状态误差。
%   位置、速度和完整姿态误差始终写回名义导航状态并清零。IMU 零偏
%   是否反馈由 feedback_imu_bias 控制。

    if nargin < 3 || isempty(feedback_imu_bias)
        feedback_imu_bias = false;
    end
    if ~isscalar(feedback_imu_bias)
        error('feedback_imu_bias 必须为逻辑标量。');
    end
    feedback_imu_bias = logical(feedback_imu_bias);
    navstate.pos = navstate.pos - kf.x(1:3);
    navstate.vel = navstate.vel - kf.x(4:6);

    attitude_error_quaternion = rotvec2quat(kf.x(7:9));
    navstate.qbn = quatProd(attitude_error_quaternion, navstate.qbn);
    navstate.cbn = quat2dcm(navstate.qbn);
    navstate.att = dcm2euler(navstate.cbn);
    kf.x(7:9) = 0;

    if feedback_imu_bias
        navstate.gyrbias = navstate.gyrbias + kf.x(10:12);
        navstate.accbias = navstate.accbias + kf.x(13:15);
        kf.x(10:15) = 0;
    end

    param = Param();
    [navstate.Rm, navstate.Rn] = getRmRn(navstate.pos(1), param);
    navstate.gravity = getGravity(navstate.pos);
    kf.x(1:6) = 0;
end
