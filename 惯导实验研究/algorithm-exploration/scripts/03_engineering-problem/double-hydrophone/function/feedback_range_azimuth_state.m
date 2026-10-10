function [kf, navstate] = feedback_range_azimuth_state( ...
        kf, navstate, feedback_attitude, feedback_imu_bias, ...
        feedback_heading_only)
%FEEDBACK_RANGE_AZIMUTH_STATE 反馈距离+方位角联合更新的 15 状态误差。
%   位置和速度始终反馈并清零。feedback_attitude=false 时，7:9 维姿态
%   误差不写回名义姿态，也不清零，而是作为跨历元保留的辅助状态继续
%   参与后续量测预测。feedback_heading_only=true 时只将第 9 维航向
%   误差写回名义姿态，并清除 7:9 维，不反馈横滚和俯仰。IMU 零偏采用
%   相同的可选反馈机制。

    if nargin < 3 || isempty(feedback_attitude)
        feedback_attitude = false;
    end
    if ~isscalar(feedback_attitude)
        error('feedback_attitude 必须为逻辑标量。');
    end
    feedback_attitude = logical(feedback_attitude);
    if nargin < 4 || isempty(feedback_imu_bias)
        feedback_imu_bias = false;
    end
    if ~isscalar(feedback_imu_bias)
        error('feedback_imu_bias 必须为逻辑标量。');
    end
    feedback_imu_bias = logical(feedback_imu_bias);
    if nargin < 5 || isempty(feedback_heading_only)
        feedback_heading_only = false;
    end
    if ~isscalar(feedback_heading_only)
        error('feedback_heading_only 必须为逻辑标量。');
    end
    feedback_heading_only = logical(feedback_heading_only);
    if feedback_attitude && feedback_heading_only
        error('全姿态反馈和仅航向反馈不能同时启用。');
    end

    navstate.pos = navstate.pos - kf.x(1:3);
    navstate.vel = navstate.vel - kf.x(4:6);

    if feedback_attitude
        attitude_error_quaternion = rotvec2quat(kf.x(7:9));
        navstate.qbn = quatProd(attitude_error_quaternion, navstate.qbn);
        navstate.cbn = quat2dcm(navstate.qbn);
        navstate.att = dcm2euler(navstate.cbn);
        kf.x(7:9) = 0;
    elseif feedback_heading_only
        heading_error_quaternion = rotvec2quat([0; 0; kf.x(9)]);
        navstate.qbn = quatProd(heading_error_quaternion, navstate.qbn);
        navstate.cbn = quat2dcm(navstate.qbn);
        navstate.att = dcm2euler(navstate.cbn);
        kf.x(7:9) = 0;
    end

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
