function [kf, navstate, closed_loop_transition] = ...
        update_decoupled_height_rad(kf, navstate, height_measurement)
%UPDATE_DECOUPLED_HEIGHT_RAD rad位置误差状态的解耦高度更新。
% x(1:3)=[dLat(rad),dLon(rad),dH(m)]。高度残差对x_H的偏导为+1；
% 只反馈高度和下向速度，不影响水平状态及IMU零偏。

    closed_loop_transition = eye(kf.RANK);
    if numel(height_measurement) < 2 || ~isfinite(height_measurement(2))
        return;
    end

    innovation = navstate.pos(3) - height_measurement(2);
    measurement_matrix = zeros(1, kf.RANK);
    measurement_matrix(3) = 1;
    measurement_variance = kf.depthstd ^ 2;
    full_gain = kf.P * measurement_matrix' / ...
        (measurement_matrix * kf.P * measurement_matrix' ...
        + measurement_variance);
    feedback_mask = zeros(kf.RANK, 1);
    feedback_mask([3, 6]) = 1;
    kalman_gain = full_gain .* feedback_mask;

    kf.Z = innovation;
    kf.Zkk_1 = measurement_matrix * kf.x;
    kf.x = kf.x + kalman_gain * (innovation - kf.Zkk_1);
    identity = eye(kf.RANK);
    closed_loop_transition = identity - kalman_gain * measurement_matrix;
    kf.P = closed_loop_transition * kf.P * closed_loop_transition' ...
        + kalman_gain * measurement_variance * kalman_gain';
    kf.P = (kf.P + kf.P') / 2;

    navstate.pos(3) = navstate.pos(3) - kf.x(3);
    navstate.vel(3) = navstate.vel(3) - kf.x(6);
    kf.x(3) = 0;
    kf.x(6) = 0;
    navstate.gravity = getGravity(navstate.pos);
end
