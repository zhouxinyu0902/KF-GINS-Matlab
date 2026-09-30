function [kf, navstate, consider, closed_loop_transition] = ...
        schmidt_consider_height_update(kf, navstate, consider, ...
        height_measurement, position_error_unit)
%SCHMIDT_CONSIDER_HEIGHT_UPDATE Existing decoupled height update plus Pxb.

closed_loop_transition = eye(kf.RANK);
if numel(height_measurement) < 2 || ~isfinite(height_measurement(2))
    return;
end

innovation = navstate.pos(3)-height_measurement(2);
measurement_matrix = zeros(1, kf.RANK);
if lower(string(position_error_unit)) == "rad"
    measurement_matrix(3) = 1;
else
    measurement_matrix(3) = -1;
end
measurement_variance = kf.depthstd^2;
full_gain = kf.P*measurement_matrix'/( ...
    measurement_matrix*kf.P*measurement_matrix'+measurement_variance);
feedback_mask = zeros(kf.RANK, 1);
feedback_mask([3, 6]) = 1;
kalman_gain = full_gain.*feedback_mask;

kf.Z = innovation;
kf.Zkk_1 = measurement_matrix*kf.x;
kf.x = kf.x+kalman_gain*(innovation-kf.Zkk_1);
closed_loop_transition = eye(kf.RANK)-kalman_gain*measurement_matrix;
kf.P = closed_loop_transition*kf.P*closed_loop_transition' ...
    + kalman_gain*measurement_variance*kalman_gain';
kf.P = 0.5*(kf.P+kf.P');
consider.Pxb = closed_loop_transition*consider.Pxb;

if lower(string(position_error_unit)) == "rad"
    navstate.pos(3) = navstate.pos(3)-kf.x(3);
else
    navstate.pos(3) = navstate.pos(3)+kf.x(3);
end
navstate.vel(3) = navstate.vel(3)-kf.x(6);
kf.x(3) = 0;
kf.x(6) = 0;
navstate.gravity = getGravity(navstate.pos);
end
