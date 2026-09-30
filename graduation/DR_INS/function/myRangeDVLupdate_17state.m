function kf = myRangeDVLupdate_17state(navstate, DVL, Rangedata, kf)
%MYRANGEDVLUPDATE_17STATE Horizontal range, depth and DVL velocity update.
% State 16 is DVL scale-factor correction; state 17 is DVL yaw correction.

param = Param();

%% Predicted horizontal range
beacon_pos = Rangedata(4:6)';
[rm, rn] = getRmRn(beacon_pos(1), param);
h = beacon_pos(3);
DR = diag([rm + h, (rn + h) * cos(beacon_pos(1)), -1]);
delta_pos = DR * (navstate.pos - beacon_pos);
horizontal_range_m = norm(delta_pos(1:2));

%% DVL velocity corrected with the current calibration estimates
velocity_d_rfu = DVL(2:4);
velocity_d_frd = [velocity_d_rfu(2); velocity_d_rfu(1); ...
    -velocity_d_rfu(3)];

scale_hat = navstate.dvlscale;
yaw_hat = navstate.dvlyaw;
c = cos(yaw_hat);
s = sin(yaw_hat);
Cbd = [c, -s, 0; s, c, 0; 0, 0, 1];
velocity_body_frd = Cbd' * velocity_d_frd / (1 + scale_hat);
velocity_dvl_ned = navstate.cbn * velocity_body_frd;

%% Residual: depth, horizontal range and DVL velocity
Z = [navstate.pos(3) - (-DVL(5)); ...
    horizontal_range_m - Rangedata(3); ...
    navstate.vel - velocity_dvl_ned];

%% Measurement matrix
H = zeros(5, kf.RANK);
range_jacobian = (navstate.pos - beacon_pos)' * (DR ^ 2) / ...
    horizontal_range_m;
H(1, 3) = 1;
H(2, 1:2) = range_jacobian(1:2);
H(3:5, 4:6) = eye(3);
H(3:5, 7:9) = -skew(velocity_dvl_ned);
H(3:5, 16) = -velocity_dvl_ned / (1 + scale_hat);
H(3:5, 17) = -navstate.cbn * ...
    (skew([0; 0; 1]) * velocity_body_frd);

R = diag([0.2, 5, 0.01, 0.01, 0.01].^2);

%% EKF update
kf.Z = Z;
kf.Zkk_1 = H * kf.x;
K = kf.P * H' / (H * kf.P * H' + R);
kf.x = kf.x + K * (Z - kf.Zkk_1);
I = eye(kf.RANK);
kf.P = (I - K * H) * kf.P * (I - K * H)' + K * R * K';

end
