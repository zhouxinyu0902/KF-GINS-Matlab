function kf = myLBLDVLupdate_17state(navstate, DVL, LBL, kf)
%MYLBLDVLUPDATE_17STATE LBL position, depth and DVL velocity update.
% State 16 is DVL scale-factor correction; state 17 is DVL yaw correction.

glvs;

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

%% Residual: LBL horizontal position, depth and DVL velocity
Z = [navstate.pos(1:2) - LBL(2:3)'; ...
    navstate.pos(3) - (-DVL(5)); ...
    navstate.vel - velocity_dvl_ned];

%% Measurement matrix
H = zeros(6, kf.RANK);
H(1:3, 1:3) = eye(3);
H(4:6, 4:6) = eye(3);
H(4:6, 7:9) = -skew(velocity_dvl_ned);
H(4:6, 16) = -velocity_dvl_ned / (1 + scale_hat);
H(4:6, 17) = -navstate.cbn * ...
    (skew([0; 0; 1]) * velocity_body_frd);

lbl_position_std_rad = [2, 2] / glv.Re;
R = diag([lbl_position_std_rad, 0.2, 0.01, 0.01, 0.01].^2);

%% EKF update
kf.Z = Z;
kf.Zkk_1 = H * kf.x;
K = kf.P * H' / (H * kf.P * H' + R);
kf.x = kf.x + K * (Z - kf.Zkk_1);
I = eye(kf.RANK);
kf.P = (I - K * H) * kf.P * (I - K * H)' + K * R * K';

end
