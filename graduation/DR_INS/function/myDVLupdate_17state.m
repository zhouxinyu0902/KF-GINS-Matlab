function kf = myDVLupdate_17state(navstate,DVLdata,kf)

%% Raw measurements
velocity_d_rfu = DVLdata(2:4);
depth_meas_m = DVLdata(5);

% DVL RFU -> DVL FRD
velocity_d_frd = [velocity_d_rfu(2);
                  velocity_d_rfu(1);
                 -velocity_d_rfu(3)];

%% Current DVL calibration estimates
scale_hat = navstate.dvlscale;
yaw_hat = navstate.dvlyaw;

c = cos(yaw_hat);
s = sin(yaw_hat);

% body -> DVL
Cbd = [ c,-s,0;
        s, c,0;
        0, 0,1];

% DVL -> body
Cdb = Cbd';

%% Correct raw DVL velocity
velocity_body_frd = ...
    Cdb * velocity_d_frd / (1 + scale_hat);

%% Transform into navigation frame
velocity_dvl_ned = ...
    navstate.cbn * velocity_body_frd;

%% Residual
depth_residual_m = ...
    navstate.pos(3) - (-depth_meas_m);

dvl_residual_mps = ...
    navstate.vel - velocity_dvl_ned;

Z = [depth_residual_m;
     dvl_residual_mps];

%% Measurement matrix
H = zeros(4,kf.RANK);

% depth
H(1,3) = 1;

% velocity error
H(2:4,4:6) = eye(3);

% attitude error
H(2:4,7:9) = -skew(velocity_dvl_ned);

% DVL scale-factor correction: delta_K = K_true - K_hat.
% Since the residual is INS velocity minus corrected DVL velocity,
% its derivative with respect to delta_K is negative.
H(2:4,16) = ...
    -velocity_dvl_ned / (1 + scale_hat);

% DVL yaw correction: delta_alpha = alpha_true - alpha_hat.
ez = [0;0;1];

H(2:4,17) = ...
    -navstate.cbn * ...
    (skew(ez) * velocity_body_frd);

%% Measurement noise
R = diag([0.2^2;
          repmat(0.01^2,3,1)]);

%% EKF update
kf.Z = Z;
kf.Zkk_1 = H*kf.x;

K = kf.P*H'/(H*kf.P*H' + R);

kf.x = kf.x + K*(Z-kf.Zkk_1);

I = eye(kf.RANK);

kf.P = (I-K*H)*kf.P*(I-K*H)' + K*R*K';

end
