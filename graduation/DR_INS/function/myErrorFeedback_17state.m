function [kf, navstate] = myErrorFeedback_17state(kf, navstate)

b = 1;

%% 1. Position and velocity error feedback
navstate.pos = navstate.pos - b * kf.x(1:3);

navstate.vel(1:2) = navstate.vel(1:2) - b * kf.x(4:5);
navstate.vel(3)   = navstate.vel(3)   - b * kf.x(6);

%% 2. Attitude error feedback
qpn = rotvec2quat(b * kf.x(7:9));
navstate.qbn = quatProd(qpn, navstate.qbn);
navstate.cbn = quat2dcm(navstate.qbn);
navstate.att = dcm2euler(navstate.cbn);

%% 3. IMU error feedback
% State definition:
% delta_bg = bg_true - bg_hat
% delta_ba = ba_true - ba_hat
navstate.gyrbias = navstate.gyrbias + b * kf.x(10:12);
navstate.accbias = navstate.accbias + b * kf.x(13:15);

%% 4. DVL calibration parameter feedback
% State 16:
% delta_K = K_true - K_hat
navstate.dvlscale = navstate.dvlscale + b * kf.x(16);

% State 17:
% delta_alpha = alpha_true - alpha_hat
% Unit: rad
navstate.dvlyaw = navstate.dvlyaw + b * kf.x(17);

% Keep yaw installation angle within [-pi, pi]
navstate.dvlyaw = atan2(sin(navstate.dvlyaw), ...
                        cos(navstate.dvlyaw));

%% 5. Update navigation-related parameters
param = Param();
[navstate.Rm, navstate.Rn] = getRmRn(navstate.pos(1), param);
navstate.gravity = getGravity(navstate.pos);

%% 6. Reset error-state estimate
kf.x = zeros(kf.RANK, 1);

end