function kf = myInsPropagate_17state(navstate, thisimu, dt, kf)

param = Param();

%% Copy navigation-state parameters
pos = navstate.pos;
vel = navstate.vel;
cbn = navstate.cbn;
rm = navstate.Rm;
rn = navstate.Rn;
gravity = navstate.gravity;

%% IMU data
omega = thisimu(2:4,1) / dt;
accel = thisimu(5:7,1) / dt;

%% Geometric parameters
rmh = rm + pos(3);
rnh = rn + pos(3);

wie_n = [param.WGS84_WIE*cos(pos(1));
         0;
        -param.WGS84_WIE*sin(pos(1))];

wen_n = [vel(2)/(rn+pos(3));
        -vel(1)/(rm+pos(3));
        -vel(2)*tan(pos(1))/(rn+pos(3))];

%% State-transition matrix
% 17-state:
% 1:3    position error
% 4:6    velocity error
% 7:9    attitude error
% 10:12  gyro bias error
% 13:15  accelerometer bias error
% 16     DVL scale-factor error
% 17     DVL yaw-installation-angle error

F = zeros(kf.RANK, kf.RANK);
PHI = eye(kf.RANK);

%% ------------------------------------------------------------
% 1. Position error
% -------------------------------------------------------------
Frr = zeros(3,3);

Frr(1,3) = -vel(1)/(rmh^2);

Frr(2,1) = ...
    vel(2)*tan(pos(1))/(rnh*cos(pos(1)));

Frr(2,3) = ...
    -vel(2)/(rnh^2*cos(pos(1)));

F(1:3,1:3) = Frr;

Frv = diag([1/rmh, ...
            1/(cos(pos(1))*rnh), ...
            -1]);

F(1:3,4:6) = Frv;

%% ------------------------------------------------------------
% 2. Velocity error
% -------------------------------------------------------------
Fvr = zeros(3,3);

Fvr(1,1) = ...
    -2*vel(2)*param.WGS84_WIE*cos(pos(1))/rmh ...
    -vel(2)^2/rmh/rnh/cos(pos(1))^2;

Fvr(1,3) = ...
    vel(1)*vel(3)/rmh/rmh ...
    -vel(2)^2*tan(pos(1))/rnh/rnh;

Fvr(2,1) = ...
    2*param.WGS84_WIE * ...
    (vel(1)*cos(pos(1))-vel(3)*sin(pos(1)))/rmh ...
    +vel(1)*vel(2)/rmh/rnh/cos(pos(1))^2;

Fvr(2,3) = ...
    (vel(2)*vel(3) ...
    +vel(1)*vel(2)*tan(pos(1))) ...
    /rnh/rnh;

Fvr(3,1) = ...
    2*param.WGS84_WIE*vel(2)*sin(pos(1))/rmh;

Fvr(3,3) = ...
    -vel(2)^2/rnh/rnh ...
    -vel(1)^2/rmh/rmh ...
    +2*gravity/(sqrt(rm*rn)+pos(3));

Fvr(:,1) = Fvr(:,1)*rmh;
Fvr(:,3) = -Fvr(:,3);

F(4:6,1:3) = Fvr;

Fvv = zeros(3,3);

Fvv(1,1) = vel(3)/rmh;

Fvv(1,2) = ...
    -2*(param.WGS84_WIE*sin(pos(1)) ...
    +vel(2)*tan(pos(1))/rnh);

Fvv(1,3) = vel(1)/rmh;

Fvv(2,1) = ...
    2*param.WGS84_WIE*sin(pos(1)) ...
    +vel(2)*tan(pos(1))/rnh;

Fvv(2,2) = ...
    (vel(3)+vel(1)*tan(pos(1)))/rnh;

Fvv(2,3) = ...
    2*param.WGS84_WIE*cos(pos(1)) ...
    +vel(2)/rnh;

Fvv(3,1) = -2*vel(1)/rmh;

Fvv(3,2) = ...
    -2*(param.WGS84_WIE*cos(pos(1)) ...
    +vel(2)/rnh);

F(4:6,4:6) = Fvv;

%% Attitude error -> velocity error
F(4:6,7:9) = skew(cbn*accel);

%% Accelerometer bias error -> velocity error
F(4:6,13:15) = cbn;

%% ------------------------------------------------------------
% 3. Attitude error
% -------------------------------------------------------------
Fphir = zeros(3,3);

Fphir(1,1) = ...
    -param.WGS84_WIE*sin(pos(1))/rmh;

Fphir(1,3) = ...
    vel(2)/rnh/rnh;

Fphir(2,3) = ...
    -vel(1)/rmh/rmh;

Fphir(3,1) = ...
    -param.WGS84_WIE*cos(pos(1))/rmh ...
    -vel(2)/rmh/rnh/cos(pos(1))^2;

Fphir(3,3) = ...
    -vel(2)*tan(pos(1))/rnh/rnh;

Fphir(:,1) = Fphir(:,1)*rmh;
Fphir(:,3) = -Fphir(:,3);

F(7:9,1:3) = Fphir;

Fphiv = zeros(3,3);

Fphiv(1,2) = 1/rnh;
Fphiv(2,1) = -1/rmh;
Fphiv(3,2) = -tan(pos(1))/rnh;

F(7:9,4:6) = Fphiv;

F(7:9,7:9) = -skew(wie_n+wen_n);

%% Gyroscope bias error -> attitude error
F(7:9,10:12) = -cbn;

%% ------------------------------------------------------------
% 4. IMU bias dynamics
% -------------------------------------------------------------
corrtime = 3600;

F(10:12,10:12) = ...
    -1/corrtime*eye(3);

F(13:15,13:15) = ...
    -1/corrtime*eye(3);

%% ------------------------------------------------------------
% 5. DVL calibration parameter dynamics
% -------------------------------------------------------------
% First version:
% Treat DVL scale factor and yaw installation angle as constants.
%
% delta_K_dot     = 0
% delta_alpha_dot = 0
%
% F is initialized to zero, so these lines are optional.
F(16,16) = 0;
F(17,17) = 0;

%% Discrete state-transition matrix
PHI = PHI + F*dt;

%% ------------------------------------------------------------
% 6. Noise-drive matrix
% -------------------------------------------------------------
% Noise rank remains 12:
%
% 1:3    accelerometer white noise
% 4:6    gyro white noise
% 7:9    gyro-bias driving noise
% 10:12  accelerometer-bias driving noise

G = zeros(kf.RANK, kf.NOISE_RANK);

G(4:6,1:3) = cbn;
G(7:9,4:6) = cbn;

G(10:12,7:9) = eye(3);
G(13:15,10:12) = eye(3);

% Rows 16 and 17 remain zero.
% Therefore no process noise is injected into the two DVL
% calibration parameters in the first version.

%% Discrete process noise
Qd = G*kf.Qc*G'*dt;

Qd = (PHI*Qd*PHI' + Qd)/2;

%% ------------------------------------------------------------
% 7. Error-state and covariance propagation
% -------------------------------------------------------------
kf.P = PHI*kf.P*PHI' + Qd;

% Numerical symmetry
kf.P = 0.5*(kf.P+kf.P');

kf.Pk_k1 = kf.P;

kf.x = PHI*kf.x;

kf.phi = PHI;

end