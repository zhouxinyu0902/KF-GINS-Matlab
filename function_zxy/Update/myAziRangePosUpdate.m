%% 距离更新
function kf = myAziRangePosUpdate(navstate, rangedata, depthdata, kf)
% measurement matrix and noise matrix
DR = diag([navstate.Rm + navstate.pos(3),...
    (navstate.Rn + navstate.pos(3))*cos(navstate.pos(1)), -1]);
% measurement innovation

pos_xy = calc_position_from_beacon([0,0],rangedata(3),rangedata(7),r2d(navstate.att(3)));
kf.pos_rad1 = dxyz2pos([pos_xy,0],rangedata(4:6)');

% pos_rad = rangedata(8:9);
% DXYZ = DR * (pos_rad1-rangedata(8:10))';
global posstd
global depstd
Z = navstate.pos - [kf.pos_rad1(1:2)';depthdata(2)];
vk = [posstd;posstd;depstd];
R = diag(power(DR^-1*vk, 2));% m m m
H = zeros(3, kf.RANK);
H(1:3, 1:3) = eye(3);

% update covariance and state vector
K = kf.P * H' / (H * kf.P * H' + R);
kf.x = kf.x + K*(Z - H*kf.x);
kf.P = (eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';