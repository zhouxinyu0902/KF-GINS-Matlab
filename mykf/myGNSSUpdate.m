% GNSS位置更新
function kf = myGNSSUpdate(navstate, gnssdata, kf)
%% GNSS position update
% measurement innovation
Z = navstate.pos - gnssdata(2:4)';% N系下的NED

% measurement matrix and noise matrix
DR = diag([navstate.Rm + navstate.pos(3),...
    (navstate.Rn + navstate.pos(3))*cos(navstate.pos(1)), -1]);

vk=[2;2;2];
R = diag(power(DR^-1*vk, 2));% m m m
H = zeros(3, kf.RANK);
H(1:3, 1:3) = eye(3);

% update covariance and state vector
K = kf.P * H' / (H * kf.P * H' + R);
kf.x = kf.x + K*(Z - H*kf.x);
kf.P=(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
end