function kf = myRangeUpdate(navstate, Rangedata, vk, kf, thisimu, dt)
param = Param();
%% GNSS position update
% measurement innovation
bcn=Rangedata(4:6)';
bcn(4:5)=bcn(4:5)*param.D2R;

[~,range_ins]=caldot2dot(navstate.pos,bcn);

Z = range_ins-Rangedata(3);% N系下的NED

% measurement matrix and noise matrix
R = diag(vk);% m m m
H = zeros(1, kf.RANK);
H(1:3, 1:3) = eye(3);
H(1:3, 7:9) = skew(navstate.cbn * antlever);

% update covariance and state vector
K = kf.P * H' / (H * kf.P * H' + R);
kf.x = kf.x + K*(Z - H*kf.x);
kf.P=(eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
end