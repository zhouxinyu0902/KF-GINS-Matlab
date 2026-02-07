% GNSS位置更新
function kf = myGNSSUpdate_15state(navstate, gnssdata, kf)
%% GNSS position update
% measurement matrix and noise matrix
DR = diag([navstate.Rm + navstate.pos(3),...
    (navstate.Rn + navstate.pos(3))*cos(navstate.pos(1)), -1]);
% measurement innovation
if size(gnssdata,1) > 7
    Z = [navstate.pos - gnssdata(2:4);
        navstate.vel - gnssdata(8:10)];% N系下的NED
    vk = [DR^-1*gnssdata(5:7)*2; gnssdata(11:13)];
    if vk(4)==0
        vk(4)=0.01;
    end
    if vk(5)==0
        vk(5)=0.01;
    end
    if vk(6)==0
        vk(6)=0.01;
    end
    R = diag(vk.^2);
    H = zeros(6, kf.RANK);
    H(1:3, 1:3) = eye(3);
    H(4:6, 4:6) = eye(3);
else
    Z = navstate.pos - gnssdata(2:4);% N系下的NED
    vk = gnssdata(5:7)*2;
    R = diag(power(DR^-1*vk, 2));% m m m
    H = zeros(3, kf.RANK);
    H(1:3, 1:3) = eye(3);
end
% update covariance and state vector
K = kf.P * H' / (H * kf.P * H' + R);
kf.x = kf.x + K*(Z - H * kf.x);
kf.P = (eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
end