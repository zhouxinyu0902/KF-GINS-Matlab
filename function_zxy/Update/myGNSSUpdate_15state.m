% GNSS位置更新
function kf = myGNSSUpdate_15state(navstate, gnssdata, kf)
%% GNSS position update
% measurement matrix and noise matrix
DR = diag([navstate.Rm + navstate.pos(3),...
    (navstate.Rn + navstate.pos(3))*cos(navstate.pos(1)), -1]);
% measurement innovation
if size(gnssdata,1) == 13
    Z = [navstate.pos - gnssdata(2:4);
        navstate.vel - gnssdata(8:10)];% N系下的NED, 导出数据时已经转换为地向速度了
    
    if gnssdata(5)==0||gnssdata(6)==0||gnssdata(7)==0
        vk(1:3) = DR^-1*[0.02;0.02;0.02];
    else
        vk(1:3) = DR^-1*gnssdata(5:7);
    end
    if gnssdata(11)==0||gnssdata(12)==0||gnssdata(13)==0
        vk(4:6) = [0.01;0.01;0.01];
    else
        vk(4:6) = gnssdata(11:13);
    end
    R = diag(vk.^2);
    H = zeros(6, kf.RANK);
    H(1:3, 1:3) = eye(3);
    H(4:6, 4:6) = eye(3);
elseif size(gnssdata,1) == 7
    Z = navstate.pos - gnssdata(2:4);% N系下的NED
    if gnssdata(5)==0||gnssdata(6)==0||gnssdata(7)==0
        vk = [0.02;0.02;0.02];
    else
        vk = gnssdata(5:7);
    end
    % vk = [1;1;0.2];
    R = diag(power(DR^-1*vk, 2));% m m m
    H = zeros(3, kf.RANK);
    H(1:3, 1:3) = eye(3);
elseif size(gnssdata,1) == 4
    Z = navstate.pos - gnssdata(2:4);% N系下的NED
    vk = [2;2;2];
    R = diag(power(DR^-1*vk, 2));% m m m
    H = zeros(3, kf.RANK);
    H(1:3, 1:3) = eye(3);
end
% update covariance and state vector
K = kf.P * H' / (H * kf.P * H' + R);
kf.x = kf.x + K*(Z - H * kf.x);
kf.P = (eye(kf.RANK) - K*H) * kf.P * (eye(kf.RANK) - K*H)' + K * R * K';
end