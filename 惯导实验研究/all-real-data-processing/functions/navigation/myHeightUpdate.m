function kf = myHeightUpdate(navstate, depthdata, kf)
%MYHEIGHTUPDATE rad,rad,m 位置误差状态下的高度/垂向速度更新。
% 仅更新高度误差（第3维）和垂向速度误差（第6维），水平状态不变。

    innovation = navstate.pos(3) - depthdata(2);
    kf.Z = innovation;
    H = zeros(1, kf.RANK);
    H(3) = 1;
    kf.Zkk_1 = H * kf.x;
    R = kf.depthstd^2;

    S = H * kf.P * H' + R;
    K_full = kf.P * H' / S;

    mask = zeros(kf.RANK, 1);
    mask(3) = 1;
    mask(6) = 1;
    K = K_full .* mask;
    kf.x = kf.x + K * (innovation - kf.Zkk_1);

    I = eye(kf.RANK);
    IKH = I - K * H;
    kf.P = IKH * kf.P * IKH' + K * R * K';
end
