function kf = myHeightUpdate_m(navstate, depthdata, kf)
%MYHEIGHTUPDATE_M m,m,m 位置误差状态下的高度/垂向速度更新。
% m 制位置误差采用 NED 方向定义，第3维为向下位置误差，因此量测矩阵
% 的高度项取 -1；状态反馈时由调用脚本使用相应符号。

    innovation = navstate.pos(3) - depthdata(2);
    kf.Z = innovation;
    H = zeros(1, kf.RANK);
    H(3) = -1;
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
