function kf = DRinspropagation(kf, dr)
%DRINSPROPAGATION DR误差状态传播
%
% 对应原myekf('fk') + myekf('algo','T')，但拆成独立函数。
% 状态定义为 true - DR_estimate。

    m = kf.m;
    Ft = zeros(m, m);

    VE = dr.vn(1);
    VN = dr.vn(2);
    phi = dr.att(3);

    switch m
        case 4
            % x=[dK; dYaw; dLat; dLon]
            Ft(3,1) = VN / dr.eth.RMh;
            Ft(3,2) = VE / dr.eth.RMh;
            Ft(4,1) = VE / dr.eth.clRNh;
            Ft(4,2) = -VN / dr.eth.clRNh;
            Ft(4,3) = VE * dr.eth.sl / (dr.eth.clRNh * dr.eth.cl);

        case 5
            % x=[dK; dYaw_c; dYaw_s; dLat; dLon]
            Ft(4,1) = VN / dr.eth.RMh;
            Ft(4,2) = VE * cos(2*phi) / dr.eth.RMh;
            Ft(4,3) = VE * sin(2*phi) / dr.eth.RMh;

            Ft(5,1) = VE / dr.eth.clRNh;
            Ft(5,2) = -VN * cos(2*phi) / dr.eth.clRNh;
            Ft(5,3) = -VN * sin(2*phi) / dr.eth.clRNh;
            Ft(5,4) = VE * dr.eth.sl / (dr.eth.clRNh * dr.eth.cl);

        otherwise
            error('DRinspropagation当前只实现4维和5维状态。');
    end

    Phi = eye(m) + Ft * dr.ts;

    % 与你myekf中的离散化方式保持一致
    Qk = (kf.Qt + Phi * kf.Qt * Phi') * dr.ts / 2;

    kf.Ft = Ft;
    kf.Phikk_1 = Phi;
    kf.xkk_1 = Phi * kf.xk;
    kf.Pxkk_1 = Phi * kf.Pxk * Phi' + Qk;

    kf.xk = kf.xkk_1;
    kf.Pxk = 0.5 * (kf.Pxkk_1 + kf.Pxkk_1');
end
