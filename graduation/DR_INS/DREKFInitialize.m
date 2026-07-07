function kf = DREKFInitialize(cfg, dr)
%DREKFINITIALIZE 初始化DR/RANGE误差状态EKF
%
% 第一版默认4维状态：
%   x = [dK; dYaw; dLat; dLon]
% 其中状态定义为 true - DR_estimate。

    glvs;

    kf = struct();
    kf.m = cfg.kf.state_dim;
    kf.n = 1;
    kf.ts = dr.ts;

    kf.xk = zeros(kf.m, 1);
    kf.xkk_1 = zeros(kf.m, 1);

    eth0 = earth(dr.pos, dr.vn);
    lat_std = cfg.kf.init_pos_std_m / eth0.RMh;
    lon_std = cfg.kf.init_pos_std_m / eth0.clRNh;

    switch kf.m
        case 4
            dx0 = [cfg.kf.init_dk_std; cfg.kf.init_yaw_std; lat_std; lon_std];
            qv  = [cfg.kf.q_dk; cfg.kf.q_yaw; cfg.kf.q_pos_m/eth0.RMh; cfg.kf.q_pos_m/eth0.clRNh];
        case 5
            % x=[dK; dYaw_c; dYaw_s; dLat; dLon]
            dx0 = [cfg.kf.init_dk_std; cfg.kf.init_yaw_std; cfg.kf.init_yaw_std; lat_std; lon_std];
            qv  = [cfg.kf.q_dk; cfg.kf.q_yaw; cfg.kf.q_yaw; cfg.kf.q_pos_m/eth0.RMh; cfg.kf.q_pos_m/eth0.clRNh];
        otherwise
            error('当前第一版建议使用4维或5维状态，cfg.kf.state_dim=%d暂未配置初始化参数。', kf.m);
    end

    kf.Pxk = diag(dx0).^2;
    kf.Qt  = diag(qv).^2;
    kf.Rk  = cfg.range_std^2;

    kf.Ft = zeros(kf.m);
    kf.Phikk_1 = eye(kf.m);
    kf.Hk = zeros(1, kf.m);
    kf.yk = 0;
    kf.ykk_1 = 0;

    kf.MK = zeros(kf.m);
    kf.last_update_used = false;
end
