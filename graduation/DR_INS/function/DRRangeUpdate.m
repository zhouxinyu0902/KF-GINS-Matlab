function [kf, info] = DRRangeUpdate(kf, dr, beacon_k, cfg)
%DRRANGEUPDATE 单信标斜距量测更新
%
% beacon_k格式：
%   [time, slant_range, horizontal_range, beacon_lat, beacon_lon, beacon_h]
% 单位：
%   s, m, m, rad, rad, m
%
% 量测采用第二列斜距。第三列水平距离仅保存，不参与第一版滤波。

    z = beacon_k(3);
    beacon_pos = beacon_k(4:6)';

    if any(~isfinite(beacon_pos)) || ~isfinite(z)
        info = make_info(beacon_k(1), z, NaN, NaN, NaN, false);
        return;
    end

    if ~isfield(dr, 'eth') || isempty(dr.eth)
        dr.eth = earth(dr.pos, dr.vn); 
    end

    dlat = dr.pos(1) - beacon_pos(1);
    dlon = dr.pos(2) - beacon_pos(2);
    dh   = dr.pos(3) - beacon_pos(3);

    dN_like = dlat * dr.eth.RMh;
    dE_like = dlon * dr.eth.clRNh;

    R_pred = sqrt(dN_like^2 + dE_like^2 );
    if R_pred < 1e-6
        info = make_info(beacon_k(1), z, R_pred, NaN, NaN, false);
        return;
    end

    % 几何线性化：dR/ddlat, dR/ddlon
    b_lat = dlat * dr.eth.RMh^2 / R_pred;
    b_lon = dlon * dr.eth.clRNh^2 / R_pred;

    H = zeros(1, kf.m);
    switch kf.m
        case 4
            pos_idx = [3, 4];
        case 5
            pos_idx = [4, 5];
        otherwise
            error('DRRangeUpdate当前只支持4维和5维状态。');
    end
    H(pos_idx) = [b_lat, b_lon];

    y = z - R_pred;
    y_pred = H * kf.xk;
    innov = y - y_pred;

    S = H * kf.Pxk * H' + kf.Rk;
    sqrtS = sqrt(max(S, eps));

    used = true;
    if isfield(cfg.kf, 'use_gate') && cfg.kf.use_gate
        if abs(innov) > cfg.kf.gate_sigma * sqrtS
            used = false;
        end
    end

    kf.Hk = H;
    kf.yk = y;
    kf.ykk_1 = y_pred;
    kf.last_update_used = used;

    if used
        K = kf.Pxk * H' / S;
        I = eye(kf.m);

        kf.Kk = K;
        kf.xk = kf.xk + K * innov;

        % Joseph形式，数值更稳定
        kf.Pxk = (I - K*H) * kf.Pxk * (I - K*H)' + K * kf.Rk * K';
        kf.Pxk = 0.5 * (kf.Pxk + kf.Pxk');

        % 信息累积，便于后续可观测度分析
        kf.Mk = H' / kf.Rk * H;
        kf.MK = kf.MK + kf.Mk;
    end

    info = make_info(beacon_k(1), z, R_pred, innov, sqrtS, used);
end

function info = make_info(t, z, pred, innov, sqrtS, used)
    info = struct();
    info.time = t;
    info.range_meas = z;
    info.range_pred = pred;
    info.innovation = innov;
    info.sqrtS = sqrtS;
    info.used = used;
end
