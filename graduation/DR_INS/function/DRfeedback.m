function [kf, dr] = DRfeedback(kf, dr, cfg)
%DRFEEDBACK 将EKF误差状态反馈到DR主状态
%
% 本程序中状态定义为：x = true - DR_estimate。
% 因此位置反馈采用：dr.pos = dr.pos + x_pos。
%
% 第一版默认只反馈水平位置，dK和dYaw保留在滤波状态中，便于观察参数可观测性。

    coef = cfg.feedback.coef;

    switch kf.m
        case 4
            kod_idx = 1;
            yaw_idx = 2;
            pos_idx = [3, 4];
        case 5
            kod_idx = 1;
            yaw_idx = [];
            pos_idx = [4, 5];
        otherwise
            error('DRfeedback当前只支持4维和5维状态。');
    end

    % 位置反馈：lat/lon
    if cfg.feedback.position
        dpos = coef * kf.xk(pos_idx);
        dr.pos(1:2) = dr.pos(1:2) + dpos;
        kf.xk(pos_idx) = kf.xk(pos_idx) - dpos;
    end

    % DVL刻度因子反馈：后续DRmechanization中会用 dvl / dr.kod
    if cfg.feedback.dvl_scale
        dk = coef * kf.xk(kod_idx);
        dr.kod = dr.kod + dk;
        dr.kod = max(dr.kod, 0.1);
        kf.xk(kod_idx) = kf.xk(kod_idx) - dk;
    end

    % 航向反馈：保存为yaw_corr，后续每个罗盘历元都会加上该修正量。
    if cfg.feedback.yaw && ~isempty(yaw_idx)
        dyaw = coef * kf.xk(yaw_idx);
        if ~isfield(dr, 'yaw_corr')
            dr.yaw_corr = 0;
        end
        dr.yaw_corr = dr.yaw_corr + dyaw;
        kf.xk(yaw_idx) = kf.xk(yaw_idx) - dyaw;
    end

    dr.eth = earth(dr.pos, dr.vn);
    dr.avp = [dr.att; dr.vn; dr.pos];
end
