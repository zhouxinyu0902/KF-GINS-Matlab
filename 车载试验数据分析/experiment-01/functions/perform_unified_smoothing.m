function [nav_matrix, bridge_error, smoothed_state_buffer] = ...
    perform_unified_smoothing(state_buffer, xk_final, param, range_idx, ...
    smooth_method, pos_unit, Pk_corr_buffer, Pk_pred_buffer, PHI_buffer)
%PERFORM_UNIFIED_SMOOTHING 对单个测距区间执行 RTS 或线性误差平滑。
% state_buffer 每行为 [time, lat, lon, h, vn, ve, vd, roll, pitch, yaw]。
% pos_unit='rad' 时误差前三维为 [rad,rad,m]；'m' 时为 NED 米制。

    num_pts = size(state_buffer, 1);
    state_rank = length(xk_final);
    xs = zeros(num_pts, state_rank);

    if strcmpi(smooth_method, 'RTS')
        xs(num_pts, :) = xk_final';
        for k = num_pts-1:-1:1
            Pk_corr = reshape(Pk_corr_buffer(k, :), ...
                state_rank, state_rank);
            Pk_pred = reshape(Pk_pred_buffer(k, :), ...
                state_rank, state_rank);
            Phi_next = reshape(PHI_buffer(k, :), ...
                state_rank, state_rank);
            Pk_pred = (Pk_pred + Pk_pred') / 2;
            Pk_pred = Pk_pred + eye(state_rank) * 1e-12;
            Gk = Pk_corr * Phi_next' * pinv(Pk_pred);
            xs(k, :) = (Gk * xs(k + 1, :)')';
        end
    elseif strcmpi(smooth_method, 'Linear')
        for k = 1:num_pts
            xs(k, :) = (k / num_pts) * xk_final';
        end
    else
        error('未知平滑方式：%s。请选择 RTS 或 Linear。', smooth_method);
    end

    bridge_error = xs(1, :)';
    nav_matrix = zeros(11, num_pts);
    smoothed_state_buffer = zeros(size(state_buffer));

    for k = 1:num_pts
        position_error = xs(k, 1:3)';
        velocity_error = xs(k, 4:6)';

        if strcmpi(pos_unit, 'm')
            [rm, rn] = getRmRn(state_buffer(k, 2), param);
            rm_plus_h = rm + state_buffer(k, 4);
            rn_plus_h = rn + state_buffer(k, 4);
            cos_lat = cos(state_buffer(k, 2));
            DR_inv = diag([1 / rm_plus_h, ...
                1 / (rn_plus_h * cos_lat), -1]);
            position_rad = state_buffer(k, 2:4)' - ...
                DR_inv * position_error;
        else
            position_rad = state_buffer(k, 2:4)' - position_error;
        end

        velocity_ned = state_buffer(k, 5:7)' - velocity_error;
        attitude_rad = state_buffer(k, 8:10)';
        position_output = [position_rad(1:2) * param.R2D; ...
            position_rad(3)];
        attitude_output = attitude_rad * param.R2D;

        smoothed_state_buffer(k, :) = [state_buffer(k, 1), ...
            position_rad', velocity_ned', attitude_rad'];
        nav_matrix(:, k) = [range_idx; state_buffer(k, 1); ...
            position_output; velocity_ned; attitude_output];
    end
end
