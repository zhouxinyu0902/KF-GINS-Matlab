function log_det_FIM = calculate_log_det_FIM_three_beacons(pb1, pb2, pb3, P_auv, sigma_r)
    [num_points, ~] = size(P_auv);
    FIM_total = zeros(2,2);
    
    % 输入验证
    if any(isnan([pb1, pb2, pb3])) || any(isinf([pb1, pb2, pb3]))
        log_det_FIM = -1e10;
        return;
    end

    valid_points = 0;
    for k = 1:num_points
        % 计算到三个信标的相对位置
        d_vec1 = P_auv(k, :) - pb1;
        d_vec2 = P_auv(k, :) - pb2;
        d_vec3 = P_auv(k, :) - pb3;
        
        range1 = norm(d_vec1);
        range2 = norm(d_vec2);
        range3 = norm(d_vec3);
        
        % 避免数值问题
        if range1 < 1e-6 || range2 < 1e-6 || range3 < 1e-6
            continue;
        end
        
        % 计算单位向量
        h_k1 = d_vec1 / range1;
        h_k2 = d_vec2 / range2;
        h_k3 = d_vec3 / range3;
        
        % 累加三个信标的FIM贡献
        FIM_k1 = (1/sigma_r^2) * (h_k1' * h_k1);
        FIM_k2 = (1/sigma_r^2) * (h_k2' * h_k2);
        FIM_k3 = (1/sigma_r^2) * (h_k3' * h_k3);
        
        FIM_total = FIM_total + FIM_k1 + FIM_k2 + FIM_k3;
        valid_points = valid_points + 1;
    end
    
    if valid_points == 0
        log_det_FIM = -1e10;
        return;
    end
    
    det_FIM = det(FIM_total);
    if det_FIM <= 0 || isnan(det_FIM)
        log_det_FIM = -1e10;
    else
        log_det_FIM = log(det_FIM);
    end
end