function log_det_FIM = calculate_log_det_FIM_two_beacons(pb1, pb2, P_auv, sigma_r)
    [num_points, ~] = size(P_auv);
    FIM_total = zeros(2,2);
    
    % 添加输入验证
    if any(isnan([pb1, pb2])) || any(isinf([pb1, pb2]))
        log_det_FIM = -1e10;
        return;
    end

    valid_points = 0;
    for k = 1:num_points
        % 计算相对位置
        d_vec1 = P_auv(k, :) - pb1;
        d_vec2 = P_auv(k, :) - pb2;
        
        range1 = norm(d_vec1);
        range2 = norm(d_vec2);
        
        % 避免数值问题
        if range1 < 1e-6 || range2 < 1e-6 || isnan(range1) || isnan(range2)
            continue;
        end
        
        % 简化计算，移除可能导致问题的坐标转换
        h_k1 = d_vec1 / range1;
        h_k2 = d_vec2 / range2;
        
        % 累加FIM
        FIM_k1 = (1/sigma_r^2) * (h_k1' * h_k1);
        FIM_k2 = (1/sigma_r^2) * (h_k2' * h_k2);
        FIM_total = FIM_total + FIM_k1 + FIM_k2;
        valid_points = valid_points + 1;
    end
    
    % 检查是否有有效的观测点
    if valid_points == 0
        log_det_FIM = -1e10;
        return;
    end
    
    % 计算行列式
    det_FIM = det(FIM_total);
    if det_FIM <= 0 || isnan(det_FIM) || isinf(det_FIM)
        log_det_FIM = -1e10;
    else
        log_det_FIM = log(det_FIM);
    end
end