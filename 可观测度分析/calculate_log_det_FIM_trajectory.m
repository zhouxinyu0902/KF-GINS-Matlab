function log_det_FIM = calculate_log_det_FIM_trajectory(pb_trajectory, P_auv, sigma_r)
    % pb_trajectory: 移动信标轨迹，每行是一个时间点的位置[x, y]
    % P_auv: AUV轨迹，每行是一个时间点的位置[x, y]
    % sigma_r: 距离测量噪声标准差
    
    [num_points, ~] = size(P_auv);
    FIM_total = zeros(2,2); % 二维平面定位，FIM是2x2矩阵

    for k = 1:num_points
        % 计算相对位置向量
        d_vec = P_auv(k, :) - pb_trajectory(k, :);
        range = norm(d_vec);

        % 避免除零错误
        if range < 1e-6
            log_det_FIM = -inf; % 如果信标和AUV位置重合，赋予负无穷大代价
            return;
        end
        
        % 坐标转换（保持原有的坐标转换）
        % d_vec = [d_vec(1)*6371000, d_vec(2)*6371000*cos(36/180*pi)];
        
        % 计算单位向量 h_k
        h_k = d_vec / range;

        % 计算并累加瞬时FIM
        FIM_k = (1/sigma_r^2) * (h_k' * h_k);
        FIM_total = FIM_total + FIM_k;
    end

    % 计算行列式并取对数
    det_FIM = det(FIM_total);
    if det_FIM <= 0
        log_det_FIM = -inf; % 非正定矩阵，不可观测，赋予极大惩罚
    else
        log_det_FIM = log(det_FIM);
    end
end