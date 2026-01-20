function instantaneous_observability = calculate_instantaneous_observability(beacon_pos, auv_trajectory, sigma_r)
% 计算轨迹上每个点的瞬时可观测度
% 输入：
%   beacon_pos: 1x2 或 1x3，信标位置
%   auv_trajectory: Nx2 或 Nx3，AUV轨迹位置序列
%   sigma_r: 距离测量噪声标准差
% 输出：
%   instantaneous_observability: 结构体，包含各种可观测度指标
    
    [num_points, dim] = size(auv_trajectory);
    
    % 初始化输出结构
    instantaneous_observability = struct();
    instantaneous_observability.mu_D = zeros(num_points, 1);  % D-最优性指标
    instantaneous_observability.mu_C = zeros(num_points, 1);  % 条件数指标
    instantaneous_observability.range = zeros(num_points, 1); % 距离
    instantaneous_observability.bearing = zeros(num_points, 1); % 方位角
    
    for i = 1:num_points
        % 计算相对位置向量
        d_vec = auv_trajectory(i, 1:2) - beacon_pos(1:2);
        range_i = norm(d_vec);
        
        % 避免除零错误
        if range_i < 1e-6
            instantaneous_observability.mu_D(i) = -inf;
            instantaneous_observability.mu_C(i) = inf;
            instantaneous_observability.range(i) = range_i;
            instantaneous_observability.bearing(i) = 0;
            continue;
        end
        d_vec=[d_vec(1)*6371000,d_vec(2)*6371000*cos(36/180*pi)];
        % 计算单位向量
        h_i = d_vec / range_i;
        
        % 计算瞬时FIM
        FIM_i = (1/sigma_r^2) * (h_i' * h_i);
        [instantaneous_observability.eigenvalues(:,i),...
            bb,...
            instantaneous_observability.info_ratio(:,:,i),...
            instantaneous_observability.relative_observability(:,i)] =...
        analyze_directional_observability(FIM_i);
        instantaneous_observability.eigenvectors_1(:,i)=bb(:,1);
        instantaneous_observability.eigenvectors_2(:,i)=bb(:,2);
        % 计算特征值
        eigenvals = eig(FIM_i);
        lambda_max = max(eigenvals);
        lambda_min = min(eigenvals);
        
        % 计算可观测度指标
        det_FIM = det(FIM_i);
        if det_FIM <= 0
            instantaneous_observability.mu_D(i) = -inf;
        else
            instantaneous_observability.mu_D(i) = log(det_FIM);
        end
        
        if lambda_min > 1e-10
            instantaneous_observability.mu_C(i) = lambda_max / lambda_min;
        else
            instantaneous_observability.mu_C(i) = inf;
        end
        
        % 存储辅助信息
        instantaneous_observability.range(i) = range_i;
        instantaneous_observability.bearing(i) = atan2(d_vec(2), d_vec(1));
    end
end
function [eigenvalues,eigenvectors,info_ratio,relative_observability] = analyze_directional_observability(FIM)
% 分析FIM在不同方向上的可观测度分布
    
    [V, D] = eig(FIM);
    eigenvals = diag(D);
    
    % 按特征值降序排列
    [eigenvals_sorted, idx] = sort(eigenvals, 'descend');
    V_sorted = V(:, idx);
    
    eigenvalues = eigenvals_sorted;
    eigenvectors = V_sorted;
    info_ratio = eigenvals_sorted / sum(eigenvals_sorted);
    
    % 各方向的相对可观测度
    relative_observability = eigenvals_sorted / max(eigenvals_sorted);
end
