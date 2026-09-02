function dx_smooth_seq = RunRtsBackward(P_seq, P_pred_seq, Phi_seq)
% 输入均为 [Dim x Dim x N] 的矩阵序列
    
    [dim_x, ~, num_steps] = size(P_seq);
    dx_smooth_seq = zeros(dim_x, num_steps);
    
    % 1. 设置终点误差 (闭环系统设为0)
    dx_smooth_seq(:, num_steps) = zeros(dim_x, 1);
    
    % 2. 反向递归
    for k = (num_steps - 1) : -1 : 1
        
        % 取出矩阵
        Pk = P_seq(:, :, k);              % P_k|k
        Pk1_pred = P_pred_seq(:, :, k+1); % P_k+1|k (注意是 k+1)
        Fk = Phi_seq(:, :, k);            % Phi_k->k+1
        
        % 计算平滑增益 Ck = Pk * Fk' * inv(Pk+1_pred)
        % 使用左除 / 提高数值稳定性
        Ck = Pk * Fk' / Pk1_pred;
        
        % 计算平滑误差
        % dx_smooth_k = dx_k(0) + Ck * (dx_smooth_k+1 - dx_pred_k+1(0))
        dx_next = dx_smooth_seq(:, k+1);
        dx_curr = Ck * dx_next;
        
        % 保存
        dx_smooth_seq(:, k) = dx_curr;
    end
end