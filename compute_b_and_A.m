function [b, A] = compute_b_and_A(x_hat_i, x_hat_0, z_observations, ...
                                 F_matrices, H_matrices, P0_inv, ...
                                 Q_inv_list, R_inv_list, h_function, n)
    % 计算b向量和A矩阵
    %
    % 输入参数:
    %   x_hat_i: 当前状态估计，cell数组{x_0, x_1, ..., x_k}，每个x_k是2n×1列向量
    %   x_hat_0: 初始状态估计 (2n×1列向量)
    %   z_observations: 观测值，cell数组{z_1, z_2, ..., z_n}
    %   F_matrices: 状态转移矩阵，cell数组{F_0, F_1, ..., F_{k-1}} (每个2n×2n)
    %   H_matrices: 观测矩阵，cell数组{H_1, H_2, ..., H_n}
    %   P0_inv: 初始协方差矩阵的逆 (2n×2n)
    %   Q_inv_list: 过程噪声协方差矩阵逆，cell数组{Q_0^{-1}, Q_1^{-1}, ...} (每个2n×2n)
    %   R_inv_list: 观测噪声协方差矩阵逆，cell数组{R_1^{-1}, R_2^{-1}, ...}
    %   h_function: 观测函数句柄
    %   n: 状态维度参数(根据Π矩阵定义)
    %
    % 输出参数:
    %   b: 计算得到的b向量
    %   A: 计算得到的A矩阵

    %% 1. 构建Π矩阵(根据图片1公式22)
    num_states = length(x_hat_i);
    Pi = zeros(2*n, 2*n*num_states);
    Pi(:, 1:2*n) = eye(2*n); % Π = [I_{2n} 0 ... 0]
    
    %% 2. 计算b向量(根据图片2公式)
    b = zeros(2*n*num_states, 1);
    
    % 第一部分: 初始状态项 Π^T P0^{-1} (x_hat_i{1} - x_hat_0)
    initial_diff = x_hat_i{1} - x_hat_0;
    b = b + Pi' * P0_inv * initial_diff;
    
    % 第二部分: 状态转移项 Σ F_{k-1}^T Q_{k-1}^{-1} (x_hat_i{k} - F_{k-1} x_hat_i{k-1})
    for k = 2:num_states
        state_diff = x_hat_i{k} - F_matrices{k-1} * x_hat_i{k-1};
        % 构建选择矩阵(类似图片1公式24中的结构)
        selector = zeros(2*n, 2*n*num_states);
        selector(:, (k-2)*2*n+1:(k-1)*2*n) = -F_matrices{k-1};
        selector(:, (k-1)*2*n+1:k*2*n) = eye(2*n);
        
        b = b + selector' * Q_inv_list{k-1} * state_diff;
    end
    
    % 第三部分: 观测项 Σ H_k^T R_k^{-1} (z_k - h(x_hat_i{k}))
    num_observations = length(z_observations);
    for k = 1:num_observations
        state_idx = min(k, num_states); % 处理观测数可能多于状态数的情况
        innovation = z_observations{k} - h_function(x_hat_i{state_idx});
        
        % 构建H选择矩阵(类似图片1公式23中的结构)
        H_selector = zeros(size(H_matrices{k},1), 2*n*num_states);
        H_selector(:, (state_idx-1)*2*n+1:state_idx*2*n) = -H_matrices{k};
        
        b = b + H_selector' * R_inv_list{k} * innovation;
    end
    
    %% 3. 计算A矩阵(根据图片3公式)
    A = Pi' * P0_inv * Pi;
    
    % 添加状态转移项的贡献
    for k = 2:num_states
        selector = zeros(2*n, 2*n*num_states);
        selector(:, (k-2)*2*n+1:(k-1)*2*n) = -F_matrices{k-1};
        selector(:, (k-1)*2*n+1:k*2*n) = eye(2*n);
        
        A = A + selector' * Q_inv_list{k-1} * selector;
    end
    
    % 添加观测项的贡献
    for k = 1:num_observations
        state_idx = min(k, num_states);
        H_selector = zeros(size(H_matrices{k},1), 2*n*num_states);
        H_selector(:, (state_idx-1)*2*n+1:state_idx*2*n) = -H_matrices{k};
        
        A = A + H_selector' * R_inv_list{k} * H_selector;
    end
end