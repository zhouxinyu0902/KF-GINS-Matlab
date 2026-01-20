function [Delta_theta_opt, Delta_v_opt, num_iter] = solve_coupled_imus(Delta_theta, Delta_v, ...
                                                    Delta_b_g, Delta_b_a, ...
                                                    M, C, A, ...
                                                    sigma_g, sigma_a, ...
                                                    Delta_theta_prev, Delta_v_prev)
%SOLVE_COUPLED_IMUS 使用固定点迭代联合求解最优角速度和加速度。
%
%   输入参数:
%   - Delta_theta: (3x1) 原始陀螺仪测量值 (Delta_theta~)
%   - Delta_v: (3x1) 原始加速度计测量值 (Delta_v~)
%   - bg: (3x1) 陀螺仪偏差估计 (bg)
%   - ba: (3x1) 加速度计偏差估计 (ba)
%   - K, M, C, A: (标量/向量) 根据外部约束推导出的系数项。
%   - sigma_g: (标量) 陀螺仪测量标准差 (用于sigma_g^2)
%   - sigma_a: (标量) 加速度计测量标准差 (用于sigma_a^2)
%   - w_prev: (3x1) 上一步的最优wib (用于初始化或迭代)
%   - f_prev: (3x1) 上一步的最优fb (用于初始化或迭代)
%
%   输出参数:
%   - w_opt: (3x1) 最终最优角速度 (Wib)
%   - f_opt: (3x1) 最终最优加速度 (fb)
%   - num_iter: 迭代次数

    
    % --- I. 参数与初始化 ---
    MAX_ITER = 50;          % 最大迭代次数
    TOL = 1e-14;             % 收敛阈值 (例如 1e-6 rad/s 和 m/s^2)
    
    % 使用初始估计值 (去除偏差的测量值) 作为第一次迭代的输入
    Delta_theta_k = Delta_theta;
    Delta_v_k = Delta_v;
    
    % 或者使用上一步最优解作为初始化
    if nargin == 10 % 如果提供了 w_prev, f_prev
        Delta_theta_k = Delta_theta_prev;
        Delta_v_k = Delta_v_prev;
    end
    
    Delta_theta_next = zeros(3, 1);
    Delta_v_next = zeros(3, 1);
    
    % --- II. 迭代求解 ---
    for k = 1:MAX_ITER
        Delta_theta_prev = Delta_theta_k;
        Delta_v_prev = Delta_v_k;
        
        % 1. 求解新的 f_b (使用前一步的 w_ib)
        % 公式: f^b = (f_tilde - ba) - sigma_a^2 * [ M * [w_ib_prev x] * C - A ]
        
        % 计算 [w_ib_prev x] 
        Delta_theta_skew = skew_symmetric(Delta_theta_prev);
        
        % 计算修正项 M * [w_ib_prev x] * C - A
        correction_f = M * Delta_theta_skew * C - 0.01*A;
        
        Delta_v_next = (Delta_v - Delta_b_a) - (sigma_a^2 * correction_f);
        
        
        % 2. 求解新的 w_ib (使用新的 f_b)
        % 公式: w_ib^b = (w_tilde - bg) + sigma_g^2 * M * [f_b_next x] * C
        
        % 计算 [f_b_next x] 
        Delta_v_skew = skew_symmetric(Delta_v_next);
        
        % 计算修正项 M * [f_b_next x] * C
        correction_w = M * Delta_v_skew * C;
        
        Delta_theta_next = (Delta_theta - Delta_b_g) + (sigma_g^2 * correction_w);
        
        
        % 3. 检查收敛性
        delta_Delta_theta = norm(Delta_theta_next - Delta_theta_prev);
        delta_Delta_v = norm(Delta_v_next - Delta_v_prev);
        
        if (delta_Delta_theta < TOL) && (delta_Delta_v < TOL)
            Delta_theta_opt = Delta_theta_next;
            Delta_v_opt = Delta_v_next;
            num_iter = k;
            return;
        end
        
        % 更新迭代变量
        Delta_theta_k = Delta_theta_next;
        Delta_v_k = Delta_v_next;
    end
    
    % --- III. 结果输出 ---
    warning('Fixed-point iteration did not converge within %d iterations.', MAX_ITER);
    Delta_theta_opt = Delta_theta_next;
    Delta_v_opt = Delta_v_next;
    num_iter = MAX_ITER;

end


% 辅助函数：计算反对称矩阵 [v x]
function S = skew_symmetric(v)
    S = [ 0,    -v(3),  v(2);
          v(3),  0,    -v(1);
         -v(2),  v(1),  0  ];
end