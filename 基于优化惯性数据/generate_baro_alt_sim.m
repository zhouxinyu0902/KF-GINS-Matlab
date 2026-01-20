function [time, h_baro_meas, error_markov] = generate_baro_alt_sim(h_true, bias, sigma, tau, dt)
% GENERATE_BARO_ALT_SIM_VECTOR 使用一阶马尔科夫过程生成气压高度计仿真数据
%
% 输入参数:
%   h_true    - 真实的参考高度 (N*1 列向量)
%   bias      - 高度测量的零偏 (m)。例如: 0.5
%   sigma     - 一阶马尔科夫过程的稳态标准差 (m)。例如: 0.5
%   tau       - 一阶马尔科夫过程的相关时间 (s)。例如: 10
%   dt        - 测量周期/采样间隔 (s)。例如: 0.01
%
% 输出参数:
%   time          - 时间向量 (N*1 列向量, s)
%   h_baro_meas   - 气压高度计仿真测量值 (N*1 列向量, m)
%   error_markov  - 一阶马尔科夫过程生成的误差 (N*1 列向量, m)

    %% 1. 参数检查与初始化
    if nargin < 5
        error('输入参数不足，请提供 h_true (N*1向量), bias, sigma, tau, dt 五个参数。');
    end
    
    % 检查 h_true 必须是向量
    if isscalar(h_true)
        error('h_true 必须是 N*1 的参考高度向量，而不是标量。');
    end

    N = length(h_true); % 总采样点数
    T_total = (N - 1) * dt; % 仿真总时长
    time = (0:N-1)' * dt;   % 时间向量 (N*1 列向量)

    % 确保 h_true 是列向量 (N*1)
    if size(h_true, 2) > 1 && size(h_true, 1) == 1 
        % 如果输入是 1*N 行向量，则转置为 N*1
        h_true = h_true';
        warning('输入 h_true 是行向量，已自动转置为 N*1 列向量。');
    elseif size(h_true, 2) ~= 1
        error('h_true 必须是 N*1 的列向量。');
    end

    %% 2. 计算一阶马尔科夫过程系数
    % 离散化的递推系数
    alpha = exp(-dt / tau);

    % 驱动噪声的标准差
    sigma_w = sigma * sqrt(1 - exp(-2 * dt / tau));

    %% 3. 生成一阶马尔科夫过程误差
    error_markov = zeros(N, 1); % 存储马尔科夫过程生成的误差 (N*1)
    
    % 使用递推公式生成误差
    for k = 2:N
        % 生成高斯白噪声 (均值0, 标准差1)
        wk = randn(); 
        
        % 递推公式: Error[k] = alpha * Error[k-1] + sigma_w * w[k]
        error_markov(k) = alpha * error_markov(k-1) + sigma_w * wk;
    end

    %% 4. 生成气压高度计仿真测量值
    % 气压高度计测量值 = 真实高度 + 零偏 + 马尔科夫过程误差
    % 零偏 (bias) 会自动广播到 h_true 和 error_markov 的维度
    h_baro_meas = h_true + bias + error_markov; 
    
    disp('气压高度计仿真数据生成完毕。');
    disp(['  仿真总时长 T_total: ', num2str(T_total), 's']);
    disp(['  驱动噪声标准差 sigma_w: ', num2str(sigma_w)]);
    disp(['  采样点数 N: ', num2str(N)]);

end