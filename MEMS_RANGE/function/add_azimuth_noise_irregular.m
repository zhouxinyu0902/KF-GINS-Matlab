function meas_azi = add_azimuth_noise_irregular(true_azi_deg, params)
% ADD_AZIMUTH_NOISE_IRREGULAR 添加非线性且不规律的方位角误差
%
% 输入:
%   true_azi_deg : 真实方位角 (度)
%   params       : 结构体
%       .sigma_min : 最佳精度 (0度处，1倍标准差)
%       .sigma_max : 最差精度 (90度处，1倍标准差)
%       .power_fac : 非线性指数
%       .jitter    : 包络抖动强度
%
% 输出:
%   meas_azi     : 加噪后的方位角
    
    % 1. 转换为弧度
    azi_rad = deg2rad(true_azi_deg);
    
    % 2. 计算基础非线性标准差模型
    shape_factor = abs(sin(azi_rad)) .^ params.power_fac;
    base_sigma = params.sigma_min + (params.sigma_max - params.sigma_min) .* shape_factor;
    
    % 3. 生成 Sigma 的随机抖动
    jitter_mod = 1 + (rand(size(true_azi_deg)) - 0.5) * 2 * params.jitter;
    
    % 4. 计算最终的动态标准差
    final_sigma = base_sigma .* jitter_mod;
    
    % 5. 生成高斯白噪声
    noise = randn(size(true_azi_deg)) .* final_sigma;
    
    % =======================================================
    % 【新增步骤】 限幅处理：确保噪声不超过给定最大值的两倍
    % 这里的“给定最大值”指的是 params.sigma_max
    % =======================================================
    noise_limit = 2 * params.sigma_max; 
    
    % 强制截断：大于上限设为上限，小于下限设为下限
    noise(noise > noise_limit) = noise_limit;
    noise(noise < -noise_limit) = -noise_limit;
    
    % 6. 叠加噪声并归一化
    meas_raw = true_azi_deg + noise;
    meas_azi = mod(meas_raw + 180, 360) - 180;
end