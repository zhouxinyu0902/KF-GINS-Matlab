function meas_azi = add_azimuth_noise_model(true_azi_deg, sigma_min, sigma_max)
% ADD_AZIMUTH_NOISE_MODEL 给方位角添加角度依赖的高斯白噪声
%
% 输入:
%   true_azi_deg : 真实方位角 (度)，支持标量或向量
%   sigma_min    : 最佳精度 (在 0°, ±180° 处)，单位：度
%   sigma_max    : 最差精度 (在 ±90° 处)，单位：度
%
% 输出:
%   meas_azi     : 加噪后的方位角 (度)，已归一化到 [-180, 180]

    % 1. 转换为弧度用于计算权重
    azi_rad = deg2rad(true_azi_deg);
    
    % 2. 计算动态标准差 sigma(theta)
    % 使用 |sin(theta)| 模型模拟端射方向精度下降
    % 0度/180度时 -> sigma_min
    % +/-90度时   -> sigma_max
    current_sigma = sigma_min + (sigma_max - sigma_min) * abs(sin(azi_rad));
    
    % 3. 生成非平稳高斯白噪声
    noise = randn(size(true_azi_deg)) .* current_sigma;
    
    % 4. 叠加噪声
    meas_raw = true_azi_deg + noise;
    
    % 5. 角度归一化到 [-180, 180]
    meas_azi = mod(meas_raw + 180, 360) - 180;
end