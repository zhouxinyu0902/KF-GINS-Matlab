function meas_azi = add_azimuth_noise_irregular(true_azi_deg, params)
% ADD_AZIMUTH_NOISE_IRREGULAR 添加非线性且不规律的方位角误差
%
% 输入:
%   true_azi_deg : 真实方位角 (度)
%   params       : 结构体，包含以下字段:
%       .sigma_min : 最佳精度 (0度处)
%       .sigma_max : 最差精度 (90度处)
%       .power_fac : (新) 非线性指数。1为线性，>1为凹曲线(推荐2)，<1为凸曲线。
%       .jitter    : (新) 包络抖动强度 (0~1之间)。例如 0.2 表示 sigma 有 +/-20% 的随机波动。
%
% 输出:
%   meas_azi     : 加噪后的方位角 (度)，归一化到 [-180, 180]

    % 1. 转换为弧度
    azi_rad = deg2rad(true_azi_deg);
    
    % 2. 计算基础非线性标准差模型 (Base Sigma Curve)
    % 使用幂函数 |sin(x)|^p 来改变曲线形状，使其不那么线性
    shape_factor = abs(sin(azi_rad)) .^ params.power_fac;
    base_sigma = params.sigma_min + (params.sigma_max - params.sigma_min) .* shape_factor;
    
    % 3. 生成 Sigma 的随机抖动 (Sigma Jitter)
    % 生成一个在 [1 - jitter, 1 + jitter] 之间均匀分布的随机系数
    % 这使得标准差本身不再是一条完美的曲线，增加了“不规律性”
    jitter_mod = 1 + (rand(size(true_azi_deg)) - 0.5) * 2 * params.jitter;
    
    % 4. 计算最终的动态标准差
    final_sigma = base_sigma .* jitter_mod;
    
    % 5. 生成高斯白噪声并叠加
    noise = randn(size(true_azi_deg)) .* final_sigma;
    meas_raw = true_azi_deg + noise;
    
    % 6. 归一化
    meas_azi = mod(meas_raw + 180, 360) - 180;
end