function [measured_azimuth_deg, sigma_deg, noise_deg] = ...
        add_irregular_azimuth_noise(true_azimuth_deg, options)
%ADD_IRREGULAR_AZIMUTH_NOISE 构造与 MEMS 方位角实验一致的非均匀噪声。
%   正侧向（|方位角|接近 90°）噪声较大，正横以外噪声较小；随机抖动
%   用于模拟不同观测时刻的测角质量变化，最终噪声限制在 2*sigma_max。

    required_fields = {'sigma_min_deg', 'sigma_max_deg', ...
        'shape_power', 'jitter_ratio'};
    for field_index = 1:numel(required_fields)
        if ~isfield(options, required_fields{field_index})
            error('方位角噪声配置缺少字段：%s', required_fields{field_index});
        end
    end

    shape = abs(sind(true_azimuth_deg)) .^ options.shape_power;
    base_sigma = options.sigma_min_deg + ...
        (options.sigma_max_deg - options.sigma_min_deg) .* shape;
    jitter = 1 + (2 * rand(size(true_azimuth_deg)) - 1) .* ...
        options.jitter_ratio;
    sigma_deg = max(eps, base_sigma .* jitter);

    noise_deg = randn(size(true_azimuth_deg)) .* sigma_deg;
    noise_limit_deg = 2 * options.sigma_max_deg;
    noise_deg = min(max(noise_deg, -noise_limit_deg), noise_limit_deg);
    measured_azimuth_deg = wrap_to_180_local(true_azimuth_deg + noise_deg);
end

function angle = wrap_to_180_local(angle)
    angle = mod(angle + 180, 360) - 180;
end
