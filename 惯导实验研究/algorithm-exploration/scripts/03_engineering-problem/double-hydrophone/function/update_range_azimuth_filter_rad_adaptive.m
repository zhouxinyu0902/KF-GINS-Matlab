function [kf, adaptive_azimuth_std_deg, adaptive_info] = ...
        update_range_azimuth_filter_rad_adaptive( ...
        navstate, range_data, depth_data, kf, default_azimuth_std_deg, ...
        prior_navstate, prior_kf, prior_azimuth_deg, ...
        estimate_attitude_error, estimate_imu_bias_error)
%UPDATE_RANGE_AZIMUTH_FILTER_RAD_ADAPTIVE 先验一致性门控的自适应联合更新。
%   线阵相位差测角在接近法向 0 deg 时灵敏度较好，在接近端射
%   +/-90 deg 时角度误差会按 1/|cos(theta)| 放大。先计算只放大的
%   几何方差倍率：
%
%     g = max(1, (cos(45 deg) / max(|cos(theta)|, 0.2))^2)
%
%   再利用距离预更新副本给出的先验角及协方差计算归一化角度差：
%
%     q = wrap(theta_selected - theta_prior)^2 / (P_theta_prior + R0)
%
%   q <= 1 时保持固定 R；q >= 3.84 时完整启用几何放大；中间区域
%   线性插值。最终 R_eff = R0 * (1 + w * (g - 1))，始终不小于 R0。

    if nargin < 5 || isempty(default_azimuth_std_deg) || ...
            ~isscalar(default_azimuth_std_deg) || ...
            ~isfinite(default_azimuth_std_deg) || ...
            default_azimuth_std_deg <= 0
        error('default_azimuth_std_deg 必须为正有限标量。');
    end
    if numel(range_data) < 7 || ~isfinite(range_data(7))
        error('自适应距离+方位角更新要求 range_data 第 7 列为方位角。');
    end
    if nargin < 8 || isempty(prior_navstate) || isempty(prior_kf) || ...
            ~isscalar(prior_azimuth_deg) || ...
            ~isfinite(prior_azimuth_deg)
        error('归一化自适应更新要求有效的距离预更新先验状态、协方差和方位角。');
    end
    if nargin < 9 || isempty(estimate_attitude_error)
        estimate_attitude_error = true;
    end
    if ~isscalar(estimate_attitude_error)
        error('estimate_attitude_error 必须为逻辑标量。');
    end
    estimate_attitude_error = logical(estimate_attitude_error);
    if nargin < 10 || isempty(estimate_imu_bias_error)
        estimate_imu_bias_error = true;
    end
    if ~isscalar(estimate_imu_bias_error)
        error('estimate_imu_bias_error 必须为逻辑标量。');
    end
    estimate_imu_bias_error = logical(estimate_imu_bias_error);

    measured_azimuth_deg = mod(range_data(7) + 180, 360) - 180;
    principal_angle_deg = abs(asind(sind(measured_azimuth_deg)));
    cosine_floor = 0.20;
    reference_cosine = cosd(45);
    effective_cosine = max(abs(cosd(principal_angle_deg)), cosine_floor);
    geometry_std_scale = max(1.0, reference_cosine / effective_cosine);
    geometry_r_ratio = geometry_std_scale ^ 2;

    param = Param();
    beacon = range_data(4:6)';
    [rm, rn] = getRmRn(beacon(1), param);
    scale_n = rm + beacon(3);
    scale_e = (rn + beacon(3)) * cos(prior_navstate.pos(1));
    delta_n = (beacon(1) - prior_navstate.pos(1)) * scale_n;
    delta_e = (beacon(2) - prior_navstate.pos(2)) * scale_e;
    horizontal_range_sq = delta_n ^ 2 + delta_e ^ 2;
    if horizontal_range_sq < 1e-6
        error('信标与距离预更新位置过近，无法计算先验方位角方差。');
    end

    prior_H = zeros(1, prior_kf.RANK);
    prior_H(1) = delta_e / horizontal_range_sq * scale_n;
    prior_H(2) = -delta_n / horizontal_range_sq * scale_e;
    prior_H(9) = -1;
    prior_azimuth_variance_rad2 = real(prior_H * prior_kf.P * prior_H');
    prior_azimuth_variance_rad2 = max(prior_azimuth_variance_rad2, 0);

    angle_difference_rad = atan2( ...
        sin(deg2rad(measured_azimuth_deg - prior_azimuth_deg)), ...
        cos(deg2rad(measured_azimuth_deg - prior_azimuth_deg)));
    default_variance_rad2 = deg2rad(default_azimuth_std_deg) ^ 2;
    normalized_innovation_q = angle_difference_rad ^ 2 / ...
        (prior_azimuth_variance_rad2 + default_variance_rad2);
    activation_weight = min(1.0, max(0.0, ...
        (normalized_innovation_q - 1.0) / (3.84 - 1.0)));
    effective_r_ratio = 1.0 + activation_weight * ...
        (geometry_r_ratio - 1.0);
    adaptive_scale = sqrt(effective_r_ratio);
    adaptive_azimuth_std_deg = ...
        default_azimuth_std_deg * adaptive_scale;

    adaptive_range_data = range_data;
    adaptive_range_data(9) = adaptive_azimuth_std_deg;
    kf = update_range_azimuth_filter_rad(navstate, adaptive_range_data, ...
        depth_data, kf, adaptive_azimuth_std_deg, ...
        estimate_attitude_error, estimate_imu_bias_error);
    kf.default_azimuth_std_deg = default_azimuth_std_deg;
    kf.adaptive_azimuth_std_deg = adaptive_azimuth_std_deg;
    kf.adaptive_azimuth_scale = adaptive_scale;
    kf.adaptive_principal_angle_deg = principal_angle_deg;
    kf.prior_azimuth_std_deg = ...
        rad2deg(sqrt(prior_azimuth_variance_rad2));
    kf.normalized_azimuth_innovation_q = normalized_innovation_q;
    kf.geometry_azimuth_r_ratio = geometry_r_ratio;
    kf.adaptive_activation_weight = activation_weight;
    kf.effective_azimuth_r_ratio = effective_r_ratio;

    adaptive_info = struct( ...
        'prior_azimuth_std_deg', kf.prior_azimuth_std_deg, ...
        'normalized_innovation_q', normalized_innovation_q, ...
        'geometry_r_ratio', geometry_r_ratio, ...
        'activation_weight', activation_weight, ...
        'effective_r_ratio', effective_r_ratio);
end
