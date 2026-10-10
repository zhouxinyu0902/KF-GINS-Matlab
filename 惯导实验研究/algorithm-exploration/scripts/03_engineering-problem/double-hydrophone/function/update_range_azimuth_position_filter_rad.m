function [kf, update_info] = update_range_azimuth_position_filter_rad( ...
        navstate, range_data, depth_data, kf, default_azimuth_std_deg, ...
        prior_navstate, prior_kf, prior_azimuth_deg, options)
%UPDATE_RANGE_AZIMUTH_POSITION_FILTER_RAD 条件式位置+深度更新。
%   使用同一帧距离和相对方位角反算二维位置，并根据相位测角灵敏度、
%   航向先验方差、距离和先验一致性判断该位置是否可用。
%
%   位置可用时，量测为：
%     [深度；北向位置；东向位置]
%   位置不可用时，退回既有固定 R 或 Adaptive 距离+方位角更新。
%
%   反算位置由同一帧距离和方位角得到，因此位置更新时不再重复加入
%   原始距离和方位角。R_NE 保留极坐标误差传播产生的非对角项。

    if nargin < 9 || isempty(options)
        options = struct();
    end
    required_fields = {'phase_std_deg', 'baseline_m', 'carrier_hz', ...
        'sound_speed_mps'};
    for field_index = 1:numel(required_fields)
        if ~isfield(options, required_fields{field_index})
            error('位置联合更新缺少 options.%s。', ...
                required_fields{field_index});
        end
    end
    options = set_default(options, 'baseline_install_deg', 0);
    options = set_default(options, 'installation_std_deg', 0);
    options = set_default(options, 'position_covariance_scale', 1.0);
    options = set_default(options, 'position_information_weight', 1.0);
    options = set_default(options, 'minimum_position_cosine', 0.2);
    options = set_default(options, 'maximum_position_std_m', 300);
    options = set_default(options, 'maximum_normalized_q', 6.63);
    options = set_default(options, 'fallback_mode', "adaptive");
    options = set_default(options, 'estimate_attitude_error', true);
    options = set_default(options, 'estimate_imu_bias_error', true);
    if ~isscalar(options.estimate_attitude_error) || ...
            ~isscalar(options.estimate_imu_bias_error)
        error('姿态和 IMU 零偏估计开关必须为逻辑标量。');
    end
    options.estimate_attitude_error = ...
        logical(options.estimate_attitude_error);
    options.estimate_imu_bias_error = ...
        logical(options.estimate_imu_bias_error);
    positive_values = [default_azimuth_std_deg, options.phase_std_deg, ...
        options.baseline_m, options.carrier_hz, options.sound_speed_mps, ...
        options.position_covariance_scale, options.maximum_position_std_m, ...
        options.maximum_normalized_q];
    if any(~isfinite(positive_values)) || any(positive_values <= 0)
        error('方位角和位置联合更新的标准差、尺度及门限必须为正有限值。');
    end
    if options.position_information_weight < 0 || ...
            options.position_information_weight > 1 || ...
            options.minimum_position_cosine < 0 || ...
            options.minimum_position_cosine > 1 || ...
            options.installation_std_deg < 0
        error('位置权重、余弦门限或安装角标准差设置无效。');
    end
    if numel(range_data) < 7 || ~isfinite(range_data(7))
        error('距离+方位角+位置更新要求 range_data 第 7 列为方位角。');
    end
    fallback_mode = lower(options.fallback_mode);
    if ~ismember(fallback_mode, ["adaptive", "fixed"])
        error('fallback_mode 只能设置为 "adaptive" 或 "fixed"。');
    end

    measured_azimuth_deg = mod(range_data(7) + 180, 360) - 180;
    principal_angle_deg = abs(asind(sind(measured_azimuth_deg)));
    selected_cosine = abs(cosd(principal_angle_deg));

    %% 与现有 Adaptive 更新一致的方位角 R
    cosine_floor = 0.20;
    reference_cosine = cosd(45);
    effective_cosine = max(selected_cosine, cosine_floor);
    geometry_std_scale = max(1.0, reference_cosine / effective_cosine);
    geometry_r_ratio = geometry_std_scale ^ 2;

    param = Param();
    beacon = range_data(4:6)';
    [rm, rn] = getRmRn(beacon(1), param);
    prior_scale_n = rm + beacon(3);
    prior_scale_e = (rn + beacon(3)) * cos(prior_navstate.pos(1));
    prior_delta_n = (beacon(1) - prior_navstate.pos(1)) * prior_scale_n;
    prior_delta_e = (beacon(2) - prior_navstate.pos(2)) * prior_scale_e;
    prior_horizontal_range_sq = prior_delta_n ^ 2 + prior_delta_e ^ 2;
    if prior_horizontal_range_sq < 1e-6
        error('信标与距离预更新位置过近，无法计算先验方位角方差。');
    end

    prior_H = zeros(1, prior_kf.RANK);
    prior_H(1) = prior_delta_e / prior_horizontal_range_sq * prior_scale_n;
    prior_H(2) = -prior_delta_n / prior_horizontal_range_sq * prior_scale_e;
    prior_H(9) = -1;
    prior_azimuth_variance_rad2 = real(prior_H * prior_kf.P * prior_H');
    prior_azimuth_variance_rad2 = max(prior_azimuth_variance_rad2, 0);

    angle_difference_rad = wrap_to_pi_local( ...
        deg2rad(measured_azimuth_deg - prior_azimuth_deg));
    default_variance_rad2 = deg2rad(default_azimuth_std_deg) ^ 2;
    normalized_innovation_q = angle_difference_rad ^ 2 / ...
        (prior_azimuth_variance_rad2 + default_variance_rad2);
    activation_weight = min(1.0, max(0.0, ...
        (normalized_innovation_q - 1.0) / (3.84 - 1.0)));
    effective_r_ratio = 1.0 + activation_weight * ...
        (geometry_r_ratio - 1.0);
    adaptive_azimuth_std_deg = default_azimuth_std_deg * ...
        sqrt(effective_r_ratio);

    %% 相位噪声传播到位置的预测精度
    wavelength_m = options.sound_speed_mps / options.carrier_hz;
    if selected_cosine > 1e-8
        phase_azimuth_std_rad = wavelength_m * options.phase_std_deg / ...
            (360 * options.baseline_m * selected_cosine);
    else
        phase_azimuth_std_rad = inf;
    end
    adaptive_azimuth_std_rad = deg2rad(adaptive_azimuth_std_deg);
    position_azimuth_std_rad = max(phase_azimuth_std_rad, ...
        adaptive_azimuth_std_rad);
    yaw_variance_rad2 = max(real(prior_kf.P(9, 9)), 0);
    installation_variance_rad2 = deg2rad( ...
        options.installation_std_deg) ^ 2;
    absolute_bearing_variance_rad2 = position_azimuth_std_rad ^ 2 + ...
        yaw_variance_rad2 + installation_variance_rad2;
    measured_horizontal_range_m = range_data(3);
    predicted_position_std_m = sqrt(kf.rangstd ^ 2 + ...
        measured_horizontal_range_m ^ 2 * ...
        absolute_bearing_variance_rad2);

    geometry_usable = selected_cosine >= options.minimum_position_cosine;
    consistency_usable = ...
        normalized_innovation_q <= options.maximum_normalized_q;
    accuracy_usable = isfinite(predicted_position_std_m) && ...
        predicted_position_std_m <= options.maximum_position_std_m;
    weight_usable = options.position_information_weight > 0;
    position_usable = geometry_usable && consistency_usable && ...
        accuracy_usable && weight_usable;

    fallback_reason = "none";
    if ~geometry_usable
        fallback_reason = "poor-geometry";
    elseif ~consistency_usable
        fallback_reason = "prior-inconsistent";
    elseif ~accuracy_usable
        fallback_reason = "large-position-std";
    elseif ~weight_usable
        fallback_reason = "zero-position-weight";
    end

    %% 记录本帧实际构造的局部位置量测（无论最终是否采用）
    absolute_bearing_rad = wrap_to_pi_local( ...
        deg2rad(measured_azimuth_deg + ...
        rad2deg(prior_navstate.att(3) + prior_kf.x(9)) + ...
        options.baseline_install_deg + 90));
    radial_unit = [cos(absolute_bearing_rad); ...
        sin(absolute_bearing_rad)];
    tangential_unit = [-sin(absolute_bearing_rad); ...
        cos(absolute_bearing_rad)];
    measured_vehicle_from_beacon_ne_m = ...
        -measured_horizontal_range_m * radial_unit;
    range_variance_m2 = kf.rangstd ^ 2;
    depth_variance_m2 = kf.depthstd ^ 2;
    tangential_variance_m2 = measured_horizontal_range_m ^ 2 * ...
        absolute_bearing_variance_rad2;

    if ~position_usable
        if fallback_mode == "adaptive"
            [kf, adaptive_azimuth_std_deg, adaptive_info] = ...
                update_range_azimuth_filter_rad_adaptive( ...
                navstate, range_data, depth_data, kf, ...
                default_azimuth_std_deg, prior_navstate, prior_kf, ...
                prior_azimuth_deg, options.estimate_attitude_error, ...
                options.estimate_imu_bias_error);
            normalized_innovation_q = ...
                adaptive_info.normalized_innovation_q;
            geometry_r_ratio = adaptive_info.geometry_r_ratio;
            activation_weight = adaptive_info.activation_weight;
            effective_r_ratio = adaptive_info.effective_r_ratio;
            prior_azimuth_variance_rad2 = deg2rad( ...
                adaptive_info.prior_azimuth_std_deg) ^ 2;
        else
            kf = update_range_azimuth_filter_rad(navstate, range_data, ...
                depth_data, kf, default_azimuth_std_deg, ...
                options.estimate_attitude_error, ...
                options.estimate_imu_bias_error);
            adaptive_azimuth_std_deg = default_azimuth_std_deg;
            geometry_r_ratio = 1;
            activation_weight = 0;
            effective_r_ratio = 1;
        end
        position_covariance_ne_m2 = nan(2);
    else
        %% 由距离、相对方位角和距离预更新航向反算局部北东位置
        position_covariance_ne_m2 = options.position_covariance_scale * ...
            (range_variance_m2 * (radial_unit * radial_unit') + ...
            tangential_variance_m2 * ...
            (tangential_unit * tangential_unit')) / ...
            options.position_information_weight;

        %% 深度、北东位置更新；不再重复加入同源距离和方位角
        scale_n = rm + beacon(3);
        scale_e = (rn + beacon(3)) * cos(navstate.pos(1));
        predicted_vehicle_from_beacon_ne_m = [ ...
            (navstate.pos(1) - beacon(1)) * scale_n; ...
            (navstate.pos(2) - beacon(2)) * scale_e];
        residual = [ ...
            navstate.pos(3) - depth_data(2); ...
            predicted_vehicle_from_beacon_ne_m - ...
                measured_vehicle_from_beacon_ne_m];

        measurement_covariance = blkdiag( ...
            kf.depthstd ^ 2, position_covariance_ne_m2);
        H = zeros(3, kf.RANK);
        H(1, 3) = 1;
        H(2, 1) = scale_n;
        H(3, 2) = scale_e;

        predicted_residual = H * kf.x;
        innovation = residual - predicted_residual;
        innovation_covariance = H * kf.P * H' + ...
            measurement_covariance;
        gain = (kf.P * H') / innovation_covariance;
        if ~options.estimate_attitude_error
            gain(7:9, :) = 0;
        end
        if ~options.estimate_imu_bias_error
            gain(10:15, :) = 0;
        end
        measurement_state_correction = gain * innovation;
        kf.x = kf.x + measurement_state_correction;
        identity = eye(kf.RANK);
        kf.P = (identity - gain * H) * kf.P * ...
            (identity - gain * H)' + ...
            gain * measurement_covariance * gain';
        kf.P = (kf.P + kf.P') / 2;
        kf.Z = residual;
        kf.Zkk_1 = predicted_residual;
        kf.position_depth_innovation = innovation;
        kf.last_measurement_state_correction = ...
            measurement_state_correction;
    end

    kf.position_measurement_used = position_usable;
    kf.estimate_attitude_error = options.estimate_attitude_error;
    kf.estimate_imu_bias_error = options.estimate_imu_bias_error;
    kf.predicted_position_std_m = predicted_position_std_m;
    kf.position_covariance_ne_m2 = position_covariance_ne_m2;
    kf.adaptive_azimuth_std_deg = adaptive_azimuth_std_deg;
    update_info = struct( ...
        'position_used', position_usable, ...
        'fallback_reason', fallback_reason, ...
        'measured_horizontal_range_m', measured_horizontal_range_m, ...
        'measured_depth_m', depth_data(2), ...
        'measured_azimuth_deg', measured_azimuth_deg, ...
        'absolute_bearing_deg', rad2deg(absolute_bearing_rad), ...
        'measured_position_north_m', ...
            measured_vehicle_from_beacon_ne_m(1), ...
        'measured_position_east_m', ...
            measured_vehicle_from_beacon_ne_m(2), ...
        'range_variance_m2', range_variance_m2, ...
        'depth_variance_m2', depth_variance_m2, ...
        'azimuth_variance_rad2', ...
            deg2rad(adaptive_azimuth_std_deg) ^ 2, ...
        'predicted_position_std_m', predicted_position_std_m, ...
        'phase_azimuth_std_deg', rad2deg(phase_azimuth_std_rad), ...
        'adaptive_azimuth_std_deg', adaptive_azimuth_std_deg, ...
        'prior_azimuth_std_deg', ...
            rad2deg(sqrt(prior_azimuth_variance_rad2)), ...
        'normalized_innovation_q', normalized_innovation_q, ...
        'geometry_r_ratio', geometry_r_ratio, ...
        'activation_weight', activation_weight, ...
        'effective_r_ratio', effective_r_ratio, ...
        'selected_cosine', selected_cosine, ...
        'position_covariance_ne_m2', position_covariance_ne_m2);
end

function angle_rad = wrap_to_pi_local(angle_rad)
    angle_rad = atan2(sin(angle_rad), cos(angle_rad));
end

function options = set_default(options, field_name, default_value)
    if ~isfield(options, field_name) || isempty(options.(field_name))
        options.(field_name) = default_value;
    end
end
