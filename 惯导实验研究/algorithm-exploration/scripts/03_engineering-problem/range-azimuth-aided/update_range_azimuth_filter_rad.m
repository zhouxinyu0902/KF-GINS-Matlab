function kf = update_range_azimuth_filter_rad( ...
        navstate, range_data, depth_data, kf, default_azimuth_std_deg)
%UPDATE_RANGE_AZIMUTH_FILTER_RAD 距离、深度和相对方位角联合量测更新。
%   状态位置误差采用 [rad, rad, m]，方位角残差在滤波内部采用 rad。
%
%   range_data 各列：
%     1 时间；2 斜距；3 水平距离；4:6 信标 [lat(rad), lon(rad), h(m)]；
%     7 带噪相对方位角(deg)；8 真实相对方位角(deg，可选)；
%     9 方位角噪声标准差(deg，可选)。
%
%   方位角定义与 mems-range-navigation/pos2azimuth.m 一致：
%   正北为 0°、正东为 90°，先减航向角，再减去右舷基线法线偏置 90°。
%
%   相对方位角同时与水平位置、航向有关，因此 H 的第 9 列包含航向误差。
%   调用本函数时应同时使用 feedback_range_azimuth_state，把姿态误差反馈
%   到名义导航状态；否则会出现“估计航向但反馈时丢弃”的不一致。

    if nargin < 5 || isempty(default_azimuth_std_deg)
        default_azimuth_std_deg = 0.30;
    end
    if numel(range_data) < 7 || ~isfinite(range_data(7))
        error('距离+方位角更新要求 range_data 第 7 列为方位角（deg）。');
    end

    param = Param();
    beacon = range_data(4:6)';
    [rm, rn] = getRmRn(beacon(1), param);
    scale_n = rm + beacon(3);
    scale_e = (rn + beacon(3)) * cos(navstate.pos(1));

    % 信标相对当前惯导位置的北、东向分量。
    delta_n = (beacon(1) - navstate.pos(1)) * scale_n;
    delta_e = (beacon(2) - navstate.pos(2)) * scale_e;
    horizontal_range_sq = delta_n^2 + delta_e^2;
    horizontal_range = sqrt(horizontal_range_sq);
    if horizontal_range < 1e-3
        error('信标与导航位置过近，无法构造稳定的距离/方位角雅可比。');
    end

    predicted_azimuth = wrap_to_pi_local( ...
        atan2(delta_e, delta_n) - navstate.att(3) - pi / 2);
    measured_azimuth = deg2rad(range_data(7));

    % 采用“预测量测 - 实际量测”的残差定义，与 myRangeUpdate 一致。
    residual = [ ...
        horizontal_range - range_data(3); ...
        navstate.pos(3) - depth_data(2); ...
        wrap_to_pi_local(predicted_azimuth - measured_azimuth)];

    if numel(range_data) >= 9 && isfinite(range_data(9)) && range_data(9) > 0
        azimuth_std_rad = deg2rad(range_data(9));
    else
        azimuth_std_rad = deg2rad(default_azimuth_std_deg);
    end
    measurement_std = [kf.rangstd; kf.depthstd; azimuth_std_rad];
    measurement_covariance = diag(measurement_std .^ 2);

    H = zeros(3, kf.RANK);
    H(1, 1) = -delta_n / horizontal_range * scale_n;
    H(1, 2) = -delta_e / horizontal_range * scale_e;
    H(2, 3) = 1;
    H(3, 1) = delta_e / horizontal_range_sq * scale_n;
    H(3, 2) = -delta_n / horizontal_range_sq * scale_e;
    H(3, 9) = -1;

    predicted_residual = H * kf.x;
    innovation = residual - predicted_residual;
    innovation(3) = wrap_to_pi_local(innovation(3));
    innovation_covariance = H * kf.P * H' + measurement_covariance;
    gain = (kf.P * H') / innovation_covariance;

    kf.x = kf.x + gain * innovation;
    identity = eye(kf.RANK);
    kf.P = (identity - gain * H) * kf.P * (identity - gain * H)' + ...
        gain * measurement_covariance * gain';
    kf.P = (kf.P + kf.P') / 2;

    kf.Z = residual;
    kf.Zkk_1 = predicted_residual;
    kf.range_azimuth_innovation = innovation;
end

function angle = wrap_to_pi_local(angle)
    angle = atan2(sin(angle), cos(angle));
end
