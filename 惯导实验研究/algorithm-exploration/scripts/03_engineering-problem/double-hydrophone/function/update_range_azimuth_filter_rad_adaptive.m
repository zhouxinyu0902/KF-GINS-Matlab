function [kf, adaptive_azimuth_std_deg] = ...
        update_range_azimuth_filter_rad_adaptive( ...
        navstate, range_data, depth_data, kf, default_azimuth_std_deg)
%UPDATE_RANGE_AZIMUTH_FILTER_RAD_ADAPTIVE 自适应方位角噪声联合更新。
%   线阵相位差测角在接近法向 0 deg 时灵敏度较好，在接近端射
%   +/-90 deg 时角度误差会按 1/|cos(theta)| 放大。本函数以
%   45 deg 为标称参考角：
%
%     sigma(theta) = sigma_default * cos(45 deg) / |cos(theta)|
%
%   因而 0 deg 处约为 0.707*sigma_default，45 deg 处等于输入的
%   default_azimuth_std_deg。为防止 +/-90 deg 附近数值发散，
%   |cos(theta)| 下限取 0.20，对应最大约 3.536*sigma_default。
%   随后复用固定噪声版本的联合更新，保证残差、雅可比和反馈定义一致。

    if nargin < 5 || isempty(default_azimuth_std_deg) || ...
            ~isscalar(default_azimuth_std_deg) || ...
            ~isfinite(default_azimuth_std_deg) || ...
            default_azimuth_std_deg <= 0
        error('default_azimuth_std_deg 必须为正有限标量。');
    end
    if numel(range_data) < 7 || ~isfinite(range_data(7))
        error('自适应距离+方位角更新要求 range_data 第 7 列为方位角。');
    end

    measured_azimuth_deg = mod(range_data(7) + 180, 360) - 180;
    principal_angle_deg = abs(asind(sind(measured_azimuth_deg)));
    cosine_floor = 0.20;
    reference_cosine = cosd(45);
    effective_cosine = max(abs(cosd(principal_angle_deg)), cosine_floor);
    adaptive_scale = reference_cosine / effective_cosine;
    adaptive_azimuth_std_deg = ...
        default_azimuth_std_deg * adaptive_scale;

    adaptive_range_data = range_data;
    adaptive_range_data(9) = adaptive_azimuth_std_deg;
    kf = update_range_azimuth_filter_rad(navstate, adaptive_range_data, ...
        depth_data, kf, adaptive_azimuth_std_deg);
    kf.default_azimuth_std_deg = default_azimuth_std_deg;
    kf.adaptive_azimuth_std_deg = adaptive_azimuth_std_deg;
    kf.adaptive_azimuth_scale = adaptive_scale;
    kf.adaptive_principal_angle_deg = principal_angle_deg;
end
