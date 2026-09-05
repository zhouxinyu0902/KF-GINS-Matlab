function result = simulate_phase_difference( ...
    baseline_m, array_center_deg, source_deg, ...
    baseline_ref_deg, config)
%SIMULATE_PHASE_DIFFERENCE 双水听器真实相位差与含噪相位差生成。
%
% 输入：
%   baseline_m
%       双水听器基线长度，m
%
%   array_center_deg
%       阵列中心位置 [lat_deg, lon_deg]
%
%   source_deg
%       发声点位置 [lat_deg, lon_deg]
%
%   baseline_ref_deg
%       H1 -> H2 基线轴相对于正北方向顺时针的角度，deg
%       若 H1 为船尾、H2 为船首，则可直接输入船首向
%
%   config
%       .carrier_hz        载频，默认 12 kHz
%       .sound_speed_mps   声速，默认 1500 m/s
%       .phase_std_deg     随机相位噪声标准差，默认 0 deg
%       .phase_bias_deg    固定残余相位偏差，默认 0 deg
%       .channel_delay_us  双通道残余时延，默认 0 us
%
% 输出 result：
%   .path_difference_m
%   .true_phase_unwrapped_deg
%   .true_phase_wrapped_deg
%   .measured_phase_wrapped_deg
%   .injected_error_deg
%   .source_bearing_deg
%   .angle_to_normal_deg
%   .range_h1_m
%   .range_h2_m
%
% 相位定义：
%   Delta_phi = phi1 - phi2
%             = 2*pi/lambda * (r2-r1)

%% 1. Configuration

if nargin < 5 || isempty(config)
    config = struct();
end

config = set_default(config, 'carrier_hz',       12e3);
config = set_default(config, 'sound_speed_mps',  1500);
config = set_default(config, 'phase_std_deg',    0);
config = set_default(config, 'phase_bias_deg',   0);
config = set_default(config, 'channel_delay_us', 0);

lambda = config.sound_speed_mps / config.carrier_hz;
%% 2. Source position relative to array center

[source_east_m, source_north_m] = latlon_to_local_en( ...
    array_center_deg, source_deg);

source_en = [source_east_m, source_north_m];


%% 3. Hydrophone positions in local EN frame

% Heading convention:
%   North = 0 deg
%   East  = 90 deg
baseline_unit = [ ...
    sind(baseline_ref_deg), ...
    cosd(baseline_ref_deg)];

h1_en = -0.5 * baseline_m * baseline_unit;
h2_en =  0.5 * baseline_m * baseline_unit;


%% 4. Exact geometric path difference

range_h1_m = norm(source_en - h1_en);
range_h2_m = norm(source_en - h2_en);

% Sign convention:
% Delta_r = r2 - r1
path_difference_m = range_h2_m - range_h1_m;


%% 5. True phase difference

true_phase_unwrapped_rad = ...
    2*pi/lambda * path_difference_m;

true_phase_unwrapped_deg = ...
    rad2deg(true_phase_unwrapped_rad);

true_phase_wrapped_deg = ...
    wrap_to_180(true_phase_unwrapped_deg);


%% 6. Phase measurement error

% Random phase error
random_error_deg = ...
    config.phase_std_deg * randn();

% Fixed hardware / calibration residual
bias_error_deg = config.phase_bias_deg;

% Relative channel delay -> phase bias
delay_error_deg = ...
    360 * config.carrier_hz * ...
    config.channel_delay_us * 1e-6;

injected_error_deg = ...
    random_error_deg + ...
    bias_error_deg + ...
    delay_error_deg;


%% 7. Measured wrapped phase

measured_phase_unwrapped_deg = ...
    true_phase_unwrapped_deg + injected_error_deg;

measured_phase_wrapped_deg = ...
    wrap_to_180(measured_phase_unwrapped_deg);


%% 8. Geometry information for validation

source_bearing_deg = ...
    mod(atan2d(source_east_m, source_north_m), 360);

% Right-hand normal of H1 -> H2
baseline_normal_deg = mod(baseline_ref_deg + 90, 360);

angle_to_normal_deg = wrap_to_180( ...
    source_bearing_deg - baseline_normal_deg);


%% 9. Output

result = struct();

result.path_difference_m = path_difference_m;

result.true_phase_unwrapped_deg = ...
    true_phase_unwrapped_deg;

result.true_phase_wrapped_deg = ...
    true_phase_wrapped_deg;

result.measured_phase_wrapped_deg = ...
    measured_phase_wrapped_deg;

result.injected_error_deg = injected_error_deg;

result.random_error_deg = random_error_deg;
result.bias_error_deg   = bias_error_deg;
result.delay_error_deg  = delay_error_deg;

result.source_bearing_deg = source_bearing_deg;
result.angle_to_normal_deg = angle_to_normal_deg;

result.range_h1_m = range_h1_m;
result.range_h2_m = range_h2_m;

end


%% ========================================================================
function [east_m, north_m] = latlon_to_local_en(origin_deg, point_deg)
%LATLON_TO_LOCAL_EN 小范围 WGS84 经纬度转局部 East/North。
%
% 对数 km ～ 数十 km 范围足够用于当前仿真。

a  = 6378137.0;
e2 = 6.69437999014e-3;

lat0 = deg2rad(origin_deg(1));
lon0 = deg2rad(origin_deg(2));

lat = deg2rad(point_deg(1));
lon = deg2rad(point_deg(2));

sin_lat0 = sin(lat0);

Rn = a / sqrt(1 - e2*sin_lat0^2);
Rm = a*(1-e2) / ...
    (1 - e2*sin_lat0^2)^(3/2);

north_m = (lat - lat0) * Rm;
east_m  = (lon - lon0) * Rn * cos(lat0);

end


function value = wrap_to_180(value)
%WRAP_TO_180 Wrap angle to [-180, 180).

value = mod(value + 180, 360) - 180;

end


function config = set_default(config, field_name, default_value)

if ~isfield(config, field_name) || isempty(config.(field_name))
    config.(field_name) = default_value;
end

end