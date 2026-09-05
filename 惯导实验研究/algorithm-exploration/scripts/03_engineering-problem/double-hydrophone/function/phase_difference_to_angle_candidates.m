function candidates = phase_difference_to_angle_candidates( ...
        wrapped_phase_deg, baseline_m, carrier_hz, sound_speed_mps)
%PHASE_DIFFERENCE_TO_ANGLE_CANDIDATES 将包裹相位差转换为全部可行到达角。
%   到达角以 H1->H2 基线右侧法向为零，主值范围为 [-90, 90] deg。
%   相位模型为 Delta_phi = 360*d/lambda*sin(theta) + 360*k。

    arguments
        wrapped_phase_deg (1, 1) double {mustBeFinite}
        baseline_m (1, 1) double {mustBePositive}
        carrier_hz (1, 1) double {mustBePositive}
        sound_speed_mps (1, 1) double {mustBePositive}
    end

    wavelength_m = sound_speed_mps / carrier_hz;
    maximum_unwrapped_phase_deg = 360 * baseline_m / wavelength_m;
    k_min = ceil((-maximum_unwrapped_phase_deg - wrapped_phase_deg) / 360);
    k_max = floor((maximum_unwrapped_phase_deg - wrapped_phase_deg) / 360);
    cycle_k = (k_min:k_max)';

    unwrapped_phase_deg = wrapped_phase_deg + 360 * cycle_k;
    sin_theta = wavelength_m / (360 * baseline_m) .* unwrapped_phase_deg;
    numerical_tolerance = 32 * eps(max(1, max(abs(sin_theta))));
    valid = abs(sin_theta) <= 1 + numerical_tolerance;
    cycle_k = cycle_k(valid);
    unwrapped_phase_deg = unwrapped_phase_deg(valid);
    sin_theta = max(-1, min(1, sin_theta(valid)));

    if isempty(cycle_k)
        error('当前相位差没有满足 |sin(theta)|<=1 的物理解。');
    end

    candidates = struct( ...
        'cycle_k', cycle_k, ...
        'unwrapped_phase_deg', unwrapped_phase_deg, ...
        'principal_angle_deg', asind(sin_theta), ...
        'wavelength_m', wavelength_m);
end

