function [kf, consider, diagnostics] = ...
        schmidt_consider_range_update(navstate, range_data, depth_data, ...
        kf, consider, beacon_id, position_error_unit, fast_range_std_m, ...
        nis_robust_options)
%SCHMIDT_CONSIDER_RANGE_UPDATE Joint horizontal-range and depth update.
%   The existing residual convention is retained:
%       Z = predicted range from nominal beacon - measured range.
%   The consider parameter is nominal beacon position minus true position,
%   so Hb = [-uN, -uE] for the active beacon.

if nargin < 8
    fast_range_std_m = 0;
end
if nargin < 9 || isempty(nis_robust_options)
    nis_robust_options = struct( ...
        'enabled', false, ...
        'enable_rejection', false, ...
        'soft_threshold', 3.841, ...
        'reject_threshold', 10.828, ...
        'minimum_weight', 0.10);
end
if beacon_id < 1 || beacon_id > consider.beacon_count || ...
        beacon_id ~= fix(beacon_id)
    error('当前潜标编号超出Schmidt/Consider模型范围。');
end

param = Param();
beacon_llh = range_data(4:6)';
[rm, rn] = getRmRn(navstate.pos(1), param);
h = navstate.pos(3);
DR = diag([rm+h, (rn+h)*cos(navstate.pos(1)), -1]);
delta_position_m = (DR*(navstate.pos-beacon_llh))';
horizontal_range_m = norm(delta_position_m(1:2));
if horizontal_range_m <= 1e-8 || ~isfinite(horizontal_range_m)
    error('载体与名义潜标的水平距离过小或无效。');
end
u_ne = delta_position_m(1:2)/horizontal_range_m;

measurement_residual = [horizontal_range_m-range_data(3); ...
    navstate.pos(3)-depth_data(2)];
measurement_matrix = zeros(2, kf.RANK);
switch lower(string(position_error_unit))
    case "rad"
        radial_derivative = (navstate.pos'-beacon_llh') ...
            *(DR^2)/horizontal_range_m;
        measurement_matrix(1, 1:2) = radial_derivative(1:2);
        measurement_matrix(2, 3) = 1;
    case "m"
        measurement_matrix(1, 1:2) = u_ne;
        measurement_matrix(2, 3) = -1;
    otherwise
        error('position_error_unit只能为"rad"或"m"。');
end

beacon_matrix = zeros(2, consider.parameter_count);
columns = (2*beacon_id-1):(2*beacon_id);
beacon_matrix(1, columns) = -u_ne;
range_variance = kf.rangstd^2+fast_range_std_m^2;
measurement_covariance = diag([range_variance, kf.depthstd^2]);

kf.Z = measurement_residual;
kf.Zkk_1 = measurement_matrix*kf.x;

% Gate only the horizontal range row. The depth row remains available even
% when a gross range innovation is rejected.
range_innovation = measurement_residual(1)-measurement_matrix(1, :)*kf.x;
range_cross_left = measurement_matrix(1, :)*consider.Pxb* ...
    beacon_matrix(1, :)';
range_beacon_term = beacon_matrix(1, :)*consider.Pbb* ...
    beacon_matrix(1, :)';
range_nonmeasurement_variance = measurement_matrix(1, :)*kf.P* ...
    measurement_matrix(1, :)'+2*range_cross_left+range_beacon_term;
range_nonmeasurement_variance = max(0, range_nonmeasurement_variance);
nis_decision = nis_robust_range_policy(range_innovation, ...
    range_nonmeasurement_variance, range_variance, nis_robust_options);

if nis_decision.action_code == 2
    selected_rows = 2;
else
    selected_rows = 1:2;
    measurement_covariance(1, 1) = ...
        nis_decision.effective_measurement_variance_m2;
end
[kf, consider, update_diagnostics] = schmidt_consider_update( ...
    kf, consider, measurement_matrix(selected_rows, :), ...
    beacon_matrix(selected_rows, :), measurement_residual(selected_rows), ...
    measurement_covariance(selected_rows, selected_rows));

% Recompute the horizontal range residual after the active-state feedback
% estimate, matching the diagnostic convention used by existing updates.
if lower(string(position_error_unit)) == "rad"
    corrected_position = navstate.pos-kf.x(1:3);
else
    DR_inverse = diag([1/(rm+h), ...
        1/((rn+h)*cos(navstate.pos(1))), -1]);
    corrected_position = navstate.pos-DR_inverse*kf.x(1:3);
end
corrected_delta_m = (DR*(corrected_position-beacon_llh))';
kf.Znew = norm(corrected_delta_m(1:2))-range_data(3);

diagnostics = update_diagnostics;
diagnostics.time_s = range_data(1);
diagnostics.beacon_id = beacon_id;
diagnostics.u_ne = u_ne;
diagnostics.range_innovation_m = range_innovation;
diagnostics.range_innovation_variance_m2 = ...
    nis_decision.base_innovation_variance_m2;
diagnostics.effective_range_innovation_variance_m2 = ...
    nis_decision.effective_innovation_variance_m2;
diagnostics.beacon_los_variance_m2 = range_beacon_term;
diagnostics.cross_los_variance_m2 = 2*range_cross_left;
diagnostics.range_nis = nis_decision.raw_nis;
diagnostics.effective_range_nis = nis_decision.effective_nis;
diagnostics.nis_information_weight = nis_decision.information_weight;
diagnostics.nis_action_code = nis_decision.action_code;
diagnostics.sigma_ne_m = sqrt(diag( ...
    consider.Pbb(columns, columns)))';
end
