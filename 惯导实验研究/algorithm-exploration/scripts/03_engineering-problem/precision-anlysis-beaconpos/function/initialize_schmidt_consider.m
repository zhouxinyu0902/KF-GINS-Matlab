function consider = initialize_schmidt_consider(state_count, ...
        beacon_initial_std_m, beacon_final_std_m, uncertainty_horizon_s, ...
        beacon_reference_time_s, start_time_s)
%INITIALIZE_SCHMIDT_CONSIDER Build a stacked two-axis consider model.

initial_std = reshape(beacon_initial_std_m, 1, []);
final_std = reshape(beacon_final_std_m, 1, []);
reference_time = reshape(beacon_reference_time_s, 1, []);
beacon_count = numel(initial_std);
if beacon_count < 1 || numel(final_std) ~= beacon_count || ...
        numel(reference_time) ~= beacon_count || state_count < 1 || ...
        uncertainty_horizon_s <= 0 || any(initial_std < 0) || ...
        any(final_std < initial_std)
    error('Schmidt/Consider初始化参数无效。');
end

consider.state_count = state_count;
consider.beacon_count = beacon_count;
consider.parameter_count = 2*beacon_count;
consider.Pxb = zeros(state_count, consider.parameter_count);
consider.Pbb = zeros(consider.parameter_count);
consider.initial_variance_m2 = initial_std.^2;
consider.maximum_variance_m2 = final_std.^2;
consider.variance_rate_m2_per_s = ...
    (final_std.^2-initial_std.^2)/uncertainty_horizon_s;
consider.reference_time_s = reference_time;
consider.current_time_s = start_time_s;
consider = set_consider_variance_at_time(consider, start_time_s);
end

function consider = set_consider_variance_at_time(consider, time_s)
for beacon_id = 1:consider.beacon_count
    age_s = max(time_s-consider.reference_time_s(beacon_id), 0);
    variance_m2 = consider.initial_variance_m2(beacon_id) ...
        + consider.variance_rate_m2_per_s(beacon_id)*age_s;
    variance_m2 = min(variance_m2, ...
        consider.maximum_variance_m2(beacon_id));
    columns = (2*beacon_id-1):(2*beacon_id);
    consider.Pbb(columns, columns) = variance_m2*eye(2);
end
consider.Pbb = 0.5*(consider.Pbb+consider.Pbb');
end
