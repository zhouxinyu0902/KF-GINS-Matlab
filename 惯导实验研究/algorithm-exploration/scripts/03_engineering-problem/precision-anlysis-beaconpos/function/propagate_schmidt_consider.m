function consider = propagate_schmidt_consider( ...
        consider, state_transition, new_time_s)
%PROPAGATE_SCHMIDT_CONSIDER Propagate Pxb at every INS covariance step.

if ~isequal(size(state_transition), ...
        [consider.state_count, consider.state_count])
    error('Schmidt/Consider状态转移矩阵维度不一致。');
end
if new_time_s < consider.current_time_s-1e-9
    error('Schmidt/Consider传播时间不能倒退。');
end

consider.Pxb = state_transition*consider.Pxb;
for beacon_id = 1:consider.beacon_count
    age_s = max(new_time_s-consider.reference_time_s(beacon_id), 0);
    variance_m2 = consider.initial_variance_m2(beacon_id) ...
        + consider.variance_rate_m2_per_s(beacon_id)*age_s;
    variance_m2 = min(variance_m2, ...
        consider.maximum_variance_m2(beacon_id));
    columns = (2*beacon_id-1):(2*beacon_id);
    consider.Pbb(columns, columns) = variance_m2*eye(2);
end
consider.Pbb = 0.5*(consider.Pbb+consider.Pbb');
consider.current_time_s = new_time_s;
end
