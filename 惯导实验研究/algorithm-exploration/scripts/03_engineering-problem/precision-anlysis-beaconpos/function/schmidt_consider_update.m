function [kf, consider, diagnostics] = schmidt_consider_update( ...
        kf, consider, measurement_matrix, beacon_matrix, ...
        measurement_residual, measurement_covariance)
%SCHMIDT_CONSIDER_UPDATE Update active INS states without estimating beacons.
%   The consider-parameter mean is fixed at zero. Pbb is never contracted by
%   a measurement; only Pxx and Pxb are updated.

state_count = kf.RANK;
consider_count = size(consider.Pbb, 1);
if ~isequal(size(kf.P), [state_count, state_count]) || ...
        ~isequal(size(consider.Pxb), [state_count, consider_count]) || ...
        size(measurement_matrix, 2) ~= state_count || ...
        size(beacon_matrix, 2) ~= consider_count
    error('Schmidt/Consider矩阵维度不一致。');
end

Pxx_prior = kf.P;
Pxb_prior = consider.Pxb;
Pbb_prior = consider.Pbb;
innovation = measurement_residual - measurement_matrix*kf.x;

cross_left = measurement_matrix*Pxb_prior*beacon_matrix';
beacon_term = beacon_matrix*Pbb_prior*beacon_matrix';
innovation_covariance = measurement_matrix*Pxx_prior*measurement_matrix' ...
    + cross_left + cross_left' + beacon_term + measurement_covariance;
innovation_covariance = 0.5*(innovation_covariance+innovation_covariance');

[~, chol_flag] = chol(innovation_covariance);
if chol_flag ~= 0 || any(~isfinite(innovation_covariance), 'all')
    error('Schmidt/Consider创新协方差不是有限正定矩阵。');
end

active_gain = (Pxx_prior*measurement_matrix' ...
    + Pxb_prior*beacon_matrix')/innovation_covariance;
kf.x = kf.x + active_gain*innovation;
kf.P = Pxx_prior-active_gain*innovation_covariance*active_gain';
kf.P = 0.5*(kf.P+kf.P');
consider.Pxb = Pxb_prior-active_gain*( ...
    measurement_matrix*Pxb_prior+beacon_matrix*Pbb_prior);

% Pbb is deliberately unchanged by a Schmidt measurement update.
consider.Pbb = Pbb_prior;

diagnostics.innovation = innovation;
diagnostics.innovation_covariance = innovation_covariance;
diagnostics.active_gain = active_gain;
diagnostics.beacon_term = beacon_term;
diagnostics.cross_term = cross_left+cross_left';
diagnostics.pxb_norm_before = norm(Pxb_prior, 'fro');
diagnostics.pxb_norm_after = norm(consider.Pxb, 'fro');
diagnostics.pbb_change_norm = norm(consider.Pbb-Pbb_prior, 'fro');
diagnostics.nis = innovation'/innovation_covariance*innovation;
end
