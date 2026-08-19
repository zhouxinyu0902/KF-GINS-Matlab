function obs = init_beacon_observability(beacon_positions, P0, range_std)
%INIT_BEACON_OBSERVABILITY Initialize finite-horizon observability logging.
%
% beacon_positions: [B x 3], each row is [lat(rad), lon(rad), h(m)].
% P0              : [15 x 15] initial covariance of the error-state filter.
% range_std       : horizontal range standard deviation in meters.

    validateattributes(beacon_positions, {'numeric'}, ...
        {'2d', 'ncols', 3, 'real', 'finite'});
    validateattributes(P0, {'numeric'}, ...
        {'2d', 'square', 'real', 'finite'});
    validateattributes(range_std, {'numeric'}, ...
        {'scalar', 'positive', 'real', 'finite'});

    obs.rank = size(P0, 1);
    obs.beacon_positions = beacon_positions;
    obs.range_std = range_std;
    obs.cumulative_phi = eye(obs.rank);

    % x = S*x_bar. Using prior standard deviations makes the Gramian
    % dimensionless and avoids comparing rad, m/s, rad/s, and m/s^2 directly.
    state_scale = sqrt(max(diag(P0), eps));
    obs.state_scale = diag(state_scale);

    obs.events = struct( ...
        'time', {}, ...
        'nav_pos', {}, ...
        'actual_beacon_id', {}, ...
        'phi_from_previous', {}, ...
        'H_candidates', {});
end
