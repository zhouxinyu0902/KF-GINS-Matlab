function obs = propagate_beacon_observability(obs, phi)
%PROPAGATE_BEACON_OBSERVABILITY Accumulate IMU-rate state transitions.
%
% If phi maps x(k-1) to x(k), cumulative_phi maps the error state at the
% previous acoustic update to the current epoch.

    validateattributes(phi, {'numeric'}, ...
        {'size', [obs.rank, obs.rank], 'real', 'finite'});
    obs.cumulative_phi = phi * obs.cumulative_phi;
end
