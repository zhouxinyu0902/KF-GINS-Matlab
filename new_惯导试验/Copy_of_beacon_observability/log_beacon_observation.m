function obs = log_beacon_observation(obs, time, nav_pos, actual_beacon_pos)
%LOG_BEACON_OBSERVATION Log one acoustic measurement epoch.
%
% nav_pos and actual_beacon_pos use [lat(rad); lon(rad); h(m)].
% H_candidates contains the horizontal-range Jacobian for every candidate
% beacon evaluated at the same vehicle state. This permits a fair offline
% comparison between alternating and fixed-beacon schedules.

    nav_pos = nav_pos(:);
    actual_beacon_pos = actual_beacon_pos(:);
    validateattributes(nav_pos, {'numeric'}, {'size', [3, 1], 'finite'});
    validateattributes(actual_beacon_pos, {'numeric'}, ...
        {'size', [3, 1], 'finite'});

    num_beacons = size(obs.beacon_positions, 1);
    H_candidates = zeros(num_beacons, obs.rank);
    beacon_distance = zeros(num_beacons, 1);

    for beacon_id = 1:num_beacons
        beacon_pos = obs.beacon_positions(beacon_id, :)';
        H_candidates(beacon_id, :) = horizontal_range_jacobian( ...
            nav_pos, beacon_pos, obs.rank);
        beacon_distance(beacon_id) = local_position_distance( ...
            actual_beacon_pos, beacon_pos);
    end

    [~, actual_beacon_id] = min(beacon_distance);

    event.time = time;
    event.nav_pos = nav_pos;
    event.actual_beacon_id = actual_beacon_id;
    event.phi_from_previous = obs.cumulative_phi;
    event.H_candidates = H_candidates;
    obs.events(end + 1) = event;

    obs.cumulative_phi = eye(obs.rank);
end

function H = horizontal_range_jacobian(nav_pos, beacon_pos, state_rank)
    param = Param();
    [rm, rn] = getRmRn(beacon_pos(1), param);
    h = beacon_pos(3);
    DR = diag([rm + h, (rn + h) * cos(beacon_pos(1)), -1]);

    delta_pos = DR * (nav_pos - beacon_pos);
    horizontal_range = norm(delta_pos(1:2));
    horizontal_range = max(horizontal_range, 1e-6);

    b = (nav_pos - beacon_pos)' * (DR ^ 2) / horizontal_range;
    H = zeros(1, state_rank);
    H(1:2) = b(1:2);
end

function distance = local_position_distance(pos_a, pos_b)
    param = Param();
    mean_lat = 0.5 * (pos_a(1) + pos_b(1));
    mean_h = 0.5 * (pos_a(3) + pos_b(3));
    [rm, rn] = getRmRn(mean_lat, param);
    DR = diag([rm + mean_h, (rn + mean_h) * cos(mean_lat), -1]);
    distance = norm(DR * (pos_a - pos_b));
end
