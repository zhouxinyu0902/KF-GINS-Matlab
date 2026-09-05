function selection = select_phase_azimuth_candidate( ...
        wrapped_phase_deg, navstate, beacon_position, options)
%SELECT_PHASE_AZIMUTH_CANDIDATE 用导航先验消除相位周期及线阵前后模糊。
%   beacon_position = [lat(rad), lon(rad), h(m)]。
%   输出 selected_relative_azimuth_deg 与 update_range_azimuth_filter_rad
%   的方位角定义一致：潜标绝对方位减航向，再减基线右侧法向偏置。

    required_fields = {'baseline_m', 'carrier_hz', 'sound_speed_mps', ...
        'baseline_install_deg'};
    for index = 1:numel(required_fields)
        if ~isfield(options, required_fields{index}) || ...
                isempty(options.(required_fields{index}))
            error('options.%s 不能为空。', required_fields{index});
        end
    end
    if numel(beacon_position) < 2 || any(~isfinite(beacon_position(1:2)))
        error('beacon_position 至少应包含有限的 [lat(rad), lon(rad)]。');
    end

    phase_candidates = phase_difference_to_angle_candidates( ...
        wrapped_phase_deg, options.baseline_m, options.carrier_hz, ...
        options.sound_speed_mps);

    bearing_deg = initial_bearing_deg(navstate.pos(1), navstate.pos(2), ...
        beacon_position(1), beacon_position(2));
    heading_deg = rad2deg(navstate.att(3));
    predicted_relative_azimuth_deg = wrap180( ...
        bearing_deg - heading_deg - options.baseline_install_deg - 90);
    predicted_principal_angle_deg = ...
        asind(sind(predicted_relative_azimuth_deg));

    % 每个主值角还对应线阵前后两个全方位角分支。
    principal = phase_candidates.principal_angle_deg(:);
    full_branch_a = wrap180(principal);
    full_branch_b = wrap180(180 - principal);
    full_candidates = [full_branch_a; full_branch_b];
    source_candidate_index = [(1:numel(principal))'; (1:numel(principal))'];

    rounded = round(full_candidates * 1e10) / 1e10;
    [~, unique_index] = unique(rounded, 'stable');
    full_candidates = full_candidates(unique_index);
    source_candidate_index = source_candidate_index(unique_index);

    full_difference_deg = abs(wrap180( ...
        full_candidates - predicted_relative_azimuth_deg));
    [selected_difference_deg, selected_full_index] = min(full_difference_deg);
    selected_source_index = source_candidate_index(selected_full_index);

    selection = struct();
    selection.measured_phase_deg = wrapped_phase_deg;
    selection.predicted_bearing_deg = bearing_deg;
    selection.predicted_relative_azimuth_deg = ...
        predicted_relative_azimuth_deg;
    selection.predicted_principal_angle_deg = ...
        predicted_principal_angle_deg;
    selection.candidate_cycle_k = phase_candidates.cycle_k;
    selection.candidate_principal_angle_deg = principal;
    selection.candidate_full_azimuth_deg = full_candidates;
    selection.candidate_full_difference_deg = full_difference_deg;
    selection.selected_cycle_k = ...
        phase_candidates.cycle_k(selected_source_index);
    selection.selected_principal_angle_deg = principal(selected_source_index);
    selection.selected_relative_azimuth_deg = ...
        full_candidates(selected_full_index);
    selection.selected_difference_deg = selected_difference_deg;
end

function bearing_deg = initial_bearing_deg(lat1, lon1, lat2, lon2)
    delta_lon = lon2 - lon1;
    east = cos(lat2) .* sin(delta_lon);
    north = cos(lat1) .* sin(lat2) - ...
        sin(lat1) .* cos(lat2) .* cos(delta_lon);
    bearing_deg = mod(atan2d(east, north), 360);
end

function angle_deg = wrap180(angle_deg)
    angle_deg = mod(angle_deg + 180, 360) - 180;
end

