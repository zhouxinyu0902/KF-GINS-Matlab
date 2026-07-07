function graph = collectFgoNodes(imudata, depthdata, rangedata, ...
        beacon_ids, nav0, kf_template, opts)
%COLLECTFGONODES Run nominal INS and accumulate Phi/Q between graph nodes.

    imu_times = imudata(:, 1);
    range_indices = nearestIndices(imu_times, rangedata(:, 1));
    range_alignment = imu_times(range_indices) - rangedata(:, 1);
    imu_dt = median(diff(imu_times));
    if max(abs(range_alignment)) > max(0.02, 0.55 * imu_dt)
        error('Range-to-IMU alignment error is too large (max %.6f s).', ...
            max(abs(range_alignment)));
    end

    if opts.stop_at_last_range
        last_imu_index = range_indices(end);
    else
        last_imu_index = size(imudata, 1);
    end

    regular_times = (imu_times(1):opts.node_interval_sec: ...
        imu_times(last_imu_index))';
    regular_indices = nearestIndices(imu_times, regular_times);
    node_indices = unique([1; regular_indices; range_indices]);
    node_indices = node_indices(node_indices <= last_imu_index);
    node_times = imu_times(node_indices);
    num_nodes = length(node_indices);

    range_node = zeros(size(range_indices));
    [found, range_node] = ismember(range_indices, node_indices);
    if ~all(found)
        error('A forced range node was lost while constructing graph nodes.');
    end

    navstates = cell(num_nodes, 1);
    transitions = cell(num_nodes - 1, 1);
    process_covariances = cell(num_nodes - 1, 1);

    nav = nav0;
    navstates{1} = nav;
    Phi_acc = eye(15);
    Q_acc = zeros(15);
    next_node = 2;

    for imu_index = 2:last_imu_index
        lastimu = imudata(imu_index - 1, :)';
        thisimu = imudata(imu_index, :)';
        dt = thisimu(1) - lastimu(1);
        if dt <= 0
            error('IMU timestamps must be strictly increasing.');
        end

        nav = InsMech(nav, lastimu, thisimu);

        kf_step = kf_template;
        kf_step.P = zeros(15);
        kf_step.x = zeros(15, 1);
        kf_step = myInsPropagate_15state(nav, thisimu, dt, kf_step);
        Phi_step = kf_step.phi;
        Q_step = (kf_step.P + kf_step.P') / 2;

        Q_acc = Phi_step * Q_acc * Phi_step' + Q_step;
        Phi_acc = Phi_step * Phi_acc;

        if next_node <= num_nodes && imu_index == node_indices(next_node)
            transitions{next_node - 1} = Phi_acc;
            process_covariances{next_node - 1} = ...
                regularizeProcessCovariance(Q_acc, opts.process_std_floor);
            navstates{next_node} = nav;

            Phi_acc = eye(15);
            Q_acc = zeros(15);
            next_node = next_node + 1;
        end
    end

    if next_node <= num_nodes
        error('Not all graph nodes were reached during INS propagation.');
    end

    depth_margin = 0.5 * opts.node_interval_sec;
    depth_used = depthdata(depthdata(:, 1) >= node_times(1) - depth_margin & ...
        depthdata(:, 1) <= node_times(end) + depth_margin, :);
    if isempty(depth_used)
        error('No depth samples overlap the graph-node interval.');
    end
    depth_factors = buildDepthFactors(node_times, navstates, depth_used, opts);
    range_factors = buildRangeFactors(range_node, navstates, ...
        rangedata, beacon_ids, opts);

    graph.num_nodes = num_nodes;
    graph.last_imu_index = last_imu_index;
    graph.node_indices = node_indices;
    graph.times = node_times;
    graph.navstates = navstates;
    graph.transitions = transitions;
    graph.process_covariances = process_covariances;
    graph.depth_factors = depth_factors;
    graph.range_factors = range_factors;
    graph.range_alignment_error = range_alignment;
end

function factors = buildDepthFactors(node_times, navstates, depthdata, opts)
    if length(node_times) == 1
        edges = [-inf; inf];
    else
        edges = [-inf; (node_times(1:end-1) + node_times(2:end)) / 2; inf];
    end
    bin = discretize(depthdata(:, 1), edges);

    template = struct('node', 0, 'time', 0, 'z', 0, ...
        'H', zeros(1, 15), 'sigma', 0, 'sample_count', 0);
    factors = repmat(template, length(node_times), 1);
    H = zeros(1, 15);
    H(3) = 1;

    for k = 1:length(node_times)
        values = depthdata(bin == k, 2);
        if isempty(values)
            [~, idx] = min(abs(depthdata(:, 1) - node_times(k)));
            values = depthdata(idx, 2);
        end
        measurement = median(values);
        factors(k).node = k;
        factors(k).time = node_times(k);
        factors(k).z = navstates{k}.pos(3) - measurement;
        factors(k).H = H;
        factors(k).sigma = opts.depth_std;
        factors(k).sample_count = length(values);
    end
end

function factors = buildRangeFactors(range_node, navstates, ...
        rangedata, beacon_ids, opts)

    param = Param();
    template = struct('node', 0, 'time', 0, 'z', 0, ...
        'H', zeros(1, 15), 'sigma', 0, 'beacon_id', 0, ...
        'predicted', 0, 'measured', 0);
    factors = repmat(template, length(range_node), 1);

    for r = 1:length(range_node)
        node = range_node(r);
        nav = navstates{node};
        beacon = rangedata(r, 4:6)';
        [rm, rn] = getRmRn(beacon(1), param);
        h = beacon(3);
        DR = diag([rm + h, (rn + h) * cos(beacon(1)), -1]);
        delta = DR * (nav.pos - beacon);
        predicted = max(norm(delta(1:2)), 1e-6);

        derivative = (nav.pos' - beacon') * (DR^2) / predicted;
        H = zeros(1, 15);
        H(1:2) = derivative(1:2);

        factors(r).node = node;
        factors(r).time = rangedata(r, 1);
        factors(r).z = predicted - rangedata(r, 3);
        factors(r).H = H;
        factors(r).sigma = opts.range_std;
        factors(r).beacon_id = beacon_ids(r);
        factors(r).predicted = predicted;
        factors(r).measured = rangedata(r, 3);
    end
end

function indices = nearestIndices(reference_times, query_times)
    indices = interp1(reference_times, (1:length(reference_times))', ...
        query_times, 'nearest', 'extrap');
    indices = round(indices);
end

function Q = regularizeProcessCovariance(Q, std_floor)
    Q = (Q + Q') / 2;
    Q = Q + diag(std_floor(:).^2);
    [~, p] = chol(Q);
    if p == 0
        return;
    end

    jitter = max(1e-20, 1e-12 * max(abs(diag(Q))));
    for attempt = 1:10
        Q_try = Q + eye(size(Q)) * jitter;
        [~, p] = chol(Q_try);
        if p == 0
            Q = Q_try;
            return;
        end
        jitter = jitter * 10;
    end
    error('Accumulated process covariance is not positive definite.');
end
