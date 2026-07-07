function result = fgo_experiment_rts_consistent(in_dir, output_dir, user_opts)
%FGO_EXPERIMENT_RTS_CONSISTENT Run INS/depth/range factor-graph smoothing.

    if nargin < 2
        error('Usage: fgo_experiment_rts_consistent(in_dir, output_dir, opts)');
    end
    if nargin < 3
        user_opts = struct();
    end

    checkFgoDependencies();
    opts = mergeOptions(fgo_default_options(), user_opts);
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    cfg = config_1(in_dir);
    cfg.outputfolder = output_dir;
    [kf0, nav0] = myInitialize_15state(cfg);

    imudata = importdata(cfg.imufilepath);
    range_streams = { ...
        importdata(cfg.rangefile1path), ...
        importdata(cfg.rangefile2path), ...
        importdata(cfg.rangefile3path)};

    depthdata = loadDepthData(cfg, opts);

    cfg.starttime = max(cfg.starttime, imudata(1, 1));
    cfg.endtime = min(cfg.endtime, imudata(end, 1));
    imudata = imudata(imudata(:, 1) >= cfg.starttime & ...
        imudata(:, 1) <= cfg.endtime, :);
    depthdata = depthdata(depthdata(:, 1) >= cfg.starttime & ...
        depthdata(:, 1) <= cfg.endtime, :);

    [rangedata, beacon_ids] = buildRotatingRangeData( ...
        range_streams, cfg.starttime, cfg.endtime, opts);
    valid = rangedata(:, 1) >= cfg.starttime & rangedata(:, 1) <= cfg.endtime;
    rangedata = rangedata(valid, :);
    beacon_ids = beacon_ids(valid);

    if isempty(imudata) || isempty(depthdata) || isempty(rangedata)
        error('IMU, depth, and range data must all overlap the processing interval.');
    end

    rng(opts.random_seed);
    if opts.add_simulated_noise
        rangedata(:, 3) = rangedata(:, 3) + ...
            opts.range_std * randn(size(rangedata, 1), 1);
        depthdata(:, 2) = depthdata(:, 2) + ...
            opts.depth_std * randn(size(depthdata, 1), 1);
    end

    validateDataRates(imudata, depthdata);

    fprintf('Building graph nodes from %d IMU samples...\n', size(imudata, 1));
    graph_data = collectFgoNodes(imudata, depthdata, rangedata, ...
        beacon_ids, nav0, kf0, opts);
    if ~opts.use_depth
        graph_data.depth_factors = graph_data.depth_factors([]);
    end
    if ~opts.use_range
        graph_data.range_factors = graph_data.range_factors([]);
    end

    fprintf('Solving graph with %d nodes and %d range factors...\n', ...
        graph_data.num_nodes, length(graph_data.range_factors));
    solution = solveErrorStateGraph(graph_data, kf0.P0, opts);

    corrected_nodes = cell(graph_data.num_nodes, 1);
    for k = 1:graph_data.num_nodes
        corrected_nodes{k} = applyErrorCorrection( ...
            graph_data.navstates{k}, solution.dx(:, k), ...
            opts.apply_attitude_correction);
    end

    keyframe_path = fullfile(output_dir, 'FGO-Keyframes-rad.nav');
    writeNavStates(keyframe_path, corrected_nodes);

    full_rate_path = '';
    if opts.write_full_rate
        full_rate_path = fullfile(output_dir, 'FGO-FullRate-rad.nav');
        writeFullRateTrajectory(full_rate_path, ...
            imudata(1:graph_data.last_imu_index, :), nav0, ...
            graph_data.times, solution.dx, opts);
    end

    result.options = opts;
    result.cfg = cfg;
    result.graph = graph_data;
    result.solution = solution;
    result.corrected_nodes = corrected_nodes;
    result.keyframe_path = keyframe_path;
    result.full_rate_path = full_rate_path;

    save(fullfile(output_dir, 'FGO-result.mat'), 'result', '-v7.3');
    plotFgoDiagnostics(result, fullfile(output_dir, 'FGO-diagnostics.png'));

    fprintf('FGO complete. Final weighted cost: %.6g\n', solution.cost(end));
end

function depthdata = loadDepthData(cfg, opts)
    switch lower(opts.depth_source)
        case 'truth'
            truth = importdata(cfg.truthpath);
            depthdata = truth(:, [2, 5]);
        case 'file'
            depthdata = importdata(cfg.heightfilepath);
            if size(depthdata, 2) < 2
                error('Depth file must contain at least [time, depth].');
            end
            depthdata = depthdata(:, 1:2);
        otherwise
            error('Unknown depth_source: %s', opts.depth_source);
    end
end

function opts = mergeOptions(defaults, user)
    opts = defaults;
    names = fieldnames(user);
    for i = 1:length(names)
        opts.(names{i}) = user.(names{i});
    end
end

function validateDataRates(imudata, depthdata)
    imu_dt = median(diff(imudata(:, 1)));
    depth_dt = median(diff(depthdata(:, 1)));
    fprintf('Median IMU dt: %.6f s, depth dt: %.6f s\n', imu_dt, depth_dt);
    if abs(imu_dt - 0.01) > 0.002
        warning('IMU data are not close to 100 Hz.');
    end
    if abs(depth_dt - 0.01) > 0.002
        warning('Depth data are not close to 100 Hz.');
    end
end
