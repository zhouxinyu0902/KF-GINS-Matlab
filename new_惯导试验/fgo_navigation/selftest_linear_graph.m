function selftest_linear_graph()
%SELFTEST_LINEAR_GRAPH Test the sparse graph solver without navigation data.

    opts = fgo_default_options();
    opts.max_irls_iterations = 3;
    opts.state_scale = ones(15, 1);

    graph.num_nodes = 3;
    graph.transitions = {eye(15), eye(15)};
    graph.process_covariances = {eye(15) * 0.1, eye(15) * 0.1};
    graph.depth_factors = repmat(struct( ...
        'node', 0, 'time', 0, 'z', 0, 'H', zeros(1, 15), ...
        'sigma', 0.1, 'sample_count', 1), 3, 1);
    for k = 1:3
        graph.depth_factors(k).node = k;
        graph.depth_factors(k).H(3) = 1;
        graph.depth_factors(k).z = k - 1;
    end
    graph.range_factors = repmat(struct( ...
        'node', 0, 'time', 0, 'z', 0, 'H', zeros(1, 15), ...
        'sigma', 0.1, 'beacon_id', 1, 'predicted', 0, ...
        'measured', 0), 0, 1);

    solution = solveErrorStateGraph(graph, eye(15), opts);
    assert(all(isfinite(solution.dx(:))));
    assert(solution.dx(3, 3) > solution.dx(3, 1));
    fprintf('selftest_linear_graph passed.\n');
end
