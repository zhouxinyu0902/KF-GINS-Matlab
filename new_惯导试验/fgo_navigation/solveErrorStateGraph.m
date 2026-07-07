function solution = solveErrorStateGraph(graph, prior_covariance, opts)
%SOLVEERRORSTATEGRAPH Sparse IRLS solution of the linearized error graph.

    num_nodes = graph.num_nodes;
    rank = 15;
    num_variables = rank * num_nodes;
    dx_vector = zeros(num_variables, 1);
    range_weights = ones(length(graph.range_factors), 1);
    depth_weights = ones(length(graph.depth_factors), 1);
    cost_history = zeros(opts.max_irls_iterations, 1);

    scale_vector = repmat(opts.state_scale(:), num_nodes, 1);
    S = spdiags(scale_vector, 0, num_variables, num_variables);

    for iteration = 1:opts.max_irls_iterations
        [normal_matrix, rhs] = assembleGraph( ...
            graph, prior_covariance, range_weights, depth_weights);

        scaled_matrix = S * normal_matrix * S;
        scaled_rhs = S * rhs;
        scaled_matrix = (scaled_matrix + scaled_matrix') / 2;
        scaled_matrix = scaled_matrix + speye(num_variables) * 1e-12;

        warning_state = warning('off', 'MATLAB:nearlySingularMatrix');
        cleanup = onCleanup(@() warning(warning_state));
        scaled_solution = scaled_matrix \ scaled_rhs;
        clear cleanup;
        new_dx_vector = scale_vector .* scaled_solution;

        [range_residuals, depth_residuals] = normalizedResiduals( ...
            graph, new_dx_vector);
        new_range_weights = huberWeights(range_residuals, ...
            opts.range_huber_delta);
        new_depth_weights = huberWeights(depth_residuals, ...
            opts.depth_huber_delta);

        cost_history(iteration) = robustCost(range_residuals, ...
            opts.range_huber_delta) + robustCost(depth_residuals, ...
            opts.depth_huber_delta) + graphQuadraticCost( ...
            graph, prior_covariance, new_dx_vector);

        relative_change = norm(new_dx_vector - dx_vector) / ...
            max(1, norm(new_dx_vector));
        dx_vector = new_dx_vector;
        range_weights = new_range_weights;
        depth_weights = new_depth_weights;
        if relative_change < opts.irls_tolerance
            cost_history = cost_history(1:iteration);
            break;
        end
    end

    [range_residuals, depth_residuals] = normalizedResiduals(graph, dx_vector);
    solution.dx = reshape(dx_vector, rank, num_nodes);
    solution.cost = cost_history;
    solution.range_weights = range_weights;
    solution.depth_weights = depth_weights;
    solution.range_normalized_residuals = range_residuals;
    solution.depth_normalized_residuals = depth_residuals;
    solution.irls_iterations = length(cost_history);
end

function [A, b] = assembleGraph(graph, prior_covariance, ...
        range_weights, depth_weights)

    rank = 15;
    num_variables = rank * graph.num_nodes;
    A = sparse(num_variables, num_variables);
    b = zeros(num_variables, 1);

    W0 = covarianceInverse(prior_covariance);
    idx0 = 1:rank;
    A(idx0, idx0) = A(idx0, idx0) + W0;

    for k = 1:(graph.num_nodes - 1)
        Phi = graph.transitions{k};
        W = covarianceInverse(graph.process_covariances{k});
        idx_i = nodeIndices(k, rank);
        idx_j = nodeIndices(k + 1, rank);

        A(idx_i, idx_i) = A(idx_i, idx_i) + Phi' * W * Phi;
        A(idx_i, idx_j) = A(idx_i, idx_j) - Phi' * W;
        A(idx_j, idx_i) = A(idx_j, idx_i) - W * Phi;
        A(idx_j, idx_j) = A(idx_j, idx_j) + W;
    end

    for m = 1:length(graph.depth_factors)
        factor = graph.depth_factors(m);
        weight = depth_weights(m) / factor.sigma^2;
        [A, b] = addUnaryFactor(A, b, factor, weight, rank);
    end

    for m = 1:length(graph.range_factors)
        factor = graph.range_factors(m);
        weight = range_weights(m) / factor.sigma^2;
        [A, b] = addUnaryFactor(A, b, factor, weight, rank);
    end

    A = (A + A') / 2;
end

function [A, b] = addUnaryFactor(A, b, factor, weight, rank)
    idx = nodeIndices(factor.node, rank);
    H = factor.H;
    A(idx, idx) = A(idx, idx) + weight * (H' * H);
    b(idx) = b(idx) + weight * H' * factor.z;
end

function W = covarianceInverse(P)
    P = (P + P') / 2;
    [L, flag] = chol(P, 'lower');
    if flag == 0
        Linv = L \ eye(size(L));
        W = Linv' * Linv;
        return;
    end

    jitter = max(1e-20, 1e-12 * max(abs(diag(P))));
    for attempt = 1:10
        [L, flag] = chol(P + eye(size(P)) * jitter, 'lower');
        if flag == 0
            Linv = L \ eye(size(L));
            W = Linv' * Linv;
            return;
        end
        jitter = jitter * 10;
    end
    error('Could not invert a graph covariance block.');
end

function [range_residuals, depth_residuals] = normalizedResiduals(graph, x)
    range_residuals = zeros(length(graph.range_factors), 1);
    for m = 1:length(graph.range_factors)
        f = graph.range_factors(m);
        xi = x(nodeIndices(f.node, 15));
        range_residuals(m) = (f.z - f.H * xi) / f.sigma;
    end

    depth_residuals = zeros(length(graph.depth_factors), 1);
    for m = 1:length(graph.depth_factors)
        f = graph.depth_factors(m);
        xi = x(nodeIndices(f.node, 15));
        depth_residuals(m) = (f.z - f.H * xi) / f.sigma;
    end
end

function weights = huberWeights(residuals, delta)
    magnitude = abs(residuals);
    weights = ones(size(residuals));
    outlier = magnitude > delta;
    weights(outlier) = delta ./ magnitude(outlier);
end

function value = robustCost(residuals, delta)
    magnitude = abs(residuals);
    quadratic = magnitude <= delta;
    value = sum(residuals(quadratic).^2);
    value = value + sum(2 * delta * magnitude(~quadratic) - delta^2);
end

function value = graphQuadraticCost(graph, prior_covariance, x)
    rank = 15;
    x0 = x(1:rank);
    value = x0' * covarianceInverse(prior_covariance) * x0;
    for k = 1:(graph.num_nodes - 1)
        xi = x(nodeIndices(k, rank));
        xj = x(nodeIndices(k + 1, rank));
        residual = xj - graph.transitions{k} * xi;
        value = value + residual' * ...
            covarianceInverse(graph.process_covariances{k}) * residual;
    end
end

function idx = nodeIndices(node, rank)
    idx = (node - 1) * rank + (1:rank);
end
