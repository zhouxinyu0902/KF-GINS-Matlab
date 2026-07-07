function plotFgoDiagnostics(result, output_path)
%PLOTFGODIAGNOSTICS Plot residual and correction diagnostics.

    graph = result.graph;
    solution = result.solution;
    dx = solution.dx;
    num_nodes = graph.num_nodes;

    horizontal_correction = zeros(num_nodes, 1);
    for k = 1:num_nodes
        nav = graph.navstates{k};
        horizontal_correction(k) = norm([ ...
            (nav.Rm + nav.pos(3)) * dx(1, k); ...
            (nav.Rn + nav.pos(3)) * cos(nav.pos(1)) * dx(2, k)]);
    end

    fig = figure('Color', 'w', 'Position', [100, 100, 1100, 750]);
    layout = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(solution.cost, 'o-', 'LineWidth', 1.2);
    xlabel('IRLS iteration');
    ylabel('Weighted cost');
    title('Solver convergence');
    grid on;

    nexttile;
    stem([graph.range_factors.time], ...
        solution.range_normalized_residuals, 'filled');
    yline(result.options.range_huber_delta, '--');
    yline(-result.options.range_huber_delta, '--');
    xlabel('Time (s)');
    ylabel('Normalized residual');
    title('Acoustic range residuals');
    grid on;

    nexttile;
    plot(graph.times, solution.depth_normalized_residuals, 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Normalized residual');
    title('Depth residuals');
    grid on;

    nexttile;
    plot(graph.times, horizontal_correction, 'LineWidth', 1.2);
    xlabel('Time (s)');
    ylabel('Horizontal correction (m)');
    title('Smoothed position correction');
    grid on;

    title(layout, 'RTS-consistent error-state factor graph');
    exportgraphics(fig, output_path, 'Resolution', 200);
    close(fig);
end
