function opts = fgo_default_options()
%FGO_DEFAULT_OPTIONS Configuration for the RTS-consistent factor graph.

    opts.node_interval_sec = 1.0;
    opts.range_interval_sec = 420;
    opts.range_selection_mode = 'legacy_rows'; % 'legacy_rows' or 'timestamp'

    opts.range_std = 6.0;
    opts.depth_std = 0.4;
    opts.use_range = true;
    opts.use_depth = true;
    opts.depth_source = 'truth'; % 'truth' reproduces the RTS simulation
    opts.add_simulated_noise = true;
    opts.random_seed = 1;

    opts.range_huber_delta = 3.0;
    opts.depth_huber_delta = 4.0;
    opts.max_irls_iterations = 6;
    opts.irls_tolerance = 1e-4;

    opts.apply_attitude_correction = false;
    opts.write_full_rate = true;
    opts.stop_at_last_range = true;

    % Numerical floors in the native 15-state units.
    opts.process_std_floor = [ ...
        1e-12; 1e-12; 1e-5; ...
        1e-6; 1e-6; 1e-6; ...
        1e-9; 1e-9; 1e-9; ...
        1e-12; 1e-12; 1e-12; ...
        1e-9; 1e-9; 1e-9];

    % Variable scaling used only for numerical conditioning.
    opts.state_scale = [ ...
        1e-6; 1e-6; 1; ...
        0.1; 0.1; 0.1; ...
        1e-3; 1e-3; 1e-3; ...
        1e-6; 1e-6; 1e-6; ...
        1e-4; 1e-4; 1e-4];
end
