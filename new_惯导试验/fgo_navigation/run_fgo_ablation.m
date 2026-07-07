function results = run_fgo_ablation(in_dir, output_root, base_opts)
%RUN_FGO_ABLATION Run INS, depth-only, range-only, and fused graph cases.

    if nargin < 3
        base_opts = fgo_default_options();
    end

    cases = { ...
        'INS', false, false; ...
        'INS-Depth', true, false; ...
        'INS-Range', false, true; ...
        'INS-Depth-Range', true, true};

    results = struct();
    for i = 1:size(cases, 1)
        name = cases{i, 1};
        opts = base_opts;
        opts.use_depth = cases{i, 2};
        opts.use_range = cases{i, 3};
        opts.write_full_rate = true;
        output_dir = fullfile(output_root, name);
        fprintf('\n=== %s ===\n', name);
        results.(matlab.lang.makeValidName(name)) = ...
            fgo_experiment_rts_consistent(in_dir, output_dir, opts);
    end
end
