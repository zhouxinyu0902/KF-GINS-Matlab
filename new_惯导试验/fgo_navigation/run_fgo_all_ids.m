function results = run_fgo_all_ids(input_root, output_root, ids, opts)
%RUN_FGO_ALL_IDS Run the factor graph for several input folders.
%
% Example:
%   run_fgo_all_ids('F:/2_Data/.../All_data', 'D:/FGO-output', 1:8);

    if nargin < 3 || isempty(ids)
        ids = 1:8;
    end
    if nargin < 4
        opts = fgo_default_options();
    end

    results = cell(length(ids), 1);
    for i = 1:length(ids)
        id = ids(i);
        in_dir = fullfile(input_root, ['input', num2str(id)]);
        out_dir = fullfile(output_root, ['output', num2str(id)]);
        fprintf('\n=== Running data set %d ===\n', id);
        results{i} = fgo_experiment_rts_consistent(in_dir, out_dir, opts);
    end
end
