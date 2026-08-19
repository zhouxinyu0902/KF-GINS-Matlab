%% 因子图与 KF/RTS 基线结果评估
clear; clc; close all;
script_dir = fileparts(mfilename('fullpath'));
topic_root = fileparts(fileparts(script_dir));
addpath(topic_root);
paths = setup_factor_graph_navigation();

truth_path = fullfile(paths.input, 'truth.nav');

groups = {
    'range-ins', {
        'KF-RANGE-INS.nav', ...
        'FGO-RANGE-INS-accurate.nav', ...
        'FGO-RANGE-INS-HEIGHT-full15-accurate.nav'};
    'gnss-ins', {
        'KF-GNSS-INS.nav', ...
        'FGO-GNSS-INS-keyframes.nav', ...
        'Standard-FGO-GNSS-INS-keyframes.nav'};
    'kf-rts', {
        'KF-RANGE-INS.nav', ...
        'KF-RANGE-INS-Proposed two-stage RTS.nav', ...
        'KF-RANGE-INS-Single-stage RTS.nav'};
    };

for group_index = 1:size(groups, 1)
    group_name = groups{group_index, 1};
    requested_names = groups{group_index, 2};
    result_paths = cellfun(@(name) fullfile(paths.navigation_results, name), ...
        requested_names, 'UniformOutput', false);
    exists_mask = cellfun(@isfile, result_paths);

    if nnz(exists_mask) < 2
        warning('%s 可用结果少于两个，跳过。', group_name);
        continue;
    end

    result_paths = result_paths(exists_mask);
    [fig, table_data] = calc_radial_error_gjb(truth_path, result_paths{:});
    title(sprintf('%s comparison', upper(group_name)));
    xlabel('Time (s)');
    ylabel('Horizontal radial error (m)');

    writecell(table_data, fullfile(paths.figures_tables, ...
        sprintf('%s-error-summary.xlsx', group_name)));
    exportgraphics(fig, fullfile(paths.figures_tables, ...
        sprintf('%s-radial-error.png', group_name)), 'Resolution', 600);
    close(fig);
end

fprintf('因子图结果评估完成：%s\n', paths.figures_tables);
