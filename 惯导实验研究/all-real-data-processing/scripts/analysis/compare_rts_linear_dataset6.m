%% 第 6 组数据的 RTS 与线性桥接历史结果对比
clear; clc; close all;
script_dir = fileparts(mfilename('fullpath'));
topic_root = fileparts(fileparts(script_dir));
addpath(topic_root);
topic_paths = setup_all_real_data_processing();

dataset_id = 6;
base_input_dir = fullfile(topic_paths.external_input_root, 'input6');
base_output_dir = fullfile(topic_paths.navigation_results, 'dataset6');
artifact_output_dir = fullfile(topic_paths.figures_tables, 'dataset6');
path1 = fullfile(base_output_dir, 'Origin-rad.nav');
path2 = fullfile(base_output_dir, 'RTS-DoubleSmooth-rad.nav');
path3 = fullfile(base_output_dir, 'RTS-SingleSmooth-rad.nav');
% path4 = fullfile(base_output_dir, 'Linear-DoubleSmooth-rad.nav');
path5 = fullfile(base_output_dir, 'Linear-SingleSmooth-rad.nav');
truth1 = fullfile(base_input_dir, 'truth.nav');

[fig, finalExcelData] = calc_radial_error_gjb(truth1, path1, path2, path3, path5);
% 5. 导出 Excel
outputExcelName = fullfile(artifact_output_dir, ...
    sprintf('导航系统径向误差统计报告-RTS-LINEAR-%d.xlsx', dataset_id));
writecell(finalExcelData, outputExcelName);

% 6. 导出高清图片
outputImageName = fullfile(artifact_output_dir, ...
    sprintf('补偿前后误差对比-RTS-LINEAR-%d.png', dataset_id));
exportgraphics(fig, outputImageName, 'Resolution', 600);
