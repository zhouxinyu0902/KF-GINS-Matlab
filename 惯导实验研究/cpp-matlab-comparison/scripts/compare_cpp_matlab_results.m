%% MATLAB 与 C++ 的 EKF/一次 RTS/二次 RTS 结果对比
clear; clc; close all;
script_dir = fileparts(mfilename('fullpath'));
topic_root = fileparts(script_dir);
addpath(topic_root);
paths = setup_cpp_matlab_comparison();

truth_path = fullfile(paths.external_input, 'truth.nav');
result_paths = {
    fullfile(paths.matlab_results, 'Matlab-Origin.nav');
    fullfile(paths.matlab_results, 'Matlab-RTS-SingleSmooth.nav');
    fullfile(paths.matlab_results, 'Matlab-RTS-DoubleSmooth.nav');
    fullfile(paths.cpp_results, 'Cpp-Origin.nav');
    fullfile(paths.cpp_results, 'Cpp-RTS-SingleSmooth.nav');
    fullfile(paths.cpp_results, 'Cpp-RTS-DoubleSmooth.nav')};

missing_mask = ~cellfun(@isfile, result_paths);
if any(missing_mask)
    error('缺少对比结果：\n%s', strjoin(result_paths(missing_mask), newline));
end

[fig, table_data] = calc_radial_error_gjb(truth_path, result_paths{:});
title('MATLAB and C++ navigation comparison');
xlabel('Time (s)');
ylabel('Horizontal radial error (m)');

writecell(table_data, fullfile(paths.figures_tables, ...
    'matlab-cpp-error-summary.xlsx'));
exportgraphics(fig, fullfile(paths.figures_tables, ...
    'matlab-cpp-radial-error.png'), 'Resolution', 600);

fprintf('MATLAB/C++ 对比完成：%s\n', paths.figures_tables);
