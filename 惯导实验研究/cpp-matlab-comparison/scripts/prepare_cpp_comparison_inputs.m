%% 生成 MATLAB/C++ 共用的加噪测距与高度量测
clear; clc; close all;
script_dir = fileparts(mfilename('fullpath'));
topic_root = fileparts(script_dir);
addpath(topic_root);
paths = setup_cpp_matlab_comparison();

range_stride = 420;
range_std_m = 6;
height_std_m = 0.4;
end_time = 126856;
random_seed = 1;

pva = importdata(fullfile(paths.external_input, 'pva_830.txt'));
truth = importdata(fullfile(paths.external_input, 'truth.nav'));
start_time = pva(1, 2);

range_streams = {
    importdata(fullfile(paths.external_input, 'range1.txt'));
    importdata(fullfile(paths.external_input, 'range2.txt'));
    importdata(fullfile(paths.external_input, 'range3.txt'))};
for beacon_index = 1:3
    range_streams{beacon_index} = range_streams{beacon_index}( ...
        range_stride:range_stride:end, :);
end

rangedata = zeros(size(range_streams{1}));
for beacon_index = 1:3
    rangedata(beacon_index:3:end, :) = ...
        range_streams{beacon_index}(beacon_index:3:end, :);
end
rangedata = rangedata(rangedata(:, 1) >= start_time & ...
    rangedata(:, 1) <= end_time, :);

heightdata = truth(:, [2, 5]);
heightdata = heightdata(heightdata(:, 1) >= start_time & ...
    heightdata(:, 1) <= end_time, :);

rng(random_seed);
rangedata(:, 3) = rangedata(:, 3) + ...
    normrnd(0, range_std_m, size(rangedata(:, 3)));
heightdata(:, 2) = heightdata(:, 2) + ...
    normrnd(0, height_std_m, size(heightdata(:, 2)));

writematrix(rangedata, fullfile(paths.derived_input, ...
    'rangedata_noised.txt'), 'Delimiter', 'space');
writematrix(heightdata, fullfile(paths.derived_input, ...
    'height_noised.txt'), 'Delimiter', 'space');

fprintf('C++ 对比量测已生成：%s\n', paths.derived_input);
