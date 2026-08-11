clear;
clc;
close all;

root_dir = fileparts(mfilename('fullpath'));
addpath(genpath(root_dir));

dataset_name = getenv('DR_INS_DATASET');
if isempty(dataset_name)
    dataset_name = 'data_line_N_four_quadrants';
end
output_case = getenv('DR_INS_OUTPUT_SUFFIX');
if isempty(output_case)
    output_case = dataset_name;
end
input_dir = fullfile(root_dir, 'input', dataset_name);
truthpath = fullfile(input_dir, 'reference.txt');
output_here = fullfile(root_dir, 'output_here', output_case);
output_psins = fullfile(root_dir, 'output_psins', output_case);
output_compare = fullfile(root_dir, 'output_compare', output_case);
file_prefix = ['four_beacon_here_vs_psins_', output_case];
if ~exist(output_compare, 'dir')
    mkdir(output_compare);
end

series = {
    'Here-Origin', fullfile(output_here, 'Origin-DR-1.nav');
    'Here-B1',     fullfile(output_here, 'DR-RANGE-1.nav');
    'Here-B2',     fullfile(output_here, 'DR-RANGE-2.nav');
    'Here-B3',     fullfile(output_here, 'DR-RANGE-3.nav');
    'Here-B4',     fullfile(output_here, 'DR-RANGE-4.nav');
    'PSINS-Origin', fullfile(output_psins, 'PSINS-Origin-DR-1.nav');
    'PSINS-B1',     fullfile(output_psins, 'PSINS-DR-RANGE-1.nav');
    'PSINS-B2',     fullfile(output_psins, 'PSINS-DR-RANGE-2.nav');
    'PSINS-B3',     fullfile(output_psins, 'PSINS-DR-RANGE-3.nav');
    'PSINS-B4',     fullfile(output_psins, 'PSINS-DR-RANGE-4.nav');
};

truth = importdata(truthpath);
truth = truth(:, 2:end); % [time lat lon h ...], degrees/meters

fig = figure('Color', 'w', 'Name', 'DR Range Comparison');
hold on;
grid on;
box on;
base_colors = lines(5);
stats = cell(size(series, 1) + 1, 6);
stats(1, :) = {'label', 'max_m', 'rms_m', 'mean_m', 'median_m', 'p95_m'};

for i = 1:size(series, 1)
    label = series{i, 1};
    navpath = series{i, 2};
    if exist(navpath, 'file') ~= 2
        warning('Missing result file: %s', navpath);
        continue;
    end

    nav = importdata(navpath);
    nav = nav(:, 2:end); % [time lat lon h VE VN VU pitch roll yaw]
    [time_rel, radial] = radial_error_from_truth(nav, truth);

    is_psins = startsWith(label, 'PSINS');
    if contains(label, 'Origin')
        color_idx = 1;
    else
        color_idx = str2double(extractAfter(label, 'B')) + 1;
    end
    if is_psins
        line_style = '--';
        line_width = 1.5;
    else
        line_style = '-';
        line_width = 1.2;
    end

    plot(time_rel, radial, ...
        'LineStyle', line_style, ...
        'LineWidth', line_width, ...
        'Color', base_colors(color_idx, :), ...
        'DisplayName', label);

    stats{i + 1, 1} = label;
    stats{i + 1, 2} = max(radial);
    stats{i + 1, 3} = sqrt(mean(radial .^ 2));
    stats{i + 1, 4} = mean(radial);
    stats{i + 1, 5} = median(radial);
    stats{i + 1, 6} = quantile(radial, 0.95);
end

xlabel('Time (s)');
ylabel('Horizontal radial error (m)');
legend('show', 'Location', 'best', 'NumColumns', 2);
title(sprintf('%s: Here vs PSINS', strrep(dataset_name, '_', '\_')));

png_path = fullfile(output_compare, [file_prefix, '_radial_error.png']);
xlsx_path = fullfile(output_compare, [file_prefix, '_stats.xlsx']);
csv_path = fullfile(output_compare, [file_prefix, '_stats.csv']);
exportgraphics(fig, png_path, 'Resolution', 600);
writecell(stats, xlsx_path, 'Sheet', 1);
writecell(stats, csv_path);

fprintf('Comparison figure saved: %s\n', png_path);
fprintf('Comparison stats saved : %s\n', xlsx_path);

function [time_rel, radial] = radial_error_from_truth(nav, truth)
    nav_start = nav(1, 1);
    nav_end = nav(end, 1);
    truth_start = truth(1, 1);
    truth_end = truth(end, 1);
    starttime = max(nav_start, truth_start);
    endtime = min(nav_end, truth_end);
    if starttime >= endtime
        error('No overlapping time range between nav and truth.');
    end

    dt = median(diff(nav(:, 1)));
    t = (starttime:dt:endtime)';
    nav_i = interp1(nav(:, 1), nav(:, 2:4), t, 'linear', 'extrap');
    truth_i = interp1(truth(:, 1), truth(:, 2:4), t, 'linear', 'extrap');

    D2R = pi / 180;
    lat_truth = truth_i(:, 1) * D2R;
    h_truth = truth_i(:, 3);
    dlat = (nav_i(:, 1) - truth_i(:, 1)) * D2R;
    dlon = (nav_i(:, 2) - truth_i(:, 2)) * D2R;

    [rm, rn] = wgs84_radii(lat_truth);
    dn = dlat .* (rm + h_truth);
    de = dlon .* (rn + h_truth) .* cos(lat_truth);
    radial = hypot(dn, de);
    time_rel = t - t(1);
end

function [rm, rn] = wgs84_radii(lat)
    a = 6378137.0;
    f = 1 / 298.257223563;
    e2 = f * (2 - f);
    s = sin(lat);
    den = sqrt(1 - e2 .* s .* s);
    rn = a ./ den;
    rm = a * (1 - e2) ./ (den .^ 3);
end






