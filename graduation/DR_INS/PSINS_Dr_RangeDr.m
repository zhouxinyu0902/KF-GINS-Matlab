clear;
clc;
close all;

root_dir = fileparts(mfilename('fullpath'));
addpath(genpath(root_dir));
addpath(genpath('D:\Github\PSINS\psins2401\base'));
addpath('D:\Github\PSINS\psins2401\mytest\00_all_func');
addpath(genpath('D:\Github\KF-GINS-Matlab\function_zxy\referconvert'));
glvs;
rng(1);

dataset_name = getenv('DR_INS_DATASET');
if isempty(dataset_name)
    dataset_name = 'data_line_N_four_quadrants';
end
output_case = getenv('DR_INS_OUTPUT_SUFFIX');
if isempty(output_case)
    output_case = dataset_name;
end
input_dir = fullfile(root_dir, 'input', dataset_name);
cfg = config_DR_RANGE(input_dir);
cfg.outputfolder = fullfile(root_dir, 'output_psins', output_case);
output_dir = cfg.outputfolder;
if ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

pva_ref = importdata(cfg.truthpath);
avp_ref = pvaNED2ENU(pva_ref);
compass = importdata(cfg.compasspath);
depth_data = importdata(cfg.depthpath);
dvl = importdata(cfg.dvlpath);

depth = depth_data(:, 2);
vxy = dvl(:, 2:3);
ts = cfg.range_time_tolerance * 2;
range_interval = 2;
range_tol = cfg.range_time_tolerance;

x0 = zeros(cfg.kf.state_dim, 1);
vk = zeros(cfg.kf.state_dim, 1);
vk(1) = cfg.kf.q_dk;
vk(2) = cfg.kf.q_yaw;

fprintf('Start PSINS DR/Range comparison with four independent beacons.\n');
for beacon_id = 1:4
    range_file = cfg.beaconpath1{beacon_id};
    Range = importdata(range_file);
    Range = Range(range_interval:range_interval:end, :);

    range = Range(:, [1, 3]);      % [time, horizontal_range]
    beacon_pos = Range(1, 4:6);    % [lat, lon, h], rad/rad/m

    ref_pos0 = avp_ref(1, 7:9)';
    eth_ref = earth(ref_pos0, [0; 0; 0]);
    init_dpos_m = [ ...
        (cfg.pos0(2) - ref_pos0(2)) * eth_ref.clRNh; ...
        (cfg.pos0(1) - ref_pos0(1)) * eth_ref.RMh; ...
        cfg.pos0(3) - ref_pos0(3)];
    dr = mydr('init', ref_pos0, init_dpos_m, ts);
    eth0 = earth(dr.pos, [0; 0; 0]);
    dx0 = [ ...
        cfg.kf.init_dk_std; ...
        cfg.kf.init_yaw_std; ...
        cfg.kf.init_pos_std_m / eth0.RMh; ...
        cfg.kf.init_pos_std_m / eth0.clRNh];
    vk(3) = cfg.kf.q_pos_m / eth0.RMh;
    vk(4) = cfg.kf.q_pos_m / eth0.clRNh;
    kf = myekf('init', ts, x0, dx0, vk, cfg.range_std);

    avp_dr = zeros(length(compass), 10);
    avp_range = zeros(size(range, 1), 10);
    xk_range = zeros(size(range, 1), 5);
    innovation = zeros(size(range, 1), 2);

    range_idx = 1;
    last_i = 1;

    for i = 1:length(compass)
        t = compass(i, 1);
        dr = mydr('update', dr, -depth(i), compass(i, 2:4), vxy(i, 1:2));
        avp_dr(i, :) = [dr.avp', t];

        kf = myekf('fk', kf, dr);
        kf = myekf('algo', kf, 'T');

        while range_idx <= size(range, 1) && range(range_idx, 1) < t - range_tol
            range_idx = range_idx + 1;
        end

        if range_idx <= size(range, 1) && abs(range(range_idx, 1) - t) <= range_tol
            dr.beacon = beacon_pos;
            r_meas = range(range_idx, 2);
            vertical_delta = -depth(i) - dr.beacon(3);
            slant_dr = RCompu(dr.pos', dr.beacon);
            kf.r_dr = sqrt(max(slant_dr^2 - vertical_delta^2, 0));
            kf.yk = kf.r_dr - r_meas;
            kf = myekf('hk', kf, dr, 'range');
            kf = myekf('algo', kf, 'M');

            corrected_avp = [dr.avp', t];
            corrected_avp(7) = corrected_avp(7) - kf.xk(3);
            corrected_avp(8) = corrected_avp(8) - kf.xk(4);

            avp_range(range_idx, :) = corrected_avp;
            xk_range(range_idx, :) = [kf.xk', t];
            innovation(range_idx, :) = [t, kf.yk];
            range_idx = range_idx + 1;
        end

        last_i = i;
    end

    avp_dr = avp_dr(1:last_i, :);
    avp_range = avp_range(any(avp_range, 2), :);
    xk_range = xk_range(any(xk_range, 2), :);
    innovation = innovation(any(innovation, 2), :);

    origin_nav_path = fullfile(output_dir, sprintf('PSINS-Origin-DR-%d.nav', beacon_id));
    range_nav_path = fullfile(output_dir, sprintf('PSINS-DR-RANGE-%d.nav', beacon_id));
    state_path = fullfile(output_dir, sprintf('PSINS-DR-RANGE-state-%d.txt', beacon_id));
    innov_path = fullfile(output_dir, sprintf('PSINS-DR-RANGE-innovation-%d.txt', beacon_id));

    write_psins_nav(origin_nav_path, avp_dr);
    write_psins_nav(range_nav_path, avp_range);
    writematrix(xk_range, state_path, 'Delimiter', 'tab');
    writematrix(innovation, innov_path, 'Delimiter', 'tab');

    fprintf('Beacon %d PSINS result saved: %s\n', beacon_id, range_nav_path);
end

fprintf('PSINS DR/Range comparison finished. Output folder: %s\n', output_dir);

function write_psins_nav(filepath, avp)
    fp = fopen(filepath, 'wt');
    if fp < 0
        error('Cannot open output file: %s', filepath);
    end

    R2D = 180 / pi;
    for k = 1:size(avp, 1)
        nav = [ ...
            k; ...
            avp(k, 10); ...
            avp(k, 7) * R2D; ...
            avp(k, 8) * R2D; ...
            avp(k, 9); ...
            avp(k, 4); ...
            avp(k, 5); ...
            avp(k, 6); ...
            avp(k, 1) * R2D; ...
            avp(k, 2) * R2D; ...
            avp(k, 3) * R2D];
        fprintf(fp, '%8d %12.6f %15.10f %15.10f %12.5f %12.6f %12.6f %12.6f %12.6f %12.6f %12.6f\n', nav);
    end
    fclose(fp);
end





