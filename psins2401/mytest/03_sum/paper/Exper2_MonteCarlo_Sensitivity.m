%% Exper2_MonteCarlo_Sensitivity
% Monte Carlo and one-factor-at-a-time sensitivity analysis for the
% exper=0 simulation in Exper1_Dr_Range_new.m.
%
% Outputs:
%   D:\Github\KF-GINS-Matlab\data\psins\Monte Carlo\mat
%   D:\Github\KF-GINS-Matlab\data\psins\Monte Carlo\tables
%   D:\Github\KF-GINS-Matlab\data\psins\Monte Carlo\figures
%   D:\Github\KF-GINS-Matlab\data\psins\Monte Carlo\logs
%
% The baseline Monte Carlo comparison uses common random numbers for the
% DR, 4-state, 5-state and 7-state solutions. The sensitivity analysis
% changes one factor at a time and evaluates the proposed 5-state model.

clear;
clc;
close all;

%% Paths and PSINS initialization
project_root = 'D:\Github\KF-GINS-Matlab\psins2401';
study_root = fullfile(project_root, 'mytest', '03_sum');
input_file = fullfile(study_root, 'paper', 'data_dr_square.mat');
geometry_file = 'D:\Github\KF-GINS-Matlab\data\psins\datasaved_new\data_simu_5state.mat';
output_root = getenv('DR_RANGE_MC_OUTPUT_ROOT');
if isempty(output_root)
    output_root = 'D:\Github\KF-GINS-Matlab\data\psins\Monte Carlo';
end

addpath(genpath(project_root));
glvs;

out_mat = fullfile(output_root, 'mat');
out_tables = fullfile(output_root, 'tables');
out_figures = fullfile(output_root, 'figures');
out_logs = fullfile(output_root, 'logs');
ensure_folder(output_root);
ensure_folder(out_mat);
ensure_folder(out_tables);
ensure_folder(out_figures);
ensure_folder(out_logs);

diary_file = fullfile(out_logs, 'MonteCarlo_Sensitivity.log');
diary(diary_file);
diary_cleanup = onCleanup(@() diary('off')); 

%% Reproducible experiment configuration
cfg = struct;
cfg.master_seed = 20260922;
cfg.summary_seed = 20260923;
cfg.n_mc = 200;
cfg.n_sensitivity = 100;
cfg.bootstrap_repetitions = 2000;
cfg.filter_update = 'EKF';
cfg.model_states = [4, 5, 7];
cfg.beacon_names = ["B9", "B10"];
cfg.save_representative_trajectories = true;
cfg.divergence_threshold_m = 5000;

% Set true for a fast installation/syntax test. This changes only the
% number of repetitions, not the tested parameter levels.
cfg.quick_test = strcmpi(getenv('DR_RANGE_MC_QUICK_TEST'), '1');
if cfg.quick_test
    cfg.n_mc = 2;
    cfg.n_sensitivity = 2;
    cfg.bootstrap_repetitions = 100;
end

% Sensor model used by paper/simu.m and Table 4 of the manuscript.
cfg.dvl_scale_error = 0.004;
cfg.dvl_noise_std_mps = 0.004;
cfg.compass_harmonic_amplitude_deg = -4;
cfg.compass_noise_std_deg = 0.1;
cfg.depth_noise_std_m = 0.4;

% Baseline acoustic and initialization settings.
cfg.range_noise_std_m = 5;
cfg.acoustic_interval_s = 8;
cfg.initial_position_std_m = 5;
cfg.outlier_std_m = 25;

% One-factor-at-a-time levels. Q_scale and R_scale multiply covariance,
% not standard deviation. The other filter settings remain at baseline.
cfg.sensitivity.range_noise_m = [2.5, 5, 10, 20];
cfg.sensitivity.update_interval_s = [4, 8, 16, 32];
cfg.sensitivity.Q_scale = [0.25, 1, 4];
cfg.sensitivity.R_scale = [0.25, 1, 4];
cfg.sensitivity.initial_error_scale = [0.5, 1, 2];
cfg.sensitivity.outlier_rate = [0, 0.01, 0.05, 0.10];

fprintf('============================================================\n');
fprintf('Monte Carlo and sensitivity analysis started: %s\n', datestr(now, 31));
fprintf('Baseline repetitions: %d\n', cfg.n_mc);
fprintf('Sensitivity repetitions per level: %d\n', cfg.n_sensitivity);
fprintf('Filter update label: %s\n', cfg.filter_update);
fprintf('============================================================\n');

%% Load reference trajectory and moving-beacon geometry
assert(isfile(input_file), 'Input file not found: %s', input_file);
assert(isfile(geometry_file), 'Geometry file not found: %s', geometry_file);

S = load(input_file, 'avp_ref', 'trj');
G = load(geometry_file, 'moving_beacons', 'moving_beacons1');

truth = prepare_truth(S.avp_ref, S.trj.ts);

geometry = struct;
geometry.B9.pos = G.moving_beacons.pos;
geometry.B10.pos = G.moving_beacons1.pos;

assert(size(geometry.B9.pos, 2) == 3, 'B9 position array must be N-by-3.');
assert(size(geometry.B10.pos, 2) == 3, 'B10 position array must be N-by-3.');

% Both stored moving-beacon trajectories contain 1101 samples on the
% original 0:8:8800 s acoustic time base.
geometry.B9.time = (0:size(geometry.B9.pos, 1)-1)' * cfg.acoustic_interval_s;
geometry.B10.time = (0:size(geometry.B10.pos, 1)-1)' * cfg.acoustic_interval_s;

write_config_file(cfg, input_file, geometry_file, fullfile(out_logs, 'experiment_configuration.txt'));

%% Baseline Monte Carlo: DR and 4/5/7-state models, B9 and B10
baseline_records = repmat(record_template(), 0, 1);
baseline_representative = struct;
record_index = 0;

baseline_condition = make_baseline_condition(cfg);

for run_id = 1:cfg.n_mc
    seed = cfg.master_seed + run_id;
    sensors = synthesize_sensors(truth, cfg, seed, baseline_condition.initial_error_scale);

    for beacon_id = 1:numel(cfg.beacon_names)
        beacon_name = cfg.beacon_names(beacon_id);
        beacon_geometry = geometry.(char(beacon_name));
        acoustic = synthesize_acoustic(truth, beacon_geometry, cfg, ...
            baseline_condition, seed + 100000);

        save_detail = cfg.save_representative_trajectories && run_id == 1;

        [nav_dr, metrics_dr] = run_dr_only(truth, sensors, cfg, save_detail);
        record_index = record_index + 1;
        baseline_records(record_index) = make_record( ...
            "baseline", beacon_name, "DR", run_id, seed, ...
            baseline_condition, metrics_dr, NaN); %#ok<SAGROW>

        if save_detail
            baseline_representative.(char(beacon_name)).DR = nav_dr;
        end

        for model_state = cfg.model_states
            [nav_filter, metrics_filter, diagnostic] = run_range_filter( ...
                truth, sensors, acoustic, model_state, cfg, ...
                baseline_condition, save_detail);

            gain_vs_dr = 100 * (metrics_dr.rmse_horiz_m - metrics_filter.rmse_horiz_m) ...
                / metrics_dr.rmse_horiz_m;

            record_index = record_index + 1;
            baseline_records(record_index) = make_record( ...
                "baseline", beacon_name, sprintf('%d-State', model_state), ...
                run_id, seed, baseline_condition, metrics_filter, gain_vs_dr); %#ok<SAGROW>

            if save_detail
                field_name = sprintf('State%d', model_state);
                baseline_representative.(char(beacon_name)).(field_name) = nav_filter;
                baseline_representative.(char(beacon_name)).([field_name '_diagnostic']) = diagnostic;
                baseline_representative.(char(beacon_name)).acoustic = acoustic;
            end
        end
    end

    if mod(run_id, max(1, floor(cfg.n_mc / 20))) == 0 || run_id == cfg.n_mc
        fprintf('Baseline Monte Carlo: %d/%d repetitions complete.\n', run_id, cfg.n_mc);
    end
end

baseline_table = struct2table(baseline_records);
writetable(baseline_table, fullfile(out_tables, 'baseline_monte_carlo_raw.csv'));

rng(cfg.summary_seed, 'twister');
baseline_summary = summarize_baseline(baseline_table, cfg.bootstrap_repetitions);
writetable(baseline_summary, fullfile(out_tables, 'baseline_monte_carlo_summary.csv'));

paired_summary = summarize_paired_differences(baseline_table, cfg.bootstrap_repetitions);
writetable(paired_summary, fullfile(out_tables, 'baseline_paired_model_differences.csv'));

save(fullfile(out_mat, 'baseline_monte_carlo_results.mat'), ...
    'cfg', 'baseline_table', 'baseline_summary', 'paired_summary', ...
    'baseline_representative', '-v7.3');

plot_baseline_summary(baseline_summary, out_figures);
if cfg.save_representative_trajectories
    plot_representative_navigation(truth, baseline_representative, out_figures);
end

%% One-factor-at-a-time sensitivity analysis for the proposed 5-state model
sensitivity_records = repmat(record_template(), 0, 1);
sensitivity_representative = struct;
sensitivity_index = 0;

factor_names = ["range_noise_m", "update_interval_s", "Q_scale", ...
    "R_scale", "initial_error_scale", "outlier_rate"];

for factor_id = 1:numel(factor_names)
    factor_name = factor_names(factor_id);
    levels = cfg.sensitivity.(char(factor_name));

    fprintf('Sensitivity factor %s started (%d levels).\n', factor_name, numel(levels));

    for level_id = 1:numel(levels)
        level = levels(level_id);
        condition = make_baseline_condition(cfg);
        condition = apply_sensitivity_level(condition, factor_name, level);

        for run_id = 1:cfg.n_sensitivity
            % The same seed is reused across levels and beacon geometries so
            % differences are paired and not dominated by noise realization.
            seed = cfg.master_seed + run_id;
            sensors = synthesize_sensors(truth, cfg, seed, condition.initial_error_scale);

            for beacon_id = 1:numel(cfg.beacon_names)
                beacon_name = cfg.beacon_names(beacon_id);
                beacon_geometry = geometry.(char(beacon_name));
                acoustic = synthesize_acoustic(truth, beacon_geometry, cfg, ...
                    condition, seed + 100000);

                save_detail = cfg.save_representative_trajectories && run_id == 1;
                [nav_dr, metrics_dr] = run_dr_only(truth, sensors, cfg, false);
                [nav_filter, metrics_filter, diagnostic] = run_range_filter( ...
                    truth, sensors, acoustic, 5, cfg, condition, save_detail);

                gain_vs_dr = 100 * (metrics_dr.rmse_horiz_m - metrics_filter.rmse_horiz_m) ...
                    / metrics_dr.rmse_horiz_m;

                sensitivity_index = sensitivity_index + 1;
                sensitivity_records(sensitivity_index) = make_record( ...
                    factor_name, beacon_name, "5-State", run_id, seed, ...
                    condition, metrics_filter, gain_vs_dr); %#ok<SAGROW>
                sensitivity_records(sensitivity_index).Level = level;

                if save_detail
                    factor_field = matlab.lang.makeValidName(char(factor_name));
                    level_field = matlab.lang.makeValidName(sprintf('Level_%g', level));
                    sensitivity_representative.(factor_field).(level_field).(char(beacon_name)).DR = nav_dr;
                    sensitivity_representative.(factor_field).(level_field).(char(beacon_name)).State5 = nav_filter;
                    sensitivity_representative.(factor_field).(level_field).(char(beacon_name)).diagnostic = diagnostic;
                    sensitivity_representative.(factor_field).(level_field).(char(beacon_name)).acoustic = acoustic;
                end
            end
        end

        fprintf('  %s = %g complete.\n', factor_name, level);
    end

    checkpoint_file = fullfile(out_mat, 'sensitivity_checkpoint.mat');
    save(checkpoint_file, 'cfg', 'sensitivity_records', 'sensitivity_representative', '-v7.3');
end

sensitivity_table = struct2table(sensitivity_records);
writetable(sensitivity_table, fullfile(out_tables, 'sensitivity_raw.csv'));

rng(cfg.summary_seed + 1, 'twister');
sensitivity_summary = summarize_sensitivity(sensitivity_table, cfg.bootstrap_repetitions);
writetable(sensitivity_summary, fullfile(out_tables, 'sensitivity_summary.csv'));

save(fullfile(out_mat, 'sensitivity_results.mat'), ...
    'cfg', 'sensitivity_table', 'sensitivity_summary', ...
    'sensitivity_representative', '-v7.3');

plot_sensitivity_summary(sensitivity_summary, out_figures);

%% Combined archive and completion marker
save(fullfile(out_mat, 'MonteCarlo_Sensitivity_All.mat'), ...
    'cfg', 'baseline_table', 'baseline_summary', 'paired_summary', ...
    'sensitivity_table', 'sensitivity_summary', '-v7.3');

completion_file = fullfile(out_logs, 'COMPLETED.txt');
fid = fopen(completion_file, 'wt');
assert(fid ~= -1, 'Unable to create completion marker: %s', completion_file);
fprintf(fid, 'Completed: %s\n', datestr(now, 31));
fprintf(fid, 'Baseline repetitions: %d\n', cfg.n_mc);
fprintf(fid, 'Sensitivity repetitions per level: %d\n', cfg.n_sensitivity);
fprintf(fid, 'Baseline raw rows: %d\n', height(baseline_table));
fprintf(fid, 'Sensitivity raw rows: %d\n', height(sensitivity_table));
fclose(fid);

fprintf('============================================================\n');
fprintf('All analyses completed: %s\n', datestr(now, 31));
fprintf('Results saved to: %s\n', output_root);
fprintf('============================================================\n');

%% Local functions
function ensure_folder(folder_path)
if ~exist(folder_path, 'dir')
    mkdir(folder_path);
end
end

function truth = prepare_truth(avp_ref, ts)
truth.avp_ref = avp_ref;
truth.t = avp_ref(:, end);
truth.ts = ts;
truth.N = size(avp_ref, 1);
truth.true_body_velocity = zeros(truth.N, 2);

for k = 1:truth.N
    Cnb = a2mat(avp_ref(k, 1:3));
    vb = Cnb' * avp_ref(k, 4:6)';
    truth.true_body_velocity(k, :) = vb(1:2)';
end
end

function condition = make_baseline_condition(cfg)
condition.range_noise_m = cfg.range_noise_std_m;
condition.update_interval_s = cfg.acoustic_interval_s;
condition.Q_scale = 1;
condition.R_scale = 1;
condition.initial_error_scale = 1;
condition.outlier_rate = 0;
end

function condition = apply_sensitivity_level(condition, factor_name, level)
switch char(factor_name)
    case 'range_noise_m'
        condition.range_noise_m = level;
    case 'update_interval_s'
        condition.update_interval_s = level;
    case 'Q_scale'
        condition.Q_scale = level;
    case 'R_scale'
        condition.R_scale = level;
    case 'initial_error_scale'
        condition.initial_error_scale = level;
    case 'outlier_rate'
        condition.outlier_rate = level;
    otherwise
        error('Unknown sensitivity factor: %s', factor_name);
end
end

function sensors = synthesize_sensors(truth, cfg, seed, initial_error_scale)
rng(seed, 'twister');

heading_true = truth.avp_ref(:, 3);
heading_bias = deg2rad(cfg.compass_harmonic_amplitude_deg) .* cos(2 * heading_true);
sensors.heading = heading_true + heading_bias ...
    + deg2rad(cfg.compass_noise_std_deg) .* randn(truth.N, 1);

sensors.vxy = truth.true_body_velocity .* (1 + cfg.dvl_scale_error) ...
    + cfg.dvl_noise_std_mps .* randn(truth.N, 2);
sensors.depth = truth.avp_ref(:, 9) ...
    + cfg.depth_noise_std_m .* randn(truth.N, 1);

% dxyz2pos uses [East, North, Up]. Common random numbers ensure that every
% state model starts from exactly the same realization.
sensors.initial_dpos_enu_m = [ ...
    cfg.initial_position_std_m * initial_error_scale * randn, ...
    cfg.initial_position_std_m * initial_error_scale * randn, ...
    0];
end

function acoustic = synthesize_acoustic(truth, beacon_geometry, cfg, condition, seed)
rng(seed, 'twister');

t = truth.t;
interval = condition.update_interval_s;
ratio = t ./ interval;
is_update = abs(ratio - round(ratio)) < 1e-10;
is_update = is_update & t > 0 ...
    & t >= beacon_geometry.time(1) ...
    & t <= beacon_geometry.time(end);
acoustic.nav_index = find(is_update);
acoustic.time = t(acoustic.nav_index);

beacon_lon = unwrap(beacon_geometry.pos(:, 2));
acoustic.beacon_pos = zeros(numel(acoustic.time), 3);
acoustic.beacon_pos(:, 1) = interp1(beacon_geometry.time, beacon_geometry.pos(:, 1), ...
    acoustic.time, 'linear');
acoustic.beacon_pos(:, 2) = interp1(beacon_geometry.time, beacon_lon, ...
    acoustic.time, 'linear');
acoustic.beacon_pos(:, 3) = interp1(beacon_geometry.time, beacon_geometry.pos(:, 3), ...
    acoustic.time, 'linear');

vehicle_pos = truth.avp_ref(acoustic.nav_index, 7:9);
slant_true = RCompu(vehicle_pos, acoustic.beacon_pos);
vertical_separation = vehicle_pos(:, 3) - acoustic.beacon_pos(:, 3);
acoustic.range_true = sqrt(max(slant_true.^2 - vertical_separation.^2, 0));

standard_range_noise = randn(numel(acoustic.time), 1);
outlier_uniform = rand(numel(acoustic.time), 1);
outlier_noise = cfg.outlier_std_m .* randn(numel(acoustic.time), 1);
acoustic.outlier_mask = outlier_uniform < condition.outlier_rate;
acoustic.range_measured = acoustic.range_true ...
    + condition.range_noise_m .* standard_range_noise ...
    + acoustic.outlier_mask .* outlier_noise;
acoustic.num_updates = numel(acoustic.time);
acoustic.num_outliers = nnz(acoustic.outlier_mask);
end

function [nav, metrics] = run_dr_only(truth, sensors, cfg, save_detail)
dr = mydr('init', truth.avp_ref(1, 7:9)', sensors.initial_dpos_enu_m', truth.ts);
nav_matrix = zeros(truth.N, 10);

for k = 1:truth.N
    dr = mydr('update', dr, sensors.depth(k), sensors.heading(k), sensors.vxy(k, 1:2));
    nav_matrix(k, :) = [dr.att', dr.vn', dr.pos', truth.t(k)];
end

metrics = calculate_metrics(truth.avp_ref, nav_matrix, cfg.divergence_threshold_m);
if save_detail
    nav = pack_navigation(nav_matrix, metrics);
else
    nav = struct;
end
end

function [nav, metrics, diagnostic] = run_range_filter( ...
    truth, sensors, acoustic, model_state, cfg, condition, save_detail)

[x0, dx0, vk] = filter_configuration(model_state);
dx0 = dx0 .* condition.initial_error_scale;
vk = vk .* sqrt(condition.Q_scale);
assumed_range_std = condition.range_noise_m * sqrt(condition.R_scale);

kf = myekf('init', truth.ts, x0, dx0, vk, assumed_range_std);
dr = mydr('init', truth.avp_ref(1, 7:9)', sensors.initial_dpos_enu_m', truth.ts);

nav_matrix = zeros(truth.N, 10);
if save_detail
    state_history = nan(acoustic.num_updates, model_state + 1);
    covariance_diag = nan(acoustic.num_updates, model_state + 1);
    innovation = nan(acoustic.num_updates, 2);
else
    state_history = [];
    covariance_diag = [];
    innovation = [];
end

update_counter = 1;
for k = 1:truth.N
    dr = mydr('update', dr, sensors.depth(k), sensors.heading(k), sensors.vxy(k, 1:2));
    kf = myekf('fk', kf, dr);
    kf = myekf('algo', kf, 'T');

    if update_counter <= acoustic.num_updates ...
            && k == acoustic.nav_index(update_counter)
        dr.beacon = acoustic.beacon_pos(update_counter, :);
        slant_predicted = RCompu(dr.pos', dr.beacon);
        vertical_separation = dr.pos(3) - dr.beacon(3);
        kf.r_dr = sqrt(max(slant_predicted.^2 - vertical_separation.^2, eps));
        kf.yk = kf.r_dr - acoustic.range_measured(update_counter);

        kf = myekf('hk', kf, dr, 'range');
        innovation_before_update = kf.yk - kf.ykk_1;
        kf = myekf('algo', kf, 'M', cfg.filter_update);

        % Closed-loop position-error feedback and mean reset, matching the
        % intended ES-EKF procedure. Parameter states and covariance remain.
        dr.pos(1:2) = dr.pos(1:2) - kf.xk(end-1:end);
        kf.xk(end-1:end) = 0;
        dr.avp = [dr.att; dr.vn; dr.pos];

        if save_detail
            state_history(update_counter, :) = [kf.xk', truth.t(k)];
            covariance_diag(update_counter, :) = [diag(kf.Pxk)', truth.t(k)];
            innovation(update_counter, :) = [innovation_before_update, truth.t(k)];
        end
        update_counter = update_counter + 1;
    end

    nav_matrix(k, :) = [dr.att', dr.vn', dr.pos', truth.t(k)];
end

metrics = calculate_metrics(truth.avp_ref, nav_matrix, cfg.divergence_threshold_m);

diagnostic = struct;
diagnostic.model_state = model_state;
diagnostic.num_updates = acoustic.num_updates;
diagnostic.num_outliers = acoustic.num_outliers;
diagnostic.assumed_range_std_m = assumed_range_std;
if save_detail
    diagnostic.state_history = state_history;
    diagnostic.covariance_diag = covariance_diag;
    diagnostic.innovation = innovation;
end

if save_detail
    nav = pack_navigation(nav_matrix, metrics);
else
    nav = struct;
end
end

function [x0, dx0, vk] = filter_configuration(model_state)
global glv

switch model_state
    case 4
        % [DVL scale, constant heading bias, latitude error, longitude error]
        x0 = zeros(4, 1);
        dx0 = [0.004; deg2rad(5); 5/glv.Re; 5/glv.Re];
        vk = [0; deg2rad(0.1); 0; 0];
    case 5
        % [DVL scale, cos(2psi), sin(2psi), latitude error, longitude error]
        x0 = zeros(5, 1);
        dx0 = [0.004; deg2rad(5); deg2rad(5); 5/glv.Re; 5/glv.Re];
        vk = [0; deg2rad(0.1); deg2rad(0.1); 0; 0];
    case 7
        % 5-state model plus constant DVL surge and sway velocity biases.
        x0 = zeros(7, 1);
        dx0 = [0.004; deg2rad(5); deg2rad(5); 0.0002; 0.002; 5/glv.Re; 5/glv.Re];
        vk = [0; deg2rad(0.1); deg2rad(0.1); 0; 0; 0; 0];
    otherwise
        error('model_state must be 4, 5, or 7.');
end
end

function metrics = calculate_metrics(avp_ref, nav_matrix, divergence_threshold_m)
Re = 6378137.0;
lat_ref = avp_ref(:, 7);
lon_ref = unwrap(avp_ref(:, 8));
lat_est = nav_matrix(:, 7);
lon_est = unwrap(nav_matrix(:, 8));

err_north = (lat_est - lat_ref) .* Re;
err_east = (lon_est - lon_ref) .* Re .* cos(lat_ref);
err_horiz = hypot(err_east, err_north);
err_vertical = nav_matrix(:, 9) - avp_ref(:, 9);
err_3d = hypot(err_horiz, err_vertical);

metrics.rmse_horiz_m = sqrt(mean(err_horiz.^2, 'omitnan'));
metrics.max_horiz_m = max(err_horiz, [], 'omitnan');
metrics.mean_horiz_m = mean(err_horiz, 'omitnan');
metrics.std_horiz_m = std(err_horiz, 'omitnan');
metrics.p95_horiz_m = local_percentile(err_horiz, 95);
metrics.final_horiz_m = err_horiz(find(isfinite(err_horiz), 1, 'last'));
metrics.rmse_3d_m = sqrt(mean(err_3d.^2, 'omitnan'));
metrics.diverged = any(~isfinite(err_horiz)) ...
    || metrics.max_horiz_m > divergence_threshold_m;
metrics.error_horiz_m = err_horiz;
metrics.error_east_m = err_east;
metrics.error_north_m = err_north;
end

function nav = pack_navigation(nav_matrix, metrics)
nav.avp = nav_matrix;
nav.error_horiz_m = single(metrics.error_horiz_m);
nav.error_east_m = single(metrics.error_east_m);
nav.error_north_m = single(metrics.error_north_m);
end

function record = make_record(analysis_name, beacon_name, model_name, ...
    run_id, seed, condition, metrics, gain_vs_dr)
record = record_template();
record.Analysis = string(analysis_name);
record.Beacon = string(beacon_name);
record.Model = string(model_name);
record.Run = run_id;
record.Seed = seed;
record.RangeNoiseStd_m = condition.range_noise_m;
record.UpdateInterval_s = condition.update_interval_s;
record.QScale = condition.Q_scale;
record.RScale = condition.R_scale;
record.InitialErrorScale = condition.initial_error_scale;
record.OutlierRate = condition.outlier_rate;
record.Level = NaN;
record.RMSE_Horiz_m = metrics.rmse_horiz_m;
record.Max_Horiz_m = metrics.max_horiz_m;
record.Mean_Horiz_m = metrics.mean_horiz_m;
record.Std_Horiz_m = metrics.std_horiz_m;
record.P95_Horiz_m = metrics.p95_horiz_m;
record.Final_Horiz_m = metrics.final_horiz_m;
record.RMSE_3D_m = metrics.rmse_3d_m;
record.GainVsDR_percent = gain_vs_dr;
record.Diverged = logical(metrics.diverged);
end

function record = record_template()
record = struct( ...
    'Analysis', "", ...
    'Beacon', "", ...
    'Model', "", ...
    'Run', NaN, ...
    'Seed', NaN, ...
    'RangeNoiseStd_m', NaN, ...
    'UpdateInterval_s', NaN, ...
    'QScale', NaN, ...
    'RScale', NaN, ...
    'InitialErrorScale', NaN, ...
    'OutlierRate', NaN, ...
    'Level', NaN, ...
    'RMSE_Horiz_m', NaN, ...
    'Max_Horiz_m', NaN, ...
    'Mean_Horiz_m', NaN, ...
    'Std_Horiz_m', NaN, ...
    'P95_Horiz_m', NaN, ...
    'Final_Horiz_m', NaN, ...
    'RMSE_3D_m', NaN, ...
    'GainVsDR_percent', NaN, ...
    'Diverged', false);
end

function summary = summarize_baseline(T, bootstrap_repetitions)
beacons = unique(T.Beacon, 'stable');
models = ["DR", "4-State", "5-State", "7-State"];
rows = struct([]);
idx = 0;

for b = 1:numel(beacons)
    for m = 1:numel(models)
        mask = T.Beacon == beacons(b) & T.Model == models(m);
        values = T.RMSE_Horiz_m(mask);
        maxima = T.Max_Horiz_m(mask);
        p95_values = T.P95_Horiz_m(mask);
        gains = T.GainVsDR_percent(mask);

        if isempty(values)
            continue;
        end

        [ci_low, ci_high] = bootstrap_mean_ci(values, bootstrap_repetitions);
        idx = idx + 1;
        rows(idx).Beacon = beacons(b); %#ok<AGROW>
        rows(idx).Model = models(m);
        rows(idx).N = numel(values);
        rows(idx).RMSE_Mean_m = mean(values, 'omitnan');
        rows(idx).RMSE_Std_m = std(values, 'omitnan');
        rows(idx).RMSE_CI95_Low_m = ci_low;
        rows(idx).RMSE_CI95_High_m = ci_high;
        rows(idx).Max_Median_m = median(maxima, 'omitnan');
        rows(idx).Max_IQR_m = local_iqr(maxima);
        rows(idx).P95_Mean_m = mean(p95_values, 'omitnan');
        rows(idx).GainVsDR_Mean_percent = mean(gains, 'omitnan');
        rows(idx).Divergence_Count = nnz(T.Diverged(mask));
    end
end

summary = struct2table(rows);
end

function paired = summarize_paired_differences(T, bootstrap_repetitions)
beacons = unique(T.Beacon, 'stable');
comparisons = {"5-State", "4-State"; "7-State", "5-State"};
rows = struct([]);
idx = 0;

for b = 1:numel(beacons)
    for c = 1:size(comparisons, 1)
        model_a = comparisons{c, 1};
        model_b = comparisons{c, 2};
        Ta = sortrows(T(T.Beacon == beacons(b) & T.Model == model_a, :), 'Run');
        Tb = sortrows(T(T.Beacon == beacons(b) & T.Model == model_b, :), 'Run');
        assert(isequal(Ta.Run, Tb.Run), 'Paired runs do not align for %s and %s.', model_a, model_b);
        delta = Ta.RMSE_Horiz_m - Tb.RMSE_Horiz_m;
        [ci_low, ci_high] = bootstrap_mean_ci(delta, bootstrap_repetitions);

        idx = idx + 1;
        rows(idx).Beacon = beacons(b); %#ok<AGROW>
        rows(idx).Comparison = model_a + " minus " + model_b;
        rows(idx).N = numel(delta);
        rows(idx).MeanDeltaRMSE_m = mean(delta, 'omitnan');
        rows(idx).StdDeltaRMSE_m = std(delta, 'omitnan');
        rows(idx).CI95_Low_m = ci_low;
        rows(idx).CI95_High_m = ci_high;
        rows(idx).FractionImproved = mean(delta < 0, 'omitnan');
    end
end

paired = struct2table(rows);
end

function summary = summarize_sensitivity(T, bootstrap_repetitions)
factors = unique(T.Analysis, 'stable');
beacons = unique(T.Beacon, 'stable');
rows = struct([]);
idx = 0;

for f = 1:numel(factors)
    levels = unique(T.Level(T.Analysis == factors(f)), 'sorted');
    for l = 1:numel(levels)
        for b = 1:numel(beacons)
            mask = T.Analysis == factors(f) & T.Level == levels(l) & T.Beacon == beacons(b);
            values = T.RMSE_Horiz_m(mask);
            maxima = T.Max_Horiz_m(mask);
            gains = T.GainVsDR_percent(mask);
            if isempty(values)
                continue;
            end

            [ci_low, ci_high] = bootstrap_mean_ci(values, bootstrap_repetitions);
            idx = idx + 1;
            rows(idx).Factor = factors(f); %#ok<AGROW>
            rows(idx).Level = levels(l);
            rows(idx).Beacon = beacons(b);
            rows(idx).N = numel(values);
            rows(idx).RMSE_Mean_m = mean(values, 'omitnan');
            rows(idx).RMSE_Std_m = std(values, 'omitnan');
            rows(idx).RMSE_CI95_Low_m = ci_low;
            rows(idx).RMSE_CI95_High_m = ci_high;
            rows(idx).Max_Median_m = median(maxima, 'omitnan');
            rows(idx).Max_IQR_m = local_iqr(maxima);
            rows(idx).GainVsDR_Mean_percent = mean(gains, 'omitnan');
            rows(idx).Divergence_Count = nnz(T.Diverged(mask));
        end
    end
end

summary = struct2table(rows);
end

function [ci_low, ci_high] = bootstrap_mean_ci(values, repetitions)
values = values(isfinite(values));
if isempty(values)
    ci_low = NaN;
    ci_high = NaN;
    return;
end
if numel(values) == 1
    ci_low = values;
    ci_high = values;
    return;
end

n = numel(values);
boot_mean = zeros(repetitions, 1);
for k = 1:repetitions
    sample_index = randi(n, n, 1);
    boot_mean(k) = mean(values(sample_index));
end
ci_low = local_percentile(boot_mean, 2.5);
ci_high = local_percentile(boot_mean, 97.5);
end

function value = local_percentile(data, percentile)
data = sort(data(isfinite(data)));
if isempty(data)
    value = NaN;
    return;
end
if numel(data) == 1
    value = data;
    return;
end
position = 1 + (numel(data) - 1) * percentile / 100;
lower_index = floor(position);
upper_index = ceil(position);
if lower_index == upper_index
    value = data(lower_index);
else
    fraction = position - lower_index;
    value = data(lower_index) * (1 - fraction) + data(upper_index) * fraction;
end
end

function value = local_iqr(data)
value = local_percentile(data, 75) - local_percentile(data, 25);
end

function plot_baseline_summary(summary, output_folder)
beacons = unique(summary.Beacon, 'stable');
models = ["DR", "4-State", "5-State", "7-State"];

fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 1100, 450]);
tiledlayout(1, numel(beacons), 'TileSpacing', 'compact', 'Padding', 'compact');

for b = 1:numel(beacons)
    nexttile;
    hold on;
    grid on;
    box on;
    y = nan(size(models));
    lower = nan(size(models));
    upper = nan(size(models));
    for m = 1:numel(models)
        row = summary(summary.Beacon == beacons(b) & summary.Model == models(m), :);
        if ~isempty(row)
            y(m) = row.RMSE_Mean_m;
            lower(m) = y(m) - row.RMSE_CI95_Low_m;
            upper(m) = row.RMSE_CI95_High_m - y(m);
        end
    end
    errorbar(1:numel(models), y, lower, upper, 'o-', 'LineWidth', 1.5, ...
        'MarkerFaceColor', [0.051, 0.251, 0.502]);
    xticks(1:numel(models));
    xticklabels(models);
    ylabel('Horizontal RMSE (m)');
    title(sprintf('%s moving beacon', beacons(b)));
end

export_figure(fig, fullfile(output_folder, 'baseline_rmse_95CI'));
close(fig);
end

function plot_sensitivity_summary(summary, output_folder)
factors = unique(summary.Factor, 'stable');
beacons = unique(summary.Beacon, 'stable');
colors = [0.051, 0.251, 0.502; 0.651, 0.102, 0.153];

for f = 1:numel(factors)
    factor = factors(f);
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 650, 480]);
    hold on;
    grid on;
    box on;

    for b = 1:numel(beacons)
        rows = sortrows(summary(summary.Factor == factor & summary.Beacon == beacons(b), :), 'Level');
        y = rows.RMSE_Mean_m;
        lower = y - rows.RMSE_CI95_Low_m;
        upper = rows.RMSE_CI95_High_m - y;
        x = rows.Level;
        if factor == "outlier_rate"
            x = x * 100;
        end
        errorbar(x, y, lower, upper, 'o-', 'LineWidth', 1.5, ...
            'MarkerFaceColor', colors(b, :), 'Color', colors(b, :), ...
            'DisplayName', beacons(b));
    end

    xlabel(factor_axis_label(factor));
    ylabel('Horizontal RMSE (m)');
    title(sprintf('5-State sensitivity: %s', strrep(factor, '_', ' ')));
    legend('Location', 'best');
    if factor == "Q_scale" || factor == "R_scale"
        set(gca, 'XScale', 'log');
    end

    file_name = ['sensitivity_', char(factor)];
    export_figure(fig, fullfile(output_folder, file_name));
    close(fig);
end
end

function label = factor_axis_label(factor)
switch char(factor)
    case 'range_noise_m'
        label = 'Range-noise standard deviation (m)';
    case 'update_interval_s'
        label = 'Acoustic update interval (s)';
    case 'Q_scale'
        label = 'Process-covariance scale factor';
    case 'R_scale'
        label = 'Measurement-covariance scale factor';
    case 'initial_error_scale'
        label = 'Initial-error scale factor';
    case 'outlier_rate'
        label = 'Injected range-outlier rate (%)';
    otherwise
        label = strrep(factor, '_', ' ');
end
end

function plot_representative_navigation(truth, representative, output_folder)
beacon_names = fieldnames(representative);
model_fields = {'DR', 'State4', 'State5', 'State7'};
model_labels = {'DR', '4-State', '5-State', '7-State'};
colors = [0.2, 0.2, 0.2; 0.051, 0.251, 0.502; ...
    0.651, 0.102, 0.153; 0.000, 0.600, 0.498];

[ref_e, ref_n] = local_en(truth.avp_ref(:, 7:8), truth.avp_ref(1, 7:8));

for b = 1:numel(beacon_names)
    beacon = beacon_names{b};
    fig = figure('Visible', 'off', 'Color', 'w', 'Position', [100, 100, 1050, 450]);
    tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    nexttile;
    plot(ref_e, ref_n, 'k-', 'LineWidth', 1.8, 'DisplayName', 'Reference');
    hold on;
    grid on;
    box on;
    axis equal;
    for m = 1:numel(model_fields)
        nav = representative.(beacon).(model_fields{m}).avp;
        [east, north] = local_en(nav(:, 7:8), truth.avp_ref(1, 7:8));
        plot(east, north, 'Color', colors(m, :), 'LineWidth', 1.0, ...
            'DisplayName', model_labels{m});
    end
    xlabel('East (m)');
    ylabel('North (m)');
    title(sprintf('%s representative trajectory', beacon));
    legend('Location', 'best');

    nexttile;
    hold on;
    grid on;
    box on;
    for m = 1:numel(model_fields)
        nav = representative.(beacon).(model_fields{m});
        plot(truth.t, nav.error_horiz_m, 'Color', colors(m, :), ...
            'LineWidth', 1.0, 'DisplayName', model_labels{m});
    end
    xlabel('Time (s)');
    ylabel('Horizontal error (m)');
    title(sprintf('%s representative error', beacon));
    legend('Location', 'best');

    export_figure(fig, fullfile(output_folder, ['representative_', beacon]));
    close(fig);
end
end

function [east, north] = local_en(lat_lon, origin_lat_lon)
Re = 6378137.0;
north = (lat_lon(:, 1) - origin_lat_lon(1)) .* Re;
east = (unwrap(lat_lon(:, 2)) - origin_lat_lon(2)) .* Re .* cos(origin_lat_lon(1));
end

function export_figure(fig, file_without_extension)
exportgraphics(fig, [file_without_extension, '.png'], 'Resolution', 300);
exportgraphics(fig, [file_without_extension, '.pdf'], 'ContentType', 'vector');
end

function write_config_file(cfg, input_file, geometry_file, output_file)
fid = fopen(output_file, 'wt');
assert(fid ~= -1, 'Unable to create configuration file: %s', output_file);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>

fprintf(fid, 'Generated: %s\n', datestr(now, 31));
fprintf(fid, 'Input trajectory: %s\n', input_file);
fprintf(fid, 'Moving-beacon geometry: %s\n', geometry_file);
fprintf(fid, 'Master seed: %d\n', cfg.master_seed);
fprintf(fid, 'Baseline repetitions: %d\n', cfg.n_mc);
fprintf(fid, 'Sensitivity repetitions per level: %d\n', cfg.n_sensitivity);
fprintf(fid, 'Bootstrap repetitions: %d\n', cfg.bootstrap_repetitions);
fprintf(fid, 'Filter update label: %s\n', cfg.filter_update);
fprintf(fid, 'Baseline range noise std: %.6g m\n', cfg.range_noise_std_m);
fprintf(fid, 'Baseline acoustic interval: %.6g s\n', cfg.acoustic_interval_s);
fprintf(fid, 'Baseline initial horizontal-position std: %.6g m\n', cfg.initial_position_std_m);
fprintf(fid, 'Outlier component std: %.6g m\n', cfg.outlier_std_m);
fprintf(fid, 'DVL scale error: %.6g\n', cfg.dvl_scale_error);
fprintf(fid, 'DVL noise std: %.6g m/s\n', cfg.dvl_noise_std_mps);
fprintf(fid, 'Compass harmonic amplitude: %.6g deg\n', cfg.compass_harmonic_amplitude_deg);
fprintf(fid, 'Compass noise std: %.6g deg\n', cfg.compass_noise_std_deg);
fprintf(fid, 'Depth noise std: %.6g m\n', cfg.depth_noise_std_m);
fprintf(fid, '\nSensitivity levels\n');
factor_names = fieldnames(cfg.sensitivity);
for k = 1:numel(factor_names)
    values = cfg.sensitivity.(factor_names{k});
    fprintf(fid, '%s: %s\n', factor_names{k}, mat2str(values));
end
end
