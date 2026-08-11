clear;
clc;
close all;

root_dir = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root_dir, 'function')));
addpath(genpath('D:\Github\PSINS\psins2401\base'));
glvs;

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
cfg.outputfolder = fullfile(root_dir, 'output_here', output_case);

if ~exist(cfg.outputfolder, 'dir')
    mkdir(cfg.outputfolder);
end

dvl_all = importdata(cfg.dvlpath);          % t vb_x vb_y vb_z
compass_all = importdata(cfg.compasspath);  % t pitch roll yaw, rad
depth_all = importdata(cfg.depthpath);      % t depth, m, D down positive

range_interval = 2;

fprintf('Start local DR/RANGE processing with four independent beacons.\n');
for beacon_id = 1:4
    beacon_all = importdata(cfg.beaconpath1{beacon_id});

    starttime = max([ ...
        dvl_all(1, 1), compass_all(1, 1), depth_all(1, 1), ...
        beacon_all(1, 1), cfg.starttime]);
    endtime = min([ ...
        dvl_all(end, 1), compass_all(end, 1), depth_all(end, 1), ...
        beacon_all(end, 1), cfg.endtime]);

    dvl = dvl_all(dvl_all(:, 1) >= starttime & dvl_all(:, 1) <= endtime, :);
    compass = compass_all(compass_all(:, 1) >= starttime & compass_all(:, 1) <= endtime, :);
    depth = depth_all(depth_all(:, 1) >= starttime & depth_all(:, 1) <= endtime, :);
    beacon = beacon_all(beacon_all(:, 1) >= starttime & beacon_all(:, 1) <= endtime, :);
    beacon = beacon(range_interval:range_interval:end, :);

    if isempty(dvl) || isempty(compass) || isempty(depth) || isempty(beacon)
        error('Input data is empty for beacon %d. Check path and time range.', beacon_id);
    end

    fprintf('Beacon %d time span: %.3f s -> %.3f s\n', beacon_id, starttime, endtime);

    compass_idx = DRGetClosestIndex(compass, dvl(1, 1), 1);
    depth_idx = DRGetClosestIndex(depth, dvl(1, 1), 1);
    range_idx = 1;

    dr = DRInitialize(cfg, dvl(1, :), compass(compass_idx, :), depth(depth_idx, :));
    dr_origin = dr;
    kf = DREKFInitialize(cfg, dr);

    origin_nav_path = fullfile(cfg.outputfolder, sprintf('Origin-DR-%d.nav', beacon_id));
    fused_nav_path = fullfile(cfg.outputfolder, sprintf('DR-RANGE-%d.nav', beacon_id));
    state_path = fullfile(cfg.outputfolder, sprintf('DR-RANGE-state-%d.txt', beacon_id));
    innov_path = fullfile(cfg.outputfolder, sprintf('DR-RANGE-innovation-%d.txt', beacon_id));

    fp_origin = fopen(origin_nav_path, 'wt');
    fp_fused = fopen(fused_nav_path, 'wt');
    fp_state = fopen(state_path, 'wt');
    fp_innov = fopen(innov_path, 'wt');
    if any([fp_origin, fp_fused, fp_state, fp_innov] < 0)
        error('Cannot open one or more output files for beacon %d.', beacon_id);
    end

    last_percent = 0;
    for k = 2:size(dvl, 1)
        tk = dvl(k, 1);

        compass_idx = DRGetClosestIndex(compass, tk, compass_idx);
        depth_idx = DRGetClosestIndex(depth, tk, depth_idx);

        compass_k = compass(compass_idx, :);
        depth_k = depth(depth_idx, :);
        dvl_k = dvl(k, :);

        dr_origin = DRmechanization(dr_origin, dvl_k, compass_k, depth_k);
        dr = DRmechanization(dr, dvl_k, compass_k, depth_k);
        kf = DRinspropagation(kf, dr);

        while range_idx <= size(beacon, 1) && beacon(range_idx, 1) < tk - cfg.range_time_tolerance
            range_idx = range_idx + 1;
        end

        while range_idx <= size(beacon, 1) && abs(beacon(range_idx, 1) - tk) <= cfg.range_time_tolerance
            beacon_k = beacon(range_idx, :);
            [kf, innov_info] = DRRangeUpdate(kf, dr, beacon_k, cfg);

            if innov_info.used
                [kf, dr] = DRfeedback(kf, dr, cfg);
            end

            fprintf(fp_innov, '%12.6f %12.6f %12.6f %12.6f %12.6f %d\n', ...
                innov_info.time, innov_info.range_meas, innov_info.range_pred, ...
                innov_info.innovation, innov_info.sqrtS, innov_info.used);

            range_idx = range_idx + 1;
        end

        DRWriteNavLine(fp_origin, k, dr_origin, cfg);
        DRWriteNavLine(fp_fused, k, dr, cfg);

        stdx = sqrt(diag(kf.Pxk));
        fprintf(fp_state, '%12.6f ', dr.time);
        fprintf(fp_state, '%15.8e ', kf.xk(:));
        fprintf(fp_state, '%15.8e ', stdx(:));
        fprintf(fp_state, '\n');

        if (k / size(dvl, 1) - last_percent > 0.20)
            fprintf('Beacon %d processing %d %%\n', beacon_id, floor(k * 100 / size(dvl, 1)));
            last_percent = k / size(dvl, 1);
        end
    end

    fclose(fp_origin);
    fclose(fp_fused);
    fclose(fp_state);
    fclose(fp_innov);

    fprintf('Beacon %d local result saved: %s\n', beacon_id, fused_nav_path);
end

fprintf('Local DR/RANGE processing finished. Output folder: %s\n', cfg.outputfolder);





