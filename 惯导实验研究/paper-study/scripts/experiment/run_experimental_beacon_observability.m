%% 实测数据的信标布设可观测度对比
% 对两组数据分别比较：三信标轮换、固定信标 B1/B2/B3。
% 本脚本只生成前向 ES-EKF 结果，不执行 RTS 平滑。
clear; clc; close all;
script_dir = fileparts(mfilename('fullpath'));
paper_root = fileparts(fileparts(script_dir));
addpath(paper_root);
paper_paths = setup_paper_study();
param = Param();

input_ids = 5:6;
beacon_modes = ["Alternating", "Fixed-B1", "Fixed-B2", "Fixed-B3"];
range_stride = 360;                 % 距离文件为 1 Hz，约 6 min 一次量测
range_std_m = 6;
depth_std_m = 0.4;

for input_id = input_ids
    in_dir = fullfile(paper_paths.external_experiment_root, ...
        ['input', num2str(input_id)]);
    cfg_base = config_1(in_dir);
    output_dir = fullfile(paper_paths.output_experiment, ...
        ['dataset', num2str(input_id - 4)], 'alt-B1-B2-B3');
    if ~exist(output_dir, 'dir'), mkdir(output_dir); end

    % 各模式共用相同原始数据。
    imu_raw = importdata(cfg_base.imufilepath);
    truth_raw = importdata(cfg_base.truthpath);
    range_raw = {importdata(cfg_base.rangefile1path), ...
        importdata(cfg_base.rangefile2path), ...
        importdata(cfg_base.rangefile3path)};
    for beacon_id = 1:3
        range_raw{beacon_id} = range_raw{beacon_id}( ...
            range_stride:range_stride:end, :);
    end
    beacon_positions = zeros(3, 3);
    for beacon_id = 1:3
        beacon_positions(beacon_id, :) = range_raw{beacon_id}(1, 4:6);
    end

    for beacon_mode = beacon_modes
        fprintf('\n[可观测度] Dataset %d / %s\n', ...
            input_id - 4, beacon_mode);
        rng(1); % 各模式使用同一组噪声序列，保证对比公平
        tic;

        cfg = cfg_base;
        cfg.userange = 1;
        cfg.outputfolder = output_dir;
        imudata = imu_raw;
        height = truth_raw(:, [2, 5]);

        switch beacon_mode
            case "Alternating"
                rangedata = zeros(size(range_raw{1}));
                for beacon_id = 1:3
                    rangedata(beacon_id:3:end, :) = ...
                        range_raw{beacon_id}(beacon_id:3:end, :);
                end
            case "Fixed-B1"
                rangedata = range_raw{1};
            case "Fixed-B2"
                rangedata = range_raw{2};
            case "Fixed-B3"
                rangedata = range_raw{3};
            otherwise
                error('不支持的信标模式：%s', beacon_mode);
        end

        % 统一处理时间范围并添加量测噪声。
        cfg.starttime = max(cfg.starttime, imudata(1, 1));
        cfg.endtime = min(cfg.endtime, imudata(end, 1));
        imudata = imudata(imudata(:, 1) >= cfg.starttime & ...
            imudata(:, 1) <= cfg.endtime, :);
        rangedata = rangedata(rangedata(:, 1) >= cfg.starttime & ...
            rangedata(:, 1) <= cfg.endtime, :);
        height = height(height(:, 1) >= cfg.starttime & ...
            height(:, 1) <= cfg.endtime, :);
        rangedata(:, 3) = rangedata(:, 3) + ...
            normrnd(0, range_std_m, size(rangedata, 1), 1);
        height(:, 2) = height(:, 2) + ...
            normrnd(0, depth_std_m, size(height, 1), 1);

        navpath = fullfile(output_dir, ...
            sprintf('ES-EKF-%s.nav', beacon_mode));
        navfp = fopen(navpath, 'wt');
        if navfp < 0, error('无法创建导航结果：%s', navpath); end
        file_cleanup = onCleanup(@() close_nav_files(navfp));

        [kf, navstate] = myInitialize_15state(cfg);
        kf.rangstd = range_std_m;
        kf.depthstd = depth_std_m;
        obslog = init_beacon_observability( ...
            beacon_positions, kf.P0, range_std_m);

        lastimu = imudata(1, :)';
        thisimu = lastimu;
        rangeindex = 1;
        while rangeindex <= size(rangedata, 1) && ...
                rangedata(rangeindex, 1) < thisimu(1)
            rangeindex = rangeindex + 1;
        end
        last_percent = 0;

        for imuindex = 2:size(imudata, 1)
            lastimu = thisimu;
            thisimu = imudata(imuindex, :)';
            imudt = thisimu(1) - lastimu(1);

            while rangeindex <= size(rangedata, 1) && ...
                    rangedata(rangeindex, 1) < lastimu(1)
                rangeindex = rangeindex + 1;
            end
            if rangeindex > size(rangedata, 1)
                break;
            end

            measurement_time = rangedata(rangeindex, 1);
            if lastimu(1) == measurement_time
                obslog = log_beacon_observation(obslog, measurement_time, ...
                    navstate.pos, rangedata(rangeindex, 4:6)');
                kf = myRangeUpdate(navstate, rangedata(rangeindex, :), ...
                    height(imuindex, :), kf);
                [kf, navstate] = myErrorFeedback_range(kf, navstate);
                rangeindex = rangeindex + 1;

                navstate = InsMech(navstate, lastimu, thisimu);
                kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
                obslog = propagate_beacon_observability(obslog, kf.phi);

            elseif lastimu(1) < measurement_time && thisimu(1) > measurement_time
                % 将 IMU 增量拆分到精确量测时刻。
                [firstimu, secondimu] = interpolate( ...
                    lastimu, thisimu, measurement_time);
                first_dt = firstimu(1) - lastimu(1);
                navstate = InsMech(navstate, lastimu, firstimu);
                kf = myInsPropagate_15state(navstate, firstimu, first_dt, kf);
                obslog = propagate_beacon_observability(obslog, kf.phi);

                obslog = log_beacon_observation(obslog, measurement_time, ...
                    navstate.pos, rangedata(rangeindex, 4:6)');
                kf = myRangeUpdate(navstate, rangedata(rangeindex, :), ...
                    height(imuindex, :), kf);
                [kf, navstate] = myErrorFeedback_range(kf, navstate);
                rangeindex = rangeindex + 1;

                second_dt = secondimu(1) - firstimu(1);
                navstate = InsMech(navstate, firstimu, secondimu);
                kf = myInsPropagate_15state(navstate, secondimu, second_dt, kf);
                obslog = propagate_beacon_observability(obslog, kf.phi);

            else
                navstate = InsMech(navstate, lastimu, thisimu);
                kf = myHeightUpdate(navstate, height(imuindex, :), kf);
                navstate.pos(3) = navstate.pos(3) - kf.x(3);
                navstate.vel(3) = navstate.vel(3) - kf.x(6);
                kf.x(3) = 0;
                kf.x(6) = 0;
                kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
                obslog = propagate_beacon_observability(obslog, kf.phi);
            end

            nav_write = zeros(11, 1);
            nav_write(2) = navstate.time;
            nav_write(3:5) = [navstate.pos(1:2) * param.R2D; navstate.pos(3)];
            nav_write(6:8) = navstate.vel;
            nav_write(9:11) = navstate.att * param.R2D;
            fprintf(navfp, ...
                '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', ...
                nav_write);

            if imuindex / size(imudata, 1) - last_percent > 0.20
                fprintf('  processing %d %%\n', ...
                    floor(imuindex * 100 / size(imudata, 1)));
                last_percent = imuindex / size(imudata, 1);
            end
        end
        fclose(navfp);
        clear file_cleanup;
        fprintf('  完成，耗时 %.2f s。\n', toc);

        % 三信标轮换模式包含完整几何序列，用它生成窗口可观测度结果。
        if beacon_mode == "Alternating"
            num_obs_events = numel(obslog.events);
            window_measurements = min(9, 3 * floor(num_obs_events / 3));
            artifact_dir = paper_artifact_dir(output_dir);
            observability_results = analyze_beacon_observability( ...
                obslog, window_measurements, artifact_dir);
            save(fullfile(artifact_dir, 'observ.mat'), ...
                'obslog', 'window_measurements', 'observability_results');
        end
    end
end

fprintf('\n两组实测数据的信标可观测度分析完成。\n');
