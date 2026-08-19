%% 实测数据的高度更新方式对比
% 0：不做连续高度更新；1：直接赋值；2：卡尔曼量测更新。
clear; clc; close all;
script_dir = fileparts(mfilename('fullpath'));
paper_root = fileparts(fileparts(script_dir));
addpath(paper_root);
paper_paths = setup_paper_study();
param = Param();

input_ids = 5:6;
height_update_modes = 0:2;
method_names = ["No-height update", "Direct assignment", ...
    "Measurement update"];
range_stride = 360;                 % 距离文件为 1 Hz，约 6 min 一次量测
range_std_m = 6;
depth_std_m = 0.4;

for input_id = input_ids
    fprintf('\n[高度更新] 开始处理 Dataset %d。\n', input_id - 4);
    in_dir = fullfile(paper_paths.external_experiment_root, ...
        ['input', num2str(input_id)]);
    cfg_base = config_1(in_dir);
    output_dir = fullfile(paper_paths.output_experiment, ...
        ['dataset', num2str(input_id - 4)], 'heightwayCMP');
    if ~exist(output_dir, 'dir'), mkdir(output_dir); end

    imu_raw = importdata(cfg_base.imufilepath);
    truth_raw = importdata(cfg_base.truthpath);
    range_raw = {importdata(cfg_base.rangefile1path), ...
        importdata(cfg_base.rangefile2path), ...
        importdata(cfg_base.rangefile3path)};
    for beacon_id = 1:3
        range_raw{beacon_id} = range_raw{beacon_id}( ...
            range_stride:range_stride:end, :);
    end
    rangedata_base = zeros(size(range_raw{1}));
    for beacon_id = 1:3
        rangedata_base(beacon_id:3:end, :) = ...
            range_raw{beacon_id}(beacon_id:3:end, :);
    end

    nav_files = strings(numel(height_update_modes), 1);
    for mode_index = 1:numel(height_update_modes)
        height_update_mode = height_update_modes(mode_index);
        method_name = method_names(mode_index);
        fprintf('  --> %s\n', method_name);
        rng(1); % 三种方式使用同一组量测噪声
        tic;

        cfg = cfg_base;
        cfg.userange = 1;
        cfg.outputfolder = output_dir;
        imudata = imu_raw;
        rangedata = rangedata_base;
        height = truth_raw(:, [2, 5]);

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

        navpath = fullfile(output_dir, method_name + ".nav");
        nav_files(mode_index) = string(navpath);
        navfp = fopen(navpath, 'wt');
        if navfp < 0, error('无法创建导航结果：%s', navpath); end
        file_cleanup = onCleanup(@() close_nav_files(navfp));

        [kf, navstate] = myInitialize_15state(cfg);
        kf.rangstd = range_std_m;
        kf.depthstd = depth_std_m;
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
                kf = myRangeUpdate(navstate, rangedata(rangeindex, :), ...
                    height(imuindex, :), kf);
                [kf, navstate] = myErrorFeedback_range(kf, navstate);
                rangeindex = rangeindex + 1;
                navstate = InsMech(navstate, lastimu, thisimu);
                kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);

            elseif lastimu(1) < measurement_time && thisimu(1) > measurement_time
                [firstimu, secondimu] = interpolate( ...
                    lastimu, thisimu, measurement_time);
                first_dt = firstimu(1) - lastimu(1);
                navstate = InsMech(navstate, lastimu, firstimu);
                kf = myInsPropagate_15state(navstate, firstimu, first_dt, kf);
                kf = myRangeUpdate(navstate, rangedata(rangeindex, :), ...
                    height(imuindex, :), kf);
                [kf, navstate] = myErrorFeedback_range(kf, navstate);
                rangeindex = rangeindex + 1;
                second_dt = secondimu(1) - firstimu(1);
                navstate = InsMech(navstate, firstimu, secondimu);
                kf = myInsPropagate_15state(navstate, secondimu, second_dt, kf);

            else
                navstate = InsMech(navstate, lastimu, thisimu);
                switch height_update_mode
                    case 0
                        % 仅依靠惯导高度传播。
                    case 1
                        % 保留旧论文程序的直接赋值方案。
                        navstate.pos(3) = height(imuindex, 2);
                    case 2
                        kf = myHeightUpdate(navstate, height(imuindex, :), kf);
                        navstate.pos(3) = navstate.pos(3) - kf.x(3);
                        navstate.vel(3) = navstate.vel(3) - kf.x(6);
                        kf.x = zeros(size(kf.x));
                end
                kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
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
                fprintf('      processing %d %%\n', ...
                    floor(imuindex * 100 / size(imudata, 1)));
                last_percent = imuindex / size(imudata, 1);
            end
        end
        fclose(navfp);
        clear file_cleanup;
        fprintf('      完成，耗时 %.2f s。\n', toc);
    end

    % 自动输出高度/垂向速度 RMSE 表格和对比图。
    artifact_dir = paper_artifact_dir(output_dir);
    compare_height_vertical_rmse(cfg_base.truthpath, cellstr(nav_files), ...
        'MethodNames', cellstr(method_names), ...
        'OutputFile', fullfile(artifact_dir, 'height-update-comparison.csv'), ...
        'FigureFile', fullfile(artifact_dir, 'height-update-comparison.png'));
end

fprintf('\n两组实测数据的高度更新方式对比完成。\n');
