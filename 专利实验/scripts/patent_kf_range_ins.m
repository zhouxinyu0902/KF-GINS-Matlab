clear;
close all;

%% 路径
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(script_dir);
project_root = fileparts(topic_dir);
addpath(project_root);
addpath(fullfile(project_root, 'function'));
addpath(fullfile(project_root, 'GINS-KF'));
addpath(genpath(fullfile(project_root, 'function_zxy')));
addpath(fullfile(topic_dir, 'config'));

%% 参数和配置
param = Param();
cfg = patent_Configsimu();
rangstd = cfg.measurement.range_noise_std;
depstd = cfg.measurement.depth_noise_std;
if ~exist(cfg.outputfolder, 'dir')
    mkdir(cfg.outputfolder);
end

%% 一次性读取数据
imudata_all = importdata(cfg.imufilepath);
range_all = {
    importdata(cfg.rangefile1path), ...
    importdata(cfg.rangefile2path), ...
    importdata(cfg.rangefile3path)};

imudata_all = imudata_all(imudata_all(:, 1) >= cfg.starttime, :);
imudata_all = imudata_all(imudata_all(:, 1) <= cfg.endtime, :);

%% 构造本次批处理任务
run_specs = buildRunSpecs(cfg.experiment);
fprintf('本次共运行 %d 个方案。\n', numel(run_specs));

for run_index = 1:numel(run_specs)
    spec = run_specs(run_index);
    use_range = ~strcmp(spec.mode, 'pure_ins');
    rng(cfg.randomseed);

    %% 构造当前方案的测距序列
    if use_range
        interval_samples = round(spec.interval_min * 60 / cfg.sim.sample_interval);
        if interval_samples < 1
            error('测距间隔过小：%.9g min。', spec.interval_min);
        end

        sampled_range = cell(1, 3);
        for beacon_id = 1:3
            sampled_range{beacon_id} = range_all{beacon_id}(interval_samples:interval_samples:end, :);
            sampled_range{beacon_id} = sampled_range{beacon_id}( ...
                sampled_range{beacon_id}(:, 1) >= cfg.starttime & ...
                sampled_range{beacon_id}(:, 1) <= cfg.endtime, :);
        end

        if strcmp(spec.mode, 'single')
            rangedata = sampled_range{spec.beacon_id};
        else
            % 三信标按固定序列逐次轮换；默认序列为 1-2-3。
            sequence = spec.sequence;
            rangedata = zeros(size(sampled_range{1}));
            for measurement_index = 1:size(rangedata, 1)
                sequence_index = mod(measurement_index - 1, numel(sequence)) + 1;
                beacon_id = sequence(sequence_index);
                rangedata(measurement_index, :) = ...
                    sampled_range{beacon_id}(measurement_index, :);
            end
        end

        if isempty(rangedata)
            error('方案 %s 在处理时段内没有测距数据。', spec.label);
        end
        rangedata(:, 3) = rangedata(:, 3) + rangstd * randn(size(rangedata, 1), 1);
    else
        rangedata = zeros(0, 6);
    end

    %% 规范化输出文件名
    result_name = buildResultFilename(spec, cfg.experiment.output_prefix);
    navpath = fullfile(cfg.outputfolder, result_name);
    navfp = fopen(navpath, 'wt');
    if navfp < 0
        error('无法创建结果文件：%s', navpath);
    end
    cleanup_file = onCleanup(@() fclose(navfp));

    fprintf('\n[%d/%d] %s\n', run_index, numel(run_specs), spec.label);
    fprintf('输出：%s\n', navpath);

    %% 初始化
    imudata = imudata_all;
    [kf, navstate] = myInitialize_15state(cfg);
    kf.rangstd = cfg.filter.range_std;
    kf.depthstd = cfg.filter.depth_std;
    laststate = navstate;

    lastimu = imudata(1, :)';
    thisimu = imudata(1, :)';
    rangeindex = 1;
    lastprecent = 0;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% MAIN PROCESS PROCEDURE
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for imuindex = 2:size(imudata, 1)-1
        lastimu = thisimu;
        laststate = navstate;
        thisimu = imudata(imuindex, :)';
        imudt = thisimu(1) - lastimu(1);

        if use_range
            while rangeindex <= size(rangedata, 1) && ...
                    rangedata(rangeindex, 1) < lastimu(1)
                rangeindex = rangeindex + 1;
            end
        end
        has_range = use_range && rangeindex <= size(rangedata, 1);

        if has_range && lastimu(1) == rangedata(rangeindex, 1)
            % 量测时刻与上一 IMU 时刻重合。
            thisRange = rangedata(rangeindex, :);
            depthdata = [0, depstd * randn];
            kf = myRangeUpdate(navstate, thisRange, depthdata, kf);
            [kf, navstate] = myErrorFeedback_range(kf, navstate);
            rangeindex = rangeindex + 1;
            laststate = navstate;

            imudt = thisimu(1) - lastimu(1);
            navstate = InsMech(laststate, lastimu, thisimu);
            navstate.pos(3) = 0;
            kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);

        elseif has_range && lastimu(1) < rangedata(rangeindex, 1) && ...
                thisimu(1) > rangedata(rangeindex, 1)
            % 将 IMU 增量拆分到量测时刻。
            [firstimu, secondimu] = interpolate(lastimu, thisimu, rangedata(rangeindex, 1));

            imudt = firstimu(1) - lastimu(1);
            navstate = InsMech(laststate, lastimu, firstimu);
            navstate.pos(3) = 0;
            kf = myInsPropagate_15state(navstate, firstimu, imudt, kf);

            thisRange = rangedata(rangeindex, :);
            depthdata = [0, depstd * randn];
            kf = myRangeUpdate(navstate, thisRange, depthdata, kf);
            [kf, navstate] = myErrorFeedback_range(kf, navstate);
            rangeindex = rangeindex + 1;
            laststate = navstate;
            lastimu = firstimu;

            imudt = secondimu(1) - lastimu(1);
            navstate = InsMech(laststate, lastimu, secondimu);
            navstate.pos(3) = 0;
            kf = myInsPropagate_15state(navstate, secondimu, imudt, kf);

        else
            % 无距离观测时仅进行惯导机械编排和状态传播。
            navstate = InsMech(laststate, lastimu, thisimu);
            navstate.pos(3) = depstd * randn;
            kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
        end

        %% 保存导航结果
        nav = zeros(11, 1);
        nav(2) = navstate.time;
        nav(3:5) = [navstate.pos(1) * param.R2D; ...
                    navstate.pos(2) * param.R2D; navstate.pos(3)];
        nav(6:8) = navstate.vel;
        nav(9:11) = navstate.att * param.R2D;
        fprintf(navfp, ['%2d %12.9f %12.8f %12.8f %8.4f %8.4f ' ...
            '%8.4f %8.4f %8.4f %8.4f %8.4f \n'], nav);

        if imuindex / size(imudata, 1) - lastprecent > 0.2
            fprintf('processing %d %%\n', floor(imuindex * 100 / size(imudata, 1)));
            lastprecent = imuindex / size(imudata, 1);
        end
    end

    clear cleanup_file;
    fprintf('%s finished.\n', spec.label);
end

fprintf('\n全部方案运行完成。结果目录：%s\n', cfg.outputfolder);

%% 局部函数
function specs = buildRunSpecs(experiment)
    template = struct('mode', '', 'beacon_id', 0, 'sequence', [], ...
        'interval_min', NaN, 'label', '');
    interval_count = numel(experiment.range_intervals_min);
    spec_count = double(experiment.run_pure_ins) + interval_count * ( ...
        double(experiment.run_single_beacons) * numel(experiment.single_beacon_ids) + ...
        double(experiment.run_rotating_beacons));
    specs = repmat(template, spec_count, 1);
    spec_index = 0;

    if experiment.run_pure_ins
        spec = template;
        spec.mode = 'pure_ins';
        spec.label = '纯惯导';
        spec_index = spec_index + 1;
        specs(spec_index) = spec;
    end

    for interval_min = experiment.range_intervals_min
        if experiment.run_single_beacons
            for beacon_id = experiment.single_beacon_ids
                spec = template;
                spec.mode = 'single';
                spec.beacon_id = beacon_id;
                spec.interval_min = interval_min;
                spec.label = sprintf('固定单信标 B%02d，测距间隔 %s', ...
                    beacon_id, formatIntervalTag(interval_min));
                spec_index = spec_index + 1;
                specs(spec_index) = spec;
            end
        end

        if experiment.run_rotating_beacons
            spec = template;
            spec.mode = 'rotate';
            spec.sequence = experiment.rotation_sequence;
            spec.interval_min = interval_min;
            spec.label = sprintf('轮换信标 %s，测距间隔 %s', ...
                formatSequenceTag(spec.sequence), formatIntervalTag(interval_min));
            spec_index = spec_index + 1;
            specs(spec_index) = spec;
        end
    end
end

function filename = buildResultFilename(spec, prefix)
    if strcmp(spec.mode, 'pure_ins')
        filename = sprintf('%s-pure-ins.nav', prefix);
        return;
    end

    interval_tag = formatIntervalTag(spec.interval_min);
    if strcmp(spec.mode, 'single')
        filename = sprintf('%s-range-single-b%02d-dt%s.nav', ...
            prefix, spec.beacon_id, interval_tag);
    else
        filename = sprintf('%s-range-rotate-%s-dt%s.nav', ...
            prefix, formatSequenceTag(spec.sequence), interval_tag);
    end
end

function tag = formatSequenceTag(sequence)
    parts = arrayfun(@(id) sprintf('b%02d', id), sequence, 'UniformOutput', false);
    tag = strjoin(parts, '-');
end

function tag = formatIntervalTag(interval_min)
    seconds = interval_min * 60;
    if abs(seconds - round(seconds)) < 1e-9
        tag = sprintf('%ds', round(seconds));
    else
        value = regexprep(sprintf('%.3f', seconds), '0+$', '');
        value = regexprep(value, '\.$', '');
        tag = [strrep(value, '.', 'p'), 's'];
    end
end
