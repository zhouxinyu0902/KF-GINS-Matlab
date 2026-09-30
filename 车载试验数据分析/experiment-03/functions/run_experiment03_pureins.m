function result = run_experiment03_pureins(dataset_id, position_unit, varargin)
%RUN_EXPERIMENT03_PUREINS 运行第三次车载试验纯惯导解算。

    if nargin < 2 || isempty(position_unit)
        position_unit = "rad";
    end
    position_unit = lower(string(position_unit));
    if ~ismember(position_unit, ["rad", "m"])
        error('position_unit只能取rad或m。');
    end

    parser = inputParser;
    parser.addParameter('DurationSeconds', [], ...
        @(value) isempty(value) || (isscalar(value) && value > 0));
    parser.addParameter('OutputFile', '', ...
        @(value) ischar(value) || isstring(value));
    parser.addParameter('ShowProgress', true, ...
        @(value) islogical(value) && isscalar(value));
    parser.parse(varargin{:});
    options = parser.Results;

    paths = setup_all_real_data_preprocessing(dataset_id, 'navigation');
    cfg = Config(paths.dataset_name, position_unit);
    param = Param();
    imu = readmatrix(paths.imu_120_file, 'FileType', 'text');
    if isempty(imu) || size(imu, 2) < 7 || any(diff(imu(:, 1)) <= 0)
        error('%s的imu_120.txt格式或时间轴无效。', paths.dataset_name);
    end

    duration_seconds = paths.duration_s;
    if ~isempty(options.DurationSeconds)
        duration_seconds = min(duration_seconds, options.DurationSeconds);
    end
    start_time = max(cfg.starttime, imu(1, 1));
    end_time = min(start_time + duration_seconds, imu(end, 1));
    imu = imu(imu(:, 1) >= start_time & imu(:, 1) <= end_time, :);
    if size(imu, 1) < 2
        error('%s在指定时段内没有足够IMU数据。', paths.dataset_name);
    end

    cfg.starttime = imu(1, 1);
    cfg.endtime = imu(end, 1);
    [~, navstate] = myInitialize_15state(cfg);
    navstate.time = imu(1, 1);

    if isfile(paths.pva_830_file)
        pva_first = readmatrix(paths.pva_830_file, ...
            'FileType', 'text', 'Range', '1:1');
        gps_week = pva_first(1, 1);
    else
        gps_week = 0;
    end

    sample_count = size(imu, 1);
    navigation = zeros(sample_count, 11);
    navigation(1, :) = state_to_nav_row(navstate, gps_week, param);
    last_progress = 0;
    for imu_index = 2:sample_count
        last_imu = imu(imu_index - 1, :)';
        this_imu = imu(imu_index, :)';
        imu_dt = this_imu(1) - last_imu(1);
        if ~isfinite(imu_dt) || imu_dt <= 0
            error('%s IMU第%d行时间步长无效。', ...
                paths.dataset_name, imu_index);
        end
        navstate = InsMech(navstate, last_imu, this_imu);
        navigation(imu_index, :) = ...
            state_to_nav_row(navstate, gps_week, param);

        if options.ShowProgress
            progress = floor(10 * imu_index / sample_count) / 10;
            if progress >= last_progress + 0.1
                fprintf('  %s PureINS-%s：%d%%\n', paths.dataset_name, ...
                    position_unit, round(progress * 100));
                last_progress = progress;
            end
        end
    end

    output_file = char(string(options.OutputFile));
    if isempty(output_file)
        output_directory = fullfile(paths.output, char(position_unit));
        if ~isfolder(output_directory), mkdir(output_directory); end
        output_file = fullfile(output_directory, ...
            sprintf('PureIns-%s.nav', position_unit));
    end
    writematrix(navigation, output_file, 'FileType', 'text', ...
        'Delimiter', ' ');

    result = struct('dataset_name', paths.dataset_name, ...
        'position_unit', char(position_unit), ...
        'output_file', output_file, ...
        'sample_count', sample_count, ...
        'start_time', navigation(1, 2), ...
        'end_time', navigation(end, 2), ...
        'duration_s', navigation(end, 2) - navigation(1, 2));
    fprintf('%s PureIns-%s完成：%d行，时长%.3f s。\n', ...
        paths.dataset_name, position_unit, sample_count, result.duration_s);
end

function row = state_to_nav_row(navstate, gps_week, param)
    row = [gps_week, navstate.time, ...
        navstate.pos(1) * param.R2D, ...
        navstate.pos(2) * param.R2D, navstate.pos(3), ...
        navstate.vel(:)', (navstate.att(:)' * param.R2D)];
end
