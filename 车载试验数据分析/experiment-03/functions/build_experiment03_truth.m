function result = build_experiment03_truth(dataset_id, varargin)
%BUILD_EXPERIMENT03_TRUTH 用120 IMU递推和830三维位置更新构造真值。
%
% result = build_experiment03_truth(dataset_id)
% result = build_experiment03_truth(dataset_id, 'DurationSeconds', 30, ...
%     'OutputFile', temporary_file)
%
% 830只提供纬度、经度、高度及其标准差；不使用830速度和姿态。
% 姿态与速度来自120 IMU递推，并通过位置辅助间接约束。

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
    cfg = Config(paths.dataset_name, "rad");
    param = Param();

    imu = readmatrix(paths.imu_120_file, 'FileType', 'text');
    pva830 = readmatrix(paths.pva_830_file, 'FileType', 'text');
    std830 = readmatrix(paths.std_830_file, 'FileType', 'text');
    validate_truth_inputs(imu, pva830, std830, paths.dataset_name);

    duration_seconds = paths.duration_s;
    if ~isempty(options.DurationSeconds)
        duration_seconds = min(duration_seconds, options.DurationSeconds);
    end
    start_time = max(cfg.starttime, imu(1, 1));
    end_time = min([start_time + duration_seconds, imu(end, 1), ...
        pva830(end, 2), std830(end, 1)]);
    imu = imu(imu(:, 1) >= start_time & imu(:, 1) <= end_time, :);
    if size(imu, 1) < 2
        error('%s 在指定时段内没有足够IMU数据。', paths.dataset_name);
    end

    [pva_time, pva_unique] = unique(pva830(:, 2), 'stable');
    pva_position_deg = pva830(pva_unique, 3:5);
    [std_time, std_unique] = unique(std830(:, 1), 'stable');
    position_std_m = std830(std_unique, 2:4);

    position_at_imu_deg = interp1(pva_time, pva_position_deg, ...
        imu(:, 1), 'linear', NaN);
    std_at_imu_m = interp1(std_time, position_std_m, ...
        imu(:, 1), 'linear', NaN);
    valid_position = all(isfinite(position_at_imu_deg), 2) & ...
        all(isfinite(std_at_imu_m), 2);

    cfg.starttime = imu(1, 1);
    cfg.endtime = imu(end, 1);
    [kf, navstate] = myInitialize_15state(cfg);
    navstate.time = imu(1, 1);

    sample_count = size(imu, 1);
    truth = zeros(sample_count, 11);
    gps_week = pva830(1, 1);
    truth(1, :) = state_to_truth_row(navstate, gps_week, param);

    last_progress = 0;
    for imu_index = 2:sample_count
        last_imu = imu(imu_index - 1, :)';
        this_imu = imu(imu_index, :)';
        imu_dt = this_imu(1) - last_imu(1);
        if ~isfinite(imu_dt) || imu_dt <= 0
            error('%s IMU第%d行时间步长无效：%.9f s。', ...
                paths.dataset_name, imu_index, imu_dt);
        end

        navstate = InsMech(navstate, last_imu, this_imu);
        kf = myInsPropagate_15state(navstate, this_imu, imu_dt, kf);

        if valid_position(imu_index)
            measured_position = position_at_imu_deg(imu_index, :)';
            measured_position(1:2) = measured_position(1:2) * param.D2R;
            measured_std = max(std_at_imu_m(imu_index, :)', 1e-3);
            kf = update_position_only(navstate, measured_position, ...
                measured_std, kf);
            [kf, navstate] = myErrorFeedback_noatt(kf, navstate);
        end

        truth(imu_index, :) = ...
            state_to_truth_row(navstate, gps_week, param);

        if options.ShowProgress
            progress = floor(10 * imu_index / sample_count) / 10;
            if progress >= last_progress + 0.1
                fprintf('  %s truth：%d%%\n', paths.dataset_name, ...
                    round(progress * 100));
                last_progress = progress;
            end
        end
    end

    output_file = char(string(options.OutputFile));
    if isempty(output_file)
        output_file = paths.truth_file;
    end
    output_directory = fileparts(output_file);
    if ~isfolder(output_directory)
        mkdir(output_directory);
    end
    writematrix(truth, output_file, 'FileType', 'text', ...
        'Delimiter', ' ');

    result = struct();
    result.dataset_name = paths.dataset_name;
    result.output_file = output_file;
    result.start_time = truth(1, 2);
    result.end_time = truth(end, 2);
    result.duration_s = truth(end, 2) - truth(1, 2);
    result.sample_count = sample_count;
    result.position_update_count = sum(valid_position);
    fprintf(['%s truth.nav完成：%d行，%.3f~%.3f s，' ...
        '830位置更新%d次。\n'], paths.dataset_name, sample_count, ...
        result.start_time, result.end_time, result.position_update_count);
end

function kf = update_position_only(navstate, measured_position, ...
        position_std_m, kf)
%UPDATE_POSITION_ONLY 使用830三维位置和随时间变化的标准差更新。

    position_scale = diag([navstate.Rm + navstate.pos(3), ...
        (navstate.Rn + navstate.pos(3)) * cos(navstate.pos(1)), -1]);
    innovation = navstate.pos - measured_position;
    measurement_std = position_scale \ position_std_m;
    measurement_covariance = diag(measurement_std .^ 2);
    measurement_matrix = zeros(3, kf.RANK);
    measurement_matrix(:, 1:3) = eye(3);

    gain = kf.P * measurement_matrix' / ...
        (measurement_matrix * kf.P * measurement_matrix' + ...
        measurement_covariance);
    kf.x = kf.x + gain * (innovation - measurement_matrix * kf.x);
    identity = eye(kf.RANK);
    correction = identity - gain * measurement_matrix;
    kf.P = correction * kf.P * correction' + ...
        gain * measurement_covariance * gain';
end

function row = state_to_truth_row(navstate, gps_week, param)
%STATE_TO_TRUTH_ROW 输出统一11列导航格式。

    row = [gps_week, navstate.time, ...
        navstate.pos(1) * param.R2D, ...
        navstate.pos(2) * param.R2D, navstate.pos(3), ...
        navstate.vel(:)', (navstate.att(:)' * param.R2D)];
end

function validate_truth_inputs(imu, pva830, std830, dataset_name)
%VALIDATE_TRUTH_INPUTS 检查真值构造的数据契约。

    if isempty(imu) || size(imu, 2) < 7
        error('%s imu_120.txt必须至少包含7列。', dataset_name);
    end
    if isempty(pva830) || size(pva830, 2) < 11
        error('%s pva_830.txt必须至少包含11列。', dataset_name);
    end
    if isempty(std830) || size(std830, 2) < 4
        error('%s std_830.txt必须至少包含4列。', dataset_name);
    end
    if any(diff(imu(:, 1)) <= 0)
        error('%s IMU时间必须严格递增。', dataset_name);
    end
    imu_step_error = abs(diff(imu(:, 1)) - 0.01);
    if any(imu_step_error > 1e-8)
        error(['%s imu_120.txt不是规则100 Hz时间轴，' ...
            '请先运行process_data_1重新导出。'], dataset_name);
    end
    if any(diff(pva830(:, 2)) <= 0)
        error('%s 830 PVA时间必须严格递增。', dataset_name);
    end
    if any(diff(std830(:, 1)) <= 0)
        error('%s 830标准差时间必须严格递增。', dataset_name);
    end
end
