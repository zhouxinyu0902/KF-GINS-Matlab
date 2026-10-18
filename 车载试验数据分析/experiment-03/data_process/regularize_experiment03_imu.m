function [imu_regular, repair_table, is_interpolated] = ...
        regularize_experiment03_imu(imu_raw, nominal_step_s, ...
        maximum_repair_gap_s)
%REGULARIZE_EXPERIMENT03_IMU 将短时掉帧IMU修复为规则100 Hz时间轴。
%
% imu_raw格式：[time, dtheta_x:y:z, dvel_x:y:z]。原始历元严格保留；
% 缺失历元通过相邻两端的等效角速度/比力线性插值，再乘标称周期恢复
% 为角增量和速度增量。超过maximum_repair_gap_s的间断拒绝自动修复。

    if nargin < 2 || isempty(nominal_step_s)
        nominal_step_s = 0.01;
    end
    if nargin < 3 || isempty(maximum_repair_gap_s)
        maximum_repair_gap_s = 0.5;
    end
    if size(imu_raw, 2) ~= 7 || size(imu_raw, 1) < 2
        error('IMU必须是至少2行的N×7矩阵：[time,dtheta(3),dvel(3)]。');
    end
    if any(~isfinite(imu_raw), 'all')
        error('IMU包含非有限数值。');
    end
    raw_time = imu_raw(:, 1);
    raw_step = diff(raw_time);
    if any(raw_step <= 0)
        error('IMU时间必须严格递增。');
    end

    epoch_index = round((raw_time - raw_time(1)) / nominal_step_s);
    snapped_time = raw_time(1) + epoch_index * nominal_step_s;
    alignment_tolerance_s = max(1e-6, nominal_step_s * 0.1);
    alignment_error = abs(raw_time - snapped_time);
    if any(alignment_error > alignment_tolerance_s)
        first_bad = find(alignment_error > alignment_tolerance_s, 1);
        error(['IMU第%d行无法对齐到%.6f s规则时间轴，' ...
            '时间残差为%.6f s。'], first_bad, nominal_step_s, ...
            alignment_error(first_bad));
    end
    epoch_difference = diff(epoch_index);
    if any(epoch_difference <= 0)
        error('IMU映射到规则时间轴后存在重复或倒序历元。');
    end

    gap_rows = find(epoch_difference > 1);
    missing_per_gap = epoch_difference(gap_rows) - 1;
    gap_duration = raw_step(gap_rows);
    oversized = gap_duration > maximum_repair_gap_s + ...
        alignment_tolerance_s;
    if any(oversized)
        first_large = find(oversized, 1);
        raw_row = gap_rows(first_large);
        error(['IMU在%.6f~%.6f s存在%.3f s间断（缺%d帧），' ...
            '超过%.3f s自动修复上限。'], raw_time(raw_row), ...
            raw_time(raw_row + 1), gap_duration(first_large), ...
            missing_per_gap(first_large), maximum_repair_gap_s);
    end

    total_epoch_count = epoch_index(end) + 1;
    imu_regular = nan(total_epoch_count, 7);
    imu_regular(:, 1) = raw_time(1) + ...
        (0:total_epoch_count - 1)' * nominal_step_s;
    regular_rows = epoch_index + 1;
    imu_regular(regular_rows, 2:7) = imu_raw(:, 2:7);
    is_interpolated = true(total_epoch_count, 1);
    is_interpolated(regular_rows) = false;

    gap_count = numel(gap_rows);
    gap_number = (1:gap_count)';
    start_time_s = zeros(gap_count, 1);
    end_time_s = zeros(gap_count, 1);
    gap_duration_s = zeros(gap_count, 1);
    missing_epochs = zeros(gap_count, 1);
    first_inserted_row = zeros(gap_count, 1);
    last_inserted_row = zeros(gap_count, 1);
    method = repmat("linear-rate", gap_count, 1);

    for gap_index = 1:gap_count
        left_raw_row = gap_rows(gap_index);
        right_raw_row = left_raw_row + 1;
        left_regular_row = regular_rows(left_raw_row);
        right_regular_row = regular_rows(right_raw_row);
        inserted_rows = (left_regular_row + 1:right_regular_row - 1)';
        interpolation_fraction = ...
            (1:numel(inserted_rows))' / (numel(inserted_rows) + 1);

        left_rate = imu_raw(left_raw_row, 2:7) / nominal_step_s;
        right_rate = imu_raw(right_raw_row, 2:7) / nominal_step_s;
        interpolated_rate = ...
            (1 - interpolation_fraction) .* left_rate + ...
            interpolation_fraction .* right_rate;
        imu_regular(inserted_rows, 2:7) = ...
            interpolated_rate * nominal_step_s;

        start_time_s(gap_index) = raw_time(left_raw_row);
        end_time_s(gap_index) = raw_time(right_raw_row);
        gap_duration_s(gap_index) = raw_step(left_raw_row);
        missing_epochs(gap_index) = numel(inserted_rows);
        first_inserted_row(gap_index) = inserted_rows(1);
        last_inserted_row(gap_index) = inserted_rows(end);
    end

    if any(~isfinite(imu_regular), 'all')
        error('IMU规则化后仍有未填充或非有限数值。');
    end
    regular_step = diff(imu_regular(:, 1));
    if any(abs(regular_step - nominal_step_s) > 1e-8)
        error('IMU规则化后的时间轴不是严格%.6f s等间隔。', ...
            nominal_step_s);
    end

    repair_table = table(gap_number, start_time_s, end_time_s, ...
        gap_duration_s, missing_epochs, first_inserted_row, ...
        last_inserted_row, method, ...
        'VariableNames', {'Gap', 'StartTime_s', 'EndTime_s', ...
        'GapDuration_s', 'MissingEpochs', 'FirstInsertedRow', ...
        'LastInsertedRow', 'Method'});
end
