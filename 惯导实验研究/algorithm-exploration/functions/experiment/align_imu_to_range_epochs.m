function [aligned_imu, aligned_height, info] = ...
        align_imu_to_range_epochs(imudata, height, range_times)
%ALIGN_IMU_TO_RANGE_EPOCHS Insert exact range epochs into an IMU timeline.
%   When a range measurement lies strictly between two IMU epochs, the
%   increment stored in the right-hand IMU row is split in proportion to
%   time. The two increments sum exactly to the original increment, which
%   is equivalent to calling interpolate(lastimu, thisimu, range_time) in
%   the navigation loop. Height is linearly interpolated at the new epoch.

    if size(imudata, 1) < 2 || size(imudata, 2) ~= 7
        error('imudata 必须是 N×7 矩阵：[time, dtheta(3), dvel(3)]。');
    end
    if size(height, 1) ~= size(imudata, 1) || size(height, 2) < 2
        error('height 必须与 imudata 行数一致，且至少包含 [time, depth]。');
    end
    if any(~isfinite(imudata(:))) || any(diff(imudata(:, 1)) <= 0)
        error('IMU 数据必须有限且时间严格递增。');
    end
    if any(~isfinite(height(:))) || any(diff(height(:, 1)) <= 0)
        error('高度数据必须有限且时间严格递增。');
    end

    median_imu_dt = median(diff(imudata(:, 1)));
    time_tolerance = max(1e-9, abs(median_imu_dt) * 1e-6);
    if any(abs(height(:, 1) - imudata(:, 1)) > time_tolerance)
        error('height 的时间列必须与 imudata 的时间列逐行对齐。');
    end

    range_times = sort(unique(double(range_times(:))));
    range_times = range_times(isfinite(range_times));
    aligned_imu = imudata;
    aligned_height = height;
    inserted_times = zeros(numel(range_times), 1);
    inserted_count = 0;

    for event_index = 1:numel(range_times)
        event_time = range_times(event_index);
        if event_time < aligned_imu(1, 1) - time_tolerance || ...
                event_time > aligned_imu(end, 1) + time_tolerance
            continue;
        end

        right_index = find(aligned_imu(:, 1) >= event_time, 1, 'first');
        if isempty(right_index)
            continue;
        end

        candidate_indices = unique([max(1, right_index - 1), right_index]);
        if any(abs(aligned_imu(candidate_indices, 1) - event_time) <= ...
                time_tolerance)
            continue;
        end
        if right_index == 1
            continue;
        end

        left_time = aligned_imu(right_index - 1, 1);
        right_time = aligned_imu(right_index, 1);
        if event_time <= left_time || event_time >= right_time
            error('测距时刻 %.9f s 无法落入有效 IMU 区间。', event_time);
        end

        split_ratio = (event_time - left_time) / (right_time - left_time);
        original_right_imu = aligned_imu(right_index, :);
        first_imu = original_right_imu;
        second_imu = original_right_imu;
        first_imu(1) = event_time;
        first_imu(2:7) = original_right_imu(2:7) * split_ratio;
        second_imu(2:7) = original_right_imu(2:7) * (1 - split_ratio);

        left_height = aligned_height(right_index - 1, :);
        right_height = aligned_height(right_index, :);
        event_height = left_height + split_ratio * (right_height - left_height);
        event_height(1) = event_time;

        aligned_imu = [aligned_imu(1:right_index - 1, :); first_imu; ...
            second_imu; aligned_imu(right_index + 1:end, :)];
        aligned_height = [aligned_height(1:right_index - 1, :); ...
            event_height; aligned_height(right_index:end, :)];

        inserted_count = inserted_count + 1;
        inserted_times(inserted_count) = event_time;
    end

    info = struct( ...
        'inserted_count', inserted_count, ...
        'inserted_times', inserted_times(1:inserted_count), ...
        'time_tolerance_s', time_tolerance);
end
