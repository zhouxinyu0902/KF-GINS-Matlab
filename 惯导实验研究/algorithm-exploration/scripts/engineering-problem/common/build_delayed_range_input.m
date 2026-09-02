function [delayed_range, comparison] = build_delayed_range_input( ...
        current_range, raw_range, delay_s)
%BUILD_DELAYED_RANGE_INPUT 保持到达时刻不变，用 t-delay_s 距离替换量测。
    event_count = size(current_range, 1);
    delayed_range = current_range;
    beacon_id = zeros(event_count, 1);
    source_time = current_range(:, 1)-delay_s;
    current_true = current_range(:, 2);
    delayed_true = nan(event_count, 1);
    shared_noise = current_range(:, 3)-current_range(:, 2);
    beacon_position = zeros(3, 3);
    for index = 1:3
        beacon_position(index, :) = raw_range{index}(1, 4:6);
    end
    for event_index = 1:event_count
        [position_difference, this_beacon] = min(vecnorm( ...
            beacon_position-current_range(event_index, 4:6), 2, 2));
        if position_difference > 1e-10
            error('第%d个测距事件无法匹配到信标。', event_index);
        end
        source = raw_range{this_beacon};
        beacon_id(event_index) = this_beacon;
        delayed_true(event_index) = interp1(source(:, 1), source(:, 2), ...
            source_time(event_index), 'linear', nan);
        if ~isfinite(delayed_true(event_index))
            error('第%d个事件的延迟距离超出原始数据范围。', event_index);
        end
    end
    delayed_range(:, 2) = delayed_true;
    delayed_range(:, 3) = delayed_true+shared_noise;
    comparison = table((1:event_count)', current_range(:, 1), ...
        source_time, beacon_id, current_true, delayed_true, shared_noise, ...
        current_range(:, 3), delayed_range(:, 3), delayed_true-current_true, ...
        'VariableNames', {'EventIndex', 'ArrivalTime_s', ...
        'AcquisitionTime_s', 'BeaconID', 'CurrentTrueRange_m', ...
        'DelayedTrueRange_m', 'SharedNoise_m', 'CurrentInputRange_m', ...
        'DelayedInputRange_m', 'StaleMinusCurrent_m'});
end
