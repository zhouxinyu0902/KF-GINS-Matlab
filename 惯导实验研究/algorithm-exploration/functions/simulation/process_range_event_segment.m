function [backward_position, info] = ...
    process_range_event_segment(imu_segment, height_segment, end_state, start_anchor)
%PROCESS_RANGE_EVENT_SEGMENT 对当前测距周期执行纯反向惯导推算。
%
% 在当前测距时刻 t0 调用本函数，从当前 AEKF 测距锚点反向推算到
% t0-420 s。反向过程只执行惯导机械编排，不传播卡尔曼滤波协方差，
% 也不在 t0-420 s 再次进行测距滤波。

    arguments
        imu_segment (:, 7) double
        height_segment (:, 2) double
        end_state struct
        start_anchor (3, 1) double
    end

    sample_count = size(imu_segment, 1);
    if sample_count < 2
        error('相邻测距点限定的轨迹区间至少需要包含两个 IMU 历元。');
    end
    if size(height_segment, 1) ~= sample_count
        error('IMU 区间与高度区间包含的历元数必须一致。');
    end

    backward_position = nan(3, sample_count);
    backward_state = end_state;
    backward_position(:, end) = end_state.pos;

    for sample_index = sample_count - 1:-1:1
        later_imu = imu_segment(sample_index + 1, :)';
        earlier_imu = imu_segment(sample_index, :)';

        % 只进行反向惯导机械编排。高度观测仅用于保持垂向约束，
        % 不参与水平位置滤波。
        backward_state = InsMechBackward(backward_state, later_imu, earlier_imu);
        backward_state.pos(3) = height_segment(sample_index, 2);
        backward_position(:, sample_index) = backward_state.pos;
    end

    info = struct( ...
        'segment_index', 0, ...
        'start_range_index', 0, ...
        'end_range_index', 0, ...
        'start_time', imu_segment(1, 1), ...
        'end_time', imu_segment(end, 1), ...
        'sample_count', sample_count, ...
        'backward_start_gap_m', calculate_horizontal_gap( ...
            backward_position(:, 1), start_anchor), ...
        'delayed_scale', nan, ...
        'delayed_rotation_angle_deg', nan, ...
        'delayed_endpoint_gap_before_m', nan, ...
        'delayed_endpoint_gap_after_m', nan);
end

function gap = calculate_horizontal_gap(position, anchor)
%CALCULATE_HORIZONTAL_GAP 计算反推节点与上一前向 AEKF 锚点的水平间距。
    param = Param();
    [rm, rn] = getRmRn(anchor(1), param);
    delta_north = (position(1) - anchor(1)) * (rm + anchor(3));
    delta_east = (position(2) - anchor(2)) * ...
        (rn + anchor(3)) * cos(anchor(1));
    gap = hypot(delta_north, delta_east);
end
