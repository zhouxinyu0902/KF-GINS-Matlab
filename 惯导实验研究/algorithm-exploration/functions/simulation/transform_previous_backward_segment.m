function [corrected_position, info] = ...
    transform_previous_backward_segment(previous_backward_position, ...
    backward_target_node)
%TRANSFORM_PREVIOUS_BACKWARD_SEGMENT 延迟修正上一周期的红色反推轨迹。
%
% 在测距时刻 t0 已经反推得到 t0-420 s 节点后：
%   1）取上一周期保存的红色反推轨迹 [t0-840 s, t0-420 s]；
%   2）固定该轨迹在 t0-840 s 的起点；
%   3）以当前反向推算得到的 t0-420 s 终点作为参考点；
%   4）把上一段红色轨迹在 t0-420 s 的终点移动到该参考点；
%   5）对上一段红色轨迹整体执行旋转和尺度调整。
%
% 本方法不修改当前红色区间 [t0-420 s, t0]，与直接对当前反向区间
% 执行双端点几何对齐的方法不同。

    arguments
        previous_backward_position (3, :) double
        backward_target_node (3, 1) double
    end

    if size(previous_backward_position, 2) < 2
        error('待修正的上一测距区间至少需要包含两个轨迹点。');
    end

    original_end = previous_backward_position(:, end);
    [corrected_position, scale_factor, rotation_angle_deg] = ...
        rotateAndScaleTrajectory(previous_backward_position, backward_target_node);

    info = struct( ...
        'scale_factor', scale_factor, ...
        'rotation_angle_deg', rotation_angle_deg, ...
        'endpoint_gap_before_m', calculate_horizontal_gap( ...
            original_end, backward_target_node), ...
        'endpoint_gap_after_m', calculate_horizontal_gap( ...
            corrected_position(:, end), backward_target_node));
end

function gap = calculate_horizontal_gap(position, anchor)
%CALCULATE_HORIZONTAL_GAP 计算两个节点的水平间距。
    param = Param();
    [rm, rn] = getRmRn(anchor(1), param);
    delta_north = (position(1) - anchor(1)) * (rm + anchor(3));
    delta_east = (position(2) - anchor(2)) * ...
        (rn + anchor(3)) * cos(anchor(1));
    gap = hypot(delta_north, delta_east);
end
