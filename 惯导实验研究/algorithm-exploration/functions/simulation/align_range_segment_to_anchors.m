function [aligned_position, info] = ...
    align_range_segment_to_anchors(position, start_anchor, end_anchor)
%ALIGN_RANGE_SEGMENT_TO_ANCHORS 将一段轨迹对齐到前后两个测距锚点。
%
% rotateAndScaleTrajectory 的特性是固定轨迹起点，并把轨迹终点移动到目标点。
% 因此这里使用两次变换实现双端点约束：
%   第一次（轨迹反序）：固定后端点，将前端点移动到上一测距锚点；
%   第二次（恢复正序）：固定前端点，将后端点移动到当前测距锚点。

    arguments
        position (3, :) double
        start_anchor (3, 1) double
        end_anchor (3, 1) double
    end

    if size(position, 2) < 2
        error('双端点轨迹对齐至少需要两个轨迹点。');
    end

    reversed_position = flip(position, 2);
    [reversed_aligned, start_scale, start_rotation_deg] = ...
        rotateAndScaleTrajectory(reversed_position, start_anchor);
    start_aligned = flip(reversed_aligned, 2);

    [aligned_position, end_scale, end_rotation_deg] = ...
        rotateAndScaleTrajectory(start_aligned, end_anchor);

    % 消除坐标往返转换在两个约束端点处产生的数值残差。
    aligned_position(:, 1) = start_anchor;
    aligned_position(:, end) = end_anchor;

    info = struct( ...
        'start_scale', start_scale, ...
        'start_rotation_deg', start_rotation_deg, ...
        'end_scale', end_scale, ...
        'end_rotation_deg', end_rotation_deg);
end
