function [joint_range_data, selection, preview_navstate, preview_kf] = ...
        build_range_azimuth_measurement_from_phase( ...
        navstate, kf, range_data, depth_data, wrapped_phase_deg, options)
%BUILD_RANGE_AZIMUTH_MEASUREMENT_FROM_PHASE 构造联合更新所需的观测行。
%   先在状态副本上执行距离+深度更新，再按正式滤波采用的反馈方式构造
%   候选角裁决先验；原始 kf/navstate 不在本函数中修改。返回：
%   [time, slant range, horizontal range, beacon lat/lon/h,
%    selected relative azimuth, NaN, azimuth std]。

    if numel(range_data) < 6
        error('range_data 至少应包含前 6 列。');
    end
    if ~isfield(options, 'filter_azimuth_std_deg') || ...
            ~isscalar(options.filter_azimuth_std_deg) || ...
            ~isfinite(options.filter_azimuth_std_deg) || ...
            options.filter_azimuth_std_deg <= 0
        error('options.filter_azimuth_std_deg 必须为正有限标量。');
    end
    if ~isfield(options, 'feedback_attitude')
        options.feedback_attitude = true;
    end
    if ~isfield(options, 'feedback_imu_bias')
        options.feedback_imu_bias = true;
    end
    if ~isfield(options, 'match_range_only_feedback')
        options.match_range_only_feedback = false;
    end
    if ~isfield(options, 'feedback_heading_only')
        options.feedback_heading_only = false;
    end

    preview_kf = myRangeUpdate(navstate, range_data, depth_data, kf);
    if options.match_range_only_feedback
        if options.feedback_heading_only
            [preview_kf, preview_navstate] = ...
                feedback_range_azimuth_state( ...
                preview_kf, navstate, false, true, true);
        else
            [preview_kf, preview_navstate] = ...
                myErrorFeedback_range(preview_kf, navstate);
        end
    else
        [preview_kf, preview_navstate] = ...
            feedback_range_azimuth_state(preview_kf, navstate, ...
            options.feedback_attitude, options.feedback_imu_bias);
    end

    selection = select_phase_azimuth_candidate( ...
        wrapped_phase_deg, preview_navstate, range_data(4:6), options, ...
        preview_kf.x(9));

    joint_range_data = [range_data(1:6), ...
        selection.selected_relative_azimuth_deg, NaN, ...
        options.filter_azimuth_std_deg];
end
