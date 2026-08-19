function [kf, navstate, closed_loop_transition] = update_decoupled_height( ...
        kf, navstate, height_measurement)
%UPDATE_DECOUPLED_HEIGHT 执行与 all_m.m 一致的解耦高度 Kalman 更新。
%   height_measurement = [时间, 高度观测值]。
%   高度量测只更新垂向位置误差 x(3) 和垂向速度误差 x(6)，不把高度
%   信息传入水平位置、姿态或 IMU 零偏。反馈后只清零这两个误差状态，
%   其余误差状态继续保留给测距、POS等水平量测使用。

    closed_loop_transition = eye(kf.RANK);
    if numel(height_measurement) < 2 || ...
            ~isfinite(height_measurement(2))
        return;
    end

    % 当前仿真主链采用弧度制位置误差状态：x(3) 与名义高度误差同号，
    % 因此 H(3)=1，反馈时从名义高度中减去 x(3)。
    innovation = navstate.pos(3) - height_measurement(2);
    measurement_matrix = zeros(1, kf.RANK);
    measurement_matrix(3) = 1;
    predicted_measurement = measurement_matrix * kf.x;
    measurement_variance = kf.depthstd ^ 2;

    full_gain = kf.P * measurement_matrix' / ...
        (measurement_matrix * kf.P * measurement_matrix' ...
        + measurement_variance);
    feedback_mask = zeros(kf.RANK, 1);
    feedback_mask([3, 6]) = 1;
    kalman_gain = full_gain .* feedback_mask;

    kf.Z = innovation;
    kf.Zkk_1 = predicted_measurement;
    kf.x = kf.x + kalman_gain * ...
        (innovation - predicted_measurement);

    identity = eye(kf.RANK);
    % 误差均值经过量测闭环后的转移矩阵。RTS 必须把它与惯性传播矩阵
    % 一同累计，否则高频高度更新后的协方差与状态转移不再匹配。
    closed_loop_transition = identity ...
        - kalman_gain * measurement_matrix;
    kf.P = closed_loop_transition * kf.P ...
        * closed_loop_transition' ...
        + kalman_gain * measurement_variance * kalman_gain';
    kf.P = (kf.P + kf.P') / 2;

    % 与 all_m.m 的垂向闭环反馈一致。
    navstate.pos(3) = navstate.pos(3) - kf.x(3);
    navstate.vel(3) = navstate.vel(3) - kf.x(6);
    kf.x(3) = 0;
    kf.x(6) = 0;
    navstate.gravity = getGravity(navstate.pos);
end
