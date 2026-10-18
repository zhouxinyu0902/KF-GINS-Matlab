function kf = myRangeUpdate_iterated_m(navstate, rangedata, ...
        depthdata, kf, maximum_iterations, position_tolerance_m)
%MYRANGEUPDATE_ITERATED_M 全米制15状态迭代ES-EKF测距更新。
%   内循环始终使用同一个先验均值和协方差，只改变非线性量测的
%   线性化点；收敛后仅执行一次Joseph协方差更新。

    if nargin < 5 || isempty(maximum_iterations)
        maximum_iterations = 5;
    end
    if nargin < 6 || isempty(position_tolerance_m)
        position_tolerance_m = 1e-3;
    end
    if kf.RANK ~= 15
        error('迭代测距更新要求15状态误差模型。');
    end

    beacon = rangedata(4:6)';
    measured_range = rangedata(3);
    measured_height = depthdata(2);
    if ~all(isfinite([beacon; measured_range; measured_height]))
        error('迭代测距更新收到非有限量测。');
    end

    % x(1:3)是“名义位置减真实位置”的NED米制误差。
    rm = navstate.Rm;
    rn = navstate.Rn;
    height = navstate.pos(3);
    geodetic_scale = diag([rm + height, ...
        (rn + height) * cos(navstate.pos(1)), -1]);
    delta_nominal = geodetic_scale * (navstate.pos - beacon);
    nominal_range = hypot(delta_nominal(1), delta_nominal(2));
    measurement = [nominal_range - measured_range; ...
        height - measured_height];
    measurement_covariance = diag([kf.rangstd ^ 2, kf.depthstd ^ 2]);

    prior_mean = kf.x;
    prior_covariance = (kf.P + kf.P') / 2;
    iterate = prior_mean;
    converged = false;

    [predicted, measurement_matrix, corrected_range] = ...
        linearize_measurement(iterate, delta_nominal, nominal_range, ...
        kf.RANK);
    initial_innovation = measurement - predicted;
    initial_innovation_covariance = measurement_matrix ...
        * prior_covariance * measurement_matrix' ...
        + measurement_covariance;
    initial_nis = initial_innovation' ...
        / initial_innovation_covariance * initial_innovation;
    initial_range_residual = corrected_range - measured_range;

    iteration_count = 0;
    final_measurement_matrix = measurement_matrix;
    final_kalman_gain = prior_covariance * measurement_matrix' ...
        / initial_innovation_covariance;
    for iteration = 1:maximum_iterations
        [predicted, measurement_matrix] = linearize_measurement( ...
            iterate, delta_nominal, nominal_range, kf.RANK);
        innovation_covariance = measurement_matrix ...
            * prior_covariance * measurement_matrix' ...
            + measurement_covariance;
        kalman_gain = prior_covariance * measurement_matrix' ...
            / innovation_covariance;
        final_measurement_matrix = measurement_matrix;
        final_kalman_gain = kalman_gain;

        % 标准IEKF修正：先验在整个内循环中保持不变。
        next_iterate = prior_mean + kalman_gain * (measurement ...
            - predicted + measurement_matrix * (iterate - prior_mean));
        iteration_count = iteration;
        if norm(next_iterate(1:3) - iterate(1:3), 2) ...
                <= position_tolerance_m
            iterate = next_iterate;
            converged = true;
            break;
        end
        iterate = next_iterate;
    end

    [~, ~, corrected_range] = ...
        linearize_measurement(iterate, delta_nominal, nominal_range, ...
        kf.RANK);
    identity = eye(kf.RANK);
    covariance_transition = identity ...
        - final_kalman_gain * final_measurement_matrix;

    kf.x = iterate;
    kf.P = covariance_transition * prior_covariance ...
        * covariance_transition' ...
        + final_kalman_gain * measurement_covariance ...
        * final_kalman_gain';
    kf.P = (kf.P + kf.P') / 2;
    kf.Z = measurement;
    kf.Zkk_1 = predicted;
    kf.Znew = corrected_range - measured_range;

    % 诊断字段供主脚本记录，不参与滤波递推。
    kf.range_iteration_count = iteration_count;
    kf.range_initial_residual_m = initial_range_residual;
    kf.range_final_residual_m = kf.Znew;
    kf.range_position_correction_m = ...
        norm(iterate(1:3) - prior_mean(1:3), 2);
    kf.range_initial_nis = initial_nis;
    kf.range_converged = double(converged);
end

function [predicted, measurement_matrix, corrected_range] = ...
        linearize_measurement(error_state, delta_nominal, ...
        nominal_range, state_rank)
    corrected_north = delta_nominal(1) - error_state(1);
    corrected_east = delta_nominal(2) - error_state(2);
    corrected_range = max(hypot(corrected_north, corrected_east), 1e-9);

    % measurement=[r_nom-r_meas; h_nom-h_meas]的非线性预测。
    predicted = [nominal_range - corrected_range; -error_state(3)];
    measurement_matrix = zeros(2, state_rank);
    measurement_matrix(1, 1) = corrected_north / corrected_range;
    measurement_matrix(1, 2) = corrected_east / corrected_range;
    measurement_matrix(2, 3) = -1;
end
