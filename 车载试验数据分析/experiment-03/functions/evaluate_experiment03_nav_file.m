function result = evaluate_experiment03_nav_file( ...
        truth_file, navigation_file, duration_seconds)
%EVALUATE_EXPERIMENT03_NAV_FILE 对单个导航文件计算水平径向误差。

    if nargin < 3
        duration_seconds = [];
    end
    truth = readmatrix(truth_file, 'FileType', 'text');
    navigation = readmatrix(navigation_file, 'FileType', 'text');
    if size(truth, 2) < 5 || size(navigation, 2) < 5
        error('truth和navigation都必须至少包含5列。');
    end

    truth = truth(all(isfinite(truth(:, 2:5)), 2), :);
    navigation = navigation(all(isfinite(navigation(:, 2:5)), 2), :);
    [truth_time, truth_unique] = unique(truth(:, 2), 'stable');
    truth_position = truth(truth_unique, 3:5);
    [navigation_time, navigation_unique] = unique( ...
        navigation(:, 2), 'stable');
    navigation_position = navigation(navigation_unique, 3:5);

    start_time = max(truth_time(1), navigation_time(1));
    end_time = min(truth_time(end), navigation_time(end));
    if ~isempty(duration_seconds)
        end_time = min(end_time, start_time + duration_seconds);
    end
    mask = navigation_time >= start_time & navigation_time <= end_time;
    time = navigation_time(mask);
    estimate = navigation_position(mask, :);
    if numel(time) < 2
        error('导航结果与真值没有足够的共同时间区间。');
    end
    reference = interp1(truth_time, truth_position, time, 'linear');

    [north_error, east_error] = position_error_ne(estimate, reference);
    radial_error = hypot(north_error, east_error);
    result = struct();
    result.time = time;
    result.elapsed_time = time - time(1);
    result.north_error_m = north_error;
    result.east_error_m = east_error;
    result.radial_error_m = radial_error;
    result.duration_s = result.elapsed_time(end);
    result.rmse_m = sqrt(mean(radial_error .^ 2));
    result.mean_m = mean(radial_error);
    result.median_m = median(radial_error);
    result.p95_m = prctile(radial_error, 95);
    result.maximum_m = max(radial_error);
    result.final_m = radial_error(end);
end

function [north_error, east_error] = position_error_ne(estimate, reference)
%POSITION_ERROR_NE 按WGS-84逐点将经纬度差转换为北东向米制误差。

    latitude = deg2rad(reference(:, 1));
    latitude_error = deg2rad(estimate(:, 1) - reference(:, 1));
    longitude_error = deg2rad(estimate(:, 2) - reference(:, 2));
    height = reference(:, 3);
    semi_major_axis = 6378137.0;
    eccentricity_squared = 6.6943799901413165e-3;
    denominator = sqrt(1 - eccentricity_squared .* sin(latitude) .^ 2);
    prime_vertical_radius = semi_major_axis ./ denominator;
    meridian_radius = semi_major_axis * (1 - eccentricity_squared) ./ ...
        denominator .^ 3;
    north_error = latitude_error .* (meridian_radius + height);
    east_error = longitude_error .* ...
        (prime_vertical_radius + height) .* cos(latitude);
end
