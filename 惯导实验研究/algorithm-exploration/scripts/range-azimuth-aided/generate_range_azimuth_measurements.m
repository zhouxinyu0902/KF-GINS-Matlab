function outputs = generate_range_azimuth_measurements(case_name, user_options)
%GENERATE_RANGE_AZIMUTH_MEASUREMENTS 根据仿真真值构造相对方位角观测。
%   outputs = generate_range_azimuth_measurements('case-00')
%   读取 algorithm-exploration 原仿真 truth.txt 和 range1~3.txt，在新的
%   range-azimuth-aided 输入目录中生成带第 7~9 列的三个观测文件。
%
%   新增列：7 带噪方位角(deg)，8 真实方位角(deg)，9 当前噪声标准差(deg)。
%   原文件前 6 列原样保留，不修改源仿真数据。

    if nargin < 1 || isempty(case_name)
        case_name = 'case-00';
    end
    if nargin < 2 || isempty(user_options)
        user_options = struct();
    end
    case_name = char(string(case_name));

    options.sigma_min_deg = option_value(user_options, ...
        'sigma_min_deg', 0.10);
    options.sigma_max_deg = option_value(user_options, ...
        'sigma_max_deg', 0.50);
    options.shape_power = option_value(user_options, ...
        'shape_power', 1.5);
    options.jitter_ratio = option_value(user_options, ...
        'jitter_ratio', 0.20);
    options.random_seed = option_value(user_options, 'random_seed', 20260831);
    options.baseline_normal_offset_deg = option_value(user_options, ...
        'baseline_normal_offset_deg', 90);

    script_dir = fileparts(mfilename('fullpath'));
    topic_dir = fileparts(fileparts(script_dir));
    addpath(script_dir);
    addpath(topic_dir);
    paths = setup_inertial_experiment();

    source_dir = fullfile(paths.simulation_input, case_name);
    output_dir = fullfile(source_dir, 'range-azimuth-aided');
    if ~isfolder(source_dir)
        error('找不到仿真数据集：%s', source_dir);
    end
    if ~isfolder(output_dir)
        mkdir(output_dir);
    end

    truth_path = fullfile(source_dir, 'truth.txt');
    if ~isfile(truth_path)
        error('找不到真值文件：%s', truth_path);
    end
    truth = readmatrix(truth_path, 'FileType', 'text');
    if size(truth, 2) < 11
        error('truth.txt 至少应包含 11 列，当前只有 %d 列。', size(truth, 2));
    end

    truth_time = truth(:, 2);
    truth_lat_rad = deg2rad(truth(:, 3));
    truth_lon_rad = deg2rad(truth(:, 4));
    truth_height_m = truth(:, 5);
    % 航向先展开再插值，避免跨越 ±180° 时出现错误中间值。
    truth_heading_rad = unwrap(deg2rad(truth(:, 11)));

    output_paths = cell(3, 1);
    diagnostic_tables = cell(3, 1);
    for beacon_index = 1:3
        source_path = fullfile(source_dir, sprintf('range%d.txt', beacon_index));
        if ~isfile(source_path)
            error('找不到测距文件：%s', source_path);
        end
        range_data = readmatrix(source_path, 'FileType', 'text');
        if size(range_data, 2) < 6
            error('%s 至少应包含 6 列。', source_path);
        end

        measurement_time = range_data(:, 1);
        vehicle_lat = interp1(truth_time, truth_lat_rad, ...
            measurement_time, 'linear', 'extrap');
        vehicle_lon = interp1(truth_time, truth_lon_rad, ...
            measurement_time, 'linear', 'extrap');
        vehicle_height = interp1(truth_time, truth_height_m, ...
            measurement_time, 'linear', 'extrap');
        vehicle_heading = interp1(truth_time, truth_heading_rad, ...
            measurement_time, 'linear', 'extrap');

        beacon_lat = range_data(:, 4);
        beacon_lon = range_data(:, 5);
        beacon_height = range_data(:, 6);
        [rm, rn] = wgs84_radii(beacon_lat);
        delta_n = (beacon_lat - vehicle_lat) .* (rm + beacon_height);
        delta_e = (beacon_lon - vehicle_lon) .* (rn + beacon_height) .* ...
            cos(vehicle_lat);
        true_azimuth_deg = wrap_to_180_local(rad2deg( ...
            atan2(delta_e, delta_n) - vehicle_heading) - ...
            options.baseline_normal_offset_deg);

        rng(options.random_seed + beacon_index - 1, 'twister');
        [measured_azimuth_deg, sigma_deg, noise_deg] = ...
            add_irregular_azimuth_noise(true_azimuth_deg, options);
        augmented_data = [range_data(:, 1:6), measured_azimuth_deg, ...
            true_azimuth_deg, sigma_deg];
        output_path = fullfile(output_dir, sprintf('range%d.txt', beacon_index));
        writematrix(augmented_data, output_path, 'Delimiter', ' ');
        output_paths{beacon_index} = output_path;

        diagnostic_tables{beacon_index} = table( ...
            repmat(beacon_index, numel(measurement_time), 1), ...
            measurement_time, vehicle_lat, vehicle_lon, vehicle_height, ...
            rad2deg(vehicle_heading), true_azimuth_deg, ...
            measured_azimuth_deg, sigma_deg, noise_deg, ...
            'VariableNames', {'Beacon', 'Time_s', 'VehicleLat_rad', ...
            'VehicleLon_rad', 'VehicleHeight_m', 'Heading_deg', ...
            'TrueAzimuth_deg', 'MeasuredAzimuth_deg', ...
            'AzimuthStd_deg', 'AzimuthNoise_deg'});
    end

    diagnostics = vertcat(diagnostic_tables{:});
    diagnostic_path = fullfile(output_dir, 'azimuth-measurement-diagnostics.csv');
    writetable(diagnostics, diagnostic_path);
    configuration_path = fullfile(output_dir, 'azimuth-generation-config.mat');
    save(configuration_path, 'case_name', 'options');

    outputs = struct('case_name', case_name, 'source_dir', source_dir, ...
        'output_dir', output_dir, 'range_paths', {output_paths}, ...
        'diagnostic_path', diagnostic_path, ...
        'configuration_path', configuration_path, 'options', options);
    fprintf('已生成 %s 的距离+方位角观测：%s\n', case_name, output_dir);
end

function value = option_value(options, field_name, default_value)
    if isfield(options, field_name) && ~isempty(options.(field_name))
        value = options.(field_name);
    else
        value = default_value;
    end
end

function [rm, rn] = wgs84_radii(latitude_rad)
    semi_major_axis = 6378137.0;
    eccentricity_squared = 6.6943799901413165e-3;
    denominator = sqrt(1 - eccentricity_squared .* sin(latitude_rad) .^ 2);
    rn = semi_major_axis ./ denominator;
    rm = semi_major_axis * (1 - eccentricity_squared) ./ denominator .^ 3;
end

function angle = wrap_to_180_local(angle)
    angle = mod(angle + 180, 360) - 180;
end
