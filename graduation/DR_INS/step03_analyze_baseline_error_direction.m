%% 无距离 INS/DVL 基线的位置误差方向与协方差主轴
% 独立运行 INS + DVL + depth 基线，并在每个 DVL 历元记录：
%   1) 实际北向、东向位置误差；
%   2) 实际水平误差方向（从北向顺时针为正）；
%   3) 水平位置协方差椭圆的长短轴及长轴方向。
%
% 实际误差方向依赖真值，只能用于仿真事后分析；实际导航中可用
% 协方差椭圆主轴描述滤波器认为的主要不确定方向。

clear;
close all;
clc;

script_dir = fileparts(mfilename('fullpath'));
repo_root = fileparts(fileparts(script_dir));
addpath(script_dir);
addpath(fullfile(script_dir, 'function'));
addpath(fullfile(repo_root, 'GINS-KF'));
addpath(genpath(fullfile(repo_root, 'function_zxy')));
addpath(genpath(fullfile(repo_root, 'psins2401', 'base')));
glvs;

param = Param();
cfg = config_dvl();

%% 分析设置
maximum_duration_s = 3600.0;       % 改为 inf 可分析完整航段
minimum_direction_error_m = 0.10;  % 误差太小时方向没有实际意义
minimum_covariance_axis_ratio = 1.05; % 椭圆近似圆形时不解释主轴方向
time_equality_tolerance_s = 1e-9;

% 快速检查：
% setenv('DVL_ERROR_DIRECTION_SMOKE','1'); run('analyze_baseline_error_direction.m')
smoke_test = strcmpi(strtrim(getenv('DVL_ERROR_DIRECTION_SMOKE')), '1');
if smoke_test
    maximum_duration_s = 64.0;
end

%% 加载并截取数据
imudata = importdata(cfg.imufilepath);
dvldata = importdata(cfg.dvlfilepath);
heightdata = importdata(cfg.heightfilepath);
truth = importdata(cfg.truthpath);

cfg.starttime = max(cfg.starttime, imudata(1, 1));
cfg.endtime = min([cfg.endtime, imudata(end, 1), ...
    cfg.starttime + maximum_duration_s]);
imudata = imudata(imudata(:, 1) >= cfg.starttime & ...
    imudata(:, 1) <= cfg.endtime, :);
dvldata = dvldata(dvldata(:, 1) >= cfg.starttime & ...
    dvldata(:, 1) <= cfg.endtime, :);
heightdata = heightdata(heightdata(:, 1) >= cfg.starttime & ...
    heightdata(:, 1) <= cfg.endtime, :);
truth = truth(truth(:, 2) >= cfg.starttime & ...
    truth(:, 2) <= cfg.endtime, :);

if size(imudata, 1) < 2 || size(dvldata, 1) < 2 || size(truth, 1) < 2
    error('Not enough IMU, DVL or truth samples in the selected interval.');
end
if size(heightdata, 1) ~= size(imudata, 1) || ...
        any(abs(heightdata(:, 1) - imudata(:, 1)) > 1e-9)
    error('The 100 Hz depth data must be synchronized with the IMU data.');
end
truth_at_dvl = interp1(truth(:, 2), truth(:, 3:11), ...
    dvldata(:, 1), 'linear');

%% 初始化无距离基线
[kf, navstate] = myInitialize_15state(cfg);
kf.depthstd = 0.4;
laststate = navstate;
lastimu = imudata(1, :)';
thisimu = imudata(1, :)';
dvlindex = 2;

% 列：time, N error, E error, horizontal error, error direction,
%     error axis, covariance major axis, major std, minor std,
%     covariance N/E correlation, truth heading
maximum_record_count = max(size(dvldata, 1) - 1, 1);
direction_result = zeros(maximum_record_count, 11);
covariance_ne_m2 = zeros(2, 2, maximum_record_count);
record_count = 0;

fprintf('Start baseline direction analysis, duration %.1f s.\n', ...
    imudata(end, 1) - imudata(1, 1));

%% 与 DVL_ins.m 相同的机械编排和量测更新顺序
for imuindex = 2:size(imudata, 1)
    lastimu = thisimu;
    laststate = navstate;
    thisimu = imudata(imuindex, :)';
    imudt = thisimu(1) - lastimu(1);

    while dvlindex <= size(dvldata, 1) && ...
            dvldata(dvlindex, 1) < lastimu(1) - time_equality_tolerance_s
        dvlindex = dvlindex + 1;
    end
    if dvlindex > size(dvldata, 1)
        break;
    end

    dvl_time_s = dvldata(dvlindex, 1);
    if abs(lastimu(1) - dvl_time_s) <= time_equality_tolerance_s
        update_time_s = lastimu(1);
        truth_row = truth_at_dvl(dvlindex, :);
        depth_meas_m = heightdata(max(imuindex - 1, 1), 2);
        DVLdata = [dvldata(dvlindex, :), depth_meas_m];
        kf = myDVLupdate(navstate, DVLdata, kf);
        [kf, navstate] = myErrorFeedback_15state(kf, navstate);

        record_count = record_count + 1;
        [direction_result(record_count, :), ...
            covariance_ne_m2(:, :, record_count)] = make_direction_sample( ...
            update_time_s, navstate, kf, truth_row, param);
        dvlindex = dvlindex + 1;

        laststate = navstate;
        navstate = InsMech(laststate, lastimu, thisimu);
        kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);

    elseif lastimu(1) < dvl_time_s && thisimu(1) > dvl_time_s
        [firstimu, secondimu] = interpolate(lastimu, thisimu, dvl_time_s);
        first_dt_s = firstimu(1) - lastimu(1);
        navstate = InsMech(laststate, lastimu, firstimu);
        kf = myInsPropagate_15state(navstate, firstimu, first_dt_s, kf);

        update_time_s = dvl_time_s;
        truth_row = truth_at_dvl(dvlindex, :);
        depth_meas_m = interp1(heightdata(imuindex-1:imuindex, 1), ...
            heightdata(imuindex-1:imuindex, 2), dvl_time_s, 'linear');
        DVLdata = [dvldata(dvlindex, :), depth_meas_m];
        kf = myDVLupdate(navstate, DVLdata, kf);
        [kf, navstate] = myErrorFeedback_15state(kf, navstate);

        record_count = record_count + 1;
        [direction_result(record_count, :), ...
            covariance_ne_m2(:, :, record_count)] = make_direction_sample( ...
            update_time_s, navstate, kf, truth_row, param);
        dvlindex = dvlindex + 1;

        laststate = navstate;
        lastimu = firstimu;
        second_dt_s = secondimu(1) - lastimu(1);
        navstate = InsMech(laststate, lastimu, secondimu);
        kf = myInsPropagate_15state(navstate, secondimu, second_dt_s, kf);
    else
        navstate = InsMech(laststate, lastimu, thisimu);
        height_measurement = [heightdata(imuindex, 1), -heightdata(imuindex, 2)];
        kf = myHeightUpdate(navstate, height_measurement, kf);
        navstate.pos(3) = navstate.pos(3) - kf.x(3);
        navstate.vel(3) = navstate.vel(3) - kf.x(6);
        kf.x(3) = 0;
        kf.x(6) = 0;
        kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
    end

    if record_count > 0 && mod(record_count, 2000) == 0 && ...
            abs(lastimu(1) - dvl_time_s) <= time_equality_tolerance_s
        fprintf('Recorded %d DVL epochs.\n', record_count);
    end
end

direction_result = direction_result(1:record_count, :);
covariance_ne_m2 = covariance_ne_m2(:, :, 1:record_count);

record_time_s = direction_result(:, 1);
north_error_m = direction_result(:, 2);
east_error_m = direction_result(:, 3);
horizontal_error_m = direction_result(:, 4);
error_direction_deg = direction_result(:, 5);
error_axis_deg = direction_result(:, 6);
covariance_major_axis_deg = direction_result(:, 7);
covariance_major_std_m = direction_result(:, 8);
covariance_minor_std_m = direction_result(:, 9);
covariance_correlation_ne = direction_result(:, 10);
truth_heading_deg = direction_result(:, 11);
covariance_axis_ratio = covariance_major_std_m ./ ...
    max(covariance_minor_std_m, eps);

valid_error_direction = horizontal_error_m >= minimum_direction_error_m;
error_direction_deg(~valid_error_direction) = nan;
error_axis_deg(~valid_error_direction) = nan;
covariance_axis_reliable = covariance_axis_ratio >= ...
    minimum_covariance_axis_ratio;
axis_mismatch_deg = abs(wrap_axis_90( ...
    error_axis_deg - covariance_major_axis_deg));
axis_mismatch_deg(~covariance_axis_reliable) = nan;
error_direction_unwrapped_deg = unwrap_direction_with_nan( ...
    error_direction_deg, 360);
error_axis_unwrapped_deg = unwrap_direction_with_nan(error_axis_deg, 180);
covariance_axis_unwrapped_deg = unwrap_direction_with_nan( ...
    covariance_major_axis_deg, 180);
covariance_axis_aligned_deg = align_axis_to_reference( ...
    covariance_major_axis_deg, error_axis_unwrapped_deg);
covariance_axis_reliable_deg = covariance_axis_aligned_deg;
covariance_axis_reliable_deg(~covariance_axis_reliable) = nan;

%% 保存结果
if smoke_test
    output_dir = fullfile(repo_root, 'data', 'graduation', ...
        '_smoke_error_direction');
else
    output_dir = fullfile(repo_root, 'data', 'graduation', 'INS_DVL', ...
        sprintf('error_direction_%gs', maximum_duration_s));
end
if ~isfolder(output_dir)
    mkdir(output_dir);
end

result_table = table(record_time_s, north_error_m, east_error_m, ...
    horizontal_error_m, error_direction_deg, error_direction_unwrapped_deg, ...
    error_axis_deg, covariance_major_axis_deg, ...
    covariance_axis_unwrapped_deg, covariance_major_std_m, ...
    covariance_minor_std_m, covariance_correlation_ne, ...
    covariance_axis_ratio, covariance_axis_reliable, ...
    axis_mismatch_deg, truth_heading_deg, ...
    'VariableNames', {'Time_s', 'NorthError_m', 'EastError_m', ...
    'HorizontalError_m', 'ErrorDirection_deg', ...
    'ErrorDirectionUnwrapped_deg', 'ErrorAxis_deg', ...
    'CovarianceMajorAxis_deg', 'CovarianceMajorAxisUnwrapped_deg', ...
    'CovarianceMajorStd_m', 'CovarianceMinorStd_m', ...
    'CovarianceCorrelationNE', 'CovarianceAxisRatio', ...
    'CovarianceAxisReliable', 'ErrorCovarianceAxisMismatch_deg', ...
    'TruthHeading_deg'});
writetable(result_table, fullfile(output_dir, ...
    'baseline_error_and_covariance_direction.csv'));

%% 图 1：实际位置误差方向
time_min = (record_time_s - record_time_s(1)) / 60;
time_limits_min = [time_min(1), time_min(end)];
error_fig = figure('Color', 'w', 'Name', 'Baseline position error direction');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(time_min, north_error_m, 'LineWidth', 1.1);
hold on;
plot(time_min, east_error_m, 'LineWidth', 1.1);
yline(0, 'k:');
grid on;
ylabel('Error (m)');
xlim(time_limits_min);
legend('North', 'East', 'Location', 'best');
title('INS/DVL no-range baseline position error');

nexttile;
plot(time_min, horizontal_error_m, 'LineWidth', 1.1);
grid on;
ylabel('Horizontal error (m)');
xlim(time_limits_min);

nexttile;
plot(time_min, error_direction_deg, '.', 'MarkerSize', 4);
hold on;
plot(time_min, wrap_to_180(truth_heading_deg), '--', 'LineWidth', 1.0);
grid on;
xlabel('Time (min)');
ylabel('Direction (deg)');
ylim([-180, 180]);
yticks(-180:45:180);
xlim(time_limits_min);
legend('Position-error direction', 'Vehicle heading', 'Location', 'best');
title('Direction measured clockwise from North');
exportgraphics(error_fig, fullfile(output_dir, ...
    '01_baseline_position_error_direction.png'), 'Resolution', 200);

%% 图 2：位置协方差椭圆主轴方向
covariance_fig = figure('Color', 'w', ...
    'Name', 'Position covariance principal axis');
tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(time_min, covariance_major_std_m, 'LineWidth', 1.1);
hold on;
plot(time_min, covariance_minor_std_m, 'LineWidth', 1.1);
grid on;
ylabel('1\sigma (m)');
xlim(time_limits_min);
legend('Major axis', 'Minor axis', 'Location', 'best');
title('Horizontal position covariance ellipse');

nexttile;
plot(time_min, covariance_axis_aligned_deg, ...
    'Color', [0.75, 0.75, 0.75], 'LineWidth', 0.8);
hold on;
plot(time_min, error_axis_unwrapped_deg, '.', 'MarkerSize', 4);
plot(time_min, covariance_axis_reliable_deg, ...
    'LineWidth', 1.2);
grid on;
ylabel('Axis direction (deg)');
xlim(time_limits_min);
legend('Raw covariance axis', 'Actual error axis', ...
    'Reliable covariance axis', 'Location', 'best');
title('Covariance axis aligned by 180 deg to the actual-error axis');

nexttile;
plot(time_min, covariance_axis_ratio, 'LineWidth', 1.1);
hold on;
yline(minimum_covariance_axis_ratio, 'k--');
grid on;
ylabel('\sigma_{major}/\sigma_{minor}');
xlim(time_limits_min);
title('Axis direction is unreliable when the ellipse is nearly circular');

nexttile;
plot(time_min, axis_mismatch_deg, 'LineWidth', 1.1);
hold on;
yline(45, 'k--');
grid on;
xlabel('Time (min)');
ylabel('Axis mismatch (deg)');
ylim([0, 90]);
xlim(time_limits_min);
title('Actual-error axis versus covariance major axis');
if ~any(isfinite(axis_mismatch_deg))
    text(mean(time_limits_min), 45, ...
        'No reliable axis: covariance ellipse is nearly circular', ...
        'HorizontalAlignment', 'center');
end
exportgraphics(covariance_fig, fullfile(output_dir, ...
    '02_position_covariance_principal_axis.png'), 'Resolution', 200);

save(fullfile(output_dir, 'baseline_error_direction_result.mat'), ...
    'result_table', 'covariance_ne_m2', 'maximum_duration_s', ...
    'minimum_direction_error_m', 'minimum_covariance_axis_ratio');

fprintf('Direction analysis finished: %s\n', output_dir);
fprintf('North/East RMSE: %.3f / %.3f m\n', ...
    sqrt(mean(north_error_m.^2)), sqrt(mean(east_error_m.^2)));
fprintf('Mean covariance major/minor axis ratio: %.4f\n', ...
    mean(covariance_axis_ratio));
if any(isfinite(axis_mismatch_deg))
    fprintf('Median reliable error/covariance-axis mismatch: %.2f deg\n', ...
        median(axis_mismatch_deg, 'omitnan'));
else
    fprintf(['No reliable covariance-axis direction: all axis ratios are ', ...
        'below %.2f.\n'], minimum_covariance_axis_ratio);
end


function [sample_row, covariance_ne] = make_direction_sample( ...
    update_time_s, navstate, kf, truth_row, param)
    truth_pos = [truth_row(1:2) * param.D2R, truth_row(3)]';
    [rm, rn] = getRmRn(truth_pos(1), param);
    position_transform = diag([rm + truth_pos(3), ...
        (rn + truth_pos(3)) * cos(truth_pos(1))]);

    position_error_ne = position_transform * ...
        (navstate.pos(1:2) - truth_pos(1:2));
    covariance_ne = position_transform * kf.P(1:2, 1:2) * ...
        position_transform';
    covariance_ne = 0.5 * (covariance_ne + covariance_ne');

    [eigenvectors, eigenvalues_matrix] = eig(covariance_ne);
    eigenvalues = real(diag(eigenvalues_matrix));
    [eigenvalues, order] = sort(eigenvalues, 'descend');
    eigenvectors = real(eigenvectors(:, order));
    major_vector_ne = eigenvectors(:, 1);

    horizontal_error = norm(position_error_ne);
    error_direction = wrap_to_180(atan2d( ...
        position_error_ne(2), position_error_ne(1)));
    error_axis = wrap_axis_90(error_direction);
    major_axis = wrap_axis_90(atan2d( ...
        major_vector_ne(2), major_vector_ne(1)));
    correlation_ne = covariance_ne(1, 2) / max(sqrt( ...
        covariance_ne(1, 1) * covariance_ne(2, 2)), eps);

    sample_row = [update_time_s, position_error_ne', horizontal_error, ...
        error_direction, error_axis, major_axis, ...
        sqrt(max(eigenvalues(1), 0)), sqrt(max(eigenvalues(2), 0)), ...
        correlation_ne, truth_row(9)];
end


function angle_deg = wrap_to_180(angle_deg)
    angle_deg = mod(angle_deg + 180, 360) - 180;
end


function angle_deg = wrap_axis_90(angle_deg)
% 无方向轴以 180 deg 为周期，统一映射到 [-90, 90)。
    angle_deg = mod(angle_deg + 90, 180) - 90;
end


function unwrapped_deg = unwrap_direction_with_nan(angle_deg, period_deg)
    unwrapped_deg = nan(size(angle_deg));
    valid_index = find(isfinite(angle_deg));
    if isempty(valid_index)
        return;
    end
    unwrapped_deg(valid_index) = angle_deg(valid_index);
    for index = 2:numel(valid_index)
        this_index = valid_index(index);
        previous_index = valid_index(index - 1);
        candidate = unwrapped_deg(this_index);
        candidate = candidate + period_deg * round( ...
            (unwrapped_deg(previous_index) - candidate) / period_deg);
        unwrapped_deg(this_index) = candidate;
    end
end


function aligned_deg = align_axis_to_reference(axis_deg, reference_deg)
% 为便于同图比较，给无方向轴增加 180 deg 整数倍，使其最接近参考轴。
    aligned_deg = axis_deg;
    valid = isfinite(axis_deg) & isfinite(reference_deg);
    aligned_deg(valid) = axis_deg(valid) + 180 * round( ...
        (reference_deg(valid) - axis_deg(valid)) / 180);
end
