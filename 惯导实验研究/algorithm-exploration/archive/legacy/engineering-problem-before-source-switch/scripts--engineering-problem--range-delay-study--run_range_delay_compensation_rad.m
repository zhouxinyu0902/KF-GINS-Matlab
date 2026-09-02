clear;
clc;

%% 固定4 s测距延迟补偿对比（rad位置误差状态）
% 工况1：无延迟，t时刻到达并使用t时刻距离；
% 工况2：延迟4 s且不处理，在t时刻直接使用t-4 s距离；
% 工况3：延迟4 s且补偿，将量测恢复到真实采集时刻后重算历史轨迹。
%
% 三种算法分别评价：前向EKF、二次RTS、2RTS+旋转收缩。
% 工况3是固定滞后离线等价实现：在确定性计算下，按采集时刻重算与
% 量测到达后回退4 s、更新并重推进等价；它不是零延迟实时输出。

delay_s = 4;
range_interval_s = 420;
duration_s = 4621;

%% 路径和输入
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(script_dir);
addpath(topic_dir);
paths = setup_inertial_experiment();
core_script_dir = fullfile(topic_dir, 'scripts', 'rts-algorithm-study');
addpath(core_script_dir);
expected_core_path = fullfile(core_script_dir, ...
    'run_navigation_experiment.m');
resolved_core_path = which('run_navigation_experiment');
if isempty(resolved_core_path) || ...
        ~strcmpi(resolved_core_path, expected_core_path)
    error(['run_navigation_experiment存在路径冲突。\n', ...
        '期望：%s\n实际：%s\n', ...
        '请将MATLAB当前目录切换到本脚本目录后再运行。'], ...
        expected_core_path, resolved_core_path);
end

preprocessed_dir = paths.experiment_preprocessed;
raw_dir = paths.experiment_raw;
result_root = fullfile(paths.experiment_navigation, ...
    'range-delay-compensation-rad');
artifact_root = exploration_artifact_dir(result_root);
condition_result_dirs = { ...
    fullfile(result_root, 'no-delay'), ...
    fullfile(result_root, 'delayed-4s-uncompensated'), ...
    fullfile(result_root, 'delayed-4s-rollback-replay')};

required_files = { ...
    fullfile(preprocessed_dir, 'rangedata_noised.txt'), ...
    fullfile(preprocessed_dir, 'height_noised.txt'), ...
    fullfile(raw_dir, 'range1.txt'), ...
    fullfile(raw_dir, 'range2.txt'), ...
    fullfile(raw_dir, 'range3.txt'), ...
    fullfile(raw_dir, 'IMU_120.txt'), ...
    fullfile(paths.experiment_reference, 'truth.nav')};
for file_index = 1:numel(required_files)
    if ~isfile(required_files{file_index})
        error('缺少延迟补偿对比所需输入：%s', required_files{file_index});
    end
end
if ~isfolder(result_root), mkdir(result_root); end
if ~isfolder(artifact_root), mkdir(artifact_root); end

%% 构造当前距离和4 s陈旧距离
current_range = readmatrix(fullfile(preprocessed_dir, ...
    'rangedata_noised.txt'), 'FileType', 'text');
if size(current_range, 2) < 6
    error('rangedata_noised.txt至少应包含6列。');
end
if any(abs(diff(current_range(:, 1))-range_interval_s) > 1e-6)
    error('当前测距序列并非固定%.0f s间隔。', range_interval_s);
end

raw_range = cell(3, 1);
for beacon_index = 1:3
    raw_range{beacon_index} = readmatrix(fullfile(raw_dir, ...
        sprintf('range%d.txt', beacon_index)), 'FileType', 'text');
end
[delayed_range, measurement_table] = build_delayed_range_input( ...
    current_range, raw_range, delay_s);
compensated_range = delayed_range;
compensated_range(:, 1) = measurement_table.AcquisitionTime_s;
if any(diff(compensated_range(:, 1)) <= 0)
    error('补偿后的量测采集时刻必须严格递增。');
end
writematrix(current_range, fullfile(result_root, ...
    'range-input-no-delay.txt'), 'Delimiter', ' ');
writematrix(delayed_range, fullfile(result_root, ...
    'range-input-delayed-4s.txt'), 'Delimiter', ' ');
writematrix(compensated_range, fullfile(result_root, ...
    'range-input-delayed-4s-acquisition-time-aligned.txt'), ...
    'Delimiter', ' ');
writetable(measurement_table, fullfile(artifact_root, ...
    'range-input-comparison.csv'));

%% 三种延迟工况分别运行；每次同时生成三种算法结果
common_options = struct( ...
    'duration_s', duration_s, ...
    'enable_rotation', true, ...
    'enable_fixed_lag_replay', false);

fprintf('\n========== 工况1：无延迟 ==========\n');
no_delay_options = common_options;
no_delay_options.case_name = 'no-range-delay-rad';
no_delay_options.range_data = current_range;
no_delay_outputs = run_navigation_experiment( ...
    condition_result_dirs{1}, no_delay_options);

fprintf('\n========== 工况2：延迟4 s，不处理 ==========\n');
uncompensated_options = common_options;
uncompensated_options.case_name = 'range-delay-4s-uncompensated-rad';
uncompensated_options.range_data = delayed_range;
uncompensated_outputs = run_navigation_experiment( ...
    condition_result_dirs{2}, uncompensated_options);

fprintf('\n========== 工况3：延迟4 s，回退更新并重推进 ==========\n');
compensated_options = common_options;
compensated_options.case_name = 'range-delay-4s-rollback-replay-rad';
% 核心函数按量测时间戳处理事件。把陈旧量测恢复到采集时刻后整段重算，
% 等价于固定滞后系统在到达时回退、更新并利用同一段IMU重推进。
compensated_options.range_data = compensated_range;
compensated_outputs = run_navigation_experiment( ...
    condition_result_dirs{3}, compensated_options);

outputs = {no_delay_outputs, uncompensated_outputs, compensated_outputs};
condition_names = ["无延迟"; "延迟4 s未处理"; "延迟4 s回退重推进"];
truth = readmatrix(fullfile(paths.experiment_reference, 'truth.nav'), ...
    'FileType', 'text');

%% 读取三类算法结果
ekf_nav = cell(3, 1);
double_rts_nav = cell(3, 1);
rotation_nav = cell(3, 1);
for condition_index = 1:3
    ekf_nav{condition_index} = readmatrix( ...
        outputs{condition_index}.forward_path, 'FileType', 'text');
    double_rts_nav{condition_index} = readmatrix( ...
        outputs{condition_index}.double_rts_path, 'FileType', 'text');
    rotation_nav{condition_index} = readmatrix( ...
        outputs{condition_index}.rotation_contraction_path, ...
        'FileType', 'text');
end
validate_navigation_time_axes(ekf_nav, double_rts_nav, rotation_nav);

time = ekf_nav{1}(:, 2);
truth_time_mask = time >= truth(1, 2) & time <= truth(end, 2);
ekf_mask = common_finite_mask(ekf_nav, truth_time_mask);
double_rts_mask = common_finite_mask(double_rts_nav, truth_time_mask);
rotation_mask = common_finite_mask(rotation_nav, truth_time_mask);
for condition_index = 1:3
    double_rts_mask = double_rts_mask & ...
        outputs{condition_index}.double_rts_second_pass_mask;
    % rotation_mask = rotation_mask & ...
    %     outputs{condition_index}.rotation_contraction_applied_mask;
end
if ~any(ekf_mask) || ~any(double_rts_mask) || ~any(rotation_mask)
    error('至少一种算法没有找到三个工况共同有效的严格评价区间。');
end

%% 三种算法分开对比三种延迟工况
[ekf_statistics, ~] = evaluate_three_conditions( ...
    truth, ekf_nav, ekf_mask, condition_names);
writetable(ekf_statistics, fullfile(artifact_root, ...
    'ekf-delay-compensation-statistics.csv'));

[double_rts_statistics, ~] = evaluate_three_conditions( ...
    truth, double_rts_nav, double_rts_mask, condition_names);
writetable(double_rts_statistics, fullfile(artifact_root, ...
    'double-rts-delay-compensation-statistics.csv'));

[rotation_statistics, ~] = evaluate_three_conditions( ...
    truth, rotation_nav, rotation_mask, condition_names);
writetable(rotation_statistics, fullfile(artifact_root, ...
    'double-rts-rotation-delay-compensation-statistics.csv'));

summary = build_algorithm_summary( ...
    ["前向EKF"; "二次RTS"; "2RTS+旋转收缩"], ...
    {ekf_statistics, double_rts_statistics, rotation_statistics}, ...
    [nnz(ekf_mask); nnz(double_rts_mask); nnz(rotation_mask)], ...
    [time(find(ekf_mask, 1)); time(find(double_rts_mask, 1)); ...
    time(find(rotation_mask, 1))], ...
    [time(find(ekf_mask, 1, 'last')); ...
    time(find(double_rts_mask, 1, 'last')); ...
    time(find(rotation_mask, 1, 'last'))]);
writetable(summary, fullfile(artifact_root, ...
    'delay-compensation-three-algorithm-summary.csv'));

evaluation_context = struct( ...
    'version', 1, ...
    'delay_s', delay_s, ...
    'truth_path', fullfile(paths.experiment_reference, 'truth.nav'), ...
    'artifact_root', artifact_root, ...
    'condition_names', condition_names, ...
    'condition_result_dirs', {condition_result_dirs}, ...
    'ekf_mask', ekf_mask, ...
    'double_rts_mask', double_rts_mask, ...
    'rotation_mask', rotation_mask);
context_path = fullfile(artifact_root, ...
    'range-delay-compensation-evaluation-context.mat');
save(context_path, 'evaluation_context');

fprintf('\n前向EKF：三种延迟工况\n');
disp(ekf_statistics);
fprintf('\n二次RTS：三种延迟工况\n');
disp(double_rts_statistics);
fprintf('\n2RTS+旋转收缩：三种延迟工况\n');
disp(rotation_statistics);
fprintf('\n延迟补偿汇总：\n');
disp(summary);
fprintf(['\n注意：回退重推进的前向结果具有%.0f s固定滞后；', ...
    '二次RTS及旋转收缩还叠加各自原有的固定滞后。\n'], delay_s);
fprintf('导航结果：%s\n', result_root);
fprintf('统计与绘图上下文：%s\n', artifact_root);
fprintf(['评价与绘图请单独运行 scripts/evaluation/', ...
    'engineering-applications/range-delay/', ...
    'evaluate_range_delay_compensation_rad.m\n']);

%% 局部函数
function [delayed_range, comparison] = build_delayed_range_input( ...
        current_range, raw_range, delay_s)
%BUILD_DELAYED_RANGE_INPUT 保持到达时刻不变，构造同噪声的陈旧距离。
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
        beacon_id(event_index) = this_beacon;
        source = raw_range{this_beacon};
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
        'DelayedTrueRange_m', 'SharedNoise_m', 'NoDelayInputRange_m', ...
        'DelayedInputRange_m', 'StaleMinusCurrent_m'});
end

function validate_navigation_time_axes(ekf_nav, double_rts_nav, rotation_nav)
%VALIDATE_NAVIGATION_TIME_AXES 保证九份导航结果可以逐历元比较。
    reference_time = ekf_nav{1}(:, 2);
    groups = {ekf_nav, double_rts_nav, rotation_nav};
    for group_index = 1:numel(groups)
        for condition_index = 1:3
            nav = groups{group_index}{condition_index};
            if size(nav, 1) ~= numel(reference_time) || ...
                    any(abs(nav(:, 2)-reference_time) > 1e-8)
                error('导航结果长度或时间轴不一致。');
            end
        end
    end
end

function mask = common_finite_mask(nav_results, initial_mask)
%COMMON_FINITE_MASK 生成三个工况共同有效的有限值掩码。
    mask = initial_mask;
    for index = 1:numel(nav_results)
        mask = mask & all(isfinite(nav_results{index}(:, 2:5)), 2);
    end
end

function [statistics, radial_error] = evaluate_three_conditions( ...
        truth, nav_results, evaluation_mask, condition_names)
%EVALUATE_THREE_CONDITIONS 在同一算法严格共同区间内评价三个工况。
    time = nav_results{1}(:, 2);
    truth_position = interp1(truth(:, 2), truth(:, 3:5), time, ...
        'linear', 'extrap');
    radial_error = nan(numel(time), 3);
    rmse_m = zeros(3, 1);
    mean_m = zeros(3, 1);
    median_m = zeros(3, 1);
    p95_m = zeros(3, 1);
    maximum_m = zeros(3, 1);
    for index = 1:3
        radial_error(:, index) = horizontal_radial_error( ...
            nav_results{index}(:, 3:5), truth_position);
        value = radial_error(evaluation_mask, index);
        rmse_m(index) = sqrt(mean(value.^2));
        mean_m(index) = mean(value);
        median_m(index) = median(value);
        p95_m(index) = prctile(value, 95);
        maximum_m(index) = max(value);
    end
    condition = condition_names;
    statistics = table(condition, rmse_m, mean_m, median_m, p95_m, ...
        maximum_m, 'VariableNames', {'Condition', 'RMSE_m', 'Mean_m', ...
        'Median_m', 'P95_m', 'Maximum_m'});
end

function summary = build_algorithm_summary( ...
        algorithm, statistics, sample_count, start_time, end_time)
%BUILD_ALGORITHM_SUMMARY 汇总补偿相对未处理延迟的改善量。
    no_delay_rmse_m = zeros(3, 1);
    uncompensated_rmse_m = zeros(3, 1);
    compensated_rmse_m = zeros(3, 1);
    for index = 1:3
        no_delay_rmse_m(index) = statistics{index}.RMSE_m(1);
        uncompensated_rmse_m(index) = statistics{index}.RMSE_m(2);
        compensated_rmse_m(index) = statistics{index}.RMSE_m(3);
    end
    compensation_improvement_m = ...
        uncompensated_rmse_m-compensated_rmse_m;
    compensation_improvement_percent = 100*compensation_improvement_m ...
        ./ uncompensated_rmse_m;
    compensated_minus_no_delay_m = compensated_rmse_m-no_delay_rmse_m;
    summary = table(algorithm, sample_count, start_time, end_time, ...
        no_delay_rmse_m, uncompensated_rmse_m, compensated_rmse_m, ...
        compensation_improvement_m, compensation_improvement_percent, ...
        compensated_minus_no_delay_m, ...
        'VariableNames', {'Algorithm', 'EvaluatedSampleCount', ...
        'EvaluationStartTime_s', 'EvaluationEndTime_s', 'NoDelayRMSE_m', ...
        'UncompensatedRMSE_m', 'CompensatedRMSE_m', ...
        'CompensationImprovement_m', 'CompensationImprovement_percent', ...
        'CompensatedMinusNoDelay_m'});
end

function radial_error = horizontal_radial_error(estimate, truth)
%HORIZONTAL_RADIAL_ERROR 按WGS-84曲率半径计算水平径向误差。
    latitude = deg2rad(truth(:, 1));
    height = truth(:, 3);
    [rm, rn] = wgs84_radii(latitude);
    north_error = deg2rad(estimate(:, 1)-truth(:, 1)).*(rm+height);
    east_error = deg2rad(estimate(:, 2)-truth(:, 2)).* ...
        (rn+height).*cos(latitude);
    radial_error = hypot(north_error, east_error);
end

function [rm, rn] = wgs84_radii(latitude)
%WGS84_RADII 计算WGS-84曲率半径。
    semi_major_axis = 6378137.0;
    flattening = 1/298.257223563;
    eccentricity_squared = flattening*(2-flattening);
    denominator = sqrt(1-eccentricity_squared.*sin(latitude).^2);
    rn = semi_major_axis./denominator;
    rm = semi_major_axis*(1-eccentricity_squared)./denominator.^3;
end
