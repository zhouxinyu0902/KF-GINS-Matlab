clear; clc; close all;
%% ============================================================
% case-00 第一次测距点：导航先验辅助相位周期裁决
%
% 过程：
%   1. 按原导航程序传播到第一次测距时刻
%   2. 执行距离+深度更新并反馈到 navstate
%   3. 根据约束后的导航位置、航向及已知潜标位置计算预测到达角
%   4. 读取同步的相位差量测，生成全部周期候选到达角
%   5. 选择与导航预测角最接近的候选
%   6. 使用 truth 计算真实到达角，仅用于判断是否选对
%   7. 第一次测距处理完成后立即停止
%
% 角度定义：
%   baseline_axis  : H1 -> H2 基线方向
%   baseline_normal: baseline_axis + 90 deg
%   theta          : 来波相对 baseline_normal 的主值角 [-90,90] deg
% ============================================================

%% 1. 参数
simulation_case = 'case-00';
position_error_unit = "rad";
range_interval_s = 420;
duration_s = 4621;
beacon_order = [1, 2, 3];

simulation_range_noise_std_m = 10;
simulation_depth_noise_std_m = 0.4;
random_seed = 1;

baseline_m = 3.0;                  % 双水听器基线，m
baseline_install_deg = 0;          % H1->H2 相对船首安装角，deg
carrier_hz = 12e3;                 % 载频，Hz
sound_speed_mps = 1500;            % 声速，m/s
lambda = sound_speed_mps / carrier_hz;

%% 2. 初始化工程
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);

paths = setup_inertial_experiment();
param = Param();
glvs;
rng(random_seed, 'twister');

case_name = simulation_case;
input_dir = fullfile(paths.simulation_input, case_name);
cfg = load_algorithm_exploration_config( ...
    "simulation", position_error_unit, input_dir);

if ~isfolder(input_dir)
    error('仿真输入目录不存在：%s', input_dir);
end

cfg.userange = true;

% 与原导航程序保持相同的 rad 误差状态链
range_update_function = @myRangeUpdate;
feedback_function = @myErrorFeedback_range;
height_update_function = @update_decoupled_height;
propagation_function = @myInsPropagate_15state;

%% 3. 读取 IMU、truth 和三个距离源
imudata_all = readmatrix(cfg.imufilepath, 'FileType', 'text');
truth = readmatrix(cfg.truthpath, 'FileType', 'text');

range_sources = { ...
    readmatrix(cfg.rangefile1path, 'FileType', 'text'), ...
    readmatrix(cfg.rangefile2path, 'FileType', 'text'), ...
    readmatrix(cfg.rangefile3path, 'FileType', 'text')};

%% 4. 按原导航程序构造 420 s 轮换测距事件
source_interval_s = median(diff(range_sources{1}(:, 1)));
range_stride = round(range_interval_s / source_interval_s);

for i = 1:numel(range_sources)
    range_sources{i} = range_sources{i}(range_stride:range_stride:end, :);
end

event_count = min(cellfun(@(x) size(x, 1), range_sources));
rangedata = zeros(event_count, size(range_sources{1}, 2));
range_beacon_id = zeros(event_count, 1);

for event_index = 1:event_count
    order_index = mod(event_index - 1, numel(beacon_order)) + 1;
    beacon_id = beacon_order(order_index);

    rangedata(event_index, :) = range_sources{beacon_id}(event_index, :);
    range_beacon_id(event_index) = beacon_id;
end

% 与原导航程序一致，对第3列距离加入噪声
rangedata(:, 3) = rangedata(:, 3) + ...
    simulation_range_noise_std_m * randn(size(rangedata, 1), 1);

%% 5. 截取有效时间范围
start_time = max([cfg.starttime, imudata_all(1, 1), truth(1, 2)]);
end_time = min([start_time + duration_s, cfg.endtime, ...
    imudata_all(end, 1), truth(end, 2)]);

cfg.starttime = start_time;
cfg.endtime = end_time;

imu_mask = imudata_all(:, 1) >= start_time & ...
    imudata_all(:, 1) <= end_time;
imudata = imudata_all(imu_mask, :);

range_mask = rangedata(:, 1) >= start_time & ...
    rangedata(:, 1) <= end_time;
rangedata = rangedata(range_mask, :);
range_beacon_id = range_beacon_id(range_mask);

if isempty(rangedata)
    error('当前时间范围内没有测距事件。');
end

% 高度量测与原导航程序保持一致
height_value = interp1(truth(:, 2), truth(:, 5), ...
    imudata(:, 1), 'linear', 'extrap');

height = [ ...
    imudata(:, 1), ...
    height_value + simulation_depth_noise_std_m * randn(size(height_value))];

% 将测距时刻插入 IMU 时间轴
[imudata, height] = ...
    align_imu_to_range_epochs(imudata, height, rangedata(:, 1));

%% 6. 初始化导航滤波器
[kf, navstate] = myInitialize_15state(cfg);

kf.rangstd = simulation_range_noise_std_m;
kf.depthstd = simulation_depth_noise_std_m;

last_imu = imudata(1, :)';
this_imu = imudata(1, :)';

range_index = find(rangedata(:, 1) >= this_imu(1), 1, 'first');

%% 7. 传播到第一次测距点
first_range_result = [];

for imu_index = 2:size(imudata, 1)

    last_imu = this_imu;
    this_imu = imudata(imu_index, :)';

    imu_dt = this_imu(1) - last_imu(1);
    time_tolerance = max(1e-8, abs(imu_dt) * 0.25);

    is_range_epoch = range_index <= size(rangedata, 1) && ...
        abs(last_imu(1) - rangedata(range_index, 1)) <= time_tolerance;

    %% 7.1 第一次测距时刻
    if is_range_epoch

        range_time = rangedata(range_index, 1);
        beacon_id = range_beacon_id(range_index);

        % 保存量测更新前状态，仅用于对比
        nav_before = navstate;

        % 距离 + 深度更新
        kf = range_update_function( ...
            navstate, ...
            rangedata(range_index, :), ...
            height(imu_index - 1, :), ...
            kf);

        % 将估计误差反馈到名义导航状态
        [kf, navstate] = feedback_function(kf, navstate);

        nav_after = navstate;

        %% 7.2 根据更新后的导航状态计算预测到达角
        % navstate.pos = [lat(rad), lon(rad), h(m)]
        nav_lat_rad = nav_after.pos(1);
        nav_lon_rad = nav_after.pos(2);

        % rangedata 第4、5列为潜标纬经度(rad)
        beacon_lat_rad = rangedata(range_index, 4);
        beacon_lon_rad = rangedata(range_index, 5);

        % 当前导航航向
        heading_nav_deg = mod(nav_after.att(3) * param.R2D, 360);

        % H1 -> H2 实际基线方向
        baseline_axis_nav_deg = mod( ...
            heading_nav_deg + baseline_install_deg, 360);

        % 采用右侧法向作为 theta=0 的参考法向
        baseline_normal_nav_deg = mod( ...
            baseline_axis_nav_deg + 90, 360);

        % 根据导航位置和已知潜标位置计算绝对方位角
        bearing_nav_deg = calc_bearing_deg( ...
            nav_lat_rad, nav_lon_rad, ...
            beacon_lat_rad, beacon_lon_rad);

        % 相对于基线法向的几何角
        theta_nav_raw_deg = wrap180( ...
            bearing_nav_deg - baseline_normal_nav_deg);

        % 双阵元相位只能直接表示 [-90,90] 主值角
        theta_nav_deg = asind(sind(theta_nav_raw_deg));

        %% 7.3 读取该时刻、该潜标的相位差
        phase_file = fullfile(input_dir, ...
            sprintf('phase%d.txt', beacon_id));

        if ~isfile(phase_file)
            error('找不到相位数据：%s', phase_file);
        end

        phase_data = readmatrix(phase_file, 'FileType', 'text');

        [phase_time_error, phase_index] = min( ...
            abs(phase_data(:, 1) - range_time));

        if phase_time_error > 0.5 * source_interval_s
            error('找不到与测距时刻 %.3f s 同步的相位量测。', ...
                range_time);
        end

        phase_meas_deg = phase_data(phase_index, 2);

        %% 7.4 由包裹相位生成所有周期候选角
        k_limit = ceil(baseline_m / lambda) + 1;
        k_search = -k_limit:k_limit;

        phase_candidate_deg = ...
            phase_meas_deg + 360 * k_search;

        sin_theta = lambda ./ (360 * baseline_m) .* ...
            phase_candidate_deg;

        valid = abs(sin_theta) <= 1;

        candidate_k = k_search(valid);
        candidate_theta_deg = asind(sin_theta(valid));

        %% 7.5 利用导航预测角选择最近候选
        [nav_candidate_difference_deg, selected_index] = min( ...
            abs(candidate_theta_deg - theta_nav_deg));

        selected_k = candidate_k(selected_index);
        selected_theta_deg = candidate_theta_deg(selected_index);

        %% 7.6 用 truth 计算真实到达角，仅用于验证
        [truth_time_unique, truth_idx] = unique( ...
            truth(:, 2), 'stable');

        truth_unique = truth(truth_idx, :);

        truth_lat_rad = deg2rad(interp1( ...
            truth_time_unique, truth_unique(:, 3), ...
            range_time, 'linear'));

        truth_lon_rad = deg2rad(interp1( ...
            truth_time_unique, truth_unique(:, 4), ...
            range_time, 'linear'));

        truth_yaw_rad = unwrap(deg2rad(truth_unique(:, 11)));

        heading_true_deg = mod(rad2deg(interp1( ...
            truth_time_unique, truth_yaw_rad, ...
            range_time, 'linear')), 360);

        baseline_axis_true_deg = mod( ...
            heading_true_deg + baseline_install_deg, 360);

        baseline_normal_true_deg = mod( ...
            baseline_axis_true_deg + 90, 360);

        bearing_true_deg = calc_bearing_deg( ...
            truth_lat_rad, truth_lon_rad, ...
            beacon_lat_rad, beacon_lon_rad);

        theta_true_raw_deg = wrap180( ...
            bearing_true_deg - baseline_normal_true_deg);

        theta_true_deg = asind(sind(theta_true_raw_deg));

        %% 7.7 找到理论上真正对应的候选，用于判断是否选对
        [~, true_candidate_index] = min( ...
            abs(candidate_theta_deg - theta_true_deg));

        true_candidate_k = candidate_k(true_candidate_index);
        true_candidate_theta_deg = ...
            candidate_theta_deg(true_candidate_index);

        is_correct_cycle = selected_k == true_candidate_k;

        %% 7.8 输出结果
        fprintf('\n====================================================\n');
        fprintf('第一次测距点相位周期裁决\n');
        fprintf('====================================================\n');
        fprintf('测距时刻           : %.3f s\n', range_time);
        fprintf('使用潜标           : Beacon %d\n', beacon_id);
        fprintf('测量相位差         : %.3f deg\n', phase_meas_deg);
        fprintf('\n');
        fprintf('导航更新后位置     : %.8f deg, %.8f deg\n', ...
            nav_after.pos(1)*param.R2D, ...
            nav_after.pos(2)*param.R2D);
        fprintf('导航航向           : %.3f deg\n', heading_nav_deg);
        fprintf('导航计算潜标方位   : %.3f deg\n', bearing_nav_deg);
        fprintf('导航预测到达角     : %.3f deg\n', theta_nav_deg);
        fprintf('\n');
        fprintf('真实到达角         : %.3f deg\n', theta_true_deg);
        fprintf('导航到达角误差     : %.3f deg\n', ...
            theta_nav_deg - theta_true_deg);
        fprintf('\n');
        fprintf('选中候选角         : %.3f deg\n', selected_theta_deg);
        fprintf('选中周期 k         : %d\n', selected_k);
        fprintf('真实候选周期 k     : %d\n', true_candidate_k);
        fprintf('选中角误差         : %.3f deg\n', ...
            selected_theta_deg - theta_true_deg);
        fprintf('是否选对周期       : %d\n', is_correct_cycle);
        fprintf('====================================================\n');

        %% 7.9 保存第一次测距结果
        first_range_result = [ ...
            range_time, ...
            beacon_id, ...
            nav_after.pos(1)*param.R2D, ...
            nav_after.pos(2)*param.R2D, ...
            heading_nav_deg, ...
            bearing_nav_deg, ...
            theta_nav_deg, ...
            phase_meas_deg, ...
            selected_k, ...
            selected_theta_deg, ...
            theta_true_deg, ...
            true_candidate_k, ...
            is_correct_cycle];

        result_file = fullfile(input_dir, ...
            'first_range_phase_disambiguation.txt');

        writematrix(first_range_result, ...
            result_file, 'Delimiter', ' ');

        %% 7.10 保存全部候选解
        candidate_table = table( ...
            candidate_k(:), ...
            candidate_theta_deg(:), ...
            abs(candidate_theta_deg(:) - theta_nav_deg), ...
            abs(candidate_theta_deg(:) - theta_true_deg), ...
            'VariableNames', { ...
            'Cycle_k', ...
            'CandidateAngle_deg', ...
            'DiffToNav_deg', ...
            'DiffToTruth_deg'});

        disp(candidate_table);

        candidate_file = fullfile(input_dir, ...
            'first_range_phase_candidates.txt');

        writetable(candidate_table, candidate_file, ...
            'Delimiter', ' ');

        %% 7.11 可视化第一次周期裁决
        figure('Color', 'w', 'Position', [150 150 900 500]);
        hold on; grid on; box on;

        scatter(candidate_theta_deg, ...
            zeros(size(candidate_theta_deg)), ...
            35, 'filled');

        xline(theta_nav_deg, 'b--', ...
            sprintf('导航预测 %.2f^\\circ', theta_nav_deg), ...
            'LineWidth', 1.5);

        xline(theta_true_deg, 'k-', ...
            sprintf('真值 %.2f^\\circ', theta_true_deg), ...
            'LineWidth', 1.5);

        scatter(selected_theta_deg, 0, 90, ...
            'r', 'filled');

        xlabel('到达角 \theta / deg');
        yticks([]);
        ylim([-1 1]);
        xlim([-90 90]);

        title(sprintf( ...
            '第一次测距点：Beacon %d，相位候选角周期裁决', ...
            beacon_id));

        legend('相位候选角', ...
            '导航预测角', ...
            '真实角', ...
            '导航选中候选', ...
            'Location', 'best');

        % 第一次测距点处理完立即停止
        break;
    end

    %% 7.12 非测距历元正常传播
    last_state = navstate;
    navstate = InsMech(last_state, last_imu, this_imu);

    if ~is_range_epoch
        [kf, navstate] = height_update_function( ...
            kf, navstate, height(imu_index, :));
    end

    kf = propagation_function( ...
        navstate, this_imu, imu_dt, kf);
end

if isempty(first_range_result)
    error('未运行到第一次测距事件。');
end

%% ========================================================================
% 局部函数
% ========================================================================
function bearing_deg = calc_bearing_deg( ...
    lat1_rad, lon1_rad, lat2_rad, lon2_rad)
% 根据两个经纬度计算由点1指向点2的初始方位角
% 角度定义：北=0 deg，东=90 deg，顺时针为正

dlon = lon2_rad - lon1_rad;

east_component = cos(lat2_rad) .* sin(dlon);

north_component = ...
    cos(lat1_rad).*sin(lat2_rad) - ...
    sin(lat1_rad).*cos(lat2_rad).*cos(dlon);

bearing_deg = mod( ...
    atan2d(east_component, north_component), 360);
end

function angle_deg = wrap180(angle_deg)
% 将角度限制到 [-180,180)

angle_deg = mod(angle_deg + 180, 360) - 180;
end