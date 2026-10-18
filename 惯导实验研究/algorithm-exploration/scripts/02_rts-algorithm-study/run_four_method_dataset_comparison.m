clear;
close all;
clc;
%% 四方法对比后处理：前向EKF、严格2RTS、旋转收缩与位置速度重放
% 不再重复执行前向EKF或2RTS主循环。
% 请先运行 run_rts_navigation_study.m 生成前向和二次RTS结果。

%% 1. 用户配置
data_source = "experiment";
simulation_case = 'case-00';
experiment_case = '';                  % 留空时使用默认实测案例
position_error_unit = "rad";
range_interval_s = 420;
simulation_depth_noise_std_m = 0.4;
experiment_depth_std_m = 0.4;
random_seed = 1;
strict_second_rts_only = true;
overwrite_method_results = false;
enable_rotation_contraction = true;
enable_position_velocity_replay = true;
rts_update_interval_s = 1;
rts_position_std_m = 30;
rts_velocity_std_mps = 0.3;
boundary_guard_s = 60;
boundary_position_std_m = 80;
boundary_velocity_std_mps = 1.0;
innovation_gate_sigma = 3;
robust_gate_sigma = 6;
robust_std_scale = 3;

data_source_override = strtrim(getenv('KF_GINS_DATA_SOURCE'));
if ~isempty(data_source_override), data_source = string(data_source_override); end
case_name_override = strtrim(getenv('KF_GINS_CASE_NAME'));
if ~isempty(case_name_override)
    if lower(data_source) == "simulation"
        simulation_case = char(case_name_override);
    else
        experiment_case = char(case_name_override);
    end
end
strict_override = str2double(getenv('KF_GINS_STRICT_SECOND_RTS'));
if isfinite(strict_override)
    strict_second_rts_only = strict_override ~= 0;
end
rotation_override = str2double(getenv('KF_GINS_ENABLE_ROTATION'));
if isfinite(rotation_override)
    enable_rotation_contraction = rotation_override ~= 0;
end
replay_override = str2double(getenv('KF_GINS_ENABLE_REPLAY'));
if isfinite(replay_override)
    enable_position_velocity_replay = replay_override ~= 0;
end

%% 2. 定位原始2RTS结果
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
paths = setup_inertial_experiment();
if exist('glvs', 'file') ~= 2
    psins_root = fullfile('D:\Github\PSINS', 'psins2401');
    if ~isfolder(psins_root), error('找不到PSINS工具箱：%s', psins_root); end
    addpath(genpath(psins_root));
end
param = Param();
glvs;
data_source = lower(data_source);
position_error_unit = lower(position_error_unit);
if ~ismember(data_source, ["simulation", "experiment"])
    error('data_source只能设置为"simulation"或"experiment"。');
end
if ~ismember(position_error_unit, ["rad", "m"])
    error('position_error_unit只能设置为"rad"或"m"。');
end
if data_source == "simulation"
    case_name = char(simulation_case);
    input_dir = paths.simulation_input(str2double(case_name(end)));
else
    if isempty(experiment_case), case_name = paths.default_experiment_case;
    else, case_name = char(experiment_case); end
    input_dir = paths.experiment_input(str2double(case_name(end)));
end
cfg = load_algorithm_exploration_config(data_source, position_error_unit, input_dir);
output_dir = fullfile(cfg.outputfolder, sprintf('simple-ekf-rts-%s', position_error_unit));
forward_path = fullfile(output_dir, sprintf('simple-forward-ekf-%s.nav', position_error_unit));
double_rts_path = fullfile(output_dir, sprintf('simple-rts-double-%s.nav', position_error_unit));
strict_double_rts_path = fullfile(output_dir, sprintf('four-method-rts-double-strict-%s.nav', position_error_unit));
rotation_path = fullfile(output_dir, sprintf('four-method-rts-double-rotation-%s.nav', position_error_unit));
replay_path = fullfile(output_dir, sprintf('four-method-rts-double-position-velocity-%s.nav', position_error_unit));
if ~nav_file_is_ready(forward_path) || ~nav_file_is_ready(double_rts_path)
    error(['缺少前向EKF或原始2RTS结果；本脚本不再重复运行导航核心。\n' ...
        '请先运行 run_rts_navigation_study.m。\n前向：%s\n2RTS：%s'], forward_path, double_rts_path);
end

%% 3. 从原始2RTS构造严格评价口径
forward_nav = readmatrix(forward_path, 'FileType', 'text');
double_rts_nav = readmatrix(double_rts_path, 'FileType', 'text');
validate_nav_matrix(forward_nav, '前向EKF');
validate_nav_matrix(double_rts_nav, '二次RTS');
if strict_second_rts_only
    final_segment_id = double_rts_nav(end, 1);
    strict_mask = double_rts_nav(:, 1) < final_segment_id;
    if ~any(strict_mask), error('原始2RTS中没有足够的完整二次平滑区间。'); end
    double_rts_nav = double_rts_nav(strict_mask, :);
    if overwrite_method_results || ...
            ~method_result_is_current(strict_double_rts_path, double_rts_path)
        write_nav_file(strict_double_rts_path, double_rts_nav);
    end
    comparison_double_path = strict_double_rts_path;
else
    comparison_double_path = double_rts_path;
end

%% 4. 旋转收缩
if enable_rotation_contraction
    if ~overwrite_method_results && method_result_is_current(rotation_path, comparison_double_path)
        fprintf('旋转收缩结果已存在且为最新：%s\n', rotation_path);
    else
        rotation_nav = double_rts_nav;
        segment_ids = unique(double_rts_nav(:, 1), 'stable');
        for segment_index = 1:max(0, numel(segment_ids)-1)
            current_mask = double_rts_nav(:, 1) == segment_ids(segment_index);
            next_mask = double_rts_nav(:, 1) == segment_ids(segment_index+1);
            target_position = nav_row_to_position(double_rts_nav(find(next_mask,1),:), param);
            rotation_nav(current_mask,:) = rotate_double_rts_segment(double_rts_nav(current_mask,:), target_position, param);
        end
        write_nav_file(rotation_path, rotation_nav);
    end
end

%% 5. 位置速度约束重放
if enable_position_velocity_replay
    if ~overwrite_method_results && method_result_is_current(replay_path, comparison_double_path)
        fprintf('位置速度重放结果已存在且为最新：%s\n', replay_path);
    else
        final_segment_id = double_rts_nav(end, 1);
        replay_reference_nav = double_rts_nav(double_rts_nav(:,1)<final_segment_id,:);
        if isempty(replay_reference_nav), error('严格2RTS区间不足，无法重放。'); end
        imu_all = readmatrix(cfg.imufilepath, 'FileType', 'text');
        replay_start_time = replay_reference_nav(1,2);
        replay_end_time = replay_reference_nav(end,2);
        imu_mask = imu_all(:,1)>=replay_start_time-1e-8 & imu_all(:,1)<=replay_end_time+1e-8;
        replay_imu = imu_all(imu_mask,:);
        if size(replay_imu,1)~=size(replay_reference_nav,1) || max(abs(replay_imu(:,1)-replay_reference_nav(:,2)))>1e-7
            error('历史IMU与二次RTS时间轴不一致，不能安全重放。');
        end
        uses_generated_measurements = data_source=="simulation" || (data_source=="experiment" && strcmp(case_name,'case-06'));
        if uses_generated_measurements
            truth_all = readmatrix(cfg.truthpath, 'FileType', 'text');
            rng(random_seed,'twister');
            sources={readmatrix(cfg.rangefile1path,'FileType','text'),readmatrix(cfg.rangefile2path,'FileType','text'),readmatrix(cfg.rangefile3path,'FileType','text')};
            source_dt=median(diff(sources{1}(:,1))); stride=round(range_interval_s/source_dt);
            for i=1:numel(sources), sources{i}=sources{i}(stride:stride:end,:); end
            event_count=min(cellfun(@(x) size(x,1),sources));
            consume_range_noise_draws(event_count);
            htruth=interp1(truth_all(:,2),truth_all(:,5),replay_imu(:,1),'linear','extrap');
            if data_source=="simulation", hstd=simulation_depth_noise_std_m; else, hstd=experiment_depth_std_m; end
            replay_height=[replay_imu(:,1),htruth+hstd*randn(size(htruth))];
        else
            hs=readmatrix(cfg.heightfilepath,'FileType','text');
            replay_height=[replay_imu(:,1),interp1(hs(:,1),hs(:,2),replay_imu(:,1),'linear','extrap')];
        end
        replay_options=struct('position_error_unit',position_error_unit,'filter_depth_std_m',experiment_depth_std_m, ...
            'range_interval_s',range_interval_s,'rts_update_interval_s',rts_update_interval_s, ...
            'rts_position_std_m',rts_position_std_m,'rts_velocity_std_mps',rts_velocity_std_mps, ...
            'boundary_guard_s',boundary_guard_s,'boundary_position_std_m',boundary_position_std_m, ...
            'boundary_velocity_std_mps',boundary_velocity_std_mps,'innovation_gate_sigma',innovation_gate_sigma, ...
            'robust_gate_sigma',robust_gate_sigma,'robust_std_scale',robust_std_scale);
        replay_nav=replay_position_velocity(replay_imu,replay_height,replay_reference_nav,cfg,replay_options,param);
        write_nav_file(replay_path,replay_nav);
    end
end

fprintf('\n四方法后处理结束。\n');
fprintf('方法1 前向EKF：%s\n',forward_path);
fprintf('方法2 严格2RTS：%s\n',comparison_double_path);
if enable_rotation_contraction, fprintf('方法3 旋转收缩：%s\n',rotation_path); end
if enable_position_velocity_replay, fprintf('方法4 位置速度约束：%s\n',replay_path); end

%% 局部辅助函数
function ready = nav_file_is_ready(file_path)
    file_info = dir(file_path);
    ready = isfile(file_path) && ~isempty(file_info) && file_info.bytes > 0;
end

function current = method_result_is_current(result_path, source_path)
    if ~nav_file_is_ready(result_path) || ~nav_file_is_ready(source_path)
        current = false;
        return;
    end
    result_info = dir(result_path);
    source_info = dir(source_path);
    current = result_info.datenum >= source_info.datenum;
end

function validate_nav_matrix(nav_data, label)
    if isempty(nav_data) || size(nav_data, 2) < 11 || ...
            any(~isfinite(nav_data(:, 2))) || any(diff(nav_data(:, 2)) <= 0)
        error('%s结果为空、列数不足或时间轴不严格递增。', label);
    end
end

function write_nav_file(file_path, nav_data)
    file_id = fopen(file_path, 'wt');
    if file_id < 0
        error('无法创建导航结果：%s', file_path);
    end
    cleaner = onCleanup(@() fclose(file_id));
    format = ['%2d %12.6f %12.8f %12.8f %8.4f %8.4f ', ...
        '%8.4f %8.4f %8.4f %8.4f %8.4f\n'];
    fprintf(file_id, format, nav_data');
    clear cleaner;
end

function position = nav_row_to_position(nav_row, param)
    position = [nav_row(3:4)' * param.D2R; nav_row(5)];
end

function rotated_nav = rotate_double_rts_segment( ...
        double_rts_nav, target_position, param)
    position = [double_rts_nav(:, 3:4)' * param.D2R; ...
        double_rts_nav(:, 5)'];
    position(3, :) = 0;
    target_position(3) = 0;
    rotated_position = rotateAndScaleTrajectory(position, target_position);
    rotated_nav = double_rts_nav;
    rotated_nav(:, 3:4) = rotated_position(1:2, :)' * param.R2D;
end

function replay_nav = replay_position_velocity( ...
        imudata, height, reference_nav, cfg, options, param)
    cfg.initpos(1:2) = reference_nav(1, 3:4)' * param.D2R;
    cfg.initpos(3) = height(1, 2);
    cfg.initvel = reference_nav(1, 6:8)';
    [kf, navstate] = myInitialize_15state(cfg);
    kf.depthstd = options.filter_depth_std_m;
    horizontal_scale = [navstate.Rm + navstate.pos(3); ...
        (navstate.Rn + navstate.pos(3)) * cos(navstate.pos(1))];
    if options.position_error_unit == "rad"
        kf.P(1:2, 1:2) = diag((options.rts_position_std_m ...
            ./ horizontal_scale) .^ 2);
    else
        kf.P(1:2, 1:2) = eye(2) * options.rts_position_std_m ^ 2;
    end
    kf.P(4:5, 4:5) = eye(2) * options.rts_velocity_std_mps ^ 2;

    sample_count = size(imudata, 1);
    replay_nav = nan(sample_count, 11);
    navstate.time = imudata(1, 1);
    navstate.pos(3) = height(1, 2);
    replay_nav(1, :) = state_to_nav_row(navstate, param);
    next_update_time = ceil(imudata(1, 1) / ...
        options.rts_update_interval_s) * options.rts_update_interval_s;
    if next_update_time <= imudata(1, 1) + 1e-9
        next_update_time = next_update_time + options.rts_update_interval_s;
    end
    this_imu = imudata(1, :)';
    last_progress = -1;
    for imu_index = 2:sample_count
        last_imu = this_imu;
        this_imu = imudata(imu_index, :)';
        imu_dt = this_imu(1) - last_imu(1);
        time_tolerance = max(1e-8, abs(imu_dt) * 0.25);

        if last_imu(1) >= next_update_time - time_tolerance
            measurement_nav = interp1(reference_nav(:, 2), ...
                reference_nav(:, 3:7), last_imu(1), 'linear');
            [position_std_m, velocity_std_mps] = ...
                select_replay_measurement_std(last_imu(1), options);
            [kf, accepted] = update_replay_position_velocity( ...
                navstate, measurement_nav(1:2)' * param.D2R, ...
                measurement_nav(4:5)', kf, position_std_m, ...
                velocity_std_mps, options);
            if accepted
                if options.position_error_unit == "rad"
                    [kf, navstate] = myErrorFeedback_range(kf, navstate);
                else
                    [kf, navstate] = myErrorFeedback_range_m(kf, navstate);
                end
                replay_nav(imu_index - 1, :) = state_to_nav_row(navstate, param);
            end
            next_update_time = next_update_time + options.rts_update_interval_s;
        end

        navstate = InsMech(navstate, last_imu, this_imu);
        if options.position_error_unit == "rad"
            [kf, navstate] = update_decoupled_height( ...
                kf, navstate, height(imu_index, :));
            navstate.time = this_imu(1);
            kf = myInsPropagate_15state(navstate, this_imu, imu_dt, kf);
        else
            [kf, navstate] = update_decoupled_height_m( ...
                kf, navstate, height(imu_index, :));
            navstate.time = this_imu(1);
            kf = myInsPropagate_15state_m(navstate, this_imu, imu_dt, kf);
        end
        replay_nav(imu_index, :) = state_to_nav_row(navstate, param);

        progress = floor(10 * imu_index / sample_count) * 10;
        if progress > last_progress && mod(progress, 20) == 0
            fprintf('方法4重放进度：%d %%\n', progress);
            last_progress = progress;
        end
    end
end

function [position_std_m, velocity_std_mps] = ...
        select_replay_measurement_std(time_s, options)
    distance_to_boundary = abs(time_s - round(time_s ...
        / options.range_interval_s) * options.range_interval_s);
    in_boundary_guard = distance_to_boundary <= options.boundary_guard_s ...
        && time_s > options.boundary_guard_s;
    if in_boundary_guard
        normalized_distance = distance_to_boundary / options.boundary_guard_s;
        smooth_weight = 10 * normalized_distance ^ 3 ...
            - 15 * normalized_distance ^ 4 + 6 * normalized_distance ^ 5;
        position_std_m = options.boundary_position_std_m + ...
            (options.rts_position_std_m - options.boundary_position_std_m) ...
            * smooth_weight;
        velocity_std_mps = options.boundary_velocity_std_mps + ...
            (options.rts_velocity_std_mps ...
            - options.boundary_velocity_std_mps) * smooth_weight;
    else
        position_std_m = options.rts_position_std_m;
        velocity_std_mps = options.rts_velocity_std_mps;
    end
end

function [kf, accepted] = update_replay_position_velocity( ...
        navstate, measurement_position, measurement_velocity, kf, ...
        position_std_m, velocity_std_mps, options)
    horizontal_scale = [navstate.Rm + navstate.pos(3); ...
        (navstate.Rn + navstate.pos(3)) * cos(navstate.pos(1))];
    position_residual = navstate.pos(1:2) - measurement_position;
    if options.position_error_unit == "m"
        position_residual = horizontal_scale .* position_residual;
    end
    measurement_residual = [position_residual; ...
        navstate.vel(1:2) - measurement_velocity];
    measurement_matrix = zeros(4, kf.RANK);
    measurement_matrix(1:2, 1:2) = eye(2);
    measurement_matrix(3:4, 4:5) = eye(2);
    innovation = measurement_residual - measurement_matrix * kf.x;
    if options.position_error_unit == "m"
        innovation_m = norm(innovation(1:2));
    else
        innovation_m = norm(horizontal_scale .* innovation(1:2));
    end
    velocity_innovation_mps = norm(innovation(3:4));
    normalized_innovation = max( ...
        innovation_m / max(position_std_m, eps), ...
        velocity_innovation_mps / max(velocity_std_mps, eps));
    accepted = normalized_innovation <= options.robust_gate_sigma;
    robust_downweighted = accepted && ...
        normalized_innovation > options.innovation_gate_sigma;
    if robust_downweighted
        position_std_m = position_std_m * options.robust_std_scale;
        velocity_std_mps = velocity_std_mps * options.robust_std_scale;
    end
    if accepted
        if options.position_error_unit == "m"
            measurement_std = repmat(position_std_m, 2, 1);
        else
            measurement_std = position_std_m ./ horizontal_scale;
        end
        measurement_covariance = diag([measurement_std .^ 2; ...
            repmat(velocity_std_mps ^ 2, 2, 1)]);
        innovation_covariance = measurement_matrix * kf.P ...
            * measurement_matrix' + measurement_covariance;
        kalman_gain = kf.P * measurement_matrix' / innovation_covariance;
        kf.x = kf.x + kalman_gain * innovation;
        identity = eye(kf.RANK);
        kf.P = (identity - kalman_gain * measurement_matrix) * kf.P ...
            * (identity - kalman_gain * measurement_matrix)' ...
            + kalman_gain * measurement_covariance * kalman_gain';
        kf.P = (kf.P + kf.P') / 2;
    end
end

function consume_range_noise_draws(event_count)
    discarded = randn(event_count, 1);
    if numel(discarded) ~= event_count
        error('测距噪声随机序列长度异常。');
    end
end

function row = state_to_nav_row(navstate, param)
    row = zeros(1, 11);
    row(2) = navstate.time;
    row(3:5) = [navstate.pos(1:2)' * param.R2D, navstate.pos(3)];
    row(6:8) = navstate.vel';
    row(9:11) = navstate.att' * param.R2D;
end

