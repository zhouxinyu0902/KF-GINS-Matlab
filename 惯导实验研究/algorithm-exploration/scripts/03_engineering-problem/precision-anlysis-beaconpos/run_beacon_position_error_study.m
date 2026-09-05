clear;
close all;
clc;
%% 潜标位置误差专题：完整导航执行入口
% 本脚本只读取 generate_beacon_position_error_data.m 生成的输入并产生
% 导航结果，不重新生成误差数据，也不执行结果评价。

%% 1. 用户配置
data_source = "experiment";         % "simulation" 或 "experiment"
simulation_case = 'case-00';        % 仅仿真使用
position_error_units = ["rad"];    % 可设为 ["rad", "m"]
duration_s = 4621;
filter_range_std_m = 6;
filter_depth_std_m = 0.4;
enable_feedback = true;
enable_smoothing = true;
enable_second_rts = true;

%% 2. 初始化路径并读取已经生成的输入清单
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));
addpath(topic_dir);
paths = setup_inertial_experiment();
param = Param();
glvs;

data_source = lower(data_source);
position_error_units = lower(string(position_error_units));
if ~ismember(data_source, ["simulation", "experiment"])
    error('data_source 只能设置为 "simulation" 或 "experiment"。');
end
if any(~ismember(position_error_units, ["rad", "m"])) || ...
        numel(unique(position_error_units)) ~= numel(position_error_units)
    error('position_error_units 只能包含不重复的 "rad" 或 "m"。');
end
if enable_smoothing && ~enable_feedback
    error('执行 RTS 时必须启用 enable_feedback。');
end
if enable_second_rts && ~enable_smoothing
    error('enable_second_rts=true 时必须启用 enable_smoothing。');
end

input_directories = engineering_problem_directories(paths, data_source, ...
    simulation_case, 'beacon-position-error', 'rad');
manifest_path = fullfile(input_directories.input_root, 'input-manifest.csv');
generation_context_path = fullfile(input_directories.input_root, ...
    'generation-context.mat');
if ~isfile(manifest_path) || ~isfile(generation_context_path)
    error(['缺少潜标位置误差输入清单或生成配置。请先运行 ', ...
        'generate_beacon_position_error_data.m。\n输入目录：%s'], ...
        input_directories.input_root);
end
manifest = readtable(manifest_path, 'Delimiter', ',', 'TextType', 'string');
required_columns = {'DataSource', 'DatasetId', 'ScenarioId', ...
    'EastError_m', 'NorthError_m', 'UpError_m', ...
    'HorizontalError_m', 'RangeFilePath', 'HeightFilePath'};
if ~all(ismember(required_columns, manifest.Properties.VariableNames))
    error(['输入清单版本过旧或字段不完整。请重新运行 ', ...
        'generate_beacon_position_error_data.m。']);
end
if isempty(manifest)
    error('潜标位置误差输入清单为空：%s', manifest_path);
end
if any(lower(string(manifest.DataSource)) ~= data_source)
    error('输入清单的数据来源与 data_source 不一致。');
end
loaded_generation = load(generation_context_path, 'generation_context');
if ~isfield(loaded_generation, 'generation_context')
    error('generation-context.mat 中缺少 generation_context。');
end
generation_context = loaded_generation.generation_context;
range_interval_s = generation_context.range_interval_s;

%% 3. rad/m 分别执行相同的导航架构
for position_error_unit = position_error_units
    fprintf('\n======================================================\n');
    fprintf('开始执行潜标位置误差专题：%s / %s\n', ...
        data_source, position_error_unit);
    fprintf('======================================================\n');

    if data_source == "simulation"
        input_dir = paths.simulation_input(parse_case_id(simulation_case));
        cfg = load_algorithm_exploration_config( ...
            "simulation", position_error_unit, input_dir);
        source_cfg = load_algorithm_exploration_config( ...
            "simulation", "rad", input_dir);
    else
        cfg = load_algorithm_exploration_config( ...
            "experiment", position_error_unit, []);
        source_cfg = load_algorithm_exploration_config( ...
            "experiment", "rad", []);
    end
    % rad/m 只改变误差状态定义和滤波函数；两者必须读取同一套当前数据。
    cfg = use_shared_input_paths(cfg, source_cfg);
    directories = engineering_problem_directories(paths, data_source, ...
        simulation_case, 'beacon-position-error', position_error_unit);

    imudata_all = readmatrix(cfg.imufilepath, 'FileType', 'text');
    truth = readmatrix(cfg.truthpath, 'FileType', 'text');
    height_path = char(manifest.HeightFilePath(1));
    if ~isfile(height_path)
        error('缺少生成阶段的深度输入：%s', height_path);
    end
    height_all = readmatrix(height_path, 'FileType', 'text');

    start_time = max([cfg.starttime, imudata_all(1, 1), ...
        truth(1, 2), height_all(1, 1)]);
    end_time = min([start_time+duration_s, cfg.endtime, ...
        imudata_all(end, 1), truth(end, 2), height_all(end, 1)]);
    cfg.starttime = start_time;
    cfg.endtime = end_time;
    imudata = imudata_all(imudata_all(:, 1) >= start_time & ...
        imudata_all(:, 1) <= end_time, :);
    heightdata = height_all(height_all(:, 1) >= start_time & ...
        height_all(:, 1) <= end_time, :);
    if size(imudata, 1) < 2 || isempty(heightdata)
        error('当前时间范围内的 IMU 或深度输入不足。');
    end

    [~, truth_name, truth_ext] = fileparts(cfg.truthpath);
    truth_copy_path = fullfile(directories.result_root, ...
        [truth_name, truth_ext]);
    [copy_ok, copy_message] = copyfile(cfg.truthpath, truth_copy_path, 'f');
    if ~copy_ok
        error('truth 文件复制失败：%s', copy_message);
    end

    method_ids = ["ekf"; "single-rts"; "double-rts"];
    method_names = ["前向EKF"; "一次RTS"; "二次RTS"];
    method_files = ["range-ins-forward.nav"; ...
        "range-ins-rts-single.nav"; "range-ins-rts-double.nav"];
    method_enabled = [true; enable_smoothing; ...
        enable_smoothing && enable_second_rts];
    method_ids = method_ids(method_enabled);
    method_names = method_names(method_enabled);
    method_files = method_files(method_enabled);

    scenario_count = height(manifest);
    output_dirs = strings(scenario_count, 1);
    valid_masks = cell(scenario_count, numel(method_files));

    %% 3.1 每个潜标误差工况运行同一套完整主循环
    for scenario_index = 1:scenario_count
        scenario_id = string(manifest.ScenarioId(scenario_index));
        range_path = char(manifest.RangeFilePath(scenario_index));
        if ~isfile(range_path)
            error('缺少工况 %s 的距离输入：%s', scenario_id, range_path);
        end
        rangedata_all = readmatrix(range_path, 'FileType', 'text');
        rangedata = rangedata_all(rangedata_all(:, 1) >= start_time & ...
            rangedata_all(:, 1) <= end_time, :);
        if isempty(rangedata)
            error('工况 %s 在当前时间范围内没有测距事件。', scenario_id);
        end

        nav_out_dir = fullfile(directories.result_root, scenario_id);
        if ~isfolder(nav_out_dir), mkdir(nav_out_dir); end
        output_dirs(scenario_index) = nav_out_dir;

        fprintf('\n[%s] 工况 %d/%d：%s\n', position_error_unit, ...
            scenario_index, scenario_count, scenario_id);
        result_paths = run_one_beacon_position_case(cfg, imudata, ...
            rangedata, heightdata, nav_out_dir, position_error_unit, ...
            range_interval_s, filter_range_std_m, filter_depth_std_m, ...
            enable_feedback, enable_smoothing, enable_second_rts, param);

        selected_paths = strings(numel(method_files), 1);
        selected_paths(1) = result_paths.forward;
        next_method = 2;
        if enable_smoothing
            selected_paths(next_method) = result_paths.single_rts;
            next_method = next_method+1;
        end
        if enable_smoothing && enable_second_rts
            selected_paths(next_method) = result_paths.double_rts;
        end
        valid_masks(scenario_index, :) = ...
            normalize_navigation_results(selected_paths);
    end

    %% 3.2 保存评价阶段所需的唯一上下文
    study_context = struct();
    study_context.version = 4;
    study_context.data_source = char(data_source);
    study_context.dataset_id = char(string(manifest.DatasetId(1)));
    study_context.study_id = "beacon-position-error";
    study_context.study_title = data_source_name(data_source) + ...
        "潜标位置误差敏感性";
    study_context.position_error_unit = char(position_error_unit);
    study_context.scenario_ids = string(manifest.ScenarioId);
    study_context.scenario_names = compose('水平误差 %.1f m', ...
        manifest.HorizontalError_m);
    study_context.parameter_values = manifest.HorizontalError_m;
    study_context.parameter_name = "HorizontalBeaconPositionError_m";
    study_context.parameter_label = "潜标水平位置误差（m）";
    study_context.output_dirs = output_dirs;
    study_context.truth_path = truth_copy_path;
    study_context.method_ids = method_ids;
    study_context.method_names = method_names;
    study_context.method_files = method_files;
    study_context.valid_masks = valid_masks;
    study_context.artifact_root = directories.artifact_root;
    study_context.input_manifest_path = manifest_path;
    study_context.generation_context_path = generation_context_path;
    study_context.reference_script = ...
        'scripts/02_rts-algorithm-study/run_rts_navigation_study.m';
    save(fullfile(directories.result_root, 'study-context.mat'), ...
        'study_context');
    fprintf('\n%s 结果已完成：%s\n', ...
        position_error_unit, directories.result_root);
end

disp('全部导航计算完成。需要评价时请运行 evaluate_beacon_position_error_results.m。');

function result_paths = run_one_beacon_position_case(cfg, imudata, ...
        rangedata, heightdata, output_dir, position_error_unit, ...
        range_interval_s, range_std_m, depth_std_m, enable_feedback, ...
        enable_smoothing, enable_second_rts, param)
%RUN_ONE_BEACON_POSITION_CASE 单工况完整 INS/ES-EKF/RTS 主循环。
%   导航顺序和缓存含义与 run_rts_navigation_study.m 保持一致。

    cfg.userange = enable_feedback;
    cfg.outputfolder = output_dir;
    if position_error_unit == "rad"
        range_update_function = @myRangeUpdate;
        feedback_function = @myErrorFeedback_range;
        height_update_function = @update_decoupled_height;
        propagation_function = @myInsPropagate_15state;
    else
        range_update_function = @myRangeUpdate_m;
        feedback_function = @myErrorFeedback_range_m;
        height_update_function = @update_decoupled_height_m;
        propagation_function = @myInsPropagate_15state_m;
    end

    result_paths.pure_ins = string(fullfile(output_dir, 'pure-ins.nav'));
    result_paths.forward = string(fullfile(output_dir, ...
        'range-ins-forward.nav'));
    result_paths.single_rts = string(fullfile(output_dir, ...
        'range-ins-rts-single.nav'));
    result_paths.double_rts = string(fullfile(output_dir, ...
        'range-ins-rts-double.nav'));
    nav_format = ['%2d %12.6f %12.8f %12.8f %8.4f %8.4f ', ...
        '%8.4f %8.4f %8.4f %8.4f %8.4f\n'];

    pure_ins_fp = fopen(result_paths.pure_ins, 'wt');
    forward_fp = fopen(result_paths.forward, 'wt');
    single_rts_fp = -1;
    double_rts_fp = -1;
    if pure_ins_fp < 0 || forward_fp < 0
        close_nav_files([pure_ins_fp, forward_fp]);
        error('无法创建导航结果：%s', output_dir);
    end
    if enable_smoothing
        single_rts_fp = fopen(result_paths.single_rts, 'wt');
        if single_rts_fp < 0
            close_nav_files([pure_ins_fp, forward_fp]);
            error('无法创建一次 RTS 结果：%s', output_dir);
        end
    end
    if enable_smoothing && enable_second_rts
        double_rts_fp = fopen(result_paths.double_rts, 'wt');
        if double_rts_fp < 0
            close_nav_files([pure_ins_fp, forward_fp, single_rts_fp]);
            error('无法创建二次 RTS 结果：%s', output_dir);
        end
    end
    output_cleanup = onCleanup(@() close_nav_files( ...
        [pure_ins_fp, forward_fp, single_rts_fp, double_rts_fp]));

    [kf, navstate] = myInitialize_15state(cfg);
    kf.rangstd = range_std_m;
    kf.depthstd = depth_std_m;
    pure_ins_navstate = navstate;
    this_imu = imudata(1, :)';
    range_index = find(rangedata(:, 1) >= this_imu(1), 1, 'first');
    if isempty(range_index)
        error('当前 IMU 时间范围内没有可用测距观测。');
    end

    imu_interval_s = median(diff(imudata(:, 1)));
    maximum_segment_samples = ceil(range_interval_s/imu_interval_s)+20;
    state_buffer = zeros(maximum_segment_samples, 10);
    corrected_covariance_buffer = zeros(maximum_segment_samples, 225);
    predicted_covariance_buffer = zeros(maximum_segment_samples, 225);
    transition_buffer = zeros(maximum_segment_samples, 225);
    buffer_index = 1;
    previous_single_state = [];
    previous_single_nav = [];
    previous_corrected_covariance = [];
    previous_predicted_covariance = [];
    previous_transition = [];
    previous_range_index = 0;
    last_progress = -1;
    tic;

    %% 核心主循环：与 run_rts_navigation_study.m 的导航顺序一致
    for imu_index = 2:size(imudata, 1)
        last_imu = this_imu;
        this_imu = imudata(imu_index, :)';
        imu_dt = this_imu(1)-last_imu(1);
        time_tolerance = max(1e-8, abs(imu_dt)*0.25);

        % 纯惯导链独立传播，不接受距离或深度反馈。
        pure_ins_navstate = InsMech( ...
            pure_ins_navstate, last_imu, this_imu);

        while range_index <= size(rangedata, 1) && ...
                rangedata(range_index, 1) < last_imu(1)-time_tolerance
            range_index = range_index+1;
        end
        has_range = range_index <= size(rangedata, 1);
        range_at_last_imu = has_range && ...
            abs(last_imu(1)-rangedata(range_index, 1)) <= time_tolerance;
        range_inside_interval = has_range && ...
            rangedata(range_index, 1) > last_imu(1)+time_tolerance && ...
            rangedata(range_index, 1) < this_imu(1)-time_tolerance;

        if range_at_last_imu && cfg.userange
            %% 测距与上一 IMU 历元重合：先更新，再向前传播
            kf = range_update_function(navstate, ...
                rangedata(range_index, :), heightdata(imu_index-1, :), kf);
            terminal_error = kf.x;

            if enable_smoothing && buffer_index > 1
                valid_length = buffer_index-1;
                current_state = state_buffer(1:valid_length, :);
                current_corrected_covariance = ...
                    corrected_covariance_buffer(1:valid_length, :);
                current_predicted_covariance = ...
                    predicted_covariance_buffer(1:valid_length, :);
                current_transition = transition_buffer(1:valid_length, :);
                [single_nav, bridge_error, single_state] = ...
                    perform_unified_smoothing(current_state, ...
                    terminal_error, param, range_index, 'RTS', ...
                    char(position_error_unit), ...
                    current_corrected_covariance, ...
                    current_predicted_covariance, current_transition);
                fprintf(single_rts_fp, nav_format, single_nav);

                if enable_second_rts
                    if isempty(previous_single_state)
                        previous_single_state = single_state;
                        previous_single_nav = single_nav;
                        previous_corrected_covariance = ...
                            current_corrected_covariance;
                        previous_predicted_covariance = ...
                            current_predicted_covariance;
                        previous_transition = current_transition;
                        previous_range_index = range_index;
                    else
                        double_nav = perform_unified_smoothing( ...
                            previous_single_state, bridge_error, param, ...
                            previous_range_index, 'RTS', ...
                            char(position_error_unit), ...
                            previous_corrected_covariance, ...
                            previous_predicted_covariance, ...
                            previous_transition);
                        fprintf(double_rts_fp, nav_format, double_nav);
                        previous_single_state = single_state;
                        previous_single_nav = single_nav;
                        previous_corrected_covariance = ...
                            current_corrected_covariance;
                        previous_predicted_covariance = ...
                            current_predicted_covariance;
                        previous_transition = current_transition;
                        previous_range_index = range_index;
                    end
                end
                buffer_index = 1;
                state_buffer(:) = 0;
                corrected_covariance_buffer(:) = 0;
                predicted_covariance_buffer(:) = 0;
                transition_buffer(:) = 0;
            end

            [kf, navstate] = feedback_function(kf, navstate);
            range_index = range_index+1;
            assert_buffer_capacity(buffer_index, maximum_segment_samples);
            if enable_smoothing
                corrected_covariance_buffer(buffer_index, :) = kf.P(:)';
            end
            navstate = InsMech(navstate, last_imu, this_imu);
            kf = propagation_function(navstate, this_imu, imu_dt, kf);
            if enable_smoothing
                state_buffer(buffer_index, :) = navigation_state_row(navstate);
                predicted_covariance_buffer(buffer_index, :) = kf.P(:)';
                transition_buffer(buffer_index, :) = kf.phi(:)';
                buffer_index = buffer_index+1;
            end

        elseif range_inside_interval && cfg.userange
            %% 测距位于两个 IMU 历元之间：拆分增量后精确更新
            range_time = rangedata(range_index, 1);
            [first_imu, second_imu] = interpolate( ...
                last_imu, this_imu, range_time);
            first_dt = first_imu(1)-last_imu(1);
            assert_buffer_capacity(buffer_index, maximum_segment_samples);
            if enable_smoothing
                corrected_covariance_buffer(buffer_index, :) = kf.P(:)';
            end
            navstate = InsMech(navstate, last_imu, first_imu);
            kf = propagation_function(navstate, first_imu, first_dt, kf);
            if enable_smoothing
                state_buffer(buffer_index, :) = navigation_state_row(navstate);
                predicted_covariance_buffer(buffer_index, :) = kf.P(:)';
                transition_buffer(buffer_index, :) = kf.phi(:)';
                buffer_index = buffer_index+1;
            end

            range_height = [range_time, interp1(heightdata(:, 1), ...
                heightdata(:, 2), range_time, 'linear', 'extrap')];
            kf = range_update_function(navstate, ...
                rangedata(range_index, :), range_height, kf);
            terminal_error = kf.x;

            if enable_smoothing && buffer_index > 1
                valid_length = buffer_index-1;
                current_state = state_buffer(1:valid_length, :);
                current_corrected_covariance = ...
                    corrected_covariance_buffer(1:valid_length, :);
                current_predicted_covariance = ...
                    predicted_covariance_buffer(1:valid_length, :);
                current_transition = transition_buffer(1:valid_length, :);
                [single_nav, bridge_error, single_state] = ...
                    perform_unified_smoothing(current_state, ...
                    terminal_error, param, range_index, 'RTS', ...
                    char(position_error_unit), ...
                    current_corrected_covariance, ...
                    current_predicted_covariance, current_transition);
                fprintf(single_rts_fp, nav_format, single_nav);

                if enable_second_rts
                    if isempty(previous_single_state)
                        previous_single_state = single_state;
                        previous_single_nav = single_nav;
                        previous_corrected_covariance = ...
                            current_corrected_covariance;
                        previous_predicted_covariance = ...
                            current_predicted_covariance;
                        previous_transition = current_transition;
                        previous_range_index = range_index;
                    else
                        double_nav = perform_unified_smoothing( ...
                            previous_single_state, bridge_error, param, ...
                            previous_range_index, 'RTS', ...
                            char(position_error_unit), ...
                            previous_corrected_covariance, ...
                            previous_predicted_covariance, ...
                            previous_transition);
                        fprintf(double_rts_fp, nav_format, double_nav);
                        previous_single_state = single_state;
                        previous_single_nav = single_nav;
                        previous_corrected_covariance = ...
                            current_corrected_covariance;
                        previous_predicted_covariance = ...
                            current_predicted_covariance;
                        previous_transition = current_transition;
                        previous_range_index = range_index;
                    end
                end
                buffer_index = 1;
                state_buffer(:) = 0;
                corrected_covariance_buffer(:) = 0;
                predicted_covariance_buffer(:) = 0;
                transition_buffer(:) = 0;
            end

            [kf, navstate] = feedback_function(kf, navstate);
            range_index = range_index+1;
            second_dt = second_imu(1)-first_imu(1);
            assert_buffer_capacity(buffer_index, maximum_segment_samples);
            if enable_smoothing
                corrected_covariance_buffer(buffer_index, :) = kf.P(:)';
            end
            navstate = InsMech(navstate, first_imu, second_imu);
            kf = propagation_function(navstate, second_imu, second_dt, kf);
            if enable_smoothing
                state_buffer(buffer_index, :) = navigation_state_row(navstate);
                predicted_covariance_buffer(buffer_index, :) = kf.P(:)';
                transition_buffer(buffer_index, :) = kf.phi(:)';
                buffer_index = buffer_index+1;
            end

        else
            %% 普通历元：惯导、深度更新和误差传播
            navstate = InsMech(navstate, last_imu, this_imu);
            [kf, navstate] = height_update_function( ...
                kf, navstate, heightdata(imu_index, :));
            assert_buffer_capacity(buffer_index, maximum_segment_samples);
            if enable_smoothing
                corrected_covariance_buffer(buffer_index, :) = kf.P(:)';
            end
            kf = propagation_function(navstate, this_imu, imu_dt, kf);
            if enable_smoothing
                state_buffer(buffer_index, :) = navigation_state_row(navstate);
                predicted_covariance_buffer(buffer_index, :) = kf.P(:)';
                transition_buffer(buffer_index, :) = kf.phi(:)';
                buffer_index = buffer_index+1;
            end
        end

        nav_row = [0; navstate.time; ...
            navstate.pos(1:2)*param.R2D; navstate.pos(3); ...
            navstate.vel; navstate.att*param.R2D];
        fprintf(forward_fp, nav_format, nav_row);
        pure_ins_row = [0; pure_ins_navstate.time; ...
            pure_ins_navstate.pos(1:2)*param.R2D; pure_ins_navstate.pos(3); ...
            pure_ins_navstate.vel; pure_ins_navstate.att*param.R2D];
        fprintf(pure_ins_fp, nav_format, pure_ins_row);

        progress = floor(10*imu_index/size(imudata, 1))*10;
        if progress > last_progress && mod(progress, 20) == 0
            fprintf('  处理进度：%d %%\n', progress);
            last_progress = progress;
        end
    end

    if enable_smoothing && enable_second_rts && ...
            ~isempty(previous_single_nav)
        fprintf(double_rts_fp, nav_format, previous_single_nav);
    end
    close_nav_files([pure_ins_fp, forward_fp, ...
        single_rts_fp, double_rts_fp]);
    clear output_cleanup;
    fprintf('  工况解算完成，耗时 %.2f s。\n', toc);
end

function row = navigation_state_row(navstate)
%NAVIGATION_STATE_ROW 将导航状态整理为 RTS 缓存行。
    row = [navstate.time, navstate.pos', navstate.vel', navstate.att'];
end

function assert_buffer_capacity(buffer_index, maximum_segment_samples)
%ASSERT_BUFFER_CAPACITY 检查当前测距间隔对应的 RTS 缓存容量。
    if buffer_index > maximum_segment_samples
        error(['RTS 缓存不足。请检查生成数据的测距间隔，或增大 ', ...
            'maximum_segment_samples。']);
    end
end

function masks = normalize_navigation_results(result_paths)
%NORMALIZE_NAVIGATION_RESULTS 将参与评价的方法裁剪到严格共同时间轴。
    navigation = cell(numel(result_paths), 1);
    common_time = [];
    for method_index = 1:numel(result_paths)
        navigation{method_index} = readmatrix(result_paths(method_index), ...
            'FileType', 'text');
        if isempty(navigation{method_index}) || ...
                size(navigation{method_index}, 2) < 11
            error('导航结果为空或列数不足：%s', result_paths(method_index));
        end
        if isempty(common_time)
            common_time = navigation{method_index}(:, 2);
        else
            common_time = intersect(common_time, ...
                navigation{method_index}(:, 2), 'stable');
        end
    end
    if isempty(common_time)
        error('参与评价的导航结果没有共同时间轴。');
    end

    masks = cell(1, numel(result_paths));
    for method_index = 1:numel(result_paths)
        [is_common, location] = ismember(common_time, ...
            navigation{method_index}(:, 2));
        if ~all(is_common)
            error('结果时间轴匹配失败：%s', result_paths(method_index));
        end
        navigation{method_index} = navigation{method_index}(location, :);
        write_nav_data(result_paths(method_index), ...
            navigation{method_index});
        masks{method_index} = true(numel(common_time), 1);
    end
end

function write_nav_data(file_path, nav_data)
%WRITE_NAV_DATA 使用统一导航格式覆盖写入。
    file_id = fopen(file_path, 'wt');
    if file_id < 0
        error('无法写入导航结果：%s', file_path);
    end
    cleanup = onCleanup(@() fclose(file_id));
    fprintf(file_id, ...
        ['%2d %12.6f %12.8f %12.8f %8.4f %8.4f ', ...
        '%8.4f %8.4f %8.4f %8.4f %8.4f\n'], nav_data');
end

function close_nav_files(file_ids)
%CLOSE_NAV_FILES 关闭仍处于打开状态的导航文件。
    for index = 1:numel(file_ids)
        if file_ids(index) >= 0
            try
                fclose(file_ids(index));
            catch
            end
        end
    end
end

function name = data_source_name(source)
%DATA_SOURCE_NAME 返回图表使用的数据来源名称。
    if source == "simulation"
        name = "仿真数据：";
    else
        name = "实测数据：";
    end
end

function case_id = parse_case_id(case_name)
%PARSE_CASE_ID 从 case-00 一类名称中提取数据编号。
    token = regexp(char(case_name), '^case-(\d+)$', 'tokens', 'once');
    if isempty(token)
        error('simulation_case 必须采用 case-00 一类格式。');
    end
    case_id = str2double(token{1});
end

function cfg = use_shared_input_paths(cfg, source_cfg)
%USE_SHARED_INPUT_PATHS 修正旧 m 配置仍指向历史数据目录的问题。
    fields = {'imufilepath', 'rangefile1path', 'rangefile2path', ...
        'rangefile3path', 'truthpath'};
    for index = 1:numel(fields)
        field_name = fields{index};
        cfg.(field_name) = source_cfg.(field_name);
    end
end
