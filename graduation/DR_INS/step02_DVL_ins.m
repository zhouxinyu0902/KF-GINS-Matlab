%% DVL/INS 组合导航实验
% 保留原来的惯导机械编排、量测更新和误差反馈结构。
% 只在脚本顶部选择组合方式、状态维数和信标，主循环不需要修改。
%
% combination_type:
%   'INS_DVL'       : INS + DVL + depth
%   'INS_DVL_LBL'   : INS + DVL + depth + LBL position
%   'INS_DVL_RANGE' : INS + DVL + depth + single horizontal range

clear;
% close all;
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

%% 1. 实验参数：通常只修改这一节
combination_type = 'INS_DVL';
dimension = 15;
type = {'INS_DVL',15;
    'INS_DVL',17;
    'INS_DVL_LBL',15;
    'INS_DVL_LBL',17;
    'INS_DVL_RANGE',15;
    'INS_DVL_RANGE',17;};
case_indices = 1:6;               % 设置为 1:6 可依次运行全部六种情况
trajectory_tag = '';            % 更换输入轨迹时可填写，如 'straight_4h'
record_imu_error_17state = true;
record_state_std_17state = true;
diagnostic_record_interval_s = 0.5; % IMU/DVL参数记录周期；0.5 s与DVL频率一致
smoke_case_index = str2double(getenv('DVL_INS_SMOKE_CASE'));
if isfinite(smoke_case_index)
    case_indices = smoke_case_index;
end
if any(~ismember(case_indices, 1:size(type, 1)))
    error('case_indices must contain integers from 1 to %d.', size(type, 1));
end
all_run_summaries = table();

for ii = case_indices
    combination_type = type{ii,1};
    dimension = type{ii,2};
    feedback = true;
    imu_error_record = record_imu_error_17state && dimension == 17;
    std_record = record_state_std_17state && dimension == 17;

    random_seed = 22;
    aiding_interval_s = 8.0;
    range_noise_std_m = 5.0;
    lbl_position_noise_std_m = 2.0;
    measurement_match_tolerance_s = 0.02; 
    time_equality_tolerance_s = 1e-9;
    maximum_duration_s = inf;

    % PSINS 的 dxyz2pos 输入顺序是 [East, North, Up]，不是 NED。
    beacons_enu_m = [ ...
        2000,  1800, 50; ...
        -2000,     0, 50; ...
        -2000,  1800, 50; ...
        -4000,  2000,  0; ...
        -4000, 10000,  0; ...
        4000, 10000,  0];
    beacon_index = 1;

    % 仅供快速检查使用，不影响正常运行：
    % setenv('DVL_INS_SMOKE','1'); run('DVL_ins.m')
    smoke_test = strcmpi(strtrim(getenv('DVL_INS_SMOKE')), '1');
    if smoke_test
        maximum_duration_s = 64.0;
    end

    valid_types = {'INS_DVL', 'INS_DVL_LBL', 'INS_DVL_RANGE'};
    combination_type = upper(combination_type);
    if ~any(strcmp(combination_type, valid_types))
        error('Unknown combination type: %s.', combination_type);
    end
    if ~ismember(dimension, [15, 17])
        error('dimension must be 15 or 17.');
    end
    if beacon_index < 1 || beacon_index > size(beacons_enu_m, 1) || ...
            fix(beacon_index) ~= beacon_index
        error('beacon_index must be an integer from 1 to %d.', ...
            size(beacons_enu_m, 1));
    end

    %% 2. 加载数据
    imudata = importdata(cfg.imufilepath);
    dvldata = importdata(cfg.dvlfilepath);
    heightdata = importdata(cfg.heightfilepath);
    truth = importdata(cfg.truthpath);
    simulation_data_path = fullfile(fileparts(cfg.truthpath), ...
        'simulation_data.mat');
    truth_file_info = dir(cfg.truthpath);
    input_duration_s = truth(end, 2) - truth(1, 2);
    input_timestamp = char(datetime(truth_file_info.datenum, ...
        'ConvertFrom', 'datenum', 'Format', 'yyyyMMdd_HHmmss'));
    input_id = sprintf('%s_%drows', ...
        input_timestamp, size(truth, 1));

    if isempty(imudata) || size(imudata, 2) < 7 || ...
            isempty(dvldata) || size(dvldata, 2) < 4 || ...
            isempty(heightdata) || size(heightdata, 2) < 2 || ...
            isempty(truth) || size(truth, 2) < 11
        error('IMU, DVL, depth or truth data are empty or have invalid columns.');
    end

    cfg.starttime = max(cfg.starttime, imudata(1, 1));
    cfg.endtime = min([cfg.endtime, imudata(end, 1), ...
        cfg.starttime + maximum_duration_s]);

    imudata = imudata(imudata(:, 1) >= cfg.starttime & ...
        imudata(:, 1) <= cfg.endtime, :);
    dvldata = dvldata(dvldata(:, 1) >= cfg.starttime & ...
        dvldata(:, 1) <= cfg.endtime, :);
    heightdata = heightdata(heightdata(:, 1) >= cfg.starttime & ...
        heightdata(:, 1) <= cfg.endtime, :);
    truth_window = truth(truth(:, 2) >= cfg.starttime & ...
        truth(:, 2) <= cfg.endtime, :);

    if size(imudata, 1) < 2 || size(dvldata, 1) < 2
        error('Not enough IMU or DVL samples in the selected time window.');
    end
    if size(heightdata, 1) ~= size(imudata, 1) || ...
            any(abs(heightdata(:, 1) - imudata(:, 1)) > 1e-9)
        error('The 100 Hz depth data must be synchronized with the IMU data.');
    end

    %% 3. 生成 8 s 水平距离和 LBL 仿真量测
    truth_pos_lla = truth_window(:, 3:5);
    truth_pos_lla(:, 1:2) = truth_pos_lla(:, 1:2) * param.D2R;
    origin_lla = truth_pos_lla(1, :)';
    truth_pos_enu_m = pos2dxyz(truth_pos_lla, origin_lla);

    truth_dt_s = median(diff(truth_window(:, 2)));
    aiding_stride = max(1, round(aiding_interval_s / truth_dt_s));
    if abs(aiding_stride * truth_dt_s - aiding_interval_s) > 1e-6
        error('The aiding interval must be an integer multiple of truth sample time.');
    end
    aiding_rows = (aiding_stride:aiding_stride:size(truth_window, 1))';

    beacon_enu_m = beacons_enu_m(beacon_index, :);
    beacon_lla = dxyz2pos(beacon_enu_m, origin_lla);
    beacon_lla = beacon_lla(1, 1:3);

    range_vector_enu_m = pos2dxyz(truth_pos_lla, beacon_lla');
    horizontal_range_true_m = vecnorm(range_vector_enu_m(:, 1:2), 2, 2);
    rng(random_seed, 'twister');
    horizontal_range_meas_m = horizontal_range_true_m + ...
        range_noise_std_m * randn(size(horizontal_range_true_m));
    rangedata = [truth_window(:, 2), horizontal_range_meas_m, ...
        horizontal_range_meas_m, repmat(beacon_lla, size(truth_window, 1), 1)];
    rangedata = rangedata(aiding_rows, :);

    rng(random_seed + 1, 'twister');
    lbl_noise_rad = lbl_position_noise_std_m / glv.Re;
    LBLdata = [truth_window(:, 2), ...
        truth_pos_lla(:, 1) + lbl_noise_rad * randn(size(truth_pos_lla, 1), 1), ...
        truth_pos_lla(:, 2) + lbl_noise_rad * randn(size(truth_pos_lla, 1), 1)];
    LBLdata = LBLdata(aiding_rows, :);

    %% 4. 输出目录
    if smoke_test
        result_root = fullfile(repo_root, 'data', 'graduation', '_smoke_DVL_ins');
    else
        result_root = fullfile(repo_root, 'data', 'graduation');
    end
    if ~isempty(trajectory_tag)
        safe_trajectory_tag = regexprep(trajectory_tag, '[^a-zA-Z0-9_-]', '_');
        result_root = fullfile(result_root, safe_trajectory_tag);
    end
    seed_folder = sprintf('seed_%d', random_seed);

    switch combination_type
        case 'INS_DVL'
            result_dir = fullfile(result_root, 'INS_DVL', seed_folder);
        case 'INS_DVL_LBL'
            parameter_folder = sprintf('lblStd_%gm_dt_%gs', ...
                lbl_position_noise_std_m, aiding_interval_s);
            result_dir = fullfile(result_root, 'INS_DVL_LBL', ...
                parameter_folder, seed_folder);
        case 'INS_DVL_RANGE'
            parameter_folder = sprintf( ...
                'beacon_%d_E%gm_N%gm_U%gm_rangeStd_%gm_dt_%gs', ...
                beacon_index, beacon_enu_m(1), beacon_enu_m(2), ...
                beacon_enu_m(3), range_noise_std_m, aiding_interval_s);
            result_dir = fullfile(result_root, 'INS_DVL_RANGE', ...
                parameter_folder, seed_folder);
    end
    result_dir = fullfile(result_dir, sprintf('%dstate', dimension));

    if ~isfolder(result_dir)
        mkdir(result_dir);
    end
    cfg.outputfolder = result_dir;
    cfg.figurefolder = result_dir;

    nav_filename = [combination_type, '.nav'];
    if ~feedback
        nav_filename = [combination_type, '_NO_FEEDBACK.nav'];
    end
    navpath = fullfile(result_dir, nav_filename);

    fprintf('Combination: %s\n', combination_type);
    if strcmp(combination_type, 'INS_DVL_RANGE')
        fprintf('Beacon %d [E, N, U] = [%.1f, %.1f, %.1f] m\n', ...
            beacon_index, beacon_enu_m);
    end
    fprintf('Output: %s\n', result_dir);

    %% 5. 初始化
    if dimension == 15
        [kf, navstate] = myInitialize_15state(cfg);
    else
        [kf, navstate] = myInitialize_17state(cfg);
    end
    kf.dimension = dimension;
    kf.depthstd = 0.4;
    laststate = navstate;
    lastimu = imudata(1, :)';
    thisimu = imudata(1, :)';

    dvlindex = 2;
    while dvlindex <= size(dvldata, 1) && dvldata(dvlindex, 1) < thisimu(1)
        dvlindex = dvlindex + 1;
    end
    rangeindex = find(rangedata(:, 1) >= ...
        thisimu(1) - measurement_match_tolerance_s, 1, 'first');
    if isempty(rangeindex), rangeindex = size(rangedata, 1) + 1; end
    LBLindex = find(LBLdata(:, 1) >= ...
        thisimu(1) - measurement_match_tolerance_s, 1, 'first');
    if isempty(LBLindex), LBLindex = size(LBLdata, 1) + 1; end

    maximum_output_count = size(imudata, 1) - 1;
    nav_result = zeros(maximum_output_count, 11);
    imu_sample_interval_s = median(diff(imudata(:, 1)));
    diagnostic_stride = max(1, round(diagnostic_record_interval_s / ...
        imu_sample_interval_s));
    maximum_diagnostic_count = ceil(maximum_output_count / ...
        diagnostic_stride) + 1;
    diagnostic_count = 0;
    if imu_error_record
        % time + gyro bias + accelerometer bias + gyro scale + accel scale
        % 保持与原 plot_imuerror 脚本兼容的 13 列格式。
        imu_error_result = zeros(maximum_diagnostic_count, 13);
    end
    if std_record
        std_result = zeros(maximum_diagnostic_count, kf.RANK + 1);
    end
    if ~feedback
        state_result = zeros(maximum_output_count, kf.RANK + 1);
    end
    if dimension == 17
        % time, DVL scale, DVL yaw (deg), scale std, yaw std (deg)
        dvl_calibration_result = zeros(maximum_diagnostic_count, 5);
    end

    nav_count = 0;
    range_update_count = 0;
    lbl_update_count = 0;
    last_percent = 0;
    fprintf('Start DVL/INS processing.\n');

    %% 6. 主循环
    for imuindex = 2:size(imudata, 1)
        lastimu = thisimu;
        laststate = navstate;
        thisimu = imudata(imuindex, :)';
        imudt = thisimu(1) - lastimu(1);
        thisimu = myImuCompensate(thisimu, navstate, imudt);

        while dvlindex <= size(dvldata, 1) && ...
                dvldata(dvlindex, 1) < lastimu(1) - time_equality_tolerance_s
            dvlindex = dvlindex + 1;
        end
        if dvlindex > size(dvldata, 1)
            fprintf('DVL file END.\n');
            break;
        end

        dvl_time_s = dvldata(dvlindex, 1);
        if abs(lastimu(1) - dvl_time_s) <= time_equality_tolerance_s
            % DVL 在上一 IMU 时刻：先更新，再传播到当前 IMU 时刻。
            update_time_s = lastimu(1);
            depth_meas_m = heightdata(max(imuindex - 1, 1), 2);
            DVLdata = [dvldata(dvlindex, :), depth_meas_m];
            [kf, rangeindex, LBLindex, used_range, used_lbl] = ...
                measurement_update(combination_type, update_time_s, navstate, ...
                DVLdata, rangedata, rangeindex, LBLdata, LBLindex, ...
                measurement_match_tolerance_s, kf);
            range_update_count = range_update_count + used_range;
            lbl_update_count = lbl_update_count + used_lbl;

            if feedback
                if kf.dimension == 15
                    [kf, navstate] = myErrorFeedback_15state(kf, navstate);
                elseif kf.dimension == 17
                    [kf, navstate] = myErrorFeedback_17state(kf, navstate);
                end
            end
            dvlindex = dvlindex + 1;

            laststate = navstate;
            navstate = InsMech(laststate, lastimu, thisimu);
            if kf.dimension == 15
                kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
            elseif kf.dimension == 17
                kf = myInsPropagate_17state(navstate, thisimu, imudt, kf);
            end
        elseif lastimu(1) < dvl_time_s && thisimu(1) > dvl_time_s
            % DVL 在两个 IMU 历元之间：拆分 IMU 增量。
            [firstimu, secondimu] = interpolate(lastimu, thisimu, dvl_time_s);
            first_dt_s = firstimu(1) - lastimu(1);
            navstate = InsMech(laststate, lastimu, firstimu);
            if kf.dimension == 15
                kf = myInsPropagate_15state(navstate, firstimu, first_dt_s, kf);
            else
                kf = myInsPropagate_17state(navstate, firstimu, first_dt_s, kf);
            end

            depth_meas_m = interp1(heightdata(imuindex-1:imuindex, 1), ...
                heightdata(imuindex-1:imuindex, 2), dvl_time_s, 'linear');
            DVLdata = [dvldata(dvlindex, :), depth_meas_m];
            [kf, rangeindex, LBLindex, used_range, used_lbl] = ...
                measurement_update(combination_type, dvl_time_s, navstate, ...
                DVLdata, rangedata, rangeindex, LBLdata, LBLindex, ...
                measurement_match_tolerance_s, kf);
            range_update_count = range_update_count + used_range;
            lbl_update_count = lbl_update_count + used_lbl;

            if feedback
                if kf.dimension == 15
                    [kf, navstate] = myErrorFeedback_15state(kf, navstate);
                else
                    [kf, navstate] = myErrorFeedback_17state(kf, navstate);
                end
            end
            dvlindex = dvlindex + 1;

            laststate = navstate;
            lastimu = firstimu;
            second_dt_s = secondimu(1) - lastimu(1);
            navstate = InsMech(laststate, lastimu, secondimu);
            if kf.dimension == 15
                kf = myInsPropagate_15state(navstate, secondimu, second_dt_s, kf);
            elseif kf.dimension == 17
                kf = myInsPropagate_17state(navstate, secondimu, second_dt_s, kf);
            end
        else
            % 无 DVL 时只用深度约束垂向位置和垂向速度。
            navstate = InsMech(laststate, lastimu, thisimu);
            height_measurement = [heightdata(imuindex, 1), -heightdata(imuindex, 2)];
            kf = myHeightUpdate(navstate, height_measurement, kf);
            navstate.pos(3) = navstate.pos(3) - kf.x(3);
            navstate.vel(3) = navstate.vel(3) - kf.x(6);
            kf.x(3) = 0;
            kf.x(6) = 0;
            if kf.dimension == 15
                kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
            elseif kf.dimension == 17
                kf = myInsPropagate_17state(navstate, thisimu, imudt, kf);
            end
        end

        nav_count = nav_count + 1;
        nav_result(nav_count, :) = [nav_count - 1, navstate.time, ...
            navstate.pos(1) * param.R2D, navstate.pos(2) * param.R2D, ...
            navstate.pos(3), navstate.vel', navstate.att' * param.R2D];

        record_diagnostics = mod(nav_count - 1, diagnostic_stride) == 0 || ...
            imuindex == size(imudata, 1);
        if record_diagnostics && ...
                (imu_error_record || std_record || dimension == 17)
            diagnostic_count = diagnostic_count + 1;
            if imu_error_record
                imu_error_result(diagnostic_count, :) = [navstate.time, ...
                    navstate.gyrbias' * param.R2D * 3600, ...
                    navstate.accbias' * 1e5, ...
                    navstate.gyrscale' * 1e6, ...
                    navstate.accscale' * 1e6];
            end
            if std_record
                state_std = sqrt(max(diag(kf.P), 0))';
                state_std(7:9) = state_std(7:9) * param.R2D;
                state_std(10:12) = state_std(10:12) * param.R2D * 3600;
                state_std(13:15) = state_std(13:15) * 1e5;
                if dimension == 17
                    state_std(17) = state_std(17) * param.R2D;
                end
                std_result(diagnostic_count, :) = [navstate.time, state_std];
            end
            if dimension == 17
                dvl_calibration_result(diagnostic_count, :) = [navstate.time, ...
                    navstate.dvlscale, navstate.dvlyaw * param.R2D, ...
                    sqrt(max(kf.P(16, 16), 0)), ...
                    sqrt(max(kf.P(17, 17), 0)) * param.R2D];
            end
        end
        if ~feedback
            state_result(nav_count, :) = [navstate.time, kf.x'];
        end

        current_percent = imuindex / size(imudata, 1);
        if current_percent - last_percent >= 0.20
            fprintf('processing %d %%\n', floor(100 * current_percent));
            last_percent = current_percent;
        end
    end

    %% 7. 保存结果
    nav_result = nav_result(1:nav_count, :);
    writematrix(nav_result, navpath, 'FileType', 'text', 'Delimiter', 'tab');

    if imu_error_record
        imu_error_path = fullfile(result_dir, 'imu_error.txt');
        imu_error_data = imu_error_result(1:diagnostic_count, :);
        writematrix(imu_error_data, imu_error_path, ...
            'FileType', 'text', 'Delimiter', 'tab');
        imu_error_table = array2table(imu_error_data, 'VariableNames', { ...
            'time_s', 'gyro_bias_x_dph', 'gyro_bias_y_dph', ...
            'gyro_bias_z_dph', 'acc_bias_x_mgal', 'acc_bias_y_mgal', ...
            'acc_bias_z_mgal', 'gyro_scale_x_ppm', 'gyro_scale_y_ppm', ...
            'gyro_scale_z_ppm', 'acc_scale_x_ppm', 'acc_scale_y_ppm', ...
            'acc_scale_z_ppm'});
        writetable(imu_error_table, fullfile(result_dir, 'imu_error.csv'));
    else
        imu_error_path = '';
    end
    if std_record
        nav_std_data = std_result(1:diagnostic_count, :);
        writematrix(nav_std_data, fullfile(result_dir, 'nav_std.txt'), ...
            'FileType', 'text', 'Delimiter', 'tab');
        std_names = {'time_s', 'lat_std_rad', 'lon_std_rad', ...
            'height_std_m', 'vel_n_std_mps', 'vel_e_std_mps', ...
            'vel_d_std_mps', 'roll_std_deg', 'pitch_std_deg', ...
            'yaw_std_deg', 'gyro_bias_x_std_dph', ...
            'gyro_bias_y_std_dph', 'gyro_bias_z_std_dph', ...
            'acc_bias_x_std_mgal', 'acc_bias_y_std_mgal', ...
            'acc_bias_z_std_mgal'};
        if dimension == 17
            std_names = [std_names, ...
                {'dvl_scale_std', 'dvl_yaw_std_deg'}]; %#ok<AGROW>
        end
        writetable(array2table(nav_std_data, 'VariableNames', std_names), ...
            fullfile(result_dir, 'nav_std.csv'));
    end
    if ~feedback
        writematrix(state_result(1:nav_count, :), ...
            fullfile(result_dir, 'error_state.txt'), ...
            'FileType', 'text', 'Delimiter', 'tab');
    end
    if dimension == 17
        dvl_calibration_path = fullfile(result_dir, 'dvl_calibration.txt');
        dvl_calibration_data = dvl_calibration_result(1:diagnostic_count, :);
        writematrix(dvl_calibration_data, dvl_calibration_path, ...
            'FileType', 'text', 'Delimiter', 'tab');
        dvl_calibration_table = array2table(dvl_calibration_data, ...
            'VariableNames', {'time_s', 'dvl_scale_estimate', ...
            'dvl_yaw_estimate_deg', 'dvl_scale_std', ...
            'dvl_yaw_std_deg'});
        writetable(dvl_calibration_table, ...
            fullfile(result_dir, 'dvl_calibration.csv'));
    else
        dvl_calibration_path = '';
    end

    save(fullfile(result_dir, 'run_config.mat'), ...
        'combination_type', 'dimension', 'feedback', 'random_seed', 'ii', ...
        'trajectory_tag', 'input_id', 'input_duration_s', ...
        'simulation_data_path', 'diagnostic_record_interval_s', ...
        'aiding_interval_s', 'range_noise_std_m', ...
        'lbl_position_noise_std_m', 'beacons_enu_m', 'beacon_index', ...
        'beacon_enu_m', 'navpath', 'nav_count', ...
        'range_update_count', 'lbl_update_count');

    run_info = struct('combination_type', combination_type, ...
        'dimension', dimension, 'case_index', ii, ...
        'measurement_seed', random_seed, 'feedback', feedback, ...
        'aiding_interval_s', aiding_interval_s, ...
        'range_noise_std_m', range_noise_std_m, ...
        'lbl_position_noise_std_m', lbl_position_noise_std_m, ...
        'beacon_index', beacon_index, 'beacon_enu_m', beacon_enu_m, ...
        'trajectory_tag', trajectory_tag, ...
        'input_id', input_id, 'input_duration_s', input_duration_s, ...
        'diagnostic_record_interval_s', diagnostic_record_interval_s, ...
        'range_update_count', range_update_count, ...
        'lbl_update_count', lbl_update_count);
    run_summary = save_dvl_ins_metrics(navpath, cfg.truthpath, ...
        simulation_data_path, imu_error_path, dvl_calibration_path, ...
        result_dir, run_info);
    all_run_summaries = [all_run_summaries; run_summary]; %#ok<AGROW>
    writetable(all_run_summaries, fullfile(result_root, ...
        sprintf('comparison_seed_%d.csv', random_seed)));

    error_fig = calc_error_gjb(navpath, cfg.truthpath, false, 'posradial');
    exportgraphics(error_fig, fullfile(result_dir, ...
        'navigation_error.png'), 'Resolution', 600);
    if dimension == 17
        simulation_cfg = struct();
        if isfile(simulation_data_path)
            simulation_cfg_data = load(simulation_data_path, 'cfg');
            simulation_cfg = simulation_cfg_data.cfg;
        end
        true_dvl_scale = get_struct_field(simulation_cfg, ...
            'dvl_scale_error', nan);
        true_dvl_yaw_deg = nan;
        if isfield(simulation_cfg, 'dvl_install_pry_deg')
            true_dvl_yaw_deg = simulation_cfg.dvl_install_pry_deg(3);
        end
        calibration_fig = plot_dvl_calibration(dvl_calibration_path, ...
            true_dvl_scale, true_dvl_yaw_deg);
        exportgraphics(calibration_fig, fullfile(result_dir, ...
            'calibration_result.png'), 'Resolution', 600);

        imuerrpath = imu_error_path;
        plot_imuerror;
        imu_error_fig = gcf;
        exportgraphics(imu_error_fig, fullfile(result_dir, ...
            'imu_error.png'), 'Resolution', 600);
    end
    if strcmp(combination_type, 'INS_DVL_RANGE')
        delta_east_m = beacon_enu_m(1) - truth_pos_enu_m(:, 1);
        delta_north_m = beacon_enu_m(2) - truth_pos_enu_m(:, 2);
        beacon_bearing_deg = atan2d(delta_east_m, delta_north_m);
        relative_bearing_deg = mod( ...
            beacon_bearing_deg - truth_window(:, 11) + 180, 360) - 180;

        beacon_fig = myfigurestartup(7,2.5,'paper');
        tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
        nexttile;
        plot(truth_pos_enu_m(:, 1), truth_pos_enu_m(:, 2), ...
            'LineWidth', 1.2);
        hold on;
        plot(beacon_enu_m(1), beacon_enu_m(2), 'p', ...
            'MarkerSize', 12, 'MarkerFaceColor', [0.95, 0.65, 0.10]);
        axis equal;
        grid on;
        xlabel('East (m)');
        ylabel('North (m)');
        legend('Truth trajectory', 'Beacon', 'Location', 'best');

        nexttile;
        plot((truth_window(:, 2) - truth_window(1, 2)) / 60, ...
            relative_bearing_deg, 'LineWidth', 1.2);
        hold on;
        yline(0, '--');
        yline(90, '--');
        yline(-90, '--');
        grid on;
        xlabel('Time (min)');
        ylabel('Beacon relative to heading (deg)');
        ylim([-180, 180]);
        yticks(-180:45:180);
        exportgraphics(beacon_fig, fullfile(result_dir, ...
            'beacon_geometry.png'), 'Resolution', 200);
    end

    fprintf('DVL/INS processing finished.\n');
    fprintf('Navigation result: %s\n', navpath);
    fprintf('Rows: %d, Range updates: %d, LBL updates: %d\n', ...
        nav_count, range_update_count, lbl_update_count);

end

function value = get_struct_field(data, field_name, default_value)
if isfield(data, field_name)
    value = data.(field_name);
else
    value = default_value;
end
end

function [kf, rangeindex, LBLindex, used_range, used_lbl] = ...
    measurement_update(combination_type, update_time_s, navstate, ...
    DVLdata, rangedata, rangeindex, LBLdata, LBLindex, tolerance_s, kf)
% 按组合类型选择当前 DVL 历元的量测更新。
used_range = false;
used_lbl = false;

while rangeindex <= size(rangedata, 1) && ...
        rangedata(rangeindex, 1) < update_time_s - tolerance_s
    rangeindex = rangeindex + 1;
end
while LBLindex <= size(LBLdata, 1) && ...
        LBLdata(LBLindex, 1) < update_time_s - tolerance_s
    LBLindex = LBLindex + 1;
end

switch combination_type
    case 'INS_DVL'
        if kf.dimension ==15
            kf = myDVLupdate(navstate, DVLdata, kf);
        elseif kf.dimension ==17
            kf = myDVLupdate_17state(navstate, DVLdata, kf);
        end
    case 'INS_DVL_LBL'
        if LBLindex <= size(LBLdata, 1) && ...
                abs(update_time_s - LBLdata(LBLindex, 1)) <= tolerance_s
            if kf.dimension == 15
                kf = myLBLDVLupdate(navstate, DVLdata, ...
                    LBLdata(LBLindex, :), kf);
            else
                kf = myLBLDVLupdate_17state(navstate, DVLdata, ...
                    LBLdata(LBLindex, :), kf);
            end
            LBLindex = LBLindex + 1;
            used_lbl = true;
        else
            if kf.dimension == 15
                kf = myDVLupdate(navstate, DVLdata, kf);
            else
                kf = myDVLupdate_17state(navstate, DVLdata, kf);
            end
        end
    case 'INS_DVL_RANGE'
        if rangeindex <= size(rangedata, 1) && ...
                abs(update_time_s - rangedata(rangeindex, 1)) <= tolerance_s
            if kf.dimension == 15
                kf = myRangeDVLupdate(navstate, DVLdata, ...
                    rangedata(rangeindex, :), kf);
            else
                kf = myRangeDVLupdate_17state(navstate, DVLdata, ...
                    rangedata(rangeindex, :), kf);
            end
            rangeindex = rangeindex + 1;
            used_range = true;
        else
            if kf.dimension == 15
                kf = myDVLupdate(navstate, DVLdata, kf);
            else
                kf = myDVLupdate_17state(navstate, DVLdata, kf);
            end
        end
end
end
