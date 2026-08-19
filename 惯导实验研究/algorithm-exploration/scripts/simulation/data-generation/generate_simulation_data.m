function generated_cases = generate_simulation_data( ...
        case_names, overwrite_existing)
%GENERATE_SIMULATION_DATA 统一生成惯导实验研究的五组仿真输入数据。
%   generate_simulation_data()：仅生成缺失数据，已有 case 不覆盖，但补图。
%   generate_simulation_data("case-01", true)：重生成并覆盖指定场景。
%
% 正式场景为 case-00 至 case-04。数据写入：
%   data/inertial-experiment/algorithm-exploration/input/simulation/case-XX
% 轨迹、信标和距离图由 plot_simulation_cases 自动保存到：
%   data/inertial-experiment/algorithm-exploration/figures-tables/simulation

    script_dir = fileparts(mfilename('fullpath'));
    topic_dir = fileparts(fileparts(fileparts(script_dir)));
    project_root = fileparts(fileparts(topic_dir));
    addpath(script_dir);
    addpath(topic_dir);
    setup_inertial_experiment();

    if nargin < 1 || isempty(case_names)
        case_names = compose("case-%02d", 0:4);
    end
    if nargin < 2 || isempty(overwrite_existing)
        overwrite_existing = false;
    end
    if ischar(case_names)
        case_names = string({case_names});
    else
        case_names = string(case_names(:));
    end

    all_configs = simulation_case_configs(project_root);
    valid_names = string({all_configs.name});
    if any(~ismember(case_names, valid_names))
        invalid = case_names(~ismember(case_names, valid_names));
        error('未知场景：%s。有效场景为 case-00 至 case-04。', ...
            strjoin(invalid, ', '));
    end

    input_root = fullfile(project_root, 'data', 'inertial-experiment', ...
        'algorithm-exploration', 'input', 'simulation');
    generated_cases = strings(0, 1);

    for request_index = 1:numel(case_names)
        config = all_configs(valid_names == case_names(request_index));
        case_dir = fullfile(input_root, config.name);
        core_files = {
            fullfile(case_dir, 'IMU_120.txt'), ...
            fullfile(case_dir, 'truth.txt'), ...
            fullfile(case_dir, 'range1.txt'), ...
            fullfile(case_dir, 'range2.txt'), ...
            fullfile(case_dir, 'range3.txt')};

        if all(cellfun(@isfile, core_files)) && ~overwrite_existing
            fprintf('%s 已存在，保留现有数据并只更新图片。\n', config.name);
            continue;
        end
        if ~exist(case_dir, 'dir')
            mkdir(case_dir);
        end

        generate_one_case(config, case_dir);
        generated_cases(end + 1, 1) = string(config.name); %#ok<AGROW>
    end

    % 每次调用均为所请求场景生成可视化，已有数据也会更新图片。
    plot_simulation_cases(case_names);
end

function configs = simulation_case_configs(project_root)
% 五组历史生成稿中真正形成数据集的参数表。
    reference_truth = fullfile(project_root, 'data', ...
        'inertial-experiment', 'algorithm-exploration', 'input', 'experiment-reference', 'truth.nav');

    configs = repmat(struct('name', '', 'reference_truth', '', ...
        'initial_yaw_deg', 0, 'acceleration_mps2', 0.20577, ...
        'trajectory_mode', 'straight', 'beacons_m', zeros(3, 3), ...
        'random_seed', 1), 5, 1);

    configs(1) = make_config('case-00', reference_truth, 37, 0.43, ...
        'turn-profile', [0, -5*sqrt(3), 0; -10, 5*sqrt(3), 0; ...
        -20, -5*sqrt(3), 0] * 1000, 1);
    configs(2) = make_config('case-01', reference_truth, 90, 0.20577, ...
        'straight', [5, -5*sqrt(3), 0; -5, 5*sqrt(3), 0; ...
        -15, -5*sqrt(3), 0] * 1000, 2);
    configs(3) = make_config('case-02', reference_truth, 120, 0.20577, ...
        'straight', [5*sqrt(3), -5, 0; -5*sqrt(3), 5, 0; ...
        -5*sqrt(3), -15, 0] * 1000, 3);
    configs(4) = make_config('case-03', reference_truth, 180, 0.20577, ...
        'straight', [5*sqrt(3), 5, 0; 5*sqrt(3), -15, 0; ...
        -5*sqrt(3), -5, 0] * 1000, 4);
    configs(5) = make_config('case-04', reference_truth, 105, 0.20577, ...
        'straight', [5*sqrt(2), -5*sqrt(2), 0; ...
        -5*sqrt(2), 5*sqrt(2), 0; ...
        -5*sqrt(6), -5*sqrt(6), 0] * 1000, 5);
end

function config = make_config(name, reference_truth, initial_yaw_deg, ...
        acceleration_mps2, trajectory_mode, beacons_m, random_seed)
    config = struct('name', name, 'reference_truth', reference_truth, ...
        'initial_yaw_deg', initial_yaw_deg, ...
        'acceleration_mps2', acceleration_mps2, ...
        'trajectory_mode', trajectory_mode, 'beacons_m', beacons_m, ...
        'random_seed', random_seed);
end

function generate_one_case(config, case_dir)
% 根据历史 simu_dataget*.m 的运动和 IMU 噪声参数生成单个场景。
    if ~isfile(config.reference_truth)
        error('缺少生成轨迹所需的参考真值：%s', config.reference_truth);
    end

    rng(config.random_seed);
    glvs;
    reference_pva = readmatrix(config.reference_truth, 'FileType', 'text');
    reference_avp = pvaNED2ENU(reference_pva);

    control_indices = [1, 3921, 7001, 12658, 15624, 58769, 85522, ...
        98542, 129033, 135292, 145273, 173556, 180845, 185802, ...
        192935, 210573, 238800, 265681, 293682, 307233, 327564, ...
        354446, 404289, 419691, 499999];
    if size(reference_avp, 1) < control_indices(end)
        error('参考真值长度不足：至少需要 %d 行，实际为 %d 行。', ...
            control_indices(end), size(reference_avp, 1));
    end
    yaw_deg = rad2deg(reference_avp(:, 3));
    control_yaw = yaw_deg(control_indices)';
    sample_delta = diff(control_indices);
    yaw_rate = diff(control_yaw) ./ sample_delta;
    motion_parameters = [yaw_rate * 100; sample_delta / 100];

    sample_interval_s = 0.01;
    initial_avp = [[0; 0; deg2rad(config.initial_yaw_deg)]; ...
        [0; 0; 0]; reference_avp(1, 7:9)'];
    segment = trjsegment([], 'init', 0);
    segment = trjsegment(segment, 'uniform', motion_parameters(2, 1));
    segment = trjsegment(segment, 'accelerate', 10, [], ...
        config.acceleration_mps2);
    if strcmp(config.trajectory_mode, 'turn-profile')
        for segment_index = 2:size(motion_parameters, 2)
            segment = trjsegment(segment, 'turnleft', ...
                motion_parameters(2, segment_index), ...
                motion_parameters(1, segment_index));
        end
    else
        segment = trjsegment(segment, 'uniform', 5000);
    end

    trajectory = trjsimu(initial_avp, segment.wat, ...
        sample_interval_s, 1);
    imu_error = imuerrset(0.01, 7, 0.0005, 10e-6 * 1e5 / 3600);
    noisy_imu = imuadderr(trajectory.imu, imu_error);
    truth = avpENU2NED(trajectory.avp);
    imu = imuRFU2FRD(noisy_imu);

    origin = trajectory.avp(1, 7:9);
    beacon_position = dxyz2pos(config.beacons_m, origin');
    trajectory_position = truth(:, 3:5);
    trajectory_position(:, 1:2) = deg2rad(trajectory_position(:, 1:2));
    trajectory_xyz = pos2dxyz(trajectory_position, origin');
    horizontal_range = zeros(size(trajectory_xyz, 1), 3);
    for beacon_index = 1:3
        horizontal_range(:, beacon_index) = hypot( ...
            trajectory_xyz(:, 1) - config.beacons_m(beacon_index, 1), ...
            trajectory_xyz(:, 2) - config.beacons_m(beacon_index, 2));
    end

    % 原始脚本每100个100 Hz样本输出一次距离，即1 Hz距离源数据。
    range_indices = 100:100:size(horizontal_range, 1);
    for beacon_index = 1:3
        beacon_rows = repmat(beacon_position(beacon_index, :), ...
            numel(range_indices), 1);
        range_output = [truth(range_indices, 2), ...
            horizontal_range(range_indices, beacon_index), ...
            horizontal_range(range_indices, beacon_index), beacon_rows];
        writematrix(range_output, fullfile(case_dir, ...
            sprintf('range%d.txt', beacon_index)), 'Delimiter', ' ');
    end
    writematrix(imu, fullfile(case_dir, 'IMU_120.txt'), 'Delimiter', ' ');
    writematrix(truth, fullfile(case_dir, 'truth.txt'), 'Delimiter', ' ');

    fprintf('%s 数据生成完成：%s\n', config.name, case_dir);
end

