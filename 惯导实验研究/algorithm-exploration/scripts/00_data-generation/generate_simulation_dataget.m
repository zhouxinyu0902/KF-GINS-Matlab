function generated_cases = generate_simulation_dataget( ...
        case_names, overwrite_existing)
%GENERATE_SIMULATION_DATAGET 生成 case-00 至 case-04 的仿真输入数据。
%   generate_simulation_dataget() 只补齐缺失的场景，不覆盖已有数据。
%   generate_simulation_dataget("case-00", true) 重新生成指定场景。
%   generate_simulation_dataget(["case-00","case-01"], true) 重新生成
%   多个指定场景。
%
% 每个场景写入当前统一目录：
%   data/inertial-experiment/algorithm-exploration/
%       simulation/case-XX/input
% 文件包括 IMU_120.txt、truth.txt 和 range1.txt 至 range3.txt。

%% 1. 初始化工程路径
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
paths = setup_inertial_experiment();

if nargin < 1 || isempty(case_names)
    case_names = compose("case-%02d", 0:4);
end
if nargin < 2 || isempty(overwrite_existing)
    overwrite_existing = false;
end
case_names = string(case_names(:));
if ~isscalar(overwrite_existing) || ...
        ~(islogical(overwrite_existing) || isnumeric(overwrite_existing))
    error('overwrite_existing 必须是逻辑标量。');
end
overwrite_existing = logical(overwrite_existing);

%% 2. 检查场景配置
all_configs = simulation_case_configs(paths);
valid_names = string({all_configs.name});
invalid_names = case_names(~ismember(case_names, valid_names));
if ~isempty(invalid_names)
    error('未知场景：%s。有效场景为 case-00 至 case-04。', ...
        strjoin(invalid_names, ', '));
end

generated_cases = strings(0, 1);
for request_index = 1:numel(case_names)
    config = all_configs(valid_names == case_names(request_index));
    case_dir = paths.simulation_input(config.case_id);
    core_files = {
        fullfile(case_dir, 'IMU_120.txt'), ...
        fullfile(case_dir, 'truth.txt'), ...
        fullfile(case_dir, 'range1.txt'), ...
        fullfile(case_dir, 'range2.txt'), ...
        fullfile(case_dir, 'range3.txt')};

    existing_mask = cellfun(@isfile, core_files);
    if all(existing_mask) && ~overwrite_existing
        fprintf('%s 已存在，未覆盖：%s\n', config.name, case_dir);
        continue;
    end
    if any(existing_mask) && ~overwrite_existing
        missing_files = string(core_files(~existing_mask));
        error(['%s 数据不完整。为避免混用不同批次数据，未自动补写。', ...
            '\n缺少：%s\n确认后请使用 ', ...
            'generate_simulation_dataget("%s", true) 整组重建。'], ...
            config.name, strjoin(missing_files, ', '), config.name);
    end
    if ~isfolder(case_dir)
        mkdir(case_dir);
    end

    fprintf('\n开始生成 %s ...\n', config.name);
    generate_one_case(config, case_dir);
    validate_generated_case(case_dir);
    generated_cases(end+1, 1) = string(config.name); %#ok<AGROW>
end

if isempty(generated_cases)
    fprintf('\n没有覆盖任何已有仿真数据。\n');
else
    fprintf('\n已生成：%s\n', strjoin(generated_cases, ', '));
end
end

function configs = simulation_case_configs(paths)
%SIMULATION_CASE_CONFIGS 五组仿真场景的轨迹和信标参数。
reference_truth = fullfile(paths.experiment_input(6), 'truth.nav');
if ~isfile(reference_truth)
    error('缺少构造转弯角速度所需的实测真值：%s', reference_truth);
end

template = struct('name', '', 'case_id', 0, 'reference_truth', '', ...
    'initial_yaw_deg', 0, 'acceleration_mps2', 0.20577, ...
    'trajectory_mode', 'straight', 'beacons_m', zeros(3, 3), ...
    'random_seed', 1);
configs = repmat(template, 5, 1);

configs(1) = make_config('case-00', 0, reference_truth, 37, 0.43, ...
    'turn-profile', [0, -5*sqrt(3), 0; -10, 5*sqrt(3), 0; ...
    -20, -5*sqrt(3), 0]*1000, 1);
configs(2) = make_config('case-01', 1, reference_truth, 90, 0.20577, ...
    'straight', [5, -5*sqrt(3), 0; -5, 5*sqrt(3), 0; ...
    -15, -5*sqrt(3), 0]*1000, 2);
configs(3) = make_config('case-02', 2, reference_truth, 120, 0.20577, ...
    'straight', [5*sqrt(3), -5, 0; -5*sqrt(3), 5, 0; ...
    -5*sqrt(3), -15, 0]*1000, 3);
configs(4) = make_config('case-03', 3, reference_truth, 180, 0.20577, ...
    'straight', [5*sqrt(3), 5, 0; 5*sqrt(3), -15, 0; ...
    -5*sqrt(3), -5, 0]*1000, 4);
configs(5) = make_config('case-04', 4, reference_truth, 105, 0.20577, ...
    'straight', [5*sqrt(2), -5*sqrt(2), 0; ...
    -5*sqrt(2), 5*sqrt(2), 0; ...
    -5*sqrt(6), -5*sqrt(6), 0]*1000, 5);
end

function config = make_config(name, case_id, reference_truth, ...
        initial_yaw_deg, acceleration_mps2, trajectory_mode, ...
        beacons_m, random_seed)
config = struct('name', name, 'case_id', case_id, ...
    'reference_truth', reference_truth, ...
    'initial_yaw_deg', initial_yaw_deg, ...
    'acceleration_mps2', acceleration_mps2, ...
    'trajectory_mode', trajectory_mode, ...
    'beacons_m', beacons_m, 'random_seed', random_seed);
end

function generate_one_case(config, case_dir)
%GENERATE_ONE_CASE 生成一个场景的真值、带误差 IMU 和三路理想测距。
rng(config.random_seed, 'twister');
glvs;

reference_pva = readmatrix(config.reference_truth, 'FileType', 'text');
reference_avp = pvaNED2ENU(reference_pva);
control_indices = [1, 3921, 7001, 12658, 15624, 58769, 85522, ...
    98542, 129033, 135292, 145273, 173556, 180845, 185802, ...
    192935, 210573, 238800, 265681, 293682, 307233, 327564, ...
    354446, 404289, 419691, 499999];
if size(reference_avp, 1) < control_indices(end)
    error('参考真值至少需要 %d 行，实际只有 %d 行。', ...
        control_indices(end), size(reference_avp, 1));
end

yaw_deg = rad2deg(reference_avp(:, 3));
control_yaw = yaw_deg(control_indices)';
sample_delta = diff(control_indices);
yaw_rate_deg_s = diff(control_yaw)./sample_delta*100;
segment_duration_s = sample_delta/100;

sample_interval_s = 0.01;
initial_avp = [[0; 0; deg2rad(config.initial_yaw_deg)]; ...
    [0; 0; 0]; reference_avp(1, 7:9)'];
segment = trjsegment([], 'init', 0);
segment = trjsegment(segment, 'uniform', segment_duration_s(1));
segment = trjsegment(segment, 'accelerate', 10, [], ...
    config.acceleration_mps2);
if strcmp(config.trajectory_mode, 'turn-profile')
    for segment_index = 2:numel(segment_duration_s)
        segment = trjsegment(segment, 'turnleft', ...
            segment_duration_s(segment_index), ...
            yaw_rate_deg_s(segment_index));
    end
else
    segment = trjsegment(segment, 'uniform', 5000);
end

trajectory = trjsimu(initial_avp, segment.wat, sample_interval_s, 1);
imu_error = imuerrset(0.01, 7, 0.0005, 10e-6*1e5/3600);
noisy_imu = imuadderr(trajectory.imu, imu_error);
imu = imuRFU2FRD(noisy_imu);
truth = avpENU2NED(trajectory.avp);

origin = trajectory.avp(1, 7:9);
beacon_position = dxyz2pos(config.beacons_m, origin');
trajectory_position = truth(:, 3:5);
trajectory_position(:, 1:2) = deg2rad(trajectory_position(:, 1:2));
trajectory_xyz = pos2dxyz(trajectory_position, origin');

% 三路测距源均为 1 Hz；第2、3列保持相同的理想水平距离。
range_indices = 100:100:size(trajectory_xyz, 1);
for beacon_index = 1:3
    horizontal_range = hypot( ...
        trajectory_xyz(:, 1)-config.beacons_m(beacon_index, 1), ...
        trajectory_xyz(:, 2)-config.beacons_m(beacon_index, 2));
    beacon_rows = repmat(beacon_position(beacon_index, :), ...
        numel(range_indices), 1);
    range_output = [truth(range_indices, 2), ...
        horizontal_range(range_indices), ...
        horizontal_range(range_indices), beacon_rows];
    writematrix(range_output, fullfile(case_dir, ...
        sprintf('range%d.txt', beacon_index)), 'Delimiter', ' ');
end
writematrix(imu, fullfile(case_dir, 'IMU_120.txt'), 'Delimiter', ' ');
writematrix(truth, fullfile(case_dir, 'truth.txt'), 'Delimiter', ' ');
fprintf('%s 数据写入完成：%s\n', config.name, case_dir);
end

function validate_generated_case(case_dir)
%VALIDATE_GENERATED_CASE 写入后立即检查五个核心文件的数据契约。
imu = readmatrix(fullfile(case_dir, 'IMU_120.txt'), 'FileType', 'text');
truth = readmatrix(fullfile(case_dir, 'truth.txt'), 'FileType', 'text');
if size(imu, 2) ~= 7 || size(truth, 2) ~= 11 || ...
        size(imu, 1) ~= size(truth, 1)
    error('生成的 IMU/truth 维度不正确：%s', case_dir);
end
if any(~isfinite(imu), 'all') || any(~isfinite(truth), 'all') || ...
        any(diff(imu(:, 1)) <= 0) || any(diff(truth(:, 2)) <= 0)
    error('生成的 IMU/truth 含无效值或时间未严格递增：%s', case_dir);
end

expected_range_rows = floor(size(truth, 1)/100);
for beacon_index = 1:3
    range_path = fullfile(case_dir, sprintf('range%d.txt', beacon_index));
    range_data = readmatrix(range_path, 'FileType', 'text');
    if size(range_data, 1) ~= expected_range_rows || ...
            size(range_data, 2) ~= 6 || ...
            any(~isfinite(range_data), 'all') || ...
            any(diff(range_data(:, 1)) <= 0) || ...
            any(abs(range_data(:, 2)-range_data(:, 3)) > 1e-9)
        error('生成的 range%d.txt 不满足 N×6/1 Hz 数据契约。', ...
            beacon_index);
    end
end
fprintf('%s 数据检查通过：%d 个 IMU 历元，%d 个距离历元。\n', ...
    case_dir, size(imu, 1), expected_range_rows);
end
