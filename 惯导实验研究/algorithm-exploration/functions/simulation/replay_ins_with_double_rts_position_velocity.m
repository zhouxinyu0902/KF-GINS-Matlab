function outputs = replay_ins_with_double_rts_position_velocity( ...
        case_name, result_dir, position_error_unit)
%REPLAY_INS_WITH_DOUBLE_RTS_POSITION_VELOCITY 使用2RTS位置速度约束重放INS。
%   outputs = replay_ins_with_double_rts_position_velocity( ...
%       case_name, result_dir, position_error_unit)
%   从指定数据集读取输入，并从同一结果目录读取前向EKF和二次RTS结果。

if nargin < 1 || isempty(case_name)
    case_name = 'case-01';
end
case_name = char(string(case_name));
if nargin < 3 || isempty(position_error_unit)
    position_error_unit = "rad";
end
position_error_unit = lower(string(position_error_unit));
if ~ismember(position_error_unit, ["rad", "m"])
    error('position_error_unit 只能设置为 "rad" 或 "m"。');
end

%% 二次 RTS 水平位置和速度联合约束的历史 IMU 重放
% 第一遍导航已经生成二次 RTS 位置结果。本脚本重新读取同一段历史 IMU，
% 从起点重新执行 INS 机械编排和 15 状态误差滤波：
%   1）IMU 负责100 Hz连续传播，保证输出轨迹符合惯导运动学；
%   2）每1 s使用一次二次 RTS 水平位置和水平速度作为联合伪量测；
%   3）普通位置标准差为30 m，速度标准差为0.3 m/s；
%   4）二次 RTS 分段交界前后60 s连续降低位置和速度量测权重；
%   5）高度继续采用仿真深度计结果，不使用二次 RTS 高度；
%   6）利用创新门控抑制异常伪量测造成的突变。
%
% 时延说明：本方法没有消除二次 RTS 的信息时延。若作为准实时输出，
% 当前实现仍需等待两个420 s测距区间，即至少滞后840 s（14 min）；
% 历史IMU重放还会增加少量计算时间。完整数据结束后运行则属于离线后处理。

rng(1);                              % 与第一遍仿真使用相同随机数种子。
glvs;
param = Param();

%% 参数配置
options.case_name = case_name;
% 第11个测距点位于4620 s，再保留1 s用于观察更新后的导航响应。
options.end_time_s = 4621;
options.range_interval_s = 420;
options.rts_update_interval_s = 1;
options.rts_position_std_m = 30;
options.rts_velocity_std_mps = 0.3;
options.boundary_guard_s = 60;
options.boundary_position_std_m = 80;
options.boundary_velocity_std_mps = 1.0;
options.depth_noise_std_m = 0.4;
options.filter_depth_std_m = 0.4;
options.innovation_gate_sigma = 3;
options.robust_gate_sigma = 6;
options.robust_std_scale = 3;
options.maximum_plot_points = 20000;
options.position_error_unit = position_error_unit;

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
project_root = fileparts(fileparts(topic_dir));
addpath(topic_dir);
setup_inertial_experiment();

input_dir = fullfile(project_root, 'data', 'inertial-experiment', ...
    'algorithm-exploration', 'input', 'simulation', options.case_name);
if nargin < 2 || isempty(result_dir)
    result_dir = fullfile(project_root, 'data', 'inertial-experiment', ...
        'algorithm-exploration', 'navigation-results', 'simulation', ...
        sprintf('forward-backward-%s', options.position_error_unit));
end
result_dir = char(string(result_dir));
figure_dir = exploration_artifact_dir(result_dir);
if ~exist(figure_dir, 'dir')
    mkdir(figure_dir);
end

if options.position_error_unit == "rad"
    cfg = ProcessConfigforSimu(input_dir);
else
    cfg = ProcessConfigforSimu_m(input_dir);
end
cfg.outputfolder = result_dir;

double_rts_path = fullfile(result_dir, 'range-ins-rts-double.nav');
rotation_contraction_path = fullfile(result_dir, ...
    'range-ins-rts-double-bridge-rotation.nav');
forward_path = fullfile(result_dir, 'range-ins-forward.nav');
output_path = fullfile(result_dir, ...
    'range-ins-double-rts-position-velocity-guided-replay.nav');
artifact_dir = exploration_artifact_dir(result_dir);
update_diagnostic_path = fullfile(artifact_dir, ...
    'range-ins-double-rts-position-velocity-guided-replay-updates.csv');
statistics_path = fullfile(artifact_dir, ...
    'range-ins-double-rts-position-velocity-guided-replay-statistics.csv');
figure_path = fullfile(figure_dir, ...
    'range-ins-double-rts-position-velocity-guided-replay-comparison.png');
figure_source_path = fullfile(figure_dir, ...
    'range-ins-double-rts-position-velocity-guided-replay-comparison.fig');

required_files = {cfg.imufilepath, cfg.truthpath, cfg.rangefile1path, ...
    cfg.rangefile2path, cfg.rangefile3path, double_rts_path, ...
    rotation_contraction_path, forward_path};
for file_index = 1:numel(required_files)
    if ~isfile(required_files{file_index})
        error('缺少历史 IMU 重放输入文件：%s', required_files{file_index});
    end
end

%% 读取历史数据并重建与第一遍一致的深度计序列
imu_all = readmatrix(cfg.imufilepath, 'FileType', 'text');
truth_all = readmatrix(cfg.truthpath, 'FileType', 'text');
double_rts_nav = readmatrix(double_rts_path, 'FileType', 'text');
rotation_contraction_nav = readmatrix(rotation_contraction_path, ...
    'FileType', 'text');
forward_nav = readmatrix(forward_path, 'FileType', 'text');

start_time = max([cfg.starttime, imu_all(1, 1), truth_all(1, 2), ...
    double_rts_nav(1, 2)]);
end_time = min([options.end_time_s, cfg.endtime, imu_all(end, 1), ...
    truth_all(end, 2), double_rts_nav(end, 2)]);
cfg.starttime = start_time;
cfg.endtime = end_time;

imu_mask = imu_all(:, 1) >= start_time & imu_all(:, 1) <= end_time;
imudata = imu_all(imu_mask, :);
double_rts_nav = double_rts_nav( ...
    double_rts_nav(:, 2) >= start_time & double_rts_nav(:, 2) <= end_time, :);
rotation_contraction_nav = rotation_contraction_nav( ...
    rotation_contraction_nav(:, 2) >= start_time & ...
    rotation_contraction_nav(:, 2) <= end_time, :);
forward_nav = forward_nav( ...
    forward_nav(:, 2) >= start_time & forward_nav(:, 2) <= end_time, :);

if size(double_rts_nav, 1) ~= size(imudata, 1) || ...
        max(abs(double_rts_nav(:, 2) - imudata(:, 1))) > 1e-7
    error('二次 RTS 与历史 IMU 的时间轴不一致。');
end
if size(forward_nav, 1) ~= size(imudata, 1) || ...
        max(abs(forward_nav(:, 2) - imudata(:, 1))) > 1e-7
    error('前向 EKF 与历史 IMU 的时间轴不一致。');
end
if size(rotation_contraction_nav, 1) ~= size(imudata, 1) || ...
        max(abs(rotation_contraction_nav(:, 2) - imudata(:, 1))) > 1e-7
    error('2RTS+旋转收缩与历史 IMU 的时间轴不一致。');
end

% 第一遍仿真先生成测距噪声，再生成深度计噪声。这里先消耗相同数量的
% 随机数，确保历史重放使用与第一遍完全相同的深度计序列。
range_sources = {
    readmatrix(cfg.rangefile1path, 'FileType', 'text'), ...
    readmatrix(cfg.rangefile2path, 'FileType', 'text'), ...
    readmatrix(cfg.rangefile3path, 'FileType', 'text')};
source_dt = median(diff(range_sources{1}(:, 1)));
range_step = round(options.range_interval_s / source_dt);
for source_index = 1:numel(range_sources)
    range_sources{source_index} = range_sources{source_index}( ...
        range_step:range_step:end, :);
end
event_count = min(cellfun(@(x) size(x, 1), range_sources));
discarded_range_noise = randn(event_count, 1); %#ok<RAND>
clear discarded_range_noise;              % 消耗第一遍测距噪声对应的随机数。

height_truth = interp1(truth_all(:, 2), truth_all(:, 5), imudata(:, 1), ...
    'linear', 'extrap');
depth_height = height_truth + options.depth_noise_std_m ...
    * randn(size(height_truth));
height = [imudata(:, 1), depth_height];
clear range_sources imu_all;

%% 初始化第二遍 INS/POS 组合导航
% 水平位置由二次 RTS 起点初始化；高度使用深度计；姿态、速度和IMU误差
% 初值保持原仿真配置。初始化后仍由历史 IMU 独立连续传播。
cfg.initpos(1:2) = double_rts_nav(1, 3:4)' * param.D2R;
cfg.initpos(3) = depth_height(1);
[kf, navstate] = myInitialize_15state(cfg);
kf.depthstd = options.filter_depth_std_m;
% 原仿真配置的初始位置协方差是毫米级，不适用于把2RTS作为30 m伪量测
% 的第二遍滤波。这里按伪量测精度重设水平位置初始不确定度。
initial_horizontal_scale = [navstate.Rm + navstate.pos(3); ...
    (navstate.Rn + navstate.pos(3)) * cos(navstate.pos(1))];
if options.position_error_unit == "rad"
    kf.P(1:2, 1:2) = diag((options.rts_position_std_m ...
        ./ initial_horizontal_scale) .^ 2);
else
    kf.P(1:2, 1:2) = eye(2) * options.rts_position_std_m ^ 2;
end
kf.P(4:5, 4:5) = eye(2) * options.rts_velocity_std_mps ^ 2;

sample_count = size(imudata, 1);
replay_nav = nan(sample_count, 11);
navstate.time = imudata(1, 1);
navstate.pos(3) = depth_height(1);
replay_nav(1, :) = state_to_nav_row(navstate, param);

% 从下一个整10 s时刻开始使用二次 RTS 水平位置伪量测。
next_update_time = ceil(start_time / options.rts_update_interval_s) ...
    * options.rts_update_interval_s;
if next_update_time <= start_time + 1e-9
    next_update_time = next_update_time + options.rts_update_interval_s;
end

update_diagnostics = repmat(empty_update_diagnostic(), 0, 1);
this_imu = imudata(1, :)';
last_progress = -1;

%% 历史 IMU 重放主循环
for imu_index = 2:sample_count
    last_imu = this_imu;
    this_imu = imudata(imu_index, :)';
    imu_dt = this_imu(1) - last_imu(1);
    time_tolerance = max(1e-8, abs(imu_dt) * 0.25);

    % navstate 当前对应 last_imu 时刻。伪量测更新完成后，再从该后验状态
    % 传播到 this_imu，避免同一时刻状态含义不清。
    if last_imu(1) >= next_update_time - time_tolerance
        measurement_nav = interp1(double_rts_nav(:, 2), ...
            double_rts_nav(:, 3:7), last_imu(1), 'linear');
        measurement_position = measurement_nav(1:2)' * param.D2R;
        measurement_velocity = measurement_nav(4:5)';
        [position_std_m, velocity_std_mps, is_boundary_guard] = ...
            select_measurement_std( ...
            last_imu(1), options);
        [kf, update_info] = update_horizontal_rts_position_velocity( ...
            navstate, measurement_position, measurement_velocity, kf, ...
            position_std_m, velocity_std_mps, options, ...
            options.position_error_unit);

        if update_info.accepted
            [kf, navstate] = feedback_replay_error( ...
                kf, navstate, options.position_error_unit);
            replay_nav(imu_index - 1, :) = state_to_nav_row(navstate, param);
        end

        item = empty_update_diagnostic();
        item.time_s = last_imu(1);
        item.position_std_m = update_info.effective_std_m;
        item.velocity_std_mps = update_info.effective_velocity_std_mps;
        item.in_boundary_guard = is_boundary_guard;
        item.innovation_m = update_info.innovation_m;
        item.velocity_innovation_mps = update_info.velocity_innovation_mps;
        item.normalized_innovation = update_info.normalized_innovation;
        item.robust_downweighted = update_info.robust_downweighted;
        item.accepted = update_info.accepted;
        update_diagnostics(end + 1, 1) = item; %#ok<SAGROW>

        next_update_time = next_update_time + options.rts_update_interval_s;
    end

    last_state = navstate;
    navstate = InsMech(last_state, last_imu, this_imu);
    % 与 all_m.m 一致：每个 IMU 历元用高度量测执行解耦 Kalman 更新，
    % 只反馈垂向位置和垂向速度，不影响水平 POS/速度约束状态。
    if options.position_error_unit == "rad"
        [kf, navstate] = update_decoupled_height( ...
            kf, navstate, height(imu_index, :));
    else
        [kf, navstate] = update_decoupled_height_m( ...
            kf, navstate, height(imu_index, :));
    end
    navstate.time = this_imu(1);
    if options.position_error_unit == "rad"
        kf = myInsPropagate_15state(navstate, this_imu, imu_dt, kf);
    else
        kf = myInsPropagate_15state_m(navstate, this_imu, imu_dt, kf);
    end
    replay_nav(imu_index, :) = state_to_nav_row(navstate, param);

    progress = floor(10 * imu_index / sample_count) * 10;
    if progress > last_progress && mod(progress, 20) == 0
        fprintf('历史 IMU 重放进度：%d %%\n', progress);
        last_progress = progress;
    end
end

write_nav_file(output_path, replay_nav);
update_table = struct2table(update_diagnostics);
writetable(update_table, update_diagnostic_path);

%% 统一评价前向 EKF、二次 RTS、旋转收缩和位置速度约束结果
truth_position = interp1(truth_all(:, 2), truth_all(:, 3:5), ...
    imudata(:, 1), 'linear', 'extrap');
method_names = { ...
    '前向 EKF', ...
    '二次 RTS', ...
    '2RTS+旋转收缩', ...
    '2RTS+位置速度约束'};
nav_results = {forward_nav, double_rts_nav, ...
    rotation_contraction_nav, replay_nav};
result_count = numel(nav_results);
radial_error = zeros(sample_count, result_count);
rmse_m = zeros(result_count, 1);
mean_m = zeros(result_count, 1);
median_m = zeros(result_count, 1);
p95_m = zeros(result_count, 1);
maximum_m = zeros(result_count, 1);

for result_index = 1:result_count
    radial_error(:, result_index) = calculate_horizontal_radial_error( ...
        nav_results{result_index}(:, 3:5), truth_position);
    values = radial_error(:, result_index);
    rmse_m(result_index) = sqrt(mean(values .^ 2));
    mean_m(result_index) = mean(values);
    median_m(result_index) = median(values);
    p95_m(result_index) = calculate_percentile(values, 95);
    maximum_m(result_index) = max(values);
end

method = method_names(:);
statistics = table(method, rmse_m, mean_m, median_m, p95_m, maximum_m, ...
    'VariableNames', {'Method', 'RMSE_m', 'Mean_m', 'Median_m', ...
    'P95_m', 'Maximum_m'});
writetable(statistics, statistics_path);

%% 绘制平面轨迹和水平径向误差
plot_count = min(options.maximum_plot_points, sample_count);
display_index = unique(round(linspace(1, sample_count, plot_count)));
origin = truth_position(1, :);
[truth_east, truth_north] = position_to_local_plane( ...
    truth_position(display_index, :), origin);
colors = [ ...
    0.10, 0.35, 0.75; ...
    0.05, 0.55, 0.55; ...
    0.78, 0.16, 0.52; ...
    0.20, 0.65, 0.35];
line_styles = {'-', '--', ':', '-.'};

comparison_figure = figure('Color', 'w', ...
    'Name', '二次 RTS位置和速度约束的历史 IMU 重放', ...
    'Position', [80, 120, 1500, 600]);
layout = tiledlayout(comparison_figure, 1, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, sprintf(['%s：2RTS位置+速度约束历史IMU重放', ...
    '（间隔 %.0f s，位置 %.0f m，速度 %.1f m/s）'], ...
    options.case_name, options.rts_update_interval_s, ...
    options.rts_position_std_m, options.rts_velocity_std_mps));

nexttile(layout, 1);
plot(truth_east / 1000, truth_north / 1000, 'k-', ...
    'LineWidth', 1.7, 'DisplayName', '真值');
hold on;
for result_index = 1:result_count
    [east, north] = position_to_local_plane( ...
        nav_results{result_index}(display_index, 3:5), origin);
    plot(east / 1000, north / 1000, ...
        'Color', colors(result_index, :), ...
        'LineStyle', line_styles{result_index}, ...
        'LineWidth', 1.3, 'DisplayName', method_names{result_index});
end
axis tight;
grid on;
box on;
xlabel('东向位置 (km)');
ylabel('北向位置 (km)');
title('平面轨迹（横纵轴独立缩放）');
legend('Location', 'best', 'Interpreter', 'none');

nexttile(layout, 2);
hold on;
for result_index = 1:result_count
    plot(imudata(display_index, 1), ...
        radial_error(display_index, result_index), ...
        'Color', colors(result_index, :), ...
        'LineStyle', line_styles{result_index}, ...
        'LineWidth', 1.25, 'DisplayName', method_names{result_index});
end
grid on;
box on;
xlabel('时间 (s)');
ylabel('水平径向误差 (m)');
title('水平径向误差');
legend('Location', 'best', 'Interpreter', 'none');
xlim([0, options.end_time_s]);

set(findall(comparison_figure, '-property', 'FontName'), ...
    'FontName', 'Microsoft YaHei');
exportgraphics(comparison_figure, figure_path, 'Resolution', 300);
savefig(comparison_figure, figure_source_path);

accepted_count = sum([update_diagnostics.accepted]);
robust_count = sum([update_diagnostics.robust_downweighted]);
fprintf('\n二次 RTS位置+速度约束历史IMU重放统计：\n');
disp(statistics);
fprintf('伪量测总数：%d，接受：%d，鲁棒降权：%d，拒绝：%d。\n', ...
    numel(update_diagnostics), accepted_count, robust_count, ...
    numel(update_diagnostics) - accepted_count);
fprintf('重放导航结果：%s\n', output_path);
fprintf('对比图片：%s\n', figure_path);
fprintf(['时延：准实时使用时仍至少滞后 %.0f s（%.0f min），', ...
    '离线处理不称实时延迟。\n'], 2 * options.range_interval_s, ...
    2 * options.range_interval_s / 60);

outputs = struct( ...
    'case_name', options.case_name, ...
    'position_error_unit', char(options.position_error_unit), ...
    'input_dir', input_dir, ...
    'result_dir', result_dir, ...
    'forward_path', forward_path, ...
    'double_rts_path', double_rts_path, ...
    'rotation_contraction_path', rotation_contraction_path, ...
    'position_velocity_path', output_path, ...
    'statistics_path', statistics_path, ...
    'figure_path', figure_path, ...
    'statistics', statistics);
end

%% 局部函数
function [position_std_m, velocity_std_mps, is_boundary_guard] = ...
        select_measurement_std(time_s, options)
%SELECT_MEASUREMENT_STD 在分段交界附近连续降低位置和速度量测权重。
    distance_to_boundary = abs(time_s ...
        - round(time_s / options.range_interval_s) * options.range_interval_s);
    is_boundary_guard = distance_to_boundary <= options.boundary_guard_s ...
        && time_s > options.boundary_guard_s;
    if is_boundary_guard
        % 五次平滑阶跃：交界点为80 m，离开保护区时连续恢复到30 m，
        % 一阶和二阶导数在两端均为0，避免量测权重发生硬切换。
        normalized_distance = distance_to_boundary / options.boundary_guard_s;
        smooth_weight = 10 * normalized_distance ^ 3 ...
            - 15 * normalized_distance ^ 4 + 6 * normalized_distance ^ 5;
        position_std_m = options.boundary_position_std_m ...
            + (options.rts_position_std_m ...
            - options.boundary_position_std_m) * smooth_weight;
        velocity_std_mps = options.boundary_velocity_std_mps ...
            + (options.rts_velocity_std_mps ...
            - options.boundary_velocity_std_mps) * smooth_weight;
    else
        position_std_m = options.rts_position_std_m;
        velocity_std_mps = options.rts_velocity_std_mps;
    end
end

function [kf, info] = update_horizontal_rts_position_velocity( ...
        navstate, measurement_position, measurement_velocity, kf, ...
        position_std_m, velocity_std_mps, options, position_error_unit)
%UPDATE_HORIZONTAL_RTS_POSITION_VELOCITY 联合更新水平位置和速度误差状态。
    horizontal_scale = [navstate.Rm + navstate.pos(3); ...
        (navstate.Rn + navstate.pos(3)) * cos(navstate.pos(1))];
    position_residual = navstate.pos(1:2) - measurement_position;
    if position_error_unit == "m"
        position_residual = horizontal_scale .* position_residual;
    end
    velocity_residual = navstate.vel(1:2) - measurement_velocity;
    measurement_residual = [position_residual; velocity_residual];
    state_matrix = zeros(4, kf.RANK);
    state_matrix(1:2, 1:2) = eye(2);
    state_matrix(3:4, 4:5) = eye(2);
    innovation = measurement_residual - state_matrix * kf.x;
    if position_error_unit == "m"
        innovation_m_vector = innovation(1:2);
    else
        innovation_m_vector = horizontal_scale .* innovation(1:2);
    end
    innovation_m = norm(innovation_m_vector);
    velocity_innovation_mps = norm(innovation(3:4));

    effective_std_m = position_std_m;
    effective_velocity_std_mps = velocity_std_mps;
    normalized_position_innovation = innovation_m / max(position_std_m, eps);
    normalized_velocity_innovation = velocity_innovation_mps ...
        / max(velocity_std_mps, eps);
    normalized_innovation = max(normalized_position_innovation, ...
        normalized_velocity_innovation);
    robust_downweighted = false;
    accepted = true;

    if normalized_innovation > options.robust_gate_sigma
        accepted = false;
    elseif normalized_innovation > options.innovation_gate_sigma
        effective_std_m = position_std_m * options.robust_std_scale;
        effective_velocity_std_mps = velocity_std_mps ...
            * options.robust_std_scale;
        robust_downweighted = true;
    end

    if accepted
        if position_error_unit == "m"
            measurement_std = repmat(effective_std_m, 2, 1);
        else
            measurement_std = effective_std_m ./ horizontal_scale;
        end
        measurement_covariance = diag([measurement_std .^ 2; ...
            repmat(effective_velocity_std_mps ^ 2, 2, 1)]);
        innovation_covariance = state_matrix * kf.P * state_matrix' ...
            + measurement_covariance;
        kalman_gain = kf.P * state_matrix' / innovation_covariance;
        kf.x = kf.x + kalman_gain * innovation;
        identity = eye(kf.RANK);
        kf.P = (identity - kalman_gain * state_matrix) * kf.P ...
            * (identity - kalman_gain * state_matrix)' ...
            + kalman_gain * measurement_covariance * kalman_gain';
        kf.P = (kf.P + kf.P') / 2;
    end

    info = struct('accepted', accepted, ...
        'robust_downweighted', robust_downweighted, ...
        'effective_std_m', effective_std_m, ...
        'effective_velocity_std_mps', effective_velocity_std_mps, ...
        'innovation_m', innovation_m, ...
        'velocity_innovation_mps', velocity_innovation_mps, ...
        'normalized_innovation', normalized_innovation);
end

function [kf, navstate] = feedback_replay_error( ...
        kf, navstate, position_error_unit)
%FEEDBACK_REPLAY_ERROR 将位置、速度和IMU零偏误差反馈到重放导航状态。
% 姿态反馈与当前项目15状态主链保持一致，暂不启用。
    if position_error_unit == "m"
        [kf, navstate] = myErrorFeedback_range_m(kf, navstate);
    else
        [kf, navstate] = myErrorFeedback_range(kf, navstate);
    end
end

function item = empty_update_diagnostic()
    item = struct('time_s', nan, 'position_std_m', nan, ...
        'velocity_std_mps', nan, 'in_boundary_guard', false, ...
        'innovation_m', nan, 'velocity_innovation_mps', nan, ...
        'normalized_innovation', nan, 'robust_downweighted', false, ...
        'accepted', false);
end

function row = state_to_nav_row(navstate, param)
    row = zeros(1, 11);
    row(2) = navstate.time;
    row(3:5) = [navstate.pos(1:2)' * param.R2D, navstate.pos(3)];
    row(6:8) = navstate.vel';
    row(9:11) = navstate.att' * param.R2D;
end

function write_nav_file(file_path, nav_data)
    file_id = fopen(file_path, 'w');
    if file_id < 0
        error('无法打开导航结果文件：%s', file_path);
    end
    cleaner = onCleanup(@() fclose(file_id));
    format = ['%2d %12.6f %12.8f %12.8f %8.4f %8.4f ', ...
        '%8.4f %8.4f %8.4f %8.4f %8.4f\n'];
    fprintf(file_id, format, nav_data');
    clear cleaner;
end

function radial_error = calculate_horizontal_radial_error( ...
        estimated_position, truth_position)
    latitude = deg2rad(truth_position(:, 1));
    height = truth_position(:, 3);
    [meridian_radius, prime_vertical_radius] = wgs84_radii(latitude);
    north_error = deg2rad(estimated_position(:, 1) ...
        - truth_position(:, 1)) .* (meridian_radius + height);
    east_error = deg2rad(estimated_position(:, 2) ...
        - truth_position(:, 2)) .* (prime_vertical_radius + height) ...
        .* cos(latitude);
    radial_error = hypot(north_error, east_error);
end

function [east, north] = position_to_local_plane(position, origin)
    latitude_0 = deg2rad(origin(1));
    [meridian_radius, prime_vertical_radius] = wgs84_radii(latitude_0);
    north = deg2rad(position(:, 1) - origin(1)) ...
        .* (meridian_radius + origin(3));
    east = deg2rad(position(:, 2) - origin(2)) ...
        .* (prime_vertical_radius + origin(3)) .* cos(latitude_0);
end

function [meridian_radius, prime_vertical_radius] = wgs84_radii(latitude)
    semi_major_axis = 6378137;
    flattening = 1 / 298.257223563;
    eccentricity_squared = flattening * (2 - flattening);
    denominator = sqrt(1 - eccentricity_squared .* sin(latitude) .^ 2);
    prime_vertical_radius = semi_major_axis ./ denominator;
    meridian_radius = semi_major_axis * (1 - eccentricity_squared) ...
        ./ denominator .^ 3;
end

function value = calculate_percentile(data, percentile)
    sorted_data = sort(data(:));
    position = 1 + (numel(sorted_data) - 1) * percentile / 100;
    lower_index = floor(position);
    upper_index = ceil(position);
    if lower_index == upper_index
        value = sorted_data(lower_index);
    else
        weight = position - lower_index;
        value = (1 - weight) * sorted_data(lower_index) ...
            + weight * sorted_data(upper_index);
    end
end
