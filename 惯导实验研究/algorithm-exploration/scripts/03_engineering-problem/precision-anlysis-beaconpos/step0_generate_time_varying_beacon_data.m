clear;
close all;
clc;
%% 长时时变潜标数据：仿真/实测统一生成入口
% 原range文件中的固定坐标只作为潜标的物理活动中心；
% 潜标真实初始位置在指定圆/圆环内，导航使用的名义坐标则是对该
% 真实初始位置的一次带误差测量。该名义坐标在后续时段内始终固定。
% 每个工况写入独立study_id目录，不覆盖其它误差工况。
data_source = "experiment";           % "simulation" / "experiment"
dataset_id = 'case-09';               % 数据目录名；不再限制为case-06
% data_source = "simulation";           % "simulation" / "experiment"
% dataset_id = 'case-06';               % 数据目录名；不再限制为case-06
start_time_s = [];                    % []：首个公共range时刻
generation_duration_s = 24*3600;      % []：使用全部公共range时段
motion_region_mode = 'circle';       % 'circle' / 'annulus'
activity_radius_m = 61;              % circle: 0~R

annulus_inner_radius_m = 40;         % annulus
annulus_outer_radius_m = 200;
initial_measurement_error_max_m = 10; % 导航名义坐标相对真实初始点的最大水平误差
velocity_std_mps = 0.025/2;             % 随机运动的典型速度尺度；越大，24 h内位移越活跃
maximum_speed_mps = 0.08;             % 瞬时速度硬上限
velocity_correlation_time_s = 3000;    % 越小转向越频繁；越大轨迹越平缓
random_seed = 20260908;
overwrite_existing = true;
show_figures = true;
beacon_initial_position_std_m = 5;
beacon_24h_position_std_m = 60;
beacon_uncertainty_horizon_s = 24*3600;
switch lower(motion_region_mode)
    case 'circle'
        if activity_radius_m <= 0, error('activity_radius_m必须>0。'); end
        region_inner_radius_m = 0;
        region_outer_radius_m = activity_radius_m;
        study_id = sprintf( ...
            'beacon-position-time-varying-circle-%gm-initial%gm', ...
            activity_radius_m, initial_measurement_error_max_m);
    case 'annulus'
        if annulus_inner_radius_m < 0 || annulus_outer_radius_m <= annulus_inner_radius_m
            error('圆环内外半径设置错误。');
        end
        region_inner_radius_m = annulus_inner_radius_m;
        region_outer_radius_m = annulus_outer_radius_m;
        study_id = sprintf( ...
            'beacon-position-time-varying-annulus-%g-%gm-initial%gm', ...
            region_inner_radius_m, region_outer_radius_m, ...
            initial_measurement_error_max_m);
    otherwise
        error('motion_region_mode只能为circle或annulus。');
end
%% 路径
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));
addpath(topic_dir);
paths = setup_inertial_experiment();
glvs;
dataset = resolve_beacon_position_dataset(data_source, dataset_id, "rad");
case_input_dir = dataset.input_dir;
study_input_dir = fullfile(case_input_dir,study_id);
artifact_dir = fullfile(dataset.artifact_dir,study_id);
if ~isfolder(study_input_dir), mkdir(study_input_dir); end
if ~isfolder(artifact_dir), mkdir(artifact_dir); end
motion_figure_path = fullfile(artifact_dir,sprintf('%s-%s-%gm-initial%gm-time-varying-beacon-position.png',dataset_id,motion_region_mode,region_outer_radius_m,initial_measurement_error_max_m));
motion_figure_file_path = fullfile(artifact_dir,sprintf('%s-%s-%gm-initial%gm-time-varying-beacon-position.fig',dataset_id,motion_region_mode,region_outer_radius_m,initial_measurement_error_max_m));
uncertainty_figure_path = fullfile(artifact_dir,sprintf('%s-%g-beacon-position-uncertainty-growth.png',dataset_id,beacon_24h_position_std_m));
uncertainty_figure_file_path = fullfile(artifact_dir,sprintf('%s-%g-beacon-position-uncertainty-growth.fig',dataset_id,beacon_24h_position_std_m));
source_truth_path = dataset.cfg.truthpath;
source_range_paths = strings(3,1);
range_output_paths = strings(3,1);
beacon_truth_paths = strings(3,1);
for i = 1:3
    source_range_paths(i) = fullfile(case_input_dir,sprintf('range%d.txt',i));
    range_output_paths(i) = fullfile(study_input_dir,sprintf('range%d.txt',i));
    beacon_truth_paths(i) = fullfile(study_input_dir,sprintf('beacon%d-position-truth.txt',i));
end
if ~isfile(source_truth_path) || any(~isfile(source_range_paths)), error('truth文件或range1~3.txt不完整：%s',case_input_dir); end
if velocity_std_mps <= 0 || maximum_speed_mps <= 0 || velocity_correlation_time_s <= 0, error('速度参数必须>0。'); end
if beacon_initial_position_std_m <= 0 || ...
        beacon_24h_position_std_m < beacon_initial_position_std_m || ...
        beacon_uncertainty_horizon_s <= 0
    error('潜标位置不确定度参数无效。');
end
if initial_measurement_error_max_m < 0
    error('初始位置最大误差不能为负数。');
end
context_path = fullfile(study_input_dir,'generation-context.mat');
core_paths = [range_output_paths;beacon_truth_paths;string(context_path)];
existing_mask = isfile(core_paths);
%% 已存在则直接重画
if all(existing_mask) && ~overwrite_existing
    d = readmatrix(beacon_truth_paths(1),'FileType','text');
    if size(d,2) ~= 13
        error('已有潜标真值不是version 3格式，请设置overwrite_existing=true后重建。');
    end
    time_s = d(:,1);
    physical_offset_enu_m = zeros(numel(time_s),3,3);
    navigation_error_enu_m = zeros(numel(time_s),3,3);
    navigation_error_radius_m = zeros(numel(time_s),3);
    for i = 1:3
        d = readmatrix(beacon_truth_paths(i),'FileType','text');
        if size(d,2) ~= 13 || size(d,1) ~= numel(time_s) || any(abs(d(:,1)-time_s) > 1e-8)
            error('已有潜标%d真值格式或时间轴错误。',i);
        end
        physical_offset_enu_m(:,:,i) = d(:,5:7);
        navigation_error_enu_m(:,:,i) = d(:,10:12);
        navigation_error_radius_m(:,i) = d(:,13);
    end
    loaded_context = load(context_path,'generation_context');
    generation_context = loaded_context.generation_context;
    if ~isfield(generation_context,'version') || generation_context.version < 4 || ...
            ~isfield(generation_context,'navigation_nominal_offset_enu_m')
        error('已有generation-context.mat不是version 4格式，请设置overwrite_existing=true后重建。');
    end
    if ~strcmp(generation_context.data_source,dataset.data_source) || ...
            ~strcmp(generation_context.dataset_id,dataset.dataset_id)
        error('已有数据的数据来源或数据集编号与当前配置不一致。');
    end
    motion_parameter_names = {'region_inner_radius_m','region_outer_radius_m', ...
        'initial_measurement_error_max_m','velocity_std_mps', ...
        'maximum_speed_mps','velocity_correlation_time_s','random_seed'};
    motion_parameter_values = [region_inner_radius_m,region_outer_radius_m, ...
        initial_measurement_error_max_m,velocity_std_mps,maximum_speed_mps, ...
        velocity_correlation_time_s,random_seed];
    for parameter_index = 1:numel(motion_parameter_names)
        parameter_name = motion_parameter_names{parameter_index};
        if ~isfield(generation_context,parameter_name) || ...
                abs(generation_context.(parameter_name)- ...
                motion_parameter_values(parameter_index)) > 1e-12
            error(['已有数据的运动参数与脚本顶部配置不一致（%s）。' ...
                '请设置overwrite_existing=true后重建。'],parameter_name);
        end
    end
    plot_time_varying_beacons(time_s,physical_offset_enu_m, ...
        navigation_error_enu_m,navigation_error_radius_m, ...
        generation_context.navigation_nominal_offset_enu_m, ...
        motion_region_mode,region_inner_radius_m,region_outer_radius_m, ...
        motion_figure_path,motion_figure_file_path,show_figures);
    plot_beacon_uncertainty_growth(time_s,beacon_initial_position_std_m,beacon_24h_position_std_m,beacon_uncertainty_horizon_s,uncertainty_figure_path,uncertainty_figure_file_path,show_figures);
    generation_context.beacon_initial_position_std_m = beacon_initial_position_std_m;
    generation_context.beacon_24h_position_std_m = beacon_24h_position_std_m;
    generation_context.beacon_uncertainty_horizon_s = beacon_uncertainty_horizon_s;
    generation_context.beacon_variance_growth_m2_per_day = ...
        (beacon_24h_position_std_m^2-beacon_initial_position_std_m^2)/ ...
        (beacon_uncertainty_horizon_s/86400);
    generation_context.motion_figure_path = motion_figure_path;
    generation_context.uncertainty_figure_path = uncertainty_figure_path;
    save(context_path,'generation_context');
    fprintf('数据已存在，未覆盖：%s\n',study_input_dir);
    fprintf('潜标运动图：%s\n',motion_figure_path);
    fprintf('不确定度增长图：%s\n',uncertainty_figure_path);
    return;
end
if any(existing_mask) && ~overwrite_existing, error('仅存在部分旧文件，请设置overwrite_existing=true后重建。'); end
%% 读取原始距离并选择生成时段
range_sources = cell(3,1);
for i = 1:3
    range_sources{i} = readmatrix(source_range_paths(i),'FileType','text');
    if size(range_sources{i},2) < 6 || any(~isfinite(range_sources{i}),'all'), error('range%d.txt格式错误。',i); end
end
range_time_s = range_sources{1}(:,1);
for i = 2:3
    if size(range_sources{i},1) ~= numel(range_time_s) || any(abs(range_sources{i}(:,1)-range_time_s) > 1e-8), error('三路range时间轴不一致。'); end
end
sample_interval_s = median(diff(range_time_s));
if sample_interval_s <= 0 || any(diff(range_time_s) <= 0) || ...
        any(abs(diff(range_time_s)-sample_interval_s) > 1e-6)
    error('三路range数据必须采用一致的等间隔递增时间轴。');
end
range_available_start_s = range_time_s(1);
range_available_end_s = range_time_s(end);
[truth_available_start_s,truth_available_end_s] = ...
    read_truth_time_bounds(source_truth_path);
available_start_s = max(range_available_start_s,truth_available_start_s);
available_end_s = min(range_available_end_s,truth_available_end_s);
if available_start_s >= available_end_s
    error(['range与truth没有足够的公共时间段：range %.3f~%.3f s，' ...
        'truth %.3f~%.3f s。'],range_available_start_s, ...
        range_available_end_s,truth_available_start_s, ...
        truth_available_end_s);
end
if isempty(start_time_s), start_time_s = available_start_s; end
if isempty(generation_duration_s)
    requested_end_time_s = available_end_s;
    duration_was_truncated = false;
    end_time_s = available_end_s;
else
    validateattributes(generation_duration_s, {'numeric'}, ...
        {'scalar', 'positive'});
    requested_end_time_s = start_time_s+generation_duration_s;
    end_time_s = min(requested_end_time_s, available_end_s);
    duration_was_truncated = requested_end_time_s > available_end_s+1e-8;
end
if start_time_s < available_start_s || start_time_s >= end_time_s
    error('所选生成时段不在range数据公共时间范围内。');
end
time_mask = range_time_s >= start_time_s & range_time_s <= end_time_s;
for i = 1:3
    range_sources{i} = range_sources{i}(time_mask,:);
end
range_time_s = range_sources{1}(:,1);
if isempty(range_time_s)
    error('range与truth公共时间段内没有测距采样。');
end
if duration_was_truncated
    warning(['请求生成%.2f h数据，但%s/%s只有%.2f h公共数据可用；' ...
        '本次输出最后一个测距采样为%.2f s。'], ...
        generation_duration_s/3600,dataset.data_source, ...
        dataset.dataset_id,(available_end_s-start_time_s)/3600, ...
        range_time_s(end));
end
%% 读取载体真值与潜标物理活动中心
fprintf('正在抽取%s/%s真值中的载体位置……\n', ...
    dataset.data_source,dataset.dataset_id);
[platform_position_rad_m,origin_position_rad_m] = read_truth_at_times(source_truth_path,range_time_s);
anchor_center_position_rad_m = zeros(3,3);
for i = 1:3
    anchor_center_position_rad_m(i,:) = range_sources{i}(1,4:6);
    if any(max(abs(range_sources{i}(:,4:6)-anchor_center_position_rad_m(i,:)),[],1) > 1e-10)
        error('range%d.txt中的潜标活动中心不是固定值。',i);
    end
end
anchor_center_enu_m = pos2dxyz(anchor_center_position_rad_m,origin_position_rad_m');
%% 生成时变潜标
rng(random_seed,'twister');
sample_count = numel(range_time_s);
physical_offset_enu_m = zeros(sample_count,3,3);
velocity_enu_mps = zeros(sample_count,3,3);
actual_beacon_position_rad_m = zeros(sample_count,3,3);
physical_radius_m = zeros(sample_count,3);
navigation_nominal_position_rad_m = zeros(3,3);
navigation_nominal_enu_m = zeros(3,3);
navigation_nominal_offset_enu_m = zeros(3,3);
initial_measurement_error_enu_m = zeros(3,3);
initial_measurement_error_radius_m = zeros(3,1);
navigation_error_enu_m = zeros(sample_count,3,3);
navigation_error_radius_m = zeros(sample_count,3);
for i = 1:3
    [horizontal_offset_m,horizontal_velocity_mps] = simulate_bounded_random_motion(sample_count,sample_interval_s,region_inner_radius_m,region_outer_radius_m,velocity_std_mps,maximum_speed_mps,velocity_correlation_time_s);
    physical_offset_enu_m(:,1:2,i) = horizontal_offset_m;
    velocity_enu_mps(:,1:2,i) = horizontal_velocity_mps;
    physical_radius_m(:,i) = vecnorm(horizontal_offset_m,2,2);
    actual_beacon_enu_m = repmat(anchor_center_enu_m(i,:),sample_count,1)+physical_offset_enu_m(:,:,i);
    actual_beacon_position_rad_m(:,:,i) = dxyz2pos(actual_beacon_enu_m,origin_position_rad_m');

    % 初始时刻对潜标坐标进行一次测量；误差在圆内按面积均匀抽样。
    initial_measurement_error_radius_m(i) = initial_measurement_error_max_m*sqrt(rand());
    initial_measurement_error_angle_rad = 2*pi*rand();
    initial_measurement_error_enu_m(i,1:2) = initial_measurement_error_radius_m(i)* ...
        [cos(initial_measurement_error_angle_rad),sin(initial_measurement_error_angle_rad)];
    navigation_nominal_enu_m(i,:) = actual_beacon_enu_m(1,:)+initial_measurement_error_enu_m(i,:);
    navigation_nominal_position_rad_m(i,:) = dxyz2pos(navigation_nominal_enu_m(i,:),origin_position_rad_m');
    navigation_nominal_offset_enu_m(i,:) = navigation_nominal_enu_m(i,:)-anchor_center_enu_m(i,:);
    navigation_error_enu_m(:,:,i) = actual_beacon_enu_m-repmat(navigation_nominal_enu_m(i,:),sample_count,1);
    navigation_error_radius_m(:,i) = vecnorm(navigation_error_enu_m(:,1:2,i),2,2);

    % 与myRangeUpdate使用完全相同的“潜标处局部曲率”水平距离模型，
    % 避免统一ENU原点在十几公里基线下引入额外几何偏差。
    true_horizontal_range_m = calculate_filter_horizontal_range( ...
        platform_position_rad_m,actual_beacon_position_rad_m(:,:,i));
    moving_range = range_sources{i};
    moving_range(:,2:3) = [true_horizontal_range_m,true_horizontal_range_m];
    moving_range(:,4:6) = repmat(navigation_nominal_position_rad_m(i,:),sample_count,1);
    temporary_range_path = range_output_paths(i)+".generating";
    temporary_truth_path = beacon_truth_paths(i)+".generating";
    writematrix(moving_range,temporary_range_path,'FileType','text','Delimiter',' ');
    speed_mps = vecnorm(horizontal_velocity_mps,2,2);
    beacon_truth_output = [range_time_s,actual_beacon_position_rad_m(:,:,i), ...
        physical_offset_enu_m(:,:,i),physical_radius_m(:,i),speed_mps, ...
        navigation_error_enu_m(:,:,i),navigation_error_radius_m(:,i)];
    writematrix(beacon_truth_output,temporary_truth_path,'FileType','text','Delimiter',' ');
end
%% 校验并提交
for i = 1:3
    validate_generated_files(range_output_paths(i)+".generating", ...
        beacon_truth_paths(i)+".generating",sample_count, ...
        range_time_s(1),range_time_s(end), ...
        region_inner_radius_m,region_outer_radius_m, ...
        initial_measurement_error_max_m,navigation_nominal_position_rad_m(i,:));
end
for i = 1:3
    movefile(range_output_paths(i)+".generating",range_output_paths(i),'f');
    movefile(beacon_truth_paths(i)+".generating",beacon_truth_paths(i),'f');
end
%% 保存配置
generation_context = struct();
generation_context.version = 4;
generation_context.study_id = study_id;
generation_context.data_source = dataset.data_source;
generation_context.dataset_id = dataset.dataset_id;
generation_context.start_time_s = range_time_s(1);
generation_context.end_time_s = range_time_s(end);
generation_context.duration_s = range_time_s(end)-range_time_s(1);
generation_context.requested_duration_s = generation_duration_s;
generation_context.requested_end_time_s = requested_end_time_s;
generation_context.duration_was_truncated = duration_was_truncated;
generation_context.source_truth_start_time_s = truth_available_start_s;
generation_context.source_truth_end_time_s = truth_available_end_s;
generation_context.common_available_start_time_s = available_start_s;
generation_context.common_available_end_time_s = available_end_s;
generation_context.sample_interval_s = sample_interval_s;
generation_context.motion_region_mode = motion_region_mode;
generation_context.region_inner_radius_m = region_inner_radius_m;
generation_context.region_outer_radius_m = region_outer_radius_m;
generation_context.activity_radius_m = region_outer_radius_m;
generation_context.velocity_std_mps = velocity_std_mps;
generation_context.maximum_speed_mps = maximum_speed_mps;
generation_context.velocity_correlation_time_s = velocity_correlation_time_s;
generation_context.random_seed = random_seed;
generation_context.anchor_center_position_rad_m = anchor_center_position_rad_m;
generation_context.anchor_center_enu_m = anchor_center_enu_m;
generation_context.navigation_nominal_position_rad_m = navigation_nominal_position_rad_m;
generation_context.navigation_nominal_enu_m = navigation_nominal_enu_m;
generation_context.navigation_nominal_offset_enu_m = navigation_nominal_offset_enu_m;
generation_context.initial_measurement_error_max_m = initial_measurement_error_max_m;
generation_context.initial_measurement_error_enu_m = initial_measurement_error_enu_m;
generation_context.initial_measurement_error_radius_m = initial_measurement_error_radius_m;
generation_context.minimum_observed_physical_radius_m = min(physical_radius_m,[],1);
generation_context.maximum_observed_physical_radius_m = max(physical_radius_m,[],1);
generation_context.minimum_observed_navigation_error_m = min(navigation_error_radius_m,[],1);
generation_context.maximum_observed_navigation_error_m = max(navigation_error_radius_m,[],1);
generation_context.beacon_truth_columns = { ...
    'time_s','latitude_rad','longitude_rad','height_m', ...
    'physical_east_offset_m','physical_north_offset_m','physical_up_offset_m', ...
    'physical_radius_m','speed_mps', ...
    'navigation_error_east_m','navigation_error_north_m', ...
    'navigation_error_up_m','navigation_error_radius_m'};
generation_context.range_geometry_model = ...
    'myRangeUpdate-local-tangent-at-beacon';
generation_context.source_truth_path = source_truth_path;
generation_context.source_range_paths = source_range_paths;
generation_context.dataset_input_dir = dataset.input_dir;
generation_context.range_output_paths = range_output_paths;
generation_context.beacon_truth_paths = beacon_truth_paths;
generation_context.beacon_initial_position_std_m = beacon_initial_position_std_m;
generation_context.beacon_24h_position_std_m = beacon_24h_position_std_m;
generation_context.beacon_uncertainty_horizon_s = beacon_uncertainty_horizon_s;
generation_context.beacon_variance_growth_m2_per_day = (beacon_24h_position_std_m^2-beacon_initial_position_std_m^2)/(beacon_uncertainty_horizon_s/86400);
generation_context.motion_figure_path = motion_figure_path;
generation_context.uncertainty_figure_path = uncertainty_figure_path;
save(context_path,'generation_context');
%% 绘图
plot_time_varying_beacons(range_time_s,physical_offset_enu_m, ...
    navigation_error_enu_m,navigation_error_radius_m, ...
    navigation_nominal_offset_enu_m,motion_region_mode, ...
    region_inner_radius_m,region_outer_radius_m,motion_figure_path, ...
    motion_figure_file_path,show_figures);
% plot_beacon_uncertainty_growth(range_time_s,beacon_initial_position_std_m, ...
%     beacon_24h_position_std_m,beacon_uncertainty_horizon_s, ...
%     uncertainty_figure_path,uncertainty_figure_file_path,show_figures);
fprintf('\n生成完成：%s\n',study_input_dir);
fprintf('实际生成时段：%.2f~%.2f s（%.2f h）。\n', ...
    range_time_s(1),range_time_s(end), ...
    (range_time_s(end)-range_time_s(1))/3600);
fprintf('活动区域：%s，%.1f~%.1f m\n',motion_region_mode,region_inner_radius_m,region_outer_radius_m);
for i = 1:3
    fprintf(['Beacon%d：物理半径 %.2f~%.2f m；初始测量误差 %.2f m；' ...
        '相对固定名义坐标误差 %.2f~%.2f m\n'],i, ...
        min(physical_radius_m(:,i)),max(physical_radius_m(:,i)), ...
        initial_measurement_error_radius_m(i), ...
        min(navigation_error_radius_m(:,i)),max(navigation_error_radius_m(:,i)));
end

function horizontal_range_m = calculate_filter_horizontal_range( ...
        platform_position_rad_m,beacon_position_rad_m)
%CALCULATE_FILTER_HORIZONTAL_RANGE 复现myRangeUpdate的水平距离预测模型。
a = 6378137.0;
e2 = 6.69437999014e-3;
beacon_latitude_rad = beacon_position_rad_m(:,1);
beacon_height_m = beacon_position_rad_m(:,3);
denominator = sqrt(1-e2*sin(beacon_latitude_rad).^2);
rn_m = a./denominator;
rm_m = a*(1-e2)./denominator.^3;
north_m = (platform_position_rad_m(:,1)-beacon_latitude_rad).* ...
    (rm_m+beacon_height_m);
east_m = (platform_position_rad_m(:,2)-beacon_position_rad_m(:,2)).* ...
    (rn_m+beacon_height_m).*cos(beacon_latitude_rad);
horizontal_range_m = hypot(north_m,east_m);
end

function [positions_rad_m,origin_rad_m] = read_truth_at_times(truth_path,target_time_s)
fid = fopen(truth_path,'rb');
if fid < 0, error('无法读取：%s',truth_path); end
cleanup = onCleanup(@() fclose(fid));
line1 = fgetl(fid); record_bytes = ftell(fid);
line2 = fgetl(fid); record_bytes2 = ftell(fid)-record_bytes;
v1 = sscanf(line1,'%f')'; v2 = sscanf(line2,'%f')';
if numel(v1) < 5 || numel(v2) < 5, error('真值文件首两行格式错误。'); end
origin_rad_m = [deg2rad(v1(3:4)),v1(5)];
dt = v2(2)-v1(2);
rows = round((target_time_s-v1(2))/dt)+1;
fixed_record = dt > 0 && all(rows >= 1) && ...
    all(abs(v1(2)+(rows-1)*dt-target_time_s) <= 1e-7) && ...
    record_bytes > 0 && record_bytes2 == record_bytes;
if fixed_record
    fseek(fid,0,'eof');
    file_bytes = ftell(fid);
    fixed_record = mod(file_bytes,record_bytes) == 0 && ...
        file_bytes/record_bytes >= rows(end);
end
if fixed_record
    positions_rad_m = zeros(numel(target_time_s),3);
    for k = 1:numel(rows)
        if fseek(fid,(rows(k)-1)*record_bytes,'bof') ~= 0
            error('无法定位真值文件第%d行。',rows(k));
        end
        line = fgetl(fid);
        v = sscanf(line,'%f')';
        if numel(v) < 5 || abs(v(2)-target_time_s(k)) > 1e-7
            error('真值文件第%d行时间不匹配。',rows(k));
        end
        positions_rad_m(k,:) = [deg2rad(v(3:4)),v(5)];
    end
    return;
end

% 实测文件未必是定长文本；退化为单遍顺序读取，避免整文件载入内存。
fseek(fid,0,'bof');
positions_rad_m = zeros(numel(target_time_s),3);
target_index = 1;
while target_index <= numel(target_time_s)
    line = fgetl(fid);
    if ~ischar(line), break; end
    v = sscanf(line,'%f')';
    if numel(v) < 5, continue; end
    if abs(v(2)-target_time_s(target_index)) <= 1e-7
        positions_rad_m(target_index,:) = [deg2rad(v(3:4)),v(5)];
        target_index = target_index+1;
    elseif v(2) > target_time_s(target_index)+1e-7
        error('真值文件缺少时刻%.9f。',target_time_s(target_index));
    end
end
if target_index <= numel(target_time_s)
    error('真值文件在时刻%.9f前提前结束。',target_time_s(target_index));
end
end
function [first_time_s,last_time_s] = read_truth_time_bounds(truth_path)
%READ_TRUTH_TIME_BOUNDS 不加载整份真值，读取首尾有效记录的时间。
fid = fopen(truth_path,'rb');
if fid < 0, error('无法读取：%s',truth_path); end
cleanup = onCleanup(@() fclose(fid));

first_time_s = nan;
while ~feof(fid)
    line = fgetl(fid);
    if ~ischar(line), break; end
    values = sscanf(line,'%f')';
    if numel(values) >= 2 && isfinite(values(2))
        first_time_s = values(2);
        break;
    end
end

fseek(fid,0,'eof');
file_bytes = ftell(fid);
tail_bytes = min(file_bytes,1024*1024);
fseek(fid,file_bytes-tail_bytes,'bof');
tail_text = fread(fid,tail_bytes,'*char')';
tail_lines = regexp(tail_text,'\r\n|\n|\r','split');
last_time_s = nan;
for line_index = numel(tail_lines):-1:1
    values = sscanf(tail_lines{line_index},'%f')';
    if numel(values) >= 2 && isfinite(values(2))
        last_time_s = values(2);
        break;
    end
end
if ~isfinite(first_time_s) || ~isfinite(last_time_s) || ...
        last_time_s < first_time_s
    error('无法确定真值文件的有效时间范围：%s',truth_path);
end
end
function [offset_m,velocity_mps] = simulate_bounded_random_motion(sample_count,dt_s,inner_radius_m,outer_radius_m,velocity_std_mps,maximum_speed_mps,correlation_time_s)
offset_m = zeros(sample_count,2);
velocity_mps = zeros(sample_count,2);
r0 = sqrt(inner_radius_m^2+(outer_radius_m^2-inner_radius_m^2)*rand());
a0 = 2*pi*rand();
offset_m(1,:) = r0*[cos(a0),sin(a0)];
velocity_mps(1,:) = 0.25*velocity_std_mps*randn(1,2);
rho = exp(-dt_s/correlation_time_s);
innovation_std = velocity_std_mps*sqrt(1-rho^2);
for k = 2:sample_count
    v = rho*velocity_mps(k-1,:)+innovation_std*randn(1,2);
    s = norm(v);
    if s > maximum_speed_mps, v = v*(maximum_speed_mps/s); end
    p = offset_m(k-1,:)+v*dt_s;
    [p,v] = reflect_boundary(p,v,inner_radius_m,outer_radius_m);
    offset_m(k,:) = p;
    velocity_mps(k,:) = v;
end
end
function [p,v] = reflect_boundary(p,v,rmin,rmax)
r = norm(p);
if r > rmax
    u = p/max(r,eps);
    r = max(rmin,min(rmax,2*rmax-r));
    p = r*u;
    v = v-2*dot(v,u)*u;
end
r = norm(p);
if rmin > 0 && r < rmin
    if r < eps, u = [1,0]; else, u = p/r; end
    r = min(rmax,max(rmin,2*rmin-r));
    p = r*u;
    v = v-2*dot(v,u)*u;
end
end
function validate_generated_files(range_path,truth_path,sample_count,first_time_s, ...
        final_time_s, ...
        rmin,rmax,initial_error_max_m,navigation_nominal_position)
range_data = readmatrix(range_path,'FileType','text');
truth_data = readmatrix(truth_path,'FileType','text');
if ~isequal(size(range_data),[sample_count,6]) || ...
        ~isequal(size(truth_data),[sample_count,13]) || ...
        any(~isfinite(range_data),'all') || any(~isfinite(truth_data),'all')
    error('生成文件维度或数值错误。');
end
if abs(range_data(1,1)-first_time_s) > 1e-8 || ...
        abs(range_data(end,1)-final_time_s) > 1e-8 || ...
        any(diff(range_data(:,1)) <= 0) || ...
        any(abs(range_data(:,2)-range_data(:,3)) > 1e-9)
    error('距离时间轴或理想距离错误。');
end
if any(max(abs(range_data(:,4:6)-navigation_nominal_position),[],1) > 1e-10)
    error('导航使用的名义潜标坐标发生变化。');
end
physical_radius = truth_data(:,8);
if min(physical_radius) < rmin-1e-7 || max(physical_radius) > rmax+1e-7
    error('潜标真实位置超出物理活动区域。');
end
if truth_data(1,13) > initial_error_max_m+1e-7
    error('潜标初始测量误差超过设定上限。');
end
end

%%
function plot_time_varying_beacons(time_s,physical_offset_enu_m,navigation_error_enu_m,navigation_error_radius_m,navigation_nominal_offset_enu_m,mode,rmin,rmax,png_path,fig_path,show_figure)
if show_figure, visibility='on'; else, visibility='off'; end
colors=lines(3);
elapsed_time_s = time_s-time_s(1);
stride=max(1,floor(numel(time_s)/12000));
idx=1:stride:numel(time_s);
a=linspace(0,2*pi,721);
fig=myfigurestartup(7,7,'paper');
set(fig,'Visible',visibility);
tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile; hold on;
hOuter=plot(rmax*cos(a),rmax*sin(a),'k--','LineWidth',1.2);
if rmin>0, hInner=plot(rmin*cos(a),rmin*sin(a),'k:','LineWidth',1.2); else, hInner=gobjects(0); end
hTraj=gobjects(3,1); hNominal=gobjects(3,1);
for i=1:3
    hTraj(i)=plot(physical_offset_enu_m(idx,1,i),physical_offset_enu_m(idx,2,i),'Color',colors(i,:),'LineWidth',0.8);
    plot(physical_offset_enu_m(1,1,i),physical_offset_enu_m(1,2,i),'o','Color',colors(i,:),'MarkerFaceColor',colors(i,:),'HandleVisibility','off');
    hNominal(i)=plot(navigation_nominal_offset_enu_m(i,1),navigation_nominal_offset_enu_m(i,2),'x','Color',colors(i,:),'MarkerSize',9,'LineWidth',1.5);
end
plot(0,0,'k+','MarkerSize',10,'LineWidth',1.5,'HandleVisibility','off');
plot_limit=max(rmax,max(vecnorm(navigation_nominal_offset_enu_m(:,1:2),2,2)));
axis equal; grid on;
xlim(1.10*plot_limit*[-1,1]); ylim([-1.10*plot_limit,1.42*plot_limit]);
xlabel('East offset (m)'); ylabel('North offset (m)');
if strcmpi(mode,'circle')
    title(sprintf('Physical motion inside %.0f m radius',rmax));
    legend([hOuter;hTraj;hNominal(1)],{'Boundary','Beacon 1','Beacon 2','Beacon 3','Fixed initial coordinates'},'Location','north','NumColumns',2,'FontSize',8);
else
    title(sprintf('Physical motion in %.0f~%.0f m annulus',rmin,rmax));
    legend([hOuter;hInner;hTraj;hNominal(1)],{'Outer','Inner','Beacon 1','Beacon 2','Beacon 3','Fixed initial coordinates'},'Location','north','NumColumns',2,'FontSize',8);
end
nexttile; hold on;
for i=1:3, plot(elapsed_time_s/3600,navigation_error_radius_m(:,i),'Color',colors(i,:),'LineWidth',0.8); end
grid on; xlabel('Time (h)'); ylabel('Error radius (m)'); title('Error relative to fixed initial coordinates');
nexttile; hold on;
for i=1:3, plot(elapsed_time_s/3600,navigation_error_enu_m(:,1,i),'Color',colors(i,:),'LineWidth',0.7); end
grid on; xlabel('Time (h)'); ylabel('East error (m)'); title('East error relative to fixed coordinates');
nexttile; hold on;
for i=1:3, plot(elapsed_time_s/3600,navigation_error_enu_m(:,2,i),'Color',colors(i,:),'LineWidth',0.7); end
grid on; xlabel('Time (h)'); ylabel('North error (m)'); title('North error relative to fixed coordinates');
title(tl,sprintf('Physical beacon motion and fixed coordinates (%.1f h, %s)', ...
    elapsed_time_s(end)/3600,mode));
exportgraphics(fig,png_path,'Resolution',600);
savefig(fig,fig_path);
if ~show_figure, close(fig); end
end
% function plot_time_varying_beacons(time_s,physical_offset_enu_m, ...
%         navigation_error_enu_m,navigation_error_radius_m, ...
%         navigation_nominal_offset_enu_m,mode,rmin,rmax,png_path,fig_path,show_figure)
% if show_figure
%     visibility = 'on';
% else
%     visibility = 'off';
% end
% colors = lines(3);
% stride = max(1,floor(numel(time_s)/12000));
% idx = 1:stride:numel(time_s);
% a = linspace(0,2*pi,721);
% fig = myfigurestartup(10,7,'prese');
% set(fig,'Visible',visibility);
% tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
% nexttile; hold on;
% hOuter = plot(rmax*cos(a),rmax*sin(a),'k--','LineWidth',1.2);
% if rmin > 0, hInner = plot(rmin*cos(a),rmin*sin(a),'k:','LineWidth',1.2); else, hInner = gobjects(0); end
% hTraj = gobjects(3,1);
% hNominal = gobjects(3,1);
% for i = 1:3
%     hTraj(i) = plot(physical_offset_enu_m(idx,1,i), ...
%         physical_offset_enu_m(idx,2,i),'Color',colors(i,:),'LineWidth',0.8);
%     plot(physical_offset_enu_m(1,1,i),physical_offset_enu_m(1,2,i), ...
%         'o','Color',colors(i,:),'MarkerFaceColor',colors(i,:), ...
%         'HandleVisibility','off');
%     hNominal(i) = plot(navigation_nominal_offset_enu_m(i,1), ...
%         navigation_nominal_offset_enu_m(i,2),'x','Color',colors(i,:), ...
%         'MarkerSize',9,'LineWidth',1.5);
% end
% plot(0,0,'k+','MarkerSize',10,'LineWidth',1.5,'HandleVisibility','off');
% plot_limit = max(rmax,max(vecnorm(navigation_nominal_offset_enu_m(:,1:2),2,2)));
% axis equal; 
% grid on; 
% % xlim(1.08*plot_limit*[-1,1]); ylim(1.08*plot_limit*[-1,1]);
% xlabel('East offset (m)'); ylabel('North offset (m)');
% if strcmpi(mode,'circle')
%     title(sprintf('Physical motion inside %.0f m radius',rmax));
%     legend([hOuter;hTraj;hNominal(1)], ...
%         {'Boundary','Beacon 1','Beacon 2','Beacon 3','Fixed initial coordinates'}, ...
%         'Location','best');
% else
%     title(sprintf('Physical motion in %.0f~%.0f m annulus',rmin,rmax));
%     legend([hOuter;hInner;hTraj;hNominal(1)], ...
%         {'Outer','Inner','Beacon 1','Beacon 2','Beacon 3','Fixed initial coordinates'}, ...
%         'Location','best');
% end
% nexttile; hold on;
% for i = 1:3
%     plot(time_s/3600,navigation_error_radius_m(:,i), ...
%         'Color',colors(i,:),'LineWidth',0.8);
% end
% grid on; xlabel('Time (h)'); ylabel('Error radius (m)');
% title('Error relative to fixed initial coordinates');
% nexttile; hold on;
% for i = 1:3
%     plot(time_s/3600,navigation_error_enu_m(:,1,i), ...
%         'Color',colors(i,:),'LineWidth',0.7);
% end
% grid on; xlabel('Time (h)'); ylabel('East error (m)');
% title('East error relative to fixed coordinates');
% nexttile; hold on;
% for i = 1:3
%     plot(time_s/3600,navigation_error_enu_m(:,2,i), ...
%         'Color',colors(i,:),'LineWidth',0.7);
% end
% grid on; xlabel('Time (h)'); ylabel('North error (m)');
% title('North error relative to fixed coordinates');
% title(tl,sprintf('case-06 physical beacon motion and fixed coordinates (24 h, %s)',mode));
% exportgraphics(fig,png_path,'Resolution',600);
% savefig(fig,fig_path);
% if ~show_figure, close(fig); end
% end
%%
function plot_beacon_uncertainty_growth(time_s,initial_std_m,final_std_m,horizon_s,png_path,fig_path,show_figure)
if show_figure
    visibility = 'on';
else
    visibility = 'off';
end
elapsed_time_s = time_s-time_s(1);
age_ratio = min(max(elapsed_time_s,0),horizon_s)/horizon_s;
variance_m2 = initial_std_m^2+(final_std_m^2-initial_std_m^2).*age_ratio;
std_m = sqrt(variance_m2);
fig = myfigurestartup(8,5,'prese');
set(fig,'Visible',visibility);
tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
nexttile; plot(elapsed_time_s/3600,std_m,'b-','LineWidth',1.8); grid on; xlim([0,max(elapsed_time_s(end),horizon_s)/3600]); ylim([0,1.05*final_std_m]); xlabel('Time (h)'); ylabel('1\sigma (m)'); title('Beacon position standard deviation');
nexttile; plot(elapsed_time_s/3600,variance_m2,'r-','LineWidth',1.8); grid on; xlim([0,max(elapsed_time_s(end),horizon_s)/3600]); ylim([0,1.05*final_std_m^2]); xlabel('Time (h)'); ylabel('Variance (m^2)'); title('Linear growth of beacon position variance');
title(tl,sprintf('Beacon uncertainty: %.1f m to %.1f m in 24 h',initial_std_m,final_std_m));
exportgraphics(fig,png_path,'Resolution',600);
savefig(fig,fig_path);
if ~show_figure, close(fig); end
end
