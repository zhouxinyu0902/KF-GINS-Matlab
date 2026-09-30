function generated = generate_experiment_legacy_inputs( ...
        dataset_id, overwrite_existing, dry_run)
%GENERATE_EXPERIMENT_LEGACY_INPUTS 为新实测数据集生成旧算法兼容文件。
%   generate_experiment_legacy_inputs() 默认处理 case-07。
%   generate_experiment_legacy_inputs('case-08') 可处理后续 case-0x。
%   generate_experiment_legacy_inputs(dataset_id, true) 覆盖已有兼容文件。
%   generate_experiment_legacy_inputs(dataset_id, false, true) 仅检查。
%
% 原始输入约定：
%   imu_120.txt、truth.nav、range.txt、depth_raw.txt
% 生成的兼容输入：
%   height.txt、height_noised.txt、rangedata_noised.txt、range1.txt 至
%   range3.txt，以及 legacy-input-generation-info.mat。

if nargin < 1 || isempty(dataset_id), dataset_id = 'case-07'; end
if nargin < 2 || isempty(overwrite_existing), overwrite_existing = false; end
if nargin < 3 || isempty(dry_run), dry_run = false; end

dataset_id = char(string(dataset_id));
token = regexp(dataset_id, '^case-(\d+)$', 'tokens', 'once');
if isempty(token)
    error('dataset_id 必须采用 case-0x 格式：%s', dataset_id);
end
case_number = str2double(token{1});

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
paths = setup_inertial_experiment();
input_dir = paths.experiment_input(case_number);
if ~isfolder(input_dir)
    error('实测数据集输入目录不存在：%s', input_dir);
end

source = struct( ...
    'imu', first_existing(input_dir, {'imu_120.txt', 'IMU_120.txt'}), ...
    'truth', fullfile(input_dir, 'truth.nav'), ...
    'range', fullfile(input_dir, 'range.txt'), ...
    'depth', first_existing(input_dir, {'depth_raw.txt', 'height.txt'}), ...
    'trajectory', first_existing(input_dir, ...
        {'GNSS_1s.txt', 'pva_120.txt', 'truth.nav'}));
required_names = fieldnames(source);
for index = 1:numel(required_names)
    path = source.(required_names{index});
    if ~isfile(path)
        error('缺少生成兼容数据所需文件：%s', path);
    end
end

target = struct( ...
    'height', fullfile(input_dir, 'height.txt'), ...
    'height_noised', fullfile(input_dir, 'height_noised.txt'), ...
    'rangedata_noised', fullfile(input_dir, 'rangedata_noised.txt'), ...
    'range1', fullfile(input_dir, 'range1.txt'), ...
    'range2', fullfile(input_dir, 'range2.txt'), ...
    'range3', fullfile(input_dir, 'range3.txt'), ...
    'info', fullfile(input_dir, 'legacy-input-generation-info.mat'));

fprintf('实测兼容数据集：%s\n', dataset_id);
fprintf('输入目录：%s\n', input_dir);
if dry_run
    print_plan(target, overwrite_existing);
    generated = target;
    return;
end

depth = readmatrix(source.depth, 'FileType', 'text');
validate_matrix(depth, 2, '深度/高度');
range_data = readmatrix(source.range, 'FileType', 'text');
validate_matrix(range_data, 6, '测距');
if any(diff(range_data(:, 1)) <= 0)
    error('range.txt 的时间必须严格递增：%s', source.range);
end

write_if_needed(depth(:, 1:2), target.height, overwrite_existing);
write_if_needed(depth(:, 1:2), target.height_noised, overwrite_existing);
write_if_needed(range_data(:, 1:6), target.rangedata_noised, ...
    overwrite_existing);

range_targets = {target.range1, target.range2, target.range3};
need_ranges = overwrite_existing || any(~cellfun(@isfile, range_targets));
if need_ranges
    trajectory = readmatrix(source.trajectory, 'FileType', 'text');
    [trajectory_time, trajectory_position] = trajectory_columns( ...
        trajectory, source.trajectory);
    legacy_ranges = build_legacy_ranges( ...
        range_data(:, 1:6), trajectory_time, trajectory_position);
    for beacon_index = 1:3
        write_if_needed(legacy_ranges{beacon_index}, ...
            range_targets{beacon_index}, overwrite_existing);
    end
end

generation_info = struct();
generation_info.version = 1;
generation_info.dataset_id = dataset_id;
generation_info.generated_at = char(datetime('now', ...
    'Format', 'yyyy-MM-dd HH:mm:ss'));
generation_info.input_dir = input_dir;
generation_info.source = source;
generation_info.target = target;
generation_info.range_contract = [ ...
    '旧脚本依次从 range1/range2/range3 的同一行取观测；', ...
    '兼容文件保证该轮换结果与 range.txt 完全一致。'];
if overwrite_existing || ~isfile(target.info)
    save(target.info, 'generation_info');
end
fprintf('兼容输入生成完成：%s\n', input_dir);
generated = target;
end

function path = first_existing(folder, candidates)
path = fullfile(folder, candidates{1});
for index = 1:numel(candidates)
    candidate = fullfile(folder, candidates{index});
    if isfile(candidate)
        path = candidate;
        return;
    end
end
end

function validate_matrix(data, minimum_columns, label)
if isempty(data) || size(data, 2) < minimum_columns || ...
        any(~isfinite(data(:, 1:minimum_columns)), 'all')
    error('%s数据必须是至少 %d 列的有限数值矩阵。', ...
        label, minimum_columns);
end
end

function [time, position] = trajectory_columns(data, path)
validate_matrix(data, 4, '轨迹');
if endsWith(lower(path), 'truth.nav') || size(data, 2) >= 11
    time = data(:, 2);
    position = data(:, 3:5);
else
    time = data(:, 1);
    position = data(:, 2:4);
end
if any(diff(time) <= 0)
    [time, unique_index] = unique(time, 'stable');
    position = position(unique_index, :);
end
end

function legacy = build_legacy_ranges(range_data, trajectory_time, ...
        trajectory_position)
beacons = stable_unique_rows(range_data(:, 4:6), 1e-10);
if size(beacons, 1) ~= 3
    error('range.txt 必须恰好包含 3 个固定信标，当前识别到 %d 个。', ...
        size(beacons, 1));
end

event_position = interp1(trajectory_time, trajectory_position, ...
    range_data(:, 1), 'linear', 'extrap');
event_position(:, 1:2) = deg2rad(event_position(:, 1:2));
event_ecef = geodetic_to_ecef(event_position);
legacy = cell(3, 1);
for beacon_index = 1:3
    beacon_ecef = geodetic_to_ecef(beacons(beacon_index, :));
    difference = event_ecef - beacon_ecef;
    geometric_range = sqrt(sum(difference.^2, 2));
    legacy{beacon_index} = [range_data(:, 1), geometric_range, ...
        geometric_range, repmat(beacons(beacon_index, :), ...
        size(range_data, 1), 1)];
end

% 旧入口按 1→2→3 轮换读取同一行。把实测行放回实际会被选择的文件，
% 从而保证旧入口组合后的数据逐行等于新的 range.txt。
for event_index = 1:size(range_data, 1)
    selected = mod(event_index-1, 3)+1;
    legacy{selected}(event_index, :) = range_data(event_index, 1:6);
end
end

function rows = stable_unique_rows(data, tolerance)
rows = zeros(0, size(data, 2));
for index = 1:size(data, 1)
    if isempty(rows) || all(vecnorm(rows-data(index, :), 2, 2) > tolerance)
        rows(end+1, :) = data(index, :); %#ok<AGROW>
    end
end
end

function xyz = geodetic_to_ecef(position)
% position = [lat(rad), lon(rad), height(m)]
a = 6378137.0;
eccentricity_squared = 6.69437999014e-3;
latitude = position(:, 1);
longitude = position(:, 2);
height = position(:, 3);
prime_vertical = a ./ sqrt(1-eccentricity_squared*sin(latitude).^2);
xyz = [(prime_vertical+height).*cos(latitude).*cos(longitude), ...
    (prime_vertical+height).*cos(latitude).*sin(longitude), ...
    (prime_vertical*(1-eccentricity_squared)+height).*sin(latitude)];
end

function write_if_needed(data, path, overwrite_existing)
if isfile(path) && ~overwrite_existing
    fprintf('保留已有文件：%s\n', path);
    return;
end
temporary_path = [path, '.generating.txt'];
cleanup = onCleanup(@() delete_if_exists(temporary_path));
writematrix(data, temporary_path, 'Delimiter', ' ');
[success, message] = movefile(temporary_path, path, 'f');
if ~success
    error('无法写入兼容文件 %s：%s', path, message);
end
clear cleanup;
fprintf('已生成：%s（%d 行）\n', path, size(data, 1));
end

function delete_if_exists(path)
if isfile(path), delete(path); end
end

function print_plan(target, overwrite_existing)
names = fieldnames(target);
for index = 1:numel(names)
    path = target.(names{index});
    action = '生成';
    if isfile(path) && ~overwrite_existing, action = '保留'; end
    fprintf('%s：%s\n', action, path);
end
end
