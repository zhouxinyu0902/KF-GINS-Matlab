function manifest = process_beaconpos( ...
        simulation_case, error_enu_m, scenario_ids)
%PROCESS_BEACONPOS 构造不同潜标位置误差下的三信标测距输入。
%   manifest = process_beaconpos('case-00', [0 0 0; 10 10 0.4])
%   error_enu_m 每行依次为东、北、天方向误差，单位均为 m。
%
%   本函数只修改测距文件第4至6列的潜标纬度、经度和高度，距离值、
%   时间戳及其余列保持不变。生成文件写入工程工况输入目录，不覆盖
%   原始仿真数据。

if nargin < 1 || isempty(simulation_case)
    simulation_case = 'case-00';
end
if nargin < 2 || isempty(error_enu_m)
    error_enu_m = [0, 0, 0; 10, 10, 0.4; 20, 20, 0.4; ...
        50, 50, 0.4; 100, 100, 0.4; 200, 200, 1.0];
end
if size(error_enu_m, 2) ~= 3 || any(~isfinite(error_enu_m), 'all')
    error('error_enu_m 必须是 N×3 的有限数值矩阵。');
end

simulation_case = char(string(simulation_case));
scenario_count = size(error_enu_m, 1);
if nargin < 3 || isempty(scenario_ids)
    scenario_ids = strings(scenario_count, 1);
    for index = 1:scenario_count
        scenario_ids(index) = format_scenario_id(error_enu_m(index, :));
    end
else
    scenario_ids = string(scenario_ids(:));
end
if numel(scenario_ids) ~= scenario_count || ...
        any(strlength(scenario_ids) == 0) || ...
        numel(unique(scenario_ids)) ~= scenario_count
    error('scenario_ids 必须与误差行数一致、非空且不能重复。');
end

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(script_dir));
addpath(topic_dir);
paths = setup_inertial_experiment();

source_dir = fullfile(paths.simulation_input, simulation_case);
source_paths = arrayfun(@(index) fullfile(source_dir, ...
    sprintf('range%d.txt', index)), 1:3, 'UniformOutput', false);
missing_mask = ~cellfun(@isfile, source_paths);
if any(missing_mask)
    error('缺少原始测距文件：%s', strjoin(source_paths(missing_mask), ', '));
end

generated_root = fullfile(paths.engineering_input, 'simulation', ...
    simulation_case, 'beacon-position-error');
if ~isfolder(generated_root)
    mkdir(generated_root);
end

range_path_1 = strings(scenario_count, 1);
range_path_2 = strings(scenario_count, 1);
range_path_3 = strings(scenario_count, 1);
for scenario_index = 1:scenario_count
    scenario_dir = fullfile(generated_root, scenario_ids(scenario_index));
    if ~isfolder(scenario_dir)
        mkdir(scenario_dir);
    end
    for beacon_index = 1:3
        range_data = readmatrix(source_paths{beacon_index}, ...
            'FileType', 'text');
        if size(range_data, 2) < 6
            error('测距文件至少应包含6列：%s', source_paths{beacon_index});
        end
        range_data(:, 4:6) = add_enu_offset_to_geodetic( ...
            range_data(:, 4:6), error_enu_m(scenario_index, :));
        output_path = fullfile(scenario_dir, ...
            sprintf('range%d.txt', beacon_index));
        writematrix(range_data, output_path, 'Delimiter', ' ', ...
            'FileType', 'text');
        if beacon_index == 1, range_path_1(scenario_index) = output_path; end
        if beacon_index == 2, range_path_2(scenario_index) = output_path; end
        if beacon_index == 3, range_path_3(scenario_index) = output_path; end
    end
end

manifest = table(scenario_ids, error_enu_m(:, 1), error_enu_m(:, 2), ...
    error_enu_m(:, 3), vecnorm(error_enu_m(:, 1:2), 2, 2), ...
    range_path_1, range_path_2, range_path_3, ...
    'VariableNames', {'ScenarioId', 'EastError_m', 'NorthError_m', ...
    'UpError_m', 'HorizontalError_m', 'Range1Path', 'Range2Path', ...
    'Range3Path'});
writetable(manifest, fullfile(generated_root, 'input-manifest.csv'));
fprintf('已生成 %d 组潜标位置误差输入：%s\n', ...
    scenario_count, generated_root);
end

function updated_position = add_enu_offset_to_geodetic(position, offset)
%ADD_ENU_OFFSET_TO_GEODETIC 利用WGS-84局部曲率半径转换ENU小量。
    latitude = position(:, 1);
    longitude = position(:, 2);
    height = position(:, 3);
    semi_major_axis = 6378137.0;
    eccentricity_squared = 6.69437999014e-3;
    denominator = sqrt(1-eccentricity_squared*sin(latitude).^2);
    prime_vertical_radius = semi_major_axis./denominator;
    meridian_radius = semi_major_axis*(1-eccentricity_squared)./ ...
        denominator.^3;
    longitude_scale = (prime_vertical_radius+height).*cos(latitude);
    if any(abs(longitude_scale) < 1)
        error('信标纬度过于接近极点，无法稳定换算东向位置误差。');
    end
    updated_position = [ ...
        latitude + offset(2)./(meridian_radius+height), ...
        longitude + offset(1)./longitude_scale, ...
        height + offset(3)];
end

function scenario_id = format_scenario_id(offset)
%FORMAT_SCENARIO_ID 生成不含空格和小数点的稳定工况目录名。
    if all(abs(offset) < 1e-12)
        scenario_id = "nominal";
        return;
    end
    scenario_id = "e" + encode_number(offset(1)) + ...
        "-n" + encode_number(offset(2)) + ...
        "-u" + encode_number(offset(3));
end

function text_value = encode_number(value)
%ENCODE_NUMBER 将符号和小数编码为可读目录名。
    text_value = string(sprintf('%+g', value));
    text_value = replace(text_value, "+", "p");
    text_value = replace(text_value, "-", "m");
    text_value = replace(text_value, ".", "p");
end
