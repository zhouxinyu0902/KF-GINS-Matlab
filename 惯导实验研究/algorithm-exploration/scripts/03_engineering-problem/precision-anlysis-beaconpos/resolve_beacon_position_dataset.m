function dataset = resolve_beacon_position_dataset( ...
        data_source, dataset_id, position_error_unit)
%RESOLVE_BEACON_POSITION_DATASET 统一解析潜标位置专题的仿真/实测数据集。
%   数据目录约定为：
%   data/inertial-experiment/algorithm-exploration/
%       {simulation|experiment}/{dataset_id}/{input|output}
%
%   实测数据集默认继承 ProcessConfig_exper/ProcessConfig_exper_m 的算法
%   参数，但输入和输出路径会被重定向到 dataset_id。若不同实测数据的
%   初始状态不同，可在 input/navigation-config.mat 中保存结构体
%   navigation_config（或 cfg_overrides）覆盖配置字段。

if nargin < 1 || isempty(data_source), data_source = "simulation"; end
if nargin < 2 || isempty(dataset_id)
    if lower(string(data_source)) == "experiment"
        dataset_id = 'case-07';
    else
        dataset_id = 'case-06';
    end
end
if nargin < 3 || isempty(position_error_unit), position_error_unit = "rad"; end

data_source = lower(string(data_source));
position_error_unit = lower(string(position_error_unit));
dataset_id = char(string(dataset_id));
if ~ismember(data_source, ["simulation", "experiment"])
    error('data_source只能设置为"simulation"或"experiment"。');
end
if ~ismember(position_error_unit, ["rad", "m"])
    error('position_error_unit只能设置为"rad"或"m"。');
end
if isempty(dataset_id) || any(contains(dataset_id, {'/', '\\'})) || ...
        any(strcmp(dataset_id, {'.', '..'}))
    error('dataset_id必须是单级目录名。');
end

paths = setup_inertial_experiment();
dataset_root = fullfile(paths.data, char(data_source), dataset_id);
input_dir = fullfile(dataset_root, 'input');
output_dir = fullfile(dataset_root, 'output');
artifact_dir = fullfile(output_dir, 'figures-tables');
if ~isfolder(input_dir)
    error('数据集输入目录不存在：%s', input_dir);
end

if data_source == "simulation"
    cfg = load_algorithm_exploration_config( ...
        "simulation", position_error_unit, input_dir);
    truth_candidates = {'truth.txt', 'truth.nav'};
else
    cfg = load_algorithm_exploration_config( ...
        "experiment", position_error_unit, input_dir);
    truth_candidates = {'truth.nav', 'truth.txt'};
end

navigation_config_path = fullfile(input_dir, 'navigation-config.mat');
if isfile(navigation_config_path)
    loaded = load(navigation_config_path);
    if isfield(loaded, 'navigation_config') && isstruct(loaded.navigation_config)
        cfg = apply_overrides(cfg, loaded.navigation_config);
    elseif isfield(loaded, 'cfg_overrides') && isstruct(loaded.cfg_overrides)
        cfg = apply_overrides(cfg, loaded.cfg_overrides);
    else
        error('%s必须包含结构体navigation_config或cfg_overrides。', ...
            navigation_config_path);
    end
end

truth_path = '';
for index = 1:numel(truth_candidates)
    candidate = fullfile(input_dir, truth_candidates{index});
    if isfile(candidate)
        truth_path = candidate;
        break;
    end
end
if isempty(truth_path)
    error('数据集缺少truth.txt或truth.nav：%s', input_dir);
end

% 数据集路径始终优先于继承配置中的旧路径。
cfg.case_name = dataset_id;
cfg.dataroot = paths.data;
cfg.inputfolder = input_dir;
cfg.preprocessedfolder = input_dir;
cfg.referencefolder = input_dir;
cfg.outputfolder = fullfile(output_dir, 'navigation-results');
cfg.figurefolder = artifact_dir;
cfg.imufilepath = fullfile(input_dir, 'IMU_120.txt');
cfg.gnssfilepath = fullfile(input_dir, 'pva_830.txt');
cfg.heightfilepath = first_existing_file(input_dir, ...
    {'height.txt', 'depth_raw.txt', 'height_noised.txt'});
cfg.stdfilepath = fullfile(input_dir, 'std_830.txt');
cfg.rangefilepath = first_existing_file(input_dir, ...
    {'range.txt', 'rangedata_noised.txt'});
cfg.rangefile1path = fullfile(input_dir, 'range1.txt');
cfg.rangefile2path = fullfile(input_dir, 'range2.txt');
cfg.rangefile3path = fullfile(input_dir, 'range3.txt');
cfg.truthpath = truth_path;
cfg.pureinsfilepath = fullfile(cfg.outputfolder, 'PureIns.nav');
cfg.pureinsfilepath1 = cfg.pureinsfilepath;

required_files = {cfg.imufilepath, cfg.truthpath, cfg.rangefile1path, ...
    cfg.rangefile2path, cfg.rangefile3path};
for index = 1:numel(required_files)
    if ~isfile(required_files{index})
        error('数据集缺少必需文件：%s', required_files{index});
    end
end

dataset = struct();
dataset.version = 1;
dataset.data_source = char(data_source);
dataset.dataset_id = dataset_id;
dataset.root = dataset_root;
dataset.input_dir = input_dir;
dataset.output_dir = output_dir;
dataset.artifact_dir = artifact_dir;
dataset.navigation_config_path = navigation_config_path;
dataset.cfg = cfg;
end

function path = first_existing_file(folder, candidates)
path = fullfile(folder, candidates{1});
for index = 1:numel(candidates)
    candidate = fullfile(folder, candidates{index});
    if isfile(candidate)
        path = candidate;
        return;
    end
end
end

function cfg = apply_overrides(cfg, overrides)
names = fieldnames(overrides);
for index = 1:numel(names)
    cfg.(names{index}) = overrides.(names{index});
end
end
