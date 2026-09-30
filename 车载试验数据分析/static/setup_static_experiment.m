function paths = setup_static_experiment(caseNo)
%SETUP_STATIC_EXPERIMENT Configure either static data batch.
%
%   paths = setup_static_experiment(0) uses data/experiment-data/static.
%   paths = setup_static_experiment(x) uses data/experiment-data/static-x,
%   where x is an integer from 1 through 5.
%
% Each batch contains only one effective acquisition, so input and output
% files live directly below the batch input/output directories. Acquisition
% IDs such as static-003 and static-000 are retained only in parsed metadata.

if nargin < 1 || isempty(caseNo)
    caseNo = 0;
end
validateattributes(caseNo, {'numeric'}, ...
    {'scalar', 'integer', 'nonnegative', '<=', 5}, mfilename, 'caseNo', 1);

topic_root = fileparts(mfilename('fullpath'));
vehicle_analysis_root = fileparts(topic_root);
project_root = fileparts(vehicle_analysis_root);

% Add dependencies explicitly so the run does not depend on a saved MATLAB path.
addpath(fullfile(project_root, 'function'));
addpath(fullfile(project_root, 'function_zxy'));
addpath(fullfile(project_root, 'GINS-KF'));
addpath(topic_root);

paths.root = topic_root;
paths.project_root = project_root;
paths.case_no = caseNo;
if caseNo == 0
    paths.dataset_name = 'static';
else
    paths.dataset_name = sprintf('static-%d', caseNo);
end
paths.data_root = fullfile(project_root, 'data', 'experiment-data', ...
    paths.dataset_name);
if ~isfolder(paths.data_root)
    error('setup_static_experiment:MissingDataDirectory', ...
        'Static data directory does not exist: %s', paths.data_root);
end
paths.effective_acquisition_id = ...
    discover_effective_acquisition_id(paths.data_root);
paths.input = fullfile(paths.data_root, 'input');
paths.output_root = fullfile(paths.data_root, 'output');
paths.output = fullfile(paths.output_root, 'navigation-results');
paths.artifacts = fullfile(paths.output_root, 'figures-tables');
paths.processed = fullfile(paths.data_root, 'processed');

paths.imu_file = fullfile(paths.input, 'IMU_120.txt');
paths.imu_alignment_file = fullfile(paths.input, 'IMU_120_static.txt');
paths.pva_initial_file = fullfile(paths.input, 'pva_initial.txt');
paths.pva_incomplete_file = fullfile(paths.input, 'pva_incomplete.txt');
paths.ref_pva_file = fullfile(paths.input, 'ref_pva.txt');
paths.device_pva_file = fullfile(paths.input, 'pva_file.txt');

paths.pureins_fixed_height_file = fullfile(paths.output, ...
    'PureIns-FixedHeight.nav');
paths.pureins_zero_vel_fixed_height_file = fullfile(paths.output, ...
    'PureIns-ZeroVelFixedHeight.nav');
% Compatibility alias for callers that previously expected pureins_file.
paths.pureins_file = paths.pureins_fixed_height_file;

% static has no independent device PVA. Any static-x batch may use its
% pva_file.txt when that file is present.
paths.compare_device_pva = caseNo > 0 && isfile(paths.device_pva_file);

required_files = {paths.imu_file, paths.pva_initial_file};
for k = 1:numel(required_files)
    if ~isfile(required_files{k})
        error('setup_static_experiment:MissingInput', ...
            'Required %s input file is missing: %s', ...
            paths.dataset_name, required_files{k});
    end
end

required_directories = {paths.output, paths.artifacts};
for k = 1:numel(required_directories)
    if ~isfolder(required_directories{k})
        mkdir(required_directories{k});
    end
end

if isfinite(paths.effective_acquisition_id)
    fprintf('Static experiment configured: %s (source acquisition %03d)\n', ...
        paths.dataset_name, paths.effective_acquisition_id);
else
    fprintf('Static experiment configured: %s\n', paths.dataset_name);
end
end

function dataset_id = discover_effective_acquisition_id(data_root)
%DISCOVER_EFFECTIVE_ACQUISITION_ID Select the longest complete raw pair.

disk1 = discover_ids(data_root, 'Disk1');
disk2 = discover_ids(data_root, 'Disk2');
paired_ids = intersect(disk1, disk2, 'stable');
if isempty(paired_ids)
    dataset_id = NaN;
    return;
end

stdimu_bytes = zeros(size(paired_ids));
for k = 1:numel(paired_ids)
    file_info = dir(fullfile(data_root, ...
        sprintf('Disk2_%03d.dat', paired_ids(k))));
    stdimu_bytes(k) = file_info.bytes;
end
[~, longest_index] = max(stdimu_bytes);
dataset_id = paired_ids(longest_index);
end

function ids = discover_ids(data_root, disk_name)
listing = dir(fullfile(data_root, sprintf('%s_*.dat', disk_name)));
ids = [];
expression = sprintf('^%s_(\\d{3})\\.dat$', disk_name);
for k = 1:numel(listing)
    token = regexp(listing(k).name, expression, 'tokens', 'once');
    if ~isempty(token)
        ids(end + 1) = str2double(token{1}); %#ok<AGROW>
    end
end
ids = unique(sort(ids));
end
