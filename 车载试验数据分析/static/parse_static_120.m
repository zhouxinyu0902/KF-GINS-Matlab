function Static120 = parse_static_120(dataset_ids, save_result, caseNo)
% PARSE_STATIC_120 Parse either 120-device static data batch.
% 解析120的原始数据
%   Static120 = parse_static_120()
%   Static120 = parse_static_120(dataset_ids)
%   Static120 = parse_static_120(dataset_ids, save_result)
%   Static120 = parse_static_120(dataset_ids, save_result, caseNo)
%
% The function automatically locates:
%   <repo>/data/experiment-data/<static|static-x>/Disk1_XXX.dat  (AUXA)
%   <repo>/data/experiment-data/<static|static-x>/Disk2_XXX.dat  (STDIMU)
%
% Each XXX is treated as an independent power-on/acquisition segment. The
% Values are retained as returned by read_auax_120 and read_stdimu_120 after
% selecting the longest strictly increasing time segment from each stream.
% No UTC construction, coordinate conversion, or cross-file concatenation
% is performed here.
%
% Inputs
%   dataset_ids : IDs to parse, for example 0:3. Empty/omitted selects the
%                 pair with the longest Disk2 STDIMU recording. Pass 0:3
%                 explicitly only when the discarded short runs are needed
%                 for diagnosis.
%   save_result : true (default) saves the parsed MAT file and summary CSV.
%   caseNo      : 0 (default) selects static; 1...5 select static-1...5.
%
% Outputs
%   Static120(k).auxa_raw : 34 x N raw AUXA matrix.
%   Static120(k).imu_raw  : M x 9 raw STDIMU matrix. Column 7 is the 200 Hz
%                          machine-time counter; column 9 is checksum flag.
%
% Default saved files
%   <repo>/data/experiment-data/static/processed/static_120_raw.mat
%   <repo>/data/experiment-data/static/processed/static_120_summary.csv

if nargin < 1
    dataset_ids = [];
end
if nargin < 2 || isempty(save_result)
    save_result = true;
end
if nargin < 3 || isempty(caseNo)
    caseNo = 0;
end

validateattributes(save_result, {'logical', 'numeric'}, {'scalar'}, ...
    mfilename, 'save_result', 2);
save_result = logical(save_result);
validateattributes(caseNo, {'numeric'}, ...
    {'scalar', 'integer', 'nonnegative', '<=', 5}, mfilename, 'caseNo', 3);

code_dir = fileparts(mfilename('fullpath'));
analysis_dir = fileparts(code_dir);
repo_dir = fileparts(analysis_dir);
if caseNo == 0
    batch_name = 'static';
else
    batch_name = sprintf('static-%d', caseNo);
end
data_dir = fullfile(repo_dir, 'data', 'experiment-data', ...
    batch_name);
func_dir = fullfile(analysis_dir, 'func');
output_dir = fullfile(data_dir, 'processed');

if ~isfolder(data_dir)
    error('parse_static_120:MissingDataDirectory', ...
        'Static data directory does not exist: %s', data_dir);
end
if exist('read_auax_120', 'file') ~= 2 || exist('read_stdimu_120', 'file') ~= 2
    if ~isfolder(func_dir)
        error('parse_static_120:MissingFunctionDirectory', ...
            'Parser function directory does not exist: %s', func_dir);
    end
    addpath(func_dir);
end

disk1_ids = discover_ids(data_dir, 'Disk1');
disk2_ids = discover_ids(data_dir, 'Disk2');
paired_ids = intersect(disk1_ids, disk2_ids, 'stable');

missing_disk2 = setdiff(disk1_ids, disk2_ids);
missing_disk1 = setdiff(disk2_ids, disk1_ids);
if ~isempty(missing_disk2)
    warning('parse_static_120:MissingDisk2', ...
        'Disk1 has no matching Disk2 for ID(s): %s', id_text(missing_disk2));
end
if ~isempty(missing_disk1)
    warning('parse_static_120:MissingDisk1', ...
        'Disk2 has no matching Disk1 for ID(s): %s', id_text(missing_disk1));
end

if isempty(dataset_ids)
    stdimu_bytes = zeros(size(paired_ids));
    for k = 1:numel(paired_ids)
        file_info = dir(fullfile(data_dir, ...
            sprintf('Disk2_%03d.dat', paired_ids(k))));
        stdimu_bytes(k) = file_info.bytes;
    end
    [~, longest_index] = max(stdimu_bytes);
    dataset_ids = paired_ids(longest_index);
    discarded_ids = setdiff(paired_ids, dataset_ids, 'stable');
    if ~isempty(discarded_ids)
        fprintf(['Default selection keeps the longest STDIMU dataset %03d; ' ...
            'short dataset(s) %s are ignored.\n'], ...
            dataset_ids, id_text(discarded_ids));
    end
else
    validateattributes(dataset_ids, {'numeric'}, ...
        {'vector', 'integer', 'nonnegative', 'finite'}, mfilename, 'dataset_ids', 1);
    dataset_ids = unique(dataset_ids(:).', 'stable');
    unavailable = setdiff(dataset_ids, paired_ids);
    if ~isempty(unavailable)
        error('parse_static_120:MissingPair', ...
            'No complete Disk1/Disk2 pair for ID(s): %s', id_text(unavailable));
    end
end

if isempty(dataset_ids)
    error('parse_static_120:NoDataPairs', ...
        'No matching Disk1_XXX.dat and Disk2_XXX.dat pairs were found in %s.', data_dir);
end

frame_header = uint8([235, 144, 32]);
n_dataset = numel(dataset_ids);
Static120 = repmat(empty_dataset(), 1, n_dataset);

fprintf('\n============================================================\n');
fprintf('120 static data parsing\n');
fprintf('Data directory: %s\n', data_dir);
fprintf('Dataset IDs: %s\n', id_text(dataset_ids));
fprintf('============================================================\n');

for k = 1:n_dataset
    dataset_id = dataset_ids(k);
    dataset_name = sprintf('static-%03d', dataset_id);
    auxa_name = sprintf('Disk1_%03d.dat', dataset_id);
    imu_name = sprintf('Disk2_%03d.dat', dataset_id);
    auxa_file = fullfile(data_dir, auxa_name);
    imu_file = fullfile(data_dir, imu_name);

    fprintf('\n[%d/%d] %s\n', k, n_dataset, dataset_name);
    fprintf('  AUXA   : %s\n', auxa_name);
    tic;
    auxa_raw = read_auax_120(auxa_file);
    auxa_elapsed = toc;
    if isempty(auxa_raw) || size(auxa_raw, 1) ~= 34
        error('parse_static_120:InvalidAUXA', ...
            '%s did not produce a 34-by-N AUXA matrix.', auxa_name);
    end

    fprintf('  STDIMU : %s\n', imu_name);
    tic;
    candidate_header_positions = find_stdimu_headers(imu_file, frame_header);
    if isempty(candidate_header_positions)
        error('parse_static_120:NoSTDIMUHeader', ...
            'No STDIMU frame header was found in %s.', imu_name);
    end
    header_positions = keep_dominant_frame_phase( ...
        candidate_header_positions, 36);
    rejected_header_count = numel(candidate_header_positions) - ...
        numel(header_positions);
    if rejected_header_count > 0
        fprintf('  Ignored %d embedded false header candidate(s).\n', ...
            rejected_header_count);
    end
    imu_raw = read_stdimu_120(imu_file, header_positions);
    imu_elapsed = toc;
    if isempty(imu_raw) || size(imu_raw, 2) ~= 9
        error('parse_static_120:InvalidSTDIMU', ...
            '%s did not produce an M-by-9 STDIMU matrix.', imu_name);
    end

    auxa_decoded_records = size(auxa_raw, 2);
    imu_decoded_records = size(imu_raw, 1);
    [auxa_raw, auxa_discarded_records] = ...
        keep_longest_increasing_segment(auxa_raw, auxa_raw(1, :), 2);
    [imu_raw, imu_discarded_records] = ...
        keep_longest_increasing_segment( ...
        imu_raw, double(imu_raw(:, 7)) / 200, 1);
    if auxa_discarded_records > 0 || imu_discarded_records > 0
        fprintf(['  Kept longest increasing segment; discarded ' ...
            '%d AUXA and %d STDIMU record(s).\n'], ...
            auxa_discarded_records, imu_discarded_records);
    end

    auxa_time = auxa_raw(1, :);
    imu_time = imu_raw(:, 7).' / 200;
    auxa_time_stats = time_stats(auxa_time);
    imu_time_stats = time_stats(imu_time);
    duration_difference = imu_time_stats.duration - auxa_time_stats.duration;
    checksum_ok = sum(imu_raw(:, 9) == 1);

    info = struct();
    info.auxa_decoded_records = auxa_decoded_records;
    info.auxa_records = size(auxa_raw, 2);
    info.auxa_discarded_records = auxa_discarded_records;
    info.stdimu_candidate_headers = numel(candidate_header_positions);
    info.stdimu_headers = numel(header_positions);
    info.stdimu_rejected_headers = rejected_header_count;
    info.stdimu_decoded_records = imu_decoded_records;
    info.stdimu_records = size(imu_raw, 1);
    info.stdimu_discarded_records = imu_discarded_records;
    info.stdimu_skipped_frames = numel(header_positions) - imu_decoded_records;
    info.stdimu_checksum_ok = checksum_ok;
    info.stdimu_checksum_failed = size(imu_raw, 1) - checksum_ok;
    info.auxa_time = auxa_time_stats;
    info.stdimu_time = imu_time_stats;
    info.duration_difference_s = duration_difference;
    info.auxa_parse_seconds = auxa_elapsed;
    info.stdimu_parse_seconds = imu_elapsed;

    Static120(k).id = dataset_id;
    Static120(k).name = dataset_name;
    Static120(k).files = struct('auxa', auxa_file, 'stdimu', imu_file);
    Static120(k).auxa_raw = auxa_raw;
    Static120(k).imu_raw = imu_raw;
    Static120(k).info = info;

    fprintf('  AUXA   : %d records, %.3f ~ %.3f s, median dt %.6f s\n', ...
        info.auxa_records, auxa_time_stats.start, auxa_time_stats.finish, ...
        auxa_time_stats.median_dt);
    fprintf('  STDIMU : %d/%d records, %.3f ~ %.3f s, median dt %.6f s\n', ...
        info.stdimu_records, info.stdimu_headers, imu_time_stats.start, ...
        imu_time_stats.finish, imu_time_stats.median_dt);
    fprintf('  Checksum: %d OK, %d failed; duration difference %+0.3f s\n', ...
        info.stdimu_checksum_ok, info.stdimu_checksum_failed, duration_difference);

    if auxa_time_stats.non_increasing_count > 0
        warning('parse_static_120:AUXATimeNotIncreasing', ...
            '%s contains %d non-increasing AUXA machine-time step(s).', ...
            dataset_name, auxa_time_stats.non_increasing_count);
    end
    if imu_time_stats.non_increasing_count > 0
        warning('parse_static_120:STDIMUTimeNotIncreasing', ...
            '%s contains %d non-increasing STDIMU machine-time step(s).', ...
            dataset_name, imu_time_stats.non_increasing_count);
    end
    coverage_tolerance = max(2, 0.02 * max(auxa_time_stats.duration, imu_time_stats.duration));
    if abs(duration_difference) > coverage_tolerance
        warning('parse_static_120:CoverageMismatch', ...
            ['%s AUXA/STDIMU coverage differs by %.3f s. Selected segments ' ...
             'are kept without further cropping.'], ...
            dataset_name, duration_difference);
    end
end

summary_table = build_summary(Static120);
parse_config = struct(...
    'case_no', caseNo, ...
    'batch_name', batch_name, ...
    'data_dir', data_dir, ...
    'output_dir', output_dir, ...
    'dataset_ids', dataset_ids, ...
    'frame_header', double(frame_header), ...
    'auxa_parser', 'read_auax_120', ...
    'stdimu_parser', 'read_stdimu_120', ...
    'created_at', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')));

if save_result
    if ~isfolder(output_dir)
        mkdir(output_dir);
    end
    mat_file = fullfile(output_dir, 'static_120_raw.mat');
    csv_file = fullfile(output_dir, 'static_120_summary.csv');
    save(mat_file, 'Static120', 'summary_table', 'parse_config', '-v7.3');
    writetable(summary_table, csv_file);
    fprintf('\nSaved parsed data : %s\n', mat_file);
    fprintf('Saved summary     : %s\n', csv_file);
end

fprintf('Parsing complete: %d independent acquisition segment(s).\n', n_dataset);



end

%%
function dataset = empty_dataset()
dataset = struct(...
    'id', [], ...
    'name', '', ...
    'files', struct('auxa', '', 'stdimu', ''), ...
    'auxa_raw', [], ...
    'imu_raw', [], ...
    'info', struct());
end

function ids = discover_ids(data_dir, disk_name)
listing = dir(fullfile(data_dir, sprintf('%s_*.dat', disk_name)));
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

function positions = find_stdimu_headers(filename, header)
fid = fopen(filename, 'rb');
if fid == -1
    error('parse_static_120:OpenFailed', 'Cannot open file: %s', filename);
end
cleanup = onCleanup(@() fclose(fid));
bytes = fread(fid, inf, '*uint8');
if numel(bytes) < numel(header)
    positions = [];
    return;
end
mask = bytes(1:end-2) == header(1) & ...
       bytes(2:end-1) == header(2) & ...
       bytes(3:end) == header(3);
positions = find(mask);
end

function positions = keep_dominant_frame_phase(candidate_positions, frame_length)
% Keep the fixed-length frame phase with the most header candidates. This
% rejects header-like byte patterns embedded inside the binary payload.

phase = mod(candidate_positions - 1, frame_length);
phase_count = accumarray(phase(:) + 1, 1, [frame_length, 1]);
[~, dominant_phase_index] = max(phase_count);
positions = candidate_positions(phase == dominant_phase_index - 1);
end

function [data, discarded_records] = keep_longest_increasing_segment( ...
        data, time, sample_dimension)
% Keep the longest continuous segment with a strictly increasing time axis.
% Static recordings can contain a short pre-reset fragment followed by the
% single effective acquisition beginning near zero.

time = double(time(:));
break_after = find(~isfinite(time(1:end - 1)) | ...
    ~isfinite(time(2:end)) | diff(time) <= 0);
segment_start = [1; break_after + 1];
segment_end = [break_after; numel(time)];
segment_length = segment_end - segment_start + 1;
[~, longest_index] = max(segment_length);
keep = segment_start(longest_index):segment_end(longest_index);
discarded_records = numel(time) - numel(keep);

if sample_dimension == 1
    data = data(keep, :);
else
    data = data(:, keep);
end
end

function stats = time_stats(time)
time = double(time(:));
valid = isfinite(time);
time = time(valid);
if isempty(time)
    stats = struct('start', NaN, 'finish', NaN, 'duration', NaN, ...
        'median_dt', NaN, 'non_increasing_count', 0);
    return;
end
delta = diff(time);
positive_delta = delta(delta > 0 & isfinite(delta));
if isempty(positive_delta)
    median_dt = NaN;
else
    median_dt = median(positive_delta);
end
stats = struct(...
    'start', time(1), ...
    'finish', time(end), ...
    'duration', time(end) - time(1), ...
    'median_dt', median_dt, ...
    'non_increasing_count', sum(delta <= 0));
end

function summary = build_summary(data)
n = numel(data);
id = zeros(n, 1);
name = strings(n, 1);
auxa_records = zeros(n, 1);
stdimu_headers = zeros(n, 1);
stdimu_records = zeros(n, 1);
stdimu_skipped_frames = zeros(n, 1);
checksum_ok = zeros(n, 1);
checksum_failed = zeros(n, 1);
auxa_start_s = zeros(n, 1);
auxa_end_s = zeros(n, 1);
stdimu_start_s = zeros(n, 1);
stdimu_end_s = zeros(n, 1);
duration_difference_s = zeros(n, 1);
auxa_non_increasing = zeros(n, 1);
stdimu_non_increasing = zeros(n, 1);

for k = 1:n
    info = data(k).info;
    id(k) = data(k).id;
    name(k) = string(data(k).name);
    auxa_records(k) = info.auxa_records;
    stdimu_headers(k) = info.stdimu_headers;
    stdimu_records(k) = info.stdimu_records;
    stdimu_skipped_frames(k) = info.stdimu_skipped_frames;
    checksum_ok(k) = info.stdimu_checksum_ok;
    checksum_failed(k) = info.stdimu_checksum_failed;
    auxa_start_s(k) = info.auxa_time.start;
    auxa_end_s(k) = info.auxa_time.finish;
    stdimu_start_s(k) = info.stdimu_time.start;
    stdimu_end_s(k) = info.stdimu_time.finish;
    duration_difference_s(k) = info.duration_difference_s;
    auxa_non_increasing(k) = info.auxa_time.non_increasing_count;
    stdimu_non_increasing(k) = info.stdimu_time.non_increasing_count;
end

summary = table(id, name, auxa_records, stdimu_headers, stdimu_records, ...
    stdimu_skipped_frames, checksum_ok, checksum_failed, auxa_start_s, ...
    auxa_end_s, stdimu_start_s, stdimu_end_s, duration_difference_s, ...
    auxa_non_increasing, stdimu_non_increasing);
end

function text = id_text(ids)
if isempty(ids)
    text = '(none)';
else
    text = strjoin(compose('%03d', ids), ', ');
end
end
