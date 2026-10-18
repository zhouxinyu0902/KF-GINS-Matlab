%% PROCESS_INS_DVL_USBL Parse the sea-trial batch containing inertial data.
% Executable script (not a function).
%
% Scope:
%   - PHINS log1 decoded data (INS/IMU/DVL/depth/Kalman/UTC)
%   - PHINS log2 data
%   - the last three incomplete USBL files, concatenated in file order
%
% This script does not perform the overall five-file USBL analysis and does
% not draw USBL overview figures. Run scripts/analyze_usbl_overview.m for
% that independent task. All table exports are tab-delimited UTF-8 TXT.

clear;
clc;

%% Paths
scriptDir = string(fileparts(mfilename('fullpath')));
scriptsDir = string(fileparts(scriptDir));
projectDir = string(fileparts(scriptsDir));
dataDir = fullfile(projectDir, "信标标定试验数据");
outputDir = fullfile(projectDir, "解析后的数据", "ins_batch");

mustBeFolder(dataDir);
requiredParsers = ["parse_phins_log1_decoded.m", "parse_phins_log2.m", ...
    "parse_ramses_usbl.m"];
for k = 1:numel(requiredParsers)
    mustBeFile(fullfile(scriptsDir, requiredParsers(k)));
end
addpath(scriptsDir);
cleanupPath = onCleanup(@() rmpath(scriptsDir)); 

if ~isfolder(outputDir), mkdir(outputDir); end
parserScratch = string(tempname);
mkdir(parserScratch);
cleanupScratch = onCleanup(@() cleanup_parser_scratch(parserScratch)); 

log1File = fullfile(dataDir, "log1-20230430-0347.dat");
log2File = fullfile(dataDir, "log2-20230430-0347.dat");
decodedLog1File = fullfile(projectDir, "解析后的数据", ...
    "log1-20230430-0347_decoded.txt");
mustBeFile(log1File); %#ok<NASGU>
mustBeFile(log2File);
mustBeFile(decodedLog1File);

usblFiles = dir(fullfile(dataDir, "USBL-BOX-R-*.log"));
usblFiles = usblFiles(~[usblFiles.isdir]);
[~, order] = sort(lower(string({usblFiles.name})));
usblFiles = usblFiles(order);
assert(numel(usblFiles) >= 3, 'INS:MissingUSBL', ...
    'At least three USBL files are required.');
selectedUsbl = string(fullfile({usblFiles(end-2:end).folder}, ...
    {usblFiles(end-2:end).name}))';

fprintf('Inertial batch input : %s\n', dataDir);
fprintf('Output               : %s\n', outputDir);
fprintf('Selected USBL files  :\n');
for k = 1:numel(selectedUsbl)
    fprintf('  %s\n', selectedUsbl(k));
end

%% Call the three parser types
log1 = parse_phins_log1_decoded(decodedLog1File, parserScratch);
log2 = parse_phins_log2(log2File, parserScratch);
usblLast3 = parse_selected_usbl(selectedUsbl, parserScratch);

%% Export parsed tables only; no cross-dataset or overview analysis
write_txt(log1.imu, fullfile(outputDir, "log1_imu.txt"));
write_txt(log1.depth, fullfile(outputDir, "log1_depth.txt"));
write_txt(log1.temperature, fullfile(outputDir, "log1_temperature.txt"));
write_txt(log1.navigation, fullfile(outputDir, "log1_navigation.txt"));
write_txt(log1.kalman, fullfile(outputDir, "log1_kalman.txt"));
write_txt(log1.dvl_bottom_track, fullfile(outputDir, "log1_dvl_bottom_track.txt"));
write_txt(log1.utc, fullfile(outputDir, "log1_utc.txt"));
write_txt(log1.message_counts, fullfile(outputDir, "log1_message_counts.txt"));

write_txt(log2.navigation, fullfile(outputDir, "log2_navigation.txt"));
write_txt(log2.ptsag, fullfile(outputDir, "log2_ptsag.txt"));
write_txt(log2.message_counts, fullfile(outputDir, "log2_message_counts.txt"));

write_txt(usblLast3.ptsag, fullfile(outputDir, "usbl_last3_ptsag.txt"));
write_txt(usblLast3.ptsah, fullfile(outputDir, "usbl_last3_ptsah.txt"));
write_txt(usblLast3.echo_utc, fullfile(outputDir, "usbl_last3_echo_utc.txt"));
write_txt(usblLast3.message_counts, ...
    fullfile(outputDir, "usbl_last3_message_counts.txt"));

save(fullfile(outputDir, "ins_batch_parsed.mat"), ...
    'log1', 'log2', 'usblLast3', 'selectedUsbl', '-v7.3');

fprintf('\nCompleted inertial-batch parsing.\n');
fprintf('  log1: IMU %d, navigation %d, DVL %d\n', ...
    height(log1.imu), height(log1.navigation), height(log1.dvl_bottom_track));
fprintf('  log2: navigation %d, PTSAG %d\n', ...
    height(log2.navigation), height(log2.ptsag));
fprintf('  last-three USBL: PTSAG %d, PTSAH %d, ECHO UTC %d\n', ...
    height(usblLast3.ptsag), height(usblLast3.ptsah), ...
    height(usblLast3.echo_utc));

%% Local helpers
function result = parse_selected_usbl(files, scratch)
parts = cell(numel(files), 1);
for k = 1:numel(files)
    parts{k} = parse_ramses_usbl(files(k), scratch);
end
result = parts{1};
result.source_files = files;
for k = 2:numel(parts)
    result.ptsag = [result.ptsag; parts{k}.ptsag]; %#ok<AGROW>
    result.ptsah = [result.ptsah; parts{k}.ptsah]; %#ok<AGROW>
    result.echo_utc = [result.echo_utc; parts{k}.echo_utc]; %#ok<AGROW>
    result.message_counts = [result.message_counts; ...
        parts{k}.message_counts]; %#ok<AGROW>
end
if ~isempty(result.ptsag)
    result.ptsag = sortrows(result.ptsag, {'utc_time','transponder_id'});
end
if ~isempty(result.echo_utc)
    result.echo_utc = sortrows(result.echo_utc, 'received_utc');
end
end

function write_txt(tbl, file)
writetable(tbl, file, 'FileType', 'text', 'Delimiter', '\t', ...
    'Encoding', 'UTF-8');
end

function cleanup_parser_scratch(folder)
folder = string(folder);
tempRoot = string(tempdir);
if isfolder(folder) && startsWith(lower(folder), lower(tempRoot))
    rmdir(folder, 's');
end
end
