function result = parse_ramses_usbl(inputSpec, outputDir)
%PARSE_RAMSES_USBL Parse one or more RAMSES 6000 USBL text logs.
%   INPUTSPEC may be a file, a folder, or a wildcard such as
%   "USBL-BOX-R-*.log". PTSAG positions are converted from degrees/minutes
%   to signed decimal degrees. PTSAH heading and PIXOG ECHO UTC records are
%   also retained.

arguments
    inputSpec (1,1) string
    outputDir (1,1) string = fullfile(fileparts(inputSpec), 'parsed')
end

files = resolve_files(inputSpec);
if isempty(files)
    error('RAMSES:NoInputFiles', 'No USBL log matched: %s', inputSpec);
end
if ~isfolder(outputDir)
    mkdir(outputDir);
end

allPtsag = empty_ptsag_table();
allPtsah = empty_ptsah_table();
allEchoUtc = empty_echo_table();
allCounts = table(strings(0, 1), strings(0, 1), zeros(0, 1), ...
    'VariableNames', {'source_file', 'message_type', 'count'});

for fileIndex = 1:numel(files)
    file = string(fullfile(files(fileIndex).folder, files(fileIndex).name));
    fprintf('Reading RAMSES USBL log: %s\n', file);
    lines = readlines(file, 'WhitespaceRule', 'preserve', 'EmptyLineRule', 'skip');
    source = repmat(file, numel(lines), 1);

    ptsagMask = startsWith(lines, "$PTSAG,");
    ptsag = parse_ptsag_lines(lines(ptsagMask), source(ptsagMask));
    allPtsag = [allPtsag; ptsag]; %#ok<AGROW>

    echoMask = startsWith(lines, "$PIXOG,ECHO,UTC,");
    echoUtc = parse_echo_lines(lines(echoMask), source(echoMask));
    allEchoUtc = [allEchoUtc; echoUtc]; %#ok<AGROW>

    ptsahMask = startsWith(lines, "$PTSAH,");
    ptsah = parse_ptsah_lines(lines(ptsahMask), source(ptsahMask));
    allPtsah = [allPtsah; ptsah]; %#ok<AGROW>

    counts = count_message_types(lines);
    counts.source_file = repmat(file, height(counts), 1);
    counts = movevars(counts, 'source_file', 'Before', 1);
    allCounts = [allCounts; counts]; %#ok<AGROW>
end

if ~isempty(allPtsag)
    allPtsag = sortrows(allPtsag, {'utc_time', 'transponder_id'});
end
if ~isempty(allEchoUtc)
    allEchoUtc = sortrows(allEchoUtc, 'received_utc');
end

writetable(allPtsag, fullfile(outputDir, 'ramses_usbl_ptsag.csv'));
writetable(allPtsah, fullfile(outputDir, 'ramses_usbl_ptsah.csv'));
writetable(allEchoUtc, fullfile(outputDir, 'ramses_usbl_echo_utc.csv'));
writetable(allCounts, fullfile(outputDir, 'ramses_usbl_message_counts.csv'));

result = struct('source_files', string(fullfile({files.folder}, {files.name}))', ...
    'ptsag', allPtsag, 'ptsah', allPtsah, 'echo_utc', allEchoUtc, ...
    'message_counts', allCounts);
fprintf('  PTSAG: %d; PTSAH: %d; ECHO UTC: %d\n', ...
    height(allPtsag), height(allPtsah), height(allEchoUtc));
end

function files = resolve_files(inputSpec)
if isfolder(inputSpec)
    files = dir(fullfile(inputSpec, 'USBL-BOX-R-*.log'));
else
    files = dir(inputSpec);
end
files = files(~[files.isdir]);
[~, order] = sort({files.name});
files = files(order);
end

function tableOut = parse_ptsag_lines(lines, sourceFiles)
n = numel(lines);
frameNumber = nan(n, 1);
utcTime = NaT(n, 1, 'TimeZone', 'UTC');
transponderId = nan(n, 1);
latitudeDeg = nan(n, 1);
longitudeDeg = nan(n, 1);
positionFlag = strings(n, 1);
computedDepthM = nan(n, 1);
depthValidity = nan(n, 1);
deviceDepthM = nan(n, 1);
checksumOk = false(n, 1);
for k = 1:n
    p = split_sentence(lines(k));
    f = p.fields;
    frameNumber(k) = str2double(erase(field_or_empty(f, 2), "#"));
    utcTime(k) = make_utc(field_or_empty(f, 3), field_or_empty(f, 4), ...
        field_or_empty(f, 5), field_or_empty(f, 6));
    transponderId(k) = number_or_nan(f, 7);
    latitudeDeg(k) = dm_to_degrees(number_or_nan(f, 8), field_or_empty(f, 9));
    longitudeDeg(k) = dm_to_degrees(number_or_nan(f, 10), field_or_empty(f, 11));
    positionFlag(k) = field_or_empty(f, 12);
    computedDepthM(k) = number_or_nan(f, 13);
    depthValidity(k) = number_or_nan(f, 14);
    deviceDepthM(k) = number_or_nan(f, 15);
    checksumOk(k) = p.checksumOk;
end
utcTime.Format = 'yyyy-MM-dd HH:mm:ss.SSS';
tableOut = table(sourceFiles, frameNumber, utcTime, transponderId, ...
    latitudeDeg, longitudeDeg, positionFlag, computedDepthM, depthValidity, ...
    deviceDepthM, checksumOk, ...
    'VariableNames', {'source_file', 'frame_number', 'utc_time', ...
    'transponder_id', 'latitude_deg', 'longitude_deg', 'position_flag', ...
    'computed_depth_m', 'depth_validity', 'device_depth_m', 'checksum_ok'});
end

function tableOut = parse_ptsah_lines(lines, sourceFiles)
n = numel(lines);
deviceId = nan(n, 1);
headingDeg = nan(n, 1);
auxValue = nan(n, 1);
checksumOk = false(n, 1);
for k = 1:n
    p = split_sentence(lines(k));
    deviceId(k) = number_or_nan(p.fields, 2);
    headingDeg(k) = number_or_nan(p.fields, 3);
    auxValue(k) = number_or_nan(p.fields, 4);
    checksumOk(k) = p.checksumOk;
end
tableOut = table(sourceFiles, deviceId, headingDeg, auxValue, checksumOk, ...
    'VariableNames', {'source_file', 'device_id', 'heading_deg', ...
    'aux_value', 'checksum_ok'});
end

function tableOut = parse_echo_lines(lines, sourceFiles)
n = numel(lines);
receivedUtc = NaT(n, 1, 'TimeZone', 'UTC');
echoedUtc = NaT(n, 1, 'TimeZone', 'UTC');
checksumOk = false(n, 1);
for k = 1:n
    p = split_sentence(lines(k));
    f = p.fields;
    receivedUtc(k) = make_utc(field_or_empty(f, 4), field_or_empty(f, 5), ...
        field_or_empty(f, 6), field_or_empty(f, 7));
    echoedUtc(k) = make_utc(field_or_empty(f, 8), field_or_empty(f, 9), ...
        field_or_empty(f, 10), field_or_empty(f, 11));
    checksumOk(k) = p.checksumOk;
end
receivedUtc.Format = 'yyyy-MM-dd HH:mm:ss.SSSSSS';
echoedUtc.Format = 'yyyy-MM-dd HH:mm:ss.SSSSSS';
tableOut = table(sourceFiles, receivedUtc, echoedUtc, checksumOk, ...
    'VariableNames', {'source_file', 'received_utc', 'echoed_utc', 'checksum_ok'});
end

function tableOut = empty_ptsag_table()
tableOut = table(strings(0, 1), zeros(0, 1), NaT(0, 1, 'TimeZone', 'UTC'), ...
    zeros(0, 1), zeros(0, 1), zeros(0, 1), strings(0, 1), zeros(0, 1), ...
    zeros(0, 1), zeros(0, 1), false(0, 1), ...
    'VariableNames', {'source_file', 'frame_number', 'utc_time', ...
    'transponder_id', 'latitude_deg', 'longitude_deg', 'position_flag', ...
    'computed_depth_m', 'depth_validity', 'device_depth_m', 'checksum_ok'});
end

function tableOut = empty_ptsah_table()
tableOut = table(strings(0, 1), zeros(0, 1), zeros(0, 1), zeros(0, 1), ...
    false(0, 1), 'VariableNames', {'source_file', 'device_id', ...
    'heading_deg', 'aux_value', 'checksum_ok'});
end

function tableOut = empty_echo_table()
tableOut = table(strings(0, 1), NaT(0, 1, 'TimeZone', 'UTC'), ...
    NaT(0, 1, 'TimeZone', 'UTC'), false(0, 1), ...
    'VariableNames', {'source_file', 'received_utc', 'echoed_utc', 'checksum_ok'});
end

function parsed = split_sentence(line)
line = strip(line);
star = strfind(line, "*");
if isempty(star)
    body = line;
else
    body = extractBefore(line, star(1));
end
parsed = struct('fields', split(body, ","), ...
    'checksumOk', nmea_checksum_ok(line));
end

function ok = nmea_checksum_ok(line)
line = char(strip(line));
star = find(line == '*', 1, 'last');
if isempty(star) || star + 2 > numel(line) || line(1) ~= '$'
    ok = false;
    return;
end
actual = uint8(0);
for k = 2:(star - 1)
    actual = bitxor(actual, uint8(line(k)));
end
expected = hex2dec(line(star + 1:star + 2));
ok = double(actual) == expected;
end

function value = number_or_nan(fields, index)
value = str2double(field_or_empty(fields, index));
end

function value = field_or_empty(fields, index)
if index <= numel(fields)
    value = fields(index);
else
    value = "";
end
end

function secondsOfDay = hhmmss_to_seconds(value)
v = str2double(value);
if isnan(v)
    secondsOfDay = NaN;
    return;
end
hour = floor(v / 10000);
minute = floor((v - hour * 10000) / 100);
second = v - hour * 10000 - minute * 100;
secondsOfDay = hour * 3600 + minute * 60 + second;
end

function utc = make_utc(timeText, dayText, monthText, yearText)
sec = hhmmss_to_seconds(timeText);
day = str2double(dayText);
month = str2double(monthText);
year = str2double(yearText);
if any(isnan([sec, day, month, year]))
    utc = NaT(1, 1, 'TimeZone', 'UTC');
else
    utc = datetime(year, month, day, 'TimeZone', 'UTC') + seconds(sec);
end
end

function degrees = dm_to_degrees(dm, hemisphere)
if isnan(dm)
    degrees = NaN;
    return;
end
degrees = floor(dm / 100) + mod(dm, 100) / 60;
if any(strcmpi(hemisphere, ["S", "W"]))
    degrees = -degrees;
end
end

function counts = count_message_types(lines)
keys = strings(numel(lines), 1);
for k = 1:numel(lines)
    p = split_sentence(lines(k));
    f = p.fields;
    if isempty(f)
        keys(k) = "<empty>";
    elseif f(1) == "$PIXOG" && numel(f) >= 3
        keys(k) = f(1) + "," + f(2) + "," + f(3);
    else
        keys(k) = f(1);
    end
end
[message_type, ~, group] = unique(keys);
count = accumarray(group, 1);
counts = sortrows(table(message_type, count), 'count', 'descend');
end

