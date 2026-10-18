function result = parse_phins_log2(inputFile, outputDir)
%PARSE_PHINS_LOG2 Parse the plain-text PHINS log2 stream.
%   RESULT = PARSE_PHINS_LOG2(INPUTFILE, OUTPUTDIR) reads NMEA-compatible
%   sentences, checks their XOR checksums, combines matching HEHDT,
%   PIXSE/POSITI, PIXSE/TIME__ and PIXSE/STDPOS records by sequence, and
%   parses embedded RAMSES PTSAG records.

arguments
    inputFile (1,1) string
    outputDir (1,1) string = fullfile(fileparts(inputFile), 'parsed')
end

mustBeFile(inputFile);
if ~isfolder(outputDir)
    mkdir(outputDir);
end

fprintf('Reading PHINS text log: %s\n', inputFile);
lines = readlines(inputFile, 'WhitespaceRule', 'preserve', ...
    'EmptyLineRule', 'skip');

headingLines = lines(startsWith(lines, "$HEHDT,"));
positionLines = lines(startsWith(lines, "$PIXSE,POSITI,"));
timeLines = lines(startsWith(lines, "$PIXSE,TIME__,"));
stdLines = lines(startsWith(lines, "$PIXSE,STDPOS,"));
ptsagLines = lines(startsWith(lines, "$PTSAG,"));

nHeading = numel(headingLines);
nPosition = numel(positionLines);
nTime = numel(timeLines);
nStd = numel(stdLines);
n = min([nHeading, nPosition, nTime, nStd]);
if numel(unique([nHeading, nPosition, nTime, nStd])) ~= 1
    warning('PHINS:UnequalNavigationCounts', ...
        ['Navigation sentence counts differ (HEHDT=%d, POSITI=%d, ' ...
         'TIME__=%d, STDPOS=%d). The first %d complete epochs are used.'], ...
        nHeading, nPosition, nTime, nStd, n);
end

headingDeg = nan(n, 1);
latitudeDeg = nan(n, 1);
longitudeDeg = nan(n, 1);
altitudeM = nan(n, 1);
utcSecondsOfDay = nan(n, 1);
latitudeStdM = nan(n, 1);
longitudeStdM = nan(n, 1);
altitudeStdM = nan(n, 1);
checksumOk = false(n, 4);

for k = 1:n
    h = split_sentence(headingLines(k));
    p = split_sentence(positionLines(k));
    t = split_sentence(timeLines(k));
    s = split_sentence(stdLines(k));

    headingDeg(k) = number_or_nan(h.fields, 2);
    latitudeDeg(k) = number_or_nan(p.fields, 3);
    longitudeDeg(k) = number_or_nan(p.fields, 4);
    altitudeM(k) = number_or_nan(p.fields, 5);
    utcSecondsOfDay(k) = hhmmss_to_seconds(field_or_empty(t.fields, 3));
    latitudeStdM(k) = number_or_nan(s.fields, 3);
    longitudeStdM(k) = number_or_nan(s.fields, 4);
    altitudeStdM(k) = number_or_nan(s.fields, 5);
    checksumOk(k, :) = [h.checksumOk, p.checksumOk, t.checksumOk, s.checksumOk];
end

utcTime = infer_file_date(inputFile, utcSecondsOfDay);
navigation = table((1:n)', utcTime, utcSecondsOfDay, headingDeg, ...
    latitudeDeg, longitudeDeg, altitudeM, latitudeStdM, longitudeStdM, ...
    altitudeStdM, all(checksumOk, 2), ...
    'VariableNames', {'sequence', 'utc_time', 'utc_seconds_of_day', ...
    'heading_deg', 'latitude_deg', 'longitude_deg', 'altitude_m', ...
    'latitude_std_m', 'longitude_std_m', 'altitude_std_m', 'checksum_ok'});

ptsag = parse_ptsag_lines(ptsagLines, repmat(string(inputFile), numel(ptsagLines), 1));
messageCounts = count_message_types(lines);

[~, stem] = fileparts(inputFile);
writetable(navigation, fullfile(outputDir, stem + "_navigation.csv"));
writetable(ptsag, fullfile(outputDir, stem + "_ptsag.csv"));
writetable(messageCounts, fullfile(outputDir, stem + "_message_counts.csv"));

result = struct('source_file', inputFile, 'navigation', navigation, ...
    'ptsag', ptsag, 'message_counts', messageCounts);
fprintf('  Navigation epochs: %d; embedded PTSAG: %d\n', ...
    height(navigation), height(ptsag));
end

function utcTime = infer_file_date(inputFile, secondsOfDay)
[~, baseName] = fileparts(inputFile);
token = regexp(char(baseName), '(?<!\d)(20\d{6})(?!\d)', 'tokens', 'once');
if isempty(token)
    utcTime = NaT(size(secondsOfDay), 'TimeZone', 'UTC');
    return;
end
baseDate = datetime(token{1}, 'InputFormat', 'yyyyMMdd', 'TimeZone', 'UTC');
utcTime = baseDate + seconds(secondsOfDay);
utcTime.Format = 'yyyy-MM-dd HH:mm:ss.SSSSSS';
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
tableOut = table(sourceFiles, (1:n)', frameNumber, utcTime, ...
    transponderId, latitudeDeg, longitudeDeg, positionFlag, computedDepthM, ...
    depthValidity, deviceDepthM, checksumOk, ...
    'VariableNames', {'source_file', 'sequence', 'frame_number', 'utc_time', ...
    'transponder_id', 'latitude_deg', 'longitude_deg', 'position_flag', ...
    'computed_depth_m', 'depth_validity', 'device_depth_m', 'checksum_ok'});
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
    elseif any(f(1) == ["$PIXSE", "$PIXOR"]) && numel(f) >= 2
        keys(k) = f(1) + "," + f(2);
    else
        keys(k) = f(1);
    end
end
[message_type, ~, group] = unique(keys);
count = accumarray(group, 1);
counts = sortrows(table(message_type, count), 'count', 'descend');
end

