function result = parse_phins_log1_decoded(inputFile, outputDir)
%PARSE_PHINS_LOG1_DECODED Parse PostproDecode text output into MATLAB tables.
%   The proprietary raw packet layout is decoded by PostproDecode first.
%   This function then separates the fixed-column text telegrams. UTC for
%   each table is interpolated from the decoded Utc anchor telegrams.

arguments
    inputFile (1,1) string
    outputDir (1,1) string = fullfile(fileparts(inputFile), 'parsed')
end

mustBeFile(inputFile);
if ~isfolder(outputDir)
    mkdir(outputDir);
end

fprintf('Reading decoded PHINS log1: %s\n', inputFile);
lines = readlines(inputFile, 'WhitespaceRule', 'preserve', 'EmptyLineRule', 'skip');

imu = parse_numeric(lines, "Imu2", { ...
    'time_tag_0p1ms', 'sequence', 'imu_status', ...
    'delta_angle_x_rad', 'delta_angle_y_rad', 'delta_angle_z_rad', ...
    'delta_velocity_x_mps', 'delta_velocity_y_mps', 'delta_velocity_z_mps', ...
    'validity'});

depth = parse_numeric(lines, "Depth", { ...
    'time_tag_0p1ms', 'depth_m', 'validity', 'source_code', ...
    'depth_std_m', 'sensor_value'});

temperature = parse_numeric(lines, "Temperature", { ...
    'time_tag_0p1ms', 'temperature_1', 'temperature_2', 'temperature_3', ...
    'temperature_4', 'temperature_5', 'temperature_6', ...
    'temperature_7', 'temperature_8'});

navigation = parse_numeric(lines, "PhinsNav", { ...
    'time_tag_0p1ms', 'ins_status_1', 'ins_status_2', 'algorithm_status', ...
    'heading_raw', 'roll_raw', 'pitch_raw', 'latitude_rad', 'longitude_rad', ...
    'altitude_m', 'velocity_north_mps', 'velocity_east_mps', ...
    'velocity_up_mps', 'angular_rate_x_radps', 'angular_rate_y_radps', ...
    'angular_rate_z_radps'});
if ~isempty(navigation)
    fixedScale = 180 / 2^31;
    navigation.heading_deg = mod(navigation.heading_raw * fixedScale, 360);
    navigation.roll_deg = navigation.roll_raw * fixedScale;
    navigation.pitch_deg = navigation.pitch_raw * fixedScale;
    navigation.latitude_deg = rad2deg(navigation.latitude_rad);
    navigation.longitude_deg = rad2deg(navigation.longitude_rad);
end

kalman = parse_numeric(lines, "KalmanState", { ...
    'time_tag_0p1ms', 'heading_std_rad', 'roll_std_rad', 'pitch_std_rad', ...
    'latitude_std_m', 'longitude_std_m', 'altitude_std_m', ...
    'velocity_east_std_mps', 'velocity_north_std_mps', 'velocity_up_std_mps', ...
    'field_11', 'field_12', 'field_13', 'field_14', 'field_15', ...
    'field_16', 'field_17', 'field_18', 'field_19'});

dvlBottomTrack = parse_numeric(lines, "LogBT", { ...
    'time_tag_0p1ms', 'velocity_x_mps', 'velocity_y_mps', 'velocity_z_mps', ...
    'configured_sound_speed_mps', 'measured_sound_speed_mps', 'altitude_m', ...
    'velocity_x_std_mps', 'velocity_y_std_mps', 'velocity_z_std_mps', ...
    'validity_code', 'field_12'});

utc = parse_numeric(lines, "Utc", { ...
    'time_tag_0p1ms', 'utc_tag_0p1ms', 'day', 'month', 'year'});
utc.utc_time = make_utc_anchors(utc);

imu = add_utc_time(imu, utc);
depth = add_utc_time(depth, utc);
temperature = add_utc_time(temperature, utc);
navigation = add_utc_time(navigation, utc);
kalman = add_utc_time(kalman, utc);
dvlBottomTrack = add_utc_time(dvlBottomTrack, utc);

messageCounts = count_decoded_types(lines);

[~, stem] = fileparts(inputFile);
writetable(imu, fullfile(outputDir, stem + "_imu.csv"));
writetable(depth, fullfile(outputDir, stem + "_depth.csv"));
writetable(temperature, fullfile(outputDir, stem + "_temperature.csv"));
writetable(navigation, fullfile(outputDir, stem + "_navigation.csv"));
writetable(kalman, fullfile(outputDir, stem + "_kalman.csv"));
writetable(dvlBottomTrack, fullfile(outputDir, stem + "_dvl_bottom_track.csv"));
writetable(utc, fullfile(outputDir, stem + "_utc.csv"));
writetable(messageCounts, fullfile(outputDir, stem + "_message_counts.csv"));

result = struct('source_file', inputFile, 'imu', imu, 'depth', depth, ...
    'temperature', temperature, 'navigation', navigation, 'kalman', kalman, ...
    'dvl_bottom_track', dvlBottomTrack, 'utc', utc, ...
    'message_counts', messageCounts);
fprintf(['  Imu2: %d; Depth: %d; PhinsNav: %d; KalmanState: %d; ' ...
    'LogBT: %d; Utc: %d\n'], height(imu), height(depth), height(navigation), ...
    height(kalman), height(dvlBottomTrack), height(utc));
end

function tableOut = parse_numeric(lines, type, names)
prefix = type + string(char(9));
selected = lines(startsWith(lines, prefix));
nFields = numel(names);
if isempty(selected)
    tableOut = array2table(zeros(0, nFields), 'VariableNames', names);
    return;
end

payload = extractAfter(selected, strlength(prefix));
buffer = char(join(payload, newline));
format = repmat('%f', 1, nFields);
values = textscan(buffer, format, 'Delimiter', '\t', ...
    'MultipleDelimsAsOne', false, 'CollectOutput', true);
data = values{1};
if size(data, 2) ~= nFields
    error('PHINS:DecodedColumnCount', ...
        '%s has %d numeric columns; expected %d.', type, size(data, 2), nFields);
end
tableOut = array2table(data, 'VariableNames', names);
end

function utcTime = make_utc_anchors(utcTable)
n = height(utcTable);
utcTime = NaT(n, 1, 'TimeZone', 'UTC');
for k = 1:n
    secondsOfDay = utcTable.utc_tag_0p1ms(k) * 1e-4;
    utcTime(k) = datetime(utcTable.year(k), utcTable.month(k), utcTable.day(k), ...
        'TimeZone', 'UTC') + seconds(secondsOfDay);
end
utcTime.Format = 'yyyy-MM-dd HH:mm:ss.SSSS';
end

function tableOut = add_utc_time(tableIn, utcAnchors)
tableOut = tableIn;
if isempty(tableIn) || height(utcAnchors) < 2
    tableOut.utc_time = NaT(height(tableIn), 1, 'TimeZone', 'UTC');
    tableOut = movevars(tableOut, 'utc_time', 'After', 'time_tag_0p1ms');
    return;
end
anchorX = utcAnchors.time_tag_0p1ms;
anchorY = posixtime(utcAnchors.utc_time);
[anchorX, uniqueIndex] = unique(anchorX, 'stable');
anchorY = anchorY(uniqueIndex);
unixSeconds = interp1(anchorX, anchorY, tableIn.time_tag_0p1ms, ...
    'linear', 'extrap');
utcTime = datetime(unixSeconds, 'ConvertFrom', 'posixtime', 'TimeZone', 'UTC');
utcTime.Format = 'yyyy-MM-dd HH:mm:ss.SSSS';
tableOut.utc_time = utcTime;
tableOut = movevars(tableOut, 'utc_time', 'After', 'time_tag_0p1ms');
end

function counts = count_decoded_types(lines)
message_type = extractBefore(lines + string(char(9)), string(char(9)));
[message_type, ~, group] = unique(message_type);
count = accumarray(group, 1);
counts = sortrows(table(message_type, count), 'count', 'descend');
end

