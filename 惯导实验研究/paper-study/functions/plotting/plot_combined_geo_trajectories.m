function figHandle = plot_combined_geo_trajectories( ...
    datasetSpecs, outputDir, baseName, varargin)
%PLOT_COMBINED_GEO_TRAJECTORIES Plot two geographic datasets with one legend.
%
% Each element of datasetSpecs must contain:
%   truthPath  - reference navigation file
%   rangeFiles - range files containing beacon coordinates
%   panelTitle - panel title, for example "Dataset 1"

parser = inputParser;
addParameter(parser, "BeaconNames", strings(0), ...
    @(x) isstring(x) || iscellstr(x));
addParameter(parser, "Downsample", 100, ...
    @(x) isnumeric(x) && isscalar(x) && x >= 1);
addParameter(parser, "BaseMap", "streets", ...
    @(x) ischar(x) || isstring(x));
addParameter(parser, "FigureSize", [7.16, 3.35], ...
    @(x) isnumeric(x) && numel(x) == 2 && all(x > 0));
parse(parser, varargin{:});
opts = parser.Results;

datasetSpecs = datasetSpecs(:);
numDatasets = numel(datasetSpecs);
if numDatasets ~= 2
    error("The shared geographic layout requires exactly two datasets.");
end

plotData = cell(numDatasets, 1);
for datasetIndex = 1:numDatasets
    plotData{datasetIndex} = loadGeographicDataset( ...
        datasetSpecs(datasetIndex), opts.Downsample);
end

numBeacons = size(plotData{1}.beaconLLADeg, 1);
beaconNames = string(opts.BeaconNames(:));
if isempty(beaconNames)
    beaconNames = compose("Beacon %d", 1:numBeacons).';
end
if numel(beaconNames) ~= numBeacons
    error("BeaconNames must match the number of range files.");
end

if ~isfolder(outputDir)
    mkdir(outputDir);
end

figHandle = figure( ...
    "Visible", "off", ...
    "Color", "w", ...
    "Units", "inches", ...
    "Position", [1, 1, opts.FigureSize]);

panelPositions = [ ...
    0.055, 0.10, 0.415, 0.72; ...
    0.535, 0.10, 0.415, 0.72];

beaconColors = lines(numBeacons);
markerList = {"^", "s", "d", "v", "o", "p"};
sharedHandles = gobjects(numBeacons + 2, 1);

for datasetIndex = 1:numDatasets
    data = plotData{datasetIndex};
    gx = geoaxes(figHandle, "Position", panelPositions(datasetIndex, :));
    hold(gx, "on");
    gx.FontName = "Times New Roman";
    gx.FontSize = 8;
    gx.LineWidth = 0.8;
    if isprop(gx, "LatitudeLabel")
        gx.LatitudeLabel.String = "";
    end
    if isprop(gx, "LongitudeLabel")
        gx.LongitudeLabel.String = "";
    end

    trajectoryHandle = geoplot( ...
        gx, ...
        data.trajectoryLatDeg, ...
        data.trajectoryLonDeg, ...
        "k-", ...
        "LineWidth", 1.2, ...
        "DisplayName", "Vehicle trajectory");

    startHandle = geoscatter( ...
        gx, ...
        data.trajectoryLatDeg(1), ...
        data.trajectoryLonDeg(1), ...
        40, ...
        "filled", ...
        "MarkerFaceColor", [0.85, 0.325, 0.098], ...
        "MarkerEdgeColor", "none", ...
        "DisplayName", "Start point");

    beaconHandles = gobjects(numBeacons, 1);
    for beaconIndex = 1:numBeacons
        beaconHandles(beaconIndex) = geoscatter( ...
            gx, ...
            data.beaconLLADeg(beaconIndex, 1), ...
            data.beaconLLADeg(beaconIndex, 2), ...
            65, ...
            beaconColors(beaconIndex, :), ...
            "filled", ...
            "MarkerEdgeColor", "k", ...
            "LineWidth", 0.7, ...
            "DisplayName", beaconNames(beaconIndex));
        beaconHandles(beaconIndex).Marker = ...
            markerList{mod(beaconIndex - 1, numel(markerList)) + 1};
    end

    try
        geobasemap(gx, string(opts.BaseMap));
    catch mapError
        warning(mapError.identifier, "%s", mapError.message);
    end

    setGeographicLimits(gx, data);
    title(gx, sprintf("(%c) %s", ...
        'a' + datasetIndex - 1, string(datasetSpecs(datasetIndex).panelTitle)));

    if datasetIndex == 1
        sharedHandles = [trajectoryHandle; startHandle; beaconHandles];
    end
end

legendLabels = ["Vehicle trajectory"; "Start point"; beaconNames];
lgd = legend( ...
    sharedHandles, ...
    cellstr(legendLabels), ...
    "Orientation", "horizontal", ...
    "NumColumns", numel(legendLabels), ...
    "Interpreter", "none", ...
    "Box", "off");
lgd.Units = "normalized";
lgd.Position = [0.12, 0.90, 0.76, 0.06];
lgd.AutoUpdate = "off";

drawnow;
exportFigurePair(figHandle, outputDir, string(baseName));

fprintf("Combined geographic figure exported: %s\n", ...
    fullfile(outputDir, string(baseName)));

end


function data = loadGeographicDataset(spec, downsample)

requiredFields = ["truthPath", "rangeFiles", "panelTitle"];
for fieldIndex = 1:numel(requiredFields)
    if ~isfield(spec, requiredFields(fieldIndex))
        error("Dataset specification is missing field: %s", ...
            requiredFields(fieldIndex));
    end
end

truth = readNumericFile(spec.truthPath);
if size(truth, 2) < 5
    error("truth.nav must contain at least five columns: %s", spec.truthPath);
end

sampleIndex = 1:max(1, round(downsample)):size(truth, 1);
trajectoryLLADeg = truth(sampleIndex, 3:5);

rangeFiles = string(spec.rangeFiles(:));
numBeacons = numel(rangeFiles);
beaconLLADeg = zeros(numBeacons, 3);

for beaconIndex = 1:numBeacons
    rangeData = readNumericFile(rangeFiles(beaconIndex));
    if size(rangeData, 2) < 6
        error("Range file must contain at least six columns: %s", ...
            rangeFiles(beaconIndex));
    end

    beaconPosition = median(rangeData(:, 4:6), 1, "omitnan");
    if abs(beaconPosition(1)) <= pi / 2 + 0.1 && ...
            abs(beaconPosition(2)) <= pi + 0.1
        beaconPosition(1:2) = rad2deg(beaconPosition(1:2));
    end
    beaconLLADeg(beaconIndex, :) = beaconPosition;
end

data = struct;
data.trajectoryLatDeg = trajectoryLLADeg(:, 1);
data.trajectoryLonDeg = trajectoryLLADeg(:, 2);
data.beaconLLADeg = beaconLLADeg;

end


function setGeographicLimits(gx, data)

allLat = [data.trajectoryLatDeg; data.beaconLLADeg(:, 1)];
allLon = [data.trajectoryLonDeg; data.beaconLLADeg(:, 2)];

latSpan = max(allLat) - min(allLat);
lonSpan = max(allLon) - min(allLon);
latMargin = max(0.15 * latSpan, 1e-4);
lonMargin = max(0.15 * lonSpan, 1e-4);

geolimits( ...
    gx, ...
    [min(allLat) - latMargin, max(allLat) + latMargin], ...
    [min(allLon) - lonMargin, max(allLon) + lonMargin]);

end


function data = readNumericFile(filePath)

if ~isfile(filePath)
    error("File does not exist: %s", filePath);
end

try
    data = readmatrix(filePath, "FileType", "text");
catch
    importedData = importdata(filePath);
    if isstruct(importedData)
        data = importedData.data;
    else
        data = importedData;
    end
end

if isempty(data) || ~isnumeric(data)
    error("File contains no numeric data: %s", filePath);
end

data = data(~all(isnan(data), 2), :);
data = data(:, ~all(isnan(data), 1));

end


function exportFigurePair(figHandle, outputDir, baseName)

pngFile = fullfile(outputDir, baseName + ".png");
pdfFile = fullfile(outputDir, baseName + ".pdf");

exportgraphics(figHandle, pngFile, "Resolution", 600);
exportgraphics( ...
    figHandle, pdfFile, ...
    "ContentType", "image", ...
    "Resolution", 400);

end
