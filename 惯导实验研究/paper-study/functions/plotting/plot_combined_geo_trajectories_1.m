function figHandle = plot_combined_geo_trajectories_1( ...
    datasetSpecs, outputDir, baseName, varargin)
%PLOT_COMBINED_GEO_TRAJECTORIES Plot two geographic datasets with one legend.
%
% Each element of datasetSpecs must contain:
%   truthPath  - reference navigation file
%   rangeFiles - range files containing beacon coordinates
%   panelTitle - panel title, for example "Dataset 1"
%
% New Parameter:
%   GeoMode: true -> geoaxes with basemap; false -> plain axes (no map, lat/lon as X/Y)

parser = inputParser;
addParameter(parser, "BeaconNames", strings(0), ...
    @(x) isstring(x) || iscellstr(x));
addParameter(parser, "Downsample", 100, ...
    @(x) isnumeric(x) && isscalar(x) && x >= 1);
addParameter(parser, "BaseMap", "streets", ...
    @(x) ischar(x) || isstring(x));
addParameter(parser, "FigureSize", [7.16, 3.35], ...
    @(x) isnumeric(x) && numel(x) == 2 && all(x > 0));
addParameter(parser, "GeoMode", true, ...
    @(x) islogical(x) && isscalar(x));   % 新增开关
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
    pos = panelPositions(datasetIndex,:);
    
    if opts.GeoMode
        ax = geoaxes(figHandle, "Position", pos);
    else
        ax = axes(figHandle, "Position", pos);
    end
    hold(ax, "on");
    ax.FontName = "Times New Roman";
    ax.FontSize = 8;
    ax.LineWidth = 0.8;

    if opts.GeoMode
        if isprop(ax, "LatitudeLabel")
            ax.LatitudeLabel.String = "";
        end
        if isprop(ax, "LongitudeLabel")
            ax.LongitudeLabel.String = "";
        end
    else
        ax.XLabel.String = "Longitude (deg)";
        ax.YLabel.String = "Latitude (deg)";
    end

    % 轨迹绘制分支
    if opts.GeoMode
        trajectoryHandle = geoplot( ...
            ax, ...
            data.trajectoryLatDeg, ...
            data.trajectoryLonDeg, ...
            "k-", ...
            "LineWidth", 1.2, ...
            "DisplayName", "Vehicle trajectory");
        startHandle = geoscatter( ...
            ax, ...
            data.trajectoryLatDeg(1), ...
            data.trajectoryLonDeg(1), ...
            40, ...
            "filled", ...
            "MarkerFaceColor", [0.85, 0.325, 0.098], ...
            "MarkerEdgeColor", "none", ...
            "DisplayName", "Start point");
    else
        trajectoryHandle = plot(ax, ...
            data.trajectoryLonDeg, data.trajectoryLatDeg, ...
            "k-", ...
            "LineWidth",1.2,...
            "DisplayName","Vehicle trajectory");
        startHandle = scatter(ax, ...
            data.trajectoryLonDeg(1), data.trajectoryLatDeg(1), ...
            40, ...
            "filled", ...
            "MarkerFaceColor", [0.85, 0.325, 0.098], ...
            "MarkerEdgeColor", "none", ...
            "DisplayName", "Start point");
    end

    beaconHandles = gobjects(numBeacons,1);
    for beaconIndex = 1:numBeacons
        latB = data.beaconLLADeg(beaconIndex,1);
        lonB = data.beaconLLADeg(beaconIndex,2);
        if opts.GeoMode
            h = geoscatter(ax, latB, lonB, 65, beaconColors(beaconIndex,:), ...
                "filled","MarkerEdgeColor","k","LineWidth",0.7,...
                "DisplayName",beaconNames(beaconIndex));
        else
            h = scatter(ax, lonB, latB, 65, beaconColors(beaconIndex,:), ...
                "filled","MarkerEdgeColor","k","LineWidth",0.7,...
                "DisplayName",beaconNames(beaconIndex));
        end
        h.Marker = markerList{mod(beaconIndex - 1, numel(markerList)) + 1};
        beaconHandles(beaconIndex) = h;
    end

    % 仅地图模式加载底图
    if opts.GeoMode
        try
            geobasemap(ax, string(opts.BaseMap));
        catch mapError
            warning(mapError.identifier, "%s", mapError.message);
        end
    end

    % 设置坐标范围，兼容两种模式
    setAxesLimits(ax, data, opts.GeoMode);

    title(ax, sprintf("(%c) %s", ...
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

%% 替换原来 setGeographicLimits，增加GeoMode分支
function setAxesLimits(ax, data, isGeoMode)
allLat = [data.trajectoryLatDeg; data.beaconLLADeg(:, 1)];
allLon = [data.trajectoryLonDeg; data.beaconLLADeg(:, 2)];
latSpan = max(allLat) - min(allLat);
lonSpan = max(allLon) - min(allLon);
latMargin = max(0.15 * latSpan, 1e-4);
lonMargin = max(0.15 * lonSpan, 1e-4);

latLim = [min(allLat)-latMargin, max(allLat)+latMargin];
lonLim = [min(allLon)-lonMargin, max(allLon)+lonMargin];

if isGeoMode
    geolimits(ax, latLim, lonLim);
else
    ax.XLim = lonLim;
    ax.YLim = latLim;
end
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
exportgraphics(     figHandle, pdfFile,    "Resolution", 600);
end
