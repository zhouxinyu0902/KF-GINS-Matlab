function figHandle = plot_combined_radial_comparison( ...
    datasetSpecs, outputDir, baseName, varargin)
%PLOT_COMBINED_RADIAL_COMPARISON Plot datasets as panels with one legend.
%
% Each element of datasetSpecs must contain:
%   truthPath  - reference navigation file
%   navFiles   - navigation result files in the common method order
%   panelTitle - panel title, for example "Scenario 1"

parser = inputParser;
addParameter(parser, "Labels", strings(0), ...
    @(x) isstring(x) || iscellstr(x));
addParameter(parser, "YLimits", [], ...
    @(x) isempty(x) || isnumeric(x));
addParameter(parser, "FigureSize", [7.16, 3.25], ...
    @(x) isnumeric(x) && numel(x) == 2 && all(x > 0));
addParameter(parser, "FontSize", 10, ...
    @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(parser, "LabelFontSize", 9, ...
    @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(parser, "TitleFontSize", 9, ...
    @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(parser, "LegendFontSize", 9, ...
    @(x) isnumeric(x) && isscalar(x) && x > 0);
parse(parser, varargin{:});
opts = parser.Results;

datasetSpecs = datasetSpecs(:);
numDatasets = numel(datasetSpecs);
if numDatasets < 1
    error("datasetSpecs must contain at least one dataset.");
end

labels = string(opts.Labels(:));
seriesByDataset = cell(numDatasets, 1);

for datasetIndex = 1:numDatasets
    validateDatasetSpec(datasetSpecs(datasetIndex));
    navFiles = cellstr(string(datasetSpecs(datasetIndex).navFiles));

    [sourceFig, ~] = calc_radial_error_gjb( ...
        datasetSpecs(datasetIndex).truthPath, navFiles{:});
    cleanupSource = onCleanup(@() closeFigureIfValid(sourceFig));

    seriesByDataset{datasetIndex} = extractLineSeries(sourceFig);
    clear cleanupSource;

    if isempty(labels)
        labels = string({seriesByDataset{datasetIndex}.DisplayName});
    end

    if numel(seriesByDataset{datasetIndex}) ~= numel(labels)
        error( ...
            "Dataset %d contains %d plotted methods, but %d labels were supplied.", ...
            datasetIndex, numel(seriesByDataset{datasetIndex}), numel(labels));
    end
end

yLimits = normalizeYLimits(opts.YLimits, numDatasets);

if ~isfolder(outputDir)
    mkdir(outputDir);
end

% figHandle = figure( ...
%     "Visible", "off", ...
%     "Color", "w", ...
%     "Units", "inches", ...
%     "Position", [1, 1, opts.FigureSize]);
figHandle = myfigurestartup(7,3,'paper');

layout = tiledlayout( ...
    figHandle, 1, numDatasets, ...
    "Padding", "compact", ...
    "TileSpacing", "compact");

panelAxes = gobjects(numDatasets, 1);
panelLines = cell(numDatasets, 1);

for datasetIndex = 1:numDatasets
    panelAxes(datasetIndex) = nexttile(layout);
    ax = panelAxes(datasetIndex);
    hold(ax, "on");
    grid(ax, "on");
    box(ax, "on");

    sourceSeries = seriesByDataset{datasetIndex};
    lines = gobjects(numel(sourceSeries), 1);

    for methodIndex = 1:numel(sourceSeries)
        lines(methodIndex) = plot( ...
            ax, ...
            sourceSeries(methodIndex).XData, ...
            sourceSeries(methodIndex).YData, ...
            "Color", sourceSeries(methodIndex).Color, ...
            "LineStyle", sourceSeries(methodIndex).LineStyle, ...
            "LineWidth", max(1.0, sourceSeries(methodIndex).LineWidth), ...
            "DisplayName", labels(methodIndex));
    end

    panelLines{datasetIndex} = lines;
    xlabel(ax, "Time (s)");
    ylabel(ax, "Radial error (m)");
    title(ax, sprintf("(%c) %s", ...
        'a' + datasetIndex - 1, string(datasetSpecs(datasetIndex).panelTitle)));

    maxTime = max(arrayfun(@(item) max(item.XData), sourceSeries));
    xlim(ax, [0, maxTime]);
    if ~isempty(yLimits)
        ylim(ax, yLimits(datasetIndex, :));
    end

    set(ax, ...
        "FontName", "Times New Roman", ...
        "FontSize", opts.FontSize, ...
        "LineWidth", 0.8);
    ax.XLabel.FontSize = opts.LabelFontSize;
    ax.YLabel.FontSize = opts.LabelFontSize;
    ax.Title.FontSize = opts.TitleFontSize;
    ax.Title.FontWeight = "normal";
    ax.XAxis.Exponent = 0;
    ax.YAxis.Exponent = 0;
end

lgd = legend( ...
    panelAxes(1), ...
    panelLines{1}, ...
    cellstr(labels), ...
    "Orientation", "horizontal", ...
    "NumColumns", numel(labels), ...
    "Interpreter", "none", ...
    "Box", "off");
lgd.Layout.Tile = "north";
lgd.AutoUpdate = "off";
lgd.FontName = "Times New Roman";
lgd.FontSize = opts.LegendFontSize;

drawnow;
exportFigurePair(figHandle, outputDir, string(baseName));

fprintf("Combined radial-error figure exported: %s\n", ...
    fullfile(outputDir, string(baseName)));

end


function validateDatasetSpec(spec)

requiredFields = ["truthPath", "navFiles", "panelTitle"];
for fieldIndex = 1:numel(requiredFields)
    if ~isfield(spec, requiredFields(fieldIndex))
        error("Dataset specification is missing field: %s", ...
            requiredFields(fieldIndex));
    end
end

if ~isfile(spec.truthPath)
    error("Truth file does not exist: %s", spec.truthPath);
end

navFiles = string(spec.navFiles(:));
missingFiles = navFiles(~isfile(navFiles));
if ~isempty(missingFiles)
    error("Navigation result does not exist: %s", missingFiles(1));
end

end


function series = extractLineSeries(figHandle)

axesCandidates = findall(figHandle, "Type", "axes");
sourceAx = gobjects(0);
maxLineCount = 0;

for axesIndex = 1:numel(axesCandidates)
    lines = findall(axesCandidates(axesIndex), "Type", "line");
    if numel(lines) > maxLineCount
        sourceAx = axesCandidates(axesIndex);
        maxLineCount = numel(lines);
    end
end

if isempty(sourceAx) || maxLineCount == 0
    error("The source radial-error figure contains no line series.");
end

sourceLines = flipud(findall(sourceAx, "Type", "line"));
series = repmat(struct( ...
    "XData", [], ...
    "YData", [], ...
    "Color", [0, 0, 0], ...
    "LineStyle", "-", ...
    "LineWidth", 1, ...
    "DisplayName", ""), numel(sourceLines), 1);

for lineIndex = 1:numel(sourceLines)
    series(lineIndex).XData = sourceLines(lineIndex).XData;
    series(lineIndex).YData = sourceLines(lineIndex).YData;
    series(lineIndex).Color = sourceLines(lineIndex).Color;
    series(lineIndex).LineStyle = string(sourceLines(lineIndex).LineStyle);
    series(lineIndex).LineWidth = sourceLines(lineIndex).LineWidth;
    series(lineIndex).DisplayName = string(sourceLines(lineIndex).DisplayName);
end

end


function yLimits = normalizeYLimits(rawLimits, numDatasets)

if isempty(rawLimits)
    yLimits = [];
    return;
end

yLimits = double(rawLimits);
if isvector(yLimits) && numel(yLimits) == 2
    yLimits = repmat(reshape(yLimits, 1, 2), numDatasets, 1);
end

if ~isequal(size(yLimits), [numDatasets, 2])
    error("YLimits must be empty, a two-element vector, or an N-by-2 matrix.");
end

if any(yLimits(:, 1) >= yLimits(:, 2))
    error("Each YLimits row must be strictly increasing.");
end

end


function exportFigurePair(figHandle, outputDir, baseName)

pngFile = fullfile(outputDir, baseName + ".png");
pdfFile = fullfile(outputDir, baseName + ".pdf");

exportgraphics(figHandle, pngFile, "Resolution", 600);
try
    exportgraphics(figHandle, pdfFile, "ContentType", "vector");
catch
    exportgraphics( ...
        figHandle, pdfFile, ...
        "ContentType", "image", ...
        "Resolution", 600);
end

end


function closeFigureIfValid(figHandle)

if isgraphics(figHandle, "figure")
    close(figHandle);
end

end
