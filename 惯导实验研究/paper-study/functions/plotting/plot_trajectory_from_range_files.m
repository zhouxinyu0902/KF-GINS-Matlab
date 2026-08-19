function result = plot_trajectory_from_range_files( ...
    truthFile, rangeFiles, outputDir, varargin)
%PLOT_TRAJECTORY_FROM_RANGE_FILES
% 根据truth.nav和多个距离文件绘制轨迹及信标分布。
%
% 距离文件格式：
%   [time, range1, range2, beaconLat, beaconLon, beaconAlt]
%
% 输入：
%   truthFile  - truth.nav完整路径，或truth.nav所在文件夹
%   rangeFiles - 距离文件路径，cell或string数组
%   outputDir  - 图片输出文件夹
%
% 可选参数：
%   BeaconNames - 信标名称，例如["Beacon 1","Beacon 2","Beacon 3"]
%   FilePrefix  - 输出文件名前缀
%   Downsample  - truth.nav降采样倍数，默认100
%   PlotGeo     - 是否绘制地理底图，默认false
%   BaseMap     - 地理底图类型，默认"streets"
%   FigureTitle - 图片标题，默认无标题
%   AxisLimits  - ENU坐标范围[xmin xmax ymin ymax]，默认自动
%   SaveFig     - 是否保存MATLAB fig文件，默认false
%
% 输出：
%   result      - 坐标、图句柄和输出路径

%% 参数解析
parser = inputParser;

addParameter(parser, "BeaconNames", strings(0), ...
    @(x) isstring(x) || iscellstr(x));

addParameter(parser, "FilePrefix", "trajectory-beacon-layout", ...
    @(x) ischar(x) || isstring(x));

addParameter(parser, "Downsample", 100, ...
    @(x) isnumeric(x) && isscalar(x) && x >= 1);

addParameter(parser, "PlotGeo", false, ...
    @(x) islogical(x) || isnumeric(x));

addParameter(parser, "BaseMap", "streets", ...
    @(x) ischar(x) || isstring(x));

addParameter(parser, "FigureTitle", "", ...
    @(x) ischar(x) || isstring(x));

addParameter(parser, "AxisLimits", [], ...
    @(x) isempty(x) || (isnumeric(x) && numel(x) == 4));

addParameter(parser, "SaveFig", false, ...
    @(x) islogical(x) || isnumeric(x));

parse(parser, varargin{:});
opts = parser.Results;

rangeFiles = string(rangeFiles(:));
numBeacons = numel(rangeFiles);

if numBeacons < 1
    error("至少需要输入一个距离文件。");
end

if isempty(opts.BeaconNames)
    beaconNames = compose("Beacon %d", 1:numBeacons);
else
    beaconNames = string(opts.BeaconNames(:));

    if numel(beaconNames) ~= numBeacons
        error("BeaconNames数量必须与距离文件数量一致。");
    end
end

if isfolder(truthFile)
    truthFile = fullfile(truthFile, "truth.nav");
end

truthFile = string(truthFile);
outputDir = string(outputDir);
filePrefix = string(opts.FilePrefix);

if ~isfile(truthFile)
    error("未找到参考轨迹文件：%s", truthFile);
end

if ~isfolder(outputDir)
    mkdir(outputDir);
end

%% 读取参考轨迹
truth = readNumericFile(truthFile);

if size(truth, 2) < 5
    error("truth.nav至少应包含5列数据。");
end

downsampleStep = max(1, round(opts.Downsample));
sampleIndex = 1:downsampleStep:size(truth, 1);
truth = truth(sampleIndex, :);

% truth格式：
% 第2列：时间
% 第3列：纬度，单位deg
% 第4列：经度，单位deg
% 第5列：高程，单位m
time = truth(:, 2);
trajectoryLLADeg = truth(:, 3:5);

trajectoryPos = trajectoryLLADeg;
trajectoryPos(:, 1:2) = deg2rad(trajectoryPos(:, 1:2));

%% 从距离文件中提取信标坐标
beaconPos = zeros(numBeacons, 3);

for i = 1:numBeacons

    if ~isfile(rangeFiles(i))
        error("未找到距离文件：%s", rangeFiles(i));
    end

    rangeData = readNumericFile(rangeFiles(i));

    if size(rangeData, 2) < 6
        error("距离文件至少应包含6列：%s", rangeFiles(i));
    end

    % 第4～6列为信标纬度、经度和高程
    currentBeacon = median( ...
        rangeData(:, 4:6), 1, "omitnan");

    if any(~isfinite(currentBeacon))
        error("无法从距离文件中提取有效信标坐标：%s", ...
            rangeFiles(i));
    end

    % 自动判断经纬度是角度还是弧度
    if abs(currentBeacon(1)) > pi / 2 + 0.1 || ...
            abs(currentBeacon(2)) > pi + 0.1

        currentBeacon(1:2) = deg2rad(currentBeacon(1:2));
    end

    beaconPos(i, :) = currentBeacon;
end

%% 转换至局部ENU坐标系
glvs;

originPos = trajectoryPos(1, :)';

trajectoryENU = pos2dxyz(trajectoryPos, originPos);
beaconENU = pos2dxyz(beaconPos, originPos);

%% 绘制局部ENU轨迹
% myfigurestartup内部已经创建坐标轴并设置字体、字号等属性
figENU = myfigurestartup(4, 3, 'zxy');
figure(figENU);
% 直接获取myfigurestartup创建的坐标轴，不能重新使用axes(figENU)
ax = gca;
cla(ax);
hold(ax, "on");
grid(ax, "on");

%% 轨迹
plot(ax, ...
    trajectoryENU(:, 1), ...
    trajectoryENU(:, 2), ...
    "k-", ...
    "LineWidth", 1.3, ...
    "DisplayName", "Vehicle trajectory");

%% 起点
scatter(ax, ...
    trajectoryENU(1, 1), ...
    trajectoryENU(1, 2), ...
    45, ...
    "filled", ...
    "MarkerFaceColor", [0.8500, 0.3250, 0.0980], ...
    "MarkerEdgeColor", "none", ...
    "DisplayName", "Start point");

%% 信标
beaconColors = lines(numBeacons);
markerList = {"^", "s", "d", "v", "o", "p"};

for i = 1:numBeacons

    markerIndex = mod(i - 1, numel(markerList)) + 1;

    scatter(ax, ...
        beaconENU(i, 1), ...
        beaconENU(i, 2), ...
        75, ...
        markerList{markerIndex}, ...
        "filled", ...
        "MarkerFaceColor", beaconColors(i, :), ...
        "MarkerEdgeColor", "k", ...
        "LineWidth", 0.7, ...
        "DisplayName", beaconNames(i));

    % 标签字号与myfigurestartup设置的坐标轴字号一致
    text(ax, ...
        beaconENU(i, 1), ...
        beaconENU(i, 2), ...
        "  " + beaconNames(i), ...
        "FontName", ax.FontName, ...
        "FontSize", ax.FontSize, ...
        "VerticalAlignment", "bottom", ...
        "HorizontalAlignment", "left");
end

xlabel(ax, "East (m)");
ylabel(ax, "North (m)");

axis(ax, "equal");

if ~isempty(opts.AxisLimits)
    axis(ax, opts.AxisLimits);
end

if strlength(string(opts.FigureTitle)) > 0
    title(ax, string(opts.FigureTitle));
end

legend(ax, ...
    "Location", "best", ...
    "AutoUpdate", "off");

ax.XAxis.Exponent = 0;
ax.YAxis.Exponent = 0;
drawnow;
%% 导出ENU图片
enuFigFile = fullfile(outputDir, filePrefix + ".fig");
enuPngFile = fullfile(outputDir, filePrefix + ".png");
enuPdfFile = fullfile(outputDir, filePrefix + ".pdf");



exportgraphics( ...
    figENU, ...
    enuPngFile, ...
    "Resolution", 600);


exportgraphics( ...
    figENU, ...
    enuPdfFile, ...
    "ContentType", "vector");


fprintf("Trajectory figure exported:\n");
fprintf("  PNG: %s\n", enuPngFile);
fprintf("  PDF: %s\n", enuPdfFile);


%% 可选：绘制地理底图
figGeo = gobjects(0);
geoPngFile = "";
geoPdfFile = "";
geoFigFile = "";

if logical(opts.PlotGeo)

    trajectoryLatDeg = trajectoryLLADeg(:, 1);
    trajectoryLonDeg = trajectoryLLADeg(:, 2);

    beaconLLADeg = beaconPos;
    beaconLLADeg(:, 1:2) = rad2deg(beaconLLADeg(:, 1:2));

    % 先用myfigurestartup建立统一尺寸和样式的figure
    figGeo = myfigurestartup(4, 3, 'zxy');
    figure(figGeo);

    % 保存myfigurestartup创建的普通坐标轴字体属性
    templateAx = gca;

    fontName = templateAx.FontName;
    fontSize = templateAx.FontSize;
    axesLineWidth = templateAx.LineWidth;

    % GeographicAxes不能与普通axes重叠
    delete(templateAx);

    gx = geoaxes(figGeo);
    hold(gx, "on");

    % 将myfigurestartup中的字体属性传递给geoaxes
    gx.FontName = fontName;
    gx.FontSize = fontSize;
    gx.LineWidth = axesLineWidth;

    %% 地理轨迹
    geoplot(gx, ...
        trajectoryLatDeg, ...
        trajectoryLonDeg, ...
        "k-", ...
        "LineWidth", 1.3, ...
        "DisplayName", "Vehicle trajectory");

    %% 起点
    geoscatter(gx, ...
        trajectoryLatDeg(1), ...
        trajectoryLonDeg(1), ...
        45, ...
        "filled", ...
        "MarkerFaceColor", [0.8500, 0.3250, 0.0980], ...
        "MarkerEdgeColor", "none", ...
        "DisplayName", "Start point");

    %% 信标
    for i = 1:numBeacons

        geoscatter(gx, ...
            beaconLLADeg(i, 1), ...
            beaconLLADeg(i, 2), ...
            75, ...
            beaconColors(i, :), ...
            "filled", ...
            "MarkerEdgeColor", "k", ...
            "LineWidth", 0.7, ...
            "DisplayName", beaconNames(i));
    end

    geobasemap(gx, string(opts.BaseMap));

    % GeographicAxes不能使用xlabel和ylabel
    if isprop(gx, "LatitudeLabel")
        gx.LatitudeLabel.String = "Latitude";
    end

    if isprop(gx, "LongitudeLabel")
        gx.LongitudeLabel.String = "Longitude";
    end

    if strlength(string(opts.FigureTitle)) > 0
        title(gx, string(opts.FigureTitle));
    end

    legend(gx, ...
        "Location", "best", ...
        "AutoUpdate", "off");

    %% 自动设置地理显示范围
    allLat = [trajectoryLatDeg; beaconLLADeg(:, 1)];
    allLon = [trajectoryLonDeg; beaconLLADeg(:, 2)];

    latSpan = max(allLat) - min(allLat);
    lonSpan = max(allLon) - min(allLon);

    latMargin = max(0.15 * latSpan, 1e-4);
    lonMargin = max(0.15 * lonSpan, 1e-4);

    geolimits(gx, ...
        [min(allLat) - latMargin, ...
         max(allLat) + latMargin], ...
        [min(allLon) - lonMargin, ...
         max(allLon) + lonMargin]);

    drawnow;

    %% 导出地理底图


    geoPngFile = fullfile( ...
        outputDir, filePrefix + "-geo.png");

    geoPdfFile = fullfile( ...
        outputDir, filePrefix + "-geo.pdf");

    if logical(opts.SaveFig)
        savefig(figGeo, geoFigFile);
    else
        geoFigFile = "";
    end

    exportgraphics( ...
        figGeo, ...
        geoPngFile, ...
        "Resolution", 600);

    % 在线底图通常不能可靠地导出为纯矢量图，
    % 因此优先尝试，失败后使用600 dpi图像PDF
    try
        exportgraphics( ...
            figGeo, ...
            geoPdfFile, ...
            "ContentType", "vector");
    catch
        exportgraphics( ...
            figGeo, ...
            geoPdfFile, ...
            "ContentType", "image", ...
            "Resolution", 600);
    end

    fprintf("Geographic figure exported:\n");
    fprintf("  PNG: %s\n", geoPngFile);
    fprintf("  PDF: %s\n", geoPdfFile);

    if logical(opts.SaveFig)
        fprintf("  FIG: %s\n", geoFigFile);
    end
end

%% 返回结果
result = struct;

result.time = time;

result.trajectoryLLADeg = trajectoryLLADeg;

result.beaconLLADeg = beaconPos;
result.beaconLLADeg(:, 1:2) = ...
    rad2deg(result.beaconLLADeg(:, 1:2));

result.trajectoryENU = trajectoryENU;
result.beaconENU = beaconENU;

result.figENU = figENU;
result.figGeo = figGeo;

result.enuFigFile = enuFigFile;
result.enuPngFile = enuPngFile;
result.enuPdfFile = enuPdfFile;

result.geoFigFile = geoFigFile;
result.geoPngFile = geoPngFile;
result.geoPdfFile = geoPdfFile;

end


function data = readNumericFile(filePath)
%READNUMERICFILE 读取.nav、.txt和.csv等数值文本文件

filePath = char(filePath);

if ~isfile(filePath)
    error("文件不存在：%s", filePath);
end

try
    % 强制作为文本文件读取，兼容.nav扩展名
    data = readmatrix( ...
        filePath, ...
        "FileType", "text");

catch ME1

    try
        importedData = importdata(filePath);

        if isstruct(importedData)
            data = importedData.data;
        else
            data = importedData;
        end

    catch ME2
        error( ...
            "无法读取文件：%s\nreadmatrix错误：%s\nimportdata错误：%s", ...
            filePath, ME1.message, ME2.message);
    end
end

if isempty(data) || ~isnumeric(data)
    error("文件中未读取到有效数值：%s", filePath);
end

% 删除全为空值的行和列
data = data(~all(isnan(data), 2), :);
data = data(:, ~all(isnan(data), 1));

if isempty(data)
    error("删除空行和空列后不存在有效数值：%s", filePath);
end

end