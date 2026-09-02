%% 仿真结果统一导出
clear;
clc;
close all;
paper_paths = setup_paper_study();

%% ========================================================================
% 输出目录
% ========================================================================
figDir = paper_paths.paper_figures;
tabDir = paper_paths.paper_tables;

ensureFolder(figDir);
ensureFolder(tabDir);

%% Fig. 1: 两组轨迹与信标布局合并并共享图例
simulationInput1 = paper_paths.simulation_dataset1;
simulationInput2 = paper_paths.simulation_dataset2;

trajectorySpecs(1) = buildExperimentTrajectorySpec( ...
    simulationInput1, "Dataset 1");
trajectorySpecs(2) = buildExperimentTrajectorySpec( ...
    simulationInput2, "Dataset 2");

plot_combined_geo_trajectories_1( ...
    trajectorySpecs, ...
    figDir, ...
    "simu-trajectory-and-beacon-layout-combined", ...
    "BeaconNames", ["Beacon 1", "Beacon 2", "Beacon 3"], ...
    "Downsample", 100, ...
    "GeoMode",false);
%% ========================================================================
% Fig. 5: 两组仿真算法径向误差合并为双面板并共享图例
% ========================================================================
simulationSpecs(1) = buildSimulationAlgorithmSpec( ...
    paper_paths.simulation_dataset1, ...
    paper_paths.output_simulation_dataset1, ...
    "Scenario 1");

simulationSpecs(2) = buildSimulationAlgorithmSpec( ...
    paper_paths.simulation_dataset2, ...
    paper_paths.output_simulation_dataset2, ...
    "Scenario 2");

plot_combined_radial_comparison( ...
    simulationSpecs, ...
    figDir, ...
    "simu-observ-algoCMP-combined", ...
    "Labels", [ ...
        "Forward ES-EKF", ...
        "Segmented local single-stage RTS", ...
        "Two-stage cross-window RTS"], ...
    "YLimits", [0, 500; 0, 500]);
%% ========================================================================
% 仿真数据组1
% ========================================================================
processSimulationDataset( ...
    paper_paths.simulation_dataset1, ...
    paper_paths.output_simulation_dataset1, ...
    "1", ...
    "Scenario 1", ...
    [0, 500], ...
    figDir, ...
    tabDir);


%% ========================================================================
% 仿真数据组2
% ========================================================================
processSimulationDataset( ...
    paper_paths.simulation_dataset2, ...
    paper_paths.output_simulation_dataset2, ...
    "2", ...
    "Scenario 2", ...
    [0, 500], ...
    figDir, ...
    tabDir);





%% ========================================================================
% 单组仿真数据处理
% ========================================================================
function processSimulationDataset( ...
    inDir, navRootDir, dataTag, datasetTitle, ...
    algoYLim, figDir, tabDir)

cfg = config_simu(inDir);

dataTag = string(dataTag);
prefix = "simu-" + dataTag + "-";

% 固定信标和轮换信标结果所在目录
altDir = fullfile(navRootDir, "alt-B1-B2-B3");

fprintf("\n");
fprintf("============================================================\n");
fprintf("Processing simulation %s: %s\n", dataTag, datasetTitle);
fprintf("============================================================\n");


%% ------------------------------------------------------------------------
% % 1. 仿真轨迹与信标布局
% % -------------------------------------------------------------------------
% rangeFiles = {
%     fullfile(inDir, "range1.txt")
%     fullfile(inDir, "range2.txt")
%     fullfile(inDir, "range3.txt")
%     };
% 
% if isfile(cfg.truthpath) && allFilesExist(rangeFiles)
% 
%     plot_trajectory_from_range_files( ...
%         cfg.truthpath, ...
%         rangeFiles, ...
%         figDir, ...
%         "BeaconNames", ["Beacon 1", "Beacon 2", "Beacon 3"], ...
%         "FilePrefix", prefix + "trajectory-beacon-layout", ...
%         "Downsample", 100, ...
%         "FigureTitle", "", ...
%         "PlotGeo", false);
% 
% else
%     warning( ...
%         "Simulation %s缺少truth.nav或range文件，跳过轨迹图。", ...
%         dataTag);
% end


%% ------------------------------------------------------------------------
% 2. 水平可观测性分析
% -------------------------------------------------------------------------
observFile = fullfile(paper_artifact_dir(altDir), "observ.mat");

if isfile(observFile)

    observData = load( ...
        observFile, ...
        "obslog", ...
        "window_measurements");

    obsResults = analyze_beacon_observability_v2( ...
        observData.obslog, ...
        observData.window_measurements);

    % 可观测性图片
    export_beacon_observability_results( ...
        obsResults, ...
        figDir, ...
        "ExportMat", false, ...
        "ExportComparisonFigure", true, ...
        "ExportBoxFigure", true, ...
        "ExportSummaryCSV", false, ...
        "ExportWindowCSV", false, ...
        "FilePrefix", prefix + "obser");

    % 可观测性统计表
    export_beacon_observability_results( ...
        obsResults, ...
        tabDir, ...
        "ExportMat", false, ...
        "ExportComparisonFigure", false, ...
        "ExportBoxFigure", false, ...
        "ExportSummaryCSV", true, ...
        "ExportWindowCSV", false, ...
        "FilePrefix", prefix + "obser");

    % % 若导出函数仅生成PNG，则补充PDF
    % convertMatchingPngsToPdf( ...
    %     figDir, ...
    %     prefix + "obser*.png");

else
    fprintf( ...
        "未找到可观测性文件，跳过该部分：\n%s\n", ...
        observFile);
end


%% ------------------------------------------------------------------------
% 3. 固定信标与轮换信标对比
% -------------------------------------------------------------------------
geometryNavFiles = {
    fullfile(altDir, "PureIns.nav")
    fullfile(altDir, "ES-EKF-Fixed-B1.nav")
    fullfile(altDir, "ES-EKF-Fixed-B2.nav")
    fullfile(altDir, "ES-EKF-Fixed-B3.nav")
    fullfile(altDir, "ES-EKF-Alternating.nav")
    };
labels = {'Pure INS', 'Fixed-B1', 'Fixed-B2', 'Fixed-B3', 'Alternating'};
if allFilesExist(geometryNavFiles)

    geometryYLim = [];
    if dataTag == "1"
        % 为Dataset 1中的纯惯导峰值保留上部留白。
        geometryYLim = [0, 3200];
    end

    runRadialComparison( ...
        cfg.truthpath, ...
        geometryNavFiles, ...
        datasetTitle, ...
        geometryYLim, ...
        figDir, ...
        tabDir, ...
        prefix + "observ-threewaysCMP", labels, true);

else
    fprintf( ...
        "Simulation %s缺少固定信标或轮换信标结果，跳过几何对比。\n", ...
        dataTag);

    printMissingFiles(geometryNavFiles);
end


%% ------------------------------------------------------------------------
% 4. ES-EKF、单阶段RTS和两阶段RTS对比
% -------------------------------------------------------------------------
algoNavFiles = {
    fullfile(navRootDir, "ES-EKF.nav")
    fullfile(navRootDir, "Single-stage RTS.nav")
    fullfile(navRootDir, "Proposed two-stage RTS.nav")
    };
labels = { ...
    'Forward ES-EKF', ...
    'Segmented local single-stage RTS', ...
    'Two-stage cross-window RTS'};
if allFilesExist(algoNavFiles)

    runRadialComparison( ...
        cfg.truthpath, ...
        algoNavFiles, ...
        datasetTitle, ...
        algoYLim, ...
        figDir, ...
        tabDir, ...
        prefix + "observ-algoCMP", labels, false);

else
    warning( ...
        "Simulation %s缺少算法对比结果，跳过该部分。", ...
        dataTag);

    printMissingFiles(algoNavFiles);
end

fprintf("Finished: Simulation %s\n", dataTag);

end


%% ========================================================================
% 径向误差计算、绘图及表格导出
% ========================================================================
function runRadialComparison( ...
    truthPath, navFiles, datasetTitle, yLimits, ...
    figDir, tabDir, baseName, labels, exportPlot)

if nargin < 9
    exportPlot = true;
end

navFiles = cellstr(string(navFiles));

[figHandle, excelData] = calc_radial_error_gjb( ...
    truthPath, navFiles{:});

scaleH = 1.3;

pos = figHandle.Position;   % [left, bottom, W, H]
pos(4) = pos(4) * scaleH;   % 仅高度放大，宽度不变
figHandle.Position = pos;

% 锁死，savefig保存住这个窗口尺寸
figHandle.Position = figHandle.Position;

figure(figHandle);

xlabel("Time (s)");
ylabel("Radial error (m)");
title(datasetTitle);
lgd = legend(labels, ...
    "Location", "northoutside", ...
    "Orientation", "horizontal", ...
    "NumColumns", min(3, numel(labels)), ...
    "Interpreter", "none");
lgd.AutoUpdate = "off";
if ~isempty(yLimits)
    ylim(yLimits);
end

grid on;
box on;

ax = gca;
% set(ax, ...
%     "FontName", "Times New Roman", ...
%     "FontSize", 10, ...
%     "LineWidth", 0.8);

ax.XAxis.Exponent = 0;
ax.YAxis.Exponent = 0;

%% 图片导出
if exportPlot
    exportFigurePair( ...
        figHandle, ...
        figDir, ...
        baseName);
end

%% 表格导出
excelData = roundNumericCells( ...
    excelData, 2);

excelFile = fullfile( ...
    tabDir, ...
    baseName + ".xlsx");

writecell( ...
    excelData, ...
    excelFile, ...
    "Sheet", "RMSE");

if exportPlot
    fprintf("Figure saved: %s\n", ...
        fullfile(figDir, baseName));
end

fprintf("Table saved:  %s\n", ...
    excelFile);

close(figHandle);

end


%% ========================================================================
% 同步导出PNG和PDF
% ========================================================================
function exportFigurePair(figHandle, figDir, baseName)

pngFile = fullfile( ...
    figDir, ...
    baseName + ".png");

pdfFile = fullfile( ...
    figDir, ...
    baseName + ".pdf");

% 600 dpi PNG
exportgraphics( ...
    figHandle, ...
    pngFile, ...
    "Resolution", 600);

% 优先导出矢量PDF
try
    exportgraphics( ...
        figHandle, ...
        pdfFile, ...
        "ContentType", "vector");
catch
    exportgraphics( ...
        figHandle, ...
        pdfFile, ...
        "ContentType", "image", ...
        "Resolution", 600);
end

end


%% ========================================================================
% 构造合并算法图所需的单组仿真配置
% ========================================================================
function spec = buildSimulationAlgorithmSpec(inDir, navRootDir, panelTitle)

cfg = config_simu(inDir);

spec = struct;
spec.truthPath = cfg.truthpath;
spec.navFiles = {
    fullfile(navRootDir, "ES-EKF.nav")
    fullfile(navRootDir, "Single-stage RTS.nav")
    fullfile(navRootDir, "Proposed two-stage RTS.nav")
    };
spec.panelTitle = panelTitle;

end


%% ========================================================================
% Cell数组中的数值保留指定小数位
% ========================================================================
function data = roundNumericCells(data, numDigits)

for i = 1:numel(data)

    if isnumeric(data{i}) && ~isempty(data{i})
        data{i} = round(data{i}, numDigits);
    end
end

end


%% ========================================================================
% 判断文件是否全部存在
% ========================================================================
function tf = allFilesExist(fileList)

fileList = string(fileList);

if isempty(fileList) || any(strlength(fileList) == 0)
    tf = false;
    return;
end

tf = all(isfile(fileList));

end


%% ========================================================================
% 从多个候选路径中选取第一个存在的文件
% ========================================================================
function selectedFile = pickExistingFile(candidateFiles)

candidateFiles = string(candidateFiles);

index = find( ...
    isfile(candidateFiles), ...
    1, ...
    "first");

if isempty(index)
    selectedFile = "";
else
    selectedFile = candidateFiles(index);
end

end


%% ========================================================================
% 打印缺失文件
% ========================================================================
function printMissingFiles(fileList)

fileList = string(fileList);

for i = 1:numel(fileList)

    if strlength(fileList(i)) == 0
        fprintf("  Missing file: empty path\n");

    elseif ~isfile(fileList(i))
        fprintf("  Missing file: %s\n", fileList(i));
    end
end

end


%% ========================================================================
% 创建文件夹
% ========================================================================
function ensureFolder(folderPath)

if ~isfolder(folderPath)
    mkdir(folderPath);
end

end


%% ========================================================================
% 将指定PNG补充转换为PDF
% ========================================================================
function convertMatchingPngsToPdf(figDir, pattern)

pngFiles = dir( ...
    fullfile(figDir, pattern));

for i = 1:numel(pngFiles)

    pngFile = fullfile( ...
        pngFiles(i).folder, ...
        pngFiles(i).name);

    [~, fileName] = fileparts(pngFile);

    pdfFile = fullfile( ...
        figDir, ...
        fileName + ".pdf");

    if ~isfile(pdfFile)
        convertPngToPdf( ...
            pngFile, ...
            pdfFile);
    end
end

end


%% ========================================================================
% 单张PNG转换为PDF
% ========================================================================
function convertPngToPdf(pngFile, pdfFile)

imageData = imread(pngFile);

figHandle = figure( ...
    "Visible", "off", ...
    "Color", "w");

ax = axes( ...
    figHandle, ...
    "Position", [0, 0, 1, 1]);

image(ax, imageData);
axis(ax, "image");
axis(ax, "off");

exportgraphics( ...
    figHandle, ...
    pdfFile, ...
    "ContentType", "image", ...
    "Resolution", 600);

close(figHandle);

end
%% ========================================================================
%  构造一组实测轨迹与信标配置
% ========================================================================
function spec = buildExperimentTrajectorySpec(inDir, panelTitle)

cfg = config_simu(inDir);

spec = struct;
spec.truthPath = cfg.truthpath;
spec.rangeFiles = {
    fullfile(inDir, "range1.txt")
    fullfile(inDir, "range2.txt")
    fullfile(inDir, "range3.txt")
    };
spec.panelTitle = panelTitle;

end