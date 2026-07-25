%% 试验结果统一导出
clear;
clc;

%% 输出目录
rootOutput = "D:\WPS云盘\469639050\WPS云盘\成果\2_INS_RANGE";
figDir = fullfile(rootOutput, "figs");
tabDir = fullfile(rootOutput, "tab");

ensureFolder(figDir);
ensureFolder(tabDir);

%% 根据生成的距离文件统一绘制轨迹和信标

%% 数据组5，对应论文中的Dataset 1
processDataset( ...
    'F:/2_Data/惯导试验/实验数据/All_data/input5/', ...
    "D:\Github\KF-GINS-Matlab\new_惯导试验\exper\output5", ...
    5, ...
    "Dataset 1 ", ...
    [0, 1000], ...
    figDir, ...
    tabDir);

%% 数据组6，对应论文中的Dataset 2
processDataset( ...
    'F:\2_Data\惯导试验\实验数据\All_data\input6', ...
    "D:\Github\KF-GINS-Matlab\new_惯导试验\exper\output6", ...
    6, ...
    "Dataset 2 ", ...
    [0, 500], ...
    figDir, ...
    tabDir);


%% ========================================================================
%  单组数据处理
% ========================================================================
function processDataset( ...
    inDir, navOutputDir, fileTag, datasetTitle, algoYLim, figDir, tabDir)

cfg = config_1(inDir);
cfg.outputfolder = navOutputDir;

prefix = "exper-" + fileTag + "-";
altDir = fullfile(cfg.outputfolder, "alt-B1-B2-B3");

fprintf("\n========== Processing %s ==========\n", datasetTitle);
%%
rangeFiles = {
    fullfile(inDir, "range1.txt")
    fullfile(inDir, "range2.txt")
    fullfile(inDir, "range3.txt")
    };

plot_trajectory_from_range_files( ...
    cfg.truthpath, ...
    rangeFiles, ...
    figDir, ...
    "BeaconNames", ["Beacon 1", "Beacon 2", "Beacon 3"], ...
    "FilePrefix", "exper-"+ fileTag +"trajectory-and-beacon-layout-" , ...
    "Downsample", 100, ...
    "PlotGeo", true);
%% 1. 可观测性分析
observFile = fullfile(altDir, "observ.mat");

data = load(observFile, "obslog", "window_measurements");

obsResults = analyze_beacon_observability_v2( ...
    data.obslog, data.window_measurements);

% 图像单独导出至figs
export_beacon_observability_results( ...
    obsResults, figDir, ...
    "ExportMat", false, ...
    "ExportComparisonFigure", false, ...
    "ExportBoxFigure", true, ...
    "ExportSummaryCSV", false, ...
    "ExportWindowCSV", false, ...
    "FilePrefix", prefix + "obser");

% 表格单独导出至tab
export_beacon_observability_results( ...
    obsResults, tabDir, ...
    "ExportMat", false, ...
    "ExportComparisonFigure", false, ...
    "ExportBoxFigure", false, ...
    "ExportSummaryCSV", true, ...
    "ExportWindowCSV", false, ...
    "FilePrefix", prefix + "obser");



%% 2. 固定信标与轮换信标对比
navFiles = {
    fullfile(cfg.outputfolder, "PureIns.nav")
    fullfile(altDir, "ES-EKF-Fixed-B1.nav")
    fullfile(altDir, "ES-EKF-Fixed-B2.nav")
    fullfile(altDir, "ES-EKF-Fixed-B3.nav")
    fullfile(altDir, "ES-EKF-Alternating.nav")
    };
labels={'PureIns','ES-EKF-Fixed-B1','ES-EKF-Fixed-B2','ES-EKF-Fixed-B3','ES-EKF-Alternating'};
runRadialComparison( ...
    cfg.truthpath, navFiles, datasetTitle, [], ...
    figDir, tabDir, prefix + "observ-threewaysCMP",labels);


%% 3. 高度处理方式对比
heightDir = fullfile(cfg.outputfolder, "heightwayCMP");

heightNavFiles = {
    fullfile(heightDir, "No-height update.nav")
    fullfile(heightDir, "Direct assignment.nav")
    fullfile(heightDir, "Measurement update.nav")
    };
labels={'No-height update','Direct assignment','Measurement update'};
runRadialComparison( ...
    cfg.truthpath, heightNavFiles, datasetTitle, [], ...
    figDir, tabDir, prefix + "height-method-CMP",labels);

% 垂向误差和垂向速度误差
heightCsv = fullfile(tabDir, prefix + "height-vd-rmse.csv");
heightPng = fullfile(figDir, prefix + "height-vd-error.png");
heightPdf = fullfile(figDir, prefix + "height-vd-error.pdf");

figBefore = findall(groot, "Type", "figure");

heightResults = compare_height_vertical_rmse( ...
    cfg.truthpath, heightNavFiles, ...
    "MethodNames", ...
    ["No-height update", "Direct assignment", "Measurement update"], ...
    "OutputFile", heightCsv, ...
    "FigureFile", heightPng); %#ok<NASGU>

figAfter = findall(groot, "Type", "figure");
newFigures = getNewFigures(figBefore, figAfter);

if ~isempty(newFigures)
    % 优先直接从图句柄导出矢量PDF
    exportgraphics( ...
        newFigures(1), heightPdf, ...
        "ContentType", "vector");

    close(newFigures);
elseif isfile(heightPng)
    % 若函数内部关闭了图窗，则使用PNG生成PDF
    convertPngToPdf(heightPng, heightPdf);
else
    warning("未找到高度误差图：%s", heightPng);
end


%% 4. ES-EKF、单阶段RTS与两阶段RTS对比
algoNavFiles = {
    fullfile(cfg.outputfolder, "ESKF.nav")
    fullfile(cfg.outputfolder, "Single-stage RTS.nav")
    fullfile(cfg.outputfolder, "Proposed two-stage RTS.nav")
    };
labels ={'Forward ES-EKF','Segmented local single-stage RTS','Two-stage cross-window RTS'};
runRadialComparison( ...
    cfg.truthpath, algoNavFiles, datasetTitle, algoYLim, ...
    figDir, tabDir, prefix + "observ-algoCMP",labels);


% %% 5. 长基线结果对比
% if fileTag == 5
%     % 数据组5：只比较三种LBL辅助算法
%     lblNavFiles = {
%         fullfile(cfg.outputfolder, "ESKF-LBL.nav")
%         fullfile(cfg.outputfolder, "Single-stage RTS-LBL.nav")
%         fullfile(cfg.outputfolder, "Proposed two-stage RTS-LBL.nav")
%         };
% 
% else
%     % 数据组6：同时比较单距离与LBL结果
%     lblNavFiles = {
%         fullfile(cfg.outputfolder, "ESKF.nav")
%         fullfile(cfg.outputfolder, "ESKF-LBL.nav")
%         fullfile(cfg.outputfolder, "Single-stage RTS.nav")
%         fullfile(cfg.outputfolder, "Single-stage RTS-LBL.nav")
%         fullfile(cfg.outputfolder, "Proposed two-stage RTS.nav")
%         fullfile(cfg.outputfolder, "Proposed two-stage RTS-LBL.nav")
%         };
% end
% 
% runRadialComparison( ...
%     cfg.truthpath, lblNavFiles, datasetTitle, [0, 200], ...
%     figDir, tabDir, prefix + "LBL-observ-algoCMP");

fprintf("Finished: %s\n", datasetTitle);

end


%% ========================================================================
%  径向误差计算、绘图和表格导出
% ========================================================================
function runRadialComparison( ...
    truthPath, navFiles, datasetTitle, yLimits, ...
    figDir, tabDir, baseName,labels)

[figHandle, excelData] = calc_radial_error_gjb( ...
    truthPath, navFiles{:});

figure(figHandle);

xlabel("Time (s)");
ylabel("Radial error (m)");
title(datasetTitle);
legend(labels);
if ~isempty(yLimits)
    ylim(yLimits);
end

grid on;
box on;

% PNG与PDF均保存至figs
exportFigurePair(figHandle, figDir, baseName);

% 数值统一保留两位小数
excelData = roundNumericCells(excelData, 2);

% 表格仅保存至tab
excelFile = fullfile(tabDir, baseName + ".xlsx");
writecell(excelData, excelFile, "Sheet", "RMSE");

fprintf("Figure saved: %s\n", fullfile(figDir, baseName));
fprintf("Table saved:  %s\n", excelFile);

close(figHandle);

end


%% ========================================================================
%  同步导出PNG和PDF
% ========================================================================
function exportFigurePair(figHandle, figDir, baseName)

pngFile = fullfile(figDir, baseName + ".png");
pdfFile = fullfile(figDir, baseName + ".pdf");

% 高分辨率位图
exportgraphics( ...
    figHandle, pngFile, ...
    "Resolution", 600);

% 优先输出矢量PDF
try
    exportgraphics( ...
        figHandle, pdfFile, ...
        "ContentType", "vector");
catch
    % 个别图形对象不支持矢量输出时退化为高分辨率图像
    exportgraphics( ...
        figHandle, pdfFile, ...
        "ContentType", "image", ...
        "Resolution", 600);
end

end


%% ========================================================================
%  Cell数组中的数值统一保留小数位数
% ========================================================================
function data = roundNumericCells(data, numDigits)

for i = 1:numel(data)
    if isnumeric(data{i}) && ~isempty(data{i})
        data{i} = round(data{i}, numDigits);
    end
end

end


%% ========================================================================
%  创建文件夹
% ========================================================================
function ensureFolder(folderPath)

if ~isfolder(folderPath)
    mkdir(folderPath);
end

end


%% ========================================================================
%  获取函数调用后新生成的图窗
% ========================================================================
function newFigures = getNewFigures(figBefore, figAfter)

if isempty(figAfter)
    newFigures = gobjects(0);
    return;
end

isNew = false(size(figAfter));

for i = 1:numel(figAfter)
    isNew(i) = ~any(figAfter(i) == figBefore);
end

newFigures = figAfter(isNew);

end


%% ========================================================================
%  将指定PNG补充转换为PDF
%  仅作为无法获得原始图句柄时的兜底方式
% ========================================================================
function convertMatchingPngsToPdf(figDir, pattern)

pngFiles = dir(fullfile(figDir, pattern));

for i = 1:numel(pngFiles)
    pngFile = fullfile(pngFiles(i).folder, pngFiles(i).name);

    [~, fileName] = fileparts(pngFile);
    pdfFile = fullfile(figDir, fileName + ".pdf");

    if ~isfile(pdfFile)
        convertPngToPdf(pngFile, pdfFile);
    end
end

end


%% ========================================================================
%  单张PNG转换为PDF
% ========================================================================
function convertPngToPdf(pngFile, pdfFile)

img = imread(pngFile);

figHandle = figure( ...
    "Visible", "off", ...
    "Color", "w");

ax = axes( ...
    figHandle, ...
    "Position", [0, 0, 1, 1]);

image(ax, img);
axis(ax, "image");
axis(ax, "off");

exportgraphics( ...
    figHandle, pdfFile, ...
    "ContentType", "image", ...
    "Resolution", 600);

close(figHandle);

end