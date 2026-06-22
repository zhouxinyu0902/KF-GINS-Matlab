clear; clc;
glvs;

%% ==================== 0. 用户配置区 ====================

% 这里改成你当前想看的工况文件夹
% baseFolder = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Square_Beacon1_Trj\';
baseFolder = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Circle_Beacon1_Trj\';
% baseFolder = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Line_Beacon1_Trj\';

inputFolder = fullfile(baseFolder, 'input');

saveFig = true;      % 是否导出图片
figDpi  = 600;       % 导出分辨率

%% ==================== 1. 文件路径 ====================

pathpos    = fullfile(inputFolder, 'beacon_pos.mat');
truthPath1 = fullfile(inputFolder, 'truth.nav');
truthPath2 = fullfile(inputFolder, 'truth_stage2.nav');

if ~exist(pathpos, 'file')
    error('找不到文件：%s', pathpos);
end
if ~exist(truthPath1, 'file')
    error('找不到文件：%s', truthPath1);
end
if ~exist(truthPath2, 'file')
    error('找不到文件：%s', truthPath2);
end

[~, expName] = fileparts(baseFolder);

%% ==================== 2. 加载潜标位置 ====================

mapData = load(pathpos);

if ~isfield(mapData, 'pos0_geo')
    error('beacon_pos.mat 中缺少 pos0_geo。');
end
pos0_geo = mapData.pos0_geo;

if isfield(mapData, 'S_true_xyz')
    S_true_xyz = mapData.S_true_xyz;
elseif isfield(mapData, 'S_true_geo')
    S_true_xyz = pos2dxyz(mapData.S_true_geo, pos0_geo);
else
    error('beacon_pos.mat 中缺少 S_true_xyz 或 S_true_geo。');
end

if isfield(mapData, 'S_gnss_xyz')
    S_gnss_xyz = mapData.S_gnss_xyz;
elseif isfield(mapData, 'S_gnss_geo')
    S_gnss_xyz = pos2dxyz(mapData.S_gnss_geo, pos0_geo);
else
    error('beacon_pos.mat 中缺少 S_gnss_xyz 或 S_gnss_geo。');
end

%% ==================== 3. 加载阶段一/阶段二轨迹 ====================

truth1 = local_read_numeric(truthPath1);
truth2 = local_read_numeric(truthPath2);

auv_geo_st1 = local_extract_geo_from_truth(truth1, pos0_geo);
auv_geo_st2 = local_extract_geo_from_truth(truth2, pos0_geo);

auv_xyz_st1 = pos2dxyz(auv_geo_st1, pos0_geo);
auv_xyz_st2 = pos2dxyz(auv_geo_st2, pos0_geo);

%% ==================== 4. 画图：全局 + 局部 ====================

% fig = figure( ...
%     'Name', ['工况轨迹图_', expName], ...
%     'Color', [1 1 1], ...
%     'Position', [100 100 1300 520]);
fig = myfigurestartup(7,3,'zxy');
%% -------- 左图：全局图 --------
subplot(1,2,1);

plot(S_gnss_xyz(:,1)/1000, S_gnss_xyz(:,2)/1000, ...
    'b^', 'MarkerSize', 9, 'LineWidth', 2); hold on;

plot(S_true_xyz(:,1)/1000, S_true_xyz(:,2)/1000, ...
    'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

plot(auv_xyz_st1(:,1)/1000, auv_xyz_st1(:,2)/1000, ...
    'm-', 'LineWidth', 2);

plot(auv_xyz_st2(:,1)/1000, auv_xyz_st2(:,2)/1000, ...
    'g--', 'LineWidth', 2);

for i = 1:size(S_gnss_xyz,1)
    text(S_gnss_xyz(i,1)/1000, S_gnss_xyz(i,2)/1000, ...
        sprintf('  B%d', i), ...
        'FontSize', 10, 'FontWeight', 'bold');

    % GNSS参考点到真实发声点的偏移箭头
    quiver( ...
        S_gnss_xyz(i,1)/1000, S_gnss_xyz(i,2)/1000, ...
        (S_true_xyz(i,1)-S_gnss_xyz(i,1))/1000, ...
        (S_true_xyz(i,2)-S_gnss_xyz(i,2))/1000, ...
        0, 'k', 'LineWidth', 1.2, 'MaxHeadSize', 0.8);
end

grid on;
axis equal;
xlabel('东向 E (km)');
ylabel('北向 N (km)');
title('全局图');
legend( ...
    '潜标GNSS参考点', ...
    '水下真实发声点', ...
    '阶段一标定轨迹', ...
    '阶段二巡航轨迹', ...
    'GNSS到真实点偏移', ...
    'Location', 'best');

%% -------- 右图：局部放大图 --------
subplot(1,2,2);

plot(S_gnss_xyz(:,1), S_gnss_xyz(:,2), ...
    'b^', 'MarkerSize', 10, 'LineWidth', 2); hold on;

plot(S_true_xyz(:,1), S_true_xyz(:,2), ...
    'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

plot(auv_xyz_st1(:,1), auv_xyz_st1(:,2), ...
    'm-', 'LineWidth', 2.5);

plot(auv_xyz_st2(:,1), auv_xyz_st2(:,2), ...
    'g--', 'LineWidth', 1.5);

for i = 1:size(S_gnss_xyz,1)
    text(S_gnss_xyz(i,1), S_gnss_xyz(i,2), ...
        sprintf('  B%d', i), ...
        'FontSize', 10, 'FontWeight', 'bold');

    quiver( ...
        S_gnss_xyz(i,1), S_gnss_xyz(i,2), ...
        S_true_xyz(i,1)-S_gnss_xyz(i,1), ...
        S_true_xyz(i,2)-S_gnss_xyz(i,2), ...
        0, 'k', 'LineWidth', 1.2, 'MaxHeadSize', 0.8);
end

grid on;
axis equal;
xlabel('东向 E (m)');
ylabel('北向 N (m)');
title('局部放大图');

% 只根据阶段一标定轨迹 + 潜标位置自动设置局部视野
% xAll = [auv_xyz_st1(:,1); S_gnss_xyz(:,1); S_true_xyz(:,1)];
% yAll = [auv_xyz_st1(:,2); S_gnss_xyz(:,2); S_true_xyz(:,2)];
% local_set_axis_margin(xAll, yAll, 500);
xlim([-2000,2000]);
ylim([10000,14000]);
legend( ...
    '潜标GNSS参考点', ...
    '水下真实发声点', ...
    '阶段一标定轨迹', ...
    '阶段二巡航轨迹', ...
    'GNSS到真实点偏移', ...
    'Location', 'best');

sgtitle(['工况轨迹展示：', strrep(expName, '_', '\_')]);

%% ==================== 5. 导出图片 ====================

if saveFig
    savePath = fullfile(inputFolder, ['全局与局部轨迹图_', expName, '.png']);
    local_export_fig(fig, savePath, figDpi);
    fprintf('图像已导出：%s\n', savePath);
end

%% ==================== 本地辅助函数 ====================

function data = local_read_numeric(filepath)

    filepath = char(filepath);

    raw = importdata(filepath);
    if isstruct(raw)
        data = raw.data;
    else
        data = raw;
    end

    if isempty(data)
        error('文件为空或无法读取数值数据：%s', filepath);
    end
end

function geo = local_extract_geo_from_truth(truth, pos0_geo)
% 自动识别 truth 文件中的位置列

    candidateList = {};

    if size(truth, 2) >= 9
        candidateList{end+1} = truth(:, 7:9);
    end

    if size(truth, 2) >= 5
        candidateList{end+1} = truth(:, 3:5);
    end

    bestGeo = [];
    bestScore = inf;

    for k = 1:numel(candidateList)
        tmp = candidateList{k};

        lat = tmp(:,1);
        lon = tmp(:,2);
        h   = tmp(:,3);

        % 如果看起来像度数，就转弧度
        if median(abs(lat), 'omitnan') > pi/2 || median(abs(lon), 'omitnan') > pi
            lat = lat * pi / 180;
            lon = lon * pi / 180;
        end

        score = abs(median(lat, 'omitnan') - pos0_geo(1)) + ...
                abs(median(lon, 'omitnan') - pos0_geo(2));

        if score < bestScore
            bestScore = score;
            bestGeo = [lat, lon, h];
        end
    end

    if isempty(bestGeo)
        error('无法从 truth 数据中识别位置列。');
    end

    geo = bestGeo;
end

function local_set_axis_margin(x, y, margin)

    if nargin < 3 || isempty(margin)
        margin = 500;
    end

    xmin = min(x) - margin;
    xmax = max(x) + margin;
    ymin = min(y) - margin;
    ymax = max(y) + margin;

    xlim([xmin, xmax]);
    ylim([ymin, ymax]);
end

function local_export_fig(figHandle, savePath, dpi)

    try
        exportgraphics(figHandle, savePath, 'Resolution', dpi);
    catch
        warning('exportgraphics 不可用，改用 print 导出。');
        [folderPath, fileName, ~] = fileparts(savePath);
        if ~exist(folderPath, 'dir')
            mkdir(folderPath);
        end
        print(figHandle, fullfile(folderPath, fileName), '-dpng', ['-r', num2str(dpi)]);
    end
end