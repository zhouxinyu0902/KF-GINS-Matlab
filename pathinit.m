%% MATLAB 路径初始化
clear;
clc;

%% ============================================================
% 0. 临时处理 MATLAB 内置 Perl 的中文 Locale 问题
% =============================================================

oldLCALL = getenv('LC_ALL');
oldLANG  = getenv('LANG');

setenv('LC_ALL', 'C');
setenv('LANG', 'C');

%% ============================================================
% 1. 恢复 MATLAB 官方默认路径
% =============================================================

restoredefaultpath;
rehash toolboxcache;

%% ============================================================
% 2. 添加自己的工程
% =============================================================

folderPath = 'D:\Github\KF-GINS-Matlab';

if exist(folderPath, 'dir')
    addpath(genpath(folderPath));
    fprintf('已添加工程路径：\n%s\n', folderPath);
else
    warning('工程目录不存在：%s', folderPath);
end

%% ============================================================
% 3. 清理仍然不存在的路径
% =============================================================

pathList = strsplit(path, pathsep);
pathList = pathList(~cellfun('isempty', pathList));

invalidMask = false(size(pathList));

for i = 1:numel(pathList)
    invalidMask(i) = exist(pathList{i}, 'dir') ~= 7;
end

invalidPaths = pathList(invalidMask);

if ~isempty(invalidPaths)

    fprintf('\n发现 %d 个无效路径：\n', numel(invalidPaths));

    for i = 1:numel(invalidPaths)
        fprintf('删除：%s\n', invalidPaths{i});
        rmpath(invalidPaths{i});
    end

else
    fprintf('\n没有发现无效路径。\n');
end

%% ============================================================
% 4. 永久保存
% =============================================================

status = savepath;

if status == 0
    fprintf('\nMATLAB 路径已重新构建并永久保存。\n');
else
    warning('savepath 保存失败。');
end

%% ============================================================
% 5. 恢复原来的环境变量
% =============================================================

setenv('LC_ALL', oldLCALL);
setenv('LANG', oldLANG);