folderPath = 'D:\Github\KF-GINS-Matlab';

%% 1. 检查工程目录
if ~exist(folderPath, 'dir')
    warning('文件夹不存在: %s', folderPath);
    return;
end

%% 2. 清理 MATLAB path 中已经不存在的路径
pathList = strsplit(path, pathsep);
pathList = pathList(~cellfun('isempty', pathList));

invalidMask = false(size(pathList));

for i = 1:numel(pathList)
    invalidMask(i) = ~exist(pathList{i}, 'dir');
end

invalidPaths = pathList(invalidMask);

if ~isempty(invalidPaths)
    fprintf('发现 %d 个不存在的路径，开始清理：\n', numel(invalidPaths));

    for i = 1:numel(invalidPaths)
        fprintf('  删除: %s\n', invalidPaths{i});
        rmpath(invalidPaths{i});
    end
else
    disp('未发现不存在的 MATLAB 路径。');
end

%% 3. 清理当前工程已有的旧路径
pathList = strsplit(path, pathsep);

isProjectPath = startsWith(pathList, folderPath, ...
    'IgnoreCase', true);

oldProjectPaths = pathList(isProjectPath);

if ~isempty(oldProjectPaths)
    rmpath(oldProjectPaths{:});
    fprintf('已清理 KF-GINS-Matlab 旧路径，共 %d 个。\n', ...
        numel(oldProjectPaths));
end

%% 4. 重新生成工程路径
fullPathList = genpath(folderPath);

%% 5. 添加工程及所有子目录
addpath(fullPathList);

fprintf('已重新添加工程路径：\n%s\n', folderPath);

%% 6. 永久保存 MATLAB 路径
status = savepath;

if status == 0
    disp('MATLAB 路径已永久保存。');
else
    warning('savepath 保存失败，当前 MATLAB 会话仍然有效。');
end