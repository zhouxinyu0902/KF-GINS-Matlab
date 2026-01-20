folderPath='D:\Github\KF-GINS-Matlab';
if ~exist(folderPath, 'dir')
    warning('文件夹不存在: %s', folderPath);
    return;
end

% 获取该文件夹及其所有子文件夹的完整路径字符串
fullPathList = genpath(folderPath);

% 移除旧路径（如果存在）
% 注意：如果路径非常多，contains 检查是必要的以防报错
currPath = path;
if contains(currPath, folderPath)
    rmpath(fullPathList);
    disp(['已清理子路径: ', folderPath]);
end

% 重新添加
addpath(fullPathList);
savepath; % (可选) 如果你想永久保存这个更改

disp(['路径已重置并刷新: ', folderPath]);