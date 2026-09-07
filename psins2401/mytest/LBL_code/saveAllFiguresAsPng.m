function saveAllFiguresAsPng(dpi)
% saveAllFiguresAsPng(dpi)
% 保存所有当前打开的 MATLAB figure 为 PNG 格式。
% 每个 figure 将以其句柄作为文件名（例如 'Figure_1.png', 'Figure_2.png' 等）。
%
% 输入:
%   dpi - 输出图像的分辨率，例如 600 代表 600 DPI。

if nargin < 1
    dpi = 600; % 默认分辨率为 600 DPI
end

% 查找所有类型为 'figure' 的图形对象
hFigs = findall(0, 'Type', 'figure');

if isempty(hFigs)
    fprintf('当前没有打开的 figure，无需保存。\n');
    return;
end

fprintf('正在保存 %d 个 figure 为 PNG 格式，分辨率 %d DPI...\n', length(hFigs), dpi);

% 遍历所有找到的 figure
for k = 1:length(hFigs)
    currentFig = hFigs(k);

    % 获取 figure 的句柄或编号
    figNum = currentFig.Number; % 或者 currentFig.Tag 如果你给 figure 设置了Tag

    % 构建文件名
    fileName = sprintf('Figure_%d.png', k+10);

    % 设置当前 figure 为活动 figure (可选，但推荐，以确保 print 函数作用于正确的 figure)
    figure(currentFig);

    % 使用 print 命令保存 figure
    % -dpng: 指定输出格式为 PNG
    % -r<dpi>: 指定分辨率
    % -opengl: 使用 OpenGL 渲染器，有时可以改善输出质量
    % -painters: 使用矢量渲染器，对于线条图效果更好，但可能不适用于复杂图形
    % 建议根据你的图内容选择合适的渲染器。对于大多数科学绘图，-opengl 或不指定通常足够。
    % 如果是高质量的出版物图，可以尝试 -painters 或 -vector
    
    try
        print(currentFig, fileName, '-dpng', ['-r', num2str(dpi)]);
        fprintf('  已保存: %s\n', fileName);
    catch ME
        fprintf('  保存 %s 失败: %s\n', fileName, ME.message);
    end
end

fprintf('所有 figure 保存操作完成。\n');

end

% --- 使用示例 ---
% 假设你已经有几个打开的 figure
% 例如：
% figure(1); plot(rand(10)); title('Figure 1');
% figure(2); surf(peaks); title('Figure 2');
% figure(3); imagesc(magic(5)); colorbar; title('Figure 3');

% 调用函数来保存所有 figure
% saveAllFiguresAsPng(600); % 保存为 600 DPI
% 或者使用默认值 (600 DPI)
% saveAllFiguresAsPng();