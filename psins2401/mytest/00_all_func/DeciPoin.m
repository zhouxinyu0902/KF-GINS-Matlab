function DeciPoin(xx,yy)
%DOT 此处显示有关此函数的摘要
%   此处显示详细说明
% 设置 x 轴和 y 轴刻度标签的格式
yticks = get(gca, 'YTick');  % 获取当前的 y 轴刻度位置
yformatSpec = sprintf('%%.%df', yy);
yticklabels = arrayfun(@(y) num2str(y, yformatSpec), yticks, 'UniformOutput', false);
set(gca, 'YTickLabel', yticklabels);% 设置格式化后的刻度标签
if xx~=0
    xticks = get(gca, 'XTick');
    xformatSpec = sprintf('%%.%df', xx);
    xticklabels = arrayfun(@(x) num2str(x, xformatSpec), xticks, 'UniformOutput', false);
    set(gca, 'XTickLabel', xticklabels);
end
end

