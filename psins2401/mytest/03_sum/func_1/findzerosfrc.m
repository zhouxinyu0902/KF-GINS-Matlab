function [index,startIndices]=findzerosfrc(v_ym,len)
% 初始化索引数组
startIndices = [];
% 遍历v_ym，寻找连续为0的片段
startIndex = 0; % 记录连续为0的片段的起始索引
k=1;
for i = 1:length(v_ym)
    if v_ym(i) == 0
        if startIndex == 0 % 尚未开始记录一个片段
            startIndex = i; % 开始记录当前片段的起始索引
        end
    else
        if startIndex ~= 0 && i - startIndex < len % 如果找到了片段的结束，并且长度小于20
            startIndices(1:2,k) = [startIndex, i-1]; % 保存起始索引
            k=k+1;
        end
        startIndex = 0; % 重置startIndex，开始搜索下一个片段
    end
end
index=[];
for i=1:length(startIndices)
    index=[index,startIndices(1,i):startIndices(2,i)];
end
end
