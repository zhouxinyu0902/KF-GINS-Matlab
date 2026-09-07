function [index0,index1,index3,index3_back,index3_for] = repeated(TT)
[uniqueNums, ~, idx] = unique(TT);% 使用unique函数找到数列中的不同数及其出现的次数
counts = accumarray(idx,1);
repeatedNums3 = uniqueNums(counts==3);% 找出出现3次的数
repeatedNums1 = uniqueNums(counts==1);% 找出出现1次的数
missingNums = setdiff(TT(1):TT(end),TT); % 找出缺失的数 在选择时间段内不缺失值
ki=1;
kii=1;
index3_for=[];
index3_back=[];
for i=1:length(repeatedNums3)
    for j=1:length(repeatedNums1)
        if repeatedNums1(j)-repeatedNums3(i)==1
            repeated3_back(ki)=repeatedNums3(i);index3_back(ki)=find(TT==repeated3_back(ki), 1, 'last' ); ki=ki+1;
        elseif repeatedNums1(j)-repeatedNums3(i)==-1
            repeated3_for(kii)=repeatedNums3(i); index3_for(kii)=find(TT==repeated3_for(kii), 1 );kii=kii+1;
        end
    end
end
index0=FindIndex(missingNums,TT);
index1=FindIndex(repeatedNums1,TT);
index3=FindIndex(repeatedNums3,TT);
end
function index=FindIndex(sequence,TT)
index=[];
for i=1:length(sequence)
    index(i)=median(find(TT==sequence(i)));
end
end
