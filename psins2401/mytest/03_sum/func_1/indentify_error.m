function [data,TT] = indentify_error(TT,data,size,thres,yla)
%INDENTIFY_ERROR 识别异常值的索引 
% 示例数据  
% data = [10, 12, 13, 12, 100, 14, 15, 16]; % 100 是一个异常值     
% 初始化结果数组  
result = zeros(1, length(data));  
% size=10;  
% 遍历数组
for i = 1:length(data)  
    % 对于第一个点，只有后一个点和右边（如果索引存在）  
    if i >= 1 && i <= size+1
        diff_sum =mean(abs(data(i) - data(i+1:i+size))) ; % 假设左右邻居间隔为1  
    % 对于最后一个点，只有前一个点和左边（如果索引存在）  
    elseif i >= length(data)-size
        diff_sum = mean(abs(data(i) - data(i-size:i-1))); % 假设左右邻居间隔为1  
    % 对于其他点，计算与2k个邻居的差值  
    else  
        diff_sum = mean(abs(data(i) - data([i-size:i-1,i+1:i+size])));% 假设左右邻居间隔为1  
    end  
    % 存储结果  
    result(i) = diff_sum;  
end  
id=result>thres;
% 绘图
myfigurestartup(2.5,2.5,'paper');set(0,'defaultLineMarkerSize',4)
yyaxis right,plot(TT,result,'.');xygo('hh mm ss','KNN-Range')
hold on,plot(TT(1):0.1:TT(end), ...
    ones(1,length(TT(1):0.1:TT(end)))*thres)

yyaxis left,
set(0,'defaultLineMarkerSize',6)
plot(TT,data,'.');
set(0,'defaultLineMarkerSize',5)
hold on,plot(TT(id),data(id),'o','Color','g')
xlim([TT(1) TT(end)])
ylabel(yla)
legend('raw data','abnormal','knn-range','threshold')
ConvertXAxisTime
data(id)=[];
TT(id)=[];
switch(yla(1))
    case 'l'
        DeciPoin(0,3)
end
end

