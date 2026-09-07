function [TT,tt,DistData]=timepro(time,TimeZone,DistData,type)
[DistData,TimeInSec]=timeprocess(DistData,TimeZone);
[tstart,tend]=timechoose(time,TimeInSec);%选择时间段
switch(type)
    case 'LBL'
        index=tstart(1):tend(end);
    case 'USBL'
        index=tstart:tend;
    otherwise
        disp('TYPE ERROR')
end
TT=TimeInSec(index);
tt=timeindexprocess(TT); %% 时间索引处理,将时间秒数转为以0开始的时间点
DistData=DistData(:,index);
end
function [tstart_1,tend_1]=timechoose(time,TimeInSec)
% 根据选择的一段时间，将TimeInSec对应的一段时间的起始索引和结束索引找出来
% tstart = time(1)*3600+time(2)*60+time(3);
% tend = time(4)*3600+time(5)*60+time(6);
tstart = time(1);
tend = time(2);
tstart_1=find(TimeInSec==tstart);
tend_1=find(TimeInSec==tend); 
end
function t=timeindexprocess(TimeInSec)
% 根据TimeInSec数据得到起始为零的时间序列
TimeInSec=TimeInSec-TimeInSec(1);
for i=1:length(TimeInSec)-2
    if (TimeInSec(i+1)==TimeInSec(i))||(TimeInSec(i+1)==TimeInSec(i+2))
        TimeInSec(i+1)=TimeInSec(i)+0.5;
        TimeInSec(i+2)=TimeInSec(i)+1;
    elseif TimeInSec(i+1)==TimeInSec(i)
        TimeInSec(i+1)=TimeInSec(i)+0.5;
    end
end
t=TimeInSec;
end
function [data,TimeInSec]=timeprocess(data,TimeZone)
% 根据时区和时分秒数据，得到转换后的TimeInSec
TimeInSec = data(end-2,:)*3600+data(end-1,:)*60+data(end,:)+TimeZone*3600;
I=find(TimeInSec-TimeInSec(1)<0);  %如果后面的时间变小了，说明跨越了格林威治时间午夜，需要把午夜前的时间减去24小时
if ~isempty(I)
    TimeInSec(I) = TimeInSec(I)+24*3600;
    TimeInSec = TimeInSec-24*3600; %所有数据都没有在北京时间0点前的，所以不会出现负值
end
[TimeInSec,I] = sort(TimeInSec);
data=data(:,I);
end

