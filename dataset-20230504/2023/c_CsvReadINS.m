clear
LOG=[];
Time=[];
% for i=0:7
%    s{i+1}=sprintf('ins_20230503_0%02d.csv',i); 
% end
% for i=0:37
%    s{i+9}=sprintf('ins_20230504_0%02d.csv',i);
% end
s={'log_20230504_001.csv'};
% 纬度经度 前进速度 三个姿态角 深度 高度
for i=1:length(s)
    [log,tct]=xlsread(s{i});
    LOG=[LOG;log];
    Time=[Time;
      tct(4:end,end)];
end
for i=1:length(Time)
    tt(i,1:3)=sscanf(Time{i},'%d:%d:%f');
end
%%
tsec(:,1)=tt(:,1)*3600+tt(:,2)*60+tt(:,3)+86400*(tt(:,1) >= 0&tt(:,1) <= 20);
insdata=[LOG,tsec];
save 2023/insdata.mat insdata

