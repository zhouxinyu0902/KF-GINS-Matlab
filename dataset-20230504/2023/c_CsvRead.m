clear
LOG=[];
Time=[];
% 名称定好
% for i=0:7
%    s{i+1}=sprintf('log_20230503_0%02d.csv',i); 
% end
% for i=0:37
%    s{i+9}=sprintf('log_20230504_0%02d.csv',i);
% end
% s={'log_20230504_000.csv'};
for i=0:4
   s{i+1}=sprintf('log_20230504_0%02d.csv',i);
end
% 读取文件
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
% tsec(:,1)=tt(:,1)*3600+tt(:,2)*60+tt(:,3)+86400*(tt(:,1) >= 0&tt(:,1) <= 20);
tsec(:,1)=tt(:,1)*3600+tt(:,2)*60+tt(:,3); % 没有跨一天
time=LOG(:,1); 
% 位置
lat=LOG(:,72);
lon=LOG(:,73);
latgps=LOG(:,10);
longps=LOG(:,11);
latins=LOG(:,35);
lonins=LOG(:,37);

depthins=LOG(:,33);
depthall=LOG(:,68);

% 速度
dvllong=LOG(:,20);
dvltran=LOG(:,21);
dvlvert=LOG(:,24);
% DVL_ref=[dvllong,dvltran,dvlvert];
DVL_ref=[-dvltran,dvllong,dvlvert];
insVlong=LOG(:,36); % 纵向速度（前进）
insVtran=LOG(:,38);
insVvert=LOG(:,39);
% ins_ref=[insVlong,insVtran,insVvert];
ins_ref=[-insVtran,insVlong,insVvert];% 统一为右前上

myfigurestartup(12,5,'prese')
string={'right','front','up'};
for i=1:3
subplot(1,3,i)
plot(tsec,DVL_ref(:,i))
hold on
plot(tsec,ins_ref(:,i))
legend('DVL','ins')
title(string{i})
ConvertXAxisTime;
end

% DVL对准
scale=0.09721;
speed_dvl=DVL_ref/(1-scale); % DVL刻度系数补偿

myfigurestartup(12,5,'prese')
string={'right','front','up'};
for i=1:3
subplot(1,3,i)
plot(tsec,ins_ref(:,i))
hold on
plot(tsec,speed_dvl(:,i))
legend('ins','DVL')
title(string{i})
end
%% 姿态
heading=LOG(:,69);
headingrate=LOG(:,70);
pitch=LOG(:,74);
pitchrate=LOG(:,75);
roll=LOG(:,76);
rollrate=LOG(:,77);
attitude_ref=[d2r([-pitch,roll]),yawcvt(d2r(-heading))];
anglerate_ref=[-pitchrate,rollrate,-headingrate];

save 2023\matData\imualignment DVL_ref ins_ref attitude_ref anglerate_ref tsec
data=[time,lat,lon,depthall,latgps,longps,latins,lonins,depthins,...
    dvllong,dvltran,dvlvert,insVlong,insVtran,insVvert,...
    heading,headingrate,pitch,pitchrate,roll,rollrate,tsec];

% velocity=[time,dvllong,dvltran,dvlvert,insVlong,insVtran,insVvert,tsec];
% save 2023\matData\velocity-0017.mat velocity

% clearvars -except data


%% 轨迹和深度
figure
subplot 121
plot(data(1,3),data(1,2),'*', ...
    data(:,3),data(:,2),'.')
grid on

subplot 122
plot(data(:,end),data(:,4))
ConvertXAxisTime;
%% 走直线的部分
num=17*60+86400; % 00:17:00开始
num1=30618+86400; % 开始上升的时刻
index=find(data(:,end)-num<0.1&data(:,end)-num>0);
index1=find(data(:,end)-num1<0.1&data(:,end)-num1>0);

figure
subplot 121
plot(data(index,3),data(index,2),'*', ...
    data(index:index1,3),data(index:index1,2))
grid on

subplot 122
plot(data(index:index1+20,end),data(index:index1+20,4))
ConvertXAxisTime;

datastraight=data(index:index1+20,:);
%%
save 2023/data.mat data datastraight