clear
load('03_sum\2023\matData\PHINS-sample.mat')
glvs
%% 时间，各类数据的起始和结束时间
format bank;
data1=["depth","DVL","GPS","Ins_D","Ins_Nav"];
data={'depth','DVL','GPS','Ins_D','Ins_Nav'};
disp(data)
for i=1:5
    value=eval(data{i});
    start(i)=value(end,1);
    endtime(i)=value(end,end);
end
disp(start)
disp(endtime)
disp(t)
% 选择起始时间和终止时间：30605~42421 
%% 深度计和GPS绘图
% figure,plot(Ins_Nav(3,:)) % 越来越往上,向上的速度是正的
tt=7900;
t1=tt+1250;
myfigurestartup(10,5,'prese')
plot(GPS(2,1:tt),GPS(1,1:tt))
hold on
plot(GPS(2,tt:t1),GPS(1,tt:t1))
hold on 
plot(GPS(2,t1:end),GPS(1,t1:end))
xygo('lon','lat')
legend('PHASE 1','PHASE 2','PHASE 3')
%% 选择一个时间段的数据
format bank;
tstart=39920;
tend=41120;

t1=find(abs(GPS(end,:)-tstart)<0.5);
t2=find(abs(GPS(end,:)-tend)<0.5);
figure
plot(GPS(2,t1:t2),GPS(1,t1:t2))
% 一共1200s数据20min
GPS_chosen=GPS(:,t1:t2);

% 0.01s IMU
t1=find(abs(Ins_D(end,:)-tstart)<0.008)
t2=find(abs(Ins_D(end,:)-tend)<0.005)
Ins_D_chosen=Ins_D(:,t1:t2);

% 0.2s DVL
t1=find(abs(DVL(end,:)-tstart)<0.15)
t2=find(abs(DVL(end,:)-tend)<0.13)
DVL(end,t2)-DVL(end,t1)
DVL_chosen=DVL(:,t1:t2);

% 1s INS-NAV
t1=find(abs(Ins_Nav(end,:)-tstart)<0.8)
t2=find(abs(Ins_Nav(end,:)-tend)<0.8)
Ins_Nav(end,t2)-Ins_Nav(end,t1)
Ins_Nav_chosen=Ins_Nav(:,t1:t2);

% 1s 深度计
t1=find(abs(depth(end,:)-tstart)<0.7)
t2=find(abs(depth(end,:)-tend)<0.7)
depth(end,t2)-depth(end,t1)
depth_chosen=depth(:,t1:t2);
 
tt_nav=Ins_Nav_chosen(end,:)-Ins_Nav_chosen(end,1);
tt_gps=GPS_chosen(end,:)-GPS_chosen(end,1);
tt_dvl=DVL_chosen(end,:)-DVL_chosen(end,1);
tt_IMU=Ins_D_chosen(end,:)-Ins_D_chosen(end,1);

clear tt t1 t2 tend tstart

format bank;
data={'depth_chosen','DVL_chosen','GPS_chosen','Ins_D_chosen','Ins_Nav_chosen'};
disp(data)
for i=1:5
    value=eval(data{i});
    start(i)=value(end,1);
    endtime(i)=value(end,end);
end
disp(start)
disp(endtime)
%% 姿态速度位置绘图
attitude=[d2r([-Ins_Nav_chosen(6,:);Ins_Nav_chosen(5,:)]);yawcvt(d2r(Ins_Nav_chosen(4,:))')'];
% yawcvt：默认将顺时针0~360变成-180-180
speed_n=[Ins_Nav_chosen(8,:);Ins_Nav_chosen(7,:);Ins_Nav_chosen(9,:)]; % Ins_Nav，NEU转为ENU
position=[d2r([Ins_Nav_chosen(1,:);Ins_Nav_chosen(2,:)]);Ins_Nav_chosen(3,:);tt_nav];
positiongps=[d2r([GPS_chosen(1,:);GPS_chosen(2,:)]);GPS_chosen(3,:);tt_gps];

myfigurestartup(12,8,'prese')
string={'pitch','roll','heading','east','north','up'};
for i=1:3
subplot(3,3,i)
plot(tt_nav(end,:),r2d(attitude(i,:)))
title(string{i})
xygo('t/s','angel/deg')
grid on
subplot(3,3,i+3)
plot(tt_nav(end,:),speed_n(i,:))
grid on
title(string{i+3})
xygo('t/s','velocity/(m/s)')
end
dxyz=pos2dxyz(position',position(1:3,1));
subplot(3,3,[7,8,9]),plot(dxyz(:,1),dxyz(:,2)),xygo('east/m','north/m')
hold on
dxyz1=pos2dxyz(positiongps',positiongps(1:3,1));
plot(dxyz1(:,1),dxyz1(:,2)),xygo('east/m','north/m')
% subplot(3,3,[7,8,9]),plot(r2d(position(2,:)),r2d(position(1,:))),xygo('lat','lon')
%% DVL对准与ins-nav对比，在{b}中比较，因为参考姿态是1Hz，DVL是5Hz。
% close all
misaligment=d2r([-0.033;0;44.812]);% 安装角 
% PHINS:pitch头朝下为正，heading顺时针为正
% 工具箱:pitch头朝上为正，heading北偏西为正
Cbd=a2mat(misaligment); 
% 安装测量系{d}与载体坐标系{b}的旋转矩阵
scale=0.0171;
% scale=0.05;
% speed_dvl=[-DVL(2,:);DVL(1,:);DVL(3,:);]/(1-scale); % DVL测的速度前左上，按照右前上排序
% DVL测的速度右手坐标系右前上，按照右前上排序
speed_dvl=[DVL_chosen(1,:);DVL_chosen(2,:);DVL_chosen(3,:);]/(1-scale); 
for i=1:length(speed_dvl)
    % 将DVL测得安装坐标系的速度转换到载体坐标系中 
    speed_dvl_b(:,i)=Cbd*speed_dvl(:,i);      
end
% ins-nav速度ENU，转换到载体坐标系中
for i=1:length(attitude)
    Cnb=a2mat(attitude(:,i)); % Cnb,Cnb'=Cbn
    speed_b(:,i)=Cnb'*speed_n(:,i); % 参考的载体坐标系的速度
end

myfigurestartup(12,4,'prese')
string={'right','front','up'};
for i=1:3
    subplot(1,3,i)
    plot(tt_nav(end,:),speed_b(i,:))
    hold on
    plot(tt_dvl(end,:),speed_dvl_b(i,:))
    legend('Ins-Nav','DVL')
    grid on
    title(string{i})
end
%% IMU与参考结果的增量比较，在{b}系里边
% 首先将参考结果的增量转到{b}里边，att不用，vel需要(已转)
attitude_diff=diff(attitude,1,2);
attitude0=attitude(:,1);
speed_b_diff=diff(speed_b,1,2);
speed_b_0=speed_b(:,1);
% 然后计算IMU的增量和IMU积分值
gyro=d2r([-Ins_D_chosen(2,:);Ins_D_chosen(1,:);Ins_D_chosen(3,:)]*1e-3/3600);
acc=[-Ins_D_chosen(2,:);Ins_D_chosen(1,:);Ins_D_chosen(6,:)]*1e-7;

position=[d2r([Ins_Nav_chosen(1,:);Ins_Nav_chosen(2,:)]);Ins_Nav_chosen(3,:)];
tt=Ins_Nav_chosen(end,:)-Ins_Nav_chosen(end,1);
avp=[attitude',speed_n',position',tt'];

eth = earth(position(:,1), speed_n(:,1));
attitude_sum=attitude0+cumsum(gyro,2);
speed_b_sum=speed_b_0+cumsum(acc,2);

myfigurestartup(12,8,'prese')
string={'pitch','roll','heading','dpitch','droll','dheading'};
for i=1:3
    % 角度
    subplot(2,3,i)
    plot(tt_nav(end,:),r2d(attitude(i,:)))
    hold on
    plot(tt_IMU(end,:),r2d(attitude_sum(i,:)))
    legend('Ins-Nav','IMU')
    xygo('t/s','angel/deg')
    title(string{i})
    % 角度增量
    subplot(2,3,i+3)
    plot(tt_IMU(end,:),r2d(gyro(i,:)))
    hold on
    plot(tt_nav(end,2:end),r2d(attitude_diff(i,:)/100),'.')
    legend('IMU','Ins-Nav')
    xygo('t/s','angel/deg')
    title(string{i+3})
end
%% 常值补偿
% IMU
% 陀螺仪
% close all
pitch_d=(-Ins_D_chosen(2,:)+90)/(1+0);
roll_d=(Ins_D_chosen(1,:)+90)/(1+0);
heading_d=(Ins_D_chosen(3,:)-0)/(1+0);
gyro_bias_compensate=d2r([pitch_d;roll_d;heading_d]*1e-3/3600);
% 加速度计
acc_bias_compensate=([Ins_D_chosen(4,:);Ins_D_chosen(5,:);Ins_D_chosen(6,:)]-400)/(1+0.04)*1e-7;
attitude_sum_1=attitude0+cumsum(gyro_bias_compensate,2);
myfigurestartup(12,8,'prese')
for i=1:3
    subplot(2,3,i)
    plot(tt_nav(end,:),r2d(attitude(i,:)))
    hold on
    plot(tt_IMU(end,:),r2d(attitude_sum_1(i,:)))
    legend('Ins-Nav','IMU')
    title(string{i})
    grid on   
    subplot(2,3,i+3)
    plot(tt_IMU(end,:),r2d(gyro_bias_compensate(i,:)))
    hold on
    plot(tt_nav(end,2:end),r2d(attitude_diff(i,:)/50))
    legend('IMU','Ins-Nav')
    title(string{i+3})
    grid on
    % ylim([-0.1,0.1]); 
end
%% 
avp(:,8)=positiongps(2,:)';
% imuplot(imu)
%% 惯导解算
% close all
time=Ins_D_chosen(end,:)-Ins_D_chosen(end,1);
% 坐标轴的转换
gyro=d2r([-Ins_D_chosen(2,:);Ins_D_chosen(1,:);Ins_D_chosen(3,:)]*1e-3/3600);
acc=[-Ins_D_chosen(5,:)+200;Ins_D_chosen(4,:)+200;Ins_D_chosen(6,:)+200]*1e-7;
imu=[gyro;acc;time]';
ts=0.01;
avp0=[avp(1,1:6)';d2r(GPS_chosen(1:2,1));-depth_chosen(1,1)];
ins = myins('initial',ts,avp0);
ll=length(imu);
avp_ins=prealloc(ll,10);
for i=1:ll
    t=imu(i,end);
    ins = myins('update',ins,imu(i,1:6));
    ins.pos(3)=-1.2;
    avp_ins(i,:)=[ins.avp',t];
end
%% 绘图
myfigurestartup(10,10,'prese')
insplot(avp)
figure
insplot(avp_ins)
avpcmpplot(avp,avp_ins);
