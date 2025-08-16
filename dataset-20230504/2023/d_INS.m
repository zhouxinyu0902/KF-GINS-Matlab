clear
load('sum\2023\matData\PHINS.mat')
load('2023\matData\imualignment.mat')
glvs
figure,plot(Ins_Nav(13,:)) % 越来越往上,向上的速度是正的
%% PHINS参考系为前左上,工具箱为右前上
%% DVL对准
misaligment=d2r([2.171;0;-45.317]); % 对准角
Cbm=a2mat(misaligment); % 安装测量系与载体坐标系的旋转矩阵
scale=0.09721;
speed_dvl=[-DVL(8,:);DVL(9:10,:)]/(1); % DVL测得速度
for i=1:length(speed_dvl)
    speed_dvl_b(:,i)=Cbm*speed_dvl(:,i);
end
string={'right','front','up'};
for i=1:3
subplot(1,3,i)
plot(tsec,DVL_ref(:,i))
hold on
plot(DVL(end,:),speed_dvl_b(i,:))
legend('DVL-ref','DVL')
title(string{i})
end
%% NEU速度投影到b系：右前上
attitude=[d2r([-Ins_Nav(16,:);Ins_Nav(15,:)])',yawcvt(d2r(-Ins_Nav(14,:)'))];
speed=[-Ins_Nav(18,:);Ins_Nav(17,:);Ins_Nav(19,:)]; % Ins_Nav,NEU速度
for i=1:length(attitude)
   matr=a2mat(attitude(i,:));% Cnb
   speed_b(:,i)=matr'*speed(:,i);  % 参考的载体坐标系的速度
end
myfigurestartup(12,5,'prese')
string={'right','front','up'};
for i=1:3
subplot(1,3,i)
plot(Ins_Nav(end,:),speed_b(i,:))
hold on
plot(tsec,ins_ref(:,i))
legend('INS-Nav','INS-LOG') % log中的是载体坐标系中的速度
title(string{i})
end
%% PHINS与AUV对齐
v0=speed_dvl_b(:,1); % 初始的载体坐标系速度
att0=attitude(1,:); % 初始姿态
gyro=d2r([-Ins_D(9,:);Ins_D(10,:);-Ins_D(11,:)]*1e-3/3600);
acc=[-Ins_D(13,:);Ins_D(12,:);Ins_D(14,:)]*1e-7; 
time=Ins_D(end,:)-Ins_D(end,1);
% accel=acc/0.02;
% angel=atan(accel(1,:)./accel(3,:));
% angel_mean=mean(atan(accel(1,:)./accel(3,:)));
% misaligment_imu=[zeros(size(angel));-angel;zeros(size(angel))]';
% misaligment_imu_1=[0;-angel_mean;0]';
% for i=1:length(angel)
%     accel_aligned(:,i)=a2mat(misaligment_imu(i,:))*accel(:,i);
%     accel_aligned_2(:,i)=a2mat(misaligment_imu_1)*accel(:,i);
% end
%% 这种计算安装角的方式是假设左右向加速度为0
angel=atan(acc(1,:)./acc(3,:));
angel_mean=mean(atan(acc(1,:)./acc(3,:)));
misaligment_imu=[zeros(size(angel));-angel;zeros(size(angel))]';
misaligment_imu_1=[0;-angel_mean;0]';
for i=1:length(angel)
    acc_aligned(:,i)=a2mat(misaligment_imu(i,:))*acc(:,i);% 使用每个时刻的对准
    acc_aligned_2(:,i)=a2mat(misaligment_imu_1)*acc(:,i);% 使用平均值对准
    gyro_aligned(:,i)=a2mat(misaligment_imu(i,:))*gyro(:,i);
    gyro_aligned_2(:,i)=a2mat(misaligment_imu_1)*gyro(:,i);
end
% figure,plot(angel)
% figure,plot(Ins_Nav(end,:),r2d(attitude(:,2)))
%% 姿态
att=cumsum(gyro_aligned_2,2);
figure
string={'pitch','roll','heading'};
for i=1:3
subplot(1,3,i)
plot(Ins_D(end,:),r2d(att(i,:)+att0(i)))
hold on
plot(Ins_Nav(end,:),r2d(attitude(:,i)))
hold on
plot(tsec,r2d(attitude_ref(:,i)))
legend('IMU','INS-Nav','INS-LOG')
title(string{i})
end
%% 速度
position=[d2r([Ins_Nav(11,:);Ins_Nav(12,:)]);Ins_Nav(13,:)];
tt=Ins_Nav(end,:)-Ins_Nav(end,1);
avp=[attitude,speed',position',tt'];
myfigurestartup(10,10,'prese')
insplot(avp)
depth_intep=interp1(depth(end,:),depth(9,:),Ins_D(end,:),'linear');

eth = earth(position(:,1), speed(:,1));
acc1=acc_aligned;
acc1(3,:)=acc(3,:)-eth.g*0.02;
vvv=cumsum(acc,2);
figure
for i=1:3
subplot(1,3,i)
plot(Ins_D(end,:),vvv(i,:)+v0(i))
hold on
plot(DVL(end,:),speed_dvl_b(i,:))
legend('IMU','DVL')
end
%%
imu=[gyro;acc;time]';
imuplot(imu);
%% 使用已有的计算结果
datt=diff(avp(:,1:3));
dvel=diff(speed');

myfigurestartup(12,5,'prese')
for i=1:3
subplot(1,3,i)
plot(Ins_Nav(end,2:end),datt(:,i)/50)
hold on
plot(Ins_D(end,:),gyro(i,:))
legend('ref','IMU')
end
myfigurestartup(12,5,'prese')
for i=1:3
subplot(1,3,i)
plot(Ins_D(end,:),acc(i,:))
hold on
plot(Ins_Nav(end,2:end),dvel(:,i)/50)
legend('IMU','ref')
end
%% 惯导解算
gyro=d2r([-Ins_D(9,:);Ins_D(10,:);-Ins_D(11,:)]*1e-3/3600);
acc=[-Ins_D(13,:);Ins_D(12,:);Ins_D(14,:)]*1e-7; 
time=Ins_D(end,:)-Ins_D(end,1);
imu=[gyro;acc;time]';
ts=0.02;
avp_ins=[];
ins = myins('initial',ts,avp(1,1:9)');
% ins.dvbias=[-15;-34;-15];% 单位为mGAL
% ins.phimbias=-[0.027;0.027;0.027]; % 单位为deg/h
ins.phimbias=zeros(3,1);
ins.dvbias=zeros(3,1);
ll=length(imu);
for i=1:ll
    t=imu(i,end);
    ins = myins('update',ins,imu(i,1:6));
    ins.pos(3)=-depth_intep(i);
    avp_ins(i,:)=[ins.avp',t];
end
myfigurestartup(10,10,'prese')
insplot(avp_ins)

gyro=d2r([-Ins_D(9,:);Ins_D(10,:);Ins_D(11,:)]*1e-3/3600);
acc=[Ins_D(12,:);Ins_D(13,:);Ins_D(14,:)]*1e-7; 
time=Ins_D(end,:)-Ins_D(end,1);
imu=[gyro;acc;time]';
ts=0.02;
avp_ins=[];
ins = myins('initial',ts,avp(1,1:9)');
% ins.dvbias=[-15;-34;-15];% 单位为mGAL
% ins.phimbias=-[0.027;0.027;0.027]; % 单位为deg/h
ins.phimbias=zeros(3,1);
ins.dvbias=zeros(3,1);
ll=length(imu);
for i=1:ll
    t=imu(i,end);
    ins = myins('update',ins,imu(i,1:6));
    ins.pos(3)=-depth_intep(i);
    avp_ins(i,:)=[ins.avp',t];
end
myfigurestartup(10,10,'prese')
insplot(avp_ins)
%%
close all
myfigurestartup(10,10,'prese')
insplot(avp)
myfigurestartup(10,10,'prese')
insplot(avp_ins)
%%
imu11=avp2imu(avp,avp(1,7:9)');
inspure(imu11,avp(1,1:9));