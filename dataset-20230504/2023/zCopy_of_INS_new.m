clear
load('sum\2023\matData\PHINS-new.mat')
% load('2023\matData\imualignment.mat') %导出来的参考log文件里面
load('DELPH data\att_vel.mat') % 导出的组合结果的姿态
% glvs
% figure,plot(Ins_Nav(3,:)) % 越来越往上,向上的速度是正的
attitude_ref1=attitude_ref(:,101:2:end);
speed_n_ref1=speed_n_ref(:,101:2:end);
speed_b_ref1=speed_n_ref(:,101:2:end);
Ins_Nav=Ins_Nav(:,2:end);
Ins_D=Ins_D(:,46:end-49);
% 对准时间
format bank;
string={'Ins_Nav','Ins_D','attitude_ref1'};
t(1)=Ins_Nav(end,1); % 采样时间1s
t(2)=Ins_D(end,1); % 采样时间0.02s 
t(3)=attitude_ref1(end,1); % 采样时间0.02s 
disp(string)
disp(t)
t(1)=Ins_Nav(end,end); 
t(2)=Ins_D(end,end);
t(3)=attitude_ref1(end,end);
disp(t)
t(1)=Ins_Nav(end,end)-Ins_Nav(end,1);
t(2)=Ins_D(end,end)-Ins_D(end,1);
t(3)=attitude_ref1(end,end)-attitude_ref1(end,1);
disp(t)
% 初始时间按惯导数据算1023.67~4984.67；一共3961s
t_Nav=0:3961;
t_ref=0:0.02:3961-0.02;
%% PHINS参考系为前左上,工具箱为右前上
glvs
attitude=[d2r([Ins_Nav(6,:);Ins_Nav(5,:)]);yawcvt(mod(d2r(Ins_Nav(4,:))'-90,360))'];
attitude_ref=[d2r([-attitude_ref1(3,:);attitude_ref1(2,:)]);yawcvt(d2r(attitude_ref1(1,:))')'];

speed_n=[Ins_Nav(8,:);Ins_Nav(7,:);Ins_Nav(9,:)]; % Ins_Nav，NEU转为ENU
% speed_n_ref=[speed_n_ref1(2,:);speed_n_ref1(1,:);speed_n_ref1(3,:)];
% speed_b_ref=[speed_b_ref1(2,:);speed_b_ref1(1,:);speed_b_ref1(3,:)];
% ins-nav速度ENU，转换到载体坐标系中
for i=1:length(attitude)
    Cnb=a2mat(attitude(:,i)); % Cnb,Cnb'=Cbn
    speed_b(:,i)=Cnb'*speed_n(:,i); % 参考的载体坐标系的速度
end

tt=Ins_Nav(end,:)-Ins_Nav(end,1);
position=[d2r([Ins_Nav(1,:);Ins_Nav(2,:)]);Ins_Nav(3,:);tt];
avp=[attitude',speed_n',position',tt'];

myfigurestartup(12,8,'prese')
string={'pitch','roll','heading','east','north','up'};
for i=1:3
subplot(3,3,i)
plot(Ins_Nav(end,:),r2d(attitude(i,:)))
title(string{i})
xygo('t/s','angel/deg')
% hold on
% plot(attitude_ref1(end,:),r2d(attitude_ref(i,:)))
grid on
subplot(3,3,i+3)
plot(Ins_Nav(end,:),speed_n(i,:))
% hold on
% plot(attitude_ref1(end,:),speed_n_ref(i,:))
grid on
title(string{i+3})
xygo('t/s','velocity/(m/s)')
end
dxyz=pos2dxyz(position',position(1:3,1));
subplot(3,3,[7,8,9]),plot(dxyz(:,1),dxyz(:,2)),xygo('east/m','north/m')
%%
load('psins2401\mytest\sum\2023\matData\USBL.mat')
load('psins2401\mytest\sum\2023\matData\LBL.mat')
figure

plot(dxyz(:,1),dxyz(:,2)),xygo('east/m','north/m')
%% DVL速度查看
% misaligment=d2r([2.171;0;-45.317]); % 对准角
% scale=0.09721;

misaligment=d2r([0.891;0;44.654]); % 校准后的对准角
scale=-0.01197;

Cbd=a2mat(misaligment); % 安装测量系与载体坐标系的旋转矩阵

speed_dvl=[DVL(1,:);DVL(2,:);DVL(3,:);]/(1+scale); 
for i=1:length(speed_dvl)
    % 将DVL测得安装坐标系的速度转换到载体坐标系中 
    speed_dvl_b(:,i)=Cbd*speed_dvl(:,i);      
end

myfigurestartup(12,4,'prese')
string={'right','front','up'};
for i=1:3
    subplot(1,3,i)
    plot(Ins_Nav(end,:),speed_b(i,:))
    hold on
    plot(DVL(end,:),speed_dvl_b(i,:))
    legend('Ins-Nav','DVL')
    grid on
    title(string{i})
end
%% IMU的数据检查，与参考结果的增量比较，姿态在{b}系，速度在{n}系
% ACC bias在200ug内，FOG bias在50m°/h内
% 陀螺仪的数据检查
% 首先计算参考结果的的增量
attitude_diff=diff(attitude,1,2);
attitude0=attitude(:,1);
% 然后计算IMU的增量和IMU积分值
% gyro=d2r([Ins_D(1,:);Ins_D(2,:);Ins_D(3,:)]*1e-3/3600);
gyro=d2r([-Ins_D(1,:);Ins_D(2,:);Ins_D(3,:)]*1e-3/3600);
% gyro_reverse=diff(attitude_ref,1,2);

attitude_sum=attitude0+cumsum(gyro,2);
myfigurestartup(12,8,'prese')
string={'pitch','roll','heading','dpitch','droll','dheading'};
for i=1:3
    % 角度
    subplot(2,3,i)
    plot(Ins_Nav(end,:),r2d(attitude(i,:)))
    % plot(attitude_ref1(end,:),r2d(attitude_ref(i,:)))
    hold on
    plot(Ins_D(end,:),r2d(attitude_sum(i,:)))
    legend('Ins-Nav','IMU')
    xygo('t/s','angel/deg')
    title(string{i})
    % 角度增量
    subplot(2,3,i+3)
    plot(Ins_D(end,:),r2d(gyro(i,:)))
    hold on
    plot(Ins_Nav(end,2:end),r2d(attitude_diff(i,:)/50),'.')
    % hold on
    % plot(Ins_D(end,2:end),r2d(gyro_reverse(i,:)))
    legend('IMU','Ins-Nav')
    xygo('t/s','angel/deg')
    title(string{i+3})
end
%% 加速度计的速度检查 
acc=[-Ins_D(4,:);Ins_D(5,:);Ins_D(6,:)]*1e-7; % 加速度计

% {b}坐标系中的速度增量，速度来自实时导航结果转到{b}中
speed_b_diff=diff(speed_b,1,2);
speed_b_0=speed_b(:,1);
speed_b_sum=speed_b_0+cumsum(acc,2);
% {n}坐标系中的速度增量，速度来自实时导航结果
speed_n_diff=diff(speed_n,1,2);
speed_n_0=speed_n(:,1);
% {n}坐标系中的速度增量，速度来自软件组合导航结果的
speed_n_diff_Ref=diff(speed_n_ref,1,2);

for i=1:length(attitude_ref)
    dveln(:,i)=a2mat(d2r([0,0,0]))*a2mat(attitude_ref(:,i))*acc(:,i); 
    % dveln(:,i)=a2mat(d2r([5,0,0]))*a2mat(attitude_ref(:,i))*acc(:,i); 
    % 加速度计的值由{b}转向{n}
end
% 每隔50个值加起来，计算1s的速度增量
for i=1:3
    bb=reshape(dveln(i,1:end-1),50,3961);
    dveln_1s(i,:)=sum(bb,1);
end
speed_n_sum=speed_n_0+cumsum(dveln,2); % 叠加ENU的初始值

myfigurestartup(12,8,'prese')
string={'v_{east}','v_{north}','v_{up}','dv_E','dv_N','dv_U'};
for i=1:3
    % 速度
    subplot(2,3,i)
    plot(Ins_Nav(end,:),speed_n(i,:))
    hold on
    plot(Ins_D(end,:),speed_n_sum(i,:))
    legend('Ins-Nav','IMU')
    xygo('t/s','Vel/(m/s)')
    title(string{i})
    % 速度增量
    subplot(2,3,i+3)
    % plot(Ins_D(end,:),dveln(i,:))
    plot(Ins_D(end,2:50:end),dveln_1s(i,:))
    hold on
    plot(Ins_Nav(end,2:end),speed_n_diff(i,:),'.')
    % plot(Ins_D(end,2:end),speed_n_diff_Ref(i,:),'.')
    legend('IMU','Ins-Nav')
    xygo('t/s','Vel/(m/s)')
    title(string{i+3})
end
%% 惯导解算
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
%%
myfigurestartup(10,10,'prese')
insplot(avp_ins)
%%
ts=0.02;
ins = myins('initial',ts,avp(1,1:9)');
ll=length(imu);
for i=1:ll
    t=imu(i,end);
    ins = myins('update',ins,imu(i,1:6));
    ins.pos(3)=-depth_intep(i);
    avp_ins(i,:)=[ins.avp',t];

end
%%
close all
myfigurestartup(10,10,'prese')
insplot(avp)
myfigurestartup(10,10,'prese')
insplot(avp_ins)
%%
imu11=avp2imu(avp,avp(1,7:9)');
inspure(imu11,avp(1,1:9));