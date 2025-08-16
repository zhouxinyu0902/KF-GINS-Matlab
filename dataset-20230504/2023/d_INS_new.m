% 惯导解算（使用导出的数据进行）
clear
load('matData\PHINS-0017.mat')
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
%% ins-nav的姿态速度绘图
glvs
attitude=[d2r([-Ins_Nav(6,:);Ins_Nav(5,:)]);yawcvt(d2r(Ins_Nav(4,:))')'];
speed_n=[Ins_Nav(8,:);Ins_Nav(7,:);Ins_Nav(9,:)]; % Ins_Nav，NEU转为ENU
tt=Ins_Nav(end,:)-Ins_Nav(end,1);
position=[d2r([Ins_Nav(1,:);Ins_Nav(2,:)]);Ins_Nav(3,:);tt];
avp=[attitude',speed_n',position'];

myfigurestartup(12,8,'prese')
string={'pitch','roll','heading','east','north','up'};
for i=1:3
subplot(3,3,i)
plot(Ins_Nav(end,:),r2d(attitude(i,:)))
title(string{i})
xygo('t/s','angel/deg')
grid on
subplot(3,3,i+3)
plot(Ins_Nav(end,:),speed_n(i,:))
grid on
title(string{i+3})
xygo('t/s','velocity/(m/s)')
end
dxyz=pos2dxyz(position',position(1:3,1));
subplot(3,3,[7,8,9]),plot(dxyz(:,1),dxyz(:,2)),xygo('east/m','north/m'),axis equal
% subplot(3,3,[7,8,9]),plot(r2d(position(2,:)),r2d(position(1,:))),xygo('lat','lon')
%% DVL对准与ins-nav对比
% close all
% PHINS:pitch头朝下为正，heading顺时针为正
% 工具箱:pitch头朝上为正，heading北偏西为正

misaligment=d2r([-2.171;0;45.317]);% 安装角 
% misaligment=d2r([-2;5;45]);% 安装角  
Cbd=a2mat(misaligment); % 安装测量系{d}与载体坐标系{b}的旋转矩阵
Cbb=a2mat(d2r([0;0;0]));% 安装误差矩阵
scale=0.09721;
% scale=0.05;

% DVL测的速度右手坐标系右前上，按照右前上排序
% speed_dvl=[-DVL(2,:);DVL(1,:);DVL(3,:);]/(1-scale); % DVL测的速度前左上，按照右前上排序
speed_dvl=[DVL(1,:);DVL(2,:);DVL(3,:);]/(1-scale); 

% 是转换到载体坐标系进行对比，因为DVL测得的速度在载体坐标系下，由于没有合适的姿态能将其转换到导航坐标系
for i=1:length(speed_dvl)
    speed_dvl_b(:,i)=Cbb*Cbd*speed_dvl(:,i); % 将DVL测得安装坐标系的速度转换到载体坐标系中     
end

for i=1:length(attitude)
    Cnb=a2mat(attitude(:,i)); % Cnb,Cnb'=Cbn
    Cbb_1=a2mat(d2r([0;0;0]));
    speed_b(:,i)=Cbb_1*Cnb'*speed_n(:,i); % ins-nav速度ENU，转换到载体坐标系中，载体坐标系的速度
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
%% 惯导解算
% 加速度计偏差在200ug，陀螺仪在50m°/h=0.05°/h
% close all
time=Ins_D(end,:)-Ins_D(end,1);
% 选择三种对齐方式
gyro=d2r([Ins_D(1,:);Ins_D(2,:);Ins_D(3,:)]*1e-3/3600);
acc=[Ins_D(4,:);Ins_D(5,:);Ins_D(6,:)]*1e-7;

% gyro=d2r([-Ins_D(1,:);Ins_D(2,:);Ins_D(3,:)]*1e-3/3600);
% acc=[-Ins_D(4,:);Ins_D(5,:);Ins_D(6,:)]*1e-7;

% gyro=d2r([-Ins_D(2,:);Ins_D(1,:);Ins_D(3,:)]*1e-3/3600);
% acc=[-Ins_D(5,:);Ins_D(4,:);Ins_D(6,:)]*1e-7;
imu=[gyro;acc;time]';
% 抬头矫正
% gyro_bais=gyro;
% acc_bais=acc;
% for i=1:length(gyro)
%     gyro_bais(:,i)=a2mat(d2r([-5,0,0]))*gyro(:,i);
%     acc_bais(:,i)=a2mat(d2r([-5,0,0]))*acc(:,i);
% end
% imu=[gyro_bais;acc_bais;time]';
ts=0.02;
avp_ins=[];
ins = myins('initial',ts,avp(1,1:9)');
depth_intep=interp1(depth(end,:),depth(1,:),Ins_D(end,:),'linear');
ll=length(imu);
avp_ins(1,:)=[ins.avp',imu(1,end)];
for i=2:ll
    t=imu(i,end);
    ins = myins('update',ins,imu(i,1:6));
    ins.pos(3)=-depth_intep(i);
    avp_ins(i,:)=[ins.avp',t];
end
%% 绘图
myfigurestartup(10,10,'prese')
insplot(avp_ins)
figure
insplot(avp)
%% 
dphi=r2d(avp_ins(1:50:end,3)-avp(:,3));
figure
plot(avp(:,end),dphi)
%% 舒勒震荡 84min
figure
plot(avp_ins(:,end),avp_ins(:,4))
hold on
plot(avp_ins(:,end),avp_ins(:,5))
legend('E','N')
%% 计算CEP误差
% 输入：East_error, North_error（向量）
err=avpcmp(avp_ins,avp);
sigma_x = std(err(:,7)*glv.Re); 
sigma_y = std(err(:,8)*glv.Re);
if (sigma_x == sigma_y) && (0 < corr(East_error, North_error) <0.1)
    CEP = 1.1774 * sigma_x; 
else
    CEP = 0.5887 * (sigma_x + sigma_y);
end
output=sprintf("CEP误差：%.2f m",CEP);
disp(output)
CEP/1852
%% 计算径向误差
close all
myfigurestartup(5,5,'prese')
% trjsee(avp_ins_ref,'2d',avp_ins,avp_ins1),legend('true trajectory','INS','PSINS')
trjsee(avp,'2d',avp_ins)
% 误差绘图
[RadialError,dll]=RCompu(avp(:,7:9),avp_ins(1:50:end,7:9));
figure,plot(avp(:,end),RadialError)
xygo('t/s','Error/m')
RadialError(end)/1852 % 换算为海里，漂移

output=sprintf('-----以下总时长为%.2f秒≈%.2f h-----',avp(end,end),avp(end,end)/3600);
disp(output)
output1=sprintf("最大径向误差：\n  %.2f m,%.2f 海里",max(RadialError),max(RadialError)/1852);
disp(output1)

% 计算CEP误差
sigma_x = std(dll(:,2)); 
sigma_y = std(dll(:,1));
if (sigma_x == sigma_y) && (abs(corr(dll(:,2), dll(:,1))) <0.1)
    CEP = 1.1774 * sigma_x; 
else
    CEP = 0.5887 * (sigma_x + sigma_y);
    CEP = 1.1774*sqrt(sigma_x^2 + sigma_y^2);
end
output2=sprintf("正态分布CEP误差：\n  %.2f m,%.2f 海里",CEP,CEP/1852);
disp(output2)
output3=sprintf("非正态分布CEP误差（取中位数）：\n  %.2f m,%.2f 海里",median(RadialError),median(RadialError)/1852);
disp(output3)
