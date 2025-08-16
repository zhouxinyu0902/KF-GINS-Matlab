%% 数据检查
glvs
clear
load('2023\matData\LBL.mat')
load('2023\matData\PHINS.mat')
load('2023\matData\USBL.mat')
%% 轨迹对比
myfigurestartup(5,5,'prese')
plot(POS_auv(9,:),POS_auv(8,:),'.',Ins_Nav(12,:),Ins_Nav(11,:),'.',...
    PTSAGpos(10,:),PTSAGpos(9,:),'.')
legend('LBL','PHINS','USBL')
%% IMU数据
gyro=d2r([Ins_D(9,:);Ins_D(10,:);Ins_D(11,:)]*1e-3/3600);
acc=[Ins_D(13,:);Ins_D(12,:);Ins_D(14,:)]*1e-7; 
time=Ins_D(end,:)-Ins_D(end,1);
imu=[gyro;acc;time]';
imuplot(imu)
%% 仿真
close all
avp0 = [[0;0;0]; [0;0;0];[d2r([15.8206,115.147240]),-4128.846]']; 
xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      20); % 保持原来的状态不变
seg = trjsegment(seg, 'accelerate',   5, xxx, 0.3); % 加速
seg = trjsegment(seg, 'uniform',      45);
seg = trjsegment(seg, 'turnleft', 15, 6);
seg = trjsegment(seg, 'uniform',      40);
seg = trjsegment(seg, 'turnleft', 15, 6);
seg = trjsegment(seg, 'uniform',      70);
seg = trjsegment(seg, 'turnleft', 15, 6);
seg = trjsegment(seg, 'uniform',      43);
seg = trjsegment(seg, 'turnleft', 15, 6);
seg = trjsegment(seg, 'uniform',      35);
trj= trjsimu(avp0, seg.wat, 0.02, 1); 
myfigurestartup(7,7,'prese')
insplot(trj.avp)
%% 仿真惯导解算
imuerr = imuerrset(1, 0, 0.01, 3.6); 
trjimu= imuadderr(trj.imu, imuerr);
avp00 = trj.avp(1,1:9)';
ins = myins('initial',0.02,avp00);
depth=[];
depth=trj.avp(:,9)+normrnd(0,0.2,size(trj.avp(:,9)));
for i=1:2:length(trj.avp)-1
    t=trjimu(i+1,end);
    ins = myins('update',ins,trjimu(i:i+1,1:6));
    ins.pos(3)=depth(i);
    avp_ins((i+1)/2,:)=[ins.avp',t];
end
%%
figure
trjsee(trj.avp,'2d',avp_ins)
legend('ref','ins','start')
avpcmpplot(trj.avp,avp_ins);
%%
figure
plot(acc(1,:));
figure
plot(acc(2,:));
v=cumsum(acc(1,:));
dv=diff(DVL(8:10,:)');
vv=cumsum(trjimu(:,4),1);
%%
myfigurestartup(15,5,'prese')
subplot 131
plot(DVL(end,:),DVL(8,:),'.')
subplot 132
plot(DVL(end,:),DVL(9,:),'.')
subplot 133
plot(DVL(end,:),DVL(10,:),'.')

myfigurestartup(15,5,'prese')
subplot 131
plot(Ins_Nav(end,:),Ins_Nav(18,:),'.')
subplot 132
plot(Ins_Nav(end,:),Ins_Nav(17,:),'.')
subplot 133
plot(Ins_Nav(end,:),Ins_Nav(19,:),'.')
