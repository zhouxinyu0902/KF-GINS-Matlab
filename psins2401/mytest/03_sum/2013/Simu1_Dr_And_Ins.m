clear
load('data_1\avp.mat')
glvs
%% 航位推算的数据准备（DVL、罗盘）
trj=trj_05; % 采样时间0.5s
avp_ref=trj.avp;
ts=trj.ts;
ll=length(avp_ref);
% DVL 存在安装角偏差（忽略） 刻度系数误差 常值误差
inst = [0;0;0]*glv.min;
kod = 1.05; % 手册标注0.4%±2mm/s
w_dvl = 0.004;% 手册标注4mm/s，测高高度0.5~30
% 罗盘 存在常值和随机误差
const_cmps=d2r(0.5);
web=d2r(0.01);
% 仿真航向角
compass(:,1)=avp_ref(:,1)+const_cmps+normrnd(0,web,size(avp_ref(:,1)));
compass(:,2)=avp_ref(:,2)+const_cmps+normrnd(0,web,size(avp_ref(:,2)));
compass(:,3)=avp_ref(:,3)+const_cmps+normrnd(0,web,size(avp_ref(:,3)));
compass(:,4)=avp_ref(:,end);

% compass_markov = add_markov_noise(avp_ref(:,3), 60, randn_cmps,0.5);
% 仿真DVL
VXY=prealloc(length(avp_ref),4,4);
for i=1:ll
    cnb=a2mat(avp_ref(i,1:3)); 
    vxyz=cnb'*avp_ref(i,4:6)'; % 转置是因为相对角度
    VXY(i,1:3)=vxyz; % 真实的载体坐标系的速度
end
VXY(:,4)=avp_ref(:,end);
vxy(:,1:3)=VXY(:,1:3)*kod+normrnd(0,w_dvl,length(VXY),3);
vxy(:,4)=VXY(:,4);
% 深度计
depthstd=0.4;
depth=avp_ref(:,9)+normrnd(0,depthstd,size(avp_ref(:,9)));
clear  VXY vxyz i cnbr cbbdot
%% 航位推算
dpos=[1;1;0.2];
dr = mydr('init',trj.avp0(7:9),dpos,ts); % 航位推算初始误差设置

% 可观察不同状态初始值对误差估计的影响
% x0=[0.05;d2r(0.5);0.1/glv.Re;0.1/glv.Re];
% dx0=[0.01;d2r(0.1);0.1/glv.Re;0.1/glv.Re];

x0=[0;0;0;0];% 初始值
dx0=[0.03;d2r(1);1/glv.Re;1/glv.Re];% 初始值不确定性

vk=[0,web,0,0];
rk=6;
% 卡尔曼初始化：采样时间+初始值+初始误差+过程噪声+测量噪声
kf = myekf('init',ts,x0,dx0,vk,rk);


avp_dr=prealloc(length(compass),10);
for i=1:length(compass)
    t=compass(i,end);
    dr=mydr('update',dr,depth(i),compass(i,3),vxy(i,1:2));
    % dr=mydr('update',dr,depth(i),compass_markov(i),vxy(i,1:2));
    avp_dr(i,:)=[dr.avp',t];

    kf = myekf('fk',kf, dr);
    kf = myekf('algo',kf,'T');
    xkpk(i,:)=[kf.xk',diag(kf.Pxk)',t];
end
%% 航位推算轨迹+误差绘图 
myfigurestartup(6,3,'prese');
subplot 121,
trjsee(avp_ref,'2d',avp_dr),legend('true trajectory','DR')
% axis equal
subplot 122, % 误差绘图
RadialError=RCompu(avp_ref(:,7:9),avp_dr(:,7:9));
plot(avp_ref(:,end),RadialError)
xlim([avp_ref(1,end) avp_ref(end,end)])
xygo('t/s','Error/m')
% print(gcf, 'New Folder\仿真-航位推算.png', '-dpng','-r600');
% print(gcf, 'C:\Users\23764\OneDrive\文档\LATEX\距离辅助航位推算总报告（2024）\picture\仿真-航位推算.png', '-dpng','-r600');
%% 系统传播状态
dr_err=avpcmp(avp_dr,avp_ref);
myfigurestartup(5,5,'paper')
xk_plot(0.05,d2r(0.5),dr_err,{'propa'},{xkpk(:,[1:4,end])},0)
%% 航位推算相关数据保存
save data_1\data_dr vxy compass trj dr_err RadialError avp_dr avp_ref web depthstd depth
%% 惯导误差设置 
trj=trj_005; % 惯导采样时间
imu_ref = trj.imu;
imuerr = imuerrset(0.001, 1, 0.01, 10); 
imu= imuadderr(imu_ref, imuerr);

avp_ins_ref=trj.avp;
avp0=trj.avp0;

ll=length(avp_ins_ref);
ts=trj.ts;

imuplot(imu)
% 惯导解算
davp = avperrset([0.5;0.5;5], 0.01, [0.1;0.1;0.1]);
avp00 = avpadderr(avp0,davp);

ins = myins('initial',ts,avp00);
% ins1 = insinit(avp00,ts);
depth=[];
depth=avp_ins_ref(:,9)+normrnd(0,0.2,size(avp_ins_ref(:,9)));
for i=1:2:ll-1
    t=imu(i+1,end);
    ins = myins('update',ins,imu(i:i+1,1:6));
    ins.pos(3)=depth(i);
    avp_ins((i+1)/2,:)=[ins.avp',t];

    % %与工具箱内的没差
    % ins1 = insupdate(ins1,imu(i:i+1,1:6));
    % ins1.pos(3)=depth(i);
    % avp_ins1((i+1)/2,:)=[ins1.avp',t];
end
%% 轨迹绘图
close all
myfigurestartup(5,5,'prese');
% trjsee(avp_ins_ref,'2d',avp_ins,avp_ins1),legend('true trajectory','INS','PSINS')
trjsee(avp_ins_ref,'2d',avp_ins)
% 误差绘图
RadialError=RCompu(avp_ins_ref(2:2:ll,7:9),avp_ins(:,7:9));
figure,plot(avp_ins_ref(2:2:ll,end),RadialError)
xygo('t/s','Error/m')
RadialError(7200)/1852 % 换算为海里，2h漂移


