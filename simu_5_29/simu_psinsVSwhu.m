clear all
%% 仿真轨迹
glvs
avp0 = [[0;0;0]; [0;0;0]; [d2r([15;115]);-1200]];
ts=0.01;
xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      20); % 保持原来的状态不变
seg = trjsegment(seg, 'accelerate',   5, xxx, 2); % 加速
seg = trjsegment(seg, 'uniform',      1200);
trj= trjsimu(avp0, seg.wat, ts, 1); 
%%
myfigurestartup(10,10,'prese')
insplot(trj.avp)
sum(trj.wat(:,1))
%% 仿真imu数据,增加噪声
eb=0.005; % cfg.gyrbiasstd = 0.027; % [deg/h]
db=10; % cfg.accbiasstd = 15; % [mGal]
web=0.0003; % cfg.gyrarw = 0.003; % [deg/sqrt(h)]
wdb=1e-7; % cfg.accvrw = 0.03; % [m/s/sqrt(h)]

% eb=2;db=0.36*980;web=0.15;wdb=200/9.8;
% eb=0.1;db=0.1;web=0.05;wdb=10;
rng(1);
imuerr = imuerrset(eb, db, web, wdb, web, 4, wdb ,4, 5 , 10, 5, 10, 10, 10, 10); 
trjimu= imuadderr(trj.imu, imuerr);
%% 坐标系转换
pva_ref = avpENU2NED(trj.avp);
pva0 = pva_ref(1,:);
IMUFRD = imuRFU2FRD(trjimu);
pos0 = avp0(7:9);
xyz_true = pos2dxyz(trj.avp(:,7:9),pos0);
truthpath='simu_5_29/pva_ref.txt';
writematrix(pva_ref,truthpath,'Delimiter',' ');
%% PSINS内
ll=length(trjimu);
[avp_psins, xyz_psins]= prealloc(ll-1,10,3);

ins = myins('initial',ts,avp0);
avp_psins(1,:) = [ins.avp',trjimu(1,end)];
xyz_psins(1,:) = pos2dxyz(ins.pos',pos0);
tic
for i=2:ll-1
    t = trjimu(i,end);
    ins = myins('update',ins,trjimu(i,1:6));
    avp_psins(i,:)=[ins.avp',t];
    xyz_psins(i,:) = pos2dxyz(ins.pos',pos0);
end
toc
%% WHU惯导解算
% 初始化
 
navstate.pos = avp0(7:9);
navstate.vel = pva0(6:8)';
navstate.att = pva0(9:11)'/180*pi;
navstate.qbn = euler2quat(navstate.att);
navstate.cbn = euler2dcm(navstate.att);
navstate.gyrbias = zeros(3,1);
navstate.accbias = zeros(3,1);
navstate.gyrscale = zeros(3,1);
navstate.accscale = zeros(3,1);
param = Param();
[navstate.Rm, navstate.Rn] = getRmRn(navstate.pos(1), param);
navstate.gravity = getGravity(navstate.pos);

ll=length(IMUFRD);
[pva, xyz_whu]= prealloc(ll-1,11,3);
pva(1,:)=[0;IMUFRD(1,1);...
navstate.pos(1:2)/pi*180;navstate.pos(3);navstate.vel;navstate.att/pi*180];    
xyz_whu(1,:) = pos2dxyz(navstate.pos',pos0);
tic
for i=2:ll-1
    laststate = navstate;
    lastimu = IMUFRD(i,:)';
    thisimu = IMUFRD(i+1,:)';
    navstate = InsMech(laststate, lastimu, thisimu);
    pva(i,:)=[0;IMUFRD(i,1);...
    navstate.pos(1:2)/pi*180;navstate.pos(3);navstate.vel;navstate.att/pi*180];    
    xyz_whu(i,:) = pos2dxyz(navstate.pos',pos0);
end
toc
%% 轨迹对比，笛卡尔坐标系
figure,
plot(xyz_whu(:,1),xyz_whu(:,2));hold on;
plot(xyz_psins(:,1),xyz_psins(:,2));hold on;
plot(xyz_true(:,1),xyz_true(:,2))
%% 定义地址
whupath='simu_5_29/pva_whu.txt';
psinspath='simu_5_29/pva_psins.txt';
% 保存结果和参考结果
writematrix(pva,whupath,'Delimiter',' ');
pva_psins = avpENU2NED(avp_psins);
writematrix(pva_psins,psinspath,'Delimiter',' ');
%% 绘图结果
close all
plot_result(truthpath)
plot_result(whupath)
plot_cmp(whupath,truthpath)
%% 计算误差以及绘图
calc_error(whupath,truthpath)
calc_error(psinspath,truthpath)
