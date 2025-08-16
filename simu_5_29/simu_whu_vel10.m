clear all
%% 仿真轨迹
glvs
avp0 = [[0;0;0]; [0;0;0]; [d2r([15;115]);-1200]];
ts=0.01;
xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      10); % 保持原来的状态不变
seg = trjsegment(seg, 'accelerate',   5, xxx, 2); % 加速
seg = trjsegment(seg, 'uniform',      3585);
trj= trjsimu(avp0, seg.wat, ts, 1);
% 
myfigurestartup(10,10,'prese')
insplot(trj.avp)
sprintf('总的时间:%.2f s',sum(trj.wat(:,1)))
disp(ans)
%% 仿真距离
RCompu(trj.avp(1,7:9),trj.avp(end,7:9))
% 三个静止信标
truthddm(:,2)=trj.avp(:,end);
truthddm(:,3:4)=trj.avp(:,7:8)*180/pi;
truthddm(:,5)=trj.avp(:,9);
close all
pos0=avp0(7:9);
BCN(1,:)=dxyz2pos([5*sqrt(3),10,0]*1000,pos0);
BCN(2,:)=dxyz2pos([-5*sqrt(3),0,20/1000]*1000,pos0);
BCN(3,:)=dxyz2pos([-5*sqrt(3),20,10/1000]*1000,pos0);
% BCN(1,:)=dxyz2pos([5*sqrt(3),0,0]*400,pos0);
% BCN(2,:)=dxyz2pos([-5*sqrt(3),-10,20/200]*400,pos0);
% BCN(3,:)=dxyz2pos([-5*sqrt(3),10,10/200]*100,pos0);
BCNddm=BCN;
BCNddm(:,1:2)=r2d(BCN(:,1:2));% 得到ddm
figure
plot(truthddm(:,4),truthddm(:,3))
hold on
for i=1:3
    plot(BCNddm(i,2),BCNddm(i,1),'*')
end

plot_beacon_distances_with_custom_func(BCNddm,BCN,truthddm(:,3:5))
axis equal
%%
for t=[100,1000,6000,42000] %1\10\60\420s周期
    ll=length(trj.avp)/t; 
    bcn3beacon=repmat(BCNddm, floor(ll/3), 1);

    if mod(ll,3)>=2
        bcn3beacon(floor(ll/3)*3+1:floor(ll/3)*3+2,:)=BCNddm(1:2,:);
    elseif mod(ll,3)>=1&&mod(ll,3)<=1
        bcn3beacon(floor(ll/3)*3+1,:)=BCNddm(1,:);
    end
    truthref=truthddm(t:t:end,:);
    range_3beacon=bcn2range(truthref,bcn3beacon);
    range_3beacon(:,2:3)=range_3beacon(:,2:3)+normrnd(0,5,size(range_3beacon(:,2:3)));
    range_3beacon_output_file=sprintf("simu_5_29\\input\\range_3beacon_%ds.txt",t/100);
    try
        writematrix(range_3beacon, range_3beacon_output_file, 'Delimiter', ' ');
        fprintf('距离信息已成功写入到 %s\n', range_3beacon_output_file);
    catch ME
        error('错误：写入文件失败。错误信息：%s', ME.message);
    end
end
%% 加噪声处理
% 仿真imu数据,增加噪声
eb=0.005; 
db=10; 
web=0.0003; 
wdb=1e-7/3600*1e5; 
rng(1);
imuerr = imuerrset(eb, db, web, wdb, web, 4, wdb ,4, 5 , 10, 5, 10, 10, 10, 10); 
trjimu= imuadderr(trj.imu, imuerr);
% 设置初始对准精度
avperr = avperrset([0.008,0.008,0.06]*60,0.01,1);
avp0 = avpadderr(avp0,avperr);
%% 坐标系转换
pva_ref = avpENU2NED(trj.avp);
pva0 = avpENU2NED([avp0;trj.avp(1,end)]');
IMUFRD = imuRFU2FRD(trjimu);% avp2pva
%% 保存文件
% 参考值
path='simu_5_29/input/truth.nav';
fp=fopen(path,'wt');
fprintf(fp, '%.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f\n', pva_ref');
fclose(fp);

path='simu_5_29/input/imu.nav';
fp=fopen(path,'wt');
fprintf(fp, '%.10f %.10f %.10f %.10f %.10f %.10f %.10f \n', IMUFRD');
fclose(fp);

% 仿真LBL
LBL(:,1:2)=pva_ref(100:100:end,3:4)+normrnd(0,0.2/glv.Re*180/pi,size(pva_ref(100:100:end,3:4)));
LBL(:,3)=-pva_ref(100:100:end,5)+normrnd(0,0.2,size(pva_ref(100:100:end,5)));
LBL(:,4)=pva_ref(100:100:end,2);
path='simu_5_29/input/LBL.nav';
fp=fopen(path,'wt');
fprintf(fp, '%.10f %.10f %.10f %.10f \n', LBL');
fclose(fp);

% 仿真深度计
depth(:,1)=pva_ref(20:20:end,5)+normrnd(0,0.2,size(pva_ref(20:20:end,5)));
depth(:,2)=pva_ref(20:20:end,2);
path='simu_5_29/input/depth-nav.nav';
fp=fopen(path,'wt');
fprintf(fp, '%.10f %.10f\n', depth');
fclose(fp);
% 
%% WHU惯导解算
% 初始化
%
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

% 卡尔曼初始化
kf.RANK = 15;
kf.NOISE_RANK = 6;
kf.P = zeros(kf.RANK, kf.RANK);
kf.Qc = zeros(kf.NOISE_RANK, kf.NOISE_RANK);
kf.x = zeros(kf.RANK, 1);

% Qc,变成6*6的矩阵
kf.Qc(1:3, 1:3) = power(imuerr.web(1), 2) * eye(3, 3);
kf.Qc(4:6, 4:6) = power(imuerr.wdb(1), 2) * eye(3, 3);

% P0
kf.P(1:3, 1:3) = diag(power(avperr(7:9), 2));
kf.P(4:6, 4:6) = diag(power(avperr(4:6), 2));
kf.P(7:9, 7:9) = diag(power(avperr(1:3), 2));
kf.P(10:12, 10:12) = diag(power(imuerr.eb, 2));
kf.P(13:15, 13:15) = diag(power(imuerr.db, 2));

kf.P0 = kf.P;
%
ll=length(IMUFRD);
pva= prealloc(ll-1,11);
pva(1,:)=[0;IMUFRD(1,1);...
navstate.pos(1:2)/pi*180;navstate.pos(3);navstate.vel;navstate.att/pi*180];
tic
for i=2:ll-1
    laststate = navstate;
    lastimu = IMUFRD(i,:)';
    thisimu = IMUFRD(i+1,:)';
    navstate = InsMech(laststate, lastimu, thisimu);
    kf = myInsPropagate_15state(navstate, thisimu, thisimu(1)-lastimu(1), kf);
    pva(i,:)=[0;IMUFRD(i,1);...
    navstate.pos(1:2)/pi*180;navstate.pos(3);navstate.vel;navstate.att/pi*180];
    if mod(IMUFRD(i,1),1)==0

        LBLdata=zeros(3,1);
        err=avperrset(0,0,0.25);
        LBLdata(1:2,1) = trj.avp(i,7:8)'+normrnd(0,err(7),2,1);
        LBLdata(3,1) = -trj.avp(i,9)+normrnd(0,err(9),1,1);
        
        kf = myLBLUpdate(navstate, LBLdata, kf);
        % [kf, navstate] = myErrorFeedback_15state(kf, navstate);
        % kf = myRangeUpdate(navstate, Rangedata, depthdata, kf);
    end
end
toc
%% 定义地址
whupath='simu_5_29/pva_whu_10_insLBL.txt';

% 保存结果和参考结果
writematrix(pva,whupath,'Delimiter',' ');
%%
truthpath='simu_5_29/input/truth.nav';
%% 绘图结果
close all
% plot_result(truthpath)
% plot_result(whupath)
% %%
% plot_cmp(whupath,truthpath)%% 轨迹对比，笛卡尔坐标系
%% 计算误差以及绘图
close all
calc_error(whupath,truthpath)

