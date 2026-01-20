clear all
% 模拟直线轨迹与三个潜标的位置
%% 定信标位置和轨迹初始点
glvs
% 定义参考原点/第一个信标的绝对位置，单位为度 [纬度, 经度, 高度]
% beacon0=d2r([17,117,0]);
beacon0=d2r([17.574,117.7900,0]);
% 117.791, 17.576
% 根据距离设置定义其余两个信标和轨迹原点，dxyz和纬度经度rrm
dxyz_original=[0,0,0;
    10,10*sqrt(3),0;
    20,0,0;
    0,5*sqrt(3),0;
    7,10,0]*1000/4;

% 定义旋转角度（15度）
theta_deg = 0;
theta_rad = deg2rad(theta_deg);
% 创建绕Z轴的旋转矩阵
R = [cos(theta_rad), -sin(theta_rad), 0;
     sin(theta_rad), cos(theta_rad),  0;
     0,              0,              1];
% 对每个点应用旋转
dxyz_rotated = (R * dxyz_original')'; % 转置以便矩阵乘法
% 使用旋转后的坐标
dxyz = dxyz_rotated;

% 分开获取信标和轨迹原点
rrm=dxyz2pos(dxyz,beacon0');
ddm=r2d(rrm(:,1:2));
beaconxyz=dxyz(1:3,:);
beaconrrm=rrm(1:3,:);
beaconddm=ddm(1:3,:);
id=5;
pos0xyz=dxyz(id,:);
pos0rrm=rrm(id,:);
pos0ddm=ddm(id,:);
% 绘图
plot_beacon_data(beaconddm, beaconxyz)

%% 生成轨迹
ts = 0.01;  
avp0 = [[0;0;d2r(-95)]; [0;0;0]; pos0rrm']; 
xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      20);
seg = trjsegment(seg, 'accelerate',   10, xxx, 0.20576); 
seg = trjsegment(seg, 'uniform',      1000); 
seg = trjsegment(seg, 'turnright', 15, 170/15);
seg = trjsegment(seg, 'uniform',      1000);
seg = trjsegment(seg, 'turnleft', 15, 170/15);
seg = trjsegment(seg, 'uniform',      1000);
seg = trjsegment(seg, 'turnright', 15, 170/15);
seg = trjsegment(seg, 'uniform',      1000);
seg = trjsegment(seg, 'turnleft', 15, 170/15);
seg = trjsegment(seg, 'uniform',      1000);
seg = trjsegment(seg, 'turnright', 15, 170/15);
seg = trjsegment(seg, 'uniform',      1000);
seg = trjsegment(seg, 'turnleft', 15, 170/15);
seg = trjsegment(seg, 'uniform',      1000);
seg = trjsegment(seg, 'turnright', 15, 170/15);
seg = trjsegment(seg, 'uniform',      1000);
seg = trjsegment(seg, 'turnleft', 15, 170/15);
seg = trjsegment(seg, 'uniform',      1000);
seg = trjsegment(seg, 'deaccelerate',   10, xxx, 0.20576); 


% seg = trjsegment(xxx, 'init',         0);
% seg = trjsegment(seg, 'uniform',      20);
% seg = trjsegment(seg, 'accelerate',   10, xxx, 0.20576); 
% seg = trjsegment(seg, 'uniform',      3600*1.35); 
% seg = trjsegment(seg, 'turnleft',  60, 3);
% seg = trjsegment(seg, 'uniform',      3600*1.35); 
% seg = trjsegment(seg, 'deaccelerate',   10, xxx, 0.20576); 

trj = trjsimu(avp0, seg.wat, ts, 1); % 只需要位置和姿态信息就可以
[nn, ts, nts] = nnts(1, trj.ts);
sum(trj.wat(:,1))
% 轨迹与参考点绘图
% 除以 1000 将米转换为公里。
trajectory_xyz_km = pos2dxyz(trj.avp(:,7:9), beacon0') / 1000;
trajectory_ddm=trj.avp(:,7:9);
trajectory_ddm(:,1:2)=r2d(trajectory_ddm(:,1:2));
%% 绘图
% plot_trajectory_and_beacons(trajectory_xyz_km, beaconxyz, beaconddm, trajectory_ddm)
plot_trajectory_and_beacons_m(trajectory_xyz_km*1000, beaconxyz)
%% 计算信标与轨迹中每一个点的距离并绘图
beacon_xyz_km = beaconxyz/1000;
% 获取轨迹点的坐标
trajectory_x = trajectory_xyz_km(:, 1);
trajectory_y = trajectory_xyz_km(:, 2);
trajectory_z = trajectory_xyz_km(:, 3); % 如果需要3D距离
for i=1:3
    % 获取信标的坐标 (东向，北向，天向)
    beacon1_x = beacon_xyz_km(i, 1);
    beacon1_y = beacon_xyz_km(i, 2);
    beacon1_z = beacon_xyz_km(i, 3); % 如果需要3D距离

    % 计算每个轨迹点到第一个信标的距离
    % 这里计算的是3D欧氏距离，如果只需要2D水平距离，请注释掉z分量
    distances_km(:,i) = sqrt((trajectory_x - beacon1_x).^2 + ...
        (trajectory_y - beacon1_y).^2 + ...
        (trajectory_z - beacon1_z).^2);
    
    % 创建新的图窗来绘制距离曲线
    figure;
    plot(trj.avp(:,10), distances_km(:,i), 'LineWidth', 1.5); % 洋红色实线
    xlabel('时间 (s) ');
    ylabel('距离 (km)');
    title('轨迹点到信标的距离');
    grid on;
end
distances_N_by_1_max = max(distances_km, [], 2);


%% 初始化
avp_ref = trj.avp;
avp0_ref = avp_ref(1,1:9);
avp0_err = avperrset([0.01,0.01,0.05]*60,0.1,1);
avp0_use = avpadderr(avp0_ref,avp0_err);

imu_ref = trj.imu;
% imu_err = imuerrset(0.002, 0.1, 0.001, 10);
% imu_err = imuerrset(0.005, 0.1, 0.001, 10);
% imu_err = imuerrset(0.005, 10, 0.0003, 0.01);
% 设置IMU误差
eb=0.003;
db=7;
web=0.0003;
wdb=2.7778e-05;%1e-6m/s/sqrt(Hz)
%1e-6*1e5/3600 = 2.7778e-05

eb=0.027;
db=15;
web=0.003;
wdb= 0.03*1e5/3600;% m/s/sqrt(Hz)转为ug
rng(1);
imu_err = imuerrset(eb, db, web, wdb); 

imu_use = imuadderr(imu_ref, imu_err);
%%
% %% 惯导解算
% tic;
% lastprecent=0;
% ins = myins('initial',0.01,avp0_use); % 初始姿态很重要
% % ins = myins('initial',0.01,avp0_ref');
% ll = length(avp_ref);
% avp_pureins=prealloc(ll,10);
% % l_used=3600/0.01;
% % avp_pureins=prealloc(l_used,10);
% l_used=ll;
% for i=1:l_used
%     t = imu_use(i,end);
%     ins = myins('update',ins,imu_use(i,1:6));
%     ins.pos(3)=0;
%     avp_pureins(i,:)=[ins.avp',t];
%     if (i/l_used- lastprecent> 0.01)
%         disp("processing " + num2str(floor(i * 100 / l_used)) + " %!");
%         lastprecent = i / l_used ;
%     end
% end
% toc
% %% 绘图
% avperr1 = avpcmpplot(avp_ref, avp_pureins);
% plotTrajectoriesComparison(avp_ref, avp_pureins, 0.01)
%% 惯导+距离
% tic;
% lastprecent=0;
% ins = myins('initial',0.01,avp0_use); % 初始姿态很重要
% ins_pure = myins('initial',0.01,avp0_use); % 初始姿态很重要
% % ins = myins('initial',0.01,avp0_ref');
% 
% % ll = length(avp_ref);
% % avp_pureins=prealloc(ll,10);
% % l_used=ll;
% 
% 
% % KF初始化
% % 初始状态和误差设置
% x0=[avp0_err*0;imu_err.eb*0;imu_err.db*0];
% dx0=[avp0_err*1.5;imu_err.eb*1.1;imu_err.db*1.1];
% 
% % x0=[davp0;imuerr.eb;imuerr.db]*0;
% % dx0=[davp0;imuerr.eb;imuerr.db]; % 非常理想的状态
% 
% % 过程噪声+量测噪声
% vk = [imu_err.web;imu_err.wdb;zeros(9,1)];
% rk = 15; % ins/range
% 
% % KF初始化
% kf = myekf1('init',ts,x0,dx0,vk,rk);
% kf.Pmin = [avperrset(0.01,1e-4,0.1); gabias(1e-3, [1,10])].^2;
% kf.pconstrain=1;
% 
% ki=1;
% l_used=3600/0.01;
% [avp_pureins,avp_ins_full,avp_pureins_full,xkpk]=prealloc(l_used,10,10,10,31);
% 
% for i=1:l_used
%     t = imu_use(i,end);
%     ins = myins('update',ins,imu_use(i,1:6));
%     ins_pure = myins('update',ins_pure,imu_use(i,1:6));
% 
%     kf = myekf1('fk',kf, ins);
%     kf = myekf1('algo',kf,'T');
%     if mod(t,40)==0
%         ins.bcn=beaconrrm(1,:);
%         ins.Slantr_ins = RCompu(ins.pos',ins.bcn);
%         ins.range_m=distances_km(i,1)*1000+randn*rk;
%         kf.yk=ins.range_m-ins.Slantr_ins;
%         kf = myekf1('hk',kf, ins,'Slantrange');
%         kf = myekf1('algo',kf, 'M');
%         xkpk(ki,:)=[kf.xk',diag(kf.Pxk)',t];
%         [kf, ins] = kffeedback(kf, ins, 1, 'avp');
%         avp_pureins(ki,:)=[ins.avp',t];
%         ki=ki+1;
%     end
%     ins.pos(3)=0;
%     ins_pure.pos(3)=0;
% 
%     avp_ins_full(i,:)=[ins.avp',t];
%     avp_pureins_full(i,:)=[ins_pure.avp',t];
% 
%     if (i/l_used- lastprecent> 0.01)
%         disp("processing " + num2str(floor(i * 100 / l_used)) + " %!");
%         lastprecent = i / l_used ;
%     end
% end
% toc
%%
tic;
lastprecent=0;
ins = myins('initial',0.01,avp0_use); % 初始姿态很重要
ins_pure = myins('initial',0.01,avp0_use); % 初始姿态很重要
% ins = myins('initial',0.01,avp0_ref');

% ll = length(avp_ref);
% avp_pureins=prealloc(ll,10);
% l_used=ll;


% KF初始化
% 初始状态和误差设置
% x0=[avp0_err*0.5;imu_err.eb*0.5;imu_err.db*0.5];
% dx0=[avp0_err*1;imu_err.eb*1;imu_err.db*1];

x0=[avp0_err;imu_err.eb;imu_err.db]*0;
dx0=[avp0_err;imu_err.eb;imu_err.db]; % 非常理想的状态

% 过程噪声+量测噪声
vk = [imu_err.web;imu_err.wdb;zeros(9,1)];
% rk = 10; % ins/range
rk = [5,0.4]; % ins/range

%%
%% 保存数据
IMUFRD=imuRFU2FRD(imu_use);
path='专利实验/input/line-imu.nav';
fp=fopen(path,'wt');
fprintf(fp, '%.10f %.10f %.10f %.10f %.10f %.10f %.10f \n', IMUFRD');
fclose(fp);

pva_ref=avpENU2NED(avp_ref);
path='专利实验/input/line-truth.nav';
fp=fopen(path,'wt');
fprintf(fp, '%.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f\n', pva_ref');
fclose(fp);


bcn=repmat(beaconrrm(1,:),size(distances_km(:,1)));
range_m=distances_km(:,1)*1000;
range_save1=[avp_ref(:,end),range_m,range_m,bcn];

path='专利实验/input/line-range1.nav';
fp=fopen(path,'wt');
fprintf(fp, '%.10f %.10f %.10f %.10f %.10f %.10f\n', range_save1');
fclose(fp);

bcn=repmat(beaconrrm(2,:),size(distances_km(:,2)));
range_m=distances_km(:,2)*1000;
range_save2=[avp_ref(:,end),range_m,range_m,bcn];

path='专利实验/input/line-range2.nav';
fp=fopen(path,'wt');
fprintf(fp, '%.10f %.10f %.10f %.10f %.10f %.10f\n', range_save2');
fclose(fp);

bcn=repmat(beaconrrm(3,:),size(distances_km(:,3)));
range_m=distances_km(:,3)*1000;
range_save3=[avp_ref(:,end),range_m,range_m,bcn];
path='专利实验/input/line-range3.nav';
fp=fopen(path,'wt');
fprintf(fp, '%.10f %.10f %.10f %.10f %.10f %.10f\n', range_save3');
fclose(fp);
%% KF初始化
kf = myekf1('init',ts,x0,dx0,vk,rk);
kf.Pmin = [avperrset(0.01,1e-4,0.1); gabias(1e-3, [1,10])].^2;
kf.pconstrain=1;

ki=1;
% l_used=7200/0.01;
l_used=length(avp_ref);
[avp_pureins,avp_ins_full,avp_pureins_full,xkpk]=prealloc(l_used,10,10,10,31);

for i=1:l_used
    t = imu_use(i,end);
    ins = myins('update',ins,imu_use(i,1:6));
    ins_pure = myins('update',ins_pure,imu_use(i,1:6));

    kf = myekf1('fk',kf, ins);
    kf = myekf1('algo',kf,'T');
    if mod(t,420)==0
        nn=mod(t/420-1,3)+1;
        ins.bcn=beaconrrm(nn,:);
        ins.Slantr_ins = RCompu(ins.pos',ins.bcn);
        ins.range_m=distances_km(i,nn)*1000+randn*rk(1);
        range_save(ki,:)=[t,ins.range_m,ins.range_m,ins.bcn];
        height(ki,:)=[t,0];
        kf.yk=[ins.range_m-ins.Slantr_ins;rk(2)*randn];
        kf = myekf1('hk',kf, ins,'HorizR_h');
        kf = myekf1('algo',kf, 'M');
        xkpk(ki,:)=[kf.xk',diag(kf.Pxk)',t];
        [kf, ins] = kffeedback(kf, ins, 1, 'avp');
        avp_pureins(ki,:)=[ins.avp',t];
        ki=ki+1;
    end
    ins.pos(3)=0;
    ins_pure.pos(3)=0;

    avp_ins_full(i,:)=[ins.avp',t];
    avp_pureins_full(i,:)=[ins_pure.avp',t];

    if (i/l_used- lastprecent> 0.2)
        disp("processing " + num2str(floor(i * 100 / l_used)) + " %!");
        lastprecent = i / l_used ;
    end
end
toc
%% 绘图反馈之后的轨迹
% plotTrajectoriesComparison(avp_ref, avp_ins_full, 0.01)
close all
[f_trajectory, f_errors] = plotTrajectoriesComparison_Three(avp_ref, avp_ins_full, avp_pureins_full);
hold on
plot(avp_ref(1:l_used,end),distances_N_by_1_max(1:l_used)*1000*0.02,'DisplayName','2%D误差界限')
%%
close all
[f_trajectory, f_errors] = plotTrajectoriesComparison_Three(avp_ref, avp_ins_full, avp_pureins_full);
hold on
plot(avp_ref(1:l_used,end),400*ones(1,l_used),'DisplayName','400m误差界限')
%%
twoDdata_error_limit=[avp_ref(1:l_used,end),distances_N_by_1_max(1:l_used)*1000*0.02];
[eval_results, f_errors_plot] = evaluateNavigationPerformance(avp_ref, avp_ins_full, avp_pureins_full, twoDdata_error_limit);
%%
twoDdata_error_limit=[avp_ref(1:l_used,end),400*ones(l_used,1)];
[eval_results, f_errors_plot] = evaluateNavigationPerformance(avp_ref, avp_ins_full, avp_pureins_full, twoDdata_error_limit);

%% 单反馈点的状态估计和结果，用于查看估计准确度
xkpk(ki:end,:)=[];
avp_pureins(ki:end,:)=[];
avp_offset=avp_pureins(:,1:9)-xkpk(:,1:9);
avp_offset(:,10)=avp_pureins(:,10);
%% 比较 反馈前后的误差
avperr = avpcmpplot(avp_ref, avp_pureins);
avperr1 = avpcmpplot(avp_ref, avp_offset);

plotTrajectoriesComparison(avp_ref(1:l_used,:), avp_pureins, 0.01)
plotTrajectoriesComparison(avp_ref(1:l_used,:), avp_offset, 0.01)
%% 误差进行绘图 估计误差与实际误差对比
tt=60*100;
error=avp_pureins(:,1:9)-avp_ref(tt:tt:i,1:9);
error(:,10)=avp_pureins(:,10);

myfigurestartup(10,10,'prese')
subplot 331,plot(xkpk(:,end),xkpk(:,7),error(:,end),error(:,7))
legend('propagate-dlat','dlat')
subplot 332,plot(xkpk(:,end),xkpk(:,8),error(:,end),error(:,8))
legend('propagate-dlon','dlon')
subplot 333,plot(xkpk(:,end),xkpk(:,9),error(:,end),error(:,9))
legend('propagate-dh','dh')
subplot 337,plot(xkpk(:,end),xkpk(:,1),error(:,end),error(:,1))
legend('propagate-dpitch','dpitch')
subplot 338,plot(xkpk(:,end),xkpk(:,2),error(:,end),error(:,2))
legend('propagate-droll','droll')
subplot 339,plot(xkpk(:,end),xkpk(:,3),error(:,end),error(:,3))
legend('propagate-dheading','dlon')
subplot 334,plot(xkpk(:,end),xkpk(:,4),error(:,end),error(:,4))
legend('propagate-dve','dve')
subplot 335,plot(xkpk(:,end),xkpk(:,5),error(:,end),error(:,5))
legend('propagate-dvn','dvn')
subplot 336,plot(xkpk(:,end),xkpk(:,6),error(:,end),error(:,6))
legend('propagate-dvu','dvu')

