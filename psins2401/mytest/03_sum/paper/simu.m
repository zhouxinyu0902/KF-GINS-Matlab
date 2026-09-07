close all
clear
glvs
% avp0 = [[0;0;d2r(-90)]; [0;0;0]; glv.pos0]; 
xxx = [];
% 扫描式
v = 0.4;

% len=2000;
% inter=200;
% dt1=len/v;
% dt2=inter/v;
% seg = trjsegment(xxx, 'init',         0);
% seg = trjsegment(seg, 'uniform',      20); % 保持原来的状态不变
% seg = trjsegment(seg, 'accelerate',   10, xxx, v/10); % 加速
% seg = trjsegment(seg, 'uniform',      dt1); % 保持原来的状态不变
% for i=1:4
% seg = trjsegment(seg, 'turnleft', 90, 1);
% seg = trjsegment(seg, 'uniform',      dt2);
% seg = trjsegment(seg, 'turnleft', 90, 1);
% seg = trjsegment(seg, 'uniform',      dt1);
% seg = trjsegment(seg, 'turnright', 90, 1);
% seg = trjsegment(seg, 'uniform',      dt2);
% seg = trjsegment(seg, 'turnright', 90, 1);
% seg = trjsegment(seg, 'uniform',      dt1);
% end
% seg = trjsegment(seg, 'turnleft', 90, 1);
% seg = trjsegment(seg, 'uniform',      dt2);
% seg = trjsegment(seg, 'turnleft', 90, 1);
% seg = trjsegment(seg, 'uniform',      dt1);
% seg = trjsegment(seg, 'turnright', 90, 1);
% seg = trjsegment(seg, 'uniform',      dt2);
% seg = trjsegment(seg, 'turnright', 90, 1);
% seg = trjsegment(seg, 'uniform',      dt1);
% seg = trjsegment(seg, 'turnleft', 90, 1);
% seg = trjsegment(seg, 'uniform',      dt2);
% seg = trjsegment(seg, 'turnleft', 90, 1);
% seg = trjsegment(seg, 'uniform',      dt1);
% seg = trjsegment(seg, 'turnright', 90, 1);
% seg = trjsegment(seg, 'uniform',      dt2);
% seg = trjsegment(seg, 'turnright', 90, 1);

% 方形轨迹
% v = 0.2;
% avp0 = [[0;0;d2r(180)]; [0;0;0]; [0.307000306109691	2.05584958502621	-3893.06600000000]']; 
% seg = trjsegment(xxx, 'init',         0);
% seg = trjsegment(seg, 'accelerate',   10, xxx, v/10); % 加速
% seg = trjsegment(seg, 'uniform',      5000-200);
% seg = trjsegment(seg, 'turnleft', 90-10, 1);
% seg = trjsegment(seg, 'uniform',      1600);
% seg = trjsegment(seg, 'accelerate',   10, xxx, 0.22/10); % 加速
% seg = trjsegment(seg, 'turnleft', 90+10, 1);
% seg = trjsegment(seg, 'uniform',      2000+200+1);
% seg = trjsegment(seg, 'turnleft', 90, 1);
% seg = trjsegment(seg, 'uniform',      1400);
% seg = trjsegment(seg, 'turnleft', 90, 1);

% 竖线
% avp0 = [[0;0;d2r(180)]; [0;0;0]; glv.pos0]; 
% seg = trjsegment(xxx, 'init',         0);
% seg = trjsegment(seg, 'accelerate',   10, xxx, v/10); % 加速
% seg = trjsegment(seg, 'uniform',      5000);

% 横线
% avp0 = [[0;0;d2r(90)]; [0;0;0]; glv.pos0]; 
% seg = trjsegment(xxx, 'init',         0);
% seg = trjsegment(seg, 'accelerate',   10, xxx, v/10); % 加速
% seg = trjsegment(seg, 'uniform',      5000);


% trj_001= trjsimu(avp0, seg.wat, 0.01, 1); 
% trj_005=trjsimu(avp0, seg.wat, 0.05, 1); 
% trj_01=trjsimu(avp0, seg.wat, 0.1, 1); 
trj_05=trjsimu(avp0, seg.wat, 0.5, 1); 
%%
trj_05.avp(end,end)
myfigurestartup(10,10,'prese');
insplot(trj_05.avp)
glvs
%% 航位推算的数据准备（DVL、罗盘）
trj=trj_05; % 采样时间0.5s
avp_ref = trj.avp;
ts=trj.ts;
ll=length(avp_ref);
% DVL 存在安装角偏差（忽略） 刻度系数误差 常值误差
inst = [0.1; 0.1; 0.2]*glv.min;
kod = 1.004; % 手册标注0.4%±2mm/s
w_dvl = 0.004;% 手册标注4mm/s，测高高度0.5~30
% 罗盘 存在常值和随机误差
web = d2r(0.1);
% 仿真航向角
const_pitch_roll = d2r(0.1); 
% const_yaw = d2r(4); 

% 新的仿真航向角的方式
% 假设 avp_ref(:,3) 是以弧度为单位的航向角
% 使用 cos(2 * theta) 构造周期函数
% 当 theta = 0 或 pi (180 deg) 时，cos 为 1，结果为 4
% 当 theta = -pi/2 (-90 deg) 时，cos 为 -1，结果为 -4
dynamic_bias_deg = -4 * cos(2 * avp_ref(:,3)); 
const_yaw = d2r(dynamic_bias_deg); % 转换为弧度

web_pr = d2r(0.05); % Pitch/Roll 噪声
web_y  = web;  % Yaw 噪声 (磁罗盘噪声通常较大)

compass = zeros(ll, 4);
compass(:,1) = avp_ref(:,1) + const_pitch_roll + normrnd(0, web_pr, ll, 1);
compass(:,2) = avp_ref(:,2) + const_pitch_roll + normrnd(0, web_pr, ll, 1);
compass(:,3) = avp_ref(:,3) + const_yaw        + normrnd(0, web_y,  ll, 1);
compass(:,4) = avp_ref(:,end);

figure
subplot 121 
plot(compass(:,3))
hold on 
plot(avp_ref(:,3))
subplot 122 
plot(r2d(compass(:,3))-r2d(avp_ref(:,3)))

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
depthstd = 0.4;
depth=avp_ref(:,9)+normrnd(0,depthstd,size(avp_ref(:,9)));

load('data_1\deep-sea_optimized.mat','USBL_out','HorizRangePropaTsm');
moving_beacons.pos=[d2r(USBL_out.Transducer.Lat)',d2r(USBL_out.Transducer.Lon)',-USBL_out.Ship.Depth'];
moving_beacons.range = HorizRangePropaTsm(1,:);
% save paper\data_dr_col vxy compass trj avp_ref web depthstd depth const_yaw
% save paper\data_dr_col_minus vxy compass trj avp_ref web depthstd depth const_yaw
% save paper\data_dr_row vxy compass trj avp_ref web depthstd depth const_yaw
% save paper\data_dr_row_minus vxy compass trj avp_ref web depthstd depth const_yaw
save paper\data_dr_square vxy compass trj avp_ref web depthstd depth const_yaw moving_beacons
% save paper\data_dr_scan vxy compass trj avp_ref web depthstd depth const_yaw