% 1. 环境与参数设置
clear all

global glv; if isempty(glv), glvs; end 
ts = 0.01;               % 提高步长以加快 2000 秒轨迹的仿真速度
v0 = 0.5;               % 巡航航速 (m/s)
target_depth = 1000;    % 目标深度 (m)
pitch_angle = 15;       % 下潜角度 (deg)
pitch_rate = 1.5;       % 俯仰变化速率 (deg/s)

% 初始状态：水面 [0,0,0], 速度 2m/s, 位置[经, 纬, 高0]
avp0 = [[0;0;0]; [0;v0;0]; [deg2rad(34); deg2rad(108); 0]]; 

% 2. 构建 1000 米下潜轨迹段
seg = trjsegment([], 'init', v0);
% seg = trjsegment(seg, 'uniform', 30); % 水面平飞 10s

% % 计算达到 15 度俯仰所需时间: 15 / 1.5 = 10s
seg = trjsegment(seg, 'headdown', 10, pitch_rate); 

% 计算保持 15 度下潜 1000 米所需的剩余时间 (约 1930s)
% 设置转弯速率为 3 deg/s (转一圈约 120s)
seg = trjsegment(seg, 'turnleft', 1930*4, 3); 

% 改平 (抬起头)
seg = trjsegment(seg, 'headup', 10, pitch_rate);
% seg = trjsegment(seg, 'uniform', 20);

% seg = trjsegment(seg, 'uniform',      300);
% seg = trjsegment(seg, 'turnleft', 90, 1 );
% seg = trjsegment(seg, 'uniform',      300);
% seg = trjsegment(seg, 'turnleft', 90, 1 );
% seg = trjsegment(seg, 'uniform',      300);
% seg = trjsegment(seg, 'turnleft', 90, 1 );
% seg = trjsegment(seg, 'uniform',      300);
trj = trjsimu(avp0, seg.wat, ts, 1); % 只需要位置和姿态信息就可以
figure('Color', 'w');

% 3. 提取轨迹并转换坐标
lat = trj.avp(:,7); lon = trj.avp(:,8); h = trj.avp(:,9);
xyztrj = zeros(length(h), 3);
xyztrj(:,1) = (lon - lon(1)) * glv.Re * cos(lat(1)); % 东向 X
xyztrj(:,2) = (lat - lat(1)) * glv.Re;              % 北向 Y
xyztrj(:,3) = h;                                    % 高度 Z (负值表示水下)

plot3(xyztrj(:,1), xyztrj(:,2), xyztrj(:,3), 'b-', 'LineWidth', 1.5); hold on;
% plot3(stations(:,1), stations(:,2), stations(:,3), 'k^', 'MarkerSize', 10, 'MarkerFaceColor', 'y');
grid on; axis equal; view(3);
xlabel('East (m)'); ylabel('North (m)'); zlabel('Depth (m)');
title('1000米螺旋下潜 3D 轨迹');
save data.mat xyztrj trj
%%
eb=0.007;
db=7;
web=0.0005;
wdb=10e-6*1e5/3600;
trj.imu= imuadderr(trj.imu, imuerrset(eb, db, web, wdb));

pva = avpENU2NED(trj.avp);
truthpath='D:\GitHub\KF-GINS-Matlab\潜标位置标定/truth.nav';
navfp = fopen(truthpath, 'wt');
fprintf(navfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', pva');
imu = imuRFU2FRD(trj.imu);
imupath='D:\GitHub\KF-GINS-Matlab\潜标位置标定/imu.txt';
navfp = fopen(imupath, 'wt');
fprintf(navfp, '%12.6f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f\n', imu');