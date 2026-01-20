%% 用于仿真与实验数据类似的数据
clear all
refpva=importdata('旋转收缩方案/input/truth.nav');
imu120=importdata('旋转收缩方案/input/IMU_120.txt');
%% 转换坐标系
glvs
refavp=pvaNED2ENU(refpva);
imu1201=imuFRD2RFU(imu120);
%% 模拟轨迹
yaw=r2d(refavp(:,3));
xx=[1,3921,7001,12658,15624,58769,85522,98542,129033,135292,145273,...
    173556,180845,185802,192935,210573,238800,265681,293682,307233,...
    327564,354446,404289,419691,499999];
yy=yaw(xx)';
% myfigurestartup(5,5,'prese')
% % plot(yaw)
% % hold on
% plot(xx,yy)
dx=diff(xx);
dy=diff(yy);
w=dy./dx;
m=[w*100;dx/100];
%%
ts = 0.01;  
avp0 = [[0;0;d2r(37)]; [0;0;0]; refavp(1,7:9)']; 
xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      m(2,1));
seg = trjsegment(seg, 'accelerate',   10, xxx, 0.43); 
for i=2:24
    seg = trjsegment(seg, 'turnleft', m(2,i),m(1,i));
end

trj = trjsimu(avp0, seg.wat, ts, 1); % 只需要位置和姿态信息就可以
[nn, ts, nts] = nnts(1, trj.ts);
sum(trj.wat(:,1))
%%
% figure
% insplot(trj.avp)
% figure
% insplot(refavp)
%% 
imu_use11=addnoise(trj.imu);
%    0.01 eb - gyro constant bias (deg/h)
%    7 db - acc constant bias (ug)
%    0.0005 web - angular random walk (deg/sqrt(h))
%    10e-6*1e5/3600 wdb - velocity random walk (ug/sqrt(Hz))
eb=0.01;
db=7;
web=0.0005;
wdb=10e-6*1e5/3600;
% imu_use = imuadderr(trj.imu, imuerrset(eb, db, web, wdb,...
%     web, 4, wdb ,4, 5 , 10, 5, 10, 10, 10, 10));

imu_use = imuadderr(trj.imu, imuerrset(eb, db, web, wdb));

imuplot(imu_use11)
imuplot(imu_use)
% eb=0.05;
% db=10;
% web=0.003;
% wdb=10e-5*1e5/3600;
% % imu_use005 = imuadderr(trj.imu, imuerrset(eb, db, web, wdb,...
% %     web, 4, wdb ,4, 5 , 10, 5, 10, 10, 10, 10));
% imu_use005 = imuadderr(trj.imu, imuerrset(eb, db, web, wdb));
% eb=0.003;
% db=7;
% web=0.0003;
% wdb=10e-7*1e5/3600;
% 
% % imu_use0003 = imuadderr(trj.imu, imuerrset(eb, db, web, wdb,...
% %     web, 4, wdb ,4, 5 , 10, 5, 10, 10, 10, 10));
% imu_use0003 = imuadderr(trj.imu, imuerrset(eb, db, web, wdb));
% eb=0.005;
% db=7;
% web=0.0003;
% wdb=10e-7*1e5/3600;
% % imu_use0005 = imuadderr(trj.imu, imuerrset(eb, db, web, wdb,...
% %     web, 4, wdb ,4, 5 , 10, 5, 10, 10, 10, 10));
% imu_use0005 = imuadderr(trj.imu, imuerrset(eb, db, web, wdb));

%%
% avp01=refavp(1,1:9);
% avp01=avpadderr(avp01,avperrset([0.003; 0.003; 0.023]*60,0.01,1));
% avp00=avpadderr(avp0,avperrset([0.003; 0.003; 0.023]*60,0.01,1));
% 
% avp001=inspure(imu1201,avp01);
% avp000=inspure(imu_use,avp00);
% 
% avpcmpplot(trj.avp,avp000);
% avpcmpplot(refavp,avp001);
%%
refpva_simu=avpENU2NED(trj.avp);
imu_simu=imuRFU2FRD(imu_use);
%%
% 定义参考原点的绝对位置，单位为度 [纬度, 经度, 高度]
beacon0 = trj.avp(1,7:9);
% 原始等边三角形坐标（单位：米）
dxyz_original = [0, -5*sqrt(3), 0;
                -10, 5*sqrt(3), 0;
                -20, -5*sqrt(3), 0;] * 1000;
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
rrm = dxyz2pos(dxyz, beacon0');
ddm = r2d(rrm(:, 1:2));
beaconxyz = dxyz(1:3, :);
beaconrrm = rrm(1:3, :);
beaconddm = ddm(1:3, :);

trajectory_ddm=refpva_simu(:, 3:5);
trajectory_rrm=refpva_simu(:, 3:5);
trajectory_rrm(:,1:2)=d2r(trajectory_rrm(:,1:2));
trajectory_xyz = pos2dxyz(trajectory_rrm, beacon0');

% 绘图
plot_trajectory_and_beacons(trajectory_xyz/1000, beaconxyz, beaconddm, trajectory_ddm)

trajectory_x = trajectory_xyz(:, 1);
trajectory_y = trajectory_xyz(:, 2);
for i=1:3
    % 获取信标的坐标 (东向，北向，天向)
    beacon1_x = beaconxyz(i, 1);
    beacon1_y = beaconxyz(i, 2);

    % 计算每个轨迹点到第一个信标的2D距离

    distances(:,i) = sqrt((trajectory_x - beacon1_x).^2 + ...
        (trajectory_y - beacon1_y).^2);
    % 创建新的图窗来绘制距离曲线
    figure;
    plot(refpva_simu(:,2), distances(:,i), 'LineWidth', 1.5); % 洋红色实线
    xlabel('时间 (s) ');
    ylabel('距离 (km)');
    title('轨迹点到信标的距离');
    grid on;
end
distances_N_by_1_max = max(distances, [], 2);
%%
beacon1=ones(length(distances),3)*diag(beaconrrm(1,:));
beacon2=ones(length(distances),3)*diag(beaconrrm(2,:));
beacon3=ones(length(distances),3)*diag(beaconrrm(3,:));
id=100:100:length(distances);
range1=[refpva_simu(id,2),distances(id,1),distances(id,1),beacon1(id,:)];
range2=[refpva_simu(id,2),distances(id,2),distances(id,2),beacon2(id,:)];
range3=[refpva_simu(id,2),distances(id,3),distances(id,3),beacon3(id,:)];

output_file="D:\Github\KF-GINS-Matlab\旋转收缩方案\input_simu\range1.txt";
try
    writematrix(range1, output_file, 'Delimiter', ' ');
    fprintf('信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
output_file="D:\Github\KF-GINS-Matlab\旋转收缩方案\input_simu\range2.txt";
try
    writematrix(range2, output_file, 'Delimiter', ' ');
    fprintf('信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
output_file="D:\Github\KF-GINS-Matlab\旋转收缩方案\input_simu\range3.txt";
try
    writematrix(range3, output_file, 'Delimiter', ' ');
    fprintf('信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
%%
output_file="D:\Github\KF-GINS-Matlab\旋转收缩方案\input_simu\IMU_120.txt";
try
    writematrix(imu_simu, output_file, 'Delimiter', ' ');
    fprintf('信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
imu_simu005=imuRFU2FRD(imu_use005);
output_file="D:\Github\KF-GINS-Matlab\旋转收缩方案\input_simu\IMU_120_005.txt";
try
    writematrix(imu_simu005, output_file, 'Delimiter', ' ');
    fprintf('信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end

imu_simu0003=imuRFU2FRD(imu_use0003);
output_file="D:\Github\KF-GINS-Matlab\旋转收缩方案\input_simu\IMU_120_0003.txt";
try
    writematrix(imu_simu0003, output_file, 'Delimiter', ' ');
    fprintf('信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end

imu_simu0005=imuRFU2FRD(imu_use0005);
output_file="D:\Github\KF-GINS-Matlab\旋转收缩方案\input_simu\IMU_120_0005.txt";
try
    writematrix(imu_simu0005, output_file, 'Delimiter', ' ');
    fprintf('信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end

output_file="D:\Github\KF-GINS-Matlab\旋转收缩方案\input_simu\truth.txt";
try
    writematrix(refpva_simu, output_file, 'Delimiter', ' ');
    fprintf('信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end