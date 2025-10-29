%% 仿真数据，已有imu数据与truth数据
clear
clc
glvs
truth=importdata('dataset-simu/line/input/truth.nav');
fprintf("参考结果的频率：%.2f Hz\n",1/(truth(2,2)-truth(1,2)))
%% 输出文件夹
outputpath='dataset-simu\line\input';
%% 仿真gnss数据和LBL数据，两者精度不同，频率不同
Re=6378137;
gnss = truth(100:100:end,2:5);
gnss(:,2:3)=gnss(:,2:3)+normrnd(0,0.02/Re*180/pi,size(gnss(:,2:3)));
gnss(:,4)=gnss(:,4)+normrnd(0,0.02,size(gnss(:,4)));

LBL(:,1:3)=truth(100:100:end,3:5);
LBL(:,4)=truth(1:100:end,2);
LBL(:,1:2)=LBL(:,1:2)+normrnd(0,2/Re*180/pi,size(LBL(:,1:2)));
LBL(:,3)=-LBL(:,3)+normrnd(0,2,size(LBL(:,3)));

output_file=outputpath+"\gnss.txt";
try
    writematrix(gnss, output_file, 'Delimiter', ' ');
    fprintf('gnss信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end

output_file=outputpath+"\LBL.txt";
try
    writematrix(LBL, output_file, 'Delimiter', ' ');
    fprintf('LBL信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
%% 高度信息仿真-高频-按照参考结果仿真的
height=truth(:,[2,5]);
plot(height(:,1),height(:,2))

% 添加误差
% 深度计0.02%×深度的误差，1500米的时候0.3m
% ISD 400的频率最高是100Hz
height(:,2)=height(:,2)+normrnd(0,0.5,size(height(:,2)));
height_output_file=outputpath+"\height-100Hz.txt";
try
    writematrix(height, height_output_file, 'Delimiter', ' ');
    fprintf('高度信息已成功写入到 %s\n', height_output_file);
catch ME
    error('错误：写入 height.txt 文件失败。错误信息：%s', ME.message);
end
%% 仿真信标，以gnss为参考计算信标距离
%% 仿真移动信标
% glvs
% pos0=d2r(truth(1,3:5));
% dxyz=[200,200,0];
% avp0 = [[0;0;d2r(91)]; [0;0;0]; dxyz2pos(dxyz,pos0')'];
% ts=1;
% xxx = [];
% % seg = trjsegment(xxx, 'init',         0);
% % seg = trjsegment(seg, 'uniform',      20); % 保持原来的状态不变
% % seg = trjsegment(seg, 'accelerate',   5, xxx, 0.3); % 加速
% % seg = trjsegment(seg, 'uniform',      1200);
% % % seg = trjsegment(seg, 'headdown', 10, 5);
% % seg = trjsegment(seg, 'turnleft', 150, 0.6);
% % seg = trjsegment(seg, 'uniform',      400);
% % seg = trjsegment(seg, 'turnleft', 20, 3);
% % seg = trjsegment(seg, 'uniform',      700);
% % seg = trjsegment(seg, 'turnleft', 74, 0.6);
% % seg = trjsegment(seg, 'uniform',      430);
% % seg = trjsegment(seg, 'turnleft', 18.43, 4);
% % seg = trjsegment(seg, 'uniform',      350);
% 
% seg = trjsegment(xxx, 'init',         0);
% seg = trjsegment(seg, 'accelerate',   5, xxx, 0.1); % 加速
% seg = trjsegment(seg, 'uniform',      1000);
% seg = trjsegment(seg, 'turnleft', 150, 0.6);
% seg = trjsegment(seg, 'uniform',      500);
% seg = trjsegment(seg, 'turnright', 20, 3);
% seg = trjsegment(seg, 'uniform',      500);
% seg = trjsegment(seg, 'turnleft', 70, 0.6);
% seg = trjsegment(seg, 'uniform',      600);
% seg = trjsegment(seg, 'turnright', 18, 4);
% seg = trjsegment(seg, 'uniform',      400);
% 
% % fprintf("航行器轨迹时长（s）：%.2f\n",length(truth)*0.005)
% fprintf("航行器轨迹时长（s）：%.2f\n",truth(end,2)-truth(1,2))
% fprintf("仿真轨迹时长（s）：%.2f\n",sum(seg.wat(:,1)))
% param = Param();
% trj= trjsimu(avp0, seg.wat, ts, 1);
% 
% bcnMoving=[trj.avp(1:length(gnss),7:8)*param.R2D,trj.avp(1:length(gnss),9)];
% figure
% plot(bcnMoving(:,2),bcnMoving(:,1))
% hold on
% plot(truth(:,4),truth(:,3))
%% 三个信标
beacon0 = d2r([truth(1,3), truth(1,4), 0]);
% 原始等边三角形坐标（单位：米）
dxyz_original = [0, -5*sqrt(3), 0;
                -10, 5*sqrt(3), 0;
                -20, -5*sqrt(3), 0;] * 1000;
% 定义旋转角度（15度）
theta_deg = 15;
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
GNSS_1s = gnss;
trj=GNSS_1s(:, 2:4);
trj(:,1:2)=d2r(trj(:,1:2));
trajectory_xyz = pos2dxyz(trj, beacon0');
trajectory_ddm=GNSS_1s(:, 2:4);

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
end
% 创建新的图窗来绘制距离曲线
myfigurestartup(12,5,'prese');
for i=1:3
    subplot(1,3,i)
    plot(GNSS_1s(:,1), distances(:,i), 'LineWidth', 1.5); % 洋红色实线
    xlabel('时间 (s) ');
    ylabel('距离 (km)');
    title('轨迹点到信标的距离');
    grid on;
end
%% 
beacon1=ones(length(distances),3)*diag(beaconrrm(1,:));
beacon2=ones(length(distances),3)*diag(beaconrrm(2,:));
beacon3=ones(length(distances),3)*diag(beaconrrm(3,:));
range1=[GNSS_1s(:,1),distances(:,1),distances(:,1),beacon1];
range2=[GNSS_1s(:,1),distances(:,2),distances(:,2),beacon2];
range3=[GNSS_1s(:,1),distances(:,3),distances(:,3),beacon3];
range_beacon={range1,range2,range3};
%% 保存数据
for i=1:3
    range_beacon{i}(:,2:3)=range_beacon{i}(:,2:3)+normrnd(0,5,size(range_beacon{i}(:,2:3)));
    range_static_output_file=outputpath+sprintf("//range_static_%d.txt",i);
    try
        writematrix(range_beacon{i}, range_static_output_file, 'Delimiter', ' ');
        fprintf('距离信息已成功写入到 %s\n', range_static_output_file);
    catch ME
        error('错误：写入 range_static.txt 文件失败。错误信息：%s', ME.message);
    end
end

