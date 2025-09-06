%% 仿真数据，已有imu数据与truth数据
clear
clc
truth=importdata('dataset-simu/line/input/truth.nav');
fprintf("参考结果的频率：%.2f Hz\n",1/(truth(2,2)-truth(1,2)))
%% 输出文件夹
outputpath='dataset-simu\line\input';
%% 仿真gnss数据和LBL数据，两者精度不同，频率不同
Re=6378137;
gnss=truth(200:200:end,2:5);
gnss(:,2:3)=gnss(:,2:3)+normrnd(0,0.02/Re*180/pi,size(gnss(:,2:3)));
gnss(:,4)=gnss(:,4)+normrnd(0,0.02,size(gnss(:,4)));

LBL(:,1:3)=truth(200:200:end,3:5);
LBL(:,4)=truth(1:200:end,2);
LBL(:,1:2)=LBL(:,1:2)+normrnd(0,2/Re*180/pi,size(LBL(:,1:2)));
LBL(:,3)=-LBL(:,3)+normrnd(0,2,size(LBL(:,3)));


myfigurestartup(7,7,'prese')
plot(truth(:,4),truth(:,3))
hold on
plot(gnss(:,3),gnss(:,2))
hold on
plot(LBL(:,2),LBL(:,1))
legend('ref','gnss','LBL')
axis equal

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
%% 高度信息仿真-低频-按照GNSS仿真的
height=gnss(:,[1,4]);
figure
plot(height(:,1),height(:,2))

% 添加误差
% 深度计0.02%×深度的误差，1500米的时候0.3m
% ISD 400的频率最高是100Hz
height(:,2)=height(:,2)+normrnd(0,0.3,size(height(:,2)));
hold on
plot(height(:,1),height(:,2))
height_output_file=outputpath+"\height-1Hz.txt";
try
    writematrix(height, height_output_file, 'Delimiter', ' ');
    fprintf('高度信息已成功写入到 %s\n', height_output_file);
catch ME
    error('错误：写入 height.txt 文件失败。错误信息：%s', ME.message);
end
%% 高度信息仿真-高频-按照参考结果仿真的
height=truth(1:2:end,[2,5]);
plot(height(:,1),height(:,2))

% 添加误差
% 深度计0.02%×深度的误差，1500米的时候0.3m
% ISD 400的频率最高是100Hz
height(:,2)=height(:,2)+normrnd(0,0.3,size(height(:,2)));
hold on
plot(height(:,1),height(:,2))
height_output_file=outputpath+"\height-100Hz.txt";
try
    writematrix(height, height_output_file, 'Delimiter', ' ');
    fprintf('高度信息已成功写入到 %s\n', height_output_file);
catch ME
    error('错误：写入 height.txt 文件失败。错误信息：%s', ME.message);
end
%% 仿真信标，以gnss为参考计算信标距离
%% 仿真移动信标
glvs
avp0 = [[0;0;d2r(25)]; [0;0;0]; [d2r(truth(1,3:4))';-1200]];
ts=1;
xxx = [];
% seg = trjsegment(xxx, 'init',         0);
% seg = trjsegment(seg, 'uniform',      20); % 保持原来的状态不变
% seg = trjsegment(seg, 'accelerate',   5, xxx, 0.3); % 加速
% seg = trjsegment(seg, 'uniform',      1200);
% % seg = trjsegment(seg, 'headdown', 10, 5);
% seg = trjsegment(seg, 'turnleft', 150, 0.6);
% seg = trjsegment(seg, 'uniform',      400);
% seg = trjsegment(seg, 'turnleft', 20, 3);
% seg = trjsegment(seg, 'uniform',      700);
% seg = trjsegment(seg, 'turnleft', 74, 0.6);
% seg = trjsegment(seg, 'uniform',      430);
% seg = trjsegment(seg, 'turnleft', 18.43, 4);
% seg = trjsegment(seg, 'uniform',      350);

seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'accelerate',   5, xxx, 0.3); % 加速
seg = trjsegment(seg, 'uniform',      1200);
seg = trjsegment(seg, 'turnleft', 150, 0.6);
seg = trjsegment(seg, 'uniform',      500);
seg = trjsegment(seg, 'turnleft', 20, 3);
seg = trjsegment(seg, 'uniform',      700);
seg = trjsegment(seg, 'turnleft', 70, 0.6);
seg = trjsegment(seg, 'uniform',      600);
seg = trjsegment(seg, 'turnleft', 18, 4);
seg = trjsegment(seg, 'uniform',      400);

% fprintf("航行器轨迹时长（s）：%.2f\n",length(truth)*0.005)
fprintf("航行器轨迹时长（s）：%.2f\n",truth(end,2)-truth(1,2))
fprintf("仿真轨迹时长（s）：%.2f\n",sum(seg.wat(:,1)))
param = Param();
trj= trjsimu(avp0, seg.wat, ts, 1);
figure
insplot(trj.avp(:,7:9))
bcnMoving=[trj.avp(1:length(gnss),7:8)*param.R2D,trj.avp(1:length(gnss),9)];

%% 三个信标
pos=truth(:,3:5);
pos(:,1:2)=d2r(pos(:,1:2));
pos0=truth(1,3:5)';
pos0(1:2)=d2r(pos0(1:2));
beaconxyz=[[5,-5*sqrt(3),0]*1000;...
    [-5,5*sqrt(3),0]*1000;...
    [-15,-5*sqrt(3),0]*1000;...
    [2000,2000,0]];
for i=1:4
  BCN(i,:)=dxyz2pos(beaconxyz(i,:),pos0);  
end
BCNddm=BCN;
BCNddm(:,1:2)=r2d(BCN(:,1:2));
%% 所有信标绘图
close all
figure
plot(truth(:,4),truth(:,3))
hold on
plot(bcnMoving(:,2),bcnMoving(:,1))
for i=1:3
    hold on
    plot(BCNddm(i,2),BCNddm(i,1),'*')
end
% 以起点为坐标原点，绘制以km为单位的图
trajectory_xyz_km=pos2dxyz(pos,pos0)/1000;
plot_trajectory_and_beacons(trajectory_xyz_km, beaconxyz, BCNddm, truth(:,3:5))
%% 信标位置生成信标距离
range_beacon=cell(1,4);
truth_1s=truth(200:200:end,:);
for i=1:4
    range_beacon{i}=bcn2range(truth_1s,BCNddm(i,:));
end
range_moving=bcn2range(truth_1s,bcnMoving);
%%
xyztrj=pos2dxyz(pos(200:200:end,:),pos0);

distances_m = sqrt((xyztrj(:,1) - beaconxyz(1,1)).^2 + ...
    (xyztrj(:,2)  - beaconxyz(1,2)).^2 + ...
    (xyztrj(:,3)  - beaconxyz(1,3)).^2);
figure
plot(1:3660,distances_m)
hold on
plot(1:3660,range_beacon{1}(:,2))
%% 保存数据
for i=1:4
    range_beacon{i}(:,2:3)=range_beacon{i}(:,2:3)+normrnd(0,5,size(range_beacon{i}(:,2:3)));
    range_static_output_file=outputpath+sprintf("//range_static_%d.txt",i);
    try
        writematrix(range_beacon{i}, range_static_output_file, 'Delimiter', ' ');
        fprintf('距离信息已成功写入到 %s\n', range_static_output_file);
    catch ME
        error('错误：写入 range_static.txt 文件失败。错误信息：%s', ME.message);
    end
end

range_moving(:,2:3)=range_moving(:,2:3)+normrnd(0,5,size(range_moving(:,2:3)));
range_moving_output_file=outputpath+"\range_moving.txt";
try
    writematrix(range_moving, range_moving_output_file, 'Delimiter', ' ');
    fprintf('距离信息已成功写入到 %s\n', range_moving_output_file);
catch ME
    error('错误：写入 range_moving.txt 文件失败。错误信息：%s', ME.message);
end

