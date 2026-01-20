clear all
glvs
avp0 = [[0;0;d2r(0)]; [0;0;0]; [d2r([28.227074;112.996809]);0]];
ts=0.01;
%%
% xxx = [];
% seg = trjsegment(xxx, 'init',         0);
% seg = trjsegment(seg, 'uniform',      20); % 保持原来的状态不变
% seg = trjsegment(seg, 'accelerate',   3/(20/9.8), xxx, 20/9.8); % 加速
% for i=1:7
%     seg = trjsegment(seg, 'uniform',      22);
%     seg = trjsegment(seg, 'turnleft', 18, 5);
%     seg = trjsegment(seg, 'uniform',      64);
%     seg = trjsegment(seg, 'turnleft', 18, 5);
%     seg = trjsegment(seg, 'uniform',      64);
%     seg = trjsegment(seg, 'turnleft', 18, 5);
%     seg = trjsegment(seg, 'uniform',      64);
%     seg = trjsegment(seg, 'turnleft', 18, 5);
%     seg = trjsegment(seg, 'uniform',     42);
% end
% seg = trjsegment(seg, 'uniform',      22);
% seg = trjsegment(seg, 'turnleft', 18, 5);
% seg = trjsegment(seg, 'uniform',      64);
% seg = trjsegment(seg, 'turnleft', 18, 5);
% seg = trjsegment(seg, 'uniform',      50);
% seg = trjsegment(seg, 'deaccelerate',   3/(20/9.8), xxx, 20/9.8);
n=3;
xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      20); % 保持原来的状态不变
seg = trjsegment(seg, 'accelerate',   10, xxx, 0.2); % 加速
seg = trjsegment(seg, 'uniform',      22*n);
seg = trjsegment(seg, 'turnleft', 18, 5);
seg = trjsegment(seg, 'uniform',      64*n);
seg = trjsegment(seg, 'turnleft', 18, 5);
seg = trjsegment(seg, 'uniform',      64*n);
seg = trjsegment(seg, 'turnleft', 18, 5);
seg = trjsegment(seg, 'uniform',      64*n);
seg = trjsegment(seg, 'turnleft', 18, 5);
seg = trjsegment(seg, 'uniform',     42*n);

seg = trjsegment(seg, 'uniform',      22*n);
seg = trjsegment(seg, 'turnleft', 18, 5);
seg = trjsegment(seg, 'uniform',      64*n);
seg = trjsegment(seg, 'turnleft', 18, 5);
seg = trjsegment(seg, 'uniform',      64*n);
seg = trjsegment(seg, 'turnleft', 18, 5);
seg = trjsegment(seg, 'uniform',      64*n);
seg = trjsegment(seg, 'turnleft', 18, 5);
seg = trjsegment(seg, 'uniform',     42*n);

seg = trjsegment(seg, 'uniform',      22*n);
seg = trjsegment(seg, 'turnleft', 18, 5);
seg = trjsegment(seg, 'uniform',      64*n);
seg = trjsegment(seg, 'turnleft', 18, 5);
seg = trjsegment(seg, 'uniform',      64*n);
seg = trjsegment(seg, 'turnleft', 18, 5);
seg = trjsegment(seg, 'uniform',      64*n);
seg = trjsegment(seg, 'turnleft', 18, 5);
seg = trjsegment(seg, 'uniform',     42*n);

seg = trjsegment(seg, 'uniform',      22*n);
seg = trjsegment(seg, 'turnleft', 18, 5);
seg = trjsegment(seg, 'uniform',      64*n);
seg = trjsegment(seg, 'turnleft', 18, 5);
seg = trjsegment(seg, 'uniform',      64*n);
seg = trjsegment(seg, 'turnleft', 18, 5);
seg = trjsegment(seg, 'uniform',      64*n);
seg = trjsegment(seg, 'turnleft', 18, 5);
seg = trjsegment(seg, 'uniform',     42*n);

repeats=1;
trj2= trjsimu(avp0, seg.wat, ts, repeats);

figure()
myfigurestartup(7,7,'prese')
insplot(trj2.avp)
% fprintf('轨迹总时长%d（s）\n',repeats*sum(trj2.wat(:,1)))
fprintf('轨迹总时长%d（s）\n',sum(trj2.wat(:,1)))
%%
% eb=0.003;
% db=7;
% web=0.0003;
% wdb=1e-6*1e5/3600;%1e-6m/s/sqrt(Hz),1e-6*1e5/3600 = 2.7778e-05

eb=0.1;
db=[300,300,-300];
web=0.01;
wdb=30;
% eb=0.1;
% db=100;
% web=0.01;
% wdb=10;
% 1e-5*1e5/3600;%1e-6 m/s/sqrt(Hz)
%1e-6*1e5/3600 = 2.7778e-05
rng(1);
imuerr = imuerrset(eb, db, web, wdb, web, 4, wdb ,4, 5 , 10, 5, 10, 10, 10, 10);
% imuerr = imuerrset(eb, db, web, wdb);
trjimu_line= imuadderr(trj2.imu, imuerr);

%%
path = 'D:\Github\KF-GINS-Matlab\MEMS_RANGE\data_simu';
imu_line = imuRFU2FRD(trjimu_line);
imupath=[path,'\imu.nav'];
imufp=fopen(imupath,'wt');
fprintf(imufp, '%.9f %.10f %.10f %.10f %.10f %.10f %.10f \n', imu_line');
fclose(imufp);

pva_ref_line = avpENU2NED(trj2.avp);
truthpath=[path,'\truth.nav'];
truthfp=fopen(truthpath,'wt');
fprintf(truthfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', pva_ref_line');
fclose(truthfp);
%% 仿真数据，已有imu数据与truth数据
clc
glvs
truth=importdata(truthpath);
fprintf("参考结果的频率：%.2f Hz\n",1/(truth(2,2)-truth(1,2)))
% 输出文件夹
outputpath=path;
%% 仿真gnss数据和LBL数据，两者精度不同，频率不同
Re=6378137;
gnss = truth(100:100:end,2:5);
gnss(:,2:3)=gnss(:,2:3)+normrnd(0,0.02/Re*180/pi,size(gnss(:,2:3)));
gnss(:,4)=gnss(:,4)+normrnd(0,0.02,size(gnss(:,4)));

output_file=outputpath+"\gnss.txt";
try
    writematrix(gnss, output_file, 'Delimiter', ' ');
    fprintf('gnss信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end

%% 示例调用
% % 设定参数
% BIAS = 0;         % 零偏: 0.5m
% SIGMA = 0.5;        % 标准差: 0.5m
% TAU = 10;           % 相关时间: 10s
% DT = 0.01;          % 采样间隔: 0.01s (100Hz)
% h_true_vector = truth(:,5);
% 
% % 2. 调用函数 (输入 h_true_vector 是 N*1)
% [t, h_meas, h_err] = generate_baro_alt_sim(h_true_vector, BIAS, SIGMA, TAU, DT);
% % 绘图展示结果
% figure;
% subplot(2,1,1);
% plot(t, h_err);
% title(['一阶马尔科夫过程误差 (\tau = ', num2str(TAU), 's, \sigma = ', num2str(SIGMA), 'm)']);
% xlabel('时间 (s)');
% ylabel('误差 (m)');
% grid on;
% 
% subplot(2,1,2);
% plot(t, h_true_vector, 'k-', 'LineWidth', 1.5);
% hold on;
% plot(t, h_meas, 'r:', 'LineWidth', 1);
% title('气压高度计仿真测量值 (N*1 向量输入)');
% xlabel('时间 (s)');
% ylabel('高度 (m)');
% legend('真实高度', '测量高度', 'Location', 'best');
% grid on;
% height_markov=[truth(:,2),h_meas];
% height_markov_output_file=outputpath+"\height-markov-10Hz.txt";
% try
%     writematrix(height_markov, height_markov_output_file, 'Delimiter', ' ');
%     fprintf('高度信息已成功写入到 %s\n', height_markov_output_file);
% catch ME
%     error('错误：写入 height.txt 文件失败。错误信息：%s', ME.message);
% end
%% 高度信息仿真-高频-按照参考结果仿真的
height=truth(:,[2,5]);
% 添加误差
% 深度计0.02%×深度的误差，1500米的时候0.3m
% ISD 400的频率最高是100Hz
% height(:,2)=height(:,2)+normrnd(0,0.5,size(height(:,2)));
height_output_file = outputpath+"\height-100Hz.txt";
try
    writematrix(height, height_output_file, 'Delimiter', ' ');
    fprintf('高度信息已成功写入到 %s\n', height_output_file);
catch ME
    error('错误：写入 height.txt 文件失败。错误信息：%s', ME.message);
end
%% 仿真信标，以gnss为参考计算信标距离
%% 单个信标
beacon0 = d2r([truth(1,3), truth(1,4), 0]);
% 原始等边三角形坐标（单位：米）
% dxyz_original = [0, 200, 0;
%     -250, 200, 0;
%     -400, 100, 0;
%     -400,-150, 0;
%     -250, -250, 0;
%     0, -250, 0;
%     100, -150, 0;
%     100, 100, 0;
%     -200, 0, 0];

dxyz_original = [200, 400, 0];
beaconums=size(dxyz_original,1);
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
beaconxyz = dxyz;
beaconrrm = rrm;
beaconddm = ddm;
trj=truth(:, 3:5);
trj(:,1:2)=d2r(trj(:,1:2));
trajectory_xyz = pos2dxyz(trj, beacon0');
trajectory_ddm=truth(:, 3:5);
% 绘图
plot_trajectory_and_beacons_m(trajectory_xyz, beaconxyz)
%
trajectory_x = trajectory_xyz(:, 1);
trajectory_y = trajectory_xyz(:, 2);
for i=1:beaconums
    % 获取信标的坐标 (东向，北向，天向)
    beacon1_x = beaconxyz(i, 1);
    beacon1_y = beaconxyz(i, 2);
    % 计算每个轨迹点到第一个信标的2D距离
    distances(:,i) = sqrt((trajectory_x - beacon1_x).^2 + ...
        (trajectory_y - beacon1_y).^2);
end
% 创建新的图窗来绘制距离曲线
% figure
% myfigurestartup(12,5,'prese');
% for i=1:beaconums
%     subplot(1,3,i)
%     plot(truth(:,2), distances(:,i), 'LineWidth', 1.5); % 洋红色实线
%     xlabel('时间 (s) ');
%     ylabel('距离 (km)');
%     title('轨迹点到信标的距离');
%     grid on;
% end

L = size(distances, 1);
range_beacon = cell(1, beaconums);
% 对每个信标进行循环处理
for i = 1:beaconums
    beacon_i = ones(L, 3) * diag(beaconrrm(i,:));
    range_i = [
        truth(:, 2), ...        % 真值数据 (L*1)
        distances(:, i), ...    % 测距值 (L*1)
        distances(:, i), ...    % 测距值复制 (L*1)
        beacon_i            % 信标位置 (L*3)
        ];
    range_beacon{i} = range_i;
end

%% 保存数据
for i=1:beaconums
    range_static_output_file=outputpath+sprintf("//range_static_%d.txt",i);
    try
        writematrix(range_beacon{i}, range_static_output_file, 'Delimiter', ' ');
        fprintf('距离信息已成功写入到 %s\n', range_static_output_file);
    catch ME
        error('错误：写入 range_static.txt 文件失败。错误信息：%s', ME.message);
    end
end
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
% beacon0 = d2r([truth(1,3), truth(1,4), 0]);
% % 原始等边三角形坐标（单位：米）
% dxyz_original = [0, -5*sqrt(3), 0;
%                 -10, 5*sqrt(3), 0;
%                 -20, -5*sqrt(3), 0;] * 1000;
% % 定义旋转角度（15度）
% theta_deg = 15;
% theta_rad = deg2rad(theta_deg);
% % 创建绕Z轴的旋转矩阵
% R = [cos(theta_rad), -sin(theta_rad), 0;
%      sin(theta_rad), cos(theta_rad),  0;
%      0,              0,              1];
% % 对每个点应用旋转
% dxyz_rotated = (R * dxyz_original')'; % 转置以便矩阵乘法
% % 使用旋转后的坐标
% dxyz = dxyz_rotated;
% % 分开获取信标和轨迹原点
% rrm = dxyz2pos(dxyz, beacon0');
% ddm = r2d(rrm(:, 1:2));
% beaconxyz = dxyz(1:3, :);
% beaconrrm = rrm(1:3, :);
% beaconddm = ddm(1:3, :);
% GNSS_1s = gnss;
% trj=GNSS_1s(:, 2:4);
% trj(:,1:2)=d2r(trj(:,1:2));
% trajectory_xyz = pos2dxyz(trj, beacon0');
% trajectory_ddm=GNSS_1s(:, 2:4);
%
% % 绘图
% plot_trajectory_and_beacons(trajectory_xyz/1000, beaconxyz, beaconddm, trajectory_ddm)
% trajectory_x = trajectory_xyz(:, 1);
% trajectory_y = trajectory_xyz(:, 2);
% for i=1:3
%     % 获取信标的坐标 (东向，北向，天向)
%     beacon1_x = beaconxyz(i, 1);
%     beacon1_y = beaconxyz(i, 2);
%     % 计算每个轨迹点到第一个信标的2D距离
%     distances(:,i) = sqrt((trajectory_x - beacon1_x).^2 + ...
%         (trajectory_y - beacon1_y).^2);
% end
% % 创建新的图窗来绘制距离曲线
% myfigurestartup(12,5,'prese');
% for i=1:3
%     subplot(1,3,i)
%     plot(GNSS_1s(:,1), distances(:,i), 'LineWidth', 1.5); % 洋红色实线
%     xlabel('时间 (s) ');
%     ylabel('距离 (km)');
%     title('轨迹点到信标的距离');
%     grid on;
% end
% %%
% beacon1=ones(length(distances),3)*diag(beaconrrm(1,:));
% beacon2=ones(length(distances),3)*diag(beaconrrm(2,:));
% beacon3=ones(length(distances),3)*diag(beaconrrm(3,:));
% range1=[GNSS_1s(:,1),distances(:,1),distances(:,1),beacon1];
% range2=[GNSS_1s(:,1),distances(:,2),distances(:,2),beacon2];
% range3=[GNSS_1s(:,1),distances(:,3),distances(:,3),beacon3];
% range_beacon={range1,range2,range3};
% %% 保存数据
% for i=1:3
%     range_beacon{i}(:,2:3)=range_beacon{i}(:,2:3)+normrnd(0,5,size(range_beacon{i}(:,2:3)));
%     range_static_output_file=outputpath+sprintf("//range_static_%d.txt",i);
%     try
%         writematrix(range_beacon{i}, range_static_output_file, 'Delimiter', ' ');
%         fprintf('距离信息已成功写入到 %s\n', range_static_output_file);
%     catch ME
%         error('错误：写入 range_static.txt 文件失败。错误信息：%s', ME.message);
%     end
% end