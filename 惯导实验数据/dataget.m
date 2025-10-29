clear
pva_830=importdata('惯导实验数据/input/pva_830.txt');
gps_830=importdata('惯导实验数据/input/gps_830.txt');
gps_430=importdata('惯导实验数据/input/gps_430.txt');
pva_RS=importdata('惯导实验数据/input/pva_RS.txt');
pv_RS_gnss=importdata('惯导实验数据/input/pv_RS_gnss.txt');
% %% 830纬度经度对比
% figure
% % subplot(1,2,1)
% plot(pva_830(:,2),pva_830(:,4))
% hold on
% plot(pva_RS(:,2),pva_RS(:,4))
% plot(gps_830(:,1),gps_830(:,4))
% plot(pva_830(:,2),pva_830(:,4))
% legend('830','120','gps-830','830-interp')
% % subplot(1,2,2)
% % plot(pva_830(:,2),pva_830(:,3))
% % hold on
% % plot(pva_RS(:,2),pva_RS(:,3))
% % plot(gps_830(:,1),gps_830(:,3))
% % plot(pva_830(:,2),pva_830(:,3))
% legend('830','120','gps-830','830-interp')
% figure
% index1{1}=find(pva_830(:,2)==125267):find(pva_830(:,2)==125317);
% index1{2}=find(pva_830(:,2)==125468):find(pva_830(:,2)==125515);
% index1{3}=find(pva_830(:,2)==125544):find(pva_830(:,2)==125576);
% index1{4}=find(pva_830(:,2)==126021):find(pva_830(:,2)==126146);
% for i=1:4
%     subplot(2,2,i)
%     plot(pva_830(index1{i},2),pva_830(index1{i},4))
%     % subplot(1,2,2)
%     % plot(pva_830(index1,2),pva_830(index1,3))
% end

%%
figure 
plot(diff(pva_830(:,5)/0.01))
hold on
plot(pva_830(:,8))
legend('diff')
%%
% figure
% subplot 121
% plot(pva_830(290000:end,2),pva_830(290000:end,4))
% hold on
% plot(pva_430(290000:end,2),pva_430(290000:end,4))
% legend('830','430')
% subplot 122
% plot(pva_830(290000:end,2),pva_830(290000:end,3))
% hold on
% plot(pva_430(290000:end,2),pva_430(290000:end,3))
% legend('830','430')
% %%
% index1=1:100000;
% index2=290000:length(pva_830);
% pva_8301=pva_830;
% tic
% pva_8301(index1,4) = smooth(pva_8301(index1,2),pva_8301(index1,4), 0.01, 'rloess');
% pva_8301(index2,4) = smooth(pva_8301(index2,2),pva_8301(index2,4), 0.01,'rloess');
% toc
% tic
% pva_8301(index1,3) = smooth(pva_8301(index1,2),pva_8301(index1,3), 0.01, 'rloess');
% pva_8301(index2,3) = smooth(pva_8301(index2,2),pva_8301(index2,3), 0.01,'rloess');
% toc
% %%
% index1=1:100000;
% index2=290000:length(pva_430_interp);
% pva_430_interp1=pva_430_interp;
% tic
% pva_430_interp1(index1,4) = smooth(pva_430_interp1(index1,2),pva_430_interp1(index1,4), 0.01, 'rloess');
% pva_430_interp1(index2,4) = smooth(pva_430_interp1(index2,2),pva_430_interp1(index2,4), 0.01,'rloess');
% toc
% 
% tic
% pva_430_interp1(index1,3) = smooth(pva_430_interp1(index1,2),pva_430_interp1(index1,3), 0.01, 'rloess');
% pva_430_interp1(index2,3) = smooth(pva_430_interp1(index2,2),pva_430_interp1(index2,3), 0.01,'rloess');
% toc
% %% 转为需要的
% output_file="D:\Github\KF-GINS-Matlab\惯导实验数据\input\pva_830.txt";
% try
%     writematrix(pva_8301, output_file, 'Delimiter', ' ');
%     fprintf('830导航信息已成功写入到 %s\n', output_file);
% catch ME
%     error('错误：写入文件失败。错误信息：%s', ME.message);
% end
% output_file="D:\Github\KF-GINS-Matlab\惯导实验数据\input\pva_430_interp.txt";
% try
%     writematrix(pva_430_interp1, output_file, 'Delimiter', ' ');
%     fprintf('830导航信息已成功写入到 %s\n', output_file);
% catch ME
%     error('错误：写入文件失败。错误信息：%s', ME.message);
% end
%% 水平位置对比
% glvs
% xyz_gps830=pos2dxyz(d2r(gps_830(:,3:5)),d2r(gps_830(1,3:5))');
% xyz_truth120=pos2dxyz(d2r(pva_RS(:,3:5)),d2r(gps_830(1,3:5))');
% xyz_truth830=pos2dxyz(d2r(pva_830(:,3:5)),d2r(gps_830(1,3:5))');
% xyz_pv_RS_gnss=pos2dxyz(d2r(pv_RS_gnss(:,3:5)),d2r(gps_830(1,3:5))');
% figure
% plot(xyz_gps830(:,1),xyz_gps830(:,2))
% hold on
% plot(xyz_truth120(:,1),xyz_truth120(:,2))
% plot(xyz_truth830(:,1),xyz_truth830(:,2))
% plot(xyz_pv_RS_gnss(:,1),xyz_pv_RS_gnss(:,2))
% legend('gps830','120-result','830-result','120-gnss')
% % 高度对比
% figure
% plot(gps_830(:,5))
% hold on
% plot(pva_RS(:,5))
% plot(pva_830(:,5))
% plot(pv_RS_gnss(:,5))
% legend('gps830','120-result','830-result','120-gnss')
%% 构造参考数据truth_inte
% truth120_interp=interp1(pva_RS(:,2),pva_RS(:,1:11),pva_830(:,2),"linear");
% truth_inte=[pva_830(:,1:8),truth120_interp(:,9:11)]; % 将120的姿态角作为参考
truth_inte=pva_830;
%% 构造GNSS数据：直接选择gps数据/选择参考数据
GNSS_1s=gps_830(1:100:end,:);
GNSS_01s=gps_830(1:end,:);
GNSS_ref_01s=truth_inte(1:end,[2,3:5]);
%% 根据GNSS数据构造距离数据
glvs
% % 定义参考原点/第一个信标的绝对位置，单位为度 [纬度, 经度, 高度]
% beacon0=d2r([36.40042003,120.68981831,0]);
% % 根据距离设置定义其余两个信标和轨迹原点，dxyz和纬度经度rrm
% dxyz=[0,-5*sqrt(3),0;
%     -10,5*sqrt(3),0;
%     -20,-5*sqrt(3),0;]*1000;
% % 分开获取信标和轨迹原点
% rrm=dxyz2pos(dxyz,beacon0');
% ddm=r2d(rrm(:,1:2));
% beaconxyz=dxyz(1:3,:);
% beaconrrm=rrm(1:3,:);
% beaconddm=ddm(1:3,:);
%
% 定义参考原点/第一个信标的绝对位置，单位为度 [纬度, 经度, 高度]
beacon0 = d2r([36.40042003, 120.68981831, 0]);
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
    % 创建新的图窗来绘制距离曲线
    figure;
    plot(GNSS_1s(:,1), distances(:,i), 'LineWidth', 1.5); % 洋红色实线
    xlabel('时间 (s) ');
    ylabel('距离 (km)');
    title('轨迹点到信标的距离');
    grid on;
end
distances_N_by_1_max = max(distances, [], 2);
% distances=distances+normrnd(0,15,size(distances));
%%
beacon1=ones(length(distances),3)*diag(beaconrrm(1,:));
beacon2=ones(length(distances),3)*diag(beaconrrm(2,:));
beacon3=ones(length(distances),3)*diag(beaconrrm(3,:));
range1=[GNSS_1s(:,1),distances(:,1),distances(:,1),beacon1];
range2=[GNSS_1s(:,1),distances(:,2),distances(:,2),beacon2];
range3=[GNSS_1s(:,1),distances(:,3),distances(:,3),beacon3];

output_file="D:\Github\KF-GINS-Matlab\惯导实验数据\input\range1.txt";
try
    writematrix(range1, output_file, 'Delimiter', ' ');
    fprintf('height信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
output_file="D:\Github\KF-GINS-Matlab\惯导实验数据\input\range2.txt";
try
    writematrix(range2, output_file, 'Delimiter', ' ');
    fprintf('height信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
output_file="D:\Github\KF-GINS-Matlab\惯导实验数据\input\range3.txt";
try
    writematrix(range3, output_file, 'Delimiter', ' ');
    fprintf('height信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
%% 构造高度数据：直接选择参考数据
height=truth_inte(:,[2,5]);
%% 数据保存：参考数据
output_file="D:\Github\KF-GINS-Matlab\惯导实验数据\input\truth_inte.txt";
try
    writematrix(truth_inte, output_file, 'Delimiter', ' ');
    fprintf('合成导航信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
%% 数据保存：gnss
output_file="D:\Github\KF-GINS-Matlab\惯导实验数据\input\GNSS_1s.txt";
try
    writematrix(GNSS_1s, output_file, 'Delimiter', ' ');
    fprintf('GNSS_1s信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end

output_file="D:\Github\KF-GINS-Matlab\惯导实验数据\input\GNSS_01s.txt";
try
    writematrix(GNSS_01s, output_file, 'Delimiter', ' ');
    fprintf('GNSS_1s信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
output_file="D:\Github\KF-GINS-Matlab\惯导实验数据\input\gnss_ref_01s.txt";
try
    writematrix(GNSS_ref_01s, output_file, 'Delimiter', ' ');
    fprintf('GNSS_1s信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
%% 数据保存：高度数据
output_file="D:\Github\KF-GINS-Matlab\惯导实验数据\input\height.txt";
try
    writematrix(height, output_file, 'Delimiter', ' ');
    fprintf('height信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
%%
plot_result('惯导实验数据/input/pva_430.txt','full')
plot_result('惯导实验数据/input/pva_830.txt','full')
plot_result('惯导实验数据/input/pva_RS.txt','full')
%%
calc_error('惯导实验数据/input/pva_430.txt','惯导实验数据/input/truth_inte.txt')
ylim([0,0.5])
calc_error('惯导实验数据/input/pva_RS.txt','惯导实验数据/input/truth_inte.txt')
ylim([0,5])
%% 仿真距离信息
