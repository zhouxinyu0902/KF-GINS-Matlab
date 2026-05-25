function rangedataget(path)
truth = importdata([path,'\truth.nav']);
GNSS_1s=truth(1:100:end,2:5);
%% 根据GNSS数据构造距离数据
glvs
orgin0 = d2r(GNSS_1s(1,2:4));
% 原始等边三角形坐标（单位：米）
dxyz_original = [0, -5*sqrt(3), 0;
                -10, 5*sqrt(3), 0;
                10, 5*sqrt(3), 0;] * 1000;
% 定义旋转角度（15度）
theta_deg = -20;
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
rrm = dxyz2pos(dxyz, orgin0');
ddm = r2d(rrm(:, 1:2));
beaconxyz = dxyz(1:3, :);
beaconrrm = rrm(1:3, :);
beaconddm = ddm(1:3, :);

trj=GNSS_1s(:, 2:4);
trj(:,1:2)=d2r(trj(:,1:2));
trajectory_xyz = pos2dxyz(trj, orgin0');
trajectory_ddm=GNSS_1s(:, 2:4);

% 绘图
plot_trajectory_and_beacons_m(trajectory_xyz, beaconxyz)
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

output_file=[path,'/range1.txt'];
try
    writematrix(range1, output_file, 'Delimiter', ' ');
    fprintf('height信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
output_file=[path,'/range2.txt'];
try
    writematrix(range2, output_file, 'Delimiter', ' ');
    fprintf('height信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
output_file=[path,'/range3.txt'];
try
    writematrix(range3, output_file, 'Delimiter', ' ');
    fprintf('height信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
