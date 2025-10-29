% 定义轨迹：起点、中间点、终点（经纬度）
glvs
trajectory = d2r([
    30, 120,0 ;   % 起点
    30.1,120.1,0; % 中间点
    30.2,120.2,0   % 终点
])';

% 新终点位置
newEndPoint = d2r([30.3,120.3,0])';

% 旋转轨迹
rotatedTraj = rotateTrajectory(trajectory, newEndPoint);

% 显示结果
disp('原始轨迹:');
disp(trajectory);
disp('旋转后轨迹:');
disp(rotatedTraj);
figure
plot(trajectory(2,:),trajectory(1,:))
hold on
plot(rotatedTraj(2,:),rotatedTraj(1,:))