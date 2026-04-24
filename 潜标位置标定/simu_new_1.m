clear 
close all
%%
load('data.mat')
trj_square.avp = trj.avp(200001:end,:);
%%
stations = generate_equilateral_stations(20000);
stations = [stations,[-1000;-1045;-900]];
glvs
posbeacon = dxyz2pos(stations,trj_square.avp(1,7:9)');

%%
figure;hold on
for i=1:3
    plot(posbeacon(i,1),posbeacon(i,2),'*')
end
plot(trj_square.avp(:,7),trj_square.avp(:,8))
%%
xyztrj = pos2dxyz(trj_square.avp(:,7:9),trj_square.avp(1,7:9)');
%% 偏移gnss位置
% 假设 stations 是 3x3 矩阵 [x, y, z]
% theta: 倾角 (与垂直线的夹角)
% phi:   方位角 (在 XY 平面的旋转角)

theta = deg2rad(15); % 倾斜 15 度
phi = deg2rad(45);   % 向东北 45 度方向偏移
% 1. 计算水平偏移量半径 (基于深度 stations(:,3))
dR = abs(stations(:,3)) .* tan(theta);
% 2. 利用方位角 phi 分解到 x 和 y 轴
dx = dR .* cos(phi);
dy = dR .* sin(phi);
% 3. 计算 GNSS 点坐标 (发声点坐标 + 偏移量)
stations_gnss = zeros(size(stations));
stations_gnss(:,1) = stations(:,1) + dx; % 在原位置基础上累加偏移
stations_gnss(:,2) = stations(:,2) + dy;
stations_gnss(:,3) = 0; % GNSS 在水面，深度为 0
% --- 可视化 ---
figure; hold on; grid on;
% 绘制海底发声点 (o)
plot3(stations(:,1), stations(:,2), stations(:,3), 'ro', 'MarkerSize', 8, 'DisplayName', '海底发声点');
% 绘制水面 GNSS 点 (*)
plot3(stations_gnss(:,1), stations_gnss(:,2), stations_gnss(:,3), 'b*', 'MarkerSize', 8, 'DisplayName', '水面GNSS点');
% 绘制连接线（模拟缆绳）
for i = 1:size(stations,1)
    plot3([stations(i,1), stations_gnss(i,1)], ...
          [stations(i,2), stations_gnss(i,2)], ...
          [stations(i,3), stations_gnss(i,3)], 'k--');
end
legend;
view(3);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('潜标倾斜偏移示意图');

%% 实际水平距离计算
for i=1:3
    R(:,i) = ((xyztrj(:,1)-stations(i,1)).^2+(xyztrj(:,2)-stations(i,2)).^2).^0.5;
end
R_noised = R(1:100*20:end,:)+normrnd(0,6,size(R(1:100*20:end,:)));
%　长基线解算
for i=1:length(R_noised)
    pos(i,:) = LBL_Positioning(R_noised(i,:), stations_gnss);
end
% 画图
figure,hold on 
plot(pos(:,1),pos(:,2),'.')
plot(xyztrj(1:100*20:end,1),xyztrj(1:100*20:end,2),'*')

