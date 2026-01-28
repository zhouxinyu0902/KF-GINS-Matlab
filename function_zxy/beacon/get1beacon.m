function get1beacon(path,dxyz)
glvs
truth = importdata([path,'/input','/truth.nav']);
% num = floor(1/(truth(2,2)-truth(1,2)));
num = 100;
att = truth(num:num:end,9:11);
GNSS_1s = truth(num:num:end,2:5);% 时间间隔
orgin0 = d2r(GNSS_1s(1,2:4));

% 分开获取信标和轨迹原点
rrm = dxyz2pos(dxyz, orgin0');
ddm = r2d(rrm(1:2));
beaconxyz = dxyz;
beaconrrm = rrm;
beaconddm = ddm;
trj = GNSS_1s(:, 2:4);
trj(:,1:2) = d2r(trj(:,1:2));
trajectory_xyz = pos2dxyz(trj, orgin0');
trajectory_ddm=GNSS_1s(:, 2:4);
% 绘图
% plot_trajectory_and_beacons(trajectory_xyz/1000, beaconxyz, beaconddm, trajectory_ddm)
trajectory_x = trajectory_xyz(:, 1);
trajectory_y = trajectory_xyz(:, 2);
% 获取信标的坐标 (东向，北向，天向)
beacon1_x = beaconxyz(1);
beacon1_y = beaconxyz(2);
% 计算每个轨迹点到第一个信标的2D距离
distances(:) = sqrt((trajectory_x - beacon1_x).^2 + ...
    (trajectory_y - beacon1_y).^2);
% % 创建新的图窗来绘制距离曲线
% figure;
% plot(GNSS_1s(:,1), distances(:,i), 'LineWidth', 1.5); % 洋红色实线
% xlabel('时间 (s) ');
% ylabel('距离 (km)');
% title('轨迹点到信标的距离');
% grid on;

%
beacon1=ones(length(distances),3)*diag(beaconrrm);
range1=[GNSS_1s(:,1),distances',distances',beacon1];
%% 计算方位角
for i=1:length(range1)
    range1(i,7) = pos2azimuth(trajectory_xyz(i,1:2),beaconxyz(1:2),att(i,end));
end
%%
for i=1:length(range1)
    my_pos(i,:) = calc_position_from_beacon(beaconxyz(1:2), range1(i,3)+randn*2, range1(i,7)+randn*0.2, att(i,end)+randn*0.2);
end
% myfigurestartup(7,7,'prese')
% plot(my_pos(:,1),my_pos(:,2))
% hold on
% plot(trajectory_xyz(:,1),trajectory_xyz(:,2))
% plot(beaconxyz(:,1),beaconxyz(:,2))
% myfigurestartup(7,7,'prese')
% subplot 121
% plot(my_pos(:,1)-trajectory_xyz(:,1))
% subplot 122
% plot(my_pos(:,2)-trajectory_xyz(:,2))

pos_result = dxyz2pos([my_pos(:,1:2),zeros(length(my_pos),1)],orgin0');
range1(:,8:10) = pos_result;
output_file=[path,'/input','/single_range.txt'];
try
    writematrix(range1, output_file, 'Delimiter', ' ');
    fprintf('信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end

end