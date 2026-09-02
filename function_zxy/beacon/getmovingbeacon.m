function getmovingbeacon(path,type)
glvs
% MEMS
truth = importdata([path,'/input','/pva_830.txt']);
% num = floor(1/(truth(2,2)-truth(1,2)));
num = 100;
truth1 = importdata([path,'/input','/truth.nav']);
output_file=[path,'\input','\range_moving.txt',];

% 光纤

% truth = importdata([path,'/pva_830.txt']);
% num = 100;
% truth11 = importdata([path,'/truth.nav']);
% output_file=[path,'\range_moving.txt',];


GNSS_1s = truth(num:num:end,2:5);% 时间间隔
orgin0 =[d2r(GNSS_1s(1,2:3)),GNSS_1s(1,4)] ;
ts = 1;

if type=="circle"
    dxyz_orgin0= [0, 0, 0];
    orgin0_bea = dxyz2pos(dxyz_orgin0, orgin0');
    % 圆形轨迹
    R=1000;
    V=1.5;
    N=1;
    trjbea.avp = circle(orgin0_bea',R,V,N,pi/4+pi);
elseif type == "follow"
    dxyz_orgin0 = [0, 0, 0];
    orgin0_bea = dxyz2pos(dxyz_orgin0, orgin0')';
    N = length(GNSS_1s);
    mid = floor(N/2);
    base_offset = [0, 0, 0];
    k = 1; % 轨迹缩放比例
    for ii = 1:N
        % 1. 计算当前 GNSS 点相对于起点的位移 (Delta NEU)
        delta_neu = pos2dxyz([d2r(GNSS_1s(ii,2:3)), GNSS_1s(ii,4)], orgin0_bea);
        % 2. 缩放位移 
        scaled_move = delta_neu * k;
        % 3. 计算“掉队”逻辑 (Offset)
        % if ii <= mid
        %     current_drift = (ii-1) * [0.5, 0.5, 0];
        % else
        %     max_drift = (mid-1) * [0.5, 0.5, 0];
        %     current_drift = max_drift + (ii-mid) * [-0.5, -0.5, 0];
        % end
        current_drift=[-500,-500,0];
        % 4. 最终位置 = 起点 + 缩放后的位移 + 基础偏差 + 掉队偏差
        final_offset = scaled_move + base_offset + current_drift;

        trjbea.avp(ii,7:9) = dxyz2pos(final_offset, orgin0_bea);


    end
    myfigurestartup(5,5,'prese')
    plot(d2r(GNSS_1s(:,3)),d2r(GNSS_1s(:,2)))
    hold on
    plot(trjbea.avp(:,8),trjbea.avp(:,7))

else
    if type=="trjsquare"
        % 方形轨迹
        dxyz_orgin0 = [0, 0, 0];
        orgin0_bea = dxyz2pos(dxyz_orgin0, orgin0')';
        avp0 = [[0;0;d2r(90)]; [0;0;0]; orgin0_bea'];
        xxx = [];
        seg = trjsegment(xxx, 'init',         0);
        seg = trjsegment(seg, 'accelerate',   10, xxx, 0.1);
        seg = trjsegment(seg, 'uniform',    length(truth)/num/8);
        seg = trjsegment(seg, 'turnleft', 90, 1);
        seg = trjsegment(seg, 'uniform',    length(truth)/num/5);
        seg = trjsegment(seg, 'turnright', 90, 1);
        seg = trjsegment(seg, 'uniform',      length(truth)/num/5);
        seg = trjsegment(seg, 'turnright', 90, 1);
        seg = trjsegment(seg, 'uniform',      length(truth)/num/5);
        seg = trjsegment(seg, 'turnright', 90, 1);
        seg = trjsegment(seg, 'uniform',      length(truth)/num/5);
    elseif type=="trjlike"
        dxyz_orgin0 = [0, 0, 0];
        orgin0_bea = dxyz2pos(dxyz_orgin0, orgin0')';
        avp0 = [[0;0;d2r(-45)]; [0;0;0]; orgin0_bea'];
        xxx = [];
        seg = trjsegment(xxx, 'init',         0);
        seg = trjsegment(seg, 'uniform',      20); % 保持原来的状态不变
        seg = trjsegment(seg, 'accelerate',   10, xxx, 0.1); % 加速
        seg = trjsegment(seg, 'uniform',      500);
        seg = trjsegment(seg, 'turnright', 27, 5);
        seg = trjsegment(seg, 'uniform',      1000);
        seg = trjsegment(seg, 'turnleft', 27, 5);
        seg = trjsegment(seg, 'uniform',      1000);
        seg = trjsegment(seg, 'turnright', 27, 5);
        seg = trjsegment(seg, 'uniform',      500);
        seg = trjsegment(seg, 'turnleft', 27, 5);
        seg = trjsegment(seg, 'uniform',     500);

    elseif type=="square"
        % 方形轨迹
        dxyz_orgin0 = [0, 0, 0];
        orgin0_bea = dxyz2pos(dxyz_orgin0, orgin0')';
        avp0 = [[0;0;d2r(140)]; [0;0;0]; orgin0_bea'];
        xxx = [];
        seg = trjsegment(xxx, 'init',         0);
        seg = trjsegment(seg, 'accelerate',   10, xxx, 0.1);
        seg = trjsegment(seg, 'uniform',    length(truth)/num/4);
        seg = trjsegment(seg, 'turnright', 90, 1);
        seg = trjsegment(seg, 'uniform',      length(truth)/num/4);
        seg = trjsegment(seg, 'turnright', 90, 1);
        seg = trjsegment(seg, 'uniform',      length(truth)/num/4);
        seg = trjsegment(seg, 'turnright', 90, 1);
        seg = trjsegment(seg, 'uniform',      length(truth)/num/4);
    elseif type=="line"
        % 直线轨迹
        dxyz_orgin0 = [0, 0, 0];
        orgin0_bea = dxyz2pos(dxyz_orgin0, orgin0')';
        avp0 = [[0;0;d2r(70)]; [0;0;0]; orgin0_bea'];
        xxx = [];
        seg = trjsegment(xxx, 'init',         0);
        seg = trjsegment(seg, 'accelerate',   10, xxx, 0.3);
        seg = trjsegment(seg, 'uniform',    length(truth)/num/2);
        seg = trjsegment(seg, 'deaccelerate',   10, xxx, 0.6);
        seg = trjsegment(seg, 'uniform',    length(truth)/num/2);
    elseif type=="plan"
        % 规划轨迹1： 0.99s效果很好，1s效果很差
        % avp0 = [[0;0;d2r(-30)]; [0;0;0]; orgin0_bea'];
        % xxx = [];
        % seg = trjsegment(xxx, 'init',         0);
        % seg = trjsegment(seg, 'accelerate',   10, xxx, 0.25);
        % seg = trjsegment(seg, 'uniform',    length(truth)/num/6);
        % seg = trjsegment(seg, 'turnleft', 16, 5);
        % seg = trjsegment(seg, 'uniform',    length(truth)/num/6);
        % seg = trjsegment(seg, 'turnright', 16, 5);
        % seg = trjsegment(seg, 'uniform',    length(truth)/num/6);
        % seg = trjsegment(seg, 'turnright', 45, 4);
        % seg = trjsegment(seg, 'uniform',    length(truth)/num/6);
        % seg = trjsegment(seg, 'turnleft', 16, 5);
        % seg = trjsegment(seg, 'uniform',    length(truth)/num/6);
        % seg = trjsegment(seg, 'turnright', 16, 5);
        % seg = trjsegment(seg, 'uniform',    length(truth)/num/6);

        dxyz_orgin0= [-2000, 4000, 0];
        orgin0_bea = dxyz2pos(dxyz_orgin0, orgin0');
        avp0 = [[0;0;d2r(-90)]; [0;0;0]; orgin0_bea'];
        xxx = [];
        seg = trjsegment(xxx, 'init',         0);
        seg = trjsegment(seg, 'uniform',    585);
        seg = trjsegment(seg, 'uniform',    380);
        seg = trjsegment(seg, 'accelerate',   10, xxx, 0.25);
        seg = trjsegment(seg, 'uniform',    400);
        seg = trjsegment(seg, 'turnright', 45, 4);
        seg = trjsegment(seg, 'uniform',    400);
        seg = trjsegment(seg, 'uniform',    380);
        seg = trjsegment(seg, 'uniform',    585);
    end
    trjbea= trjsimu(avp0, seg.wat, ts, 1);
end
% insplot(trjbea.avp)
beaconrrm = trjbea.avp(:,7:9);
beaconxyz = pos2dxyz(beaconrrm,orgin0');

trj = GNSS_1s(:, 2:4);
trj(:,1:2)=d2r(trj(:,1:2));
trajectory_xyz = pos2dxyz(trj, orgin0');

% 绘图
% figure
% beaconddm = ddm;
% trajectory_ddm = GNSS_1s(:, 2:4);
% plot_trajectory_and_movingbeacons(trajectory_xyz/1000, beaconxyz, beaconddm, trajectory_ddm)
trajectory_x = trajectory_xyz(:, 1);
trajectory_y = trajectory_xyz(:, 2);

beacon_x = beaconxyz(1:length(trajectory_xyz),1);
beacon_y = beaconxyz(1:length(trajectory_xyz),2);

distances(:) = sqrt((trajectory_x - beacon_x).^2 + ...
    (trajectory_y - beacon_y).^2);

range1 = [GNSS_1s(:,1),distances',distances',beaconrrm(1:length(trajectory_xyz),:)];

%% 计算方位角

att = truth1(num:num:end,9:11);
for i=1:length(range1)
    range1(i,7) = pos2azimuth(trajectory_xyz(i,1:2),beaconxyz(i,1:2),att(i,end));
end

% 
% for i=1:length(range1)
%     my_pos(i,:) = calc_position_from_beacon(beaconxyz(i,1:2), range1(i,3)+randn*2, range1(i,7)+randn*0.2, att(i,end)+randn*0.2);
% end
% % myfigurestartup(7,7,'prese')
% % plot(my_pos(:,1),my_pos(:,2))
% % hold on
% % plot(trajectory_xyz(:,1),trajectory_xyz(:,2))
% % plot(beaconxyz(:,1),beaconxyz(:,2))
% % myfigurestartup(7,7,'prese')
% % subplot 121
% % plot(my_pos(:,1)-trajectory_xyz(:,1))
% % subplot 122
% % plot(my_pos(:,2)-trajectory_xyz(:,2))
% 
% pos_result = dxyz2pos([my_pos(:,1:2),zeros(length(my_pos),1)],orgin0');
% range1(:,8:10) = pos_result;
% 


try
    writematrix(range1, output_file, 'Delimiter', ' ');
    fprintf('信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
end


function avp_circle = circle(pos0,R,V,N,deg)
%CIRCLE_SEG 根据输入的半径R和速度V，以及圈数N，以及参考初始纬经度pos0，产生圆周轨迹
T = 2*pi*R/V;omega = V/R;
t = 0:1:N*T;l = length(t);
theta = deg + omega * t;
x = R * cos(theta);
y = R * sin(theta);
vx = -V * sin(theta);
vy = V * cos(theta);
yaw = mod(theta,2*pi);
yaw(yaw>pi) = yaw(yaw>pi)-2*pi;
phi=[zeros(l,2),yaw'];
pos = dxyz2pos([x',y',zeros(l,1)], pos0);
avp_circle=[phi,vx',vy',zeros(l,1),pos,t'];
end



function trj_rotated_rad = rotate_trajectory_rad(trj_rad, center_rad, angle_rad)
% 输入:
% trj_rad:    Nx3 矩阵 [lat(rad), lon(rad), alt(m)]
% center_rad: 1x3 向量 [lat(rad), lon(rad), alt(m)]，作为旋转中心
% angle_rad:  旋转角度 (rad)，正值为逆时针

% 1. 将轨迹点转换为以旋转中心为原点的平面坐标 (Local NEU)
% 使用 PSINS 里的 pos2dxyz 函数
% trajectory_xyz: [North, East, Down/Up] (取决于你的坐标系定义，通常为东北天)
xyz = pos2dxyz(trj_rad, center_rad');

% 2. 构造二维旋转矩阵 (绕天轴旋转，即在 NE 平面上旋转)
cosA = cos(angle_rad);
sinA = sin(angle_rad);
R = [cosA, -sinA;
    sinA,  cosA];

% 3. 执行旋转
% xyz(:,1:2) 提取的是 North 和 East 坐标
% 注意：MATLAB 中 xyz 是 Nx3，我们将前两列转置进行矩阵运算
xy_rotated = (R * xyz(:, 1:2)')';

% 4. 保持高度不变，更新坐标
xyz_new = xyz;
xyz_new(:, 1:2) = xy_rotated;

% 5. 将旋转后的平面坐标转换回经纬度 (rad)
% 使用 PSINS 里的 dxyz2pos 函数
trj_rotated_rad = dxyz2pos(xyz_new, center_rad');
end