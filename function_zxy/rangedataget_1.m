function rangedataget_1(path)
%% 不是三个信标，重新设置单信标或者其他情况
truth = importdata([path,'/truth.nav']);
GNSS_1s=truth(1:100:end,2:5);
%% 根据GNSS数据构造距离数据
t=length(GNSS_1s);
glvs
orgin0 = d2r(GNSS_1s(1,2:4));
dxyz_orgin0= [5, 5, 0] * 1000;
orgin0_bea = dxyz2pos(dxyz_orgin0, orgin0');
% ts = 1;  
% avp0 = [[0;0;d2r(-80)]; [0;0;0]; orgin0_bea']; 
% xxx = [];
% seg = trjsegment(xxx, 'init',         0);
% seg = trjsegment(seg, 'accelerate',   10, xxx, 0.5); 
% seg = trjsegment(seg, 'uniform',      t/4);
% seg = trjsegment(seg, 'turnright', 90, 1);
% seg = trjsegment(seg, 'uniform',      t/4);
% seg = trjsegment(seg, 'turnright', 90, 1);
% seg = trjsegment(seg, 'uniform',      t/4);
% seg = trjsegment(seg, 'turnright', 90, 1);
% seg = trjsegment(seg, 'uniform',      t/4);
% seg = trjsegment(seg, 'turnright', 90, 1);
% 
% trjbea = trjsimu(avp0, seg.wat, ts, 1); % 只需要位置和姿态信息就可以
% 

refavp=pvaNED2ENU(truth);
ts = 1;  
avp0 = [[0;0;d2r(40)]; [0;0;0]; orgin0_bea']; 
xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'accelerate',   10, xxx, 0.3); 
seg = trjsegment(seg, 'uniform',    length(refavp)/100/2);
seg = trjsegment(seg, 'deaccelerate',   10, xxx, 0.6); 
seg = trjsegment(seg, 'uniform',    length(refavp)/100/2);
trjbea = trjsimu(avp0, seg.wat, ts, 1); % 只需要位置和姿态信息就可以

sum(trjbea.wat(:,1))


% 分开获取信标和轨迹原点
rrm = trjbea.avp(:,7:9);
ddm = r2d(rrm(:, 1:2));
beaconxyz = pos2dxyz(rrm,orgin0');
beaconddm = ddm(1:3, :);

trj=GNSS_1s(:, 2:4);
trj(:,1:2)=d2r(trj(:,1:2));
trajectory_xyz = pos2dxyz(trj, orgin0');
trajectory_ddm=GNSS_1s(:, 2:4);

% 绘图
plot_trajectory_and_movingbeacons(trajectory_xyz/1000, beaconxyz, beaconddm, trajectory_ddm)
% plot_trajectory_and_beacons(trajectory_xyz/1000, beaconxyz, beaconddm, trajectory_ddm)
trajectory_x = trajectory_xyz(:, 1);
trajectory_y = trajectory_xyz(:, 2);
distances= sqrt((trajectory_x - beaconxyz(1:length(trajectory_x),1)).^2 + ...
        (trajectory_y - beaconxyz(1:length(trajectory_x),2)).^2);

%%
range=[GNSS_1s(:,1),distances(:,1),distances(:,1),rrm(1:length(trajectory_x),:)];

output_file=[path,'/range_moving.txt'];
try
    writematrix(range, output_file, 'Delimiter', ' ');
    fprintf('信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end

