clear;
close all;

%% 路径与配置
script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(script_dir);
project_root = fileparts(topic_dir);

cfg = patent_Configsimu(false);
if ~exist(cfg.inputfolder, 'dir')
    mkdir(cfg.inputfolder);
end
if ~exist(cfg.simfigurefolder, 'dir')
    mkdir(cfg.simfigurefolder);
end

% 模拟直线轨迹与三个潜标的位置
%% 定信标位置和轨迹初始点
glvs
% 定义参考原点/第一个信标的绝对位置，单位为度 [纬度, 经度, 高度]
% beacon0=d2r([17,117,0]);
beacon0=d2r(cfg.sim.beacon_origin_deg);
% 117.791, 17.576
% 根据距离设置定义其余两个信标和轨迹原点，dxyz和纬度经度rrm
dxyz_original = cfg.sim.layout_m;

% 定义旋转角度（15度）
theta_deg = cfg.sim.rotation_deg;
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
rrm=dxyz2pos(dxyz,beacon0');
ddm=r2d(rrm(:,1:2));
beaconxyz=dxyz(1:3,:);
beaconrrm=rrm(1:3,:);
beaconddm=ddm(1:3,:);
id=cfg.sim.start_point_index;
pos0xyz=dxyz(id,:);
pos0rrm=rrm(id,:);
pos0ddm=ddm(id,:);


%% 生成轨迹
ts = cfg.sim.sample_interval;
avp0 = [[0;0;d2r(cfg.sim.initial_heading_deg)]; [0;0;0]; pos0rrm'];
xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      20);
seg = trjsegment(seg, 'accelerate',   10, xxx, 0.20576); 
seg = trjsegment(seg, 'uniform',      1000); 
seg = trjsegment(seg, 'turnright', 15, 170/15);
seg = trjsegment(seg, 'uniform',      1000);
seg = trjsegment(seg, 'turnleft', 15, 170/15);
seg = trjsegment(seg, 'uniform',      1000);
seg = trjsegment(seg, 'turnright', 15, 170/15);
seg = trjsegment(seg, 'uniform',      1000);
seg = trjsegment(seg, 'turnleft', 15, 170/15);
seg = trjsegment(seg, 'uniform',      1000);
seg = trjsegment(seg, 'turnright', 15, 170/15);
seg = trjsegment(seg, 'uniform',      1000);
seg = trjsegment(seg, 'turnleft', 15, 170/15);
seg = trjsegment(seg, 'uniform',      1000);
seg = trjsegment(seg, 'turnright', 15, 170/15);
seg = trjsegment(seg, 'uniform',      1000);
seg = trjsegment(seg, 'turnleft', 15, 170/15);
seg = trjsegment(seg, 'uniform',      1000);
seg = trjsegment(seg, 'deaccelerate',   10, xxx, 0.20576); 


% seg = trjsegment(xxx, 'init',         0);
% seg = trjsegment(seg, 'uniform',      20);
% seg = trjsegment(seg, 'accelerate',   10, xxx, 0.20576); 
% seg = trjsegment(seg, 'uniform',      3600*1.35); 
% seg = trjsegment(seg, 'turnleft',  60, 3);
% seg = trjsegment(seg, 'uniform',      3600*1.35); 
% seg = trjsegment(seg, 'deaccelerate',   10, xxx, 0.20576); 

trj = trjsimu(avp0, seg.wat, ts, 1); % 只需要位置和姿态信息就可以
[nn, ts, nts] = nnts(1, trj.ts);
sum(trj.wat(:,1))
% 轨迹与参考点绘图
% 除以 1000 将米转换为公里。
trajectory_xyz_km = pos2dxyz(trj.avp(:,7:9), beacon0') / 1000;
trajectory_ddm=trj.avp(:,7:9);
trajectory_ddm(:,1:2)=r2d(trajectory_ddm(:,1:2));
%% 绘图
% plot_trajectory_and_beacons(trajectory_xyz_km, beaconxyz, beaconddm, trajectory_ddm)
plot_trajectory_and_beacons_m(trajectory_xyz_km*1000, beaconxyz)
trajectory_fig = gcf;
saveFigurePair(trajectory_fig, cfg.simfigurefolder, ...
    'simulation-trajectory-and-beacons', cfg.figure);
%% 计算信标与轨迹中每一个点的距离并绘图
beacon_xyz_km = beaconxyz/1000;
% 获取轨迹点的坐标
trajectory_x = trajectory_xyz_km(:, 1);
trajectory_y = trajectory_xyz_km(:, 2);
trajectory_z = trajectory_xyz_km(:, 3); % 如果需要3D距离
distances_km = zeros(size(trajectory_xyz_km, 1), 3);
for i=1:3
    % 获取信标的坐标 (东向，北向，天向)
    beacon1_x = beacon_xyz_km(i, 1);
    beacon1_y = beacon_xyz_km(i, 2);
    beacon1_z = beacon_xyz_km(i, 3); % 如果需要3D距离

    % 计算每个轨迹点到第一个信标的距离
    % 这里计算的是3D欧氏距离，如果只需要2D水平距离，请注释掉z分量
    distances_km(:,i) = sqrt((trajectory_x - beacon1_x).^2 + ...
        (trajectory_y - beacon1_y).^2 + ...
        (trajectory_z - beacon1_z).^2);
    % 创建新的图窗来绘制距离曲线
    distance_fig = figure;
    plot(trj.avp(:,10), distances_km(:,i), 'LineWidth', 1.5); % 洋红色实线
    xlabel('时间 (s) ');
    ylabel('距离 (km)');
    title('轨迹点到信标的距离');
    grid on;
    saveFigurePair(distance_fig, cfg.simfigurefolder, ...
        sprintf('simulation-range-b%02d', i), cfg.figure);
end
distances_N_by_1_max = max(distances_km, [], 2);


%% 初始化
avp_ref = trj.avp;
avp0_ref = avp_ref(1,1:9);
avp0_err = avperrset([0.01,0.01,0.05]*60,0.1,1);
avp0_use = avpadderr(avp0_ref,avp0_err);

imu_ref = trj.imu;
% 设置IMU误差
eb = cfg.sim.imu_error.eb;
db = cfg.sim.imu_error.db;
web = cfg.sim.imu_error.web;
wdb = cfg.sim.imu_error.wdb;
rng(cfg.randomseed);
imu_err = imuerrset(eb, db, web, wdb); 
imu_use = imuadderr(imu_ref, imu_err);

%% 保存数据
IMUFRD=imuRFU2FRD(imu_use);
path = cfg.imufilepath;
fp = fopen(path,'wt');
fprintf(fp, '%.10f %.10f %.10f %.10f %.10f %.10f %.10f \n', IMUFRD');
fclose(fp);

pva_ref=avpENU2NED(avp_ref);
path=cfg.truthpath;
fp=fopen(path,'wt');
fprintf(fp, '%.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f %.10f\n', pva_ref');
fclose(fp);


bcn=repmat(beaconrrm(1,:),size(distances_km(:,1)));
range_m=distances_km(:,1)*1000;
range_save1=[avp_ref(:,end),range_m,range_m,bcn];
path=cfg.rangefile1path;
fp=fopen(path,'wt');
fprintf(fp, '%.10f %.10f %.10f %.10f %.10f %.10f\n', range_save1');
fclose(fp);

bcn=repmat(beaconrrm(2,:),size(distances_km(:,2)));
range_m=distances_km(:,2)*1000;
range_save2=[avp_ref(:,end),range_m,range_m,bcn];

path=cfg.rangefile2path;
fp=fopen(path,'wt');
fprintf(fp, '%.10f %.10f %.10f %.10f %.10f %.10f\n', range_save2');
fclose(fp);

bcn=repmat(beaconrrm(3,:),size(distances_km(:,3)));
range_m=distances_km(:,3)*1000;
range_save3=[avp_ref(:,end),range_m,range_m,bcn];
path=cfg.rangefile3path;
fp=fopen(path,'wt');
fprintf(fp, '%.10f %.10f %.10f %.10f %.10f %.10f\n', range_save3');
fclose(fp);

%% Local functions
function saveFigurePair(fig, output_folder, basename, figure_cfg)
    if ~isgraphics(fig, 'figure')
        warning('Cannot save figure %s because its handle is invalid.', basename);
        return;
    end
    if ~exist(output_folder, 'dir')
        mkdir(output_folder);
    end

    if figure_cfg.save_fig
        savefig(fig, fullfile(output_folder, [basename, '.fig']));
    end
    if figure_cfg.save_png
        png_path = fullfile(output_folder, [basename, '.png']);
        if exist('exportgraphics', 'file') == 2
            exportgraphics(fig, png_path, ...
                'Resolution', figure_cfg.png_resolution);
        else
            print(fig, png_path, '-dpng', ...
                sprintf('-r%d', figure_cfg.png_resolution));
        end
    end
end
