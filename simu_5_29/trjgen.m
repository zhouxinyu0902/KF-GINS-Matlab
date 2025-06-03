clear all
%% 仿真轨迹
glvs
avp0 = [[0;0;d2r(45)]; [0;0;0]; [d2r([15;115]);-1200]];
ts=0.01;
xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      20); % 保持原来的状态不变
seg = trjsegment(seg, 'accelerate',   5, xxx, 0.3); % 加速
seg = trjsegment(seg, 'uniform',      1200);
trj= trjsimu(avp0, seg.wat, ts, 1); 
%%
myfigurestartup(10,10,'prese')
insplot(trj.avp)
sum(trj.wat(:,1))
%% 仿真imu数据,增加噪声
eb=0.027; % cfg.gyrbiasstd = 0.027; % [deg/h]
db=15/0.98; % cfg.accbiasstd = 15; % [mGal]
web=0.003; % cfg.gyrarw = 0.003; % [deg/sqrt(h)]
wdb=0.03*1e6/(60*9.8); % cfg.accvrw = 0.03; % [m/s/sqrt(h)]

% eb=2;db=0.36*980;web=0.15;wdb=200/9.8;
% eb=0.1;db=0.1;web=0.05;wdb=10;
rng(1);
imuerr = imuerrset(eb, db, web, wdb); %% adi-16465
trjimu= imuadderr(trj.imu, imuerr);
%% 坐标系转换
pva_ref = avpENU2NED(trj.avp);
pva0 = pva_ref(1,:);
IMUFRD = imuRFU2FRD(trjimu);
%% 初始化
pos0 = avp0(7:9);
laststate.pos = avp0(7:9);
laststate.vel = pva0(6:8)';
laststate.att = pva0(9:11)'/180*pi;
laststate.qbn = euler2quat(laststate.att);
laststate.cbn = euler2dcm(laststate.att);
laststate.gyrbias = zeros(3,1);
laststate.accbias = zeros(3,1);
laststate.gyrscale = zeros(3,1);
laststate.accscale = zeros(3,1);
param = Param();
[laststate.Rm, laststate.Rn] = getRmRn(laststate.pos(1), param);
laststate.gravity = getGravity(laststate.pos);

xyz_true = pos2dxyz(trj.avp(:,7:9),pos0);

% 惯导解算
ll=length(IMUFRD);
[pva, xyz_ins, xyz_model]= prealloc(ll-1,11,3,2);
for i=1:ll-1
    lastimu = IMUFRD(i,:)';
    thisimu = IMUFRD(i+1,:)';
    navstate = InsMech(laststate, lastimu, thisimu);
    laststate = navstate;
    pva(i,:)=[0;IMUFRD(i,1);...
    laststate.pos(1:2)/pi*180;laststate.pos(3);laststate.vel;laststate.att/pi*180];    
    xyz_ins(i,:) = pos2dxyz(laststate.pos',pos0);
    if i>1
        xyz_model(i,:)=xyz_ins(i-1,1:2)' + laststate.vel(1:2)*ts;
    end
end
%% 轨迹对比，笛卡尔坐标系
figure,
plot(xyz_ins(:,1),xyz_ins(:,2));hold on;
plot(xyz_model(:,1),xyz_model(:,2));hold on;
plot(xyz_true(:,1),xyz_true(:,2))
%% 定义地址
truthpath='simu_5_29/pva_ref.txt';
whupath='simu_5_29/pva.txt';
psinspath='simu_5_29/pva_psins.txt';
%% 保存结果和参考结果
writematrix(pva,whupath,'Delimiter',' ');
writematrix(pva_ref,truthpath,'Delimiter',' ');
pva_psins = avpENU2NED(avp_ins);
writematrix(pva_psins,psinspath,'Delimiter',' ');
%% 绘图结果
close all
plot_result(truthpath)
plot_result(whupath)
plot_cmp(whupath,truthpath)
%% 计算误差以及绘图
calc_error(whupath,truthpath)
calc_error(psinspath,truthpath)
%% kalman初始化
sigma_lbl_pos = 3; % 米，LBL解算结果的位置标准差
R_kf = eye(2) * sigma_lbl_pos^2; % 卡尔曼滤波器的观测量噪声协方差矩阵
% 状态向量: [x; y; vx; vy]
% 维度: 4x1
x_est = zeros(4, ll);     % 估计的状态
P_est = zeros(4, 4, ll);  % 估计的协方差矩阵

% 初始状态估计 (从真实轨迹的起点稍微加点噪声作为初始值)
initial_pos_noise = 3; % 初始位置误差（m）
initial_vel_noise = 0.1; % 初始速度误差（m/s）
x_est(:, 1) = [randn*initial_pos_noise;
    randn*initial_pos_noise;
    1 + randn*initial_vel_noise;
    1 + randn*initial_vel_noise];

% 初始误差协方差矩阵
P_est(:, :, 1) = diag([10^2, 10^2, 2^2, 2^2]); % 对位置和速度的初始不确定性
dt = ts;
% 状态转移矩阵 F
F = [1 0 dt 0;
    0 1 0 dt;
    0 0 1 0;
    0 0 0 1];

% 观测矩阵 H
% 观测量是位置 (x, y)，所以H只提取状态向量中的位置分量
H = [1 0 0 0;
    0 1 0 0];

% 过程噪声协方差矩阵 Q 可以通过加速度噪声来建模。
sigma_accel_noise = 0.008; % m/s^2, 假设的加速度噪声标准差
Q = [(dt^3/3)*sigma_accel_noise^2, 0, (dt^2/2)*sigma_accel_noise^2, 0;
    0, (dt^3/3)*sigma_accel_noise^2, 0, (dt^2/2)*sigma_accel_noise^2;
    (dt^2/2)*sigma_accel_noise^2, 0, dt*sigma_accel_noise^2, 0;
    0, (dt^2/2)*sigma_accel_noise^2, 0, dt*sigma_accel_noise^2];


%% 匀速运动模型
for i=2:ll
    % 预测步
    x_pred = F * x_est(:, i-1);
    P_pred = F * P_est(:, :, i-1) * F' + Q;

    % % 更新步
    % pos_imu=pos2dxyz(avp_ins(i,7:9),pos0);
    % z_k = pos_imu(1:2)'; % 当前LBL解算出的位置观测量
    % 
    % y_k = z_k - H * x_pred; % 观测残差
    % S_k = H * P_pred * H' + R_kf; % 观测残差协方差
    % K_k = P_pred * H' * inv(S_k); % 卡尔曼增益
    K_k = zeros(4,2);
    x_est(:, i) = x_pred + K_k * y_k; % 状态更新
    P_est(:, :, i) = (eye(size(F)) - K_k * H) * P_pred; % 误差协方差更新
end
%%
figure,plot(xyz_true(:,1),xyz_true(:,2))
hold on,plot(x_est(1,:),x_est(2,:))
% legend