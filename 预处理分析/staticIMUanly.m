% 仿真产生10min的静止IMU数据
glvs
ts = 0.01;  % 100Hz
avp0 = [[0;0;d2r(-90)]; [0;0;0]; [d2r([36.365;120.6861]);2]]; 
xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      3600*2);
trj = trjsimu(avp0, seg.wat, ts, 1); % 只需要位置和姿态信息就可以
[nn, ts, nts] = nnts(1, trj.ts);
%%
imu_ref = trj.imu;
imu_err_120 = imuerrset(0.01, 10, 0.005, 0.001,0.005, 4,0.001 ,4, 5 , 10, 5, 10, 10, 10, 10);% 120
imu_err_430 = imuerrset(10, 200, 0.1, 0.035*1e5/3600);% 430
imu_err_830 = imuerrset(0.2, 100, 0.03, 0.02*1e5/3600);% 830

% 设置IMU误差
% eb=0.003;
% db=7;
% web=0.0003;
% wdb=2.7778e-05;%1e-6m/s/sqrt(Hz)
% %1e-6*1e5/3600 = 2.7778e-05
% rng(1);
% imu_err = imuerrset(eb, db, web, wdb, web, 4, wdb ,4, 5 , 10, 5, 10, 10, 10, 10); 

imu_120 = imuadderr(imu_ref, imu_err_120);
imu_430 = imuadderr(imu_ref, imu_err_430);
imu_830 = imuadderr(imu_ref, imu_err_830);

%% 进行误差分析
gyro=imu_120(:,1:3);
% 1、计算零偏
dt=0.01;
lat=36.365;
bias(1) = mean(r2d(gyro(:,1))*3600/dt);
% bias(2) = mean(r2d(gyro(:,2))*3600/dt) - (15.041 * cosd(lat));
% bias(3) = mean(r2d(gyro(:,3))*3600/dt) - (15.041 * sind(lat));
% 2、计算零偏稳定性(10s平滑)
fs=100;
smooth_time=10;
points_per_segment = smooth_time * fs;
num_segments = floor(size(gyro(:,1), 1) / points_per_segment);
bias_stability = zeros(1, 3);
segment_means = zeros(num_segments, 3);
data_reshaped = reshape(gyro(1:num_segments*points_per_segment, 1),...
    points_per_segment, num_segments);
segment_means(:, 1) = mean(data_reshaped, 1)';
bias_stability(1) = std(segment_means(:, 1));
convert_to_dph = (180/pi) * 3600; % 转换因子: rad/s -> °/h
bias_stability_dph = bias_stability * convert_to_dph;
% 3、计算零偏稳定性(Allan方差)
% 设置Allan方差计算参数
tau = logspace(-2, 3, 100)'; % 相关时间τ从0.01s到1000s
% 计算Allan方差
[tau_used, adev_x] = allanvar(gyro(:,1), tau, fs);
[~, adev_y] = allanvar(gyro(:,2), tau, fs);
[~, adev_z] = allanvar(gyro(:,3), tau, fs);

% 绘制Allan标准差曲线
figure;
loglog(tau_used, adev_x, 'r', 'LineWidth', 1.5); hold on;
loglog(tau_used, adev_y, 'g', 'LineWidth', 1.5);
loglog(tau_used, adev_z, 'b', 'LineWidth', 1.5);
grid on;
xlabel('相关时间 \tau (s)');
ylabel('Allan标准差 \sigma(\tau) (rad/s)');
title('陀螺仪Allan方差分析');
legend('X轴', 'Y轴', 'Z轴');

% 从图中识别零偏不稳定性(Allan方差最小值)
[min_adev_x, min_idx_x] = min(adev_x);
[min_adev_y, min_idx_y] = min(adev_y);
[min_adev_z, min_idx_z] = min(adev_z);

fprintf('\n【Allan方差法】零偏不稳定性 (最小值):\n');
fprintf('X轴: %.6f rad/s (τ=%.1fs)\n', min_adev_x, tau_used(min_idx_x));
fprintf('Y轴: %.6f rad/s (τ=%.1fs)\n', min_adev_y, tau_used(min_idx_y));
fprintf('Z轴: %.6f rad/s (τ=%.1fs)\n', min_adev_z, tau_used(min_idx_z));

GJB_10s(gyro_x,fs,smoothing_time);
[sigma, tau, Err] = allan_variance_analysis(gyro_x, 1/fs,...
    'title', '陀螺仪Allan方差分析');