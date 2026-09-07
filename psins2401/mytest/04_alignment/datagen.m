close all
clear
glvs
% trj = trjfile('trj_range.mat');
ts = 0.01;  
avp0 = [[d2r(30);d2r(15);d2r(80)];[0;0;0]; glv.pos0]; 
xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      20); % 保持原来的状态不变
trj = trjsimu(avp0, seg.wat, ts, 1); 
%% 添加传感器误差
imuerr = imuerrset(0.01, 50, 0.01, 10);
imu= imuadderr(trj.imu, imuerr);
IMUFRD = imuRFU2FRD(imu);
%% 检查地球自转
vb=r2d(mean(imu(1:50,1:3),1)'/0.01)*3600  
vn=r2d([0;cos(glv.pos0(1));sin(glv.pos0(1))]*glv.wie)*3600
cnb=a2mat(avp0(1:3))
vnn=cnb*vb
%% 粗对准
disp('真实欧拉角姿态：')
disp(r2d(avp0(1:3)))
CLB_coarse = coarse_leveling(IMUFRD(1:50,5:7), 0.5, 100);% 使用0.5s的数据
euler_coarse=r2d(dcm2euler(CLB_coarse));
vn=[0;0;glv.g0*1.005];
[~, att, ~] = sv2atti(vn, mean(imu(1:50,4:6),1)'/0.01);
disp('单矢量定姿态（重力矢量）：')
disp(r2d(att))
[~, att, ~] = sv2atti([0;cos(glv.pos0(1));sin(glv.pos0(1))]*glv.wie,...
    mean(imu(1:50,1:3),1)'/0.01);
disp('单矢量定姿态（地球自转角速度）：')
disp(r2d(att))

CLB_coarse1 = single_vec_coarse_leveling(IMUFRD(1:50,5:7)/0.01,-vn);
euler_coarse1=r2d(dcm2euler(CLB_coarse1));

vn1=[cos(glv.pos0(1));0;-sin(glv.pos0(1))]*glv.wie;
CLB_coarse2 = single_vec_coarse_leveling(IMUFRD(1:50,2:4),vn1);
euler_coarse2=r2d(dcm2euler(CLB_coarse2));

vn11=[0;cos(glv.pos0(1));sin(glv.pos0(1))]*glv.wie;
[qnb, att, Cnb] = dv2atti(vn, vn11, mean(imu(1:50,4:6),1)', mean(imu(1:50,1:3),1)');
euler_coarse3=r2d(att);
disp('双矢量定姿态（重力加速度+地球自转角速度）：')
disp(r2d(att))

%% 精对准
% 1. 定义状态向量和系统参数

% 状态向量 x (15x1):
% x = [delta_phi_L (3x1);         % 姿态误差
%      delta_omega_IEH_L (2x1);   % 地球自转水平分量误差
%      delta_v_L_H (2x1);         % 速度水平误差
%      delta_r_L_H (2x1);         % 位置水平散度误差
%      epsilon (3x1);             % 陀螺仪偏置
%      nabla (3x1)];              % 加速度计偏置

% 系统参数 (根据实际IMU和地理位置设置)
dt = 0.01;  % 采样时间 (s)
g = glv.g0; % 当地重力加速度 (m/s^2)
omega_e = glv.wie; % 地球自转角速度 (rad/s)
latitude = glv.pos0(1); % 地理纬度 (rad)

% 真实比力 (准静态下近似为重力反方向)
% 在L系下，假设L系是NED，则为 [0; 0; g]
current_f_sf_L = [0; 0; g]; %

% 地球自转角速度 (L系下，NED坐标系)
% 准静态下，omega_EL_L ~ 0
current_omega_IL_L = [omega_e * cos(latitude); 0; -omega_e * sin(latitude)]; %

% 陀螺仪噪声参数 (随机游走，对应过程噪声 Q)
gyro_noise_density = 0.01*glv.dpsh; % rad/s/sqrt(Hz)

% 加速度计噪声参数 (随机游走，对应过程噪声 Q)
accel_noise_density = 5*glv.ugpsHz; % m/s^2/sqrt(Hz)

% 地球自转水平分量误差的噪声 (随机游走，对应过程噪声 Q)
omega_IEH_noise_density = 1e-6; % rad/s/sqrt(Hz)

% 测量噪声 (位置散度噪声)
pos_measurement_noise_std = 1.0; % m

% 初始协方差矩阵 P (15x15，根据不确定性初始化)
P = diag([
    deg2rad(10)^2, deg2rad(10)^2, deg2rad(180)^2, ... % delta_phi (姿态误差，横滚俯仰较小，航向可能很大)
    deg2rad(0.1)^2, deg2rad(0.1)^2, ...             % delta_omega_IEH_L (地球自转水平分量误差，2D)
    1.0^2, 1.0^2, ...                               % delta_v_L_H (速度水平误差，2D)
    10.0^2, 10.0^2, ...                           % delta_r_L_H (位置水平误差，2D)
    (0.05*glv.dph)^2, (0.05*glv.dph)^2, (0.05*glv.dph)^2, ... % epsilon (陀螺仪偏置)
    (500*glv.ug)^2, (500*glv.ug)^2, (500*glv.ug)^2                      % nabla (加速度计偏置)
]);

% 初始状态向量 (通常初始化为零)
x_hat = zeros(15, 1);
x_hat(1:3)=d2r(euler_coarse);
x_est(1:15,1)=x_hat;

H = zeros(2, 15);
H(1:2, 8:9) = eye(2); % 测量的是位置水平散度误差 delta_r_L_H，对应状态向量的第 4 个 2x1 块

R = (pos_measurement_noise_std^2) * eye(2);

% *****7. 卡尔曼滤波器主循环*****

% 假设已经有 IMU 测量数据 (gyro_data, accel_data) 和 GPS 测量数据 (gps_pos_H)
% IMU 数据是 B 系下的原始数据
% GPS 数据是 N 系下的位置数据

% 伪代码主循环，需要替换为实际数据和迭代逻辑
num_steps = 1000; % 示例步数
x_hat_history = zeros(15, num_steps);
P_history = zeros(15, 15, num_steps);

% 假设 coarse_aligned_C_L_B 是粗对准后得到的初始姿态矩阵
coarse_aligned_C_L_B = CLB_coarse; 

for k = 1:num_steps
    % 1. 获取当前时刻的IMU测量
    % accel_B = imu_measurements(k).accel; % B系加速度计测量
    % gyro_B = imu_measurements(k).gyro;   % B系陀螺仪测量

    % 2. 获取当前的导航解算器状态 (通常由IMU积分得到)
    % 在实际系统中，这些值会从导航解算器中获取
    % 这里为了示例，使用固定值或根据步进时间模拟变化
    current_C_L_B = coarse_aligned_C_L_B; % 简化处理，实际需要实时更新

    % 3. **预测步 (Predict)**
    % 3.1. 计算 F 矩阵 (状态转移矩阵)
    F_k = calculate_F_matrix(current_omega_IL_L, current_C_L_B, current_f_sf_L);
    
    % 3.2. 计算状态转移矩阵 Phi (Phi = expm(F_k * dt))
    Phi_k = expm(F_k * dt);
    
    % 3.3. 计算 Q 矩阵 (过程噪声协方差矩阵)
    Q_k = calculate_Q_matrix(dt, gyro_noise_density, accel_noise_density, omega_IEH_noise_density);
    
    % 3.4. 预测状态 (x_hat(k|k-1) = Phi_k * x_hat(k-1|k-1))
    x_hat = Phi_k * x_hat;
    
    % 3.5. 预测协方差 (P(k|k-1) = Phi_k * P(k-1|k-1) * Phi_k.' + Q_k)
    P = Phi_k * P * Phi_k.' + Q_k;

    % 4. **更新步 (Update)**
    % 4.1. 获取当前时刻的GPS测量
    % Z_measured = gps_measurements(k).horizontal_pos_L; % L系下的水平位置测量 (相对于参考初始位置的偏差)
    
    % 为了示例，这里假设 Z_measured 是一个包含噪声的零向量，因为在静止对准下，期望位置偏差为零
    Z_measured = [0;0]; % 零测量

    % 4.2. 计算测量残差 (Innovation)
    y_k = Z_measured - H * x_hat;
    
    % 4.3. 计算卡尔曼增益 (K_k = P(k|k-1) * H_k.T * (H_k * P(k|k-1) * H_k.T + R_k)^-1)
    S_k = H * P * H.' + R;
    K_k = P * H.' * inv(S_k);
    
    % 4.4. 更新状态 (x_hat(k|k) = x_hat(k|k-1) + K_k * y_k)
    x_hat = x_hat + K_k * y_k;
    x_est(1:15,k+1)=x_hat;
    % 4.5. 更新协方差 (P(k|k) = (I - K_k * H_k) * P(k|k-1))
    P = (eye(15) - K_k * H) * P;

    % 5. **反馈修正** (将估计的误差反馈给主导航解算)
    % 这一步通常在每次滤波器更新后进行
    
    % 姿态修正:
    % delta_phi_error = x_hat(1:3);
    % delta_C_L_B = eye(3) - skew(delta_phi_error); % 小角度近似
    % coarse_aligned_C_L_B = delta_C_L_B * coarse_aligned_C_L_B; % 更新导航器的姿态
    x_hat(1:3) = zeros(3,1); % 姿态误差归零

    % 地球自转水平分量修正:
    % delta_omega_IEH_L_error = x_hat(4:5);
    % current_omega_IEH_L = current_omega_IEH_L + delta_omega_IEH_L_error; % 更新导航器的地球自转估计
    x_hat(4:5) = zeros(2,1); % 地球自转水平分量误差归零

    % 传感器偏置修正:
    % epsilon_error = x_hat(11:13);
    % nabla_error = x_hat(14:16);
    % current_gyro_bias = current_gyro_bias + epsilon_error; % 更新导航器的陀螺仪偏置估计
    % current_accel_bias = current_accel_bias + nabla_error; % 更新导航器的加速度计偏置估计
    x_hat(10:12) = zeros(3,1); % 陀螺仪偏置误差归零
    x_hat(13:15) = zeros(3,1); % 加速度计偏置误差归零

    % 记录历史数据
    x_hat_history(:, k) = x_hat;
    P_history(:, :, k) = P;
end

disp('精对准卡尔曼滤波器 MATLAB 代码已生成。');
disp('请根据您的实际数据和需求，替换代码中的数据输入和导航解算部分。');
disp('特别注意状态向量维度的匹配以及 F 矩阵中简化的项。');

%% 可选：结果可视化
% figure;
% subplot(3,1,1);
% plot(rad2deg(x_hat_history(1:3,:).'));
% title('姿态误差 (\delta\phi_L)');
% ylabel('角度 (度)');
% legend('Roll', 'Pitch', 'Yaw');
% 
% subplot(3,1,2);
% plot(rad2deg(x_hat_history(4:5,:).'));
% title('地球自转水平分量误差 (\delta\omega^L_{IEH})');
% ylabel('角速度 (度/秒)');
% legend('East', 'North');
% 
% subplot(3,1,3);
% plot(x_hat_history(14:16,:).');
% title('加速度计偏置 (\nabla)');
% ylabel('m/s^2');
% legend('X', 'Y', 'Z');
% 
% % 绘制P矩阵对角线元素 (方差)
% figure;
% for i = 1:15
%     subplot(5,3,i);
%     plot(squeeze(P_history(i,i,:)));
%     title(['P_{', num2str(i), num2str(i), '}']);
%     xlabel('时间步');
%     ylabel('方差');
% end
%% 函数定义
function CLB_coarse = single_vec_coarse_leveling(accel_data,vn)
vb = mean(accel_data,1)';
afa = acos(vn'*vb/norm(vn)/norm(vb));
phi = cross(vb,vn);
nphi = norm(phi);
%    if cos(afa/2)==0 ...
if nphi<10e-20      % q = rv2q(phi/nphi*afa);
    qnb = [cos(afa/2); sin(afa/2)*[1;1;1]];
else
    qnb = [cos(afa/2); sin(afa/2)*phi/nphi];
end
CLB_coarse = quat2dcm(qnb);
end


function S = skew(v)
% 计算向量的反对称矩阵
    S = [0, -v(3), v(2);
         v(3), 0, -v(1);
         -v(2), v(1), 0];
end

% 3. 实时更新 F 矩阵 (状态转移矩阵)

function F_matrix = calculate_F_matrix(current_omega_IL_L, current_C_L_B, current_f_sf_L)
% 根据当前估计值计算F矩阵
% :param current_omega_IL_L: 当前L系相对于惯性系的角速度估计 (3x1)
% :param current_C_L_B: 当前从B系到L系的姿态矩阵 (3x3)
% :param current_f_sf_L: 当前L系下的比力测量值 (3x1)
% :return: F 矩阵 (15x15)

    F_matrix = zeros(15, 15);

    % 1. 姿态误差动力学 (delta_phi_dot)
    F_matrix(1:3, 1:3) = -skew(current_omega_IL_L); % -Omega_IL^L x
    F_matrix(1:3, 4:5) = [1, 0; 0, 1; 0, 0]; % I_{3x2} for delta_omega_IEH_L (假设只取水平分量)
                                             % 注意：如果delta_omega_IEH_L是3D，这里是eye(3)
    F_matrix(1:3, 10:12) = -current_C_L_B; % -C_L^B for epsilon

    % 2. 地球自转水平分量误差动力学 (delta_omega_IEH_L_dot) - 随机游走，F为零
    % F_matrix(4:5, 4:5) = zeros(2,2); (已默认零矩阵)

    % 3. 速度水平误差动力学 (delta_v_L_H_dot)
    f_sf_L_cross = skew(current_f_sf_L); % f_sf^L x
    F_matrix(6:7, 1:3) = f_sf_L_cross(1:2,:); % 取水平分量，假设 f_sf^L x 形式是 [0, -g, 0; g, 0, 0; 0, 0, 0]
                                             % 那么其水平分量是 [[0, -g, 0]; [g, 0, 0]]
                                             % MATLAB中可以通过 F_matrix(6:7, 1:3) = skew(current_f_sf_L)(1:2,:); 实现
    F_matrix(6:7, 13:15) = current_C_L_B(1:2, :); % C_L^B 的水平分量 (for nabla)

    % 4. 位置散度误差动力学 (delta_r_L_H_dot)
    F_matrix(8:9, 6:7) = eye(2); % I_{2x2} for delta_v_L_H

    % 5. 陀螺仪偏置动力学 (epsilon_dot) - 常值，F为零
    % F_matrix(11:13, 11:13) = zeros(3,3); (已默认零矩阵)

    % 6. 加速度计偏置动力学 (nabla_dot) - 常值，F为零
    % F_matrix(14:16, 14:16) = zeros(3,3); (已默认零矩阵)
end
function Q_matrix = calculate_Q_matrix(dt, gyro_noise_density, accel_noise_density, omega_IEH_noise_density)
% 计算Q矩阵
    Q_matrix = zeros(15, 15);

    % delta_phi 的噪声 (由陀螺仪噪声引起)
    Q_matrix(1:3, 1:3) = (gyro_noise_density^2) * dt * eye(3);

    % delta_omega_IEH_L 的噪声 (随机游走)
    Q_matrix(4:5, 4:5) = (omega_IEH_noise_density^2) * dt * eye(2);

    % delta_v_L_H 的噪声 (由加速度计噪声引起)
    Q_matrix(6:7, 6:7) = (accel_noise_density^2) * dt * eye(2); % 仅水平分量

    % delta_r_L_H 的噪声 (通常不直接加噪声，由速度误差传播)

    % epsilon 的噪声 (常值偏置，但通常有随机游走分量，对应陀螺仪零偏随机游走)
    Q_matrix(10:12, 10:12) = (gyro_noise_density^2) * dt * eye(3); 

    % nabla 的噪声 (常值偏置，但通常有随机游走分量，对应加速度计零偏随机游走)
    Q_matrix(13:15, 13:15) = (accel_noise_density^2) * dt * eye(3); 
end

function CLB_coarse = coarse_leveling(accel_data, duration_s, sample_rate_hz)
% coarse_leveling: 实现静态粗对准
%   计算从当地水平坐标系 (L) 到本体坐标系 (B) 的姿态矩阵 CBL。
%
% 输入:
%   accel_data: Nx3 加速度计数据 (m/s^2), N为样本数，列为 [ax, ay, az]
%   duration_s: 用于对准的IMU数据持续时间 (秒)
%   sample_rate_hz: IMU数据的采样率 (Hz)
%
% 输出:
%   CBL_coarse: 3x3 粗对准后的姿态矩阵 (从L系到B系)

% 1. 验证输入数据长度
num_samples_required = round(duration_s * sample_rate_hz);
if size(accel_data, 1) < num_samples_required
    error('IMU数据长度不足，请提供至少 %d 秒的数据。', duration_s);
end

% 2. 截取并平均加速度计数据
% 在静态粗对准中，IMU感知的比力近似等于重力的反方向
avg_accel = mean(accel_data(1:num_samples_required, :), 1); % 平均比力向量 [ax, ay, az]

% 3. 初始化 CBL 矩阵
CLB_coarse = zeros(3, 3);

% 4. 计算 CBL 的第三行 (b_ZL^T = -a_SF^B / ||a_SF^B||)
% 这里的 a_SF^B 就是平均后的加速度计测量值
norm_a_sf = norm(avg_accel);
if norm_a_sf == 0
    error('平均加速度计模长为零，无法计算第三行。请确保IMU有重力输入。');
end
uBL_ZL = -avg_accel / norm_a_sf; % B系下L系Z轴的单位向量 (近似)

CLB_coarse(3, :) = uBL_ZL; % CBL 的第三行是 uBL_ZL 的转置，即直接赋值

% 5. 根据 B系X轴是否垂直来确定第二行和第一行
% B系X轴垂直的条件是 CBL(3,1) 的绝对值大于 0.85
if abs(CLB_coarse(3,1)) > 0.85 % abs(uBL_ZL(1)) > 0.85
    % 情况二：B系X轴接近垂直 (即 B系X轴与L系Z轴接近平行)
    % 此时 CBL(2,3) 设置为 0
    CLB_coarse(2,3) = 0; % C23 = 0
    
    denominator = sqrt(CLB_coarse(3,1)^2 + CLB_coarse(3,2)^2);
    if denominator == 0
        error('分母为零，无法计算第二行。这可能意味着B系X轴和Y轴都接近垂直，无法确定平面。');
    end
    K_val = 1 / denominator; %
    
    CLB_coarse(2,1) = K_val * CLB_coarse(3,2); % C21 = K * C32
    CLB_coarse(2,2) = -K_val * CLB_coarse(3,1); % C22 = -K * C31
    
else
    % 情况一：B系X轴不垂直 (常规情况)
    % 此时 CBL(2,1) 设置为 0
    CLB_coarse(2,1) = 0; % C21 = 0
    
    % 从图片中的公式 (6.1.1-6) 和 (6.1.1-7) 导出：
    % C22 = K * C33; C23 = -K * C32; (来自 C21^2 + C22^2 + C23^2 = 1, C21=0, C21*C31+C22*C32+C23*C33=0)
    % K = 1 / sqrt(C32^2 + C33^2)
    
    denominator = sqrt(CLB_coarse(3,2)^2 + CLB_coarse(3,3)^2);
    if denominator == 0
        error('分母为零，无法计算第二行。这可能意味着B系Y轴和Z轴都接近垂直，无法确定平面。');
    end
    K_val = 1 / denominator; %
    
    CLB_coarse(2,2) = K_val * CLB_coarse(3,3); % C22 = K * C33
    CLB_coarse(2,3) = -K_val * CLB_coarse(3,2); % C23 = -K * C32
end

% 6. 计算 CBL 的第一行 (通过第二行和第三行的叉乘)
% 姿态矩阵是正交的，并且满足右手定则
CLB_coarse(1,:) = cross(CLB_coarse(2,:), CLB_coarse(3,:)); %

% 7. 确保姿态矩阵是正交的 (理论上应该如此，但浮点误差可能导致微小偏差)
% 这步并非文档中严格要求，但实践中通常会进行。
% 可以使用SVD等方法进行正交化，这里不做强制。
% if abs(det(CBL_coarse) - 1) > 1e-6 || norm(CBL_coarse * CBL_coarse' - eye(3)) > 1e-6
%     warning('粗对准姿态矩阵可能不是严格正交矩阵。');
% end

end
