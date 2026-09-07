clear all
load('D:\GitHub\PSINS\psins2401\mytest\03_sum\data_1\deep-sea.mat','BCN','RNG','avp_LBL_DR')
%% 1、对比信标位置和轨迹
close all
BCNrrm=[BCN{1};BCN{2};BCN{3};BCN{4}];
BCNddm=BCNrrm;
BCNddm(:,1:2)=BCNddm(:,1:2)/pi*180;
trjrrm=avp_LBL_DR(:,7:9);
velref=avp_LBL_DR(:,4:6);
trjddm=trjrrm;
trjddm(:,1:2)=trjrrm(:,1:2)/pi*180;
plot_beacon_distances_with_custom_func(BCNddm,BCNrrm,trjddm)
%% 2、根据信标仿真距离信息 
% 1. 定义四个信标位置 [lat, lon, depth] (示例数据)
dt=0.5;
t=avp_LBL_DR(:,end);
param=Param();
% 2. 参数设置
measure_interval = 0.5; % 每10秒一个测量
range_noise_std = 0.15;   % 距离噪声标准差(m)

% 3. 计算各时刻AUV到信标的斜距
% 找出测量时间点对应的索引
measure_idx = 1:measure_interval/dt:length(t);
num_measures = length(measure_idx);

% 初始化存储矩阵
true_ranges = zeros(num_measures, 4);  % 真实斜距
ranges_simu_noisy = zeros(num_measures,4); % 带噪声斜距

% 计算每个测量时刻的斜距
for i = 1:num_measures
    idx = measure_idx(i);
    auv_pos = trjrrm(idx,:); % 当前AUV位置
    
    for j = 1:4
        % 计算AUV与信标j的ECEF坐标差
        [rm, rn] = getRmRn(auv_pos(1), param);
        dx = (BCNrrm(j,2) - auv_pos(2)) * ((rn+auv_pos(3))* cos(auv_pos(1))) ;
        dy = (BCNrrm(j,1) - auv_pos(1)) * (rm+ auv_pos(3)) ;
        dz = BCNrrm(j,3) - auv_pos(3);
        % 计算斜距（三维欧氏距离）
        true_ranges(i,j) = sqrt(dx^2 + dy^2 + dz^2);
        % [true_ranges_1(i,j),~]= caldot2dot(auv_pos,BCNrrm(j,:));
        % 添加高斯噪声
        ranges_simu_noisy(i,j) = true_ranges(i,j) + range_noise_std*randn();
    end
end
% 4. 结果可视化
figure;
% 轨迹和信标位置（二维投影）
subplot(2,1,1);
plot(trjddm(:,2), trjddm(:,1), 'b-'); hold on;
plot(BCNddm(:,2), BCNddm(:,1), 'rp', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('经度(°)'); ylabel('纬度(°)');
title('AUV轨迹与信标分布');
legend('AUV轨迹', '信标位置', 'Location', 'best');
grid on;

% 距离测量序列
subplot(2,1,2);
plot(measure_idx*dt, ranges_simu_noisy);
xlabel('时间(s)'); ylabel('斜距(m)');
title('AUV到各信标的斜距测量');
legend('信标1', '信标2', '信标3', '信标4');
grid on;

figure
ranges_meas=[RNG{1},RNG{2},RNG{3},RNG{4}];
plot(t,ranges_meas)
hold on
plot(measure_idx*dt, ranges_simu_noisy);
title('AUV到各信标的斜距测量仿真和实测对比');

% 数据预处理
[BCNxyz(:,1), BCNxyz(:,2)] = deg2utm(BCNddm(:,1), BCNddm(:,2)); % UTM转换
BCNxyz(:,3) = BCNddm(:,3)-BCNddm(1,3);  % 深度赋值
true_auv_xyz=[];
[true_auv_xyz(:,1),true_auv_xyz(:,2)]=deg2utm([BCNddm(1,1);trjddm(:,1)],[BCNddm(1,2);trjddm(:,2)]);
true_auv_xyz(1,:)=[];
true_auv_xyz(:,3)=trjddm(:,3)-BCNddm(1,3);

depth = trjddm(:,3)-BCNddm(1,3);
%% 3-1 线性最小二乘解算
% 切换控制变量
  % <<< 设置为 true 表示深度已知，false 表示未知
% depth_known = false; 
% mode={1,2,3,4};
mode=1;
for mode=2
switch(mode)
    case 1
        depth_known = true;
        ranges=ranges_simu_noisy;
        disp(['LS方法:','深度已知，仿真距离误差',num2str(range_noise_std),'m'])
    case 2
        depth_known = true;
        ranges=ranges_meas;
        disp(['LS方法:','深度已知，实测距离'])
    case 3
        depth_known = false;
        ranges=ranges_simu_noisy;
        disp(['LS方法:','深度未知，仿真距离误差',num2str(range_noise_std),'m'])
    case 4
        depth_known = false;
        ranges=ranges_meas;
        disp(['LS方法:','深度未知，实测距离'])
end
tic
% 准备工作
num_samples = size(ranges_simu_noisy,1);
auv_pos = zeros(num_samples,3);
residual = zeros(num_samples,1);
d = sqrt(sum(BCNxyz.^2, 2));  % 用于构造B向量
% 根据模式构建矩阵 A
if depth_known
    A = zeros(4,2);
    z_diff = zeros(4,1);
    for i = 1:4
        j = mod(i,4)+1;
        A(i,:) = [BCNxyz(j,1)-BCNxyz(i,1), BCNxyz(j,2)-BCNxyz(i,2)];
        z_diff(i) = BCNxyz(j,3) - BCNxyz(i,3);
    end
else
    A = zeros(4,3);
    for i = 1:4
        j = mod(i,4)+1;
        A(i,:) = [BCNxyz(j,1)-BCNxyz(i,1), BCNxyz(j,2)-BCNxyz(i,2), BCNxyz(j,3)-BCNxyz(i,3)];
    end
end
    
% 主解算循环
for k = 1:num_samples
    ranges_k = ranges(k,:);
    h_k = depth(k);  % 已知或模拟得到的深度
    B = zeros(4,1);

    for i = 1:4
        j = mod(i,4)+1;
        range_sq_diff = ranges_k(i)^2 - ranges_k(j)^2;
        d_sq_diff = d(j)^2 - d(i)^2;

        if depth_known
            B(i) = 0.5 * (range_sq_diff + d_sq_diff) - h_k * z_diff(i);
        else
            B(i) = 0.5 * (range_sq_diff + d_sq_diff);
        end
    end
    % 解算（式 2-3）
    if depth_known
        X = (A' * A + 1e-6 * eye(2)) \ (A' * B);  % A 是 4×2
        auv_pos(k,:) = [X(1), X(2), h_k];         % 深度已知直接赋值
    else
        X = (A' * A + 1e-6 * eye(3)) \ (A' * B);  % A 是 4×3
        auv_pos(k,:) = X';  % 三维估计
    end

    % 速度估计 (finite difference)
    if k > 1
        auv_vel(k,:) = (auv_pos(k,:) - auv_pos(k-1,:)) / dt;
    else
        auv_vel(k,:) = [0, 0, 0]; % Velocity at first step is zero or undefined
    end

    % 残差计算
    estimated_ranges = sqrt(sum((BCNxyz - auv_pos(k,:)).^2, 2));
    residual(k) = norm(estimated_ranges - ranges_k');
end
toc
% 定位误差计算与绘图
horizontal_error(:,1) = measure_idx * dt;
horizontal_error(:,2) = sqrt((auv_pos(:,1) - true_auv_xyz(:,1)).^2 + ...
                             (auv_pos(:,2) - true_auv_xyz(:,2)).^2);
if mode==3||mode==4
    depth_err=abs(auv_pos(:,3)-true_auv_xyz(:,3));
    figure
    plot(horizontal_error(:,1),depth_err)
    fprintf('平均深度误差: %.2f ± %.2f m\n', mean(depth_err), std(depth_err));
    fprintf('最大深度误差: %.2f m\n\n', max(depth_err));
end
plot_result_and_error(BCNxyz, true_auv_xyz, auv_pos, horizontal_error, residual);
plot_velerr(num_samples,auv_vel,velref,t)
end
%% 3-2 非线性最小二乘解算  
% 切换控制变量
% depth_known = true;  % true: 深度已知，false: 深度未知
% depth_known = false; 
for mode=1:4
switch(mode)
    case 1
        depth_known = true;
        ranges=ranges_simu_noisy;
        disp(['NLS方法:','深度已知，仿真距离误差',num2str(range_noise_std),'m'])
    case 2
        depth_known = true;
        ranges=ranges_meas;
        disp(['NLS方法:','深度已知，实测距离'])
    case 3
        depth_known = false;
        ranges=ranges_simu_noisy;
        disp(['NLS方法:','深度未知，仿真距离误差',num2str(range_noise_std),'m'])
    case 4
        depth_known = false;
        ranges=ranges_meas;
        disp(['NLS方法:','深度未知，实测距离'])
end
tic
% 1. 参数设置
num_measures = size(ranges_simu_noisy,1);
estimated_pos = zeros(num_measures,3); % 存储估计位置
residuals = zeros(num_measures,1);     % 存储残差
% 2. 非线性最小二乘定位解算
for k = 1:num_measures
    ranges_k = ranges(k,:);  % 当前距离测量
    if depth_known
        % 初始猜测 XY
        if k == 1
            init_guess_xy = mean(BCNxyz(:,1:2),1);
        else
            init_guess_xy = estimated_pos(k-1,1:2);
        end

        % 优化 XY，Z 固定
        options = optimoptions('lsqnonlin','Display','off','Algorithm','levenberg-marquardt');
        est_xy = lsqnonlin(@(xy) range_residuals_fixed_depth(xy, depth(k), BCNxyz, ranges_k),...
                           init_guess_xy,[],[],options);
        estimated_pos(k,:) = [est_xy, depth(k)];
        
    else
        % 初始猜测为前一估计或信标几何中心
        if k == 1
            init_guess_xyz = mean(BCNxyz,1);
        else
            init_guess_xyz = estimated_pos(k-1,:);
        end

        % 优化 XYZ
        options = optimoptions('lsqnonlin','Display','off','Algorithm','levenberg-marquardt');
        est_xyz = lsqnonlin(@(x) range_residuals(x, BCNxyz, ranges_k),...
                            init_guess_xyz,[],[],options);
        estimated_pos(k,:) = est_xyz;
    end

    % 计算残差
    estimated_ranges = sqrt(sum((BCNxyz - estimated_pos(k,:)).^2, 2));
    residuals(k) = norm(estimated_ranges - ranges_k');
end
toc
% 3. 定位误差 (仅水平面)
horizontal_error(:,1) = measure_idx * dt;
horizontal_error(:,2) = sqrt((estimated_pos(:,1) - true_auv_xyz(:,1)).^2 + ...
                             (estimated_pos(:,2) - true_auv_xyz(:,2)).^2);
if mode==3||mode==4
    depth_err=abs(auv_pos(:,3)-true_auv_xyz(:,3));
    figure
    plot(horizontal_error(:,1),depth_err)
    fprintf('平均深度误差: %.2f ± %.2f m\n', mean(depth_err), std(depth_err));
    fprintf('最大深度误差: %.2f m\n\n', max(depth_err));
end
% 4. 绘图
plot_result_and_error(BCNxyz, true_auv_xyz, estimated_pos, horizontal_error, residuals);
end
%% deg2utm函数
function [x,y] = deg2utm(lat,lon)
    lat_rad = deg2rad(lat);
    lon_rad = deg2rad(lon);
    param=Param();
    [rm,rn]=getRmRn(lat_rad(1),param);
    % 相对第一个点的坐标
    x = rn * (lon_rad - lon_rad(1)) .* cos(lat_rad(1));
    y = rm * (lat_rad - lat_rad(1));
end
function res = range_residuals_fixed_depth(xy, known_depth, BCNddm, ranges)
    % 将已知深度合并为完整位置
    pos = [xy(:)', known_depth]; % 行向量
    % 计算预测距离
    predicted_ranges = sqrt(sum((BCNddm - pos).^2, 2));
    % 残差
    res = predicted_ranges - ranges(:);
end
function res = range_residuals(x, BCNxyz, ranges)
    predicted = sqrt(sum((BCNxyz - x(:)').^2, 2));
    res = predicted - ranges(:);
end
