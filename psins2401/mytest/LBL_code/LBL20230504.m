%% 长基线数据，与USBL的一致
clear all
fid=fopen('log1-20230504-0017-lbl.xpf.txt','rt');
% fid=fopen('log1-20230504-0124-ins-LBL.xpf.txt','rt');
% fid=fopen('PHINS_6000-D-20140924-103004-ins-lbl.xpf.txt','rt');
for i=1:12
    fgets(fid);
end
LBL=fscanf(fid,'%d/%d/%d %d:%d:%d.%f %d %f %f %f %f %f \n',[13,inf]);
% 2023/05/04	00:17:04.4559	132	15.837422227487	115.142674930394	4209.720215	1936.098022	5.944000
LBL(14,:)=LBL(4,:)*3600+LBL(5,:)*60+LBL(6,:)+LBL(7,:)/10000;
LBL(1:7,:)=[];
fclose(fid);

beaconID=[132,133,134,135];% 132=90
beacon=cell(1,4);
for i=1:4
    beacon{i}=LBL(:,LBL(1,:)==beaconID(i));
    subplot(2,2,i)
    plot(diff(beacon{i}(end,:)))
    BCN(i,:)=beacon{i}(2:4,1);
end
param=Param;
BCN(:,1)=BCN(:,1)*param.D2R;
BCN(:,2)=BCN(:,2)*param.D2R;
%%
% fid=fopen('log1-20230430-0347-ins-nav.xpf.txt','rt');
fid=fopen('log1-20230504-0017-ins-nav.xpf.txt','rt');
% fid=fopen('log1-20230504-0124-ins-NAV.xpf.txt','rt');
% fid=fopen('PHINS_6000-D-20140924-103004-ins-nav.xpf.txt','rt'); 
for i=1:21
fgets(fid);
end
Ins_Nav=fscanf(fid,'%d/%d/%d %d:%d:%d.%f %d %d %d %f %f %f %f %f %f %f %f %f %f %f %f\n',[22,inf]);
Ins_Nav(23,:)=Ins_Nav(4,:)*3600+Ins_Nav(5,:)*60+Ins_Nav(6,:)+Ins_Nav(7,:)/10000;
Ins_Nav(1:10,:)=[];
% 2023/05/04	00:17:02.6748	3146529	0	67108864	15.820634196889	115.147240101896	-4128.846191	
% 351.475006	-0.744000	4.775000	1.460148	-0.375065	0.012396	0.154284	0.003127	-0.141090
% heading	roll	pitch  speedNorth	speedEast	speedUp
% heave	 surge	sway
fclose(fid);
%%
plot_beacon_distances_with_custom_func(BCN)
%%

% --- 用户输入参数 ---

% 1. 四个信标的大地坐标 (纬度 [度], 经度 [度], 深度 [米, 正值])
%    示例数据，请替换为您的实际数据
beacon_lla_depth = [
    30.0000, 120.0000, 100;  % 信标1: 纬度, 经度, 深度
    30.0010, 120.0000, 105;  % 信标2
    30.0000, 120.0010, 98;   % 信标3
    30.0010, 120.0010, 102   % 信标4
    ];

% 2. 斜距测量数据 (N_samples x 4 矩阵)
%    每行代表一个时间点的四个斜距 [R1, R2, R3, R4] (单位：米)
%    假设有 N_samples (约280) 组测量数据
%    示例数据 (请替换为您的实际数据，例如从 .mat 文件加载)
%    load('range_data.mat'); % 假设 range_measurements 在这个文件里
N_samples = 280; % 假设有280个数据点
% 用随机数据模拟，实际应用时替换为真实数据
% 这部分模拟数据仅用于演示，实际范围应基于信标间距和AUV大致深度
avg_beacon_dist_approx = 500; % 对信标间平均距离的粗略估计
avg_depth_approx = mean(beacon_lla_depth(:,3)) + 50; % AUV在信标深度以下
range_measurements = avg_depth_approx + rand(N_samples, 4) * avg_beacon_dist_approx * 0.3;
% 确保模拟的斜距大于深度差
for i=1:4
    depth_diff = abs(avg_depth_approx - beacon_lla_depth(i,3));
    range_measurements(:,i) = max(range_measurements(:,i), depth_diff + 50); % 确保斜距至少比深度差大一点
end


% --- 1. 坐标系设置与转换 ---
fprintf('步骤 1: 坐标系设置与转换...\n');

% 选择参考点 (例如第一个信标) 用于 NED 转换
% 参考点的高度设为0 (对应于WGS84椭球表面)
ref_lla = [beacon_lla_depth(1,1), beacon_lla_depth(1,2), 0];
spheroid = wgs84Ellipsoid('meter'); % 使用WGS84椭球模型

% 将信标 LLA+Depth 转换为 NED 坐标
% geodetic2ned 需要高度作为输入，深度为正，则高度为负
beacon_altitudes = -beacon_lla_depth(:,3);
[beacon_N, beacon_E, beacon_D] = geodetic2ned( ...
    beacon_lla_depth(:,1), beacon_lla_depth(:,2), beacon_altitudes, ...
    ref_lla(1), ref_lla(2), ref_lla(3), spheroid);
beacon_ned = [beacon_N, beacon_E, beacon_D]; % N x 3 矩阵，(北, 东, 地)

fprintf('信标在局部NED坐标系下的位置 (米):\n');
disp(beacon_ned);

% --- 2. 初始化AUV轨迹存储 ---
auv_trajectory_ned = zeros(N_samples, 3); % 存储AUV在NED坐标系下的位置

% --- 3. 对每一组斜距数据进行迭代最小二乘解算 ---
fprintf('\n步骤 2: 开始解算AUV位置...\n');

% ILS 参数
max_iterations = 20;  % 最大迭代次数
tolerance = 1e-6;     % 收敛阈值 (米)

% AUV位置的初始猜测值 (NED坐标)
% 第一次的初始猜测：可以使用信标的几何中心，深度为平均斜距减去平均信标深度（非常粗略）
% 或使用更复杂的线性化方法获得 (见下方 helper function)
% P_auv_guess_ned = [mean(beacon_N), mean(beacon_E), mean(beacon_D) + mean(range_measurements(1,:))/2];
P_auv_guess_ned = calculate_initial_guess_linearized(beacon_ned, range_measurements(1,:)');
if any(isnan(P_auv_guess_ned)) || any(isinf(P_auv_guess_ned))
    P_auv_guess_ned = [mean(beacon_N), mean(beacon_E), mean(beacon_D) + 20]; % 备用初始值
    fprintf('线性化初始猜测失败，使用备用初始值。\n');
end


for i = 1:N_samples
    current_ranges = range_measurements(i, :)'; % 当前时刻的4个斜距 (列向量)

    % 对于后续时间点，使用上一时刻的解作为初始猜测值
    if i > 1
        P_auv_guess_ned = auv_trajectory_ned(i-1, :);
    end

    % 调用迭代最小二乘解算器
    P_auv_solved_ned = iterative_least_squares_solver( ...
        beacon_ned, current_ranges, P_auv_guess_ned, ...
        max_iterations, tolerance);

    auv_trajectory_ned(i, :) = P_auv_solved_ned;

    if mod(i, 50) == 0 % 每50个点打印一次进度
        fprintf('已处理 %d / %d 个数据点. 当前估算AUV位置 (NED): [%.2f, %.2f, %.2f]\n', ...
            i, N_samples, P_auv_solved_ned(1), P_auv_solved_ned(2), P_auv_solved_ned(3));
    end
end

fprintf('AUV位置解算完成。\n');

% --- 4. (可选) 将AUV的NED坐标转换回大地坐标 ---
fprintf('\n步骤 3: 将AUV轨迹从NED转换回大地坐标 (纬度, 经度, 深度)...\n');
auv_lat = zeros(N_samples, 1);
auv_lon = zeros(N_samples, 1);
auv_altitude_ellipsoid = zeros(N_samples, 1); % AUV相对于椭球的高度

for i = 1:N_samples
    [auv_lat(i), auv_lon(i), auv_altitude_ellipsoid(i)] = ned2geodetic( ...
        auv_trajectory_ned(i,1), auv_trajectory_ned(i,2), auv_trajectory_ned(i,3), ...
        ref_lla(1), ref_lla(2), ref_lla(3), spheroid);
end
auv_depth = -auv_altitude_ellipsoid; % 深度为正

auv_trajectory_lla = [auv_lat, auv_lon, auv_depth];

fprintf('AUV轨迹 (LLA) 的前5个点:\n');
disp(auv_trajectory_lla(1:min(5,N_samples), :));

% --- 5. (可选) 绘图 ---
figure;
subplot(2,1,1);
plot(beacon_ned(:,2), beacon_ned(:,1), 'r^', 'MarkerSize', 10, 'MarkerFaceColor', 'red'); % 信标位置 (East-North平面)
hold on;
plot(auv_trajectory_ned(:,2), auv_trajectory_ned(:,1), 'b.-'); % AUV轨迹 (East-North平面)
xlabel('East (米)');
ylabel('North (米)');
title('AUV 轨迹 (NED坐标系 - 水平面)');
legend('信标', 'AUV轨迹', 'Location', 'best');
axis equal;
grid on;

subplot(2,1,2);
plot(1:N_samples, auv_trajectory_ned(:,3), 'g.-'); % AUV深度变化
xlabel('样本点索引');
ylabel('Depth (米)');
title('AUV 深度 (NED坐标系)');
set(gca, 'YDir','reverse'); % 通常深度向下为正，绘图时Y轴反向更直观
grid on;



% --- Helper Function: Iterative Least Squares Solver ---
function P_est_ned = iterative_least_squares_solver(beacon_positions_ned, measured_ranges, ...
    P_initial_guess_ned, max_iter, tol)
P_est_ned = P_initial_guess_ned; % (1x3 row vector)
num_beacons = size(beacon_positions_ned, 1);

for iter = 1:max_iter
    H = zeros(num_beacons, 3);      % Jacobian矩阵
    calculated_ranges = zeros(num_beacons, 1);
    residuals = zeros(num_beacons, 1);

    for j = 1:num_beacons
        beacon_j_pos = beacon_positions_ned(j, :); % (1x3)
        diff_vec = P_est_ned - beacon_j_pos;       % (1x3)
        dist_est = norm(diff_vec);

        if dist_est < 1e-3 % 避免除以零或过小的值
            dist_est = 1e-3;
        end

        calculated_ranges(j) = dist_est;
        residuals(j) = measured_ranges(j) - calculated_ranges(j);

        % 计算雅可比矩阵的行
        H(j, :) = diff_vec / dist_est;
    end

    % 求解位置修正量 delta_P (3x1 column vector)
    % delta_P = (H' * H) \ (H' * residuals); % 标准最小二乘
    % 使用伪逆增加鲁棒性，尤其是在H接近奇异时
    if rank(H'*H) < 3
        % fprintf('Warning: Jacobian product is rank deficient in iteration %d. Using pseudo-inverse.\n', iter);
        delta_P = pinv(H) * residuals; % 更鲁棒的选择
    else
        delta_P = (H' * H) \ (H' * residuals);
    end


    % 更新位置估计
    P_est_ned = P_est_ned + delta_P'; % delta_P是列向量，转置后相加

    % 检查收敛性
    if norm(delta_P) < tol
        % fprintf('Converged in %d iterations.\n', iter);
        break;
    end
end
if iter == max_iter
    % fprintf('Warning: ILS reached maximum iterations (%d) without full convergence.\n', max_iter);
end
end

% --- Helper Function: Linearized Initial Guess (Optional but can be helpful) ---
function P_guess_ned = calculate_initial_guess_linearized(beacon_pos_ned, measured_ranges)
% 使用前四个信标（或至少3个）通过线性化方程组求解初始位置
% (x-x1)^2 + (y-y1)^2 + (z-z1)^2 = R1^2
% (x-x2)^2 + (y-y2)^2 + (z-z2)^2 = R2^2
% ...
% 两两相减进行线性化:
% 2(x2-x1)x + 2(y2-y1)y + 2(z2-z1)z = R1^2-R2^2 - (x1^2+y1^2+z1^2) + (x2^2+y2^2+z2^2)
% (确保 beacon_pos_ned 和 measured_ranges 至少有3个数据点)

if size(beacon_pos_ned,1) < 3 || length(measured_ranges) < 3
    P_guess_ned = [mean(beacon_pos_ned(:,1)), mean(beacon_pos_ned(:,2)), mean(beacon_pos_ned(:,3)) + 20]; % Fallback
    warning('Not enough beacons/ranges for linearized guess, using fallback.');
    return;
end

A_lin = zeros(size(beacon_pos_ned,1)-1, 3);
b_lin = zeros(size(beacon_pos_ned,1)-1, 1);

x1 = beacon_pos_ned(1,1); y1 = beacon_pos_ned(1,2); z1 = beacon_pos_ned(1,3);
R1_sq = measured_ranges(1)^2;
K1 = x1^2 + y1^2 + z1^2;

for i = 2:min(4, size(beacon_pos_ned,1)) % Use up to 3 equations (from beacon 2,3,4 vs 1)
    xi = beacon_pos_ned(i,1); yi = beacon_pos_ned(i,2); zi = beacon_pos_ned(i,3);
    Ri_sq = measured_ranges(i)^2;
    Ki = xi^2 + yi^2 + zi^2;

    A_lin(i-1, 1) = 2 * (xi - x1);
    A_lin(i-1, 2) = 2 * (yi - y1);
    A_lin(i-1, 3) = 2 * (zi - z1);
    b_lin(i-1) = R1_sq - Ri_sq - K1 + Ki;
end

% 如果信标数少于4个，调整A_lin和b_lin的大小
if size(beacon_pos_ned,1) == 3
    A_lin = A_lin(1:2,:);
    b_lin = b_lin(1:2,:);
    % 对于只有2个方程3个未知数的情况，线性解算不唯一，需要额外约束或更好的方法
    % 这里仅为示例，实际中若信标数不足4，线性解算可能效果不佳或无解
    warning('Linearized guess with only 3 beacons might be underdetermined/ill-conditioned for 3D.');
    % 可以考虑固定一个维度（例如深度），然后解算2D位置，但这超出了通用解的范畴
    P_guess_ned = [mean(beacon_pos_ned(:,1)), mean(beacon_pos_ned(:,2)), mean(beacon_pos_ned(:,3)) + 20]; % Fallback
    return;
end


if rank(A_lin) < 3
    warning('Matrix for linearized initial guess is singular/rank-deficient. Using fallback guess.');
    P_guess_ned = [mean(beacon_pos_ned(:,1)), mean(beacon_pos_ned(:,2)), mean(beacon_pos_ned(:,3)) + mean(measured_ranges)/2];
else
    P_sol_lin = A_lin \ b_lin;
    P_guess_ned = P_sol_lin';
end


% 要运行此代码:
% 1. 将此代码保存为 .m 文件 (例如 solve_lbl_positioning.m)。
% 2. 确保您有 MATLAB 及 Mapping Toolbox (用于 geodetic2ned, ned2geodetic, wgs84Ellipsoid)。
%    如果没有Mapping Toolbox，您需要自行实现坐标转换函数或使用简化的平面假设（如果区域较小）。
% 3. 替换示例的 beacon_lla_depth 和 range_measurements 为您的真实数据。
% 4. 在MATLAB命令窗口运行: auv_trajectory_lla_result = solve_lbl_positioning();
end