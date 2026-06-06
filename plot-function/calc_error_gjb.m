function fig = calc_error_gjb(testnavpath, truthpath, apply_grubbs)
% -------------------------------------------------------------------------
% Optimized KF-GINS Error Calculation & Plotting (加入 Grubbs 剔除与 GJB 标准算法)
% -------------------------------------------------------------------------

% 默认不开启异常值剔除
if nargin < 3
    apply_grubbs = false;
end

%% 1. 数据导入
try
    temp_res = importdata(testnavpath);
    result_all = temp_res(:, 2:end);
    temp_ref = importdata(truthpath);
    ref = temp_ref(:, 2:end);
catch ME
    error('文件读取失败，请检查路径：%s', ME.message);
end

%% 2. 航向角平滑 (矢量化处理)
% 处理 180/-180 跳变，使用 rad2deg(unwrap(deg2rad(...))) 是标准做法
result_all(:, 10) = rad2deg(unwrap(deg2rad(result_all(:, 10))));
ref(:, 10) = rad2deg(unwrap(deg2rad(ref(:, 10))));

%% 3. 时间对齐与内插
starttime = max(result_all(1, 1), ref(1, 1));
endtime = min(result_all(end, 1), ref(end, 1));
dt = mean(diff(result_all(:, 1)));
time = (starttime:dt:endtime)';
% 内插
newresult = interp1(result_all(:, 1), result_all(:, 2:10), time, 'linear', 'extrap');
newref = interp1(ref(:, 1), ref(:, 2:10), time, 'linear', 'extrap');
% 计算原始误差
error_raw = newresult - newref;
% 航向角误差修正到 [-180, 180]
error_raw(:, 9) = mod(error_raw(:, 9) + 180, 360) - 180; 

%% 4. 位置误差转换 (严格采用 GJB 4727A 公式计算)
param = Param(); % 假设你有这个参数类
error_ned = zeros(size(error_raw, 1), 9);
error_ned(:, 1) = time - time(1); % 相对时间

% 获取动态 RM, RN 并分别计算北向、东向位置误差
for i = 1:size(newref, 1)
    lat_rad = newref(i, 1) * param.D2R;
    h = newref(i, 3);
    [rm, rn] = getRmRn(lat_rad, param);
    
    % GJB 4727A 公式(1) 和 公式(2) 
    error_ned(i, 2) = error_raw(i, 1) * param.D2R * (rm + h);                  % dLat -> North (m)
    error_ned(i, 3) = error_raw(i, 2) * param.D2R * (rn + h) * cos(lat_rad);   % dLon -> East (m)
    error_ned(i, 4) = -error_raw(i, 3);                                        % dH -> Down (m)
end

% 速度和姿态误差直接赋值
error_ned(:, 5:7) = error_raw(:, 4:6); % Velocity NED
error_ned(:, 8:10) = error_raw(:, 7:9); % Attitude

% 计算初始位置径向误差
radial_error = sqrt(error_ned(:, 2).^2 + error_ned(:, 3).^2);

%% ================= 新增：格拉布斯(Grubbs)异常值剔除 =================
total_removed = 0;
if apply_grubbs
    [~, keep_idx, removed_cnt] = grubbs_filter(radial_error);
    total_removed = removed_cnt;
    
    if removed_cnt > 0
        bad_idx = ~keep_idx;
        good_idx = keep_idx;
        
        % 用健康的相邻数据对被剔除点的位置误差进行插值缝合，防止画图断裂
        error_ned(bad_idx, 2) = interp1(time(good_idx), error_ned(good_idx, 2), time(bad_idx), 'linear', 'extrap');
        error_ned(bad_idx, 3) = interp1(time(good_idx), error_ned(good_idx, 3), time(bad_idx), 'linear', 'extrap');
        error_ned(bad_idx, 4) = interp1(time(good_idx), error_ned(good_idx, 4), time(bad_idx), 'linear', 'extrap');
        
        % 重新计算缝合后的径向误差
        radial_error = sqrt(error_ned(:, 2).^2 + error_ned(:, 3).^2);
    end
end
%% ====================================================================

%% 5. 统计指标计算
rms_pos = sqrt(mean(error_ned(:, 2:4).^2));
rms_vel = sqrt(mean(error_ned(:, 5:7).^2));
rms_att = sqrt(mean(error_ned(:, 8:10).^2));

rms_radial = sqrt(mean(radial_error.^2)); % 位置综合误差
max_radial = max(radial_error);           % 位置最大误差

% CEP 计算
sigma_e = std(error_ned(:, 3));
sigma_n = std(error_ned(:, 2));
CEP = 0.5887 * (sigma_e + sigma_n); % 常用近似公式

%% 6. 统一绘图 (保持原始排版)
fig = myfigurestartup(12, 7, 'prese'); 
set(gcf, 'Color', 'w', 'Name', 'Navigation Error Analysis');

% 子图 1: 位置误差
subplot(2, 2, 1);
plot(error_ned(:,1), error_ned(:, 2:4), 'LineWidth', 1.2);
grid on; title('Position Error (NED)');
xlabel('Time (s)'); ylabel('Error (m)');
legend('North', 'East', 'Down', 'Location', 'best');

% 子图 2: 速度误差
subplot(2, 2, 2);
plot(error_ned(:,1), error_ned(:, 5:7), 'LineWidth', 1.2);
grid on; title('Velocity Error (NED)');
xlabel('Time (s)'); ylabel('Error (m/s)');
legend('North', 'East', 'Down', 'Location', 'best');

% 子图 3: 姿态误差
subplot(2, 2, 3);
plot(error_ned(:,1), error_ned(:, 8:10), 'LineWidth', 1.2);
grid on; title('Attitude Error');
xlabel('Time (s)'); ylabel('Error (deg)');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');

% 子图 4: 径向误差与统计文本
subplot(2, 2, 4);
plot(error_ned(:,1), radial_error, 'r', 'LineWidth', 1.2); hold on;
yline(rms_radial, '--k', ['RMS: ', num2str(rms_radial, '%.2f')], 'LabelVerticalAlignment', 'bottom');
grid on; title('Horizontal Radial Error');
xlabel('Time (s)'); ylabel('Error (m)');

%% 7. 终端打印
fprintf('\n----- 导航误差统计结果 -----\n');
if apply_grubbs
    fprintf('[Grubbs filter] 共剔除位置异常点: %d 个\n', total_removed);
end
fprintf('位置 RMS (N E D) : %8.3f %8.3f %8.3f m\n', rms_pos);
fprintf('速度 RMS (N E D) : %8.3f %8.3f %8.3f m/s\n', rms_vel);
fprintf('姿态 RMS (R P Y) : %8.3f %8.3f %8.3f deg\n', rms_att);
fprintf('位置综合误差(RMS): %8.3f m\n', rms_radial);
fprintf('位置最大误差(Max): %8.3f m\n', max_radial);
fprintf('CEP (50%%)        : %8.3f m (%.4f nmi)\n', CEP, CEP/1852);
fprintf('----------------------------\n');

end

%% =========================================================================
%% 局部函数：格拉布斯(Grubbs)异常值自适应剔除算法 (GJB 4727A 附录A)
%% =========================================================================
function [data_clean, keep_idx, removed_count] = grubbs_filter(data_raw)
    n_total = length(data_raw);
    keep_idx = true(n_total, 1);
    removed_count = 0;
    
    n_table = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 30, 35, 40, 45, 50, 60, 70, 80, 90, 100];
    t_table = [1.15, 1.46, 1.67, 1.89, 2.02, 2.13, 2.21, 2.29, 2.36, 2.41, 2.46, 2.51, 2.55, 2.59, 2.62, 2.65, 2.68, 2.71, 2.73, 2.76, 2.78, 2.80, 2.82, 2.91, 2.98, 3.04, 3.09, 3.13, 3.20, 3.26, 3.31, 3.35, 3.38];
    
    while true
        active_data = data_raw(keep_idx);
        n_curr = length(active_data);
        if n_curr < 3, break; end
        
        [sorted_data, sorted_origin_idx] = sort(active_data);
        mu_curr = mean(sorted_data);
        S_curr = std(sorted_data);
        
        if S_curr == 0, break; end
        
        T_max = (sorted_data(end) - mu_curr) / S_curr;
        T_min = (mu_curr - sorted_data(1)) / S_curr;
        [T_test, check_edge] = max([T_min, T_max]);
        
        if check_edge == 1
            suspect_idx_in_sorted = 1;
        else
            suspect_idx_in_sorted = n_curr;
        end
        
        if n_curr <= 25
            [~, nearest_idx] = min(abs(n_table - n_curr));
            T_limit = t_table(nearest_idx);
        else
            T_limit = interp1(n_table, t_table, min(n_curr, 100), 'linear');
        end
        
        if T_test > T_limit
            remain_indices = find(keep_idx);
            target_global_idx = remain_indices(sorted_origin_idx(suspect_idx_in_sorted));
            keep_idx(target_global_idx) = false;
            removed_count = removed_count + 1;
        else
            break;
        end
    end
    data_clean = data_raw(keep_idx);
end