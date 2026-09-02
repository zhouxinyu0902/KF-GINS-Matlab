function fig=calc_error(testnavpath, truthpath)
% -------------------------------------------------------------------------
% Optimized KF-GINS Error Calculation & Plotting
% -------------------------------------------------------------------------

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
% 去除重复时间戳
[~, unique_idx] = unique(ref(:,1), 'stable');
ref = ref(unique_idx, :);

% 按时间排序
ref = sortrows(ref, 1);

% 插值
newref = interp1( ...
    ref(:,1), ...
    ref(:,2:10), ...
    time, ...
    'linear', ...
    'extrap');

newref = interp1(ref(:, 1), ref(:, 2:10), time, 'linear', 'extrap');

% 计算原始误差
error_raw = newresult - newref;

% 航向角误差修正到 [-180, 180]
error_raw(:, 9) = mod(error_raw(:, 9) + 180, 360) - 180; 

%% 4. 位置误差转换 (BLH -> NED 米)
% 使用 PSINS 标准：位置误差 [dLat, dLon, dH] 转换为 [dN, dE, dD]
param = Param(); % 假设你有这个参数类
error_ned = zeros(size(error_raw, 1), 9);
error_ned(:, 1) = time - time(1); % 相对时间

% 获取动态 RM, RN 以提高长距离精度
for i = 1:size(newref, 1)
    lat = newref(i, 1) * param.D2R;
    h = newref(i, 3);
    [rm, rn] = getRmRn(lat, param);
    
    % 投影矩阵：dLat->North, dLon->East, dH->Up (此处转为Down所以取负)
    DR = [(rm + h), 0, 0;
          0, (rn + h) * cos(lat), 0;
          0, 0, -1]; 
      
    % 位置误差转换 (从第2-4列提取 BLH)
    error_ned(i, 2:4) = (DR * ([error_raw(i, 1:2) * param.D2R,error_raw(i, 3)])')';
end

% 速度和姿态误差直接赋值
error_ned(:, 5:7) = error_raw(:, 4:6); % Velocity NED
error_ned(:, 8:10) = error_raw(:, 7:9); % Attitude

%% 5. 统计指标计算
rms_pos = sqrt(mean(error_ned(:, 2:4).^2));
rms_vel = sqrt(mean(error_ned(:, 5:7).^2));
rms_att = sqrt(mean(error_ned(:, 8:10).^2));
radial_error = sqrt(sum(error_ned(:, 2:3).^2, 2));
rms_radial = sqrt(mean(radial_error.^2));

% CEP 计算
sigma_e = std(error_ned(:, 3));
sigma_n = std(error_ned(:, 2));
CEP = 0.5887 * (sigma_e + sigma_n); % 常用近似公式

%% 6. 统一绘图
fig = myfigurestartup(12, 7, 'prese'); % 假设这是你的自定义绘图初始化
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

% 在图上直接添加统计文本
stats_str = {
    ['Position RMS (NED): ', num2str(rms_pos, '%.2f %.2f %.2f'), ' m'],...
    ['Velocity RMS (NED): ', num2str(rms_vel, '%.3f %.3f %.3f'), ' m/s'],...
    ['Attitude RMS: ', num2str(rms_att, '%.3f %.3f %.3f'), ' deg'],...
    ['CEP (50%): ', num2str(CEP, '%.2f'), ' m'],...
    ['Radial RMS: ', num2str(rms_radial, '%.2f'), ' m']
};
% % 修改后的位置参数 [左, 下, 宽, 高]
% % [0.35, 0.01, 0.3, 0.08] 表示水平居中(0.5-0.3/2)，贴近底部
% annotation('textbox', [0.35, 0.01, 0.3, 0.08], ...
%     'String', stats_str, ...
%     'Interpreter', 'none', ...
%     'FontSize', 9, ...
%     'FontName', 'TimesSimSun', ... % 使用等宽字体对齐数字
%     'HorizontalAlignment', 'center', ...
%     'VerticalAlignment', 'middle', ...
%     'FitBoxToText', 'on', ...
%     'BackgroundColor', [0.94, 0.94, 0.94], ... % 浅灰色背景更显高级
%     'EdgeColor', 'k');

%% 7. 终端打印
fprintf('\n----- 导航误差统计结果 -----\n');
fprintf('位置 RMS (N E D) : %8.3f %8.3f %8.3f m\n', rms_pos);
fprintf('速度 RMS (N E D) : %8.3f %8.3f %8.3f m/s\n', rms_vel);
fprintf('姿态 RMS (R P Y) : %8.3f %8.3f %8.3f deg\n', rms_att);
fprintf('水平径向 RMS     : %8.3f m\n', rms_radial);
fprintf('CEP (50%%)        : %8.3f m (%.4f nmi)\n', CEP, CEP/1852);
fprintf('----------------------------\n');

end