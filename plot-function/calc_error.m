% % -------------------------------------------------------------------------
% % KF-GINS-Matlab: An EKF-based GNSS/INS Integrated Navigation System in Matlab
% %
% % Copyright (C) 2024, i2Nav Group, Wuhan University
% %
% %  Author : Liqiang Wang
% % Contact : wlq@whu.edu.cn
% %    Date : 2022.11.30
% % -------------------------------------------------------------------------
% 
% function calc_error(testnavpath,truthpath)
% %% importdata data （nav文件第一列为0，不需要用）
% % testnavpath = 'dataset1/pure_ins_PSINS.txt';
% % testnavpath = 'dataset1/pcafile.txt';
% % testnavpath = 'dataset1/pure_ins.txt';
% % testnavpath = 'dataset1/NavResult.nav';
% % truthpath = 'dataset1/truth.nav';
% temp = importdata(testnavpath);
% result_all = temp(:, 2:end);
% temp=importdata(truthpath);
% ref = temp(:, 2:end);
% 
% %% heading angle smoothing, 航向角平滑
% for i=2:size(result_all, 1)
%     if (result_all(i,10) - result_all(i-1, 10)) < -180
%         result_all(i:end, 10) = result_all(i:end, 10) + 360;
%     end
%     if (result_all(i,10) - result_all(i-1, 10)) > 180
%         result_all(i:end, 10) = result_all(i:end, 10) - 360;
%     end
% end
% 
% for i=2:size(ref, 1)
%     if (ref(i,10) - ref(i-1, 10)) < -180
%         ref(i:end, 10) = ref(i:end, 10) + 360;
%     end
%     if (ref(i,10) - ref(i-1, 10)) > 180
%         ref(i:end, 10) = ref(i:end, 10) - 360;
%     end
% end
% 
% 
% %% find overlerpping data, 找到数据重合部分
% res_start = result_all(1, 1);
% res_end = result_all(end, 1);
% ref_start = ref(1, 1);
% ref_end = ref(end, 1);
% 
% if (ref_start > res_start)
%     starttime = ref_start;
% else 
%     starttime = res_start;
% end
% if (res_end > ref_end)
%     endtime = ref_end;
% else 
%     endtime = res_end;
% end
% 
% % get interpolate time, 按照采样间隔取合适的时间
% dt = mean(diff(result_all(:, 1)));
% time = starttime:dt:endtime;
% time = time';
% %%
% 
% 
% %% interpolate navresult, 测试结果和参考结果内插到同样的时刻，然后求差
% newresult = zeros(size(time, 1), 10);
% newref = zeros(size(time, 1), 10);
% error = zeros(size(time, 1), 10);
% newresult(:, 1) = time;
% newref(:, 1) = time;
% error(: ,1) = time-time(1);
% 
% newresult(:, 2:10) = interp1(result_all(:, 1), result_all(:, 2:10), time);
% newref(:, 2:10) = interp1(ref(:, 1), ref(:, 2:10), time);
% error(:, 2:10) = newresult(:, 2:10) - newref(:, 2:10);
% 
% % check heading error, 航向角误差处理
% for i = 1:size(error, 1)
%     if error(i, 10) > 180
%         error(i, 10) = error(i, 10) - 360;
%     end
%     if error(i, 10) < -180
%         error(i, 10) = error(i, 10) + 360;
%     end
% end
% 
% %% delta rad to delat position in n frame, 位置误差转到ned
% param = Param();
% first_blh = result_all(1, 2:4);
% [rm, rn] = getRmRn(first_blh(1) * param.D2R, param);
% h = first_blh(3);
% DR = diag([rm + h, (rn + h)*cos(first_blh(1) * param.D2R), -1]);
% 
% error(:, 2:3) = error(:, 2:3) * param.D2R;
% for i = 1:size(error, 1)
%     delta_pos = DR * (error(i, 2:4)');
%     error(i, 2:4) = delta_pos';
% end
% 
% 
% 
% %% plot error
% myfigurestartup(12,7,'prese')
% subplot 221
% plot(error(:,1),error(:,[4,2,3]));
% title('Position Error');
% xlabel('Time[s]');
% ylabel('Error[m]');
% legend( 'Down','North', 'East');
% grid("on");
% xlim([error(1,1) error(end,1)])
% 
% % figure;
% subplot 222
% plot(error(:,1),error(:,[7,5,6]));
% title('Velocity Error');
% xlabel('Time[s]');
% ylabel('Error[m/s]');
% legend('Down','North', 'East');
% grid("on");
% xlim([error(1,1) error(end,1)])
% 
% subplot 223
% plot(error(:,1),error(:,8:10));
% title('Attitude Error');
% xlabel('Time[s]');
% ylabel('Error[deg]');
% legend('Roll', 'Pitch', 'Yaw');
% grid("on");
% xlim([error(1,1) error(end,1)])
% 
% % 画位置误差
% subplot 224
% RadiusError=sqrt(sum(error(:,2:3).^2,2));
% plot(error(:,1),RadiusError);
% title('Radial Error');
% xlabel('Time[s]');
% ylabel('Error[m]');
% xlim([error(1,1) error(end,1)])
% grid("on");
% %% display error RMS, 输出误差
% disp('-----均方根误差RMS------')
% temp = error(:, 2:4);
% disp("位置误差（NED）: " + num2str(sqrt(mean(temp .^2))) + " m");
% temp = error(:, 5:7);
% disp("速度误差（NED）: " + num2str(sqrt(mean(temp .^2))) + " m/s");
% temp = error(:, 8:10);
% disp("姿态误差: " + num2str(sqrt(mean(temp .^2))) + " deg");
% temp = RadiusError;
% disp("径向误差: " + num2str(sqrt(mean(temp .^2))) + " m");
% %% 输出定位CEP误差
% disp('-----圆概率径向误差CEP-----')
% sigma_x = std(error(:,3));
% sigma_y = std(error(:,2));
% if (sigma_x == sigma_y) && (abs(corr(error(:,3),error(:,2))) <0.1)
%     CEP = 1.1774 * sigma_x;
% else
%     % CEP = 0.5887 * (sigma_x + sigma_y);
%     CEP = 1.1774*sqrt(sigma_x^2 + sigma_y^2);
% end
% output=sprintf("CEP误差：%.2f m, %.2f 海里",CEP,CEP/1852);
% disp(output)
% output3=sprintf("非正态分布CEP误差（取中位数）：  %.2f m,%.2f 海里",median(RadiusError),median(RadiusError)/1852);
% disp(output3)
% disp('---------------------------')
% 
% 
% 
% 
% % %% 找到共同的开始时间点
% % for m=1:size(ref,1)
% %     for n=1:size(result_all,1)
% %         if(abs(ref(m,1)-result_all(n,1))<0.001) 
% %             break;
% %         end
% %     end
% %     break;
% % end
% % 
% % %% 取相同时间段
% % ref = ref(m:end, :);
% % result_all = result_all(n:end, :);
% % result_size = size(result_all, 1);
% % ref_size = size(ref, 1);
% % comsize = min(result_size, ref_size);
% % com=zeros(comsize, 10);
% % 
% % x = 1;
% % m = 1;
% % n = 1;
% % while(1)
% %    com(x,2:10)=result_all(n,2:10)-ref(m,2:10);
% %    com(x,1)=ref(m,1);
% %    x=x+1;
% %    n=n+1;
% %    m=m+1;
% %    if((n>result_size)||(m>ref_size))
% %        break;
% %    end
% % end
% % for i=1:size(com,1)
% %     if(com(i,10)>180)
% %         com(i,10)=com(i,10)-360;
% %     end
% %     if(com(i,10)<-180)
% %         com(i,10)=com(i,10)+360;
% %     end
% % end


function calc_error(testnavpath, truthpath)
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
myfigurestartup(12, 7, 'prese'); % 假设这是你的自定义绘图初始化
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
% 修改后的位置参数 [左, 下, 宽, 高]
% [0.35, 0.01, 0.3, 0.08] 表示水平居中(0.5-0.3/2)，贴近底部
annotation('textbox', [0.35, 0.01, 0.3, 0.08], ...
    'String', stats_str, ...
    'Interpreter', 'none', ...
    'FontSize', 9, ...
    'FontName', 'Consolas', ... % 使用等宽字体对齐数字
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'middle', ...
    'FitBoxToText', 'on', ...
    'BackgroundColor', [0.94, 0.94, 0.94], ... % 浅灰色背景更显高级
    'EdgeColor', 'k');

%% 7. 终端打印
fprintf('\n----- 导航误差统计结果 -----\n');
fprintf('位置 RMS (N E D) : %8.3f %8.3f %8.3f m\n', rms_pos);
fprintf('速度 RMS (N E D) : %8.3f %8.3f %8.3f m/s\n', rms_vel);
fprintf('姿态 RMS (R P Y) : %8.3f %8.3f %8.3f deg\n', rms_att);
fprintf('水平径向 RMS     : %8.3f m\n', rms_radial);
fprintf('CEP (50%%)        : %8.3f m (%.4f nmi)\n', CEP, CEP/1852);
fprintf('----------------------------\n');

end