clear;
clc;
close all;
%% ========================================================================
% 2025-12-07 罗盘数据读取
%
% 两段罗盘连续数据：
%
% 1. 下午实验
%    2025_12_07_12_56_09_连续.txt
%
% 2. 晚上实验
%    2025_12_07_19_31_38_连续.txt
%
% 当前只做：
%   1. 读取
%   2. 北京时间 -> GPS周秒
%   3. 时间修正
%   4. 打印起止时间和持续时间
%   5. 绘制航向
%
% 暂时不和 AUAX / 830 比较
% ========================================================================
%% 0. 数据目录
data_dir = 'D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1207-longtime\raw';
GPS_WEEK = 2396;
%% 罗盘文件
compass_files = {
    '2025_12_07_12_56_09_连续.txt'
    '2025_12_07_19_31_38_连续.txt'
};
experiment_names = {
    '下午实验'
    '晚上实验'
};
convert_param = [18,0];
TIME_OFFSET = 1;
%% ========================================================================
%% 1. 读取两段罗盘数据
% ========================================================================
Compass = cell(2,1);
fprintf('\n============================================================\n');
fprintf('                    罗盘数据读取\n');
fprintf('============================================================\n');
for k = 1:2
    filename = fullfile(data_dir,compass_files{k});
    fprintf('\n------------------------------------------------------------\n');
    fprintf('%s\n',experiment_names{k});
    fprintf('文件：%s\n',filename);
    fprintf('------------------------------------------------------------\n');
    if ~exist(filename,'file')
        error('文件不存在：%s',filename);
    end
    fid = fopen(filename,'r');
    if fid < 0
        error('无法打开：%s',filename);
    end
    fgets(fid);
    raw = fscanf(fid, ...
        ['%d:%d:%f ', ...
         '%f %f %f %f %f %f %f %f %f ', ...
         '%f %f %f %f %f %f %f %f %f %f %f\n'], ...
        [23,inf]);
    fclose(fid);
    if isempty(raw)
        error('文件没有读取到有效数据：%s',filename);
    end
    n = size(raw,2);
    gps_time = nan(1,n);
    for i = 1:n
        [~,gps_time(i)] = bjt_to_utc_week_seconds( ...
            2025,12,7, ...
            raw(1,i), ...
            raw(2,i), ...
            raw(3,i), ...
            convert_param(k));
    end
    gps_time = gps_time+TIME_OFFSET;
    heading = raw(4,:);
    valid = isfinite(gps_time) & isfinite(heading);
    gps_time = gps_time(valid);
    heading = heading(valid);
    raw = raw(:,valid);
    [gps_time,idx] = sort(gps_time);
    heading = heading(idx);
    raw = raw(:,idx);
    Compass{k}.name = experiment_names{k};
    Compass{k}.time = gps_time;
    Compass{k}.heading = heading;
    Compass{k}.raw = raw;
    t_start = gps_time(1);
    t_end = gps_time(end);
    duration = t_end-t_start;
    fprintf('数据点数：%d\n',length(gps_time));
    fprintf('GPS开始：%.3f\n',t_start);
    fprintf('GPS结束：%.3f\n',t_end);
    fprintf('持续时间：%.3f s = %.2f min = %.3f h\n', ...
        duration,duration/60,duration/3600);
    try
        start_bjt = utc_week_seconds_to_bjt_with_leap(GPS_WEEK,t_start,18);
        end_bjt = utc_week_seconds_to_bjt_with_leap(GPS_WEEK,t_end,18);
        fprintf('北京时间：%s\n',start_bjt);
        fprintf('        ~ %s\n',end_bjt);
    catch
    end
end
%% ========================================================================
%% 2. 最终两次实验时间范围
% ========================================================================
fprintf('\n============================================================\n');
fprintf('                  两次实验候选有效时间段\n');
fprintf('============================================================\n');
for k = 1:2
    t0 = Compass{k}.time(1);
    t1 = Compass{k}.time(end);
    fprintf('\n%s：\n',Compass{k}.name);
    fprintf('  GPS：%.3f ~ %.3f\n',t0,t1);
    fprintf('  持续：%.2f min\n',(t1-t0)/60);
end
%% ========================================================================
%% 3. 分别绘制两段罗盘航向
% ========================================================================
figure('Name','1207 罗盘数据','NumberTitle','off');
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
for k = 1:2
    nexttile;
    t = Compass{k}.time;
    t_relative = t-t(1);
    heading = Compass{k}.heading;
    plot(t_relative,heading,'LineWidth',1);
    grid on;
    xlabel('运行时间 / s');
    ylabel('罗盘航向 / °');
    title(sprintf('%s：%.1f min',Compass{k}.name,(t(end)-t(1))/60));
end
%% ========================================================================
%% 4. 两段数据绝对时间位置
% ========================================================================
figure('Name','罗盘时间覆盖','NumberTitle','off');
hold on;
grid on;
for k = 1:2
    plot([Compass{k}.time(1),Compass{k}.time(end)],[k,k],'LineWidth',6);
end
yticks([1 2]);
yticklabels({'下午实验','晚上实验'});
xlabel('GPS周秒 / s');
title('两段罗盘数据时间覆盖');
%% ========================================================================
%% 5. 保存候选实验时间
% ========================================================================
experiment_time = zeros(2,2);
for k = 1:2
    experiment_time(k,:) = [Compass{k}.time(1),Compass{k}.time(end)];
end
fprintf('\nexperiment_time = \n');
disp(experiment_time);
save_file = 'D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1207-longtime\temp\compass.mat';
save(save_file,'Compass','experiment_time','GPS_WEEK');
fprintf('\n============================================================\n');
fprintf('罗盘数据保存完成\n');
fprintf('============================================================\n');
fprintf('%s\n',save_file);