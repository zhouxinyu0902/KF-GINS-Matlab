function plot_trj(main_file, varargin)
% PLOT_TRJ 绘制多条轨迹对比图
%   输入:
%     main_file - 主轨迹文件（作为参考基准）
%     varargin  - 其他对比轨迹文件（可选，一个或多个）
%
%   示例:
%     plot_trj('truth.txt')
%     plot_trj('truth.txt', 'gps.txt', 'ins.txt')
%     plot_trj('truth.txt', 'gps.txt', 'ins.txt', 'filtered.txt')

glvs; % 加载全局变量

% 处理输入文件
all_files = [{main_file}, varargin];
num_files = length(all_files);

% 预分配单元格数组
data_cell = cell(1, num_files);
pos_cell = cell(1, num_files);
xyz_cell = cell(1, num_files);
time_cell = cell(1, num_files);
% 读取所有文件数据
for i = 1:num_files
    try
        data_cell{i} = importdata(all_files{i});
        if isempty(data_cell{i})
            warning('文件 %s 为空或无法读取', all_files{i});
            continue;
        end
        % 提取经纬度位置（假设第3-5列为位置数据）
        time_cell{i} = data_cell{i}(:, 2);
        pos_cell{i} = data_cell{i}(:, 3:5);
        pos_cell{i}(:,1:2)=d2r(pos_cell{i}(:,1:2));
    catch ME
        warning('无法读取文件 %s: %s', all_files{i}, ME.message);
        continue;
    end
end
%%
for i = 1:num_files
    starttime(i)=time_cell{i}(1);
    endtime(i)=time_cell{i}(end);
end
selectstart=max(starttime);
selectend=min(endtime);

for i=1:num_files
    index0=find(abs(time_cell{i}-selectstart)<0.01);
    indexend=find(abs(time_cell{i}-selectend)<0.01);
    pos_cell{i} = pos_cell{i} (index0(1):indexend(1),:);
    time_cell{i} = time_cell{i} -time_cell{i} (1);
end
%%
% 转换为直角坐标（以第一个文件为参考）
ref_pos = pos_cell{1}(1,:); % 第一个有效数据作为参考
for i = 1:num_files
    xyz_cell{i} = pos2dxyz(pos_cell{i}, ref_pos');
end

% 创建图形
myfigurestartup(5,5,'prese')

% % 子图1: 经纬度轨迹
% subplot 211;
% hold on; grid on; axis equal;
%
% % 定义颜色和线型
colors = lines(num_files);
line_styles = {'-.', ':', '-', '--'};
line_styles = {'-', '-', '-', '-'};
%
% % 绘制每条轨迹
% for i = 1:num_files
%         style_idx = mod(i-1, length(line_styles)) + 1;
%         plot(data_cell{i}(:,4), data_cell{i}(:,3), ...
%              'Color', colors(i,:), ...
%              'LineStyle', line_styles{style_idx}, ...
%              'LineWidth', 1.5);
% end
%
% title('经纬度轨迹对比');
% xlabel('经度 (°)');
% ylabel('纬度 (°)');
% legend(leg, 'Location', 'best');
%
% % 子图2: 直角坐标轨迹
% subplot 212;
hold on; grid on; axis equal;

for i = 1:num_files
    style_idx = mod(i-1, length(line_styles)) + 1;
    plot(xyz_cell{i}(:,1), xyz_cell{i}(:,2), ...
        'Color', colors(i,:), ...
        'LineStyle', line_styles{style_idx}, ...
        'DisplayName', get_file_label(all_files{i}) );

end

title('直角坐标轨迹对比');
xlabel('X坐标 (m)');
ylabel('Y坐标 (m)');
legend('show', 'Location', 'best');

% % 添加统计信息
% if sum(valid_idx) > 1
%     add_statistics_annotation(valid_idx, data_cell, xyz_cell, ref_pos);
% end
end

%% 辅助函数：从文件名提取标签
function label = get_file_label(filename)
[~, name, ~] = fileparts(filename);
% 将常见的文件名映射为更有意义的标签
name_map = containers.Map(...
    {'truth', 'gps', 'ins', 'gnss', 'filter', 'ekf', 'ukf'}, ...
    {'真值', 'GPS', 'INS', 'GNSS', '滤波', 'EKF', 'UKF'}...
    );

if isKey(name_map, lower(name))
    label = name_map(lower(name));
else
    label = name;
end
end

% 辅助函数：添加统计信息注释
function add_statistics_annotation(valid_idx, data_cell, xyz_cell, ref_pos)
% 计算相对于第一个轨迹的误差统计
if sum(valid_idx) < 2 || isempty(xyz_cell{1})
    return;
end

ref_xyz = xyz_cell{1};
stats_text = sprintf('误差统计（相对于参考轨迹）:\n');

for i = 2:length(valid_idx)
    if valid_idx(i) && ~isempty(xyz_cell{i})
        % 计算位置误差
        if size(xyz_cell{i}, 1) == size(ref_xyz, 1)
            errors = sqrt(sum((xyz_cell{i} - ref_xyz).^2, 2));
            max_error = max(errors);
            rms_error = sqrt(mean(errors.^2));

            stats_text = sprintf('%s%s: RMS=%.2fm, Max=%.2fm\n', ...
                stats_text, get_file_label(data_cell{i}.filename), ...
                rms_error, max_error);
        end
    end
end

% 在图上添加文本框
annotation('textbox', [0.02, 0.02, 0.3, 0.1], ...
    'String', stats_text, ...
    'FitBoxToText', 'on', ...
    'BackgroundColor', [1, 1, 0.8], ...
    'EdgeColor', 'none', ...
    'FontSize', 9);
end