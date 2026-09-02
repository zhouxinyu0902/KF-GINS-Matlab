clear;
clc;
close all;
%% ========================================================================
% AUAX 与罗盘航向对比分析
%
% 数据处理流程：
%
% AUAX:
%   7个 dat
%       ↓
%   全部读取并合并
%       ↓
%   删除明显异常数据
%       ↓
%   根据 430.010 / 430.015 找实验边界
%       ↓
%   每段仅保留状态字 0x4300
%       ↓
%   得到若干导航段
%       ↓
%   选择第 2、4 段
%
% Compass:
%   多个连续 txt
%       ↓
%   全部读取
%       ↓
%   转 GPS 周秒
%       ↓
%   时间修正
%       ↓
%   合并、排序
%       ↓
%   重复时刻圆周平均
%
% Comparison:
%   以 AUAX 完整时间轴为基准
%       ↓
%   找罗盘重叠时间
%       ↓
%   罗盘插值到 AUAX 时间
%       ↓
%   统一航向角定义
%       ↓
%   罗盘 - AUAX
%
% ========================================================================
%% ========================================================================
%% 0. 参数配置
% ========================================================================
root_dir = ...
    'D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1205-compass';
gps_week = 2395;
% -------------------------------------------------------------------------
% AUAX 文件
% -------------------------------------------------------------------------
auax_file_num = 7;
% AUAX 导航状态
NAV_STATUS = hex2dec('4300');      % 17152
% 用于区分不同实验段的标识
SEGMENT_VALUES = [430.010, 430.015];
% 浮点比较容差
SEGMENT_TOL = 1e-6;
% 最终人工确认的有效实验段
VALID_SEGMENTS = [2, 4];
% 原代码中用于去除异常 AUAX 数据的条件
MIN_VALID_LATITUDE = 36.36;
% -------------------------------------------------------------------------
% 罗盘文件
% -------------------------------------------------------------------------
compass_files = {
    '2025_12_05_15_07_10_连续.txt'
    '2025_12_05_16_25_03_连续.txt'
    '2025_12_05_17_21_45_连续.txt'
    };
% -------------------------------------------------------------------------
% 保持你原代码中的时间转换参数
%
% 文件1:
% bjt_to_utc_week_seconds(...,18)
%
% 文件2:
% bjt_to_utc_week_seconds(...,0)
%
% 文件3:
% bjt_to_utc_week_seconds(...,18)
%
% -------------------------------------------------------------------------
compass_convert_param = [18, 0, 18];
% 原程序三个文件转换后最终都进行了 -21 s
COMPASS_POST_OFFSET = -21;
% 给罗盘截取时留出的边界裕量
INTERP_MARGIN = 1.0;
%% ========================================================================
%% 1. 读取全部 AUAX
% ========================================================================
fprintf('\n');
fprintf('============================================================\n');
fprintf('1. 读取 AUAX 数据\n');
fprintf('============================================================\n');
AUAX_raw = cell(auax_file_num,1);
for k = 1:auax_file_num
    filename = fullfile( ...
        root_dir, ...
        sprintf('Disk1_00%d.dat',k-1));
    fprintf('[%d/%d] %s\n', ...
        k,auax_file_num,filename);
    AUAX_raw{k} = read_auax_120(filename);
end
fprintf('\nAUAX 文件读取完成。\n');
%% ========================================================================
%% 2. 合并 AUAX
% ========================================================================
AUAX_all = [AUAX_raw{:}];
fprintf('\n');
fprintf('============================================================\n');
fprintf('2. AUAX 合并\n');
fprintf('============================================================\n');
fprintf('合并前总历元数：%d\n',size(AUAX_all,2));
%% ========================================================================
%% 3. 删除明显异常的 AUAX 数据
% ========================================================================
invalid_idx = ...
    ~isfinite(AUAX_all(6,:)) | ...
    AUAX_all(6,:) < MIN_VALID_LATITUDE;
fprintf('删除异常历元数：%d\n',sum(invalid_idx));
AUAX_all(:,invalid_idx) = [];
fprintf('清理后总历元数：%d\n',size(AUAX_all,2));
%% ========================================================================
%% 4. 查找 AUAX 实验分段起点
% ========================================================================
fprintf('\n');
fprintf('============================================================\n');
fprintf('3. AUAX 数据分段\n');
fprintf('============================================================\n');
segment_flag = false(1,size(AUAX_all,2));
for value = SEGMENT_VALUES
    segment_flag = segment_flag | ...
        abs(AUAX_all(1,:) - value) < SEGMENT_TOL;
end
segment_candidate = find(segment_flag);
% -------------------------------------------------------------------------
% 如果 430.010 / 430.015 连续出现多个历元，只把连续的一组认为一次启动
% -------------------------------------------------------------------------
segment_start_idx = keep_first_of_consecutive(segment_candidate);
fprintf('检测到 %d 个实验启动标志：\n', ...
    length(segment_start_idx));
disp(segment_start_idx);
%% ========================================================================
%% 5. 将 AUAX 分成多个实验段
% ========================================================================
% 状态字所在行
status_row = size(AUAX_all,1) - 2;
% 这里最后一个启动点之后的数据也作为一个段
segment_edge = [segment_start_idx, size(AUAX_all,2)+1];
n_segment = length(segment_start_idx);
AUAX_segments = cell(n_segment,1);
for k = 1:n_segment
    idx_start = segment_edge(k);
    idx_end   = segment_edge(k+1)-1;
    segment_tmp = AUAX_all(:,idx_start:idx_end);
    % ---------------------------------------------------------------
    % 仅保留导航状态 0x4300
    % ---------------------------------------------------------------
    nav_idx = ...
        segment_tmp(status_row,:) == NAV_STATUS;
    segment_tmp = segment_tmp(:,nav_idx);
    AUAX_segments{k} = segment_tmp;
    if isempty(segment_tmp)
        fprintf('AUAX 第 %d 段：无 0x4300 导航数据\n',k);
    else
        fprintf( ...
            'AUAX 第 %d 段：%7d 历元，时间 %.3f ~ %.3f\n', ...
            k, ...
            size(segment_tmp,2), ...
            segment_tmp(2,1), ...
            segment_tmp(2,end));
    end
end
%% ========================================================================
%% 6. 绘制所有 AUAX 分段轨迹
% ========================================================================
figure( ...
    'Name','AUAX 全部导航分段', ...
    'NumberTitle','off');
n_col = 2;
n_row = ceil(n_segment/n_col);
tiledlayout( ...
    n_row,n_col, ...
    'TileSpacing','compact', ...
    'Padding','compact');
for k = 1:n_segment
    nexttile;
    hold on;
    grid on;
    axis equal;
    data = AUAX_segments{k};
    if isempty(data)
        title(sprintf('AUAX 第 %d 段：无数据',k));
        continue;
    end
    plot( ...
        data(7,:), ...
        data(6,:), ...
        'LineWidth',1);
    plot( ...
        data(7,1), ...
        data(6,1), ...
        '*', ...
        'MarkerSize',10);
    plot( ...
        data(7,end), ...
        data(6,end), ...
        'o', ...
        'MarkerSize',8);
    xlabel('Longitude');
    ylabel('Latitude');
    title(sprintf('AUAX 第 %d 段',k));
end
%% ========================================================================
%% 7. 选择有效 AUAX 实验段
% ========================================================================
if any(VALID_SEGMENTS > length(AUAX_segments))
    error( ...
        'VALID_SEGMENTS 超出当前分段数量。当前只有 %d 段。', ...
        length(AUAX_segments));
end
AUAX_valid = AUAX_segments(VALID_SEGMENTS);
fprintf('\n');
fprintf('============================================================\n');
fprintf('4. 最终选择的有效 AUAX 数据\n');
fprintf('============================================================\n');
for k = 1:length(AUAX_valid)
    data = AUAX_valid{k};
    if isempty(data)
        continue;
    end
    fprintf( ...
        '实验 %d = AUAX 第 %d 段：%.3f ~ %.3f，时长 %.3f s\n', ...
        k, ...
        VALID_SEGMENTS(k), ...
        data(2,1), ...
        data(2,end), ...
        data(2,end)-data(2,1));
end
%% ========================================================================
%% 8. 单独绘制选中的有效 AUAX 轨迹
% ========================================================================
figure( ...
    'Name','有效 AUAX 轨迹', ...
    'NumberTitle','off');
tiledlayout( ...
    1,length(AUAX_valid), ...
    'TileSpacing','compact', ...
    'Padding','compact');
for k = 1:length(AUAX_valid)
    nexttile;
    hold on;
    grid on;
    axis equal;
    data = AUAX_valid{k};
    plot( ...
        data(7,:), ...
        data(6,:), ...
        'LineWidth',1.2);
    plot( ...
        data(7,1), ...
        data(6,1), ...
        '*', ...
        'MarkerSize',10);
    plot( ...
        data(7,end), ...
        data(6,end), ...
        'o', ...
        'MarkerSize',8);
    xlabel('Longitude');
    ylabel('Latitude');
    title(sprintf( ...
        '实验 %d：AUAX 第 %d 段', ...
        k,VALID_SEGMENTS(k)));
end
%% ========================================================================
%% 9. 读取全部罗盘文件
% ========================================================================
fprintf('\n');
fprintf('============================================================\n');
fprintf('5. 读取罗盘数据\n');
fprintf('============================================================\n');
Compass_raw = cell(length(compass_files),1);
for k = 1:length(compass_files)
    filename = fullfile( ...
        root_dir, ...
        compass_files{k});
    Compass_raw{k} = read_compass_data( ...
        filename, ...
        2025,12,5, ...
        compass_convert_param(k), ...
        COMPASS_POST_OFFSET);
    fprintf( ...
        '罗盘文件 %d：%7d 历元，%.3f ~ %.3f，时长 %.3f s\n', ...
        k, ...
        size(Compass_raw{k},2), ...
        Compass_raw{k}(1,1), ...
        Compass_raw{k}(1,end), ...
        Compass_raw{k}(1,end)-Compass_raw{k}(1,1));
end
%% ========================================================================
%% 10. 合并罗盘
% ========================================================================
Compass_all = [Compass_raw{:}];
% 删除非法数据
valid_idx = ...
    isfinite(Compass_all(1,:)) & ...
    isfinite(Compass_all(2,:));
Compass_all = Compass_all(:,valid_idx);
% 按时间排序
[~,idx_sort] = sort(Compass_all(1,:));
Compass_all = Compass_all(:,idx_sort);
%% ========================================================================
%% 11. 合并重复时间
%
% 使用圆周平均：
%
% 359° + 1° → 0°
%
% 而不是普通平均的 180°
% ========================================================================
time_raw    = Compass_all(1,:);
heading_raw = mod(Compass_all(2,:),360);
[time_unique,~,group_idx] = unique(time_raw);
mean_sin = accumarray( ...
    group_idx(:), ...
    sind(heading_raw(:)), ...
    [], ...
    @mean);
mean_cos = accumarray( ...
    group_idx(:), ...
    cosd(heading_raw(:)), ...
    [], ...
    @mean);
heading_unique = atan2d(mean_sin,mean_cos);
heading_unique = mod(heading_unique,360);
Compass_all = [ ...
    time_unique(:)'; ...
    heading_unique(:)'];
fprintf('\n');
fprintf('罗盘合并完成：%d 个唯一时间历元\n', ...
    size(Compass_all,2));
fprintf( ...
    '罗盘总时间范围：%.3f ~ %.3f s\n', ...
    Compass_all(1,1), ...
    Compass_all(1,end));
%% ========================================================================
%% 12. 绘制全部罗盘原始航向
% ========================================================================
figure( ...
    'Name','完整罗盘数据', ...
    'NumberTitle','off');
plot( ...
    Compass_all(1,:) - Compass_all(1,1), ...
    Compass_all(2,:), ...
    'LineWidth',1);
grid on;
xlabel('相对时间 / s');
ylabel('罗盘航向 / °');
title('合并后的完整罗盘航向');
ylim([0 360]);
%% ========================================================================
%% 13. AUAX 与罗盘时间匹配
% ========================================================================
Compare = cell(length(AUAX_valid),1);
fprintf('\n');
fprintf('============================================================\n');
fprintf('6. AUAX 与罗盘匹配\n');
fprintf('============================================================\n');
for k = 1:length(AUAX_valid)
    AUAX_tmp = AUAX_valid{k};
    %% --------------------------------------------------------------------
    % AUAX 完整时间
    % ---------------------------------------------------------------------
    t_auax = AUAX_tmp(2,:);
    t0 = t_auax(1);
    t1 = t_auax(end);
    %% --------------------------------------------------------------------
    % AUAX 航向
    %
    % 你原来的转换：
    %
    % >0 : 360-angle
    % <0 : -angle
    %
    % 等价于：
    %
    % mod(-angle,360)
    %
    % ---------------------------------------------------------------------
    heading_auax_raw = AUAX_tmp(3,:);
    heading_auax = mod(-heading_auax_raw,360);
    %% --------------------------------------------------------------------
    % 寻找和该 AUAX 实验段相关的罗盘范围
    % ---------------------------------------------------------------------
    compass_idx = ...
        Compass_all(1,:) >= t0-INTERP_MARGIN & ...
        Compass_all(1,:) <= t1+INTERP_MARGIN;
    Compass_tmp = Compass_all(:,compass_idx);
    fprintf('\n');
    fprintf('---------------- 实验 %d ----------------\n',k);
    fprintf( ...
        'AUAX 第 %d 段\n', ...
        VALID_SEGMENTS(k));
    fprintf( ...
        'AUAX 时间：      %.3f ~ %.3f，时长 %.3f s\n', ...
        t0,t1,t1-t0);
    if isempty(Compass_tmp)
        fprintf('该实验时间内无罗盘数据。\n');
        heading_compass_interp = ...
            nan(size(t_auax));
    else
        fprintf( ...
            '罗盘候选时间：  %.3f ~ %.3f\n', ...
            Compass_tmp(1,1), ...
            Compass_tmp(1,end));
        %% ----------------------------------------------------------------
        % 罗盘航向 unwrap 后插值
        % -----------------------------------------------------------------
        compass_heading_unwrap = ...
            unwrap(deg2rad(Compass_tmp(2,:)));
        heading_compass_interp_rad = interp1( ...
            Compass_tmp(1,:), ...
            compass_heading_unwrap, ...
            t_auax, ...
            'linear', ...
            NaN);
        heading_compass_interp = ...
            mod(rad2deg(heading_compass_interp_rad),360);
    end
    %% --------------------------------------------------------------------
    % 航向差
    %
    % 罗盘 - AUAX
    %
    % 限制到 [-180,180)
    % ---------------------------------------------------------------------
    dheading = wrap180_local( ...
        heading_compass_interp - heading_auax);
    %% --------------------------------------------------------------------
    % 找有效重叠区
    % ---------------------------------------------------------------------
    valid_compare = ...
        isfinite(heading_auax) & ...
        isfinite(heading_compass_interp);
    if any(valid_compare)
        first_valid = find(valid_compare,1,'first');
        last_valid  = find(valid_compare,1,'last');
        fprintf( ...
            '有效重叠时间：  %.3f ~ %.3f\n', ...
            t_auax(first_valid), ...
            t_auax(last_valid));
        fprintf( ...
            '相对 AUAX 起点：%.3f ~ %.3f s\n', ...
            t_auax(first_valid)-t0, ...
            t_auax(last_valid)-t0);
        fprintf( ...
            '有效对比点：    %d / %d\n', ...
            sum(valid_compare), ...
            length(valid_compare));
        diff_valid = dheading(valid_compare);
        fprintf( ...
            '航向差均值：    %.4f deg\n', ...
            mean(diff_valid));
        fprintf( ...
            '航向差标准差：  %.4f deg\n', ...
            std(diff_valid));
        fprintf( ...
            '航向差 RMS：    %.4f deg\n', ...
            sqrt(mean(diff_valid.^2)));
    else
        fprintf('没有有效的 AUAX / 罗盘重叠数据。\n');
    end
    %% --------------------------------------------------------------------
    % 保存
    %
    % 注意：
    % 保存的是完整 AUAX 时间轴。
    % 无罗盘覆盖的位置 heading_compass 和 dheading 都为 NaN。
    % ---------------------------------------------------------------------
    Compare{k}.segment_index = VALID_SEGMENTS(k);
    Compare{k}.time_abs = t_auax;
    Compare{k}.time = t_auax - t0;
    Compare{k}.time_start = t0;
    Compare{k}.heading_auax = heading_auax;
    Compare{k}.heading_compass = ...
        heading_compass_interp;
    Compare{k}.dheading = dheading;
    Compare{k}.valid = valid_compare;
end
%% ========================================================================
%% 14. 分别绘制每个实验
% ========================================================================
for k = 1:length(Compare)
    result = Compare{k};
    t = result.time;
    figure( ...
        'Name',sprintf('实验 %d 航向对比',k), ...
        'NumberTitle','off');
    tiledlayout( ...
        2,1, ...
        'TileSpacing','compact', ...
        'Padding','compact');
    %% --------------------------------------------------------------------
    % 航向对比
    % ---------------------------------------------------------------------
    nexttile;
    hold on;
    grid on;
    plot( ...
        t, ...
        result.heading_auax, ...
        'LineWidth',1);
    plot( ...
        t, ...
        result.heading_compass, ...
        'LineWidth',1);
    xlabel('AUAX 运行时间 / s');
    ylabel('航向角 / °');
    title(sprintf( ...
        '实验 %d：AUAX 第 %d 段', ...
        k,result.segment_index));
    legend( ...
        '120 AUAX', ...
        '罗盘', ...
        'Location','best');
    xlim([0 t(end)]);
    ylim([0 360]);
    %% --------------------------------------------------------------------
    % 航向差
    % ---------------------------------------------------------------------
    nexttile;
    hold on;
    grid on;
    plot( ...
        t, ...
        result.dheading, ...
        'LineWidth',1);
    yline(0,'--');
    xlabel('AUAX 运行时间 / s');
    ylabel('\Delta\psi / °');
    title('罗盘 - AUAX');
    xlim([0 t(end)]);
end
%% ========================================================================
%% 15. 展开航向角后再画一次
%
% 这个图更适合检查两套航向是否存在：
%
% 1. 时间错位
% 2. 方向定义错误
% 3. 比例/符号异常
%
% ========================================================================
for k = 1:length(Compare)
    result = Compare{k};
    t = result.time;
    heading_auax_unwrap = ...
        rad2deg(unwrap(deg2rad(result.heading_auax)));
    heading_compass_unwrap = ...
        nan(size(result.heading_compass));
    valid = result.valid;
    % 只对连续有效的罗盘数据进行 unwrap
    valid_group = find_continuous_groups(valid);
    for g = 1:length(valid_group)
        idx = valid_group{g};
        heading_compass_unwrap(idx) = ...
            rad2deg( ...
            unwrap( ...
            deg2rad(result.heading_compass(idx))));
    end
    figure( ...
        'Name',sprintf('实验 %d 展开航向',k), ...
        'NumberTitle','off');
    hold on;
    grid on;
    plot( ...
        t, ...
        heading_auax_unwrap, ...
        'LineWidth',1);
    plot( ...
        t, ...
        heading_compass_unwrap, ...
        'LineWidth',1);
    xlabel('AUAX 运行时间 / s');
    ylabel('展开航向角 / °');
    title(sprintf( ...
        '实验 %d：展开后的航向变化',k));
    legend( ...
        '120 AUAX', ...
        '罗盘', ...
        'Location','best');
    xlim([0 t(end)]);
end
%% ========================================================================
%% 16. 两个实验的航向差放在一起
% ========================================================================
figure( ...
    'Name','所有实验航向误差', ...
    'NumberTitle','off');
hold on;
grid on;
for k = 1:length(Compare)
    result = Compare{k};
    plot( ...
        result.time, ...
        result.dheading, ...
        'LineWidth',1, ...
        'DisplayName', ...
        sprintf( ...
        '实验 %d（AUAX段 %d）', ...
        k,result.segment_index));
end
xlabel('各实验 AUAX 运行时间 / s');
ylabel('罗盘 - AUAX / °');
title('不同实验航向误差');
legend('Location','best');
%% ========================================================================
%% 17. 航向相关误差
%
% 后续用于分析罗盘误差随航向角的规律
% ========================================================================
figure( ...
    'Name','航向相关误差', ...
    'NumberTitle','off');
hold on;
grid on;
for k = 1:length(Compare)
    result = Compare{k};
    valid = result.valid;
    plot( ...
        result.heading_auax(valid), ...
        result.dheading(valid), ...
        '.', ...
        'DisplayName', ...
        sprintf('实验 %d',k));
end
xlabel('AUAX 航向 / °');
ylabel('罗盘 - AUAX / °');
xlim([0 360]);
title('航向角与罗盘误差关系');
legend('Location','best');
%% ========================================================================
%% 18. AUAX / 罗盘时间覆盖关系
%
% 这张图专门检查时间是否匹配。
%
% 横轴统一使用 GPS 周秒。
% ========================================================================
figure( ...
    'Name','数据时间覆盖关系', ...
    'NumberTitle','off');
hold on;
grid on;
% 罗盘总体范围
y_compass = 1;
plot( ...
    [Compass_all(1,1),Compass_all(1,end)], ...
    [y_compass,y_compass], ...
    'LineWidth',6);
for k = 1:length(AUAX_valid)
    data = AUAX_valid{k};
    y_auax = k+1;
    plot( ...
        [data(2,1),data(2,end)], ...
        [y_auax,y_auax], ...
        'LineWidth',6);
end
yticks(1:length(AUAX_valid)+1);
label_text = cell(1,length(AUAX_valid)+1);
label_text{1} = '罗盘';
for k = 1:length(AUAX_valid)
    label_text{k+1} = ...
        sprintf('AUAX 实验%d',k);
end
yticklabels(label_text);
xlabel('GPS 周秒 / s');
title('AUAX 与罗盘时间覆盖关系');
%% ========================================================================
%% 局部函数
% ========================================================================
function Compass = read_compass_data( ...
    filename, ...
    year,month,day, ...
    convert_param, ...
    post_offset)
%READ_COMPASS_DATA 读取一个罗盘连续数据文件
%
% 输出：
%
% Compass(1,:) : GPS 周秒
% Compass(2,:) : 航向角 / deg
    fid = fopen(filename,'r');
    if fid < 0
        error('无法打开罗盘文件：%s',filename);
    end
    % 跳过文件第一行
    fgets(fid);
    % 与原始代码相同：
    %
    % HH:MM:SS.xxx + 20个数据
    %
    raw = fscanf( ...
        fid, ...
        ['%d:%d:%f ', ...
         '%f %f %f %f %f %f %f %f %f ', ...
         '%f %f %f %f %f %f %f %f %f %f %f\n'], ...
        [23,inf]);
    fclose(fid);
    if isempty(raw)
        error('罗盘文件为空或读取格式错误：%s',filename);
    end
    n = size(raw,2);
    gps_time = nan(1,n);
    for i = 1:n
        [~,gps_time(i)] = ...
            bjt_to_utc_week_seconds( ...
            year, ...
            month, ...
            day, ...
            raw(1,i), ...
            raw(2,i), ...
            raw(3,i), ...
            convert_param);
    end
    % 原始程序中的额外时间修正
    gps_time = gps_time + post_offset;
    % 第4行是航向
    heading = raw(4,:);
    Compass = [ ...
        gps_time; ...
        heading];
end
function idx_new = keep_first_of_consecutive(idx)
%KEEP_FIRST_OF_CONSECUTIVE
%
% 例如：
%
% idx = [100 101 102 300 301 500]
%
% 输出：
%
% [100 300 500]
    if isempty(idx)
        idx_new = [];
        return;
    end
    new_group = ...
        [true, diff(idx) > 1];
    idx_new = idx(new_group);
end
function angle = wrap180_local(angle)
%WRAP180_LOCAL
%
% 将角度转换到：
%
% [-180,180)
    angle = mod(angle + 180,360) - 180;
end
function groups = find_continuous_groups(mask)
%FIND_CONTINUOUS_GROUPS
%
% 将逻辑数组中连续为 true 的部分分别提取出来。
    mask = logical(mask(:)');
    edge = diff([false,mask,false]);
    idx_start = find(edge == 1);
    idx_end = find(edge == -1)-1;
    groups = cell(length(idx_start),1);
    for i = 1:length(idx_start)
        groups{i} = ...
            idx_start(i):idx_end(i);
    end
end