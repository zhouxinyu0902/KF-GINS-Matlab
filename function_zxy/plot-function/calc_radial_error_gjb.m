function [fig,finalExcelData] = calc_radial_error_gjb(truthpath, varargin)
% CALC_RADIAL_ERROR_GJB 计算并绘制多个导航结果的水平径向误差。
%
% 输入:
%   truthpath - 真值文件路径
%   varargin  - 一个或多个测试结果文件路径。
%               最后一个参数若为 logical 类型，则作为 apply_grubbs 开关。
%
% 输出统计量（单次试验）:
%   RMS    - 评价时段内水平径向误差均方根，单位 m；
%   Max    - 评价时段内最大水平径向误差，单位 m；
%   RER    - 评价终点水平径向误差除以累计运行时间，单位 n mile/h；
%   Median - 评价时段内水平径向误差中位数，单位 m。
%
% 说明:
%   评价时长最多取前3600 s；不足3600 s时使用全部公共有效数据。
%   本函数不计算R50。R50应由多次独立试验各自的RER另行统计。
%
% 调用示例:
%   calc_radial_error_gjb('truth.txt', 'test1.txt', 'test2.txt')
%   calc_radial_error_gjb('truth.txt', 'test1.txt', 'test2.txt', true)
    %% 1. 参数检查和初始化
    if nargin < 2
        error('至少需要提供真值文件和一个测试结果文件');
    end
    % 智能解析最后一个参数是否为剔除开关
    apply_grubbs = false; 
    if islogical(varargin{end})
        apply_grubbs = varargin{end};
        varargin(end) = []; % 把布尔值弹出去，剩下的全是文件路径
    end
    if isempty(varargin)
        error('未检测到测试结果文件路径');
    end
    test_files = varargin;
    num_tests = length(test_files);
    % 绘图样式定义
    colors = lines(num_tests); 
    colors = [
    0.00, 0.00, 0.00;  % 纯黑：最通用的基准色，黑白打印也完全清晰
    0.85, 0.15, 0.15;  % 正酒红：低饱和红色，不会刺眼且辨识度拉满
    0.10, 0.35, 0.70;  % 深海蓝：顶刊使用频率最高的主色，视觉柔和不发飘
    0.10, 0.60, 0.20;  % 深墨绿：和蓝、红的色差极大，几乎不会混淆
    0.60, 0.20, 0.70;  % 暗紫：低饱和紫色，和前四种颜色边界清晰
    0.95, 0.50, 0.10   % 深橙：替代容易和白色混淆的明黄，对比度充足
    % 0.900, 0.520, 0.550;   % wine red, light
    % 0.720, 0.180, 0.220;   % wine red, main
    % 0.450, 0.650, 0.850;   % blue, light
    % 0.120, 0.330, 0.600;   % blue, main
    % 0.500, 0.760, 0.640;   % group 3, light green
    % 0.000, 0.500, 0.360;   % group 3, main green
];
    % line_styles={':','-',':','-',':','-',':','-'};
    % line_styles = {'-', '--', ':', '-.'};
    line_styles = {'-','-','-','-','-','-'};
    dot = {'none','none','none','none','none','none','none'};
    %% 2. 读取真值数据
    try
        temp = importdata(truthpath);
        truth_data = temp(:, 2:end); % 假设第一列是序号，剔除
    catch
        error('无法读取真值文件: %s', truthpath);
    end
    % 预处理真值航向角 (解卷绕)
    truth_data = smooth_heading(truth_data);
    %% 3. 处理每个测试文件
    all_results = struct('time', {}, 'err', {}, 'radial', {}, 'label', {});
    % 用于记录所有文件的公共时间范围
    common_start = -inf;
    common_end = inf;
    valid_indices = []; % 记录成功处理的文件索引
    for i = 1:num_tests
        try
            % 读取测试数据
            temp = importdata(test_files{i});
            test_data = temp(:, 2:end);
            % 预处理测试航向角
            test_data = smooth_heading(test_data);
            % 获取标签
            [~, name, ~] = fileparts(test_files{i});
            label = get_system_label(name);
            % --- 核心计算 (已更新为 GJB 动态逐点计算) ---
            [time, err] = align_and_calculate_error(test_data, truth_data);
            % 计算初始径向误差 (水平)
            radial_error = sqrt(sum(err(:, 2:3).^2, 2));
            % --- 【新增】可选的 Grubbs 异常点剔除与插值缝合 ---
            if apply_grubbs
                [~, keep_idx, removed_cnt] = grubbs_filter(radial_error);
                if removed_cnt > 0
                    bad_idx = ~keep_idx;
                    good_idx = keep_idx;
                    % 缝合断点
                    err(bad_idx, 2) = interp1(time(good_idx), err(good_idx, 2), time(bad_idx), 'linear', 'extrap');
                    err(bad_idx, 3) = interp1(time(good_idx), err(good_idx, 3), time(bad_idx), 'linear', 'extrap');
                    % 重新计算缝合后的径向误差
                    radial_error = sqrt(sum(err(:, 2:3).^2, 2));
                    fprintf('[%s] 剔除并缝合异常点: %d 个\n', label, removed_cnt);
                end
            end
            % 存储结果
            all_results(i).time = time;
            all_results(i).err = err;
            all_results(i).radial = radial_error;
            all_results(i).label = label;
            if isempty(time)
                warning('文件 %s 与真值无时间重叠', test_files{i});
                continue;
            end
            common_start = max(common_start, time(1));
            common_end = min(common_end, time(end));
            valid_indices = [valid_indices, i];
        catch ME
            warning('处理文件 %s 时出错: %s', test_files{i}, ME.message);
        end
    end
    if isempty(valid_indices)
        error('没有有效的数据可供绘图数据导出');
    end
    %% 4. 统一截取公共时间段
    for i = valid_indices
        mask = (all_results(i).time >= common_start) & (all_results(i).time <= common_end);
        all_results(i).time   = all_results(i).time(mask);
        all_results(i).err    = all_results(i).err(mask, :);
        all_results(i).radial = all_results(i).radial(mask);
        if ~isempty(all_results(i).time)
            all_results(i).time = all_results(i).time - all_results(i).time(1);
        end
    end
    %% 5. 绘制径向误差对比图
    if exist('myfigurestartup', 'file')
        fig = myfigurestartup(6, 4, 'prese');
    else
        fig = figure('Color', 'w'); 
    end
    hold on; grid on; box on;
    for i = valid_indices
        if ~isempty(all_results(i).radial)
            style_idx = mod(i-1, length(line_styles)) + 1;
            plot(all_results(i).time, all_results(i).radial, ...
                 'Color', colors(i,:), ...
                 'LineStyle', line_styles{style_idx}, ...
                 'Marker',dot{style_idx},...
                 'DisplayName', all_results(i).label);
        end
    end
    xlabel('时间 (s)');
    ylabel('水平径向误差 (m)');
    legend('show', 'Location', 'best');
    if ~isempty(valid_indices)
         xlim([0, max(cellfun(@(x) max(x), {all_results(valid_indices).time}))]);
    end
    %% 6. 单次试验统计：RMS、Max、终点RER、中位数
    % RER只取本次试验评价终点的一个值：
    %   RER = [终点水平径向误差(n mile)] / [累计运行时间(h)]。
    % 不再把同一次试验的各个时间历元当成独立RER样本，也不在这里计算R50。
    excelHeader = {'系统模型组件', 'RMS (m)', 'Max (m)', ...
        'RER (n mile/h)', 'Median (m)'};
    num_valid = length(valid_indices);
    excelBody = cell(num_valid, 5);
    row_cnt = 1;

    fprintf('\n============================================================\n');
    fprintf('单次导航试验水平位置误差统计\n');
    fprintf('RER = 评价终点径向误差(n mile) / 累计运行时间(h)\n');
    fprintf('本函数不计算R50；R50请使用多次独立试验的RER另行计算。\n');
    fprintf('============================================================\n');

    for i = valid_indices
        re = all_results(i).radial;
        time = all_results(i).time;
        if ~isempty(re)
            evaluation_end_time = min(3600, time(end));
            evaluation_mask = time >= 0 & time <= evaluation_end_time;
            evaluation_time = time(evaluation_mask);
            evaluation_radial = re(evaluation_mask);

            rms_val = sqrt(mean(evaluation_radial.^2));
            max_val = max(evaluation_radial);
            median_val = median(evaluation_radial);

            % 评价终点使用最后一个不晚于评价时长的有效历元。
            endpoint_time_s = evaluation_time(end);
            endpoint_radial_m = evaluation_radial(end);
            if endpoint_time_s > 0
                rer_nmi_per_h = (endpoint_radial_m / 1852) / ...
                    (endpoint_time_s / 3600);
            else
                rer_nmi_per_h = NaN;
            end

            excelBody{row_cnt, 1} = all_results(i).label;
            excelBody{row_cnt, 2} = rms_val;
            excelBody{row_cnt, 3} = max_val;
            excelBody{row_cnt, 4} = rer_nmi_per_h;
            excelBody{row_cnt, 5} = median_val;

            fprintf('\n[%s]\n', all_results(i).label);
            fprintf('评价时长 : %.3f s\n', endpoint_time_s);
            fprintf('RMS      : %.6f m\n', rms_val);
            fprintf('Max      : %.6f m\n', max_val);
            fprintf('RER      : %.9f n mile/h\n', rer_nmi_per_h);
            fprintf('Median   : %.6f m\n', median_val);
            row_cnt = row_cnt + 1;
        end
    end
    finalExcelData = [excelHeader; excelBody];
end
%% ================== 辅助函数区域 ==================
%% 辅助函数：时间对齐和误差计算 (已更新为 GJB 4727A 逐点公式)
function [time, err] = align_and_calculate_error(test_data, truth_data)
    [test_data, test_dt] = select_longest_continuous_segment(...
        test_data, 'test navigation');
    [truth_data, ~] = select_longest_continuous_segment(...
        truth_data, 'reference navigation');
    res_start = test_data(1, 1);
    res_end   = test_data(end, 1);
    ref_start = truth_data(1, 1);
    ref_end   = truth_data(end, 1);
    starttime = max(res_start, ref_start);
    endtime   = min(res_end, ref_end);
    if starttime >= endtime
        error('测试数据与真值数据时间无重叠');
    end
    dt = test_dt;
    n_interp = floor((endtime - starttime) / dt) + 1;
    if n_interp > 5e7
        error('calc_radial_error_gjb:ExcessiveInterpolation', ...
            ['公共时间范围将产生%d个插值点，请检查时间戳是否存在' ...
             '异常跳变。'], n_interp);
    end
    time = (starttime:dt:endtime)'; 
    new_test  = interp1(test_data(:, 1), test_data(:, 2:10), time, 'linear', 'extrap');
    new_truth = interp1(truth_data(:, 1), truth_data(:, 2:10), time, 'linear', 'extrap');
    error_raw = new_test - new_truth;
    error_raw(:, 8) = mod(error_raw(:, 8) + 180, 360) - 180; 
    error_raw(:, 9) = mod(error_raw(:, 9) + 180, 360) - 180; 
    param = Param();
    err = zeros(size(error_raw, 1), 10);
    err(:, 1) = time - time(1); 
    % --- 严格按 GJB 4727A 公式(1)与(2)动态逐点转换 ---
    n_points = length(time);
    for i = 1:n_points
        lat_rad = deg2rad(new_truth(i, 1));
        h       = new_truth(i, 3);
        [rm, rn] = getRmRn(lat_rad, param);
        err(i, 2) = deg2rad(error_raw(i, 1)) *(rm + h);                  % 北向误差 (m)
        err(i, 3) = deg2rad(error_raw(i, 2)) *(rn + h) *cos(lat_rad);   % 东向误差 (m)
    end
    err(:, 4)    = -error_raw(:, 3); % 垂向高度误差
    err(:, 5:10) = error_raw(:, 4:9);
end

function [data, nominal_dt_s] = select_longest_continuous_segment( ...
        data, axis_name)
% Select the longest valid block before interpolation. Never fill a large
% timestamp hole with a fictitious linear navigation trajectory.

    time_s = double(data(:, 1));
    delta_s = diff(time_s);
    positive_delta = delta_s(delta_s > 0 & isfinite(delta_s));
    if isempty(positive_delta)
        error('calc_radial_error_gjb:InvalidTimeAxis', ...
            '%s has no valid positive interval.', axis_name);
    end
    nominal_dt_s = median(positive_delta);
    % 允许短时间丢帧仍属于同一次试验。原先固定0.05 s会把static-3
    % 中约0.38 s的短缺测误判成新试验，进而错误地截断评价时长。
    % 超过0.5 s（或5个正常采样周期）的跳变才作为真正断段处理。
    max_gap_s = max(0.500, 5 * nominal_dt_s);
    break_after = find(~isfinite(delta_s) | delta_s <= 0 | ...
        delta_s > max_gap_s);
    segment_start = [1; break_after + 1];
    segment_end = [break_after; size(data, 1)];
    [~, longest_index] = max(segment_end - segment_start + 1);
    keep = segment_start(longest_index):segment_end(longest_index);
    discarded = size(data, 1) - numel(keep);
    if discarded > 0
        warning('calc_radial_error_gjb:DiscardedDiscontinuousData', ...
            '%s discarded %d row(s) outside its longest continuous segment.', ...
            axis_name, discarded);
        data = data(keep, :);
    end
end
%% 辅助函数：航向角平滑 (解卷绕)
function data = smooth_heading(data)
    heading_rad = deg2rad(data(:, 9));
    heading_unwrapped = unwrap(heading_rad);
    data(:, 9) = rad2deg(heading_unwrapped);
end
%% 辅助函数：获取系统标签
function label = get_system_label(filename)
    name_map = containers.Map({'Origin_Drop_1', 'Origin_Drop_3', 'Origin_Drop_5', 'Origin_Drop_7', 'Origin_Drop_9', 'Origin_Drop_11', 'ukf', 'truth', 'mems'}, ...
        {'Origin-Drop-1', 'Origin-Drop-3', 'Origin-Drop-5', 'Origin-Drop-7', 'Origin-Drop-9', 'Origin-Drop-11', 'UKF', '真值', 'MEMS-EKF'});
    lower_name = lower(filename);
    keys = name_map.keys;
    label = filename; % 默认使用原文件名
    for i = 1:length(keys)
        if contains(lower_name, keys{i})
            label = name_map(keys{i}); 
            break;
        end
    end
end
%% 辅助函数：格拉布斯异常值剔除算法 (针对海量导航数据进行了提速优化)
function [data_clean, keep_idx, removed_count] = grubbs_filter(data_raw)
    n_total = length(data_raw);
    keep_idx = true(n_total, 1);
    removed_count = 0;
    % GJB 4727A 小样本查表 (N <= 100)
    n_table = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 30, 35, 40, 45, 50, 60, 70, 80, 90, 100];
    t_table = [1.15, 1.46, 1.67, 1.89, 2.02, 2.13, 2.21, 2.29, 2.36, 2.41, 2.46, 2.51, 2.55, 2.59, 2.62, 2.65, 2.68, 2.71, 2.73, 2.76, 2.78, 2.80, 2.82, 2.91, 2.98, 3.04, 3.09, 3.13, 3.20, 3.26, 3.31, 3.35, 3.38];
    while true
        active_data = data_raw(keep_idx);
        n_curr = length(active_data);
        if n_curr < 3, break; end
        mu_curr = mean(active_data);
        S_curr = std(active_data);
        if S_curr == 0, break; end
        if n_curr <= 100
            % 【模式A：小样本严格单点剔除】
            [~, nearest_idx] = min(abs(n_table - n_curr));
            T_limit = t_table(nearest_idx);
            % O(N) 极值查找代替 O(N log N) 的全局排序，极大提速
            [max_val, max_idx] = max(active_data);
            [min_val, min_idx] = min(active_data);
            T_max = (max_val - mu_curr) / S_curr;
            T_min = (mu_curr - min_val) / S_curr;
            if T_max > T_min
                T_test = T_max; suspect_idx = max_idx;
            else
                T_test = T_min; suspect_idx = min_idx;
            end
            if T_test > T_limit
                active_indices = find(keep_idx);
                keep_idx(active_indices(suspect_idx)) = false;
                removed_count = removed_count + 1;
            else
                break; % 无异常点，退出循环
            end
        else
            % 【模式B：海量数据大样本批量剔除】
            % 对于几万个点，采用大样本渐近临界值公式，防止把正态分布尾巴误当做粗差
            T_limit = sqrt(2 *log(n_curr)); 
            % 计算所有点的偏离度
            deviations = abs(active_data - mu_curr) / S_curr;
            bad_local_idx = find(deviations > T_limit);
            if ~isempty(bad_local_idx)
                active_indices = find(keep_idx);
                % 批量剔除，一次循环解决所有越界点，消灭死循环
                keep_idx(active_indices(bad_local_idx)) = false; 
                removed_count = removed_count + length(bad_local_idx);
            else
                break; % 清洗完毕
            end
        end
    end
    data_clean = data_raw(keep_idx);
end
