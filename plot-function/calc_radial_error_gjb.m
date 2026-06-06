function [fig,finalExcelData] = calc_radial_error_gjb(truthpath, varargin)
% CALC_RADIAL_ERROR 计算并绘制多个导航结果的径向误差对比 (带 Excel 自动导出功能)
%
% 输入:
%   truthpath - 真值文件路径
%   varargin  - 一个或多个测试结果文件路径。
%               【新特性】：最后一个参数若为 logical 类型 (true/false)，则作为 apply_grubbs 的开关。
%
% 调用示例:
%   calc_radial_error('truth.txt', 'test1.txt', 'test2.txt')        % 默认不剔除
%   calc_radial_error('truth.txt', 'test1.txt', 'test2.txt', true)  % 开启 Grubbs 剔除

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
    line_styles = {'-', '-', '-', '-'};

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
                 'DisplayName', all_results(i).label);
        end
    end
    xlabel('时间 (s)');
    ylabel('水平径向误差 (m)');
    legend('show', 'Location', 'best');
    if ~isempty(valid_indices)
         xlim([0, max(cellfun(@(x) max(x), {all_results(valid_indices).time}))]);
    end

    %% 6. 核心：提取统计结果并自动导出 Excel
    excelHeader = {'系统模型组件', '最大径向误差 (Max/m)', '均方根误差 (RMS/m)', '均值 (Mean/m)', '中位数 (Median/m)', '95%分位数 (95%/m)'};
    num_valid = length(valid_indices);
    excelBody = cell(num_valid, 6);
    
    row_cnt = 1;
    for i = valid_indices
        re = all_results(i).radial;
        if ~isempty(re)
            rms_val = sqrt(mean(re.^2));
            max_val = max(re);
            mean_val = mean(re);
            median_val = median(re);
            quantile_95 = quantile(re, 0.95);
            
            excelBody{row_cnt, 1} = all_results(i).label;
            excelBody{row_cnt, 2} = max_val;
            excelBody{row_cnt, 3} = rms_val;
            excelBody{row_cnt, 4} = mean_val;
            excelBody{row_cnt, 5} = median_val;
            excelBody{row_cnt, 6} = quantile_95;
            
            row_cnt = row_cnt + 1;
        end
    end
    
    finalExcelData = [excelHeader; excelBody];
end

%% ================== 辅助函数区域 ==================

%% 辅助函数：时间对齐和误差计算 (已更新为 GJB 4727A 逐点公式)
function [time, err] = align_and_calculate_error(test_data, truth_data)
    res_start = test_data(1, 1);
    res_end   = test_data(end, 1);
    ref_start = truth_data(1, 1);
    ref_end   = truth_data(end, 1);
    
    starttime = max(res_start, ref_start);
    endtime   = min(res_end, ref_end);
    
    if starttime >= endtime
        error('测试数据与真值数据时间无重叠');
    end
    
    dt = mean(diff(test_data(:, 1)));
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
        
        err(i, 2) = deg2rad(error_raw(i, 1)) * (rm + h);                  % 北向误差 (m)
        err(i, 3) = deg2rad(error_raw(i, 2)) * (rn + h) * cos(lat_rad);   % 东向误差 (m)
    end
    
    err(:, 4)    = -error_raw(:, 3); % 垂向高度误差
    err(:, 5:10) = error_raw(:, 4:9);
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
            T_limit = sqrt(2 * log(n_curr)); 
            
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