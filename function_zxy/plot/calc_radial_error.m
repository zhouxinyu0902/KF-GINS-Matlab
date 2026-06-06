function [fig,finalExcelData] = calc_radial_error(truthpath, varargin)
% CALC_RADIAL_ERROR 计算并绘制多个导航结果的径向误差对比 (带 Excel 自动导出功能)
%
% 输入:
%   truthpath - 真值文件路径
%   varargin  - 一个或多个测试结果文件路径

    %% 1. 参数检查和初始化
    if nargin < 2
        error('至少需要提供真值文件和一个测试结果文件');
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

            % --- 核心计算 ---
            [time, err] = align_and_calculate_error(test_data, truth_data);

            % 计算径向误差 (水平)
            radial_error = sqrt(sum(err(:, 2:3).^2, 2));

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
        % fig = myfigurestartup(6, 3, 'paper');
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
    % 准备 Excel 表头
    excelHeader = {'系统模型组件', '最大径向误差 (Max/m)', '均方根误差 (RMS/m)', '均值 (Mean/m)', '中位数 (Median/m)', '95%分位数 (95%/m)'};
    
    % 动态创建用于装载 Excel 数据的 cell 数组
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
            
            % 将当前系统的统计结果压入对应的 cell 行中
            excelBody{row_cnt, 1} = all_results(i).label;
            excelBody{row_cnt, 2} = max_val;
            excelBody{row_cnt, 3} = rms_val;
            excelBody{row_cnt, 4} = mean_val;
            excelBody{row_cnt, 5} = median_val;
            excelBody{row_cnt, 6} = quantile_95;
            
            row_cnt = row_cnt + 1;
        end
    end
    
    % 垂直堆叠表头与实体内容
    finalExcelData = [excelHeader; excelBody];
    

end

%% ================== 辅助函数区域 ==================

%% 辅助函数：时间对齐和误差计算 (深度优化版)
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
    ref_lat_deg = new_truth(1, 1); 
    ref_h       = new_truth(1, 3); 

    [rm, rn] = getRmRn(ref_lat_deg * param.D2R, param);
    scale_vec = [rm + ref_h, (rn + ref_h) * cos(ref_lat_deg * param.D2R), -1];

    err = zeros(size(error_raw, 1), 10);
    err(:, 1) = time - time(1); 

    d_latlon_rad = error_raw(:, 1:2) * param.D2R;
    d_alt        = error_raw(:, 3);

    err(:, 2) = d_latlon_rad(:, 1) * scale_vec(1); 
    err(:, 3) = d_latlon_rad(:, 2) * scale_vec(2); 
    err(:, 4) = d_alt * scale_vec(3);              

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
            % ✨【Bug 修复核心】：将外层的 {} 改为 () 
            label = name_map(keys{i}); 
            break;
        end
    end
end