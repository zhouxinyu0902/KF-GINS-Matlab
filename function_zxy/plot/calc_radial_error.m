function calc_radial_error(truthpath, varargin)
% CALC_RADIAL_ERROR 计算并绘制多个导航结果的径向误差对比
%
% 输入:
%   truthpath - 真值文件路径
%   varargin  - 一个或多个测试结果文件路径
%
% 示例:
%   calc_radial_error('truth.nav', 'ins_result.nav', 'gps_result.nav', 'filtered_result.nav')

    %% 参数检查和初始化
    if nargin < 2
        error('至少需要提供真值文件和一个测试结果文件');
    end
    
    test_files = varargin;
    num_tests = length(test_files);
    
    % 颜色和线型定义
    colors = lines(num_tests);
%  colors = [
%     [0, 0.4470, 0.7410];  % 1. 蓝色 (标准)
%     [0.8500, 0.3250, 0.0980]; % 2. 橙色 (高对比度)
%     [0.9290, 0.6940, 0.1250]; % 3. 黄色 (标准)
%     [1.0, 0.0, 0.0]           % 4. 纯红色 (突出显示/焦点)
% ];
    line_styles = {'-', '--', ':', '-.'};
    markers = {'o', 's', 'd', '^', 'v'};
    width=[0.2,0.5,1.5,2];
    %% 读取真值数据
    try
        temp = importdata(truthpath);
        truth_data = temp(:, 2:end);
        % fprintf('成功读取真值文件: %s\n', truthpath);
    catch
        error('无法读取真值文件: %s', truthpath);
    end
    
    %% 处理航向角平滑（真值）
    truth_data = smooth_heading(truth_data);
    
    %% 处理每个测试文件
    all_errors = cell(1, num_tests);
    all_times = cell(1, num_tests);
    radial_errors = cell(1, num_tests);
    file_labels = cell(1, num_tests);
    
    for i = 1:num_tests
        try
            % 读取测试数据
            temp = importdata(test_files{i});
            test_data = temp(:, 2:end);
            % fprintf('成功读取测试文件: %s\n', test_files{i});
            
            % 航向角平滑
            test_data = smooth_heading(test_data);
            
            % 获取文件标签（用于图例）
            [~, name, ~] = fileparts(test_files{i});
            file_labels{i} = get_system_label(name);
            
            % 时间对齐和误差计算
            [time, err] = align_and_calculate_error(test_data, truth_data);
            

            % 计算径向误差
            radial_error = sqrt(sum(err(:, 2:3).^2, 2)); % 只考虑水平方向
            
            % 存储结果
            all_times{i} = time;
            all_errors{i} = err;
            radial_errors{i} = radial_error;
            starttime(i) = time(1);
            endtime(i) = time(end);
            
        catch ME
            warning('处理文件 %s 时出错: %s', test_files{i}, ME.message);
            continue;
        end
    end
    %%
    selectstart=max(starttime);
    selectend=min(endtime);

    for i=1:num_tests
        index0=find(abs(all_times{i}-selectstart)<0.01);
        indexend=find(abs(all_times{i}-selectend)<0.01);
        all_errors{i} = all_errors{i} (index0(1):indexend(1));
        radial_errors{i} = radial_errors{i} (index0(1):indexend(1));
        all_times{i} = all_times{i} (index0(1):indexend(1));
        all_times{i} = all_times{i} -all_times{i} (1);
    end
    %% 绘制径向误差对比图
    myfigurestartup(12,5,'prese')
    
    % 主图：径向误差时间序列
    % subplot(2, 1, 1);
    hold on;
    grid on;
    
    for i = 1:num_tests
        if ~isempty(radial_errors{i})
            style_idx = mod(i-1, length(line_styles)) + 1;
            marker_idx = mod(i-1, length(markers)) + 1;
            
            plot(all_times{i}, radial_errors{i}, ...
                 'Color', colors(i,:), ...
                 'LineStyle', line_styles{style_idx}, ...
                 'LineWidth', width(style_idx), ...
                 'Marker', markers{marker_idx}, ...
                 'MarkerIndices', 1:100:length(all_times{i}), ...
                 'MarkerSize', 2, ...
                 'DisplayName', file_labels{i});

            % plot(all_times{i}, radial_errors{i}, ...
            %      'Color', colors(i,:), ...
            %      'LineStyle', line_styles{style_idx}, ...
            %      'LineWidth', 1.5, ...
            %      'Marker', markers{marker_idx}, ...
            %      'MarkerIndices', 1:100:length(all_times{i}), ...
            %      'MarkerSize', 2, ...
            %      'DisplayName', file_labels{i});
            
        end
    end
    
    % title('径向误差时间序列对比');
    xlabel('时间 (s)');
    ylabel('径向误差 (m)');
    legend('show');
    xlim([min(cellfun(@min, all_times)), max(cellfun(@max, all_times))]);
    
    %% 绘制误差统计分布
    % subplot(2, 1, 2);
    % hold on;
    
    % % 准备箱线图数据
    % box_data = [];
    % group_labels = {};
    % 
    % for i = 1:num_tests
    %     if ~isempty(radial_errors{i})
    %         box_data = [box_data; radial_errors{i}];
    %         group_labels = [group_labels; repmat({file_labels{i}}, length(radial_errors{i}), 1)];
    %     end
    % end
    % 
    % boxplot(box_data, group_labels);
    % title('径向误差分布统计');
    % ylabel('径向误差 (m)');
    % grid on;
    
    %% 输出统计结果
    fprintf('\n===== 径向误差统计结果 =====\n');
    fprintf('%-15s %-8s %-8s %-8s %-8s %-8s\n', '系统','max', 'RMS', '均值', '中位数', '95%分位');
    fprintf('-------------------------------------------------\n');
    
    for i = 1:num_tests
        if ~isempty(radial_errors{i})
            re = radial_errors{i};
            rms_error = sqrt(mean(re.^2));
            mean_error = mean(re);
            median_error = median(re);
            quantile_95 = quantile(re, 0.95);
            max_error = max(re);
            fprintf('%-15s %-8.2f %-8.2f %-8.2f %-8.2f %-8.2f\n', ...
                    file_labels{i}, max_error, rms_error, mean_error, median_error, quantile_95);
        end
    end
    
    %% 输出CEP误差
    % fprintf('\n===== 圆概率误差(CEP) =====\n');
    % for i = 1:num_tests
    %     if ~isempty(all_errors{i})
    %         east_error = all_errors{i}(:, 3);  % 东向误差
    %         north_error = all_errors{i}(:, 2); % 北向误差
    % 
    %         sigma_x = std(east_error);
    %         sigma_y = std(north_error);
    % 
    %         if (sigma_x == sigma_y) && (abs(corr(east_error, north_error)) < 0.1)
    %             CEP = 1.1774 * sigma_x;
    %         else
    %             CEP = 1.1774 * sqrt((sigma_x^2 + sigma_y^2)/2);
    %         end
    % 
    %         fprintf('%s: CEP = %.2f m (%.4f 海里)\n', ...
    %                 file_labels{i}, CEP, CEP/1852);
    %     end
    % end
end

%% 辅助函数：航向角平滑
function data = smooth_heading(data)
    for i = 2:size(data, 1)
        if (data(i, 9) - data(i-1, 9)) < -180
            data(i:end, 9) = data(i:end, 9) + 360;
        end
        if (data(i, 9) - data(i-1, 9)) > 180
            data(i:end, 9) = data(i:end, 9) - 360;
        end
    end
end

%% 辅助函数：时间对齐和误差计算
function [time, err] = align_and_calculate_error(test_data, truth_data)
    % 找到数据重叠部分
    res_start = test_data(1, 1);
    res_end = test_data(end, 1);
    ref_start = truth_data(1, 1);
    ref_end = truth_data(end, 1);
    
    starttime = max(res_start, ref_start);
    endtime = min(res_end, ref_end);
    
    if starttime >= endtime
        error('测试数据与真值数据时间无重叠');
    end
    
    % 内插时间序列
    dt = mean(diff(test_data(:, 1)));
    time = starttime:dt:endtime;
    time = time';
    
    % 内插数据
    new_test = interp1(test_data(:, 1), test_data(:, 2:10), time);
    new_truth = interp1(truth_data(:, 1), truth_data(:, 2:10), time);
    
    % 计算误差
    error_ned = new_test - new_truth;
    
    % 处理航向角误差
    for i = 1:size(error_ned, 1)
        if error_ned(i, 9) > 180
            error_ned(i, 9) = error_ned(i, 9) - 360;
        end
        if error_ned(i, 9) < -180
            error_ned(i, 9) = error_ned(i, 9) + 360;
        end
    end
    
    % 位置误差转换到ENU坐标系
    param = Param();
    first_blh = test_data(1, 2:4);
    [rm, rn] = getRmRn(first_blh(1) * param.D2R, param);
    h = first_blh(3);
    DR = diag([rm + h, (rn + h)*cos(first_blh(1) * param.D2R), -1]);
    
    error_ned(:, 1:2) = error_ned(:, 1:2) * param.D2R;
    err = zeros(size(new_test));
    err(:, 1) = time - time(1); % 相对时间
    
    for i = 1:size(error_ned, 1)
        delta_pos = DR * (error_ned(i, 1:3)');
        err(i, 2:4) = delta_pos';
    end
    err(:, 5:10) = error_ned(:, 4:9);
end

%% 辅助函数：获取系统标签
function label = get_system_label(filename)
    % 将文件名映射为有意义的标签
    name_map = containers.Map(...
        {'pure_ins', 'gps', 'ins', 'gnss', 'filter', 'ekf', 'ukf', 'truth'}, ...
        {'纯惯性', 'GPS', 'INS', 'GNSS', '滤波', 'EKF', 'UKF', '真值'}...
    );
    
    if isKey(name_map, lower(filename))
        label = name_map(lower(filename));
    else
        label = filename;
    end
end