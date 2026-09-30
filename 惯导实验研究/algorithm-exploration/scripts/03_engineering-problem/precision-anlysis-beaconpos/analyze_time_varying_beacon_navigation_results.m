function master_summary = analyze_time_varying_beacon_navigation_results(varargin)
%ANALYZE_TIME_VARYING_BEACON_NAVIGATION_RESULTS 统一评价时变潜标导航结果。
%   本函数不读取已有统计表或图片，只读取各数据集的 truth 与 NAV 文件，
%   调用 calc_radial_error_gjb 重新计算未补偿、位置不确定度补偿和真实潜标
%   位置三类结果。输出保存到：
%
%     <dataset>/output/figures-tables/<study_id>/
%
%   默认扫描：
%     simulation/case-00、case-05、case-06
%     experiment/case-06、case-07、case-08、case-09
%
%   示例：
%     analyze_time_varying_beacon_navigation_results
%     analyze_time_varying_beacon_navigation_results( ...
%         'DatasetFilter', "simulation/case-05", ...
%         'AlgorithmFilter', "rts-double")
%
%   可选参数：
%     DatasetFilter   - "simulation/case-05"形式的字符串数组；空表示全部
%     StudyFilter     - study_id字符串数组；空表示全部beacon-position-*结果
%     AlgorithmFilter - forward-ekf、rts-single、rts-double；空表示全部
%     ApplyGrubbs     - 是否启用calc_radial_error_gjb的Grubbs处理，默认false
%     MaximumPlotPoints - 每条曲线导出时保留的最大点数，默认20000
%     CloseFigures    - 导出后是否关闭图窗，默认true

parser = inputParser;
parser.addParameter('DatasetFilter', strings(0,1), ...
    @(x) isstring(x) || ischar(x) || iscellstr(x));
parser.addParameter('StudyFilter', strings(0,1), ...
    @(x) isstring(x) || ischar(x) || iscellstr(x));
parser.addParameter('AlgorithmFilter', strings(0,1), ...
    @(x) isstring(x) || ischar(x) || iscellstr(x));
parser.addParameter('ApplyGrubbs', false, ...
    @(x) islogical(x) && isscalar(x));
parser.addParameter('MaximumPlotPoints', 20000, ...
    @(x) isnumeric(x) && isscalar(x) && isfinite(x) && x >= 100);
parser.addParameter('CloseFigures', true, ...
    @(x) islogical(x) && isscalar(x));
parser.parse(varargin{:});
options = parser.Results;
options.DatasetFilter = normalize_string_list(options.DatasetFilter);
options.StudyFilter = normalize_string_list(options.StudyFilter);
options.AlgorithmFilter = normalize_string_list(options.AlgorithmFilter);

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));
addpath(topic_dir);
paths = setup_inertial_experiment();
addpath(fullfile(paths.project, 'function_zxy', 'plot-function'));
if exist('calc_radial_error_gjb', 'file') ~= 2
    error('找不到calc_radial_error_gjb.m，请检查function_zxy/plot-function。');
end

dataset_source = ["simulation"; "simulation"; "simulation"; ...
    "experiment"; "experiment"; "experiment"; "experiment"];
dataset_id = ["case-00"; "case-05"; "case-06"; ...
    "case-06"; "case-07"; "case-08"; "case-09"];
dataset_key = dataset_source + "/" + dataset_id;
if ~isempty(options.DatasetFilter)
    keep = ismember(dataset_key, options.DatasetFilter);
    dataset_source = dataset_source(keep);
    dataset_id = dataset_id(keep);
    dataset_key = dataset_key(keep);
end
if isempty(dataset_key)
    error('DatasetFilter没有匹配到预设数据集。');
end

algorithms = struct( ...
    'id', {"forward-ekf", "rts-single", "rts-double"}, ...
    'name', {"前向EKF", "单次RTS", "双次RTS"}, ...
    'file', {"simple-forward-ekf-rad.nav", ...
             "simple-rts-single-rad.nav", ...
             "simple-rts-double-rad.nav"});
if ~isempty(options.AlgorithmFilter)
    keep = ismember(string({algorithms.id}), options.AlgorithmFilter);
    algorithms = algorithms(keep);
end
if isempty(algorithms)
    error('AlgorithmFilter没有匹配到forward-ekf、rts-single或rts-double。');
end

mode_ids = ["uncompensated-rad", "compensated-rad", ...
    "truth-beacon-reference-rad"];
mode_names = ["未补偿（固定潜标位置）", "位置不确定度补偿", ...
    "真实潜标位置"];

master_summary = empty_summary_table();
for dataset_index = 1:numel(dataset_key)
    data_source = dataset_source(dataset_index);
    case_id = dataset_id(dataset_index);
    case_root = fullfile(paths.data, data_source, case_id);
    navigation_root = fullfile(case_root, 'output', 'navigation-results');
    if ~isfolder(navigation_root)
        warning('导航结果目录不存在，跳过：%s', navigation_root);
        continue;
    end

    truth_path = find_truth_path(fullfile(case_root, 'input'), data_source);
    if strlength(truth_path) == 0
        warning('缺少truth.txt/truth.nav，跳过：%s', case_root);
        continue;
    end
    [truth_start_s, truth_end_s] = read_text_time_bounds(truth_path);

    studies = dir(fullfile(navigation_root, 'beacon-position-*'));
    studies = studies([studies.isdir]);
    if ~isempty(options.StudyFilter)
        keep = ismember(string({studies.name}), options.StudyFilter);
        studies = studies(keep);
    end
    if isempty(studies)
        fprintf('[%s] 没有可分析的潜标结果目录。\n', dataset_key(dataset_index));
        continue;
    end

    for study_index = 1:numel(studies)
        study_id = string(studies(study_index).name);
        result_root = fullfile(studies(study_index).folder, ...
            studies(study_index).name);
        artifact_root = fullfile(case_root, 'output', 'figures-tables', ...
            study_id);
        if ~isfolder(artifact_root)
            mkdir(artifact_root);
        end

        fprintf('\n[%s/%s] 重新分析NAV结果。\n', ...
            dataset_key(dataset_index), study_id);
        study_summary = empty_summary_table();
        report_messages = strings(0,1);

        for algorithm_index = 1:numel(algorithms)
            algorithm = algorithms(algorithm_index);
            candidate_paths = strings(numel(mode_ids),1);
            for mode_index = 1:numel(mode_ids)
                candidate_paths(mode_index) = fullfile(result_root, ...
                    mode_ids(mode_index), algorithm.file);
            end
            available = arrayfun(@(p) isfile(p), candidate_paths);
            if any(~available)
                message = sprintf('%s缺少NAV：%s。', algorithm.name, ...
                    strjoin(mode_ids(~available), ', '));
                report_messages(end+1,1) = string(message); %#ok<AGROW>
            end
            if nnz(available) < 2
                message = sprintf('%s可用模式不足2个，跳过。缺少：%s', ...
                    algorithm.name, strjoin(mode_ids(~available), ', '));
                warning('%s', message);
                report_messages(end+1,1) = string(message); %#ok<AGROW>
                continue;
            end

            nav_paths = candidate_paths(available);
            available_mode_ids = mode_ids(available);
            available_mode_names = mode_names(available);
            nav_cells = cellstr(nav_paths);
            try
                input_information = [dir(char(truth_path)); ...
                    arrayfun(@(p) dir(char(p)), nav_paths)];
                use_large_file_backend = any([input_information.bytes] > ...
                    256*1024^2);
                if use_large_file_backend
                    [figure_handle, statistics_cell] = ...
                        calc_radial_error_gjb_large_files(char(truth_path), ...
                        nav_cells, options.ApplyGrubbs, ...
                        options.MaximumPlotPoints);
                    report_messages(end+1,1) = sprintf( ...
                        '%s使用低内存GJB大文件后端。', algorithm.name); %#ok<AGROW>
                else
                    [figure_handle, statistics_cell] = calc_radial_error_gjb( ...
                        char(truth_path), nav_cells{:}, options.ApplyGrubbs);
                end
            catch ME
                message = sprintf('%s分析失败：%s', algorithm.name, ME.message);
                warning('%s', message);
                report_messages(end+1,1) = string(message); %#ok<AGROW>
                continue;
            end

            if size(statistics_cell,1)-1 ~= numel(nav_paths)
                if isgraphics(figure_handle), close(figure_handle); end
                message = sprintf('%s有效结果数量与输入NAV数量不一致，跳过导出。', ...
                    algorithm.name);
                warning('%s', message);
                report_messages(end+1,1) = string(message); %#ok<AGROW>
                continue;
            end

            statistics_cell(2:end,1) = cellstr(available_mode_names(:));
            format_comparison_figure(figure_handle, available_mode_names, ...
                data_source, case_id, algorithm.name, ...
                options.MaximumPlotPoints);

            output_stem = "navigation-error-" + algorithm.id;
            png_path = fullfile(artifact_root, output_stem + ".png");
            fig_path = fullfile(artifact_root, output_stem + ".fig");
            csv_path = fullfile(artifact_root, output_stem + "-statistics.csv");
            xlsx_path = fullfile(artifact_root, output_stem + "-statistics.xlsx");
            exportgraphics(figure_handle, png_path, 'Resolution', 300);
            savefig(figure_handle, fig_path);
            writecell(statistics_cell, csv_path);
            writecell(statistics_cell, xlsx_path, 'Sheet', 'statistics');

            algorithm_summary = build_algorithm_summary( ...
                data_source, case_id, study_id, algorithm, ...
                available_mode_ids, available_mode_names, nav_paths, ...
                statistics_cell, truth_start_s, truth_end_s);
            study_summary = [study_summary; algorithm_summary]; %#ok<AGROW>
            master_summary = [master_summary; algorithm_summary]; %#ok<AGROW>

            if options.CloseFigures && isgraphics(figure_handle)
                close(figure_handle);
            end
        end

        if ~isempty(study_summary)
            writetable(study_summary, fullfile(artifact_root, ...
                'navigation-error-summary.csv'));
            writetable(study_summary, fullfile(artifact_root, ...
                'navigation-error-summary.xlsx'), 'Sheet', 'summary');
        end
        write_analysis_report(fullfile(artifact_root, ...
            'navigation-error-analysis-report.txt'), data_source, case_id, ...
            study_id, truth_path, result_root, artifact_root, ...
            study_summary, report_messages, options);
    end
end

fprintf('\n统一潜标位置误差分析完成，共生成%d行统计结果。\n', ...
    height(master_summary));
end

function [figure_handle, statistics_cell] = calc_radial_error_gjb_large_files( ...
        truth_path, nav_paths, apply_grubbs, maximum_plot_points)
% 与calc_radial_error_gjb保持相同的水平GJB误差和统计口径，但只读取
% 时间、纬度、经度和高度，避免24 h NAV同时驻留内存。
[fixed_record,truth_first_s,truth_dt_s,truth_record_bytes] = ...
    fixed_truth_record_information(truth_path);
if fixed_record
    [figure_handle,statistics_cell] = ...
        calc_fixed_record_navigation_files(truth_path,nav_paths, ...
        apply_grubbs,maximum_plot_points,truth_first_s,truth_dt_s, ...
        truth_record_bytes);
    return;
end
truth = read_time_position_columns(truth_path);
truth_time = truth(:,1);
truth_position = truth(:,2:4);
clear truth;

nav_count = numel(nav_paths);
nav_start_s = nan(nav_count,1);
nav_end_s = nan(nav_count,1);
for index = 1:nav_count
    [nav_start_s(index),nav_end_s(index)] = ...
        read_text_time_bounds(nav_paths{index});
end
common_start_s = max([truth_time(1);nav_start_s]);
common_end_s = min([truth_time(end);nav_end_s]);
if common_start_s >= common_end_s
    error('NAV与真值没有公共时间段。');
end

statistics_header = {'系统模型组件', ...
    '最大径向误差 (Max/m)', '最大径向误差 (Max/nmi)', ...
    '均方根误差 (RMS/m)', '均值 (Mean/m)', ...
    '中位数 (Median/m)', '95%分位数 (95%/m)', ...
    'CEP50 (m)', 'CEP50 (nmi)', '位置误差率 (nmi/h)', '误差率备注'};
statistics_body = cell(nav_count,11);
plot_time = cell(nav_count,1);
plot_radial = cell(nav_count,1);

for index = 1:nav_count
    navigation = read_time_position_columns(nav_paths{index});
    mask = navigation(:,1) >= common_start_s & ...
        navigation(:,1) <= common_end_s;
    navigation = navigation(mask,:);
    time_s = navigation(:,1);
    navigation_position = navigation(:,2:4);
    clear navigation mask;

    truth_at_navigation = align_truth_position( ...
        truth_time,truth_position,time_s);
    [north_error_m,east_error_m] = calculate_gjb_horizontal_error( ...
        navigation_position,truth_at_navigation);
    relative_time_s = time_s-common_start_s;
    radial_error_m = hypot(north_error_m,east_error_m);
    if apply_grubbs
        keep = grubbs_keep_mask(radial_error_m);
        if any(~keep) && nnz(keep) >= 2
            north_error_m(~keep) = interp1(time_s(keep), ...
                north_error_m(keep),time_s(~keep),'linear','extrap');
            east_error_m(~keep) = interp1(time_s(keep), ...
                east_error_m(keep),time_s(~keep),'linear','extrap');
            radial_error_m = hypot(north_error_m,east_error_m);
        end
    end

    maximum_m = max(radial_error_m);
    rmse_m = sqrt(mean(radial_error_m.^2));
    mean_m = mean(radial_error_m);
    median_m = median(radial_error_m);
    p95_m = quantile(radial_error_m,0.95);
    cep50_m = quantile(radial_error_m,0.50);
    rate_end_s = min(3600,relative_time_s(end));
    rate_mask = relative_time_s > 0 & relative_time_s <= rate_end_s;
    if any(rate_mask)
        rer = (radial_error_m(rate_mask)/1852) ./ ...
            (relative_time_s(rate_mask)/3600);
        position_error_rate = 0.8326*sqrt(mean(rer.^2));
    else
        position_error_rate = nan;
    end
    if relative_time_s(end) >= 3600
        rate_note = '前3600s';
    else
        rate_note = sprintf('有效时长不足3600s，仅%.1fs', ...
            relative_time_s(end));
    end
    [~,label] = fileparts(nav_paths{index});
    statistics_body(index,:) = {label,maximum_m,maximum_m/1852, ...
        rmse_m,mean_m,median_m,p95_m,cep50_m,cep50_m/1852, ...
        position_error_rate,rate_note};

    display_index = unique(round(linspace(1,numel(relative_time_s), ...
        min(maximum_plot_points,numel(relative_time_s)))));
    plot_time{index} = relative_time_s(display_index);
    plot_radial{index} = radial_error_m(display_index);
    clear time_s navigation_position truth_at_navigation north_error_m ...
        east_error_m relative_time_s radial_error_m rate_mask rer;
end

if exist('myfigurestartup','file') == 2
    figure_handle = myfigurestartup(6,4,'prese');
else
    figure_handle = figure('Color','w');
end
axes_handle = axes(figure_handle);
hold(axes_handle,'on'); grid(axes_handle,'on'); box(axes_handle,'on');
colors = [0.00 0.00 0.00;0.85 0.15 0.15;0.10 0.35 0.70; ...
    0.10 0.60 0.20;0.60 0.20 0.70;0.95 0.50 0.10];
for index = 1:nav_count
    plot(axes_handle,plot_time{index},plot_radial{index}, ...
        'Color',colors(mod(index-1,size(colors,1))+1,:), ...
        'LineWidth',1.0,'DisplayName',statistics_body{index,1});
end
xlabel(axes_handle,'时间 (s)');
ylabel(axes_handle,'水平径向误差 (m)');
legend(axes_handle,'show','Location','best');
statistics_cell = [statistics_header;statistics_body];
end

function [figure_handle,statistics_cell] = ...
        calc_fixed_record_navigation_files(truth_path,nav_paths, ...
        apply_grubbs,maximum_plot_points,truth_first_s,truth_dt_s, ...
        truth_record_bytes)
nav_count = numel(nav_paths);
[truth_start_s,truth_end_s] = read_text_time_bounds(truth_path);
nav_start_s = nan(nav_count,1);
nav_end_s = nan(nav_count,1);
for index = 1:nav_count
    [nav_start_s(index),nav_end_s(index)] = ...
        read_text_time_bounds(nav_paths{index});
end
common_start_s = max([truth_start_s;nav_start_s]);
common_end_s = min([truth_end_s;nav_end_s]);
if common_start_s >= common_end_s
    error('NAV与真值没有公共时间段。');
end

statistics_header = {'系统模型组件', ...
    '最大径向误差 (Max/m)', '最大径向误差 (Max/nmi)', ...
    '均方根误差 (RMS/m)', '均值 (Mean/m)', ...
    '中位数 (Median/m)', '95%分位数 (95%/m)', ...
    'CEP50 (m)', 'CEP50 (nmi)', '位置误差率 (nmi/h)', '误差率备注'};
statistics_body = cell(nav_count,11);
plot_time = cell(nav_count,1);
plot_radial = cell(nav_count,1);
format = '%*f %f %f %f %f %*f %*f %*f %*f %*f %*f';
chunk_size = 200000;

for file_index = 1:nav_count
    fid_nav = fopen(nav_paths{file_index},'r');
    if fid_nav < 0, error('无法读取NAV：%s',nav_paths{file_index}); end
    nav_cleanup = onCleanup(@() fclose(fid_nav));
    first_line = fgetl(fid_nav);
    second_line = fgetl(fid_nav);
    first_values = sscanf(first_line,'%f')';
    second_values = sscanf(second_line,'%f')';
    nav_dt_s = second_values(2)-first_values(2);
    fseek(fid_nav,0,'bof');
    expected_count = round((common_end_s-common_start_s)/nav_dt_s)+1;
    radial_error_m = nan(expected_count+10,1);
    time_s = nan(expected_count+10,1);
    output_index = 0;

    fid_truth = fopen(truth_path,'rb');
    if fid_truth < 0, error('无法读取真值：%s',truth_path); end
    truth_cleanup = onCleanup(@() fclose(fid_truth));
    while ~feof(fid_nav)
        values = textscan(fid_nav,format,chunk_size,'CollectOutput',true, ...
            'ReturnOnError',false);
        navigation = values{1};
        if isempty(navigation), break; end
        mask = navigation(:,1) >= common_start_s & ...
            navigation(:,1) <= common_end_s;
        navigation = navigation(mask,:);
        if isempty(navigation), continue; end
        navigation_time = navigation(:,1);
        truth_rows = round((navigation_time-truth_first_s)/truth_dt_s)+1;
        if any(diff(truth_rows) ~= 1)
            error('NAV时间轴无法映射到定长真值记录：%s',nav_paths{file_index});
        end
        if fseek(fid_truth,(truth_rows(1)-1)*truth_record_bytes,'bof') ~= 0
            error('无法定位真值第%d行。',truth_rows(1));
        end
        truth_values = textscan(fid_truth,format,numel(truth_rows), ...
            'CollectOutput',true,'ReturnOnError',false);
        truth = truth_values{1};
        if size(truth,1) ~= numel(truth_rows) || ...
                max(abs(truth(:,1)-navigation_time)) > 1e-7
            error('NAV与真值时间不一致：%s',nav_paths{file_index});
        end
        [north_m,east_m] = calculate_gjb_horizontal_error( ...
            navigation(:,2:4),truth(:,2:4));
        count = numel(navigation_time);
        target = output_index+(1:count);
        if target(end) > numel(radial_error_m)
            growth = max(chunk_size,target(end)-numel(radial_error_m));
            radial_error_m(end+growth,1) = nan;
            time_s(end+growth,1) = nan;
        end
        radial_error_m(target) = hypot(north_m,east_m);
        time_s(target) = navigation_time;
        output_index = target(end);
        clear values navigation mask navigation_time truth_rows ...
            truth_values truth north_m east_m;
    end
    clear nav_cleanup truth_cleanup;
    radial_error_m = radial_error_m(1:output_index);
    time_s = time_s(1:output_index);
    relative_time_s = time_s-time_s(1);
    if apply_grubbs
        keep = grubbs_keep_mask(radial_error_m);
        if any(~keep) && nnz(keep) >= 2
            radial_error_m(~keep) = interp1(time_s(keep), ...
                radial_error_m(keep),time_s(~keep),'linear','extrap');
        end
    end

    maximum_m = max(radial_error_m);
    rmse_m = sqrt(mean(radial_error_m.^2));
    mean_m = mean(radial_error_m);
    median_m = median(radial_error_m);
    p95_m = quantile(radial_error_m,0.95);
    cep50_m = quantile(radial_error_m,0.50);
    rate_end_s = min(3600,relative_time_s(end));
    rate_mask = relative_time_s > 0 & relative_time_s <= rate_end_s;
    if any(rate_mask)
        rer = (radial_error_m(rate_mask)/1852) ./ ...
            (relative_time_s(rate_mask)/3600);
        position_error_rate = 0.8326*sqrt(mean(rer.^2));
    else
        position_error_rate = nan;
    end
    if relative_time_s(end) >= 3600
        rate_note = '前3600s';
    else
        rate_note = sprintf('有效时长不足3600s，仅%.1fs', ...
            relative_time_s(end));
    end
    [~,label] = fileparts(nav_paths{file_index});
    statistics_body(file_index,:) = {label,maximum_m,maximum_m/1852, ...
        rmse_m,mean_m,median_m,p95_m,cep50_m,cep50_m/1852, ...
        position_error_rate,rate_note};
    display_index = unique(round(linspace(1,numel(relative_time_s), ...
        min(maximum_plot_points,numel(relative_time_s)))));
    plot_time{file_index} = relative_time_s(display_index);
    plot_radial{file_index} = radial_error_m(display_index);
    clear radial_error_m time_s relative_time_s rate_mask rer;
end

if exist('myfigurestartup','file') == 2
    figure_handle = myfigurestartup(6,4,'prese');
else
    figure_handle = figure('Color','w');
end
axes_handle = axes(figure_handle);
hold(axes_handle,'on'); grid(axes_handle,'on'); box(axes_handle,'on');
colors = [0.00 0.00 0.00;0.85 0.15 0.15;0.10 0.35 0.70; ...
    0.10 0.60 0.20;0.60 0.20 0.70;0.95 0.50 0.10];
for index = 1:nav_count
    plot(axes_handle,plot_time{index},plot_radial{index}, ...
        'Color',colors(mod(index-1,size(colors,1))+1,:), ...
        'LineWidth',1.0,'DisplayName',statistics_body{index,1});
end
xlabel(axes_handle,'时间 (s)');
ylabel(axes_handle,'水平径向误差 (m)');
legend(axes_handle,'show','Location','best');
statistics_cell = [statistics_header;statistics_body];
end

function [fixed_record,first_time_s,dt_s,record_bytes] = ...
        fixed_truth_record_information(path)
fid = fopen(path,'rb');
if fid < 0, error('无法读取真值：%s',path); end
cleanup = onCleanup(@() fclose(fid));
line1 = fgetl(fid);
record_bytes = ftell(fid);
line2 = fgetl(fid);
second_record_bytes = ftell(fid)-record_bytes;
values1 = sscanf(line1,'%f')';
values2 = sscanf(line2,'%f')';
first_time_s = values1(2);
dt_s = values2(2)-values1(2);
fseek(fid,0,'eof');
file_bytes = ftell(fid);
fixed_record = record_bytes > 0 && second_record_bytes == record_bytes && ...
    mod(file_bytes,record_bytes) == 0 && dt_s > 0;
end

function data = read_time_position_columns(path)
fid = fopen(path,'r');
if fid < 0, error('无法读取文件：%s',path); end
cleanup = onCleanup(@() fclose(fid));
format = '%*f %f %f %f %f %*f %*f %*f %*f %*f %*f';
values = textscan(fid,format,'CollectOutput',true, ...
    'ReturnOnError',false);
data = values{1};
if size(data,2) ~= 4 || isempty(data) || any(~isfinite(data),'all')
    error('文件时间或位置列格式错误：%s',path);
end
end

function truth_at_navigation = align_truth_position( ...
        truth_time,truth_position,navigation_time)
truth_dt = median(diff(truth_time));
indices = round((navigation_time-truth_time(1))/truth_dt)+1;
exact_grid = all(indices >= 1 & indices <= numel(truth_time));
if exact_grid
    exact_grid = max(abs(truth_time(indices)-navigation_time)) <= 1e-7;
end
if exact_grid
    truth_at_navigation = truth_position(indices,:);
else
    truth_at_navigation = interp1(truth_time,truth_position, ...
        navigation_time,'linear','extrap');
end
end

function [north_error_m,east_error_m] = calculate_gjb_horizontal_error( ...
        navigation_position,truth_position)
a = 6378137.0;
e2 = 6.69437999014e-3;
truth_latitude_rad = deg2rad(truth_position(:,1));
denominator = sqrt(1-e2*sin(truth_latitude_rad).^2);
rn_m = a./denominator;
rm_m = a*(1-e2)./denominator.^3;
north_error_m = deg2rad(navigation_position(:,1)-truth_position(:,1)).* ...
    (rm_m+truth_position(:,3));
east_error_m = deg2rad(navigation_position(:,2)-truth_position(:,2)).* ...
    (rn_m+truth_position(:,3)).*cos(truth_latitude_rad);
end

function keep = grubbs_keep_mask(data)
keep = true(size(data));
n_table = [3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 30 35 40 45 50 60 70 80 90 100];
t_table = [1.15 1.46 1.67 1.89 2.02 2.13 2.21 2.29 2.36 2.41 2.46 2.51 2.55 2.59 2.62 2.65 2.68 2.71 2.73 2.76 2.78 2.80 2.82 2.91 2.98 3.04 3.09 3.13 3.20 3.26 3.31 3.35 3.38];
while true
    active = data(keep);
    count = numel(active);
    if count < 3, break; end
    average = mean(active);
    sigma = std(active);
    if sigma == 0, break; end
    if count <= 100
        [~,nearest] = min(abs(n_table-count));
        threshold = t_table(nearest);
        deviation = abs(active-average)/sigma;
        [maximum_deviation,suspect] = max(deviation);
        if maximum_deviation <= threshold, break; end
        active_indices = find(keep);
        keep(active_indices(suspect)) = false;
    else
        threshold = sqrt(2*log(count));
        bad = abs(active-average)/sigma > threshold;
        if ~any(bad), break; end
        active_indices = find(keep);
        keep(active_indices(bad)) = false;
    end
end
end

function values = normalize_string_list(values)
if ischar(values)
    values = string({values});
elseif iscell(values)
    values = string(values(:));
else
    values = string(values(:));
end
values = values(strlength(values) > 0);
end

function truth_path = find_truth_path(input_root, data_source)
if data_source == "simulation"
    candidates = ["truth.txt", "truth.nav"];
else
    candidates = ["truth.nav", "truth.txt"];
end
truth_path = "";
for index = 1:numel(candidates)
    candidate = fullfile(input_root, candidates(index));
    if isfile(candidate)
        truth_path = string(candidate);
        return;
    end
end
end

function format_comparison_figure(figure_handle, mode_names, ...
        data_source, case_id, algorithm_name, maximum_plot_points)
axes_handles = findobj(figure_handle, 'Type', 'Axes');
if isempty(axes_handles), return; end
axes_handle = axes_handles(1);
line_handles = flipud(findobj(axes_handle, 'Type', 'Line'));
line_count = min(numel(line_handles), numel(mode_names));
for index = 1:line_count
    set(line_handles(index), 'DisplayName', mode_names(index), ...
        'LineWidth', 1.0);
    x = get(line_handles(index), 'XData');
    y = get(line_handles(index), 'YData');
    if numel(x) > maximum_plot_points
        display_index = unique(round(linspace(1, numel(x), ...
            maximum_plot_points)));
        set(line_handles(index), 'XData', x(display_index), ...
            'YData', y(display_index));
    end
end
title(axes_handle, sprintf('%s/%s：%s潜标位置处理对比', ...
    data_source, case_id, algorithm_name), 'Interpreter', 'none');
legend(axes_handle, 'show', 'Location', 'best');
set(findall(figure_handle, '-property', 'FontName'), ...
    'FontName', 'TimesSimSun');
end

function summary = build_algorithm_summary(data_source, case_id, study_id, ...
        algorithm, mode_ids, mode_names, nav_paths, statistics_cell, ...
        truth_start_s, truth_end_s)
row_count = numel(mode_ids);
maximum_m = cell2mat(statistics_cell(2:end,2));
rmse_m = cell2mat(statistics_cell(2:end,4));
mean_m = cell2mat(statistics_cell(2:end,5));
median_m = cell2mat(statistics_cell(2:end,6));
p95_m = cell2mat(statistics_cell(2:end,7));
cep50_m = cell2mat(statistics_cell(2:end,8));
position_error_rate = cell2mat(statistics_cell(2:end,10));

nav_start_s = nan(row_count,1);
nav_end_s = nan(row_count,1);
for index = 1:row_count
    [nav_start_s(index), nav_end_s(index)] = ...
        read_text_time_bounds(nav_paths(index));
end
evaluation_start_s = max([truth_start_s; nav_start_s]);
evaluation_end_s = min([truth_end_s; nav_end_s]);
evaluation_duration_s = evaluation_end_s-evaluation_start_s;

improvement = nan(row_count,1);
gap_to_truth = nan(row_count,1);
uncompensated_index = find(mode_ids == "uncompensated-rad", 1);
truth_index = find(mode_ids == "truth-beacon-reference-rad", 1);
if ~isempty(uncompensated_index) && rmse_m(uncompensated_index) > 0
    improvement = 100*(rmse_m(uncompensated_index)-rmse_m) ./ ...
        rmse_m(uncompensated_index);
end
if ~isempty(truth_index) && rmse_m(truth_index) > 0
    gap_to_truth = 100*(rmse_m-rmse_m(truth_index)) ./ rmse_m(truth_index);
end

summary = table( ...
    repmat(string(data_source),row_count,1), ...
    repmat(string(case_id),row_count,1), ...
    repmat(string(study_id),row_count,1), ...
    repmat(string(algorithm.id),row_count,1), ...
    repmat(string(algorithm.name),row_count,1), ...
    string(mode_ids(:)), string(mode_names(:)), string(nav_paths(:)), ...
    repmat(evaluation_start_s,row_count,1), ...
    repmat(evaluation_end_s,row_count,1), ...
    repmat(evaluation_duration_s,row_count,1), ...
    maximum_m, rmse_m, mean_m, median_m, p95_m, cep50_m, ...
    position_error_rate, improvement, gap_to_truth, ...
    'VariableNames', empty_summary_table().Properties.VariableNames);
end

function summary = empty_summary_table()
summary = table( ...
    strings(0,1), strings(0,1), strings(0,1), strings(0,1), ...
    strings(0,1), strings(0,1), strings(0,1), strings(0,1), ...
    zeros(0,1), zeros(0,1), zeros(0,1), zeros(0,1), ...
    zeros(0,1), zeros(0,1), zeros(0,1), zeros(0,1), ...
    zeros(0,1), zeros(0,1), zeros(0,1), zeros(0,1), ...
    'VariableNames', { ...
    'DataSource','DatasetId','StudyId','AlgorithmId','AlgorithmName', ...
    'ModeId','ModeName','NavPath','EvaluationStart_s','EvaluationEnd_s', ...
    'EvaluationDuration_s','Maximum_m','RMSE_m','Mean_m','Median_m', ...
    'P95_m','CEP50_m','PositionErrorRate_nmi_per_h', ...
    'RMSEImprovementVsUncompensated_pct','RMSEGapVsTruth_pct'});
end

function [first_time_s, last_time_s] = read_text_time_bounds(path)
fid = fopen(path, 'rb');
if fid < 0, error('无法读取文件：%s', path); end
cleanup = onCleanup(@() fclose(fid));

first_time_s = nan;
while ~feof(fid)
    line = fgetl(fid);
    if ~ischar(line), break; end
    values = sscanf(line, '%f')';
    if numel(values) >= 2 && isfinite(values(2))
        first_time_s = values(2);
        break;
    end
end
if ~isfinite(first_time_s)
    error('文件没有有效时间记录：%s', path);
end

fseek(fid, 0, 'eof');
file_bytes = ftell(fid);
tail_bytes = min(file_bytes, 1024*1024);
fseek(fid, file_bytes-tail_bytes, 'bof');
tail = fread(fid, [1, tail_bytes], '*char');
lines = splitlines(string(tail));
last_time_s = nan;
for index = numel(lines):-1:1
    values = sscanf(char(lines(index)), '%f')';
    if numel(values) >= 2 && isfinite(values(2))
        last_time_s = values(2);
        break;
    end
end
if ~isfinite(last_time_s)
    error('无法读取文件末尾时间：%s', path);
end
end

function write_analysis_report(report_path, data_source, case_id, study_id, ...
        truth_path, result_root, artifact_root, summary, messages, options)
fid = fopen(report_path, 'w', 'n', 'UTF-8');
if fid < 0
    warning('无法写入分析报告：%s', report_path);
    return;
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '时变潜标导航误差分析报告\n');
fprintf(fid, '========================================\n');
fprintf(fid, '数据源：%s\n', data_source);
fprintf(fid, '数据集：%s\n', case_id);
fprintf(fid, '工况：%s\n', study_id);
fprintf(fid, '真值：%s\n', truth_path);
fprintf(fid, 'NAV根目录：%s\n', result_root);
fprintf(fid, '输出目录：%s\n', artifact_root);
fprintf(fid, 'Grubbs处理：%d\n', options.ApplyGrubbs);
fprintf(fid, '统计结果行数：%d\n', height(summary));
if ~isempty(summary)
    fprintf(fid, '统一评价时段：%.6f ~ %.6f s\n', ...
        min(summary.EvaluationStart_s), max(summary.EvaluationEnd_s));
end
if ~isempty(messages)
    fprintf(fid, '\n跳过或异常信息：\n');
    for index = 1:numel(messages)
        fprintf(fid, '- %s\n', messages(index));
    end
end
fprintf(fid, '\n本报告及统计表均由truth与NAV重新计算，未读取旧分析文件。\n');
end
