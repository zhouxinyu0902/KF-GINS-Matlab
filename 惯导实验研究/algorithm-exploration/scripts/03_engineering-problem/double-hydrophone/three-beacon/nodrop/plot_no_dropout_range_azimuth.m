clear;clc;close all;
%% 1. 基本配置
analysis_jobs = {
    "simulation", 'case-00'
    "experiment", 'case-06'};
phase_std_degs = [1,3,5,10];
stage_ids = {'forward','rts-single','rts-double'};
stage_labels = {'前向 EKF','一次 RTS','二次 RTS'};
range_only_names = {'simple-forward-ekf-rad.nav','simple-rts-single-rad.nav','simple-rts-double-rad.nav'};
range_azimuth_names = {'range-phase-azimuth-forward.nav','range-phase-azimuth-rts-single.nav','range-phase-azimuth-rts-double.nav'};
apply_grubbs = false;
show_figures = false;
export_resolution_dpi = 600;
method_count = numel(phase_std_degs)+1;
method_labels = cell(1,method_count);
method_labels{1} = '仅测距';
for i = 1:numel(phase_std_degs)
    method_labels{i+1} = sprintf('\\sigma_{\\phi}=%g^\\circ',phase_std_degs(i));
end
for job_index = 1:size(analysis_jobs,1)
data_source = analysis_jobs{job_index,1};
case_name = analysis_jobs{job_index,2};
fprintf('\n============================================================\n');
fprintf('开始无掉帧分析：%s / %s\n',data_source,case_name);
fprintf('============================================================\n');
%% 2. 路径配置
script_dir = fileparts(mfilename('fullpath'));
algorithm_root = fileparts(fileparts(fileparts(fileparts(script_dir))));
addpath(algorithm_root);
paths = setup_inertial_experiment();
case_id = sscanf(case_name,'case-%d',1);
if isempty(case_id) || ~isscalar(case_id) || case_id<0 || case_id~=fix(case_id)
    error('case_name 必须采用 case-XX 格式。');
end
if data_source == "experiment"
    input_dir = paths.experiment_input(case_id);
    navigation_dir = paths.experiment_navigation(case_id);
    artifact_root = paths.experiment_artifacts(case_id);
    cfg = load_algorithm_exploration_config('experiment','rad',input_dir);
elseif data_source == "simulation"
    input_dir = paths.simulation_input(case_id);
    navigation_dir = paths.simulation_navigation(case_id);
    artifact_root = paths.simulation_artifacts(case_id);
    cfg = load_algorithm_exploration_config('simulation','rad',input_dir);
end
truth_path = cfg.truthpath;
navigation_root = fullfile(navigation_dir,'navigation-results');
range_only_dir = fullfile(navigation_root,'simple-ekf-rts-rad');
analysis_dir = fullfile(artifact_root,'no-dropout-range-azimuth-analysis');
figure_dir = fullfile(analysis_dir,'figures');
if ~isfolder(analysis_dir), mkdir(analysis_dir); end
if ~isfolder(figure_dir), mkdir(figure_dir); end
gjb_function_dir = fullfile(paths.project,'function_zxy','plot-function');
if isfolder(gjb_function_dir), addpath(gjb_function_dir); end
if exist('calc_radial_error_gjb','file')~=2
    error('未找到 calc_radial_error_gjb.m。');
end
if ~isfile(truth_path), error('真值文件不存在：%s',truth_path); end
if ~isfolder(range_only_dir), error('仅测距结果目录不存在：%s',range_only_dir); end
%% 3. 汇总表初始化
summary_csv_path = fullfile(analysis_dir,'no-dropout-range-azimuth-summary.csv');
summary_xlsx_path = fullfile(analysis_dir,'no-dropout-range-azimuth-summary.xlsx');
if isfile(summary_xlsx_path), delete(summary_xlsx_path); end
summary_header = {
    '处理阶段','方法','是否使用测向','相位差噪声标准差 (deg)',...
    '导航文件','RMS (m)','Max (m)','RER (n mile/h)','Median (m)',...
    'RMS 相对仅测距改善 (%)','Max 相对仅测距改善 (%)',...
    'RER 相对仅测距改善 (%)','Median 相对仅测距改善 (%)'};
summary_body = cell(numel(stage_ids)*method_count,numel(summary_header));
summary_row = 0;
%% 4. 分阶段计算导航误差
for s = 1:numel(stage_ids)
    stage_id = stage_ids{s};
    stage_label = stage_labels{s};
    comparison_paths = cell(method_count,1);
    comparison_paths{1} = fullfile(range_only_dir,range_only_names{s});
    for p = 1:numel(phase_std_degs)
        phase_dir = fullfile(navigation_root,sprintf('double-hydrophone-phasestd%gdeg',phase_std_degs(p)));
        comparison_paths{p+1} = fullfile(phase_dir,range_azimuth_names{s});
    end
    missing_mask = ~cellfun(@isfile,comparison_paths);
    if any(missing_mask)
        error('导航文件不存在：\n%s',strjoin(comparison_paths(missing_mask),newline));
    end
    fprintf('\n正在分析：%s，共 %d 种方法。\n',stage_label,method_count);
    [fig,stats] = calc_radial_error_gjb(truth_path,comparison_paths{:},apply_grubbs);
    numeric_stats = extract_gjb_metrics(stats,method_count,stage_label);
    stats(2:end,1) = method_labels(:);
    format_gjb_figure(fig,method_labels,stage_label,show_figures);
    output_stem = sprintf('no-dropout-%s-radial-error',stage_id);
    exportgraphics(fig,fullfile(figure_dir,[output_stem '.png']),'Resolution',export_resolution_dpi);
    savefig(fig,fullfile(figure_dir,[output_stem '.fig']));
    if ~show_figures && isgraphics(fig), close(fig); end
    writecell(stats,summary_xlsx_path,'Sheet',stage_id);
    for m = 1:method_count
        use_azimuth = m>1;
        phase_std = NaN;
        if use_azimuth, phase_std = phase_std_degs(m-1); end
        improvement = nan(1,4);
        for metric_index = 1:4
            improvement(metric_index) = relative_reduction_percent( ...
                numeric_stats(m,metric_index),numeric_stats(1,metric_index));
        end
        summary_row = summary_row+1;
        summary_body(summary_row,:) = [...
            {stage_label,method_labels{m},use_azimuth,phase_std,comparison_paths{m}},...
            num2cell(numeric_stats(m,:)),num2cell(improvement)];
    end
end
%% 5. 导出汇总表
summary_body = summary_body(1:summary_row,:);
summary_cell = [summary_header;summary_body];
writecell(summary_cell,summary_csv_path);
writecell(summary_cell,summary_xlsx_path,'Sheet','summary');
metric_definitions = {
    '项目','定义或说明'
    '对比范围','无掉帧仅测距与无掉帧测距+测向'
    '水平径向误差','r(t)=sqrt(e_N(t)^2+e_E(t)^2)，单位 m'
    'RMS','sqrt(mean(r(t)^2))，单位 m'
    'Max','max(r(t))，单位 m'
    'RER','末时刻水平径向误差（海里）/累计时长（小时），单位 n mile/h'
    'RER 窗口','评价时长不超过 3600 s；采用窗口末端误差'
    'Median','median(r(t))，单位 m'
    '公共时间段','同一处理阶段五个导航结果共同覆盖的时间段'
    '相对改善','100*(仅测距指标-测距+测向指标)/仅测距指标'
    'Grubbs',sprintf('apply_grubbs=%d',apply_grubbs)};
writecell(metric_definitions,summary_xlsx_path,'Sheet','指标说明');
%% 6. 生成汇总图
delete_obsolete_combined_absolute_figure(figure_dir);
create_absolute_metric_group_figure(summary_body,stage_labels, ...
    method_labels,[6,7],{'RMS (m)','Max (m)'},1,2, ...
    11,5.4,'无掉帧核心误差指标', ...
    'summary-primary-rms-max',figure_dir,show_figures, ...
    export_resolution_dpi);
create_absolute_metric_group_figure(summary_body,stage_labels, ...
    method_labels,[8,9],{'RER (n mile/h)','Median (m)'},1,2,11,5.4, ...
    '无掉帧补充误差指标','summary-supplementary-metrics', ...
    figure_dir,show_figures,export_resolution_dpi);
delete_obsolete_combined_heatmap(figure_dir);

% 每个指标单独成图，横轴为处理阶段，纵轴为相位差噪声标准差。
heatmap_metrics = {
    'rms',    'RMS',     6
    'max',    'MAX',     7
    'rer',    'RER',     8
    'median', 'Median',  9};
for metric_index = 1:size(heatmap_metrics,1)
    metric_id = heatmap_metrics{metric_index,1};
    metric_title = heatmap_metrics{metric_index,2};
    metric_column = heatmap_metrics{metric_index,3};
    create_single_metric_improvement_heatmap(summary_body,stage_labels, ...
        method_labels(2:end),metric_column,metric_title, ...
        sprintf('no-dropout-azimuth-improvement-%s-heatmap',metric_id), ...
        figure_dir,show_figures,export_resolution_dpi);
end
fprintf('\n无掉帧对比分析完成。\n');
fprintf('CSV：%s\nExcel：%s\n图片目录：%s\n',summary_csv_path,summary_xlsx_path,figure_dir);
end
%% 局部函数
function numeric_metrics = extract_gjb_metrics(statistics_cell,expected_count,context)
required_headers = {'RMS (m)','Max (m)','RER (n mile/h)','Median (m)'};
if size(statistics_cell,1) ~= expected_count+1
    error('%s 的 GJB 统计表行数异常。',context);
end
headers = string(statistics_cell(1,:));
metric_columns = zeros(1,numel(required_headers));
for metric_index = 1:numel(required_headers)
    column_index = find(headers == required_headers{metric_index},1);
    if isempty(column_index)
        error('%s 缺少评价指标列：%s。',context,required_headers{metric_index});
    end
    metric_columns(metric_index) = column_index;
end
numeric_metrics = cell2mat(statistics_cell(2:end,metric_columns));
end

function format_gjb_figure(fig,labels,stage_label,show_figures)
if ~isgraphics(fig), return; end
if ~show_figures, set(fig,'Visible','off'); end
ax = findobj(fig,'Type','Axes');
if isempty(ax), return; end
ax = ax(1);
lines_handle = flipud(findobj(ax,'Type','Line'));
n = min(numel(lines_handle),numel(labels));
for i = 1:n
    set(lines_handle(i),'DisplayName',labels{i},'LineWidth',1.1);
end
title(ax,sprintf('无掉帧：%s',stage_label),'Interpreter','none');
xlabel(ax,'时间 (s)');
ylabel(ax,'水平径向误差 (m)');
legend(ax,'show','Location','best','Interpreter','tex');
grid(ax,'on');box(ax,'on');
set(findall(fig,'-property','FontName'),'FontName','TimesSimSun');
end
function reduction = relative_reduction_percent(value,baseline)
reduction = NaN;
if isfinite(value) && isfinite(baseline) && baseline~=0
    reduction = 100*(baseline-value)/baseline;
end
end
function delete_obsolete_combined_heatmap(figure_dir)
obsolete_stems = {'summary-relative-improvement', ...
    'no-dropout-azimuth-improvement-cep50-heatmap', ...
    'no-dropout-azimuth-improvement-p95-heatmap', ...
    'no-dropout-azimuth-improvement-mean-heatmap', ...
    'no-dropout-azimuth-improvement-position-error-rate-heatmap'};
for stem_index = 1:numel(obsolete_stems)
    for extension = {'.png','.fig'}
        obsolete_path = fullfile(figure_dir, ...
            [obsolete_stems{stem_index} extension{1}]);
        if isfile(obsolete_path)
            delete(obsolete_path);
        end
    end
end
end
function delete_obsolete_combined_absolute_figure(figure_dir)
for extension = {'.png','.fig'}
    obsolete_path = fullfile(figure_dir, ...
        ['summary-absolute-metrics' extension{1}]);
    if isfile(obsolete_path)
        delete(obsolete_path);
    end
end
end
function create_absolute_metric_group_figure(summary_body,stage_labels, ...
        method_labels,metric_columns,metric_labels,layout_rows, ...
        layout_columns,figure_width,figure_height,overall_title, ...
        output_stem,figure_dir,show_figures,dpi)
colors = lines(numel(method_labels));
markers = {'o','s','^','d','v'};
fig = myfigurestartup(figure_width,figure_height,'prese');
if ~show_figures, set(fig,'Visible','off'); end
tl = tiledlayout(fig,layout_rows,layout_columns, ...
    'TileSpacing','loose','Padding','compact');
legend_handles = gobjects(numel(method_labels),1);
for k = 1:numel(metric_columns)
    ax = nexttile(tl);
    hold(ax,'on');
    for m = 1:numel(method_labels)
        values = nan(numel(stage_labels),1);
        for s = 1:numel(stage_labels)
            mask = strcmp(summary_body(:,1),stage_labels{s}) & strcmp(summary_body(:,2),method_labels{m});
            if any(mask)
                values(s) = summary_body{find(mask,1),metric_columns(k)};
            end
        end
        h = plot(ax,1:numel(stage_labels),values,['-' markers{m}], ...
            'Color',colors(m,:),'LineWidth',1.3,'MarkerSize',6, ...
            'DisplayName',method_labels{m});
        if k == 1
            legend_handles(m) = h;
        end
    end
    xticks(ax,1:numel(stage_labels));
    xticklabels(ax,stage_labels);
    xtickangle(ax,20);
    ylabel(ax,metric_labels{k});
    title(ax,metric_labels{k});
    grid(ax,'on');
    box(ax,'on');
end
lgd = legend(legend_handles,method_labels,'Orientation','horizontal','Interpreter','tex','NumColumns',numel(method_labels));
lgd.Layout.Tile = 'north';
title(tl,overall_title,'Interpreter','none');
set(findall(fig,'-property','FontName'),'FontName','TimesSimSun');
exportgraphics(fig,fullfile(figure_dir,[output_stem '.png']),'Resolution',dpi);
savefig(fig,fullfile(figure_dir,[output_stem '.fig']));
if ~show_figures, close(fig); end
end
function create_single_metric_improvement_heatmap(summary_body, ...
        stage_labels,phase_labels,metric_column,metric_title,output_stem, ...
        figure_dir,show_figures,dpi)
values = nan(numel(phase_labels),numel(stage_labels));
for stage_index = 1:numel(stage_labels)
    baseline_mask = strcmp(summary_body(:,1),stage_labels{stage_index}) & ...
        strcmp(summary_body(:,2),'仅测距');
    if ~any(baseline_mask)
        continue;
    end
    baseline_value = summary_body{find(baseline_mask,1),metric_column};
    for method_index = 1:numel(phase_labels)
        mask = strcmp(summary_body(:,1),stage_labels{stage_index}) & ...
            strcmp(summary_body(:,2),phase_labels{method_index});
        if any(mask)
            value = summary_body{find(mask,1),metric_column};
            values(method_index,stage_index) = ...
                relative_reduction_percent(value,baseline_value);
        end
    end
end

finite_values = values(isfinite(values));
if isempty(finite_values)
    color_limit = 10;
else
    color_limit = max(10,ceil(max(abs(finite_values))/10)*10);
end

fig = myfigurestartup(3,3,'prese');
if ~show_figures, set(fig,'Visible','off'); end
axes_handles = findobj(fig,'Type','Axes');
if isempty(axes_handles)
    ax = axes('Parent',fig);
else
    ax = axes_handles(1);
    cla(ax,'reset');
end
imagesc(ax,values);
colormap(ax,red_white_blue_colormap(128));
clim(ax,[-color_limit,color_limit]);
xlim(ax,[0.5,numel(stage_labels)+0.5]);
ylim(ax,[0.5,numel(phase_labels)+0.5]);
xticks(ax,1:numel(stage_labels));
xticklabels(ax,stage_labels);
yticks(ax,1:numel(phase_labels));
yticklabels(ax,phase_labels);
set(ax,'TickLabelInterpreter','tex','FontSize',9, ...
    'LineWidth',0.8,'Box','on','Layer','top');
xlabel(ax,'处理阶段');
title(ax,sprintf('无掉帧 / %s',metric_title), ...
    'Interpreter','none','FontWeight','normal');
cb = colorbar(ax);
cb.Ticks = linspace(-color_limit,color_limit,5);
cb.Label.String = '改善 (%)';
cb.FontSize = 8;
for method_index = 1:size(values,1)
    for stage_index = 1:size(values,2)
        if isfinite(values(method_index,stage_index))
            label_text = sprintf('%+.1f%%', ...
                values(method_index,stage_index));
        else
            label_text = '--';
        end
        text(ax,stage_index,method_index,label_text, ...
            'HorizontalAlignment','center', ...
            'VerticalAlignment','middle','FontSize',8, ...
            'FontWeight','bold','Interpreter','none', ...
            'Color',[0.08,0.08,0.08]);
    end
end
set(findall(fig,'-property','FontName'),'FontName','TimesSimSun');
exportgraphics(fig,fullfile(figure_dir,[output_stem '.png']), ...
    'Resolution',dpi);
savefig(fig,fullfile(figure_dir,[output_stem '.fig']));
if ~show_figures, close(fig); end
end

function cmap = red_white_blue_colormap(n)
red = [0.85,0.30,0.25];
blue = [0.25,0.45,0.80];
cmap = [linspace(red(1),1,n)',linspace(red(2),1,n)',linspace(red(3),1,n)';...
        linspace(1,blue(1),n)',linspace(1,blue(2),n)',linspace(1,blue(3),n)'];
end
