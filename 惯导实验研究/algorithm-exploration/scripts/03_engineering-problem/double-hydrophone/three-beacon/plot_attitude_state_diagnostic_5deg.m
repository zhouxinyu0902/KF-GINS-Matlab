close all;
clc;
%% 5° 相位误差下的姿态误差状态逐帧诊断
% 默认读取全姿态反馈的独立诊断结果。若需检查当前主结果，可将
% result_name_suffix 设为空字符串。
if ~exist('data_source', 'var') || isempty(data_source)
    data_source = "simulation";
end
if ~exist('case_name', 'var') || isempty(case_name)
    case_name = 'case-00';
end
if ~exist('phasestd', 'var') || isempty(phasestd)
    phasestd = 5;
end
if ~exist('result_name_suffix', 'var') || isempty(result_name_suffix)
    result_name_suffix = 'full-attitude-diagnostic';
end
if phasestd ~= 5
    error('本脚本用于 Phase STD = 5° 的逐帧姿态诊断。');
end

script_dir = fileparts(mfilename('fullpath'));
topic_dir = fileparts(fileparts(fileparts(script_dir)));
addpath(topic_dir);
paths = setup_inertial_experiment();

case_id = sscanf(case_name, 'case-%d', 1);
result_name = sprintf('double-hydrophone-phasestd%ddeg', phasestd);
if ~isempty(result_name_suffix)
    result_name = sprintf('%s-%s', result_name, char(result_name_suffix));
end
if lower(string(data_source)) == "simulation"
    result_dir = fullfile(paths.simulation_navigation(case_id), ...
        'navigation-results', result_name);
elseif lower(string(data_source)) == "experiment"
    result_dir = fullfile(paths.experiment_navigation(case_id), ...
        'navigation-results', result_name);
else
    error('data_source 只能设置为 "simulation" 或 "experiment"。');
end

diagnostic_mat_path = fullfile(result_dir, ...
    'range-phase-azimuth-attitude-state-diagnostic.mat');
if ~isfile(diagnostic_mat_path)
    error(['找不到姿态状态诊断文件：%s\n请先运行 ' ...
        'run_double_hydrophone_forward_ekf.m。'], diagnostic_mat_path);
end
diagnostic = load(diagnostic_mat_path);
data = diagnostic.attitude_state_diagnostic_data;
feedback_mode = string(diagnostic.attitude_feedback_mode);

event_id = data(:, 1);
true_prior_deg = data(:, 4:6);
estimate_prior_deg = data(:, 7:9);
sigma_prior_deg = data(:, 10:12);
measurement_increment_deg = data(:, 13:15);
estimate_post_deg = data(:, 16:18);
sigma_post_deg = data(:, 19:21);
feedback_deg = data(:, 22:24);
true_after_feedback_deg = data(:, 25:27);
post_error_deg = data(:, 28:30);
normalized_error = data(:, 31:33);

axis_names = {'横滚', '俯仰', '航向'};
prior_rmse_deg = sqrt(mean((estimate_prior_deg - true_prior_deg) .^ 2, 1));
post_rmse_deg = sqrt(mean(post_error_deg .^ 2, 1));
after_feedback_rmse_deg = sqrt(mean(true_after_feedback_deg .^ 2, 1));
one_sigma_coverage_pct = 100 * mean(abs(normalized_error) <= 1, 1);
two_sigma_coverage_pct = 100 * mean(abs(normalized_error) <= 2, 1);

figure_handle = figure('Color', 'w', 'Position', [80, 60, 1450, 920]);
layout = tiledlayout(figure_handle, 3, 2, ...
    'TileSpacing', 'compact', 'Padding', 'compact');
for axis_index = 1:3
    estimate_axis = nexttile(layout, 2 * axis_index - 1);
    hold(estimate_axis, 'on');
    grid(estimate_axis, 'on');
    box(estimate_axis, 'on');
    errorbar(estimate_axis, event_id, estimate_post_deg(:, axis_index), ...
        2 * sigma_post_deg(:, axis_index), 'Color', [0.85, 0.35, 0.25], ...
        'LineStyle', 'none', 'CapSize', 4, 'HandleVisibility', 'off');
    plot(estimate_axis, event_id, true_prior_deg(:, axis_index), ...
        'ko-', 'LineWidth', 1.25, 'MarkerSize', 4, ...
        'DisplayName', '真实待校正误差');
    plot(estimate_axis, event_id, estimate_prior_deg(:, axis_index), ...
        'b.--', 'LineWidth', 1.0, 'MarkerSize', 10, ...
        'DisplayName', '量测前状态估计');
    plot(estimate_axis, event_id, estimate_post_deg(:, axis_index), ...
        'r.-', 'LineWidth', 1.15, 'MarkerSize', 11, ...
        'DisplayName', '量测后状态估计');
    plot(estimate_axis, event_id, feedback_deg(:, axis_index), ...
        'd-', 'Color', [0.10, 0.55, 0.25], 'LineWidth', 1.0, ...
        'MarkerSize', 4, 'DisplayName', '实际反馈量');
    yline(estimate_axis, 0, 'k:', 'HandleVisibility', 'off');
    ylabel(estimate_axis, sprintf('%s / deg', axis_names{axis_index}));
    title(estimate_axis, sprintf( ...
        '%s：真实误差与滤波估计（误差棒为 ±2σ）', axis_names{axis_index}));
    if axis_index == 1
        legend(estimate_axis, 'Location', 'best');
    end
    if axis_index == 3
        xlabel(estimate_axis, '声学量测帧');
    end

    consistency_axis = nexttile(layout, 2 * axis_index);
    hold(consistency_axis, 'on');
    grid(consistency_axis, 'on');
    box(consistency_axis, 'on');
    plot(consistency_axis, event_id, post_error_deg(:, axis_index), ...
        'r.-', 'LineWidth', 1.15, 'MarkerSize', 11, ...
        'DisplayName', '量测后估计误差');
    plot(consistency_axis, event_id, sigma_post_deg(:, axis_index), ...
        '--', 'Color', [0.45, 0.45, 0.45], 'LineWidth', 1.0, ...
        'DisplayName', '+1σ');
    plot(consistency_axis, event_id, -sigma_post_deg(:, axis_index), ...
        '--', 'Color', [0.45, 0.45, 0.45], 'LineWidth', 1.0, ...
        'HandleVisibility', 'off');
    plot(consistency_axis, event_id, 2 * sigma_post_deg(:, axis_index), ...
        'k--', 'LineWidth', 1.0, 'DisplayName', '+2σ');
    plot(consistency_axis, event_id, -2 * sigma_post_deg(:, axis_index), ...
        'k--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    plot(consistency_axis, event_id, ...
        true_after_feedback_deg(:, axis_index), 'm.-', ...
        'LineWidth', 1.0, 'MarkerSize', 9, ...
        'DisplayName', '反馈后真实剩余误差');
    yline(consistency_axis, 0, 'k:', 'HandleVisibility', 'off');
    ylabel(consistency_axis, sprintf('%s / deg', axis_names{axis_index}));
    title(consistency_axis, sprintf( ...
        '%s：后验误差，2σ覆盖率 %.1f%%', ...
        axis_names{axis_index}, two_sigma_coverage_pct(axis_index)));
    if axis_index == 1
        legend(consistency_axis, 'Location', 'best');
    end
    if axis_index == 3
        xlabel(consistency_axis, '声学量测帧');
    end
end
sgtitle(layout, sprintf( ...
    '%s / %s / Phase STD = 5° / %s 姿态反馈逐帧诊断', ...
    char(data_source), case_name, char(feedback_mode)));

figure_path = fullfile(result_dir, ...
    'range-phase-azimuth-attitude-state-diagnostic.png');
figure_matlab_path = fullfile(result_dir, ...
    'range-phase-azimuth-attitude-state-diagnostic.fig');
exportgraphics(figure_handle, figure_path, 'Resolution', 300);
savefig(figure_handle, figure_matlab_path);

summary_path = fullfile(result_dir, ...
    'range-phase-azimuth-attitude-state-summary.txt');
summary_fp = fopen(summary_path, 'wt');
if summary_fp < 0
    error('无法创建姿态诊断汇总文件：%s', summary_path);
end
fprintf(summary_fp, ['axis prior_estimation_rmse_deg ' ...
    'posterior_estimation_rmse_deg after_feedback_true_rmse_deg ' ...
    'one_sigma_coverage_pct two_sigma_coverage_pct\n']);
for axis_index = 1:3
    fprintf(summary_fp, '%s %.10f %.10f %.10f %.3f %.3f\n', ...
        axis_names{axis_index}, prior_rmse_deg(axis_index), ...
        post_rmse_deg(axis_index), after_feedback_rmse_deg(axis_index), ...
        one_sigma_coverage_pct(axis_index), ...
        two_sigma_coverage_pct(axis_index));
end
fclose(summary_fp);

fprintf('\nPhase STD = 5°，反馈模式：%s\n', feedback_mode);
fprintf(['轴   先验估计RMSE/°  后验估计RMSE/°  ' ...
    '反馈后真实RMSE/°  2σ覆盖率/%%\n']);
for axis_index = 1:3
    fprintf('%-4s %14.6f %15.6f %17.6f %12.2f\n', ...
        axis_names{axis_index}, prior_rmse_deg(axis_index), ...
        post_rmse_deg(axis_index), after_feedback_rmse_deg(axis_index), ...
        two_sigma_coverage_pct(axis_index));
end
fprintf('逐帧图：%s\n', figure_path);
fprintf('汇总表：%s\n', summary_path);

