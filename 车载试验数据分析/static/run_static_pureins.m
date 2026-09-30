clear;
close all;
clc;
% parse_static_120([], true, 2);
% export_static_120_inputs([], 430.010, true, 2);
%% ========================================================================
% 静态纯惯导推算
%
% 同一批数据一次生成两类结果：
%   1. FixedHeight：仅固定高度，垂向速度自由演化；
%   2. ZeroVelFixedHeight：垂向速度置零并固定高度。
%
% caseNo = 0 对应 data/experiment-data/static；
% caseNo = 1...5 对应 data/experiment-data/static-1...static-5。
% 每批数据的 input/output 均不再包含采集编号子目录。
% static 缺少可独立对比的设备 PVA，因此只比较两类 PureIns；
% static-x 在 pva_file.txt 存在时额外加入设备 PVA 对比。
% ========================================================================
%% 1. 参数设置
caseNo = 2;                 % 0: static；1...5: static-1...static-5
% imuRateHz = [];             % []: 保持现有频率；static-2 可设为 100 或 200
imuRateHz = 100; 
alignmentTimeSeconds = 430.010;
durationSeconds = [];       % 推算时长，单位 s；[] 表示使用全部 IMU 数据
showProgress = true;        % 是否显示运行进度
outputFixedHeightFile = ''; % 留空则使用配置文件中的默认输出路径
outputZeroVelFixedHeightFile = '';

%% 2. 路径及配置初始化
paths = setup_static_experiment(caseNo);
cfg = create_static_pureins_config(paths);
param = Param();

if isempty(outputFixedHeightFile)
    outputFixedHeightFile = cfg.pureins_fixed_height_filepath;
end
if isempty(outputZeroVelFixedHeightFile)
    outputZeroVelFixedHeightFile = ...
        cfg.pureins_zero_vel_fixed_height_filepath;
end

%% 3. 读取并裁剪 IMU 数据
[imu, nominal_imu_dt_s, nominal_imu_rate_hz, specific_force_mps2] = ...
    load_static_imu(cfg.imufilepath);
if ~isempty(imuRateHz) && abs(nominal_imu_rate_hz - imuRateHz) > 1e-3
    fprintf('现有 IMU 为 %.3f Hz，正在重新导出为 %.3f Hz。\n', ...
        nominal_imu_rate_hz, imuRateHz);
    clear imu;
    export_static_120_inputs([], alignmentTimeSeconds, true, ...
        caseNo, imuRateHz);
    paths = setup_static_experiment(caseNo);
    cfg = create_static_pureins_config(paths);
    [imu, nominal_imu_dt_s, nominal_imu_rate_hz, ...
        specific_force_mps2] = load_static_imu(cfg.imufilepath);
end
cfg.nominal_imu_dt_s = nominal_imu_dt_s;
cfg.nominal_imu_rate_hz = nominal_imu_rate_hz;
cfg.median_specific_force_mps2 = specific_force_mps2;

start_time = max(cfg.starttime, imu(1, 1));
end_time = imu(end, 1);
if ~isempty(durationSeconds)
    end_time = min(end_time, start_time + durationSeconds);
end
imu = imu(imu(:, 1) >= start_time & imu(:, 1) <= end_time, :);
if size(imu, 1) < 2
    error('指定时间范围内有效 IMU 数据少于两行。');
end
cfg.starttime = imu(1, 1);
cfg.endtime = imu(end, 1);

%% 4. 分别运行两种约束方式
fprintf('\n============================================================\n');
fprintf('静态纯惯导推算：%s\n', cfg.datasetname);
fprintf('IMU 时间：%.3f ~ %.3f s，共 %d 个历元\n', ...
    imu(1, 1), imu(end, 1), size(imu, 1));
fprintf('IMU 采样率：%.3f Hz，等效比力中值：%.6f m/s^2\n', ...
    nominal_imu_rate_hz, specific_force_mps2);
fprintf('固定高度：%.4f m\n', cfg.fixedheight);
fprintf('============================================================\n');

navigationFixedHeight = run_pureins_mode( ...
    cfg, imu, false, param, showProgress, 'FixedHeight');
fixedHeightResult = save_pureins_result( ...
    navigationFixedHeight, outputFixedHeightFile, cfg, ...
    'FixedHeight', false);

navigationZeroVelFixedHeight = run_pureins_mode( ...
    cfg, imu, true, param, showProgress, 'ZeroVelFixedHeight');
zeroVelFixedHeightResult = save_pureins_result( ...
    navigationZeroVelFixedHeight, outputZeroVelFixedHeightFile, cfg, ...
    'ZeroVelFixedHeight', true);

fprintf('\n两类纯惯导推算完成。\n');
fprintf('仅固定高度：%s\n', fixedHeightResult.output_file);
fprintf('零垂向速度+固定高度：%s\n', zeroVelFixedHeightResult.output_file);

%% 5. 结果误差评估
if ~isfile(paths.ref_pva_file)
    warning('run_static_pureins:MissingReference', ...
        'Reference PVA is missing; error comparison was skipped: %s', ...
        paths.ref_pva_file);
    return;
end

if paths.compare_device_pva
    [fig, finalExcelData] = calc_radial_error_gjb( ...
        paths.ref_pva_file, ...
        outputFixedHeightFile, ...
        outputZeroVelFixedHeightFile, ...
        paths.device_pva_file);
    legend('PureIns-FixedHeight', ...
        'PureIns-ZeroVelFixedHeight', ...
        'DevicePVA-ZeroVelFixedHeight');
else
    [fig, finalExcelData] = calc_radial_error_gjb( ...
        paths.ref_pva_file, ...
        outputFixedHeightFile, ...
        outputZeroVelFixedHeightFile);
    legend('PureIns-FixedHeight', 'PureIns-ZeroVelFixedHeight');
    if caseNo == 0
        fprintf('static 无独立设备 PVA，已跳过设备 PVA 对比。\n');
    else
        warning('run_static_pureins:MissingDevicePVA', ...
            'pva_file.txt 不存在，已跳过设备 PVA 对比：%s', ...
            paths.device_pva_file);
    end
end

calc_error_gjb(outputFixedHeightFile, paths.ref_pva_file, true, 'all');
calc_error_gjb(outputZeroVelFixedHeightFile, paths.ref_pva_file, true, 'all');
if paths.compare_device_pva
    calc_error_gjb(paths.device_pva_file, paths.ref_pva_file, true, 'all');
end

%% 6. 保存图片和表格
save_dir = paths.artifacts;
if ~isfolder(save_dir)
    mkdir(save_dir);
end

fig_path_png = fullfile(save_dir, 'radial_error_compare.png');
exportgraphics(fig, fig_path_png, 'Resolution', 600);
fig_path_fig = fullfile(save_dir, 'radial_error_compare.fig');
savefig(fig, fig_path_fig);

header = finalExcelData(1, :);
body = finalExcelData(2:end, :);
formattedBody = body;
for i = 1:size(body, 1)
    for j = 1:size(body, 2)
        value = body{i, j};
        if isnumeric(value) && isscalar(value)
            formattedBody{i, j} = sprintf('%.3f', value);
        end
    end
end
table_to_save = [header; formattedBody];

excel_path = fullfile(save_dir, 'radial_error_statistics.xlsx');
writecell(table_to_save, excel_path);
csv_path = fullfile(save_dir, 'radial_error_statistics.csv');
writecell(table_to_save, csv_path);

fprintf('图片已保存：%s\n', fig_path_png);
fprintf('图窗文件已保存：%s\n', fig_path_fig);
fprintf('表格已保存：%s\n', excel_path);
fprintf('CSV已保存：%s\n', csv_path);

function [imu, nominal_dt_s, nominal_rate_hz, specific_force_mps2] = ...
        load_static_imu(filename)
%LOAD_STATIC_IMU Read the common IMU file and verify its increment scale.

imu = readmatrix(filename, 'FileType', 'text');
if isempty(imu) || size(imu, 2) < 7
    error('IMU_120.txt 至少应包含两行、七列数据。');
end
imu = imu(:, 1:7);
if size(imu, 1) < 2
    error('IMU 数据少于两行，无法进行惯导推算。');
end
if any(~isfinite(imu), 'all')
    error('IMU 数据中存在 NaN 或 Inf。');
end
if any(diff(imu(:, 1)) <= 0)
    error('IMU 时间轴存在重复或倒序。');
end

nominal_dt_s = median(diff(imu(:, 1)));
nominal_rate_hz = 1 / nominal_dt_s;
specific_force_mps2 = median(vecnorm(imu(:, 5:7), 2, 2)) / ...
    nominal_dt_s;
if specific_force_mps2 < 5 || specific_force_mps2 > 15
    error('run_static_pureins:InvalidIMUIncrementScale', ...
        ['IMU 速度增量与时间间隔不匹配：等效比力为 %.3f m/s^2。' ...
         '请重新运行 export_static_120_inputs，确认按真实采样周期积分。'], ...
        specific_force_mps2);
end
end

function navigation = run_pureins_mode( ...
        cfg, imu, zeroVerticalVelocity, param, showProgress, modeName)
%RUN_PUREINS_MODE Run one static pure-INS constraint mode.

[~, navstate] = myInitialize_15state(cfg);
navstate.time = imu(1, 1);
navstate.pos(3) = cfg.fixedheight;
if zeroVerticalVelocity
    navstate.vel(3) = 0;
end

sample_count = size(imu, 1);
navigation = zeros(sample_count, 11);
navigation(1, :) = navigation_row(navstate, param);
last_progress = 0;

fprintf('\n开始 %s 推算。\n', modeName);
for imu_index = 2:sample_count
    last_imu = imu(imu_index - 1, :).';
    this_imu = imu(imu_index, :).';
    imu_dt = this_imu(1) - last_imu(1);
    if ~isfinite(imu_dt) || imu_dt <= 0
        error('第 %d 行 IMU 时间间隔异常。', imu_index);
    end

    navstate = InsMech(navstate, last_imu, this_imu);
    navstate.pos(3) = cfg.fixedheight;
    if zeroVerticalVelocity
        navstate.vel(3) = 0;
    end
    navigation(imu_index, :) = navigation_row(navstate, param);

    if showProgress
        progress = floor(10 * imu_index / sample_count) / 10;
        if progress >= last_progress + 0.1
            fprintf('%s 进度：%d%%\n', modeName, round(progress * 100));
            last_progress = progress;
        end
    end
end
end

function row = navigation_row(navstate, param)
%NAVIGATION_ROW Convert the internal state to the common 11-column format.

row = [0, navstate.time, ...
    navstate.pos(1) * param.R2D, ...
    navstate.pos(2) * param.R2D, ...
    navstate.pos(3), ...
    navstate.vel(:).', ...
    navstate.att(:).' * param.R2D];
end

function result = save_pureins_result( ...
        navigation, outputFile, cfg, modeName, zeroVerticalVelocity)
%SAVE_PUREINS_RESULT Save navigation output and a mode-specific summary.

output_directory = fileparts(outputFile);
if ~isempty(output_directory) && ~isfolder(output_directory)
    mkdir(output_directory);
end
writematrix(navigation, outputFile, 'FileType', 'text', 'Delimiter', ' ');

summary_file = fullfile(output_directory, ...
    sprintf('PureIns-summary-%s.mat', modeName));
result = struct();
result.dataset_name = cfg.datasetname;
result.mode = modeName;
result.zero_vertical_velocity = zeroVerticalVelocity;
result.fixed_height = true;
result.output_file = outputFile;
result.summary_file = summary_file;
result.sample_count = size(navigation, 1);
result.start_time = navigation(1, 2);
result.end_time = navigation(end, 2);
result.duration_s = navigation(end, 2) - navigation(1, 2);
result.fixed_height_m = cfg.fixedheight;
result.initial_pva = navigation(1, :);
result.final_pva = navigation(end, :);
run_config = cfg;
save(summary_file, 'result', 'run_config', '-v7');
end
