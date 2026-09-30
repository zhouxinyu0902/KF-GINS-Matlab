clear;
close all;
clc;
%% ========================================================================
% 静态纯惯导推算
%
% 功能：
%   1. 读取对准结束后的 IMU_120.txt；
%   2. 使用 pva_initial.txt 中的初始 PVA 初始化惯导；
%   3. 全程仅进行 INS 机械编排，不进行任何外部量测更新；
%   4. 高度始终固定为初始高度；
%   5. 保存 PureIns.nav；
%   6. 使用 ref_pva.txt 评价纯惯导水平位置误差。
%
% 注意：
%   该程序虽然称为“纯惯导”，但垂向位置进行了固定高度约束，
%   主要用于评价静态条件下水平位置、速度和姿态的自由漂移。
% ========================================================================
%% 1. 参数设置
caseNo = 1;                 % 数据集编号
durationSeconds = [];       % 推算时长，单位 s；[] 表示使用全部 IMU 数据
showProgress = true;        % 是否显示运行进度
outputFile = '';            % 留空则使用配置文件中的默认输出路径
%% 2. 路径及配置初始化
paths = setup_static_experiment(caseNo);
cfg = create_static_pureins_config(paths);
param = Param();
%% 3. 读取 IMU 数据
imu = readmatrix(cfg.imufilepath, 'FileType', 'text');
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
%% 4. 确定纯惯导推算时间范围
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
%% 5. 初始化惯导状态
[~, navstate] = myInitialize_15state(cfg);
% 时间与 IMU 起始时刻严格一致
navstate.time = imu(1, 1);
% 固定高度
navstate.pos(3) = cfg.fixedheight;
%% 6. 初始化结果存储
sample_count = size(imu, 1);
navigation = zeros(sample_count, 11);
% 输出格式：
% [0, time, lat, lon, height, vn, ve, vd, roll, pitch, heading]
navigation(1, :) = [ ...
    0, ...
    navstate.time, ...
    navstate.pos(1) * param.R2D, ...
    navstate.pos(2) * param.R2D, ...
    navstate.pos(3), ...
    navstate.vel(:).', ...
    navstate.att(:).' * param.R2D];
last_progress = 0;
%% 7. 打印初始信息
fprintf('\n============================================================\n');
fprintf('静态纯惯导推算\n');
fprintf('数据集：%s\n', cfg.datasetname);
fprintf('IMU 时间：%.3f ~ %.3f s，共 %d 个历元\n', ...
    imu(1, 1), imu(end, 1), sample_count);
fprintf('初始位置 [deg deg m]：%.8f %.8f %.4f\n', ...
    cfg.initpos(1) * param.R2D, ...
    cfg.initpos(2) * param.R2D, ...
    cfg.fixedheight);
fprintf('初始速度 [m/s]：%.6f %.6f %.6f\n', ...
    cfg.initvel);
fprintf('初始姿态 [deg]：%.6f %.6f %.6f\n', ...
    cfg.initatt * param.R2D);
fprintf('高度约束：固定为 %.4f m\n', cfg.fixedheight);
fprintf('============================================================\n');
%% 8. 纯惯导机械编排
for imu_index = 2:sample_count
    last_imu = imu(imu_index - 1, :).';
    this_imu = imu(imu_index, :).';
    imu_dt = this_imu(1) - last_imu(1);
    if ~isfinite(imu_dt) || imu_dt <= 0
        error('第 %d 行 IMU 时间间隔异常。', imu_index);
    end
    % INS 机械编排
    navstate = InsMech(navstate, last_imu, this_imu);
    % 静态试验中固定高度，仅评价水平纯惯导漂移
    navstate.pos(3) = cfg.fixedheight;
    % navstate.vel(3) = 0;
    % 保存当前导航结果
    navigation(imu_index, :) = [ ...
        0, ...
        navstate.time, ...
        navstate.pos(1) * param.R2D, ...
        navstate.pos(2) * param.R2D, ...
        navstate.pos(3), ...
        navstate.vel(:).', ...
        navstate.att(:).' * param.R2D];
    % 显示运行进度
    if showProgress
        progress = floor(10 * imu_index / sample_count) / 10;
        if progress >= last_progress + 0.1
            fprintf('纯惯导进度：%d%%\n', round(progress * 100));
            last_progress = progress;
        end
    end
end
%% 9. 保存纯惯导结果
if isempty(outputFile)
    outputFile = cfg.pureinsfilepath;
end
output_directory = fileparts(outputFile);
if ~isempty(output_directory) && ~isfolder(output_directory)
    mkdir(output_directory);
end
writematrix( ...
    navigation, ...
    outputFile, ...
    'FileType', 'text', ...
    'Delimiter', ' ');
%% 10. 保存运行信息
summary_file = fullfile(output_directory, 'PureIns-summary.mat');
result = struct();
result.dataset_name = cfg.datasetname;
result.output_file = outputFile;
result.summary_file = summary_file;
result.sample_count = sample_count;
result.start_time = navigation(1, 2);
result.end_time = navigation(end, 2);
result.duration_s = navigation(end, 2) - navigation(1, 2);
result.fixed_height_m = cfg.fixedheight;
result.initial_pva = navigation(1, :);
result.final_pva = navigation(end, :);
run_config = cfg;
save( ...
    summary_file, ...
    'result', ...
    'run_config', ...
    '-v7');
fprintf('\n纯惯导推算完成：%d 个历元，共 %.3f s。\n', ...
    sample_count, result.duration_s);
fprintf('导航结果：%s\n', outputFile);
%% 11. 结果误差评估
switch caseNo
    case 0
        path1 = ...
            'D:\Github\KF-GINS-Matlab\data\experiment-data\static\input\static-003';
        path2 = ...
            'D:\Github\KF-GINS-Matlab\data\experiment-data\static\output\static-003\navigation-results';
        truthpath = fullfile(path1, 'ref_pva.txt');
        pureinspath = fullfile(path2, 'PureIns.nav');
        [fig, finalExcelData] = ...
            calc_radial_error_gjb(truthpath, pureinspath);
    case 1
        path1 = ...
            'D:\Github\KF-GINS-Matlab\data\experiment-data\static-1\input\static-000';
        path2 = ...
            'D:\Github\KF-GINS-Matlab\data\experiment-data\static-1\output\static-000\navigation-results';
        truthpath = fullfile(path1, 'ref_pva.txt');
        pureinspath = fullfile(path2, 'PureIns.nav');
        filepathown120 = fullfile(path2, 'PureIns-ZeroVelFixedHeight.nav');
        filepath120 = fullfile(path1, 'pva_file.txt');
        [fig, finalExcelData] = ...
            calc_radial_error_gjb( ...
            truthpath, ...
            pureinspath, ...
            filepathown120,...
            filepath120);
        legend('pureINS-FixedHeight','pureINS-ZeroVelFixedHeight','pureINS-ZeroVelFixedHeight-120')
        calc_error_gjb(pureinspath, truthpath, true, 'all');
        calc_error_gjb(filepath120, truthpath, true, 'all');
end
%% 保存图片和表格
save_dir = paths.artifacts;
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

% 1) 保存图片（600 dpi）
fig_path_png = fullfile(save_dir, 'radial_error_compare.png');
exportgraphics(fig, fig_path_png, 'Resolution', 600);

% 如果你还想保留 MATLAB 可编辑图窗，也可以再保存一份 .fig
fig_path_fig = fullfile(save_dir, 'radial_error_compare.fig');
savefig(fig, fig_path_fig);

% 2) 保存表格（数值保留小数点后三位）
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

% 可选：同时保存一份 csv
csv_path = fullfile(save_dir, 'radial_error_statistics.csv');
writecell(table_to_save, csv_path);

fprintf('图片已保存：%s\n', fig_path_png);
fprintf('图窗文件已保存：%s\n', fig_path_fig);
fprintf('表格已保存：%s\n', excel_path);
fprintf('CSV已保存：%s\n', csv_path);
