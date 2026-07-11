%% KF RANGE/INS 组合导航
% 高度量测：按 1 s 周期更新一次垂向。
% 距离量测：约 360 s 一次，只更新水平距离。

clear; clc; close all;

%% 1. 路径与参数
this_dir = fileparts(mfilename('fullpath'));
project_root = fullfile(this_dir, '..', '..');
input_dir = fullfile(this_dir, 'input');
output_dir = fullfile(this_dir, 'output');

addpath(genpath(project_root));
addpath(input_dir);

param = Param();
cfg = config_simu(input_dir);
cfg.outputfolder = output_dir;
if ~exist(cfg.outputfolder, 'dir')
    mkdir(cfg.outputfolder);
end

rngstd = 6;        % 水平距离噪声，m
heightstd = 0.4;   % 高度噪声，m
height_update_interval = 1.0;  % 高度量测更新周期，s。原程序为每个 IMU 历元更新。
rng(1);
tic;

smoothWay = 'RTS'; % 平滑方式，选择RTS或线性
SmoothIsOpen = 1;
%% 2. 导入 IMU、高度和 360 s 距离量测
imudata = importdata(cfg.imufilepath);
truth = importdata(cfg.truthpath);
heightdata = truth(:, [2, 5]);

range_streams = { ...
    importdata(cfg.rangefile1path), ...
    importdata(cfg.rangefile2path), ...
    importdata(cfg.rangefile3path)};
rangedata = buildRange360(range_streams);

% 不要把起始时刻推到第一条距离量测。距离是稀疏量测，如果从 359 s
% 开始但仍使用 config_simu 中 0.01 s 的初始 PVA，会造成明显初值错位。
cfg.starttime = max([cfg.starttime, imudata(1, 1), heightdata(1, 1)]);
cfg.endtime = min([cfg.endtime, imudata(end, 1), heightdata(end, 1), rangedata(end, 1)]);

imudata = imudata(imudata(:, 1) >= cfg.starttime & imudata(:, 1) <= cfg.endtime, :);
heightdata = heightdata(heightdata(:, 1) >= cfg.starttime & heightdata(:, 1) <= cfg.endtime, :);
rangedata = rangedata(rangedata(:, 1) >= cfg.starttime & rangedata(:, 1) <= cfg.endtime, :);

heightdata(:, 2) = heightdata(:, 2) + heightstd * randn(size(heightdata, 1), 1);
rangedata(:, 3) = rangedata(:, 3) + rngstd * randn(size(rangedata, 1), 1);

fprintf('\n========== KF RANGE/INS Processing ==========\n');
fprintf('处理时间: %.2f -> %.2f s\n', cfg.starttime, cfg.endtime);
fprintf('IMU/高度点数: %d, 距离点数: %d\n', size(imudata, 1), size(rangedata, 1));
fprintf('高度更新周期: %.2f s\n', height_update_interval);

%% 3. 初始化和输出
[kf, navstate] = myInitialize_15state(cfg);
kf.rangstd = rngstd;
kf.depthstd = heightstd;

navfp1 = -1;
navfp2 = -1;
navpath = fullfile(cfg.outputfolder, 'KF-RANGE-INS.nav');
navfp = fopen(navpath, 'wt');
if navfp < 0
    error('无法打开输出文件: %s', navpath);
end


% 根据设置是否启用平滑
if SmoothIsOpen == 1
    % 二次平滑结果
    navpath1 = fullfile(cfg.outputfolder, sprintf('KF-RANGE-INS-Single-stage %s.nav', smoothWay));
    navfp1 = fopen(navpath1, 'wt');
    % 单次平滑结果
    navpath2 = fullfile(cfg.outputfolder, sprintf('KF-RANGE-INS-Proposed two-stage %s.nav', smoothWay));
    navfp2 = fopen(navpath2, 'wt');
end
MAX_BUFFER_SIZE = 54000;

state_buffer = zeros(MAX_BUFFER_SIZE, 10);
Xk_k1propa   = zeros(MAX_BUFFER_SIZE, 15);
Pk_k1propa   = zeros(MAX_BUFFER_SIZE, 225);
Pk_propa = zeros(MAX_BUFFER_SIZE, 225);
PHI   = zeros(MAX_BUFFER_SIZE, 225); % 建议变量名区分开


prev_state_buffer = [];
prev_Pk_propa = [];
prev_Pk_k1propa = [];
prev_PHI = [];
prev_rangeindex = 0;

buf_idx = 1;

bridge_err = zeros(length(rangedata),15);

lastimu = imudata(1, :)';
thisimu = imudata(1, :)';
rangeindex = 1;
range_update_count = 0;
height_update_count = 0;
next_height_time = cfg.starttime + height_update_interval;
lastpercent = 0;


%% 4. 主循环：1 s 高度更新 + 360 s 距离更新
for imuindex = 2:size(imudata, 1)
    lastimu = thisimu;
    thisimu = imudata(imuindex, :)';
    imudt = thisimu(1) - lastimu(1);
    if imudt <= 0
        error('IMU 时间戳必须严格递增。');
    end

    navstate = InsMech(navstate, lastimu, thisimu);

    % 高度改为 1 s 周期更新：到达高度更新时刻时，只做垂向量测更新并立即反馈垂向。
    while next_height_time <= thisimu(1) + 5e-4
        heightrow = getNearestHeightRow(heightdata, next_height_time);
        kf = heightOnlyUpdate(navstate, heightrow, kf);
        navstate.pos(3) = navstate.pos(3) - kf.x(3);
        navstate.vel(3) = navstate.vel(3) - kf.x(6);
        kf.x(3) = 0;
        kf.x(6) = 0;
        height_update_count = height_update_count + 1;
        next_height_time = next_height_time + height_update_interval;
    end

    % 距离是 360 s 周期：到达距离时刻时只做水平距离更新。
    while rangeindex <= size(rangedata, 1) && rangedata(rangeindex, 1) <= thisimu(1) + 5e-4
        kf = rangeOnlyUpdate(navstate, rangedata(rangeindex, :), kf);


        if SmoothIsOpen == 1
            if buf_idx > 1
                valid_len = buf_idx - 1;
                sub_state_buffer = state_buffer(1:valid_len, :);

                xk_final = kf.x;
                sub_Pk_k1propa = Pk_k1propa(1:valid_len, :);
                sub_Pk_propa = Pk_propa(1:valid_len, :);
                sub_PHI = PHI(1:valid_len, :);

                % 调用 RTS 平滑函数
                [nav_matrix, bridge_error, rtsstate_buffer] = perform_unified_smoothing(sub_state_buffer, xk_final, ...
                    param, rangeindex, smoothWay, 'rad', sub_Pk_propa, sub_Pk_k1propa, sub_PHI);
                fprintf(navfp1, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix);
                bridge_err(rangeindex,:) = bridge_error;
                % 2. 判断是否有【上一个 7 分钟】的数据被缓存
                if isempty(prev_state_buffer)
                    % 如果是第一段数据 (比如 0-7min)，没法进行二次平滑，直接暂存起来
                    prev_state_buffer = rtsstate_buffer;
                    prev_Pk_propa     = sub_Pk_propa;
                    prev_Pk_k1propa   = sub_Pk_k1propa;
                    prev_PHI          = sub_PHI;
                    prev_rangeindex   = rangeindex;
                else
                    [nav_matrix_prev_resmoothed, bridge_error, smoothed_state_buffer] = perform_unified_smoothing(prev_state_buffer, bridge_error, ...
                        param, prev_rangeindex, smoothWay, 'rad', prev_Pk_propa, prev_Pk_k1propa, prev_PHI);
                    fprintf(navfp2, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix_prev_resmoothed);
                    % 更新缓存：把当前的 7-14min 变成新的“上一段”，等待 14-21min 来救赎它
                    prev_state_buffer = rtsstate_buffer;
                    prev_Pk_propa     = sub_Pk_propa;
                    prev_Pk_k1propa   = sub_Pk_k1propa;
                    prev_PHI          = sub_PHI;
                    prev_rangeindex   = rangeindex;
                end
                buf_idx = 1;
                state_buffer = zeros(MAX_BUFFER_SIZE, 10);
                Xk_k1propa = zeros(MAX_BUFFER_SIZE, 15);
                Pk_k1propa = zeros(MAX_BUFFER_SIZE, 225);
                Pk_propa = zeros(MAX_BUFFER_SIZE, 225);
                PHI = zeros(MAX_BUFFER_SIZE, 225);
            end
        end
        [kf, navstate] = myErrorFeedback_range(kf, navstate);
        rangeindex = rangeindex + 1;
        range_update_count = range_update_count + 1;
    end

    if SmoothIsOpen == 1
        Pk_propa(buf_idx,:) = kf.P(:)';
    end
    kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);


    if SmoothIsOpen == 1
        nav = [navstate.time;navstate.pos;navstate.vel;navstate.att];
        state_buffer(buf_idx,:) =  nav';
        Xk_k1propa(buf_idx,:) = kf.x(:)';
        Pk_k1propa(buf_idx,:) = kf.P(:)';
        PHI(buf_idx,:) = kf.phi(:)';
        buf_idx = buf_idx + 1;
    end
    nav = [imuindex - 1; navstate.time; ...
        navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3); ...
        navstate.vel; navstate.att * param.R2D];
    fprintf(navfp, ['%6d %12.6f %14.9f %14.9f %10.4f ', ...
        '%10.5f %10.5f %10.5f %10.6f %10.6f %10.6f\n'], nav);

    if imuindex / size(imudata, 1) - lastpercent > 0.20
        fprintf('processing %d %%\n', floor(imuindex * 100 / size(imudata, 1)));
        lastpercent = imuindex / size(imudata, 1);
    end
end
% for 循环结束，写入最后一段缓存的轨迹
if ~isempty(prev_state_buffer)
    % 最后一段没有未来信息了，我们只能用它自己第一次平滑的结果
    [nav_matrix_tail, ~, ~] = perform_unified_smoothing(prev_state_buffer, zeros(15,1), ...
        param, prev_rangeindex, 'Linear', 'rad', [], [], []);
    fprintf(navfp2, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix_tail);
end
fclose(navfp);
if navfp1 > 0
    fclose(navfp1);
end
if navfp2 > 0
    fclose(navfp2);
end
fprintf('高度更新次数: %d\n', height_update_count);
fprintf('距离更新次数: %d\n', range_update_count);
fprintf('结果文件: %s\n', navpath);
toc;

%% 局部函数
function rangedata = buildRange360(range_streams)
sample_step = 360;
for i = 1:numel(range_streams)
    range_streams{i} = range_streams{i}(sample_step:sample_step:end, :);
end
n = min(cellfun(@(x) size(x, 1), range_streams));
rangedata = zeros(n, size(range_streams{1}, 2));
seq = [1, 2, 3];
for i = 1:3
    rows = i:3:n;
    rangedata(rows, :) = range_streams{seq(i)}(rows, :);
end
rangedata = rangedata(any(rangedata, 2), :);
end

function heightrow = getNearestHeightRow(heightdata, query_time)
[~, idx] = min(abs(heightdata(:, 1) - query_time));
heightrow = heightdata(idx, :);
end

function kf = heightOnlyUpdate(navstate, heightrow, kf)
z = navstate.pos(3) - heightrow(2);
H = zeros(1, kf.RANK);
H(3) = 1;
R = kf.depthstd ^ 2;
K = kf.P * H' / (H * kf.P * H' + R);

% 高度量测只反馈到天向位置和天向速度，避免高度量测牵动水平状态。
mask = zeros(kf.RANK, 1);
mask(3) = 1;
mask(6) = 1;
K = K .* mask;

kf.x = kf.x + K * (z - H * kf.x);
I = eye(kf.RANK);
kf.P = (I - K * H) * kf.P * (I - K * H)' + K * R * K';
end

function kf = rangeOnlyUpdate(navstate, rangerow, kf)
param = Param();
beacon = rangerow(4:6)';
[rm, rn] = getRmRn(beacon(1), param);
DR = diag([rm + beacon(3), (rn + beacon(3)) * cos(beacon(1)), -1]);
delta = DR * (navstate.pos - beacon);
predicted = max(norm(delta(1:2)), 1e-6);

z = predicted - rangerow(3);
H = zeros(1, kf.RANK);
derivative = (navstate.pos' - beacon') * (DR ^ 2) / predicted;
H(1, 1:2) = derivative(1:2);
R = kf.rangstd ^ 2;

K = kf.P * H' / (H * kf.P * H' + R);
kf.x = kf.x + K * (z - H * kf.x);
I = eye(kf.RANK);
kf.P = (I - K * H) * kf.P * (I - K * H)' + K * R * K';
end
