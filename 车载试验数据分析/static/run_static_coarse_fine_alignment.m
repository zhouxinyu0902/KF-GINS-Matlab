clear;
close all;
clc;

%% ========================================================================
% 静止基座粗对准 + Kalman 精对准 + 纯惯导
%
% 数据使用方式：
%   10~70 s   ：利用重力和地球自转角速度两个矢量进行数学粗对准；
%   70~430 s  ：利用“位置不变、速度为零”进行15状态 Kalman 精对准；
%   430 s以后 ：分别使用pva_initial姿态和精对准姿态进行纯惯导推算。
%
% 导航流程与 FGO_gnss_ins.m 保持一致：
%   IMU误差补偿 -> InsMech机械编排 -> 15状态传播 -> 量测更新 -> 闭环反馈。
%
% 输出格式与仓库现有 .nav 文件一致：
%   [0, time, latitude_deg, longitude_deg, height_m,
%    velocity_N, velocity_E, velocity_D, roll_deg, pitch_deg, heading_deg]
% ========================================================================

%% 1. 用户参数
caseNo = 5;                 % 0: static；1~5: static-1 ~ static-5
imuRateHz = 100;            % []: 保持现有频率；也可设置为100或200

% 批量运行时可用环境变量临时覆盖上述参数，无需反复改脚本：
%   KF_STATIC_CASE_NO = 0~5
%   KF_STATIC_IMU_RATE_HZ = 100或200
caseNoOverride = getenv('KF_STATIC_CASE_NO');
if ~isempty(caseNoOverride)
    caseNo = str2double(caseNoOverride);
end
imuRateOverride = getenv('KF_STATIC_IMU_RATE_HZ');
if ~isempty(imuRateOverride)
    imuRateHz = str2double(imuRateOverride);
end
validateattributes(caseNo, {'numeric'}, ...
    {'scalar', 'integer', '>=', 0, '<=', 5}, mfilename, 'caseNo');
validateattributes(imuRateHz, {'numeric'}, ...
    {'scalar', 'positive'}, mfilename, 'imuRateHz');

% 已知装订位置，输入顺序为经度、纬度、高度。
bindingLongitudeDeg = 120.68946110621751;
bindingLatitudeDeg = 36.36533670805072;
bindingHeightM = 5;

coarseStartSeconds = 10;    % 前10 s留给设备启动
coarseEndSeconds = 70;      % 数学粗对准结束时刻
alignmentEndSeconds = 430;  % Kalman精对准结束时刻

% 静止位置和零速度量测，每秒更新一次。
alignmentUpdateHz = 1;
positionMeasurementStdM = [0.02; 0.02; 0.05];
zeroVelocityStdMps = [0.01; 0.01; 0.01];
finalAttitudeAveragingSeconds = 60; % 对末段稳态姿态做圆周平均

% 精对准IMU随机模型。小于1表示按更高精度建模；建议只做小幅调整。
% 当前取0.8，即在原配置基础上将ARW、VRW和零偏随机模型下调20%。
alignmentImuNoiseScale = 0.8;

% 静止单姿态下，三轴陀螺零偏和水平加计零偏与姿态存在强耦合。
% Z轴加计零偏可单独放开做实验，但本数据上会轻微恶化后续水平纯惯导，
% 因此默认仍冻结全部零偏，保证初始姿态估计更稳健。
estimateVerticalAccBias = false;
verticalAccBiasInitialStdMgal = 30;

showProgress = true;
zeroVerticalVelocity = true; % true: 零垂速+固定高度；false: 仅固定高度
outputPvaInitialFile = '';   % 使用原pva_initial姿态的纯惯导
outputAlignedFile = '';      % 使用新对准姿态的纯惯导
referenceFile = '';          % 新静止参考PVA，不覆盖旧ref_pva.txt

%% 2. 路径和数据
paths = setup_static_experiment(caseNo);

% setup_static_experiment 只添加一级目录；这里递归加入函数目录，兼容
% GINS-KF 已移动到 function/GINS-KF 的当前仓库结构。
addpath(genpath(fullfile(paths.project_root, 'function')));
addpath(genpath(fullfile(paths.project_root, 'function_zxy')));

param = Param();
alignmentFile = paths.imu_alignment_file;
navigationFile = paths.imu_file;

if ~isfile(alignmentFile)
    error('缺少对准 IMU 文件：%s', alignmentFile);
end
if ~isfile(navigationFile)
    error('缺少导航 IMU 文件：%s', navigationFile);
end

alignmentImu = readmatrix(alignmentFile, 'FileType', 'text');
navigationImu = readmatrix(navigationFile, 'FileType', 'text');
alignmentImu = alignmentImu(:, 1:7);
navigationImu = navigationImu(:, 1:7);

if size(alignmentImu, 1) < 2 || size(navigationImu, 1) < 2
    error('对准或导航 IMU 数据少于两行。');
end
if any(~isfinite(alignmentImu), 'all') || any(~isfinite(navigationImu), 'all')
    error('IMU 数据中存在 NaN 或 Inf。');
end

nominalDt = median([diff(alignmentImu(:, 1)); diff(navigationImu(:, 1))]);
nominalRateHz = 1 / nominalDt;

% 若指定频率与现有输入不同，则沿用已有导出流程重新生成输入。
if ~isempty(imuRateHz) && abs(nominalRateHz - imuRateHz) > 1e-3
    fprintf('现有 IMU 为 %.3f Hz，重新导出为 %.3f Hz。\n', ...
        nominalRateHz, imuRateHz);
    export_static_120_inputs([], alignmentEndSeconds, true, caseNo, imuRateHz);
    alignmentImu = readmatrix(alignmentFile, 'FileType', 'text');
    navigationImu = readmatrix(navigationFile, 'FileType', 'text');
    alignmentImu = alignmentImu(:, 1:7);
    navigationImu = navigationImu(:, 1:7);
    nominalDt = median([diff(alignmentImu(:, 1)); diff(navigationImu(:, 1))]);
    nominalRateHz = 1 / nominalDt;
end

if zeroVerticalVelocity
    constraintName = 'ZeroVelFixedHeight';
else
    constraintName = 'FixedHeight';
end
if isempty(outputPvaInitialFile)
    outputPvaInitialFile = fullfile(paths.output, ...
        ['PureIns-PvaInitialAtt-' constraintName '.nav']);
end
if isempty(outputAlignedFile)
    outputAlignedFile = fullfile(paths.output, ...
        ['PureIns-AlignedAtt-' constraintName '.nav']);
end
if isempty(referenceFile)
    referenceFile = fullfile(paths.output, 'ReferencePVA-AlignedAtt.nav');
end

fprintf('\n============================================================\n');
fprintf('静止粗对准 + Kalman精对准：%s\n', paths.dataset_name);
fprintf('装订经纬高：%.12f deg, %.12f deg, %.3f m\n', ...
    bindingLongitudeDeg, bindingLatitudeDeg, bindingHeightM);
fprintf('IMU频率：%.3f Hz\n', nominalRateHz);
fprintf('============================================================\n');

%% 3. 数学粗对准：重力矢量 + 地球自转角速度矢量
% 导航系和载体系均采用前-右-下（NED/FRD）。平台静止时：
%   加速度计测得的比力方向为竖直向上，即 f^n = [0,0,-g]；
%   陀螺测得的是地球自转角速度在载体系的投影。
% 两组不共线矢量可直接确定载体到导航系方向余弦矩阵 C_b^n。

alignmentElapsed = alignmentImu(:, 1) - alignmentImu(1, 1);
coarseIndex = alignmentElapsed >= coarseStartSeconds & ...
    alignmentElapsed < coarseEndSeconds;
fineIndex = alignmentElapsed >= coarseEndSeconds & ...
    alignmentElapsed < alignmentEndSeconds;

if nnz(coarseIndex) < round(10 * nominalRateHz)
    error('粗对准区间内数据不足10 s。');
end
if nnz(fineIndex) < round(10 * nominalRateHz)
    error('精对准区间内数据不足10 s。');
end

alignmentDt = [nominalDt; diff(alignmentImu(:, 1))];
gyroRateBody = alignmentImu(:, 2:4) ./ alignmentDt;
specificForceBody = alignmentImu(:, 5:7) ./ alignmentDt;
meanGyroBody = mean(gyroRateBody(coarseIndex, :), 1).';
meanSpecificForceBody = mean(specificForceBody(coarseIndex, :), 1).';

latitudeRad = bindingLatitudeDeg * param.D2R;
bindingPosition = [latitudeRad; ...
    bindingLongitudeDeg * param.D2R; bindingHeightM];

earthRateNavigation = [param.WGS84_WIE * cos(latitudeRad); ...
    0; -param.WGS84_WIE * sin(latitudeRad)];
specificForceNavigation = [0; 0; -1];

% TRIAD双矢量定姿。
navAxis1 = specificForceNavigation / norm(specificForceNavigation);
navAxis2 = cross(navAxis1, earthRateNavigation);
navAxis2 = navAxis2 / norm(navAxis2);
navAxis3 = cross(navAxis1, navAxis2);

bodyAxis1 = meanSpecificForceBody / norm(meanSpecificForceBody);
bodyAxis2 = cross(bodyAxis1, meanGyroBody);
if norm(bodyAxis2) < 1e-8
    error('地球自转矢量不可用，无法完成双矢量粗对准。');
end
bodyAxis2 = bodyAxis2 / norm(bodyAxis2);
bodyAxis3 = cross(bodyAxis1, bodyAxis2);

coarseCbn = [navAxis1, navAxis2, navAxis3] * ...
    [bodyAxis1, bodyAxis2, bodyAxis3].';

% 消除数值误差，确保结果为严格正交旋转矩阵。
[uCbn, ~, vCbn] = svd(coarseCbn);
coarseCbn = uCbn * diag([1, 1, det(uCbn * vCbn.')]) * vCbn.';
coarseAttitude = dcm2euler(coarseCbn);

fprintf('\n数学粗对准结果 [roll pitch heading]：\n');
fprintf('  %.8f  %.8f  %.8f deg\n', coarseAttitude * param.R2D);

%% 4. 建立15状态 Kalman 精对准
% 误差状态与 FGO_gnss_ins.m 相同：
%   [位置误差3，速度误差3，姿态误差3，陀螺零偏3，加计零偏3]。

cfg = create_static_pureins_config(paths);
pvaInitialAttitudeForComparison = cfg.initatt;
fineImu = alignmentImu(fineIndex, :);

validateattributes(alignmentImuNoiseScale, {'numeric'}, ...
    {'scalar', 'positive', '<=', 1}, mfilename, ...
    'alignmentImuNoiseScale');
validateattributes(verticalAccBiasInitialStdMgal, {'numeric'}, ...
    {'scalar', 'positive'}, mfilename, ...
    'verticalAccBiasInitialStdMgal');
validateattributes(finalAttitudeAveragingSeconds, {'numeric'}, ...
    {'scalar', 'positive'}, mfilename, ...
    'finalAttitudeAveragingSeconds');

% create_static_pureins_config中的原始指标为：
%   ARW 0.0005 deg/sqrt(h)，VRW 1e-5 m/s/sqrt(h)，
%   陀螺零偏稳定性0.01 deg/h，加计零偏稳定性7 mGal。
% 这里只调整精对准滤波模型，不改变后续两次纯惯导的共同配置。
cfg.gyrarw = cfg.gyrarw * alignmentImuNoiseScale;
cfg.accvrw = cfg.accvrw * alignmentImuNoiseScale;
cfg.gyrbiasstd = cfg.gyrbiasstd * alignmentImuNoiseScale;
cfg.accbiasstd = cfg.accbiasstd * alignmentImuNoiseScale;

cfg.starttime = fineImu(1, 1);
cfg.initpos = bindingPosition;
cfg.initvel = zeros(3, 1);
cfg.initatt = coarseAttitude;
cfg.initgyrbias = zeros(3, 1);
cfg.initaccbias = zeros(3, 1);
cfg.initgyrscale = zeros(3, 1);
cfg.initaccscale = zeros(3, 1);

% 粗对准后的初始不确定度。位置误差在滤波器内部使用纬经高单位。
[rm0, rn0] = getRmRn(bindingPosition(1), param);
positionScale = diag([rm0 + bindingHeightM, ...
    (rn0 + bindingHeightM) * cos(bindingPosition(1)), -1]);
cfg.initposstd = positionScale \ [0.10; 0.10; 0.10];
cfg.initvelstd = [0.05; 0.05; 0.05];
cfg.initattstd = [0.5; 0.5; 2.0] * param.D2R;
cfg.initgyrbiasstd = 1.0 * ones(3, 1) * param.D2R / 3600;
cfg.initaccbiasstd = verticalAccBiasInitialStdMgal * 1e-5 * ones(3, 1);

[kf, navstate] = myInitialize_15state(cfg);
navstate.time = fineImu(1, 1);

% 单一静止姿态下，水平姿态误差与水平加计零偏、航向误差与陀螺零偏
% 存在不可观耦合。因此先冻结全部零偏，避免零速残差被错误地分配给
% 零偏状态；随后仅放开Z轴加计零偏，它可借助已知重力大小和零速度估计。
kf.P(10:15, :) = 0;
kf.P(:, 10:15) = 0;
kf.Qc(7:12, 7:12) = 0;
if estimateVerticalAccBias
    kf.P(15, 15) = (verticalAccBiasInitialStdMgal * 1e-5)^2;
    kf.Qc(12, 12) = 2 * cfg.accbiasstd^2 / cfg.corrtime;
end

updateIntervalSamples = max(1, round(nominalRateHz / alignmentUpdateHz));
lastImuCorrected = fineImu(1, :).';
lastImuCorrected(2:4) = lastImuCorrected(2:4) - ...
    nominalDt * navstate.gyrbias;
lastImuCorrected(5:7) = lastImuCorrected(5:7) - ...
    nominalDt * navstate.accbias;

lastProgress = 0;
fineAttitudeHistory = zeros(size(fineImu, 1), 3);
fineAttitudeHistory(1, :) = navstate.att(:).';
fprintf('\n开始 Kalman 精对准：%.3f ~ %.3f s\n', ...
    fineImu(1, 1), fineImu(end, 1));

for imuIndex = 2:size(fineImu, 1)
    lastState = navstate;
    thisImuRaw = fineImu(imuIndex, :).';
    imuDt = thisImuRaw(1) - fineImu(imuIndex - 1, 1);
    if ~isfinite(imuDt) || imuDt <= 0
        error('精对准第%d行时间间隔异常。', imuIndex);
    end

    % 用上一轮反馈得到的零偏补偿当前IMU增量。
    thisImuCorrected = thisImuRaw;
    thisImuCorrected(2:4) = (thisImuRaw(2:4) - ...
        imuDt * navstate.gyrbias) ./ (ones(3, 1) + navstate.gyrscale);
    thisImuCorrected(5:7) = (thisImuRaw(5:7) - ...
        imuDt * navstate.accbias) ./ (ones(3, 1) + navstate.accscale);

    % 与 FGO_gnss_ins.m 相同：先机械编排，再传播误差状态。
    navstate = InsMech(lastState, lastImuCorrected, thisImuCorrected);
    kf = myInsPropagate_15state(navstate, thisImuCorrected, imuDt, kf);

    if mod(imuIndex - 1, updateIntervalSamples) == 0 || ...
            imuIndex == size(fineImu, 1)
        % 已知平台位置固定且速度为零。
        innovation = [navstate.pos - bindingPosition; navstate.vel];
        H = zeros(6, kf.RANK);
        H(1:3, 1:3) = eye(3);
        H(4:6, 4:6) = eye(3);

        [rm, rn] = getRmRn(navstate.pos(1), param);
        positionScale = diag([rm + navstate.pos(3), ...
            (rn + navstate.pos(3)) * cos(navstate.pos(1)), -1]);
        positionStdInternal = positionScale \ positionMeasurementStdM;
        R = diag([positionStdInternal .^ 2; zeroVelocityStdMps .^ 2]);

        K = kf.P * H.' / (H * kf.P * H.' + R);
        kf.x = kf.x + K * (innovation - H * kf.x);
        identityMatrix = eye(kf.RANK);
        kf.P = (identityMatrix - K * H) * kf.P * ...
            (identityMatrix - K * H).' + K * R * K.';

        % 闭环反馈位置、速度、姿态以及IMU零偏，然后清零误差状态。
        [kf, navstate] = myErrorFeedback_15state(kf, navstate);
    end

    lastImuCorrected = thisImuCorrected;
    fineAttitudeHistory(imuIndex, :) = navstate.att(:).';

    if showProgress
        progress = floor(10 * imuIndex / size(fineImu, 1)) / 10;
        if progress >= lastProgress + 0.1
            fprintf('精对准进度：%d%%\n', round(progress * 100));
            lastProgress = progress;
        end
    end
end

% 末段已经进入稳态，使用圆周平均抑制直接取最后一个历元的随机波动。
% 对三个欧拉角分别在单位圆上求均值，可正确处理航向跨越0/360度。
averagingIndex = fineImu(:, 1) >= ...
    fineImu(end, 1) - finalAttitudeAveragingSeconds;
alignedAttitude = atan2(mean(sin(fineAttitudeHistory(averagingIndex, :)), 1), ...
    mean(cos(fineAttitudeHistory(averagingIndex, :)), 1)).';

% 将航向统一显示到[0,360)范围，并给出相对pva_initial的差值。
attitudeSource = ["pva_initial"; "数学粗对准"; "Kalman精对准(末60s均值)"];
attitudeDeg = [pvaInitialAttitudeForComparison.'; ...
    coarseAttitude.'; alignedAttitude.'] * param.R2D;
attitudeDeg(:, 3) = mod(attitudeDeg(:, 3), 360);
attitudeDifferenceDeg = attitudeDeg - attitudeDeg(1, :);
attitudeDifferenceDeg(:, 3) = mod(...
    attitudeDifferenceDeg(:, 3) + 180, 360) - 180;
attitudeComparison = table(attitudeSource, ...
    attitudeDeg(:, 1), attitudeDeg(:, 2), attitudeDeg(:, 3), ...
    attitudeDifferenceDeg(:, 1), attitudeDifferenceDeg(:, 2), ...
    attitudeDifferenceDeg(:, 3), ...
    'VariableNames', {'source', 'roll_deg', 'pitch_deg', 'heading_deg', ...
    'delta_roll_vs_pva_deg', 'delta_pitch_vs_pva_deg', ...
    'delta_heading_vs_pva_deg'});

fprintf('\n姿态对准结果对比：\n');
disp(attitudeComparison);
posteriorAttitudeStdDeg = sqrt(diag(kf.P(7:9, 7:9))) * param.R2D;
fprintf('精对准后姿态标准差 [roll pitch heading]：%.8f  %.8f  %.8f deg\n', ...
    posteriorAttitudeStdDeg);
fprintf('最终姿态采用末%.0f s圆周平均。\n', ...
    finalAttitudeAveragingSeconds);
if estimateVerticalAccBias
    fprintf('精对准估计Z轴加计零偏：%.6f mGal\n', ...
        navstate.accbias(3) * 1e5);
    fprintf(['说明：冻结三轴陀螺零偏和水平加计零偏，仅估计可观的' ...
        'Z轴加计零偏。\n']);
else
    fprintf('说明：精对准阶段冻结全部IMU零偏，避免静止单位置不可观耦合。\n');
end

%% 5. 准备两组只相差初始姿态的纯惯导配置
% 第一组保留原pva_initial的姿态；第二组只把姿态替换为新对准结果。
% 两组的位置、速度、IMU零偏和约束方式完全相同，保证比较对象仅为姿态。
pvaInitialCfg = create_static_pureins_config(paths);
pvaInitialAttitude = pvaInitialCfg.initatt;

runConfigs = {pvaInitialCfg, pvaInitialCfg};
runNames = {'PvaInitialAtt', 'AlignedAtt'};
runFiles = {outputPvaInitialFile, outputAlignedFile};
runAttitudes = {pvaInitialAttitude, alignedAttitude};

for runIndex = 1:2
    runConfigs{runIndex}.starttime = navigationImu(1, 1);
    runConfigs{runIndex}.endtime = navigationImu(end, 1);
    runConfigs{runIndex}.initpos = bindingPosition;
    runConfigs{runIndex}.initvel = pvaInitialCfg.initvel;
    runConfigs{runIndex}.initatt = runAttitudes{runIndex};
    runConfigs{runIndex}.fixedheight = bindingHeightM;

    % 不带入精对准滤波估计的零偏，保持与run_static_pureins相同。
    runConfigs{runIndex}.initgyrbias = pvaInitialCfg.initgyrbias;
    runConfigs{runIndex}.initaccbias = pvaInitialCfg.initaccbias;
    runConfigs{runIndex}.initgyrscale = pvaInitialCfg.initgyrscale;
    runConfigs{runIndex}.initaccscale = pvaInitialCfg.initaccscale;
end

fprintf('\n两组纯惯导初始姿态 [roll pitch heading]：\n');
fprintf('  pva_initial：%.8f  %.8f  %.8f deg\n', ...
    pvaInitialAttitude * param.R2D);
fprintf('  新对准结果 ：%.8f  %.8f  %.8f deg\n', ...
    alignedAttitude * param.R2D);
fprintf('  共同初始速度：%.6f  %.6f  %.6f m/s\n', ...
    pvaInitialCfg.initvel);

%% 6. 生成新的静止参考PVA
% 新参考严格采用：装订位置、零速度、新对准姿态，并在整个导航时段保持不变。
sampleCount = size(navigationImu, 1);
referencePva = zeros(sampleCount, 11);
referencePva(:, 2) = navigationImu(:, 1);
referencePva(:, 3) = bindingLatitudeDeg;
referencePva(:, 4) = bindingLongitudeDeg;
referencePva(:, 5) = bindingHeightM;
referencePva(:, 6:8) = 0;
referencePva(:, 9:11) = repmat(...
    alignedAttitude(:).' * param.R2D, sampleCount, 1);

referenceDirectory = fileparts(referenceFile);
if ~isfolder(referenceDirectory)
    mkdir(referenceDirectory);
end
writematrix(referencePva, referenceFile, ...
    'FileType', 'text', 'Delimiter', ' ');

%% 7. 按run_static_pureins流程分别运行两次纯惯导
% 两次推算都直接使用同一份IMU增量，不使用精对准估计零偏补偿。

for runIndex = 1:2
    [~, pureInsState] = myInitialize_15state(runConfigs{runIndex});
    pureInsState.time = navigationImu(1, 1);
    pureInsState.pos = bindingPosition;
    pureInsState.vel = pvaInitialCfg.initvel;
    pureInsState.pos(3) = bindingHeightM;
    if zeroVerticalVelocity
        pureInsState.vel(3) = 0;
    end

    navigationResult = zeros(sampleCount, 11);
    navigationResult(1, :) = [0, pureInsState.time, ...
        pureInsState.pos(1) * param.R2D, ...
        pureInsState.pos(2) * param.R2D, pureInsState.pos(3), ...
        pureInsState.vel(:).', pureInsState.att(:).' * param.R2D];
    lastProgress = 0;

    fprintf('\n开始 %s 纯惯导（%s）。\n', ...
        runNames{runIndex}, constraintName);
    for imuIndex = 2:sampleCount
        lastImu = navigationImu(imuIndex - 1, :).';
        thisImu = navigationImu(imuIndex, :).';
        imuDt = thisImu(1) - lastImu(1);
        if ~isfinite(imuDt) || imuDt <= 0
            error('%s第%d行IMU时间间隔异常。', ...
                runNames{runIndex}, imuIndex);
        end

        pureInsState = InsMech(pureInsState, lastImu, thisImu);
        pureInsState.pos(3) = bindingHeightM;
        if zeroVerticalVelocity
            pureInsState.vel(3) = 0;
        end

        navigationResult(imuIndex, :) = [0, pureInsState.time, ...
            pureInsState.pos(1) * param.R2D, ...
            pureInsState.pos(2) * param.R2D, pureInsState.pos(3), ...
            pureInsState.vel(:).', pureInsState.att(:).' * param.R2D];

        if showProgress
            progress = floor(10 * imuIndex / sampleCount) / 10;
            if progress >= lastProgress + 0.1
                fprintf('%s进度：%d%%\n', ...
                    runNames{runIndex}, round(progress * 100));
                lastProgress = progress;
            end
        end
    end

    outputDirectory = fileparts(runFiles{runIndex});
    if ~isfolder(outputDirectory)
        mkdir(outputDirectory);
    end
    writematrix(navigationResult, runFiles{runIndex}, ...
        'FileType', 'text', 'Delimiter', ' ');
end

%% 8. 两次纯惯导相对新参考PVA的误差对比
artifactDirectory = paths.artifacts;
if ~isfolder(artifactDirectory)
    mkdir(artifactDirectory);
end

% 水平径向误差同图比较，并导出统一统计表。
[radialFigure, radialStatistics] = calc_radial_error_gjb(...
    referenceFile, outputPvaInitialFile, outputAlignedFile, false);
legend({'pva\_initial姿态', '新对准姿态'}, 'Location', 'best');
radialStatistics{2, 1} = 'PureIns-PvaInitialAtt';
radialStatistics{3, 1} = 'PureIns-AlignedAtt';

radialPng = fullfile(artifactDirectory, ...
    'initial_attitude_radial_error_compare.png');
radialFig = fullfile(artifactDirectory, ...
    'initial_attitude_radial_error_compare.fig');
radialCsv = fullfile(artifactDirectory, ...
    'initial_attitude_radial_error_statistics.csv');
radialXlsx = fullfile(artifactDirectory, ...
    'initial_attitude_radial_error_statistics.xlsx');
exportgraphics(radialFigure, radialPng, 'Resolution', 300);
savefig(radialFigure, radialFig);
writecell(radialStatistics, radialCsv);
writecell(radialStatistics, radialXlsx, 'Sheet', 'statistics');
close(radialFigure);

% 分别保存完整的位置、速度和姿态误差图。
errorPngFiles = cell(2, 1);
for runIndex = 1:2
    errorFigure = calc_error_gjb(...
        runFiles{runIndex}, referenceFile, false, 'all');
    errorStem = lower(runNames{runIndex});
    errorPngFiles{runIndex} = fullfile(artifactDirectory, ...
        [errorStem '_navigation_error.png']);
    errorFigFile = fullfile(artifactDirectory, ...
        [errorStem '_navigation_error.fig']);
    exportgraphics(errorFigure, errorPngFiles{runIndex}, 'Resolution', 300);
    savefig(errorFigure, errorFigFile);
    close(errorFigure);
end

fprintf('\n处理完成。\n');
fprintf('新参考PVA：%s\n', referenceFile);
fprintf('pva_initial姿态纯惯导：%s\n', outputPvaInitialFile);
fprintf('新对准姿态纯惯导：%s\n', outputAlignedFile);
fprintf('径向误差对比：%s\n', radialPng);
fprintf('径向误差统计：%s\n', radialCsv);
