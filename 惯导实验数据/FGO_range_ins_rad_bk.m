clear;
% 参数初始化
param = Param();
cfg = ProcessConfig_exper_m();
%% 定义标准差和其他设置
rngstd = 6;
depthstd = 0.4;
rng(1);

% === 使用 backwardIsOpen 相关初始化 ===
backwardIsOpen = 0;
if backwardIsOpen == 1
    ki2 = 1;
    indexrecord2(1) = 1;
    navdt1s = [];
    NAV = [];
end
% ======================================================================

smoothWay = 'RTS'; % 平滑方式，选择RTS或线性
SmoothIsOpen = 0;
feedback = 1;
tic; % 启动计时器
%% 导入数据

% 导入IMU数据
imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);
% 导入范围数据
cfg.userange = 1;
rangedata1 = importdata(cfg.rangefile1path);
rangedata2 = importdata(cfg.rangefile2path);
rangedata3 = importdata(cfg.rangefile3path);
range = {rangedata1, rangedata2, rangedata3};
% 构造范围数据
id = 420; % 420秒 = 7分钟数据周期
for i = 1:3
    range{i} = range{i}(id:id:end, :);
end
% 初始化合并后的范围数据
rangedata = zeros(size(range{1}));
seq = [1, 2, 3];
for i = 1:3
    rangedata(i:3:end, :) = range{seq(i)}(i:3:end, :);
end
% 获取时间范围
rangestarttime = rangedata(1, 1);
rangeendtime = rangedata(end, 1);
% 导入高度数据
height = importdata(cfg.heightfilepath);
heightdata = height(id * 100:id * 100:end, :);
heightstarttime = heightdata(1, 1);
heightendtime = heightdata(end, 1);

%%
IsEKFRotate = 1;
if IsEKFRotate == 1
    nav_record = zeros(length(imudata),11);
    Start = 1;
end

%% 设置文件保存路径
navpath = fullfile(cfg.outputfolder, 'Origin-rad.nav');
navfp = fopen(navpath, 'wt');
% 根据设置是否启用平滑
if SmoothIsOpen == 1
    % 二次平滑结果
    navpath1 = fullfile(cfg.outputfolder, sprintf('%s-DoubleSmooth-rad.nav', smoothWay));
    navfp1 = fopen(navpath1, 'wt');
    % 单次平滑结果
    navpath2 = fullfile(cfg.outputfolder, sprintf('%s-SingleSmooth-rad.nav', smoothWay));
    navfp2 = fopen(navpath2, 'wt');
end

% === 新增：反向推算的文件创建 ===
if backwardIsOpen == 1
    navdt1spath = fullfile(cfg.outputfolder, sprintf('Backward-rad.nav'));
    navdt1sfp = fopen(navdt1spath,'wt');
    navdt1srotatepath = fullfile(cfg.outputfolder, sprintf('BackwardRotate-rad.nav'));
    navdt1srotatefp = fopen(navdt1srotatepath,'wt');
end
% ===============================
if IsEKFRotate == 1
    navEKFRotatepath = [cfg.outputfolder,'/EKFrotate.nav'];
    navEKFRotatefp = fopen(navEKFRotatepath,'wt');
end
%% 时间调整
if cfg.starttime < imustarttime
    cfg.starttime = imustarttime;
end
if cfg.endtime > imuendtime
    cfg.endtime = imuendtime;
end
% 筛选在处理时间范围内的数据
imudata = imudata(imudata(:, 1) >= cfg.starttime & imudata(:, 1) <= cfg.endtime, :);
rangedata = rangedata(rangedata(:, 1) >= cfg.starttime & rangedata(:, 1) <= cfg.endtime, :);
rangedata_true = rangedata(:, 3);
% 添加随机噪声到范围数据
rangedata(:, 3) = rangedata(:, 3) + normrnd(0, rngstd, size(rangedata(:, 3)));
% 筛选高度数据并添加噪声
height = height(height(:, 1) >= cfg.starttime & height(:, 1) <= cfg.endtime, :);
height(:, 2) = height(:, 2) + normrnd(0, depthstd, size(height(:, 2)));
heightdata = heightdata(heightdata(:, 1) >= cfg.starttime & heightdata(:, 1) <= cfg.endtime, :);
heightdata_true = heightdata(:, 2);
heightdata(:, 2) = heightdata(:, 2) + normrnd(0, depthstd, size(heightdata(:, 2)));


%% for debug
disp("Start INS/RANGE Processing!");
lastprecent = 0;
%% initialization
[kf, navstate] = myInitialize_15state(cfg);
laststate = navstate;
kf.rangstd = rngstd;
kf.depthstd = depthstd;

% === 新增：初始化记录状态供反向滤波使用 ===
navstate0 = navstate;
pos0 = navstate.pos;
vel0 = navstate.vel;
% =========================================

% data index preprocess
lastimu = imudata(1, :)';
thisimu = imudata(1, :)';
imudt = thisimu(1, 1) - lastimu(1, 1);
rangeindex = 1;
while rangedata(rangeindex, 1) < thisimu(1, 1)
    rangeindex = rangeindex + 1;
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MAIN PROCEDD PROCEDURE!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for imuindex = 2:size(imudata, 1)
    %% 1、set value of last state
    lastimu = thisimu;
    laststate = navstate;
    old_state = navstate;
    thisimu = imudata(imuindex, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);
    %% 2、compensate IMU error
    % thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * cfg.initgyrbiasstd);
    % thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * cfg.initaccbiasstd);
    %% 3、adjust range index
    while (rangeindex <= size(rangedata, 1) && rangedata(rangeindex, 1) < lastimu(1, 1))
        rangeindex = rangeindex + 1;
    end
    if (rangeindex > size(rangedata, 1))
        % rangeindex = rangeindex - 1;
        disp('range file END!');
        break;
    end
    %% 4、determine whether gnss update is required
    if lastimu(1, 1) == rangedata(rangeindex, 1) && cfg.userange==1
        % 测量更新
        kf = myRangeUpdate(navstate, rangedata(rangeindex,:), height(imuindex,:), kf);
        rangeindex = rangeindex + 1;
        if SmoothIsOpen == 1
            if buf_idx > 1
                valid_len = buf_idx - 1;
                state_buffer = state_buffer(1:valid_len, :);
                xk_final = kf.x;
                Xk_k1propa    = Xk_k1propa(1:valid_len, :);
                Pk_k1propa    = Pk_k1propa(1:valid_len, :);
                Pk_propa    = Pk_propa(1:valid_len, :);
                PHI   = PHI(1:valid_len, :);
                % 调用 RTS 平滑函数
                [nav_matrix, bridge_error, rtsstate_buffer] = perform_unified_smoothing(state_buffer, xk_final, ...
                    param, rangeindex, smoothWay, 'rad', Pk_propa, Pk_k1propa, PHI);
                fprintf(navfp2, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix);
                bridge_err(rangeindex,:) = bridge_error;
                % 2. 判断是否有【上一个 7 分钟】的数据被缓存
                if isempty(prev_state_buffer)
                    % 如果是第一段数据 (比如 0-7min)，没法进行二次平滑，直接暂存起来
                    prev_state_buffer = rtsstate_buffer;
                    prev_Pk_propa     = Pk_propa;
                    prev_Pk_k1propa   = Pk_k1propa;
                    prev_PHI          = PHI;
                    prev_rangeindex   = rangeindex;
                else
                    [nav_matrix_prev_resmoothed, bridge_error, smoothed_state_buffer] = perform_unified_smoothing(prev_state_buffer, bridge_error, ...
                        param, prev_rangeindex, smoothWay, 'rad', prev_Pk_propa, prev_Pk_k1propa, prev_PHI);
                    fprintf(navfp1, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix_prev_resmoothed);
                    % 更新缓存：把当前的 7-14min 变成新的“上一段”，等待 14-21min 来救赎它
                    prev_state_buffer = rtsstate_buffer;
                    prev_Pk_propa     = Pk_propa;
                    prev_Pk_k1propa   = Pk_k1propa;
                    prev_PHI          = PHI;
                    prev_rangeindex   = rangeindex;
                end
                buf_idx = 1;
                state_buffer(:) = 0;
                Xk_k1propa(:)   = 0;
                Pk_k1propa(:)   = 0;
                Pk_propa(:)     = 0;
                PHI(:)          = 0;
            end
        end
        if feedback==1
            [kf, navstate] = myErrorFeedback_range(kf, navstate);
            % [kf, navstate] = myErrorFeedback_15state(kf, navstate);
        end
        laststate = navstate;
        % 惯导推算
        imudt = thisimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, thisimu);
        kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);

        % === 新增：反向推算执行模块 ===
        if backwardIsOpen == 1
            runArgs.imudata      = imudata;
            runArgs.imuindex     = imuindex;
            runArgs.ki2          = ki2;
            runArgs.kf           = kf;
            runArgs.navstate     = navstate;
            runArgs.indexrecord2 = indexrecord2;
            runArgs.rangedata    = rangedata;
            runArgs.navdt1s      = navdt1s;
            runArgs.NAV          = NAV;
            runArgs.height       = height;
            runArgs.rangeindex   = rangeindex;
            runArgs.navstate0    = navstate0;
            runArgs.meas         = "Range"; % 假设以测距为主
            [NAV, navdt1s, indexrecord2, ki2] = backward_1s(runArgs);
        end
        pos0 = navstate.pos;
        vel0 = navstate.vel;
        % =============================
        IsRangeUpdate = 1;
    else
        %% only do propagation
        % INS mechanization
        navstate = InsMech(laststate, lastimu, thisimu);
        % 调用高度卡尔曼量测更新
        kf = myHeightUpdate(navstate, height(imuindex, :), kf);
        % 反馈修正惯导状态 (仅修正天向)
        navstate.pos(3) = navstate.pos(3) - kf.x(3);
        navstate.vel(3) = navstate.vel(3) - kf.x(6);
        % 💡 反馈后，将整个误差状态量清零（代表误差已融入主状态）
        kf.x(3) = 0; kf.x(6) = 0;
        if SmoothIsOpen == 1
            Pk_propa(buf_idx,:) = kf.P(:)';
        end
        % error propagation
        kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
        if SmoothIsOpen == 1
            nav = [navstate.time;navstate.pos;navstate.vel;navstate.att];
            state_buffer(buf_idx,:) =  nav';
            Xk_k1propa(buf_idx,:) = kf.x(:)';
            Pk_k1propa(buf_idx,:) = kf.P(:)';
            PHI(buf_idx,:) = kf.phi(:)';
            buf_idx = buf_idx + 1;
        end
        IsRangeUpdate = 0;
    end
    %% save data
    % write navresult to file
    nav = zeros(11, 1);
    nav(2, 1) = navstate.time;
    nav(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
    nav(6:8, 1) = navstate.vel;
    nav(9:11, 1) = navstate.att * param.R2D;
    fprintf(navfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);

    nav_record(imuindex-1,:) = nav;
    if IsEKFRotate == 1 && IsRangeUpdate == 1   
        rotatepoint = [navstate.pos(1:2);0];% 旋转点
        index = Start:imuindex-2;
        trajectory =[d2r(nav_record(index,3:4)), zeros(length(index),1)]'; % 待旋转轨迹
        [rotatedTrajectory11, ~, ~] = rotateAndScaleTrajectory(trajectory, rotatepoint);
        rotatedTrajectory = [rotatedTrajectory11,rotatepoint];
        nav_record([index,imuindex-1],3:4) = r2d(rotatedTrajectory(1:2,:)');
        Start = imuindex;
        fprintf(navEKFRotatefp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_record([index,imuindex-1],:)');
    end
    % print processing information
    if (imuindex / size(imudata, 1) - lastprecent > 0.20)
        disp("processing " + num2str(floor(imuindex * 100 / size(imudata, 1))) + " %!");
        lastprecent = imuindex / size(imudata, 1);
    end
end
%%
% for 循环结束，写入最后一段缓存的轨迹
if ~isempty(prev_state_buffer)
    % 最后一段没有未来信息了，我们只能用它自己第一次平滑的结果
    perform_unified_smoothing(prev_state_buffer, zeros(15,1), ...
        param, rangeindex, 'Linear', 'rad', prev_Pk_propa, prev_Pk_k1propa, prev_PHI);
    fprintf(navfp1, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix);
end

% === 新增：反向推算数据写出和误差分析 ===
if backwardIsOpen == 1
    fprintf(navdt1sfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', navdt1s');
    fprintf(navdt1srotatefp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', NAV');

    % 以下是如果你需要计算误差或画图，可以取消注释
    % calc_error(navdt1spath, cfg.truthpath);
    % calc_error(navdt1srotatepath, cfg.truthpath);
    % calc_radial_error(cfg.truthpath, navpath, navdt1spath);
    % plot_trj(cfg.truthpath, navdt1spath, navdt1srotatepath, navpath);
end
% =======================================

fclose all;
toc
