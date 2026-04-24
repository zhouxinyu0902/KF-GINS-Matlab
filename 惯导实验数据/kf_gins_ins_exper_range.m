clear;
param = Param();
cfg = ProcessConfig_exper();
%%
rangstd = 2;
depthstd= 0.4;
rng(1)
backwardIsOpen = 0;
% smoothWay = 'Linear';
smoothWay = 'RTS';
SmoothIsOpen = 0;
tic
%% importdata data
% imudata
imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);

% range data
cfg.userange=1;
rangedata1 = importdata(cfg.rangefile1path);
rangedata2 = importdata(cfg.rangefile2path);
rangedata3 = importdata(cfg.rangefile3path);
range = {rangedata1,rangedata2,rangedata3};

% 构造
id = 420; % 420s=7min数据周期
for i=1:3
    range{i} = range{i}(id:id:end,:);
end
rangedata = zeros(size(range{1}));
seq = [1,2,3];
for i=1:3
    rangedata(i:3:end,:)=range{seq(i)}(i:3:end,:);
end

rangestarttime = rangedata(1, 1);
rangeendtime = rangedata(end, 1);


height = importdata(cfg.heightfilepath);
heightdata = height(id*100:id*100:end,:);
heightstarttime = heightdata(1, 1);
heightendtime = heightdata(end, 1);

%% 设置文件保存路径
navpath = [cfg.outputfolder, '/NavResult-RANGE-1.nav'];
navfp = fopen(navpath, 'wt');

if SmoothIsOpen == 1
    navpath1 = [cfg.outputfolder, '/NavResult-RANGE-',smoothWay,'Distribute-double.nav'];
    navfp1 = fopen(navpath1, 'wt');

    navpath2 = [cfg.outputfolder, '/NavResult-RANGE-',smoothWay,'Distribute.nav'];
    navfp2 = fopen(navpath2, 'wt');
end

if backwardIsOpen == 1
    navpath_bw = [cfg.outputfolder, '/NavResult-RANGE-Backward.nav'];
    navfp_bw = fopen(navpath_bw, 'wt');
end
%
% imuerrpath = [cfg.outputfolder, '/ImuError.txt'];
% imuerrfp = fopen(imuerrpath, 'wt');
%
% stdpath = [cfg.outputfolder, '/NavSTD.txt'];
% stdfp = fopen(stdpath, 'wt');
%
% xkpath = [cfg.outputfolder, '/xk_range.txt'];
% xkfp = fopen(xkpath, 'wt');
%% 获取处理时间，调整时间

if cfg.starttime < imustarttime
    cfg.starttime = imustarttime;
end
if cfg.endtime > imuendtime
    cfg.endtime = imuendtime;
end

% data in process interval
imudata = imudata(imudata(:,1) >= cfg.starttime, :);
imudata = imudata(imudata(:,1) <= cfg.endtime, :);
rangedata = rangedata(rangedata(:, 1) >= cfg.starttime, :);
rangedata = rangedata(rangedata(:, 1) <= cfg.endtime, :);
rangedata_true = rangedata(:,3) ;
rangedata(:,3) = rangedata(:,3) + normrnd(6,rangstd,size(rangedata(:,3)));

rangstd = 6;

height = height(height(:, 1) >= cfg.starttime, :);
height = height(height(:, 1) <= cfg.endtime, :);
height(:,2) = height(:,2) + normrnd(1,depthstd,size(height(:,2)));

heightdata = heightdata(heightdata(:, 1) >= cfg.starttime, :);
heightdata = heightdata(heightdata(:, 1) <= cfg.endtime, :);
heightdata_true = heightdata(:,2) ;
heightdata(:,2) = heightdata(:,2) + normrnd(1,depthstd,size(heightdata(:,2)));
writematrix(rangedata, '惯导实验数据\input\rangedata_noised.txt', 'Delimiter', 'space'); % 空格
writematrix(height, '惯导实验数据\input\height_noised.txt', 'Delimiter', 'space'); % 空格
%% 计算并提取添加的误差
% % 提取时间轴
% time_range = rangedata(:, 1);
% time_height = heightdata(:, 1);
%
% % 准确的获取方式应该是：
%  height_err_sequence = heightdata(:,2) - heightdata_true;
%  range_err_sequence = rangedata(:,3) - rangedata_true;
%
% % 开始绘图
% fig = myfigurestartup(6, 3, 'paper');
% set(fig, 'Color', 'w', 'Name', '传感器仿真误差分析');
%
% % 子图1：测距数据误差 (Range Error)
% subplot(2, 1, 1);
% plot(time_range, range_err_sequence, 'Color', [0.85, 0.33, 0.1], 'LineWidth', 1);
% hold on;
% % 绘制 6m 偏差基准线
% line([time_range(1), time_range(end)], [6, 6], 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1.5);
% grid on;
% ylabel('测距误差 (m)');
% title(['测距误差分布 (Bias = 6m, \sigma = ', num2str(rangstd), 'm)']);
% legend('含噪误差', '系统偏差 (Bias)', 'Location', 'northeast');
% xlim([time_range(1), time_range(end)])
% % 子图2：高度数据误差 (Height Error)
% subplot(2, 1, 2);
% plot(time_height, height_err_sequence, 'Color', [0, 0.45, 0.74], 'LineWidth', 1);
% hold on;
% % 绘制 1m 偏差基准线
% line([time_height(1), time_height(end)], [1, 1], 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1.5);
% grid on;
% xlabel('时间 (s)');
% ylabel('高度误差 (m)');
% title(['高度误差分布 (Bias = 1m, \sigma = ', num2str(depthstd), 'm)']);
% legend('含噪误差', '系统偏差 (Bias)', 'Location', 'northeast');
% xlim([time_range(1), time_range(end)])
% exportgraphics(fig, fullfile('D:\GitHub\KF-GINS-Matlab\惯导实验数据\New Folder\', ...
%     'F-1.png'), 'Resolution', 600);
%
% % 调整整体布局
% % sgtitle('仿真传感器误差特性可视化');
%% for debug
disp("Start INS/RANGE Processing!");
lastprecent = 0;
%% initialization
[kf, navstate] = myInitialize_15state(cfg);
laststate = navstate;
kf.rangstd = rangstd;
kf.depthstd = depthstd;
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
buf_idx1 = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MAIN PROCEDD PROCEDURE!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for imuindex = 2:size(imudata, 1)-1

    %% set value of last state
    lastimu = thisimu;
    laststate = navstate;
    thisimu = imudata(imuindex, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);

    %% compensate IMU error
    % thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * navstate.gyrbias)./(ones(3, 1) + navstate.gyrscale);
    % thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * navstate.accbias)./(ones(3, 1) + navstate.accscale);
    % thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * navstate.gyrbias);
    % thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * navstate.accbias);
    %% adjust range index
    while (rangeindex <= size(rangedata, 1) && rangedata(rangeindex, 1) < lastimu(1, 1))
        rangeindex = rangeindex + 1;
    end
    % check whether gnss data is valid
    if (rangeindex > size(rangedata, 1))
        disp('range file END!');
        break;
    end
    is_range_update = false;
    %% determine whether gnss update is requiredmyRangeUpdate_adap
    if lastimu(1, 1) == rangedata(rangeindex, 1)
        Rangedata = rangedata(rangeindex,:);
        depthdata = heightdata(rangeindex,:);

        kf = myRangeUpdate(navstate, Rangedata, depthdata, kf);

        if SmoothIsOpen == 1
            if buf_idx > 1
                valid_len = buf_idx - 1;
                state_buffer = state_buffer(1:valid_len, :);
                if strcmp(smoothWay,'RTS')
                    xk_final = kf.x;
                    Xk_k1propa    = Xk_k1propa(1:valid_len, :);
                    Pk_k1propa    = Pk_k1propa(1:valid_len, :);
                    Pk_propa    = Pk_propa(1:valid_len, :);
                    PHI   = PHI(1:valid_len, :);

                    % % 调用 RTS 平滑函数
                    [nav_matrix, bridge_error,rtsstate_buffer] = perform_RTS_smoothing(state_buffer,  Pk_propa, Pk_k1propa,  PHI, xk_final, param, rangeindex);
                    % 批量写入文件
                    fprintf(navfp2, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix);
                    bridge_err(rangeindex,:) = bridge_error;
                    % 1. 先对【当前的 7 分钟】做一次常规 RTS 平滑，拿到桥接误差 bridge_error

                    % [nav_matrix, bridge_error] = perform_RTS_smoothing_new(state_buffer, Pk_propa, Pk_k1propa, PHI, xk_final, param, rangeindex);
                    % fprintf(navfp2, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix);

                    % 2. 判断是否有【上一个 7 分钟】的数据被缓存
                    if isempty(prev_state_buffer)
                        % 如果是第一段数据 (比如 0-7min)，没法进行二次平滑，直接暂存起来
                        prev_state_buffer = rtsstate_buffer;
                        prev_Pk_propa     = Pk_propa;
                        prev_Pk_k1propa   = Pk_k1propa;
                        prev_PHI          = PHI;
                        prev_rangeindex   = rangeindex;

                    else
                        % 如果有上一个 7 分钟的数据 (比如现在算完了 7-14min)
                        % 核心操作：把 7-14min 算出的起始误差 (bridge_error)，当作 0-7min 的终点误差！

                        % [nav_matrix_prev_resmoothed, ~] = perform_RTS_smoothing_new(prev_state_buffer, prev_Pk_propa, prev_Pk_k1propa, prev_PHI, bridge_error, param, prev_rangeindex);
                        % [nav_matrix_prev_resmoothed, ~] = perform_RTS_smoothing(prev_state_buffer,  prev_Pk_propa, prev_Pk_k1propa,  prev_PHI, bridge_error, param, rangeindex);

                        nav_matrix_prev_resmoothed = perform_block_smoothing(prev_state_buffer, bridge_error, param, prev_rangeindex);
                        % 将经历过“未来信息”二次洗礼的上一段数据，写入文件！
                        fprintf(navfp1, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix_prev_resmoothed);

                        % 更新缓存：把当前的 7-14min 变成新的“上一段”，等待 14-21min 来救赎它
                        prev_state_buffer = rtsstate_buffer;
                        prev_Pk_propa     = Pk_propa;
                        prev_Pk_k1propa   = Pk_k1propa;
                        prev_PHI          = PHI;
                        prev_rangeindex   = rangeindex;
                    end
                elseif strcmp(smoothWay,'Linear')
                    nav_matrix = perform_block_smoothing(state_buffer, kf.x, param, rangeindex);
                    % 批量写入文件
                    fprintf(navfp1, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix);
                end

                % 清空所有缓存，开启下个 7min 周期
                buf_idx = 1;
            end
        end

        [kf, navstate] = myErrorFeedback_range(kf, navstate);
        % [kf, navstate] = myErrorFeedback_range_posonly(kf, navstate);

        rangeindex = rangeindex + 1;
        laststate = navstate;

        imudt = thisimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, thisimu);

        % if backwardIsOpen == 1
        %     % 计算这 7 分钟数据在整个 imudata 里的起止索引
        %     % buf_idx - 1 就是这段时间经历了多少个点
        %     valid_len = buf_idx1 - 1;
        %     start_idx = imuindex - valid_len;
        %     end_idx   = imuindex;
        % 
        %     % 【核心切片】剥离出这 7 分钟的 IMU 和高度数据
        %     imu_block    = imudata(start_idx : end_idx, :);
        %     height_block = height(start_idx : end_idx, :);
        %     if rangeindex == 2
        %         meas=[];
        %     else
        %         meas.range = rangedata(rangeindex-2,:);
        %         meas.height = heightdata(rangeindex-2,:);
        %     end
        %     % 极简调用反向推算函数
        %     nav_matrix_bw = perform_backward(imu_block, height_block, navstate, kf, param, meas,rangeindex);
        % 
        %     % 批量写入反向结果文件
        %     fprintf(navfp_bw, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix_bw);
        %     buf_idx1 = 1;
        % end
        % 
        % if SmoothIsOpen == 1
        %     Pk_propa(buf_idx,:) = kf.P(:)';
        % end
        % 
        % kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
        % 
        % if SmoothIsOpen == 1
        %     nav = [navstate.time;navstate.pos;navstate.vel;navstate.att];
        %     state_buffer(buf_idx,:) =  nav';
        %     Xk_k1propa(buf_idx,:) = kf.x(:)';
        %     Pk_k1propa(buf_idx,:) = kf.P(:)';
        %     PHI(buf_idx,:) = kf.phi(:)';
        %     buf_idx = buf_idx + 1;
        % end

    % elseif (lastimu(1, 1) < rangedata(rangeindex, 1) && thisimu(1, 1) > rangedata(rangeindex, 1))
    %     % ineterpolate imu to range time
    %     [firstimu, secondimu] = interpolate(lastimu, thisimu, rangedata(rangeindex, 1));
    % 
    %     % do propagation for first imu
    %     imudt = firstimu(1, 1) - lastimu(1, 1);
    %     navstate = InsMech(laststate, lastimu, firstimu);
    %     kf = myInsPropagate_15state(navstate, firstimu, imudt, kf);
    % 
    %     % do update
    %     Rangedata = rangedata(rangeindex,:);
    %     depthdata = heightdata(rangeindex,:);
    %     kf = myRangeUpdate(navstate, Rangedata, depthdata, kf);
    % 
    %     % 2. 【核心注入】对缓存进行平滑并批量写入
    %     if SmoothIsOpen == 1
    %         if buf_idx > 1
    %             valid_len = buf_idx - 1;
    %             state_buffer = state_buffer(1:valid_len, :);
    %             if strcmp(smoothWay,'RTS')
    %                 xk_final = kf.x;
    %                 Xk_k1propa    = Xk_k1propa(1:valid_len, :);
    %                 Pk_k1propa    = Pk_k1propa(1:valid_len, :);
    %                 Pk_propa    = Pk_propa(1:valid_len, :);
    %                 PHI   = PHI(1:valid_len, :);
    % 
    %                 % % 调用 RTS 平滑函数
    %                 [nav_matrix, bridge_error] = perform_RTS_smoothing(state_buffer,  Pk_propa, Pk_k1propa,  PHI, xk_final, param, rangeindex);
    %                 % 批量写入文件
    %                 fprintf(navfp2, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix);
    % 
    %                 % 1. 先对【当前的 7 分钟】做一次常规 RTS 平滑，拿到桥接误差 bridge_error
    % 
    %                 % [nav_matrix, bridge_error] = perform_RTS_smoothing_new(state_buffer, Pk_propa, Pk_k1propa, PHI, xk_final, param, rangeindex);
    %                 % fprintf(navfp2, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix);
    % 
    %                 % % 2. 判断是否有【上一个 7 分钟】的数据被缓存
    %                 % if isempty(prev_state_buffer)
    %                 %     % 如果是第一段数据 (比如 0-7min)，没法进行二次平滑，直接暂存起来
    %                 %     prev_state_buffer = state_buffer;
    %                 %     prev_Pk_propa     = Pk_propa;
    %                 %     prev_Pk_k1propa   = Pk_k1propa;
    %                 %     prev_PHI          = PHI;
    %                 %     prev_rangeindex   = rangeindex;
    %                 % else
    %                 %     % 如果有上一个 7 分钟的数据 (比如现在算完了 7-14min)
    %                 %     % 核心操作：把 7-14min 算出的起始误差 (bridge_error)，当作 0-7min 的终点误差！
    %                 % 
    %                 %     % [nav_matrix_prev_resmoothed, ~] = perform_RTS_smoothing_new(prev_state_buffer, prev_Pk_propa, prev_Pk_k1propa, prev_PHI, bridge_error, param, prev_rangeindex);
    %                 %     nav_matrix_prev_resmoothed = perform_block_smoothing(prev_state_buffer, bridge_error, param, prev_rangeindex);
    %                 %     % 将经历过“未来信息”二次洗礼的上一段数据，写入文件！
    %                 %     fprintf(navfp1, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix_prev_resmoothed);
    %                 % 
    %                 %     % 更新缓存：把当前的 7-14min 变成新的“上一段”，等待 14-21min 来救赎它
    %                 %     prev_state_buffer = state_buffer;
    %                 %     prev_Pk_propa     = Pk_propa;
    %                 %     prev_Pk_k1propa   = Pk_k1propa;
    %                 %     prev_PHI          = PHI;
    %                 %     prev_rangeindex   = rangeindex;
    %                 % end
    %             elseif strcmp(smoothWay,'Linear')
    %                 nav_matrix = perform_block_smoothing(state_buffer, kf.x, param, rangeindex);
    %                 % 批量写入文件
    %                 fprintf(navfp1, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix);
    %             end
    % 
    %             % 清空所有缓存，开启下个 7min 周期
    %             buf_idx = 1;
    %         end
    %     end
    % 
    %     [kf, navstate] = myErrorFeedback_range(kf, navstate);
    %     % [kf, navstate] = myErrorFeedback_range_posonly(kf, navstate);
    %     rangeindex = rangeindex + 1;
    %     laststate = navstate;
    %     lastimu = firstimu;
    % 
    %     % do propagation for second imu
    %     imudt = secondimu(1, 1) - lastimu(1, 1);
    %     navstate = InsMech(laststate, lastimu, secondimu);
    % 
    %     Pk_propa(buf_idx,:) = kf.P(:)';
    % 
    %     kf = myInsPropagate_15state(navstate, secondimu, imudt, kf);
    % 
    %     nav = [navstate.time;navstate.pos;navstate.vel;navstate.att];
    %     state_buffer(buf_idx,:) =  nav';
    %     Xk_k1propa(buf_idx,:) = kf.x(:)';
    %     Pk_k1propa(buf_idx,:) = kf.P(:)';
    %     PHI(buf_idx,:) = kf.phi(:)';
    %     buf_idx = buf_idx + 1;
    else
        %% only do propagation
        % INS mechanization
        navstate = InsMech(laststate, lastimu, thisimu);
        

        kf = myHeightUpdate(navstate, height(imuindex,:), kf);
        navstate.pos(3) = navstate.pos(3) - kf.x(3);
        navstate.vel(3) = navstate.vel(3) - kf.x(6);
        kf.x = zeros(size(kf.x));
        % navstate.pos(3) = height(imuindex,2); 
        if SmoothIsOpen == 1
            Pk_propa(buf_idx,:) = kf.P(:)';
        end

        % error propagation
        kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
        if SmoothIsOpen == 1
            % 【核心注入】将当前状态存入缓存，不立即写入文件
            nav = [navstate.time;navstate.pos;navstate.vel;navstate.att];
            state_buffer(buf_idx,:) =  nav';
            Xk_k1propa(buf_idx,:) = kf.x(:)';
            Pk_k1propa(buf_idx,:) = kf.P(:)';
            PHI(buf_idx,:) = kf.phi(:)';
            buf_idx = buf_idx + 1;
        end

        buf_idx1 = buf_idx1 + 1;
    end

    %% save data
    % xkk(imuindex-1,:)=[navstate.time;kf.x];
    % write navresult to file
    nav = zeros(11, 1);
    nav(2, 1) = navstate.time;
    nav(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
    nav(6:8, 1) = navstate.vel;
    nav(9:11, 1) = navstate.att * param.R2D;
    fprintf(navfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);
    % 保存估计的状态值
    % xk = zeros(16, 1);
    % xk(1) = navstate.time;
    % xk(2:16) = kf.x(1:15);
    % fprintf(xkfp, '%12.6f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f\n', xk);
    % write imu error, convert to common unit
    % imuerror = zeros(13, 1);
    % imuerror(1, 1) = navstate.time;
    % imuerror(2:4, 1) = navstate.gyrbias * param.R2D * 3600;
    % imuerror(5:7, 1) = navstate.accbias * 1e5;
    % imuerror(8:10, 1) = navstate.gyrscale * 1e6;
    % imuerror(11:13, 1) = navstate.accscale * 1e6;
    % fprintf(imuerrfp, '%12.6f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', imuerror);
    %
    % % write state std, convert to common unit
    % std = zeros(1, 22);
    % std(1) = navstate.time;
    % for idx=1:21
    %     std(idx + 1) = sqrt(kf.P(idx, idx));
    % end
    % std(8:10) = std(8:10) * param.R2D;
    % std(11:13) = std(11:13) * param.R2D *3600;
    % std(14:16) = std(14:16) * 1e5;
    % std(17:22) = std(17:22) * 1e6;
    % fprintf(stdfp, '%12.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f \n', std);
    %

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
    % 重新运行一次（或者你可以在前面把 nav_matrix_curr 也缓存下来直接写）
    % 这里简单起见，用全 0 补偿它的最终 xk_final，或者直接用之前算好的结果
    nav_matrix = perform_block_smoothing(rtsstate_buffer, zeros(15,1), param, rangeindex);
    fprintf(navfp1, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix);
end

fclose all;
toc
%%
calc_error(navpath,cfg.truthpath);
%%
glvs
error=sqrt((bridge_err(:,1)*glv.Re).^2+(bridge_err(:,2)*glv.Re*cos(36/180*pi)).^2);
figure
plot(error,'*')
%%
% plot_result(navpath)

%%
calc_radial_error(cfg.truthpath,navpath,navpath2,navpath1);
%%
calc_error(navpath1,cfg.truthpath);
%%
calc_error(navpath2,cfg.truthpath);
%%
calc_error(navpath_bw,cfg.truthpath);
%%
calc_radial_error(cfg.truthpath,navpath,'output\NavResult-RANGE-RTSDistribute.nav');
exportgraphics(gca, fullfile('D:\GitHub\KF-GINS-Matlab\惯导实验数据\New Folder\', ...
    'F-2.png'), 'Resolution', 600);
%%
calc_radial_error(cfg.truthpath,navpath,navpath1);
exportgraphics(gca, fullfile('D:\GitHub\KF-GINS-Matlab\惯导实验数据\New Folder\', ...
    'F-3.png'), 'Resolution', 600);
%%
calc_radial_error(cfg.truthpath,navpath,'output\NavResult-RANGE-LinearDistribute.nav', ...
    'output\NavResult-RANGE-RTSDistribute.nav');
exportgraphics(gca, fullfile('D:\GitHub\KF-GINS-Matlab\惯导实验数据\New Folder\', ...
    'F-4.png'), 'Resolution', 600);
%%
calc_radial_error(cfg.truthpath,navpath,'output\NavResult-RANGE-LinearDistribute.nav', ...
    'output\NavResult-RANGE-RTSDistribute.nav','output\NavResult-RANGE-Backward.nav');
exportgraphics(gca, fullfile('D:\GitHub\KF-GINS-Matlab\惯导实验数据\New Folder\', ...
    'F-5.png'), 'Resolution', 600);