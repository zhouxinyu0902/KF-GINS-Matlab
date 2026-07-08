clear;
param = Param();
glvs
%%
path='D:\GitHub\KF-GINS-Matlab\旋转收缩方案1/input/input6';
cfg = ProcessConfigforSemiPhy_all(path);

rangstd = 6;
depthstd = 0.4;
rng(1)
backwardIsOpen = 0;
% smoothWay = 'Linear';
smoothWay = 'RTS';
SmoothIsOpen = 1;
feedback = 1;
tic
type = {"single","moving","3","2"};
beacontype = type{3};

type = {"Range","Range+azi","Range+azi+pos","pos"};
meas = type{1};
backwardIsOpen_1s = 1;
IsEKFRotate = 1 ;
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
truth = importdata(cfg.truthpath);

%% 设置文件保存路径
navpath = [cfg.outputfolder, '/Origin.nav'];
navfp = fopen(navpath, 'wt');

if SmoothIsOpen == 1
    navpath1 = [cfg.outputfolder,'/Compensation-double.nav'];
    navfp1 = fopen(navpath1, 'wt');

    navpath2 = [cfg.outputfolder,'/Compensation.nav'];
    navfp2 = fopen(navpath2, 'wt');
end

if backwardIsOpen_1s == 1
    navdt1spath = [cfg.outputfolder,'/Backward.nav']; %% 后向滤波
    navdt1sfp = fopen(navdt1spath,'wt');
    navdt1srotatepath = [cfg.outputfolder,'/BackwardFilter.nav']; %% 后向滤波
    navdt1srotatefp = fopen(navdt1srotatepath,'wt');
end

if IsEKFRotate == 1
    navEKFRotatepath = [cfg.outputfolder,'/Rotate.nav'];
    navEKFRotatefp = fopen(navEKFRotatepath,'wt');

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
height = truth(:,[2,5]);
height = height(height(:, 1) >= cfg.starttime, :);
height = height(height(:, 1) <= cfg.endtime, :);
height(:,2) = height(:,2) + normrnd(1,depthstd,size(height(:,2)));
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
if backwardIsOpen_1s == 1
    ki2 = 1;
    indexrecord2(1) = 1;
    navdt1s = [];
    NAV = [];
end
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

navstate0 = navstate;
nav_record = zeros(length(imudata),11);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MAIN PROCEDD PROCEDURE!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for imuindex = 2:size(imudata, 1)
    % 1、set value of last state
    lastimu = thisimu;
    laststate = navstate;
    old_state = navstate;
    thisimu = imudata(imuindex, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);

    % 2、compensate IMU error
    % thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * cfg.initgyrbiasstd);
    % thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * cfg.initaccbiasstd);
    % 3、adjust range index
    while (rangeindex <= size(rangedata, 1) && rangedata(rangeindex, 1) < lastimu(1, 1))
        rangeindex = rangeindex + 1;
    end
    if (rangeindex > size(rangedata, 1))
        % rangeindex = rangeindex - 1;
        disp('range file END!');
        break;
    end
    % 4、determine whether gnss update is required
    if lastimu(1, 1) == rangedata(rangeindex, 1) && cfg.userange==1
        % 测量更新
        kf = myRangeUpdate(navstate, rangedata(rangeindex,:), height(imuindex,:), kf);
        rangeindex = rangeindex + 1;

       
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
                    fprintf(navfp2, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix);
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

         if backwardIsOpen_1s == 1 
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
                runArgs.meas         = meas;
                [NAV,navdt1s,indexrecord2,ki2] = backward_1s(runArgs);
        end

        IsRangeUpdate = 1;
    elseif (lastimu(1, 1) < rangedata(rangeindex, 1) && thisimu(1, 1) > rangedata(rangeindex, 1))&& cfg.userange==1
        kf = myRangeUpdate(navstate, rangedata(rangeindex,:), height(imuindex,:), kf);
        if feedback == 1
            [kf, navstate] = myErrorFeedback_range(kf, navstate);
            % [kf, navstate] = myErrorFeedback_15state(kf, navstate);
        end
        rangeindex = rangeindex + 1;
        laststate = navstate;
        lastimu = firstimu;

        % do propagation for second imu
        imudt = secondimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, secondimu);
        kf = myInsPropagate_15state(navstate, secondimu, imudt, kf);
        pos0 = navstate.pos;
        vel0 = navstate.vel;
        
    else
        % only do propagation
        % INS mechanization
        navstate = InsMech(laststate, lastimu, thisimu);
        navstate.pos(3) = height(imuindex,2);

        % % 调用高度卡尔曼量测更新
        % kf = myHeightUpdate(navstate, height(imuindex, :), kf);
        % 
        % % 反馈修正惯导状态 (仅修正天向)
        % navstate.pos(3) = navstate.pos(3) + kf.x(3);
        % navstate.vel(3) = navstate.vel(3) - kf.x(6);
        % 
        % % 💡 反馈后，将整个误差状态量清零（代表误差已融入主状态）
        % kf.x = zeros(size(kf.x));
        % % kf.x(3) = 0; kf.x(6) = 0;

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
        IsRangeUpdate = 0;
    end

    % save data
    % xkk(imuindex-1,:)=[navstate.time;kf.x];
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
            index = indexrecord2(ki2-1):imuindex-2;
            
            trajectory =[d2r(nav_record(index,3:4)), zeros(length(index),1)]'; % 待旋转轨迹
            [rotatedTrajectory11, ~, ~] = rotateAndScaleTrajectory(trajectory, rotatepoint);
            rotatedTrajectory = [rotatedTrajectory11,rotatepoint];
            nav_record([index,imuindex-1],3:4) = r2d(rotatedTrajectory(1:2,:)');
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
    % 重新运行一次（或者你可以在前面把 nav_matrix_curr 也缓存下来直接写）
    % 这里简单起见，用全 0 补偿它的最终 xk_final，或者直接用之前算好的结果
    nav_matrix = perform_block_smoothing(rtsstate_buffer, zeros(15,1), param, rangeindex);
    fprintf(navfp1, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix);
end
if backwardIsOpen_1s == 1
    fprintf(navdt1sfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', navdt1s');
    fprintf(navdt1srotatefp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', NAV');
end
fclose all;
toc
%%
calc_radial_error(cfg.truthpath,pureins)
exportgraphics(gca, fullfile(cfg.outputfolder, 'pureins.png'), 'Resolution', 600);
%%
calc_radial_error(cfg.truthpath,navpath)
exportgraphics(gca, fullfile(cfg.outputfolder, 'EKF.png'), 'Resolution', 600);
%%
pureins = [cfg.outputfolder,'/PureIns.nav'];
[fig,finalExcelData]=calc_radial_error(cfg.truthpath,navpath,navdt1spath, navdt1srotatepath, navEKFRotatepath,navpath2,navpath1);
outputExcelName = [cfg.outputfolder,'/导航系统径向误差统计报告-all','.xlsx'];
writecell(finalExcelData, outputExcelName);
exportgraphics(fig, fullfile(cfg.outputfolder, 'all.png'), 'Resolution', 600);
%%
calc_error(navpath2,cfg.truthpath);
%%
fig = plot_trajectory_and_beacons(cfg.truthpath, rangedata(1:3,4:6));
exportgraphics(fig, fullfile('D:\GitHub\KF-GINS-Matlab\惯导实验数据\New Folder\', ...
    'trj-bea.png'), 'Resolution', 600);
%%
glvs
error = sqrt((bridge_err(:,1)*glv.Re).^2+(bridge_err(:,2)*glv.Re*cos(36/180*pi)).^2);
figure
plot(error,'*')
%%
% plot_result(navpath)

%%
calc_radial_error(cfg.truthpath,navpath,navpath,navpath2,navpath1);
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

[fig,finalExcelData]=calc_radial_error(cfg.truthpath,pureins,navpath,navpath2);
exportgraphics(fig, fullfile('D:\GitHub\KF-GINS-Matlab\惯导实验数据\New Folder\', ...
    'F-3.png'), 'Resolution', 600);
outputExcelName = ['D:\GitHub\KF-GINS-Matlab\惯导实验数据\New Folder\','导航系统径向误差统计报告.xlsx'];
writecell(finalExcelData, outputExcelName);
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