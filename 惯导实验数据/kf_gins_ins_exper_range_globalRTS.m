clear;
param = Param();
cfg = ProcessConfig_exper();
%%
rangstd = 2;
depthstd= 0.4;
rng(1)
tic

backwardIsOpen = 0;
% smoothWay = 'Linear';
smoothWay = 'RTS';
SmoothIsOpen = 1;

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

navpath1 = [cfg.outputfolder, '/NavResult-RANGE-GlobalDistribute.nav'];
navfp1 = fopen(navpath1, 'wt');
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

height = height(height(:, 1) >= cfg.starttime, :);
height = height(height(:, 1) <= cfg.endtime, :);
height(:,2) = height(:,2) + normrnd(1,rangstd,size(height(:,2)));

heightdata = heightdata(heightdata(:, 1) >= cfg.starttime, :);
heightdata = heightdata(heightdata(:, 1) <= cfg.endtime, :);
heightdata_true = heightdata(:,2) ;
heightdata(:,2) = heightdata(:,2) + normrnd(1,rangstd,size(heightdata(:,2)));

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
disp("Start GNSS/RANGE Processing!");
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

% --- 1. 动态预分配全局内存 (根据数据真实长度) ---
MAX_BUFFER_SIZE = size(imudata, 1) + 100; 
state_buffer = zeros(MAX_BUFFER_SIZE, 10);
Pk_corr_buf  = zeros(MAX_BUFFER_SIZE, 225); % 记录 P_{k|k}
Pk_pred_buf  = zeros(MAX_BUFFER_SIZE, 225); % 记录 P_{k+1|k}
PHI_buf      = zeros(MAX_BUFFER_SIZE, 225); % 记录 Phi
feedback_buf = zeros(MAX_BUFFER_SIZE, 15);
buf_idx = 1;

disp("Start Forward EKF Processing...");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MAIN PROCESSING PROCEDURE!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for imuindex = 2:size(imudata, 1)-1
    lastimu = thisimu;
    laststate = navstate;
    thisimu = imudata(imuindex, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);
    
    while (rangeindex <= size(rangedata, 1) && rangedata(rangeindex, 1) < lastimu(1, 1))
        rangeindex = rangeindex + 1;
    end
    if (rangeindex > size(rangedata, 1)), disp('range file END!'); break; end

    %% --- 分支 1：正好对齐的测距更新 ---
    if lastimu(1, 1) == rangedata(rangeindex, 1)
        Rangedata = rangedata(rangeindex,:);
        depthdata = heightdata(rangeindex,:);
        
        kf = myRangeUpdate(navstate, Rangedata, depthdata, kf);

        feedback_buf(rangeindex, :) = kf.x(:)';

        [kf, navstate] = myErrorFeedback_range(kf, navstate);
        rangeindex = rangeindex + 1;
        
        % 【记录 1】量测更新后的 P_{k|k}
        Pk_corr_buf(buf_idx,:) = kf.P(:)';
        
        laststate = navstate;
        imudt = thisimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, thisimu);
        kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
        
    %% --- 分支 2：插值测距更新 ---
    elseif (lastimu(1, 1) < rangedata(rangeindex, 1) && thisimu(1, 1) > rangedata(rangeindex, 1))
        [firstimu, secondimu] = interpolate(lastimu, thisimu, rangedata(rangeindex, 1));
        imudt_first = firstimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, firstimu);
        kf = myInsPropagate_15state(navstate, firstimu, imudt_first, kf);
        
        Rangedata = rangedata(rangeindex,:);
        depthdata = heightdata(rangeindex,:);
        kf = myRangeUpdate(navstate, Rangedata, depthdata, kf);
        [kf, navstate] = myErrorFeedback_range(kf, navstate);
        rangeindex = rangeindex + 1;
        
        % 【记录 1】量测更新后的 P_{k|k}
        Pk_corr_buf(buf_idx,:) = kf.P(:)';
        
        laststate = navstate;
        lastimu = firstimu;
        imudt_second = secondimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, secondimu);
        kf = myInsPropagate_15state(navstate, secondimu, imudt_second, kf);
        
    %% --- 分支 3：纯推算 (带高频高度更新) ---
    else
        navstate = InsMech(laststate, lastimu, thisimu);
        kf = myHeightUpdate(navstate, height(imuindex,:), kf);
        navstate.pos(3) = navstate.pos(3) - kf.x(3);
        navstate.vel(3) = navstate.vel(3) - kf.x(6);
        kf.x = zeros(size(kf.x));
        
        % 【记录 1】量测更新后的 P_{k|k}
        Pk_corr_buf(buf_idx,:) = kf.P(:)';
        
        kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
    end
    
    %% --- 【统一记录 2】保存推算状态、预测 P 阵和转移矩阵 ---
    nav = [navstate.time; navstate.pos; navstate.vel; navstate.att];
    state_buffer(buf_idx,:) = nav';
    Pk_pred_buf(buf_idx,:)  = kf.P(:)';
    PHI_buf(buf_idx,:)      = kf.phi(:)';
    
    % (可选) 写入正向滤波文件 NavResult-RANGE-1.nav
    nav_out = zeros(11, 1);
    nav_out(2, 1) = navstate.time;
    nav_out(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
    nav_out(6:8, 1) = navstate.vel;
    nav_out(9:11, 1) = navstate.att * param.R2D;
    fprintf(navfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_out);
    
    % 游标推进
    buf_idx = buf_idx + 1;
end
disp("Forward Processing Done!");

%% =======================================================
%% 2. 终极奥义：一次性全局 RTS 平滑 (放在 for 循环结束后)
%% =======================================================
if SmoothIsOpen == 1 && strcmp(smoothWay,'RTS')
    disp("Start Global RTS Smoothing...");
    
    % 截取有效数据段
    valid_len = buf_idx - 1;
    valid_state   = state_buffer(1:valid_len, :);
    valid_Pk_corr = Pk_corr_buf(1:valid_len, :);
    valid_Pk_pred = Pk_pred_buf(1:valid_len, :);
    valid_PHI     = PHI_buf(1:valid_len, :);
    valid_feedback = feedback_buf(1:valid_len, :); % 新增
    
    % 调用带反馈注入的全局平滑
    nav_matrix_global = perform_RTS_smoothing_g(valid_state, valid_Pk_corr, valid_Pk_pred, valid_PHI, valid_feedback, param);
    
    % 一口气写入平滑结果文件
    fprintf(navfp1, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix_global);
    disp("Global RTS Smoothing Finished!");
end

fclose all;
toc
%%
% plot_result(navpath)
calc_error(navpath,cfg.truthpath);

calc_error(navpath1,cfg.truthpath);
%%
calc_radial_error(cfg.truthpath,navpath,'output\NavResult-RANGE-RTS1.nav');
exportgraphics(gca, fullfile('D:\GitHub\KF-GINS-Matlab\惯导实验数据\New Folder\', ...
    'F-2.png'), 'Resolution', 600);
%%
calc_radial_error(cfg.truthpath,navpath,navpath1,'output\NavResult-RANGE-RTSDistribute.nav');
exportgraphics(gca, fullfile('D:\GitHub\KF-GINS-Matlab\惯导实验数据\New Folder\', ...
    'F-3.png'), 'Resolution', 600);