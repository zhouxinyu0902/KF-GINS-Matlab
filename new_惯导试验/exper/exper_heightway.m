clear;
% 参数初始化
param = Param();
ID = 6;
clc
in_dir = ['F:/2_Data/惯导试验/实验数据/All_data/input',num2str(ID)];
cfg = config_1(in_dir);
cfg.outputfolder =['D:\Github\KF-GINS-Matlab\new_惯导试验\output/output',num2str(ID),'_height'];
mkdir(cfg.outputfolder);
%% 定义标准差和其他设置
rngstd = 6;
depthstd = 0.4;
smoothWay = 'RTS'; % 平滑方式，选择RTS或线性
SmoothIsOpen = 0;
feedback = 1;
tic; % 启动计时器
for height_updateway = [0,1,2]% 1为传统赋值方式，2为高度更新方式
    rng(1);
    %% 导入数据
    % for smoothWay=["RTS","Linear"]
    for smoothWay = "RTS"
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
        truth = importdata(cfg.truthpath);
        height = truth(:, [2,5]);

        % % 人为加入高度异常量测值
        % addHeightOutlier = false;
        % heightOutlierStep = 2000;
        % heightOutlierStart = 1000;
        % heightOutlierBias = 5;   % m

        caseLabel = "normal";

        % if addHeightOutlier
        %     outlierIndex = heightOutlierStart:heightOutlierStep:size(height, 1);
        %     height(outlierIndex, 2) = height(outlierIndex, 2) + heightOutlierBias;
        %     caseLabel = "abnormal";
        % end

        %% 设置文件保存路径
        switch height_updateway
            case 0
                methodLabel = "none";
            case 1
                methodLabel = "assign";
            case 2
                methodLabel = "measUpdate";
            otherwise
                error("Unsupported height_updateway: %d", height_updateway);
        end

        navName = sprintf("Origin-rad-%s-%s.nav", methodLabel, caseLabel);
        navpath = fullfile(cfg.outputfolder, navName);
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

        % writematrix(rangedata, '惯导实验数据\input_pre\rangedata_noised.txt', 'Delimiter', 'space'); % 空格
        % writematrix(height, '惯导实验数据\input_pre\height_noised.txt', 'Delimiter', 'space'); % 空格
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
        kf.rangstd = rngstd;
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
                end
                laststate = navstate;

                % 惯导推算
                imudt = thisimu(1, 1) - lastimu(1, 1);
                navstate = InsMech(laststate, lastimu, thisimu);
                kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
            else
                %% only do propagation
                % INS mechanization
                navstate = InsMech(laststate, lastimu, thisimu);

                if height_updateway == 1
                    navstate.pos(3) = height(imuindex,2);
                elseif height_updateway == 2
                    % 调用高度卡尔曼量测更新
                    kf = myHeightUpdate(navstate, height(imuindex, :), kf);

                    % 反馈修正惯导状态 (仅修正天向)
                    navstate.pos(3) = navstate.pos(3) - kf.x(3);
                    navstate.vel(3) = navstate.vel(3) - kf.x(6);

                    % 💡 反馈后，将整个误差状态量清零（代表误差已融入主状态）
                    kf.x = zeros(size(kf.x));
                end
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
            perform_unified_smoothing(rtsstate_buffer, zeros(15,1), ...
                param, rangeindex, 'Linear', 'rad', [], [], []);
            fprintf(navfp1, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix);
        end
        fclose all;
        toc
        [fig,finalExcelData] = calc_radial_error_gjb(cfg.truthpath,navpath);
    end
end
%% 对比不同高度更新方式
navFiles = {
    fullfile(cfg.outputfolder, 'Origin-rad-none-normal.nav')
    fullfile(cfg.outputfolder, 'Origin-rad-assign-normal.nav')
    fullfile(cfg.outputfolder, 'Origin-rad-measUpdate-normal.nav')

    };

[fig1,finalExcelData1] = calc_radial_error_gjb(cfg.truthpath,navFiles{:});
% 保存对比图片和表格
savePrefix = fullfile(cfg.outputfolder, "compare-none-assign-measUpdate");
% 保存图片
exportgraphics(fig1, savePrefix + ".png", "Resolution", 600);
savefig(fig1, savePrefix + ".fig");
% 保存表格
writecell(finalExcelData1, savePrefix + ".xlsx", "Sheet", "RMSE");

fprintf("对比图片已保存：%s\n", savePrefix + ".png");
fprintf("对比表格已保存：%s\n", savePrefix + ".xlsx");
truthPath = cfg.truthpath;
results = compare_height_vertical_rmse( ...
    truthPath, navFiles, ...
    'MethodNames', ["无","直接赋值", "高度量测更新"], ...
    'OutputFile', fullfile(cfg.outputfolder, 'height_vd_rmse.csv'), ...
    'FigureFile', fullfile(cfg.outputfolder, 'height_vd_error.png'));
%%
navFiles = {
    fullfile(cfg.outputfolder, 'Origin-rad-none-abnormal.nav')
    fullfile(cfg.outputfolder, 'Origin-rad-assign-abnormal.nav')
    fullfile(cfg.outputfolder, 'Origin-rad-measUpdate-abnormal.nav')

    };
[fig1,finalExcelData1] = calc_radial_error_gjb(cfg.truthpath,navFiles{:});
% 保存对比图片和表格
savePrefix = fullfile(cfg.outputfolder, "compare-none-assign-measUpdate-abnormal");
% 保存图片
exportgraphics(fig1, savePrefix + ".png", "Resolution", 600);
savefig(fig1, savePrefix + ".fig");
% 保存表格
writecell(finalExcelData1, savePrefix + ".xlsx", "Sheet", "RMSE");

fprintf("对比图片已保存：%s\n", savePrefix + ".png");
fprintf("对比表格已保存：%s\n", savePrefix + ".xlsx");
%

truthPath = cfg.truthpath;

results = compare_height_vertical_rmse( ...
    truthPath, navFiles, ...
    'MethodNames', ["无","直接赋值", "高度量测更新"], ...
    'OutputFile', fullfile(cfg.outputfolder, 'height_vd_rmse.csv'), ...
    'FigureFile', fullfile(cfg.outputfolder, 'height_vd_error.png'));