%% 实测数据的长基线式位置约束实验
% 由真值构造低频带噪位置量测，对比前向 ESKF、一次 RTS 和二次 RTS。
clear; clc; close all;
script_dir = fileparts(mfilename('fullpath'));
paper_root = fileparts(fileparts(script_dir));
addpath(paper_root);
paper_paths = setup_paper_study();
% 参数初始化
param = Param();
for input_id = 5:6
    in_dir = fullfile(paper_paths.external_experiment_root, ...
        ['input', num2str(input_id)]);
    cfg = config_1(in_dir);
    cfg.outputfolder = fullfile(paper_paths.output_experiment, ...
        ['dataset', num2str(input_id - 4)]);
    if ~exist(cfg.outputfolder, 'dir'), mkdir(cfg.outputfolder); end
    
    %% 定义标准差和其他设置
    range_std_m = 6;
    depth_std_m = 0.4;
    rng(1);
    smoothing_method = "RTS";
    enable_smoothing = 1;
    enable_feedback = 1;
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
        range_stride = 360; % 距离文件为 1 Hz，约 6 min 一次量测
        for i = 1:3
            range{i} = range{i}(range_stride:range_stride:end, :);
        end

        % 初始化合并后的范围数据
        rangedata = zeros(size(range{1}));
        seq = [1, 2, 3];

        for i = 1:3
            rangedata(i:3:end, :) = range{seq(i)}(i:3:end, :);
        end

        % 导入高度数据
        truth = importdata(cfg.truthpath);
        height = truth(:, [2,5]);

        % 类似长基线定位的位置量测：[纬度(rad), 经度(rad), 高度(m), 时间(s)]
        lbl_data = [truth(:, 3:4) * param.D2R, ...
            -truth(:, 5), truth(:, 2)];

        %% 设置文件保存路径
        if enable_feedback == 0
            navpath = fullfile(cfg.outputfolder, 'PureIns.nav');
        else
            navpath = fullfile(cfg.outputfolder, 'ESKF-LBL.nav');
        end
        navfp = fopen(navpath, 'wt');
        if navfp < 0, error('无法创建导航结果：%s', navpath); end

        % 根据设置是否启用平滑
        if enable_smoothing == 1
            navpath1 = fullfile(cfg.outputfolder, sprintf('Single-stage %s-LBL.nav', smoothing_method));
            navfp1 = fopen(navpath1, 'wt');
            % 单次平滑结果
            navpath2 = fullfile(cfg.outputfolder, sprintf('Proposed two-stage %s-LBL.nav', smoothing_method));
            navfp2 = fopen(navpath2, 'wt');
            if navfp1 < 0 || navfp2 < 0
                error('无法创建 RTS-LBL 导航结果：%s', cfg.outputfolder);
            end
        end
        file_cleanup = onCleanup(@() close_nav_files( ...
            [navfp, navfp1, navfp2]));


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
        % 添加随机噪声到范围数据
        rangedata(:, 3) = rangedata(:, 3) + normrnd(0, range_std_m, size(rangedata(:, 3)));

        % 筛选高度数据并添加噪声
        height = height(height(:, 1) >= cfg.starttime & height(:, 1) <= cfg.endtime, :);
        height(:, 2) = height(:, 2) + normrnd(0, depth_std_m, size(height(:, 2)));
        
        %
        lbl_data = lbl_data(lbl_data(:, 4) >= cfg.starttime & ...
            lbl_data(:, 4) <= cfg.endtime, :);
        lbl_data(:, 1) = lbl_data(:, 1) + ...
            normrnd(0, 2 / param.WGS84_RA, size(lbl_data, 1), 1);
        lbl_data(:, 2) = lbl_data(:, 2) + ...
            normrnd(0, 2 / param.WGS84_RA, size(lbl_data, 1), 1);
        lbl_data(:, 3) = lbl_data(:, 3) + ...
            normrnd(0, depth_std_m, size(lbl_data, 1), 1);

        % 真值文件为 100 Hz，与 1 Hz 距离量测保持相同的 360 s 间隔。
        lbl_stride = range_stride * 100;
        lbl_data = lbl_data(lbl_stride:lbl_stride:end, :);
        fprintf('\n[长基线约束] 开始处理 Dataset %d。\n', input_id - 4);
        lastprecent = 0;
        %% initialization
        [kf, navstate] = myInitialize_15state(cfg);
        laststate = navstate;
        kf.rangstd = range_std_m;
        kf.depthstd = depth_std_m;
        % data index preprocess
        lastimu = imudata(1, :)';
        thisimu = imudata(1, :)';
        imudt = thisimu(1, 1) - lastimu(1, 1);

        rangeindex = 1;
        while rangedata(rangeindex, 1) < thisimu(1, 1)
            rangeindex = rangeindex + 1;
        end

        max_buffer_size = 54000;

        state_buffer = zeros(max_buffer_size, 10);
        Xk_k1propa   = zeros(max_buffer_size, 15);
        Pk_k1propa   = zeros(max_buffer_size, 225);
        Pk_propa = zeros(max_buffer_size, 225);
        PHI   = zeros(max_buffer_size, 225); % 建议变量名区分开


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
                % kf = myRangeUpdate(navstate, rangedata(rangeindex,:), height(imuindex,:), kf);
                kf = myLBLUpdate(navstate, lbl_data(rangeindex, :)', kf);
                rangeindex = rangeindex + 1;

                if enable_smoothing == 1
                    if buf_idx > 1
                        valid_len = buf_idx - 1;
                        xk_final = kf.x;
                        sub_state_buffer = state_buffer(1:valid_len, :);
                        sub_Xk_k1propa = Xk_k1propa(1:valid_len, :);
                        sub_Pk_k1propa = Pk_k1propa(1:valid_len, :);
                        sub_Pk_propa = Pk_propa(1:valid_len, :);
                        sub_PHI = PHI(1:valid_len, :);

                        % 调用 RTS 平滑函数
                        [nav_matrix, bridge_error, rtsstate_buffer] = ...
                            perform_unified_smoothing(sub_state_buffer, xk_final, ...
                            param, rangeindex, smoothing_method, 'rad', ...
                            sub_Pk_propa, sub_Pk_k1propa, sub_PHI);
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
                            [nav_matrix_prev_resmoothed, bridge_error, ~] = ...
                                perform_unified_smoothing(prev_state_buffer, bridge_error, ...
                                param, prev_rangeindex, smoothing_method, 'rad', ...
                                prev_Pk_propa, prev_Pk_k1propa, prev_PHI);
                            fprintf(navfp2, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix_prev_resmoothed);
                            % 更新缓存：把当前的 7-14min 变成新的“上一段”，等待 14-21min 来救赎它
                            prev_state_buffer = rtsstate_buffer;
                            prev_Pk_propa     = sub_Pk_propa;
                            prev_Pk_k1propa   = sub_Pk_k1propa;
                            prev_PHI          = sub_PHI;
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
                if enable_feedback==1
                    [kf, navstate] = myErrorFeedback_range(kf, navstate);
                    % [kf, navstate] = myErrorFeedback_15state(kf, navstate);
                end
                laststate = navstate;

                % 惯导推算
                imudt = thisimu(1, 1) - lastimu(1, 1);
                navstate = InsMech(laststate, lastimu, thisimu);
                kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
            elseif (lastimu(1, 1) < rangedata(rangeindex, 1) && thisimu(1, 1) > rangedata(rangeindex, 1))&& cfg.userange==1
                % 先传播到精确的低频定位时刻，再进行位置量测更新。
                [firstimu, secondimu] = interpolate(lastimu, thisimu, ...
                    rangedata(rangeindex, 1));
                imudt = firstimu(1) - lastimu(1);
                navstate = InsMech(navstate, lastimu, firstimu);
                kf = myInsPropagate_15state(navstate, firstimu, imudt, kf);

                kf = myLBLUpdate(navstate, lbl_data(rangeindex, :)', kf);
                if enable_feedback == 1
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
            else
                %% only do propagation
                % INS mechanization
                navstate = InsMech(laststate, lastimu, thisimu);
                % navstate.pos(3) = heightdata(imuindex,2);

                % 调用高度卡尔曼量测更新
                kf = myHeightUpdate(navstate, height(imuindex, :), kf);

                % 反馈修正惯导状态 (仅修正天向)
                navstate.pos(3) = navstate.pos(3) - kf.x(3);
                navstate.vel(3) = navstate.vel(3) - kf.x(6);

                % 💡 反馈后，将整个误差状态量清零（代表误差已融入主状态）
                kf.x = zeros(size(kf.x));

                if enable_smoothing == 1
                    Pk_propa(buf_idx,:) = kf.P(:)';
                end

                % error propagation
                kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);

                if enable_smoothing == 1
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
            % print processing information
            if (imuindex / size(imudata, 1) - lastprecent > 0.20)
                disp("processing " + num2str(floor(imuindex * 100 / size(imudata, 1))) + " %!");
                lastprecent = imuindex / size(imudata, 1);
            end
        end
        %%
        % 最后一段没有后续量测，用一次 RTS 结果补齐二次 RTS 输出。
        if enable_smoothing && ~isempty(prev_state_buffer)
            % 最后一段缺少未来协方差桥接信息，原样保留一次 RTS 结果。
            [final_nav_matrix, ~, ~] = perform_unified_smoothing( ...
                rtsstate_buffer, zeros(15, 1), param, rangeindex, ...
                'Linear', 'rad', [], [], []);
            fprintf(navfp2, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', final_nav_matrix);
        end
        fclose(navfp);
        fclose(navfp1);
        fclose(navfp2);
        clear file_cleanup;
        fprintf('  Dataset %d 完成，耗时 %.2f s。\n', input_id - 4, toc);
end

fprintf('\n两组实测数据的长基线式位置约束实验完成。\n');

