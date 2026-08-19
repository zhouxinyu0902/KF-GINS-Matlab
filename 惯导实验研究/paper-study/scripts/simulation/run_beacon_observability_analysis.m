clear;
%% 三信标轮换/固定方案的导航与可观测度分析
paper_paths = setup_paper_study();
param = Param();

in_dir = paper_paths.simulation_dataset1;
cfg = config_simu(in_dir);
cfg.outputfolder = fullfile(paper_paths.output_simulation_dataset1, ...
    'alt-B1-B2-B3');
if ~isfolder(cfg.outputfolder)
    mkdir(cfg.outputfolder);
end
artifact_dir = paper_artifact_dir(cfg.outputfolder);
%% 定义标准差和其他设置
range_std_m = 6;
depth_std_m = 0.4;
rng(1);
smoothing_methods = "RTS";
measurement_modes = ["Alternating", "Fixed-B1", "Fixed-B2", "Fixed-B3"];
enable_smoothing = 0;
enable_feedback = 1;
tic; % 启动计时器
%% 导入数据
for smoothing_method = smoothing_methods
    for mode = measurement_modes
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
        range_stride = 360; % 距离文件为1 Hz，360行约为6 min
        for i = 1:3
            range{i} = range{i}(range_stride:range_stride:end, :);
        end

        % OBSERVABILITY: catalog of candidate beacon positions
        % [latitude(rad), longitude(rad), height(m)].
        beacon_positions = zeros(numel(range), 3);
        for i = 1:numel(range)
            beacon_positions(i, :) = range{i}(1, 4:6);
        end

        % 初始化合并后的范围数据
        rangedata = zeros(size(range{1}));
        seq = [1, 2, 3];

        switch mode
            case 'Alternating'
                for i = 1:3
                    rangedata(i:3:end, :) = range{seq(i)}(i:3:end, :);
                end
            case 'Fixed-B1'
                rangedata = range{1};
            case 'Fixed-B2'
                rangedata = range{2};
            case 'Fixed-B3'
                rangedata = range{3};
        end


        % 导入高度数据
        truth = importdata(cfg.truthpath);
        height = truth(:, [2,5]);


        %% 设置文件保存路径
        navpath = fullfile(cfg.outputfolder, sprintf('ES-EKF-%s.nav', mode));
        navfp = fopen(navpath, 'wt');
        if navfp < 0
            error('无法创建可观测度导航结果：%s', navpath);
        end

        % 根据设置是否启用平滑
        if enable_smoothing == 1
            % 一次分段 RTS 结果
            navpath1 = fullfile(cfg.outputfolder, ...
                sprintf('Single-stage %s-%s.nav', smoothing_method, mode));
            navfp1 = fopen(navpath1, 'wt');
            % 二次跨区间 RTS 结果
            navpath2 = fullfile(cfg.outputfolder, ...
                sprintf('Proposed two-stage %s-%s.nav', ...
                smoothing_method, mode));
            navfp2 = fopen(navpath2, 'wt');
            if navfp1 < 0 || navfp2 < 0
                error('无法创建可观测度 RTS 结果：%s', cfg.outputfolder);
            end
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

        % 添加随机噪声到范围数据
        rangedata(:, 3) = rangedata(:, 3) + normrnd(0, range_std_m, size(rangedata(:, 3)));

        % 筛选高度数据并添加噪声
        height = height(height(:, 1) >= cfg.starttime & height(:, 1) <= cfg.endtime, :);
        height(:, 2) = height(:, 2) + normrnd(0, depth_std_m, size(height(:, 2)));

        %% 初始化滤波器及可观测度日志
        disp("Start INS/RANGE Processing!");
        lastprecent = 0;
        %% initialization
        [kf, navstate] = myInitialize_15state(cfg);
        laststate = navstate;
        kf.rangstd = range_std_m;
        kf.depthstd = depth_std_m;

        % OBSERVABILITY: initialize dimensionless finite-horizon analysis.
        obslog = init_beacon_observability(beacon_positions, kf.P0, range_std_m);
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
                % OBSERVABILITY: log the pre-update state and all candidate beacon
                % Jacobians. The active beacon is inferred from columns 4:6.
                obslog = log_beacon_observation(obslog, ...
                    rangedata(rangeindex, 1), navstate.pos, ...
                    rangedata(rangeindex, 4:6)');

                % 测量更新
                kf = myRangeUpdate(navstate, rangedata(rangeindex,:), height(imuindex,:), kf);
                rangeindex = rangeindex + 1;

                if enable_smoothing == 1
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
                            param, rangeindex, smoothing_method, 'rad', Pk_propa, Pk_k1propa, PHI);
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
                                param, prev_rangeindex, smoothing_method, 'rad', prev_Pk_propa, prev_Pk_k1propa, prev_PHI);
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
                if enable_feedback==1
                    [kf, navstate] = myErrorFeedback_range(kf, navstate);
                    % [kf, navstate] = myErrorFeedback_15state(kf, navstate);
                end
                laststate = navstate;

                % 惯导推算
                imudt = thisimu(1, 1) - lastimu(1, 1);
                navstate = InsMech(laststate, lastimu, thisimu);
                kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
                obslog = propagate_beacon_observability(obslog, kf.phi);
            elseif lastimu(1) < rangedata(rangeindex, 1) && ...
                    thisimu(1) > rangedata(rangeindex, 1) && cfg.userange == 1
                % 先将惯导状态推算到精确量测时刻，再记录几何信息和更新。
                [firstimu, secondimu] = interpolate(lastimu, thisimu, ...
                    rangedata(rangeindex, 1));
                imudt = firstimu(1) - lastimu(1);
                navstate = InsMech(laststate, lastimu, firstimu);
                kf = myInsPropagate_15state(navstate, firstimu, imudt, kf);
                obslog = propagate_beacon_observability(obslog, kf.phi);
                obslog = log_beacon_observation(obslog, ...
                    rangedata(rangeindex, 1), navstate.pos, ...
                    rangedata(rangeindex, 4:6)');

                kf = myRangeUpdate(navstate, rangedata(rangeindex,:), height(imuindex,:), kf);
                if enable_feedback == 1
                    [kf, navstate] = myErrorFeedback_range(kf, navstate);
                end
                rangeindex = rangeindex + 1;
                laststate = navstate;
                lastimu = firstimu;

                % 完成量测时刻至当前 IMU 历元的剩余推算
                imudt = secondimu(1) - lastimu(1);
                navstate = InsMech(laststate, lastimu, secondimu);
                kf = myInsPropagate_15state(navstate, secondimu, imudt, kf);
                obslog = propagate_beacon_observability(obslog, kf.phi);
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
                % kf.x = zeros(size(kf.x));
                kf.x(3) = 0;kf.x(6) = 0;

                if enable_smoothing == 1
                    Pk_propa(buf_idx,:) = kf.P(:)';
                end

                % error propagation
                kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
                obslog = propagate_beacon_observability(obslog, kf.phi);

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
        if enable_smoothing && ~isempty(prev_state_buffer)
            % 最后一段没有未来信息了，我们只能用它自己第一次平滑的结果
            [final_nav_matrix, ~, ~] = perform_unified_smoothing( ...
                rtsstate_buffer, zeros(15, 1), param, rangeindex, ...
                smoothing_method, 'rad', [], [], []); %#ok<UNRCH>
            fprintf(navfp2, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', final_nav_matrix);
        end

        if mode == "Alternating"
            num_obs_events = numel(obslog.events);
            window_measurements = min(9, 3 * floor(num_obs_events / 3));
            if window_measurements < 3
                error('有效测距事件不足，无法进行可观测度分析。');
            end
            analyze_beacon_observability(obslog, window_measurements, ...
                artifact_dir);
            save(fullfile(artifact_dir, 'observ.mat'), ...
                'obslog', 'window_measurements');
        end

        fclose(navfp);
        if enable_smoothing
            fclose(navfp1); %#ok<UNRCH>
            fclose(navfp2);
        end
        toc
    end
end
%% 固定信标与轮换信标的导航误差对比
nav_files = {
    fullfile(cfg.outputfolder, 'ES-EKF-Alternating.nav')
    fullfile(cfg.outputfolder, 'ES-EKF-Fixed-B1.nav')
    fullfile(cfg.outputfolder, 'ES-EKF-Fixed-B2.nav')
    fullfile(cfg.outputfolder, 'ES-EKF-Fixed-B3.nav')
    };
method_names = ["轮换信标", "固定信标B1", "固定信标B2", "固定信标B3"];

[comparison_figure, radial_statistics] = calc_radial_error_gjb( ...
    cfg.truthpath, nav_files{:});
legend(method_names, 'Location', 'best');
xlabel('时间 (s)');
ylabel('水平径向误差 (m)');

comparison_prefix = fullfile(artifact_dir, ...
    'alternating-vs-fixed-beacons');
exportgraphics(comparison_figure, comparison_prefix + ".png", ...
    'Resolution', 600);
savefig(comparison_figure, comparison_prefix + ".fig");
writecell(radial_statistics, comparison_prefix + ".xlsx", ...
    'Sheet', 'RMSE');

fprintf('可观测度及固定/轮换信标对比结果：%s\n', artifact_dir);

