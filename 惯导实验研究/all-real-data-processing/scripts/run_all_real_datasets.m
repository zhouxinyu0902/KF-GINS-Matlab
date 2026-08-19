%% =========================================================================
% 八组实测数据统一处理入口
% 合并原 all_overall、pure-ins、rad 和 m 四套入口，避免重复维护。
% =========================================================================
clear; clc; close all;
script_dir = fileparts(mfilename('fullpath'));
topic_root = fileparts(script_dir);
addpath(topic_root);
topic_paths = setup_all_real_data_processing();

% 参数初始化
param = Param();

% 运行设置：此处是日常使用时主要需要修改的区域
input_ids = 6;
unit_types = "rad";                % 可扩展为 ["rad", "m"]
solution_modes = ["pure-ins", "range-aided"];
range_std_m = 6;
depth_std_m = 0.4;
smoothing_method = "RTS";
range_stride = 420;                % 距离文件为 1 Hz，对应 7 min
end_time_override = [];            % 空值表示处理到每组数据末尾
export_evaluation = true;

for input_id = input_ids
    fprintf('\n======================================================\n');
    fprintf('开始处理第 %d 组数据...\n', input_id);
    fprintf('======================================================\n');

    in_dir = fullfile(topic_paths.external_input_root, ...
        ['input', num2str(input_id)]);
    derived_dir = fullfile(topic_paths.derived_input, ...
        ['dataset', num2str(input_id)]);
    nav_out_dir = fullfile(topic_paths.navigation_results, ...
        ['dataset', num2str(input_id)]);
    artifact_out_dir = fullfile(topic_paths.figures_tables, ...
        ['dataset', num2str(input_id)]);
    if ~exist(nav_out_dir, 'dir'), mkdir(nav_out_dir); end
    if ~exist(artifact_out_dir, 'dir'), mkdir(artifact_out_dir); end

    for unit_type = unit_types
      for solution_mode = solution_modes
        enable_feedback = solution_mode == "range-aided";
        enable_smoothing = enable_feedback;
        rng(1); % 每次循环重置随机数种子，保证添加的噪声完全一致
        tic;

        fprintf('  --> [%s] 单位，[%s] 模式...\n', ...
            unit_type, solution_mode);

        %% 1. 动态绑定配置与底层函数 (修复了原代码中的遗漏Bug)
        if unit_type == "rad"
            cfg = create_real_dataset_config(in_dir, derived_dir, ...
                unit_type, end_time_override);
            fn_RangeUpdate   = @myRangeUpdate;
            fn_HeightUpdate  = @myHeightUpdate;
            fn_ErrorFeedback = @myErrorFeedback_range;
            fn_InsPropagate  = @myInsPropagate_15state;
            z_err_sign       = -1; % rad 模式下: pos(3) - kf.x(3)
        else
            cfg = create_real_dataset_config(in_dir, derived_dir, ...
                unit_type, end_time_override);
            fn_RangeUpdate   = @myRangeUpdate_m;
            fn_HeightUpdate  = @myHeightUpdate_m;
            fn_ErrorFeedback = @myErrorFeedback_range_m;
            fn_InsPropagate  = @myInsPropagate_15state_m;
            z_err_sign       = 1;  % m 模式下: pos(3) + kf.x(3)
        end

        cfg.outputfolder = nav_out_dir;
        cfg.userange = 1;

        %% 2. 导入与构造数据
        imudata = importdata(cfg.imufilepath);
        imustarttime = imudata(1, 1);
        imuendtime = imudata(end, 1);

        rangedata1 = importdata(cfg.rangefile1path);
        rangedata2 = importdata(cfg.rangefile2path);
        rangedata3 = importdata(cfg.rangefile3path);
        range = {rangedata1, rangedata2, rangedata3};

        % 三个固定信标按 1-2-3 次序轮换
        for i = 1:3
            range{i} = range{i}(range_stride:range_stride:end, :);
        end
        rangedata = zeros(size(range{1}));
        seq = [1, 2, 3];
        for i = 1:3
            rangedata(i:3:end, :) = range{seq(i)}(i:3:end, :);
        end

        % 导入高度数据
        truth = importdata(cfg.truthpath);
        height = truth(:, [2,5]);

        %% 3. 时间调整与加噪
        if cfg.starttime < imustarttime, cfg.starttime = imustarttime; end
        if cfg.endtime > imuendtime, cfg.endtime = imuendtime; end

        imudata = imudata(imudata(:, 1) >= cfg.starttime & imudata(:, 1) <= cfg.endtime, :);
        rangedata = rangedata(rangedata(:, 1) >= cfg.starttime & rangedata(:, 1) <= cfg.endtime, :);
        rangedata(:, 3) = rangedata(:, 3) + normrnd(0, range_std_m, size(rangedata(:, 3)));

        height = height(height(:, 1) >= cfg.starttime & height(:, 1) <= cfg.endtime, :);
        height(:, 2) = height(:, 2) + normrnd(0, depth_std_m, size(height(:, 2)));

        %% 4. 设置文件保存路径
        if ~enable_feedback
            navpath = fullfile(cfg.outputfolder, ...
                sprintf('PureIns-%s.nav', unit_type));
        else
            navpath = fullfile(cfg.outputfolder, ...
                sprintf('Origin-%s.nav', unit_type));
        end
        navfp = fopen(navpath, 'wt');
        if navfp < 0
            error('无法创建导航结果：%s', navpath);
        end
        navfp1 = -1;
        navfp2 = -1;
        if enable_smoothing
            navpath1 = fullfile(cfg.outputfolder, ...
                sprintf('%s-SingleSmooth-%s.nav', smoothing_method, unit_type));
            navfp1 = fopen(navpath1, 'wt');
            navpath2 = fullfile(cfg.outputfolder, ...
                sprintf('%s-DoubleSmooth-%s.nav', smoothing_method, unit_type));
            navfp2 = fopen(navpath2, 'wt');
            if navfp1 < 0 || navfp2 < 0
                error('无法创建 RTS 结果：%s', cfg.outputfolder);
            end
        end
        file_cleanup = onCleanup(@() close_nav_files( ...
            [navfp, navfp1, navfp2]));

        %% 5. 初始化
        [kf, navstate] = myInitialize_15state(cfg);
        laststate = navstate;
        kf.rangstd = range_std_m;
        kf.depthstd = depth_std_m;

        lastimu = imudata(1, :)';
        thisimu = imudata(1, :)';
        imudt = thisimu(1, 1) - lastimu(1, 1);

        rangeindex = 1;
        while rangedata(rangeindex, 1) < thisimu(1, 1)
            rangeindex = rangeindex + 1;
        end

        % 缓存定义
        max_buffer_size = 54000;
        state_buffer = zeros(max_buffer_size, 10);
        Xk_k1propa   = zeros(max_buffer_size, 15);
        Pk_k1propa   = zeros(max_buffer_size, 225);
        Pk_propa     = zeros(max_buffer_size, 225);
        PHI          = zeros(max_buffer_size, 225);

        prev_state_buffer = []; prev_Pk_propa = [];
        prev_Pk_k1propa = []; prev_PHI = []; prev_rangeindex = 0;

        buf_idx = 1;
        lastprecent = 0;
        bridge_err = zeros(length(rangedata),15);

        %% 6. 主循环
        for imuindex = 2:size(imudata, 1)
            lastimu = thisimu;
            laststate = navstate;
            thisimu = imudata(imuindex, :)';
            imudt = thisimu(1, 1) - lastimu(1, 1);

            while (rangeindex <= size(rangedata, 1) && rangedata(rangeindex, 1) < lastimu(1, 1))
                rangeindex = rangeindex + 1;
            end
            if (rangeindex > size(rangedata, 1))
                disp('    [Info] Range file END!');
                break;
            end

            if lastimu(1, 1) == rangedata(rangeindex, 1) && cfg.userange == 1
                % 测量更新
                kf = fn_RangeUpdate(navstate, rangedata(rangeindex,:), height(imuindex,:), kf);
                rangeindex = rangeindex + 1;

                if enable_smoothing == 1
                    if buf_idx > 1
                        valid_len = buf_idx - 1;
                        sub_state_buffer = state_buffer(1:valid_len, :);

                        xk_final = kf.x;
                        sub_Xk_k1propa = Xk_k1propa(1:valid_len, :);
                        sub_Pk_k1propa = Pk_k1propa(1:valid_len, :);
                        sub_Pk_propa   = Pk_propa(1:valid_len, :);
                        sub_PHI        = PHI(1:valid_len, :);

                        % 调用 RTS 平滑函数
                        [nav_matrix, bridge_error, rtsstate_buffer] = perform_unified_smoothing(...
                            sub_state_buffer, xk_final, param, rangeindex, smoothing_method, char(unit_type), sub_Pk_propa, sub_Pk_k1propa, sub_PHI);
                        fprintf(navfp1, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix);
                        bridge_err(rangeindex,:) = bridge_error;

                        if isempty(prev_state_buffer)
                            prev_state_buffer = rtsstate_buffer; prev_Pk_propa = sub_Pk_propa;
                            prev_Pk_k1propa = sub_Pk_k1propa; prev_PHI = sub_PHI; prev_rangeindex = rangeindex;
                        else
                            [nav_matrix_prev, bridge_error, smoothed_state_buffer] = perform_unified_smoothing(...
                                prev_state_buffer, bridge_error, param, prev_rangeindex, smoothing_method, char(unit_type), prev_Pk_propa, prev_Pk_k1propa, prev_PHI);
                            fprintf(navfp2, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix_prev);

                            prev_state_buffer = rtsstate_buffer; prev_Pk_propa = sub_Pk_propa;
                            prev_Pk_k1propa = sub_Pk_k1propa; prev_PHI = sub_PHI; prev_rangeindex = rangeindex;
                        end
                        buf_idx = 1;
                        state_buffer(:) = 0; Xk_k1propa(:) = 0; Pk_k1propa(:) = 0; Pk_propa(:) = 0; PHI(:) = 0;
                    end
                end

                if enable_feedback == 1
                    [kf, navstate] = fn_ErrorFeedback(kf, navstate);
                end

                % 惯导推算
                imudt = thisimu(1, 1) - lastimu(1, 1);
                navstate = InsMech(navstate, lastimu, thisimu);
                kf = fn_InsPropagate(navstate, thisimu, imudt, kf);

            elseif lastimu(1) < rangedata(rangeindex, 1) && ...
                    thisimu(1) > rangedata(rangeindex, 1) && cfg.userange == 1
                % 将 IMU 增量拆分到精确测距时刻，避免直接使用未定义的
                % firstimu/secondimu，并保证量测更新时刻与状态时刻一致。
                [firstimu, secondimu] = interpolate(lastimu, thisimu, ...
                    rangedata(rangeindex, 1));
                imudt = firstimu(1) - lastimu(1);
                navstate = InsMech(navstate, lastimu, firstimu);
                kf = fn_InsPropagate(navstate, firstimu, imudt, kf);

                kf = fn_RangeUpdate(navstate, rangedata(rangeindex, :), ...
                    height(imuindex, :), kf);
                if enable_feedback
                    [kf, navstate] = fn_ErrorFeedback(kf, navstate);
                end
                rangeindex = rangeindex + 1;

                lastimu = firstimu;
                imudt = secondimu(1) - lastimu(1);
                navstate = InsMech(navstate, lastimu, secondimu);
                kf = fn_InsPropagate(navstate, secondimu, imudt, kf);

            else
                %% 纯推算阶段
                navstate = InsMech(navstate, lastimu, thisimu);
                kf = fn_HeightUpdate(navstate, height(imuindex, :), kf);

                % 反馈修正惯导状态 (仅修正天向，动态适配加减号)
                navstate.pos(3) = navstate.pos(3) + z_err_sign * kf.x(3);
                navstate.vel(3) = navstate.vel(3) - kf.x(6);

                if enable_feedback
                    % 测距组合模式保留水平误差状态，只清除已反馈的天向量。
                    kf.x(3) = 0;
                    kf.x(6) = 0;
                else
                    % 与旧 exper1_pureins.m 保持一致：纯惯导基线每次完成
                    % 高度反馈后清空全部误差状态，避免测距滤波内部状态
                    % 间接影响后续的高度反馈。
                    kf.x = zeros(size(kf.x));
                end

                if enable_smoothing == 1
                    Pk_propa(buf_idx,:) = kf.P(:)';
                end

                kf = fn_InsPropagate(navstate, thisimu, imudt, kf);

                if enable_smoothing == 1
                    nav = [navstate.time; navstate.pos; navstate.vel; navstate.att];
                    state_buffer(buf_idx,:) =  nav';
                    Xk_k1propa(buf_idx,:) = kf.x(:)';
                    Pk_k1propa(buf_idx,:) = kf.P(:)';
                    PHI(buf_idx,:) = kf.phi(:)';
                    buf_idx = buf_idx + 1;
                end
            end

            % 写入并打印
            nav_write = zeros(11, 1);
            nav_write(2, 1) = navstate.time;
            nav_write(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
            nav_write(6:8, 1) = navstate.vel;
            nav_write(9:11, 1) = navstate.att * param.R2D;
            fprintf(navfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_write);

            if (imuindex / size(imudata, 1) - lastprecent > 0.20)
                fprintf('    [%s/%s] Dataset %d processing %d %%\n', ...
                    unit_type, solution_mode, input_id, ...
                    floor(imuindex * 100 / size(imudata, 1)));
                lastprecent = imuindex / size(imudata, 1);
            end
        end

        %% 7. 尾部处理与文件关闭
        if enable_smoothing && ~isempty(prev_state_buffer)
            % 最后一段没有后一测距点，不能再次执行 RTS。此处用零误差
            % Linear 路径原样写出已完成的一次 RTS 轨迹。
            [final_nav_matrix, ~, ~] = perform_unified_smoothing( ...
                rtsstate_buffer, zeros(15, 1), param, rangeindex, ...
                'Linear', char(unit_type), [], [], []);
            fprintf(navfp2, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', final_nav_matrix);
        end

        fclose(navfp);
        if enable_smoothing
            fclose(navfp1);
            fclose(navfp2);
        end
        clear file_cleanup;

        fprintf('  --> [%s/%s] 解算完成，耗时 %.2f s。\n', ...
            unit_type, solution_mode, toc);

        %% 8. 评估测距辅助、二次 RTS 与一次 RTS
        if enable_smoothing && export_evaluation
            [fig, final_excel_data] = calc_radial_error_gjb( ...
                cfg.truthpath, navpath, navpath2, navpath1);
            report_path = fullfile(artifact_out_dir, sprintf( ...
                '导航系统径向误差统计报告-%s-%d.xlsx', unit_type, input_id));
            writecell(final_excel_data, report_path);

            image_path = fullfile(artifact_out_dir, sprintf( ...
                '补偿前后误差对比-%s-%d.png', unit_type, input_id));
            exportgraphics(fig, image_path, 'Resolution', 600);
            close(fig);
        end

      end % solution_mode
    end % unit_type
end % 结束 input_id 循环

fprintf('\n八组实测数据处理完毕！\n');

