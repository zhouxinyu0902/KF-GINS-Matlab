%% =========================================================================
% 惯导/测距组合导航双模式(rad & m)批量处理脚本
% 支持 1~8 组数据，自动切换单位体系与对应的底层更新函数
% =========================================================================
clear; clc; close all;

% 参数初始化
param = Param();

% 定义标准差和其他设置
rngstd = 6;
depthstd = 0.4;
SmoothIsOpen = 1;
feedback = 1;

for ID = 5:6
    fprintf('\n======================================================\n');
    fprintf('开始处理第 %d 组数据...\n', ID);
    fprintf('======================================================\n');
    
    in_dir = ['F:/2_Data/惯导试验/实验数据/All_data/input', num2str(ID)];
    out_dir = ['D:\Github\KF-GINS-Matlab\new_惯导试验\output/output', num2str(ID)];
    if ~exist(out_dir, 'dir'), mkdir(out_dir); end
    
    % 核心逻辑：循环跑两种单位模式 (rad 和 m)
    for unitType = "rad"
        rng(1); % 每次循环重置随机数种子，保证添加的噪声完全一致
        tic;
        
        fprintf('  --> 正在使用 [%s] 单位模式进行解算...\n', unitType);
        
        %% 1. 动态绑定配置与底层函数 (修复了原代码中的遗漏Bug)
        if unitType == "rad"
            cfg = config_1(in_dir);
            fn_RangeUpdate   = @myRangeUpdate;
            fn_HeightUpdate  = @myHeightUpdate;
            fn_ErrorFeedback = @myErrorFeedback_range;
            fn_InsPropagate  = @myInsPropagate_15state;
            z_err_sign       = -1; % rad 模式下: pos(3) - kf.x(3)
        else
            cfg = config_1_m(in_dir);
            fn_RangeUpdate   = @myRangeUpdate_m;
            fn_HeightUpdate  = @myHeightUpdate_m;
            fn_ErrorFeedback = @myErrorFeedback_range_m;
            fn_InsPropagate  = @myInsPropagate_15state_m;
            z_err_sign       = 1;  % m 模式下: pos(3) + kf.x(3)
        end
        
        cfg.outputfolder = out_dir;
        cfg.userange = 1;
        
        %% 2. 导入与构造数据
        imudata = importdata(cfg.imufilepath);
        imustarttime = imudata(1, 1);
        imuendtime = imudata(end, 1);
        
        rangedata1 = importdata(cfg.rangefile1path);
        rangedata2 = importdata(cfg.rangefile2path);
        rangedata3 = importdata(cfg.rangefile3path);
        range = {rangedata1, rangedata2, rangedata3};
        
        % 构造范围数据 (420秒 = 7分钟数据周期)
        id_period = 420; 
        for i = 1:3
            range{i} = range{i}(id_period:id_period:end, :);
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
        rangedata(:, 3) = rangedata(:, 3) + normrnd(0, rngstd, size(rangedata(:, 3)));
        
        height = height(height(:, 1) >= cfg.starttime & height(:, 1) <= cfg.endtime, :);
        height(:, 2) = height(:, 2) + normrnd(0, depthstd, size(height(:, 2)));
        
        %% 4. 设置文件保存路径
        smoothWay = 'RTS';
        navpath = fullfile(cfg.outputfolder, sprintf('Origin-%s.nav', unitType));
        navfp = fopen(navpath, 'wt');
        
        if SmoothIsOpen == 1
            navpath1 = fullfile(cfg.outputfolder, sprintf('%s-DoubleSmooth-%s.nav', smoothWay, unitType));
            navfp1 = fopen(navpath1, 'wt');
            navpath2 = fullfile(cfg.outputfolder, sprintf('%s-SingleSmooth-%s.nav', smoothWay, unitType));
            navfp2 = fopen(navpath2, 'wt');
        end
        
        %% 5. 初始化
        [kf, navstate] = myInitialize_15state(cfg);
        laststate = navstate;
        kf.rangstd = rngstd;
        kf.depthstd = depthstd;
        
        lastimu = imudata(1, :)';
        thisimu = imudata(1, :)';
        imudt = thisimu(1, 1) - lastimu(1, 1);
        
        rangeindex = 1;
        while rangedata(rangeindex, 1) < thisimu(1, 1)
            rangeindex = rangeindex + 1;
        end
        
        % 缓存定义
        MAX_BUFFER_SIZE = 54000;
        state_buffer = zeros(MAX_BUFFER_SIZE, 10);
        Xk_k1propa   = zeros(MAX_BUFFER_SIZE, 15);
        Pk_k1propa   = zeros(MAX_BUFFER_SIZE, 225);
        Pk_propa     = zeros(MAX_BUFFER_SIZE, 225);
        PHI          = zeros(MAX_BUFFER_SIZE, 225); 
        
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
                
                if SmoothIsOpen == 1
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
                            sub_state_buffer, xk_final, param, rangeindex, smoothWay, char(unitType), sub_Pk_propa, sub_Pk_k1propa, sub_PHI);
                        fprintf(navfp2, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix);
                        bridge_err(rangeindex,:) = bridge_error;
                        
                        if isempty(prev_state_buffer)
                            prev_state_buffer = rtsstate_buffer; prev_Pk_propa = sub_Pk_propa;
                            prev_Pk_k1propa = sub_Pk_k1propa; prev_PHI = sub_PHI; prev_rangeindex = rangeindex;
                        else
                            [nav_matrix_prev, bridge_error, smoothed_state_buffer] = perform_unified_smoothing(...
                                prev_state_buffer, bridge_error, param, prev_rangeindex, smoothWay, char(unitType), prev_Pk_propa, prev_Pk_k1propa, prev_PHI);
                            fprintf(navfp1, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix_prev);
                            
                            prev_state_buffer = rtsstate_buffer; prev_Pk_propa = sub_Pk_propa;
                            prev_Pk_k1propa = sub_Pk_k1propa; prev_PHI = sub_PHI; prev_rangeindex = rangeindex;
                        end
                        buf_idx = 1;
                        state_buffer(:) = 0; Xk_k1propa(:) = 0; Pk_k1propa(:) = 0; Pk_propa(:) = 0; PHI(:) = 0;
                    end
                end
                
                if feedback == 1
                    [kf, navstate] = fn_ErrorFeedback(kf, navstate);
                end
                
                % 惯导推算
                imudt = thisimu(1, 1) - lastimu(1, 1);
                navstate = InsMech(navstate, lastimu, thisimu);
                kf = fn_InsPropagate(navstate, thisimu, imudt, kf);
                
            elseif (lastimu(1, 1) < rangedata(rangeindex, 1) && thisimu(1, 1) > rangedata(rangeindex, 1)) && cfg.userange == 1
                
                kf = fn_RangeUpdate(navstate, rangedata(rangeindex,:), height(imuindex,:), kf);
                if feedback == 1
                    [kf, navstate] = fn_ErrorFeedback(kf, navstate);
                end
                rangeindex = rangeindex + 1;
                
                % 插入推算 (Bug修复: 调用了统一的 fn_InsPropagate)
                imudt = secondimu(1, 1) - lastimu(1, 1);
                navstate = InsMech(navstate, lastimu, secondimu);
                kf = fn_InsPropagate(navstate, secondimu, imudt, kf);
                
            else
                %% 纯推算阶段
                navstate = InsMech(navstate, lastimu, thisimu);
                kf = fn_HeightUpdate(navstate, height(imuindex, :), kf);
                
                % 反馈修正惯导状态 (仅修正天向，动态适配加减号)
                navstate.pos(3) = navstate.pos(3) + z_err_sign * kf.x(3);
                navstate.vel(3) = navstate.vel(3) - kf.x(6);
                
                kf.x(3) = 0; kf.x(6) = 0;
                
                if SmoothIsOpen == 1
                    Pk_propa(buf_idx,:) = kf.P(:)';
                end
                
                kf = fn_InsPropagate(navstate, thisimu, imudt, kf);
                
                if SmoothIsOpen == 1
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
                fprintf('    [%s] Round %d processing %d %%\n', unitType, ID, floor(imuindex * 100 / size(imudata, 1)));
                lastprecent = imuindex / size(imudata, 1);
            end
        end
        
        %% 7. 尾部处理与文件关闭
        if SmoothIsOpen == 1 && ~isempty(prev_state_buffer)
            [nav_matrix, ~, ~] = perform_unified_smoothing(rtsstate_buffer, zeros(15,1), ...
                param, rangeindex, 'Linear', char(unitType), [], [], []);
            fprintf(navfp1, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix);
        end
        
        if exist('navfp', 'var'), fclose(navfp); end
        if exist('navfp1', 'var'), fclose(navfp1); end
        if exist('navfp2', 'var'), fclose(navfp2); end
        
        fprintf('  --> [%s] 模式解算完成！耗时: %.2f 秒\n', unitType, toc);
        
        %% 8. 计算误差并导出报表 (结合你之前的 calc_radial_error_gjb)
        % 如果文件成功生成，直接调用统计报告函数，并保存对应的 Excel 和图片
        if SmoothIsOpen == 1
            [fig, finalExcelData] = calc_radial_error_gjb(cfg.truthpath, navpath, navpath1, navpath2);
            
            outputExcelName = fullfile(out_dir, sprintf('导航系统径向误差统计报告-%s-%d.xlsx', unitType, ID));
            writecell(finalExcelData, outputExcelName);
            
            outputImageName = fullfile(out_dir, sprintf('补偿前后误差对比-%s-%d.png', unitType, ID));
            exportgraphics(fig, outputImageName, 'Resolution', 600);

            outputImageName = fullfile(out_dir, sprintf('补偿前后误差对比-%s-%d.fig', unitType, ID));
            savefig(fig,outputImageName)
        end
        
    end % 结束 unitType 循环
end % 结束 ID 循环

fprintf('\n🎉 全部 8 组数据 (m & rad 双模式) 处理完毕！\n');