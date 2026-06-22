close all
clear
clc
glvs

%% ==================== 0. 用户配置区 ====================

baseFolders{1}   = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Circle_Beacon1_Trj\';
baseFolders{2}   = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Line_Beacon1_Trj\';
baseFolders{3}  = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Square_Beacon1_Trj\';
% baseFolder   = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Circle_ArrayCenter_Trj\';
% baseFolder   = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Line_ArrayCenter_Trj\';
% baseFolder   = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Square_ArrayCenter_Trj\';
for ii = 1:numel(baseFolders)

    baseFolder = baseFolders{ii};
    inputFolder  = fullfile(baseFolder, 'input');
    pathpos      = fullfile(inputFolder, 'beacon_pos.mat');

    % baseFolder   = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_1\data1\';
    % inputFolder  = fullfile(baseFolder, 'input');
    % pathpos      = fullfile(baseFolder, 'beacon_pos.mat');


    % id           = 2;        % 当前迭代次数

    for id=1:2
        feedback     = 1;
        rangeSuffix  = '20s';    % 重构文件后缀，例如 range1_calib_1_20s.txt

        % 第一次迭代的原始真实参考角
        theta_ref_init = 20;     % deg
        phi_ref_init   = 45;     % deg

        %% ==================== 1. 参数与配置加载 ====================

        param = Param();
        cfg   = Config_10state(baseFolder, 'align');

        if ~exist(cfg.outputfolder, 'dir')
            mkdir(cfg.outputfolder);
        end

        %% ==================== 2. 自动确定本轮参考角与测距文件 ====================

        originalRangePaths = { ...
            fullfile(inputFolder, 'range1.txt'), ...
            fullfile(inputFolder, 'range2.txt'), ...
            fullfile(inputFolder, 'range3.txt')};

        if id == 1

            theta_ref_deg = theta_ref_init;
            phi_ref_deg   = phi_ref_init;

            activeRangePaths = originalRangePaths;

        else

            % 优先读取上一轮 show_result1 输出的 theta_next / phi_next
            prevRefFile = fullfile(baseFolder, sprintf('iter_%d_next_ref.mat', id - 1));

            if exist(prevRefFile, 'file')
                prevData = load(prevRefFile);
                theta_ref_deg = prevData.theta_next;
                phi_ref_deg   = prevData.phi_next;
            else
                warning('未找到上一轮参考角文件：%s，将使用手动默认值。', prevRefFile);

                % 如果没有保存上一轮结果，可以在这里手动兜底
                theta_ref_deg = 2.82;
                phi_ref_deg   = 68;
            end

            % id=2 时，自动读取 range1_calib_1_20s.txt
            % id=3 时，自动读取 range1_calib_2_20s.txt
            activeRangePaths = local_make_calib_range_paths(inputFolder, id - 1, rangeSuffix);

        end

        fprintf('\n==================== 当前迭代配置 ====================\n');
        fprintf('当前迭代次数：%d\n', id);
        fprintf('本轮参考角：theta_ref = %.6f deg, phi_ref = %.6f deg\n', theta_ref_deg, phi_ref_deg);
        fprintf('测距文件：\n');
        for i = 1:numel(activeRangePaths)
            fprintf('  #%d %s\n', i, activeRangePaths{i});
        end
        fprintf('======================================================\n\n');

        %% ==================== 3. 加载数据 ====================

        imudata = local_import_numeric(cfg.imufilepath);
        truth   = local_import_numeric(cfg.truthpath);

        rangedata1 = local_import_numeric(activeRangePaths{1});
        rangedata2 = local_import_numeric(activeRangePaths{2});
        rangedata3 = local_import_numeric(activeRangePaths{3});

        imustarttime = imudata(1, 1);
        imuendtime   = imudata(end, 1);

        %% ==================== 4. 处理时间范围裁剪 ====================

        starttime = max(cfg.starttime, imustarttime);
        endtime   = min(cfg.endtime,   imuendtime);

        cfg.starttime = starttime;
        cfg.endtime   = endtime;

        imudata = local_clip_time(imudata, cfg.starttime, cfg.endtime);

        heightdata = truth(:, [2, 5]);
        heightdata = local_clip_time(heightdata, cfg.starttime, cfg.endtime);
        heightdata(:,2) = heightdata(:,2) + normrnd(0, 0.2, size(heightdata(:,2)));

        rangedata1 = local_clip_time(rangedata1, cfg.starttime, cfg.endtime);
        rangedata2 = local_clip_time(rangedata2, cfg.starttime, cfg.endtime);
        rangedata3 = local_clip_time(rangedata3, cfg.starttime, cfg.endtime);

        %% ==================== 5. 设置文件保存路径 ====================

        navpath = fullfile(cfg.outputfolder, sprintf('NavResult_iter%d.nav', id));
        xkpath  = fullfile(cfg.outputfolder, sprintf('xk_iter%d.nav', id));

        navfp = fopen(navpath, 'wt');
        xkfp  = fopen(xkpath,  'wt');

        if navfp < 0
            error('无法打开导航结果输出文件：%s', navpath);
        end

        if xkfp < 0
            error('无法打开状态结果输出文件：%s', xkpath);
        end

        %% ==================== 6. 初始化滤波器与导航状态 ====================

        disp("Start Processing!");

        [kf, navstate] = myInitialize_10state(cfg);

        % 第二轮及以后，注入上一轮残余角度作为本轮初始参考
        if id > 1

            kf.P(9, 9)   = power(5.0 * glv.deg, 2);
            kf.P(10, 10) = power(5.0 * glv.deg, 2);
            kf.P0 = kf.P;

            navstate.theta_calib = d2r(theta_ref_deg);
            navstate.phi_calib   = d2r(phi_ref_deg);

        end

        kf.depthstd = 0.2;
        kf.rangstd  = 6;

        laststate = navstate;
        lastimu   = imudata(1, :)';
        thisimu   = imudata(1, :)';

        rangeindex  = 1;
        cfg.userange = 1;
        lastprecent = 0;
        timeTol     = 1e-8;

        %% ==================== 7. 主循环 ====================

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

            while (rangeindex <= size(rangedata1, 1) && rangedata1(rangeindex, 1) < lastimu(1, 1))
                rangeindex = rangeindex + 1;
            end

            if (rangeindex > length(rangedata1))
                % rangeindex = rangeindex - 1;
                disp('range file END!');
                break;
            end
            rangedata = [rangedata1(rangeindex,:);
                rangedata2(rangeindex,:);
                rangedata3(rangeindex,:)];
            %% 4、determine whether gnss update is required
            if lastimu(1, 1) == rangedata1(rangeindex, 1) && cfg.userange==1
                % 测量更新
                d_sub = [1000+randn*0.2; 1015+randn*0.2; 985+randn*0.2];
                kf = myRangeUpdate_10state_m(navstate, rangedata, d_sub, kf);
                rangeindex = rangeindex + 1;
                xk = zeros(11, 1);
                xk(1) = navstate.time;
                xk(2:11) = kf.x;
                xk(10) = navstate.theta_calib;
                xk(11) = navstate.phi_calib;
                fprintf(xkfp, '%12.6f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f \n', xk);
                if feedback==1
                    [kf, navstate] = myErrorFeedback_range_m_10(kf, navstate);
                    % [kf, navstate] = myErrorFeedback_15state(kf, navstate);
                end
                laststate = navstate;

                % 惯导推算
                imudt = thisimu(1, 1) - lastimu(1, 1);
                navstate = InsMech(laststate, lastimu, thisimu);
                kf = myInsPropagate_10state(navstate, thisimu, imudt, kf);
            else
                %% only do propagation
                % INS mechanization
                navstate = InsMech(laststate, lastimu, thisimu);
                navstate.pos(3) = heightdata(imuindex,2);


                % error propagation
                kf = myInsPropagate_10state(navstate, thisimu, imudt, kf);

            end

            % 6、save data
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

        %% ==================== 8. 关闭文件 ====================

        fclose(navfp);
        fclose(xkfp);

        disp("PureIns Integration Processing Finished!");

        %% ==================== 9. 误差评估 ====================

        % calc_error(navpath, cfg.truthpath);
        %
        xk = local_import_numeric(xkpath);

        %% ==================== 10. 角度收敛曲线 ====================

        if ~isempty(xk)

            myfigurestartup(7, 3, 'zxy');

            subplot(1,2,1)
            plot(r2d(xk(:,10)), 'LineWidth', 1.5)
            hold on
            plot(theta_ref_deg * ones(size(xk(:,10))), '--r', 'LineWidth', 1.5)
            grid on
            xlabel('迭代步')
            ylabel('倾角 \theta (deg)')
            legend('估计值', sprintf('本轮参考 %.3f°', theta_ref_deg), 'Location', 'best')

            subplot(1,2,2)
            plot(r2d(xk(:,11)), 'LineWidth', 1.5)
            hold on
            plot(phi_ref_deg * ones(size(xk(:,11))), '--r', 'LineWidth', 1.5)
            grid on
            xlabel('迭代步')
            ylabel('方位角 \phi (deg)')
            legend('估计值', sprintf('本轮参考 %.3f°', phi_ref_deg), 'Location', 'best')

        else
            warning('xk 文件为空，未绘制角度收敛曲线。');

        end

        %% ==================== 11. 本轮标定结果评估 ====================

        load(pathpos, 'S_gnss_geo', 'S_true_geo', 'pos0_geo');

        theta_est = xk(end, 10);
        phi_est   = xk(end, 11);

        outputExcelName = fullfile(baseFolder, sprintf('标定结果_%d.xlsx', id));


        if id==2
            prevData = load(fullfile(baseFolder, 'iter_1_next_ref.mat'), 'S_est_xyz');

            [S_est_xyz, theta_next, phi_next] = show_result1( ...
                2, ...
                S_gnss_geo, ...
                S_true_geo, ...
                pos0_geo, ...
                theta_est, ...
                phi_est, ...
                theta_ref_deg, ...
                phi_ref_deg, ...
                outputExcelName, ...
                prevData.S_est_xyz);
        else
            [S_est_xyz, theta_next, phi_next] = show_result1( ...
                id, ...
                S_gnss_geo, ...
                S_true_geo, ...
                pos0_geo, ...
                theta_est, ...
                phi_est, ...
                theta_ref_deg, ...
                phi_ref_deg, ...
                outputExcelName);
        end
        %% ==================== 12. 保存下一轮残余角度参考 ====================

        nextRefFile = fullfile(baseFolder, sprintf('iter_%d_next_ref.mat', id));

        save(nextRefFile, ...
            'theta_next', 'phi_next', ...
            'theta_est', 'phi_est', ...
            'theta_ref_deg', 'phi_ref_deg', ...
            'S_est_xyz');

        fprintf('\n下一轮参考角已保存：%s\n', nextRefFile);
        fprintf('theta_next = %.6f deg, phi_next = %.6f deg\n', theta_next, phi_next);

        %% ==================== 13. 重构下一轮测距文件 ====================
        % 注意：
        % 这里始终用原始 range1.txt / range2.txt / range3.txt 作为输入，
        % 然后根据本轮 S_est_xyz 重新生成 range*_calib_id_suffix.txt。

        outputFiles = range_reconstruct1( ...
            originalRangePaths, ...
            pathpos, ...
            inputFolder, ...
            id, ...
            S_est_xyz, ...
            rangeSuffix);

        fprintf('\n本轮重构文件输出完成：\n');
        disp(outputFiles.calib);
    end
    %% 两次结果对比
    if id == 2
        compare_two_iter_position( ...
            baseFolder, ...
            pathpos, ...
            1, ...
            2);
        pngpath  = fullfile(baseFolder, 'iter_result.png');
        exportgraphics(gcf, pngpath, 'Resolution', 600);
    end
end
%% ==================== 本地辅助函数 ====================

function data = local_import_numeric(filepath)

filepath = char(filepath);

if ~exist(filepath, 'file')
    error('文件不存在：%s', filepath);
end

raw = importdata(filepath);

if isstruct(raw)
    data = raw.data;
else
    data = raw;
end

if isempty(data)
    warning('文件为空或未读取到数值数据：%s', filepath);
end

end

function dataOut = local_clip_time(dataIn, starttime, endtime)

dataOut = dataIn(dataIn(:,1) >= starttime, :);
dataOut = dataOut(dataOut(:,1) <= endtime, :);

end

function rangePaths = local_make_calib_range_paths(inputFolder, prevId, suffixStr)

suffixStr = char(suffixStr);

rangePaths = cell(1, 3);

for k = 1:3
    rangePaths{k} = fullfile( ...
        inputFolder, ...
        sprintf('range%d_calib_%d_%s.txt', k, prevId, suffixStr));
end

end
