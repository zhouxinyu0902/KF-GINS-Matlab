%% main_DR_RANGE.m
% DVL + 罗盘 + 深度计 航位推算 + 单信标斜距 EKF校正
% 第一版：前向EKF，不含RTS/线性平滑，不做复杂异步插值。
clear;
clc;
close all;
glvs;
%% -------------------- 配置与数据读取 --------------------
% in_dir = 'D:\Github\KF-GINS-Matlab\graduation\DR_INS\input\data_lawnmower_single_side';
in_dir = 'D:\Github\KF-GINS-Matlab\graduation\DR_INS\input\data_line_N_four_quadrants';
cfg = config_DR_RANGE(in_dir);

if ~exist(cfg.outputfolder, 'dir')
    mkdir(cfg.outputfolder);
end

dvl     = importdata(cfg.dvlpath);       % t vb_x vb_y vb_z
compass = importdata(cfg.compasspath);   % t pitch roll yaw, rad
depth   = importdata(cfg.depthpath);     % t depth, m, D向下为正
for ii = 1:4
    beacon  = importdata(cfg.beaconpath1{ii});    % t slant horizontal beacon_lat beacon_lon beacon_h

    has_truth = exist(cfg.truthpath, 'file') == 2;
    if has_truth
        truth = importdata(cfg.truthpath);
    end

    % 时间范围取各数据重叠区间
    starttime = max([dvl(1,1), compass(1,1), depth(1,1), beacon(1,1), cfg.starttime]);
    endtime   = min([dvl(end,1), compass(end,1), depth(end,1), beacon(end,1), cfg.endtime]);

    dvl     = dvl(dvl(:,1) >= starttime & dvl(:,1) <= endtime, :);
    compass = compass(compass(:,1) >= starttime & compass(:,1) <= endtime, :);
    depth   = depth(depth(:,1) >= starttime & depth(:,1) <= endtime, :);
    beacon  = beacon(beacon(:,1) >= starttime & beacon(:,1) <= endtime, :);
    interval = 8;
    beacon = beacon(interval:interval:end,:);
    if isempty(dvl) || isempty(compass) || isempty(depth) || isempty(beacon)
        error('输入数据为空，请检查路径和时间范围。');
    end

    fprintf('Start DR/RANGE Processing!\n');
    fprintf('Time span: %.3f s -> %.3f s\n', starttime, endtime);

    %% -------------------- 初始化 --------------------
    compass_idx = 1;
    depth_idx   = 1;
    range_idx   = 1;

    compass_idx = DRGetClosestIndex(compass, dvl(1,1), compass_idx);
    depth_idx   = DRGetClosestIndex(depth,   dvl(1,1), depth_idx);

    % 组合导航DR状态
    dr = DRInitialize(cfg, dvl(1,:), compass(compass_idx,:), depth(depth_idx,:));

    % 纯DR状态，用于对比，不进行距离反馈
    dr_origin = dr;

    % EKF初始化
    kf = DREKFInitialize(cfg, dr);

    % 输出文件
    origin_nav_path = fullfile(cfg.outputfolder, sprintf('Origin-DR-%d.nav',ii));
    fused_nav_path  = fullfile(cfg.outputfolder, sprintf('DR-RANGE-%d.nav',ii));
    % state_path      = fullfile(cfg.outputfolder, 'DR-RANGE-state.txt');
    % innov_path      = fullfile(cfg.outputfolder, 'DR-RANGE-innovation.txt');

    fp_origin = fopen(origin_nav_path, 'wt');
    fp_fused  = fopen(fused_nav_path,  'wt');
    % fp_state  = fopen(state_path,      'wt');
    % fp_innov  = fopen(innov_path,      'wt');

    % fprintf(fp_origin, '# id time lat_deg lon_deg h VE VN VU pitch_deg roll_deg yaw_deg\n');
    % fprintf(fp_fused,  '# id time lat_deg lon_deg h VE VN VU pitch_deg roll_deg yaw_deg\n');
    % fprintf(fp_state,  '# time x1_dK x2_dYaw x3_dLat x4_dLon std1 std2 std3 std4\n');
    % fprintf(fp_innov,  '# time range_meas range_pred innovation sqrtS used\n');

    %% -------------------- 主循环 --------------------
    lastprecent = 0;

    for k = 2:size(dvl, 1)

        tk = dvl(k,1);

        % 当前DVL时刻附近的罗盘和深度数据
        compass_idx = DRGetClosestIndex(compass, tk, compass_idx);
        depth_idx   = DRGetClosestIndex(depth,   tk, depth_idx);

        compass_k = compass(compass_idx, :);
        depth_k   = depth(depth_idx, :);
        dvl_k     = dvl(k, :);

        %% 1. 纯DR推算
        dr_origin = DRmechanization(dr_origin, dvl_k, compass_k, depth_k);

        %% 2. 组合导航DR推算
        dr = DRmechanization(dr, dvl_k, compass_k, depth_k);

        %% 3. DR误差传播
        kf = DRinspropagation(kf, dr);

        %% 4. 距离量测更新与反馈
        % 跳过已经明显早于当前DR历元的距离数据
        while range_idx <= size(beacon,1) && beacon(range_idx,1) < tk - cfg.range_time_tolerance
            range_idx = range_idx + 1;
        end

        % 如果当前DR历元附近有距离观测，则执行一次或多次更新
        while range_idx <= size(beacon,1) && abs(beacon(range_idx,1) - tk) <= cfg.range_time_tolerance
            beacon_k = beacon(range_idx, :);
            [kf, innov_info] = DRRangeUpdate(kf, dr, beacon_k, cfg);

            if innov_info.used
                [kf, dr] = DRfeedback(kf, dr, cfg);
            end

            % fprintf(fp_innov, '%12.6f %12.6f %12.6f %12.6f %12.6f %12.6f %d\n', ...
            %     0, innov_info.time, innov_info.range_meas, innov_info.range_pred, ...
            %     innov_info.innovation, innov_info.sqrtS, innov_info.used);

            range_idx = range_idx + 1;
        end

        %% 5. 保存结果
        DRWriteNavLine(fp_origin, k, dr_origin, cfg);
        DRWriteNavLine(fp_fused,  k, dr,        cfg);

        % stdx = sqrt(diag(kf.Pxk));
        % fprintf(fp_state, '%12.6f ', 0);
        % fprintf(fp_state, '%12.6f ', dr.time);
        % fprintf(fp_state, '%15.8e ', kf.xk(:));
        % fprintf(fp_state, '%15.8e ', stdx(:));
        % fprintf(fp_state, '\n');

        %% 6. 进度显示
        if (k / size(dvl,1) - lastprecent > 0.20)
            fprintf('processing %d %%!\n', floor(k * 100 / size(dvl,1)));
            lastprecent = k / size(dvl,1);
        end
    end

    fclose(fp_origin);
    fclose(fp_fused);
    % fclose(fp_state);
    % fclose(fp_innov);

    fprintf('DR/RANGE processing finished.\n');
    fprintf('Origin DR nav saved to: %s\n', origin_nav_path);
    fprintf('Fused  DR nav saved to: %s\n', fused_nav_path);

    calc_radial_error_gjb(cfg.truthpath,origin_nav_path,fused_nav_path);
end