clear;
clc;
close all;
%% ========================================================================
% data07_read03_auxa_120.m
% 2025-12-07
% 120 AUXA + Disk2 IMU 时间统一（仅依赖Compass）
%
% 核心流程：
% Compass
%   -> 读取全部Disk1 AUXA
%   -> 按AUXA上机时间回跳划分多个开机段
%   -> 每个开机段独立搜索430.010/430.015 + 5820->4300
%   -> 对每段Compass选择“Compass起点之前最近”的有效对准
%   -> 保存该次开机的静止AUXA：开机 -> 430.01前
%   -> 保存对准后有效AUXA：430.01 -> Compass结束
%   -> 读取对应Disk2
%   -> 按IMU上机时间回跳划分多个开机段
%   -> 优先匹配与AUXA相同序号的开机段
%   -> 在430.01附近小时间窗内用AUXA(12:14)/IMU(:,1:3)精确匹配
%   -> 只对最终选中的Disk2开机段赋GPS时间
%   -> 保存静止IMU与对准后IMU
%
% 不依赖830。
% 不使用人为420 s补偿。
%
% 时间：
%   AUXA A(1,:)        上机时间
%   AUXA A(2,:)        原始时间+18后的GPS周秒
%   IMU raw(:,7)/200   上机时间
% ========================================================================
%% 0. 路径
raw_dir = 'D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1207-longtime\raw';
temp_dir = 'D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1207-longtime\temp';
compass_file = fullfile(temp_dir,'compass.mat');
save_file = fullfile(temp_dir,'alignment120_1207.mat');
%% 1. 参数
GPS_WEEK = 2396;
AUAX_TIME_OFFSET = 18;
MIN_VALID_TIME = 100;
STATUS_FINE = hex2dec('5820');
STATUS_NAV = hex2dec('4300');
ALIGN_MARKER = [430.010,430.015];
MARKER_TOL = 0.002;
MARKER_STATUS_WINDOW = 20;
STARTUP_MARKER = [0.005,0.010];
STARTUP_TOL = 0.003;
RESET_JUMP = -1.0;
IMU_MATCH_TIME_WINDOW = 0.050;
MACHINE_TIME_WARN = 0.020;
frameHeader = [235,144,32];
%% ========================================================================
%% 2. 读取Compass
% ========================================================================
fprintf('\n============================================================\n');
fprintf('1. 读取Compass实验时间\n');
fprintf('============================================================\n');
load(compass_file,'Compass');
if numel(Compass) ~= 21
    error('Compass应包含下午、晚上两段实验数据。');
end
Experiment = struct();
for k = 1:2
    Experiment(k).name = Compass{k}.name;
    Experiment(k).compass_range = [Compass{k}.time(1),Compass{k}.time(end)];
    fprintf('\n%s\n',Experiment(k).name);
    fprintf('  Compass：%.3f ~ %.3f\n',Experiment(k).compass_range(1),Experiment(k).compass_range(2));
    fprintf('  起点UTC：%s\n',gpssec_to_utc_string(GPS_WEEK,Experiment(k).compass_range(1)));
end
%% ========================================================================
%% 3. 读取全部Disk1 AUXA
% ========================================================================
fprintf('\n============================================================\n');
fprintf('2. 读取Disk1_000 ~ Disk1_007 AUXA\n');
fprintf('============================================================\n');
AUAX_file = cell(1,8);
for file_id = 0:7
    filename = fullfile(raw_dir,sprintf('Disk1_%03d.dat',file_id));
    if ~exist(filename,'file')
        warning('文件不存在：%s',filename);
        continue;
    end
    fprintf('\n读取 Disk1_%03d.dat\n',file_id);
    A = read_auax_120(filename);
    if isempty(A)
        fprintf('  无解析数据。\n');
        continue;
    end
    A(2,:) = A(2,:) + AUAX_TIME_OFFSET;
    AUAX_file{file_id+1} = A;
    valid_gps = isfinite(A(2,:)) & A(2,:) > MIN_VALID_TIME;
    fprintf('  总历元：%d\n',size(A,2));
    fprintf('  有效GPS历元：%d\n',sum(valid_gps));
    if any(valid_gps)
        fprintf('  有效GPS：%.3f ~ %.3f\n',min(A(2,valid_gps)),max(A(2,valid_gps)));
    end
end
%% ========================================================================
%% 4. 搜索全部有效对准事件
% ========================================================================
fprintf('\n============================================================\n');
fprintf('3. 搜索AUXA全部有效对准事件\n');
fprintf('============================================================\n');
Candidate = struct([]);
candidate_count = 0;
% for file_id = 0:7
for file_id = [3,6]
    A = AUAX_file{file_id+1};
    if isempty(A)
        continue;
    end
    run_time = A(1,:);
    gps_time = A(2,:);
    status_row = size(A,1)-2;
    status = A(status_row,:);
    [seg_start,seg_end] = split_segments(run_time,RESET_JUMP);
    fprintf('\nDisk1_%03d：检测到%d个开机段\n',file_id,numel(seg_start));
    for seg_id = 1:numel(seg_start)
        i1 = seg_start(seg_id);
        i2 = seg_end(seg_id);
        idx_seg = i1:i2;
        marker_mask = false(size(idx_seg));
        for value = ALIGN_MARKER
            marker_mask = marker_mask | abs(run_time(idx_seg)-value) <= MARKER_TOL;
        end
        marker_local_all = find(marker_mask);
        if isempty(marker_local_all)
            continue;
        end
        keep_last = [diff(marker_local_all)>1,true];
        marker_local = marker_local_all(keep_last);
        marker_global = i1+marker_local-1;
        fprintf('  boot %d：检测到%d组430.01 marker\n',seg_id,numel(marker_global));
        selected_marker = [];
        for m = numel(marker_global):-1:1
            im = marker_global(m);
            marker_run = run_time(im);
            marker_gps = gps_time(im);
            if ~isfinite(marker_gps) || marker_gps <= MIN_VALID_TIME
                continue;
            end
            before_idx = i1:im;
            fine_mask = status(before_idx) == STATUS_FINE & ...
                run_time(before_idx) >= marker_run-MARKER_STATUS_WINDOW & ...
                run_time(before_idx) <= marker_run;
            fine_id = before_idx(fine_mask);
            if isempty(fine_id)
                fprintf('    marker idx=%d run=%.6f：前方无5820\n',im,marker_run);
                continue;
            end
            fine_end = fine_id(end);
            after_start = max(im,fine_end+1);
            after_idx = after_start:i2;
            nav_mask = status(after_idx) == STATUS_NAV & ...
                run_time(after_idx) >= marker_run & ...
                run_time(after_idx) <= marker_run+MARKER_STATUS_WINDOW;
            nav_id = after_idx(nav_mask);
            if isempty(nav_id)
                fprintf('    marker idx=%d run=%.6f：后方无4300\n',im,marker_run);
                continue;
            end
            nav_start = nav_id(1);
            if ~(fine_end <= im && im <= nav_start)
                continue;
            end
            startup_idx = find_startup_index(run_time,i1,im,STARTUP_MARKER,STARTUP_TOL);
            selected_marker.im = im;
            selected_marker.fine_end = fine_end;
            selected_marker.nav_start = nav_start;
            selected_marker.startup_idx = startup_idx;
            break;
        end
        if isempty(selected_marker)
            fprintf('  boot %d：没有完整有效对准事件\n',seg_id);
            continue;
        end
        im = selected_marker.im;
        fine_end = selected_marker.fine_end;
        nav_start = selected_marker.nav_start;
        startup_idx = selected_marker.startup_idx;
        candidate_count = candidate_count+1;
        Candidate(candidate_count).file_id = file_id;
        Candidate(candidate_count).boot_segment_id = seg_id;
        Candidate(candidate_count).segment_start = i1;
        Candidate(candidate_count).segment_end = i2;
        Candidate(candidate_count).startup_index = startup_idx;
        Candidate(candidate_count).startup_run_time = run_time(startup_idx);
        Candidate(candidate_count).marker_index = im;
        Candidate(candidate_count).marker_run_time = run_time(im);
        Candidate(candidate_count).marker_time = gps_time(im);
        Candidate(candidate_count).fine_end_index = fine_end;
        Candidate(candidate_count).fine_end_time = gps_time(fine_end);
        Candidate(candidate_count).nav_start_index = nav_start;
        Candidate(candidate_count).nav_start_time = gps_time(nav_start);
        Candidate(candidate_count).fine_to_marker = run_time(im)-run_time(fine_end);
        Candidate(candidate_count).marker_to_nav = run_time(nav_start)-run_time(im);
        fprintf('  [OK] Candidate %d：boot=%d startup=%.3f marker=%.3f GPS=%.6f nav=%.3f\n', ...
            candidate_count,seg_id,run_time(startup_idx),run_time(im),gps_time(im),run_time(nav_start));
    end
end
if isempty(Candidate)
    error('没有找到满足5820 -> 430.01 -> 4300关系的有效AUXA对准事件。');
end
[~,idx_sort] = sort([Candidate.marker_time]);
Candidate = Candidate(idx_sort);
fprintf('\n============================================================\n');
fprintf('最终有效AUXA对准候选\n');
fprintf('============================================================\n');
for i = 1:numel(Candidate)
    fprintf('%2d：Disk1_%03d boot=%d start=%.3f marker=%.3f GPS=%.6f nav=%.3f\n', ...
        i,Candidate(i).file_id,Candidate(i).boot_segment_id,Candidate(i).startup_run_time, ...
        Candidate(i).marker_run_time,Candidate(i).marker_time,Candidate(i).nav_start_time);
end
%% ========================================================================
%% 5. 根据Compass选择最近对准
% ========================================================================
fprintf('\n============================================================\n');
fprintf('4. 根据Compass选择最近AUXA对准\n');
fprintf('============================================================\n');
Alignment120 = struct();
marker_time_all = [Candidate.marker_time];
used = false(size(Candidate));
for k = 1:2
    t_compass_start = Experiment(k).compass_range(1);
    id = find(marker_time_all <= t_compass_start & ~used);
    if ~isempty(id)
        [~,ii] = min(t_compass_start-marker_time_all(id));
        selected_id = id(ii);
    else
        id = find(~used);
        if isempty(id)
            error('%s：没有剩余对准候选。',Experiment(k).name);
        end
        [~,ii] = min(abs(marker_time_all(id)-t_compass_start));
        selected_id = id(ii);
    end
    used(selected_id) = true;
    C = Candidate(selected_id);
    Alignment120(k).name = Experiment(k).name;
    Alignment120(k).candidate_id = selected_id;
    Alignment120(k).file_id = C.file_id;
    Alignment120(k).boot_segment_id = C.boot_segment_id;
    Alignment120(k).startup_index = C.startup_index;
    Alignment120(k).startup_run_time = C.startup_run_time;
    Alignment120(k).marker_index = C.marker_index;
    Alignment120(k).marker_run_time = C.marker_run_time;
    Alignment120(k).marker_time = C.marker_time;
    Alignment120(k).fine_end_time = C.fine_end_time;
    Alignment120(k).nav_start_time = C.nav_start_time;
    Alignment120(k).compass_start = t_compass_start;
    Alignment120(k).compass_end = Experiment(k).compass_range(2);
    Alignment120(k).alignment_to_compass = t_compass_start-C.marker_time;
    fprintf('\n%s\n',Alignment120(k).name);
    fprintf('  Disk1_%03d，开机段%d\n',C.file_id,C.boot_segment_id);
    fprintf('  开机run：    %.6f s\n',C.startup_run_time);
    fprintf('  对准run：    %.6f s\n',C.marker_run_time);
    fprintf('  对准GPS：    %.6f\n',C.marker_time);
    fprintf('  Compass起点：%.6f\n',t_compass_start);
    fprintf('  对准提前：   %.3f s\n',t_compass_start-C.marker_time);
end
%% ========================================================================
%% 6. 保存静止AUXA和对准后有效AUXA
% ========================================================================
fprintf('\n============================================================\n');
fprintf('5. 保存静止AUXA和对准后AUXA\n');
fprintf('============================================================\n');
for k = 1:2
    C = Candidate(Alignment120(k).candidate_id);
    A = AUAX_file{C.file_id+1};
    static_idx = C.startup_index:C.marker_index-1;
    A_static = A(:,static_idx);
    valid_gps = isfinite(A(2,:)) & A(2,:) > MIN_VALID_TIME;
    dynamic_mask = (1:size(A,2)) >= C.marker_index & ...
        (1:size(A,2)) <= C.segment_end & ...
        A(2,:) <= Alignment120(k).compass_end & ...
        valid_gps;
    A_dynamic = A(:,dynamic_mask);
    if isempty(A_dynamic)
        error('%s：430.01之后没有有效AUXA数据。',Alignment120(k).name);
    end
    A0 = A(:,C.marker_index);
    Alignment120(k).auax_static = A_static;
    Alignment120(k).auax = A_dynamic;
    Alignment120(k).auax_anchor = A0;
    Alignment120(k).auxa_anchor_machine_time = A0(1);
    Alignment120(k).auxa_anchor_gps_time = A0(2);
    Alignment120(k).auax_static_time_gps = A0(2)+(A_static(1,:)-A0(1));
    fprintf('\n%s\n',Alignment120(k).name);
    fprintf('  静止AUXA：%d历元，run %.6f ~ %.6f\n',size(A_static,2),A_static(1,1),A_static(1,end));
    fprintf('  静止推算GPS：%.6f ~ %.6f\n',Alignment120(k).auax_static_time_gps(1),Alignment120(k).auax_static_time_gps(end));
    fprintf('  对准后AUXA：%d历元，GPS %.6f ~ %.6f\n',size(A_dynamic,2),A_dynamic(2,1),A_dynamic(2,end));
end
%% ========================================================================
%% 7. 读取对应Disk2原始IMU
% ========================================================================
fprintf('\n============================================================\n');
fprintf('6. 读取对应Disk2原始IMU\n');
fprintf('============================================================\n');
for k = 1:2
    file_id = Alignment120(k).file_id;
    filename_imu = fullfile(raw_dir,sprintf('Disk2_%03d.dat',file_id));
    if ~exist(filename_imu,'file')
        warning('IMU文件不存在：%s',filename_imu);
        continue;
    end
    fprintf('\n%s：Disk2_%03d.dat\n',Alignment120(k).name,file_id);
    fid = fopen(filename_imu,'rb');
    allData = fread(fid,inf,'uint8');
    fclose(fid);
    headerPositions = find(allData(1:end-2) == frameHeader(1) & ...
        allData(2:end-1) == frameHeader(2) & ...
        allData(3:end) == frameHeader(3));
    if isempty(headerPositions)
        warning('Disk2_%03d.dat没有找到IMU帧头。',file_id);
        continue;
    end
    IMU120 = read_stdimu_120(filename_imu,headerPositions);
    if size(IMU120,2) < 9
        error('Disk2_%03d解析结果不足9列。',file_id);
    end
    Alignment120(k).imu_file = sprintf('Disk2_%03d.dat',file_id);
    Alignment120(k).imu_raw_all = IMU120;
    Alignment120(k).imu_machine_time_all = IMU120(:,7)/200;
    fprintf('  IMU：%d × %d\n',size(IMU120,1),size(IMU120,2));
end
%% ========================================================================
%% 8. Disk2多开机段匹配
% ========================================================================
fprintf('\n============================================================\n');
fprintf('7. Disk2多开机段匹配\n');
fprintf('============================================================\n');
for k = 1:2
    if ~isfield(Alignment120(k),'imu_raw_all') || isempty(Alignment120(k).imu_raw_all)
        continue;
    end
    IMU120 = Alignment120(k).imu_raw_all;
    t_imu = Alignment120(k).imu_machine_time_all;
    A0 = Alignment120(k).auax_anchor;
    t_auxa_machine = A0(1);
    auax_value = A0(12:14)';
    [seg_start,seg_end] = split_segments(t_imu,RESET_JUMP);
    n_seg = numel(seg_start);
    fprintf('\n%s：Disk2检测到%d个开机段\n',Alignment120(k).name,n_seg);
    MatchCandidate = struct([]);
    match_count = 0;
    preferred_seg = Alignment120(k).boot_segment_id;
    seg_order = 1:n_seg;
    if preferred_seg <= n_seg
        seg_order = [preferred_seg,seg_order(seg_order~=preferred_seg)];
    end
    for order_id = 1:numel(seg_order)
        seg_id = seg_order(order_id);
        i1 = seg_start(seg_id);
        i2 = seg_end(seg_id);
        idx_seg = i1:i2;
        local_mask = abs(t_imu(idx_seg)-t_auxa_machine) <= IMU_MATCH_TIME_WINDOW;
        idx_local = idx_seg(local_mask);
        fprintf('  IMU开机段%d：index %d~%d，run %.3f~%.3f',seg_id,i1,i2,t_imu(i1),t_imu(i2));
        if isempty(idx_local)
            fprintf('，430.01附近无数据\n');
            continue;
        end
        diff_value = IMU120(idx_local,1:3)-auax_value;
        value_score_all = sqrt(sum(diff_value.^2,2));
        [value_score,ii] = min(value_score_all);
        best_idx = idx_local(ii);
        machine_error = t_imu(best_idx)-t_auxa_machine;
        match_count = match_count+1;
        MatchCandidate(match_count).segment_id = seg_id;
        MatchCandidate(match_count).segment_start = i1;
        MatchCandidate(match_count).segment_end = i2;
        MatchCandidate(match_count).imu_index = best_idx;
        MatchCandidate(match_count).imu_machine_time = t_imu(best_idx);
        MatchCandidate(match_count).machine_error = machine_error;
        MatchCandidate(match_count).imu_value = IMU120(best_idx,1:3);
        MatchCandidate(match_count).value_score = value_score;
        MatchCandidate(match_count).same_boot_order = seg_id == preferred_seg;
        fprintf('\n    candidate：idx=%d, run=%.6f, dt=%+.6f, score=%.9e\n', ...
            best_idx,t_imu(best_idx),machine_error,value_score);
    end
    if isempty(MatchCandidate)
        error('%s：Disk2所有开机段中都没有430.01附近候选。',Alignment120(k).name);
    end
    same_id = find([MatchCandidate.same_boot_order]);
    if ~isempty(same_id)
        candidate_pool = same_id;
    else
        candidate_pool = 1:numel(MatchCandidate);
    end
    score_pool = [MatchCandidate(candidate_pool).value_score];
    [~,ii] = min(score_pool);
    best_candidate_id = candidate_pool(ii);
    M = MatchCandidate(best_candidate_id);
    Alignment120(k).imu_match_candidates = MatchCandidate;
    Alignment120(k).imu_match_candidate_id = best_candidate_id;
    Alignment120(k).imu_segment_id = M.segment_id;
    Alignment120(k).imu_segment_start = M.segment_start;
    Alignment120(k).imu_segment_end = M.segment_end;
    Alignment120(k).imu_match_index_all = M.imu_index;
    Alignment120(k).imu_match_machine_time = M.imu_machine_time;
    Alignment120(k).machine_time_error = M.machine_error;
    Alignment120(k).imu_match_score = M.value_score;
    Alignment120(k).imu_match_value = IMU120(M.imu_index,:);
    fprintf('\n最终匹配：\n');
    fprintf('  AUXA开机段：      %d\n',Alignment120(k).boot_segment_id);
    fprintf('  Disk2开机段：     %d\n',M.segment_id);
    fprintf('  Disk2 segment：   %d ~ %d\n',M.segment_start,M.segment_end);
    fprintf('  AUXA marker run： %.6f s\n',t_auxa_machine);
    fprintf('  IMU match run：   %.6f s\n',M.imu_machine_time);
    fprintf('  上机时间差：      %+.6f s\n',M.machine_error);
    fprintf('  三轴score：        %.9e\n',M.value_score);
    if abs(M.machine_error) > MACHINE_TIME_WARN
        warning('%s：最终匹配点上机时间差为%.6f s。',Alignment120(k).name,M.machine_error);
    end
end
%% ========================================================================
%% 9. 只对选中的Disk2开机段赋GPS时间
% ========================================================================
fprintf('\n============================================================\n');
fprintf('8. 仅对选中的Disk2开机段赋绝对时间\n');
fprintf('============================================================\n');
for k = 1:2
    if ~isfield(Alignment120(k),'imu_segment_start')
        continue;
    end
    i1 = Alignment120(k).imu_segment_start;
    i2 = Alignment120(k).imu_segment_end;
    IMU_seg = Alignment120(k).imu_raw_all(i1:i2,:);
    machine_seg = Alignment120(k).imu_machine_time_all(i1:i2);
    t0_gps = Alignment120(k).auxa_anchor_gps_time;
    t0_machine = Alignment120(k).imu_match_machine_time;
    gps_seg = t0_gps+(machine_seg-t0_machine);
    static_start_run = Alignment120(k).startup_run_time;
    static_start_gps = t0_gps+(static_start_run-Alignment120(k).auxa_anchor_machine_time);
    Alignment120(k).static_start_gps = static_start_gps;
    keep_static = gps_seg >= static_start_gps & gps_seg < t0_gps;
    IMU_static = IMU_seg(keep_static,:);
    machine_static = machine_seg(keep_static);
    gps_static = gps_seg(keep_static);
    if ~isempty(IMU_static)
        Alignment120(k).imu_static_raw = IMU_static;
        Alignment120(k).imu_static_machine_time = machine_static;
        Alignment120(k).imu_static_time_gps = gps_static;
        Alignment120(k).imu_static_time = [gps_static(:),machine_static(:)];
        IMU_static_gps = IMU_static;
        IMU_static_gps(:,8) = gps_static(:);
        Alignment120(k).imu_static_raw_with_gps = IMU_static_gps;
        Alignment120(k).imu120FRD_static = imuFUR2FRD(IMU_static_gps);
    else
        warning('%s：没有截取到静止IMU。',Alignment120(k).name);
    end
    keep_dynamic = gps_seg >= t0_gps & gps_seg <= Alignment120(k).compass_end;
    IMU_dynamic = IMU_seg(keep_dynamic,:);
    machine_dynamic = machine_seg(keep_dynamic);
    gps_dynamic = gps_seg(keep_dynamic);
    if isempty(IMU_dynamic)
        error('%s：没有截取到对准后IMU。',Alignment120(k).name);
    end
    Alignment120(k).imu_raw = IMU_dynamic;
    Alignment120(k).imu_machine_time = machine_dynamic;
    Alignment120(k).imu_time_gps = gps_dynamic;
    Alignment120(k).imu_time = [gps_dynamic(:),machine_dynamic(:)];
    IMU_dynamic_gps = IMU_dynamic;
    IMU_dynamic_gps(:,8) = gps_dynamic(:);
    Alignment120(k).imu_raw_with_gps = IMU_dynamic_gps;
    Alignment120(k).imu120FRD = imuFUR2FRD(IMU_dynamic_gps);
    time_error = max(abs(Alignment120(k).imu120FRD(:,1)-gps_dynamic(:)));
    Alignment120(k).frd_time_reorder_error = time_error;
    fprintf('\n%s\n',Alignment120(k).name);
    if ~isempty(IMU_static)
        fprintf('  静止IMU：%d历元，GPS %.6f ~ %.6f\n',size(IMU_static,1),gps_static(1),gps_static(end));
    end
    fprintf('  对准后IMU：%d历元，GPS %.6f ~ %.6f\n',size(IMU_dynamic,1),gps_dynamic(1),gps_dynamic(end));
    fprintf('  FRD时间核对：%.9e s\n',time_error);
end
%% ========================================================================
%% 10. AUXA IMU 与 Disk2 IMU 原始三轴对比
% ========================================================================
fprintf('\n============================================================\n');
fprintf('9. AUXA IMU 与 Disk2 IMU 三轴对比\n');
fprintf('============================================================\n');
for k = 1:2
    if ~isfield(Alignment120(k),'auax_static') || ...
            ~isfield(Alignment120(k),'auax') || ...
            ~isfield(Alignment120(k),'imu_static_raw') || ...
            ~isfield(Alignment120(k),'imu_raw')
        warning('%s：缺少AUXA/IMU静止或动态数据，跳过IMU对比图。',Alignment120(k).name);
        continue;
    end
    plot_auxa_disk2_imu_compare(Alignment120(k));
end
%% ========================================================================
%% 11. 时间覆盖图
% ========================================================================
figure('Name','120 AUXA / IMU / Compass时间覆盖','NumberTitle','off');
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
for k = 1:2
    nexttile;
    hold on;
    grid on;
    t0 = Alignment120(k).marker_time;
    if isfield(Alignment120(k),'auax_static_time_gps') && ~isempty(Alignment120(k).auax_static_time_gps)
        plot([Alignment120(k).auax_static_time_gps(1),Alignment120(k).auax_static_time_gps(end)]-t0,[4 4],'LineWidth',6);
    end
    if isfield(Alignment120(k),'imu_static_time_gps') && ~isempty(Alignment120(k).imu_static_time_gps)
        plot([Alignment120(k).imu_static_time_gps(1),Alignment120(k).imu_static_time_gps(end)]-t0,[3 3],'LineWidth',6);
    end
    if isfield(Alignment120(k),'auax') && ~isempty(Alignment120(k).auax)
        plot([Alignment120(k).auax(2,1),Alignment120(k).auax(2,end)]-t0,[2 2],'LineWidth',6);
    end
    if isfield(Alignment120(k),'imu_time_gps') && ~isempty(Alignment120(k).imu_time_gps)
        plot([Alignment120(k).imu_time_gps(1),Alignment120(k).imu_time_gps(end)]-t0,[1 1],'LineWidth',6);
    end
    xline(0,'--','430.01');
    xline(Compass{k}.time(1)-t0,':','Compass start');
    yticks([1 2 3 4]);
    yticklabels({'IMU对准后','AUXA对准后','IMU静止','AUXA静止'});
    xlabel('相对430.01对准时刻 / s');
    title(Alignment120(k).name);
end
%% ========================================================================
%% 12. 保存
% ========================================================================
save(save_file,'Alignment120','Candidate','Experiment','-v7.3');
fprintf('\n============================================================\n');
fprintf('120 AUXA + IMU数据保存完成\n');
fprintf('%s\n',save_file);
fprintf('============================================================\n');
%% ========================================================================
%% 13. 保存内容检查
% ========================================================================
for k = 1:2
    fprintf('\n---------------- %s ----------------\n',Alignment120(k).name);
    fprintf('Disk1 / Disk2：%03d\n',Alignment120(k).file_id);
    fprintf('AUXA开机段：%d\n',Alignment120(k).boot_segment_id);
    if isfield(Alignment120(k),'imu_segment_id')
        fprintf('IMU开机段： %d\n',Alignment120(k).imu_segment_id);
    end
    fprintf('对准GPS：%.6f\n',Alignment120(k).marker_time);
    if isfield(Alignment120(k),'auax_static')
        fprintf('静止AUXA：%d历元\n',size(Alignment120(k).auax_static,2));
    end
    if isfield(Alignment120(k),'auax')
        fprintf('对准后AUXA：%d历元，%.6f ~ %.6f\n',size(Alignment120(k).auax,2), ...
            Alignment120(k).auax(2,1),Alignment120(k).auax(2,end));
    end
    if isfield(Alignment120(k),'imu_static_raw')
        fprintf('静止IMU：%d × %d，%.6f ~ %.6f\n', ...
            size(Alignment120(k).imu_static_raw,1),size(Alignment120(k).imu_static_raw,2), ...
            Alignment120(k).imu_static_time_gps(1),Alignment120(k).imu_static_time_gps(end));
    end
    if isfield(Alignment120(k),'imu_raw')
        fprintf('对准后IMU：%d × %d，%.6f ~ %.6f\n', ...
            size(Alignment120(k).imu_raw,1),size(Alignment120(k).imu_raw,2), ...
            Alignment120(k).imu_time_gps(1),Alignment120(k).imu_time_gps(end));
    end
end
%% ========================================================================
% 局部函数
% ========================================================================
function [seg_start,seg_end] = split_segments(machine_time,reset_jump)
reset_idx = find(diff(machine_time) < reset_jump);
seg_start = [1;reset_idx(:)+1];
seg_end = [reset_idx(:);numel(machine_time)];
end
function startup_idx = find_startup_index(run_time,seg_start,marker_idx,startup_marker,startup_tol)
idx = seg_start:marker_idx;
mask = false(size(idx));
for value = startup_marker
    mask = mask | abs(run_time(idx)-value) <= startup_tol;
end
id = idx(mask);
if ~isempty(id)
    startup_idx = id(1);
else
    startup_idx = seg_start;
end
end
function str = gpssec_to_utc_string(gps_week,gps_second)
GPS_UTC_LEAP = 18;
epoch = datetime(1980,1,6,0,0,0,'TimeZone','UTC');
t = epoch+days(gps_week*7)+seconds(gps_second-GPS_UTC_LEAP);
t.Format = 'yyyy-MM-dd HH:mm:ss.SSS';
str = char(t);
end
function plot_auxa_disk2_imu_compare(A)
AUXA = [A.auax_static,A.auax];
t_auxa = AUXA(1,:)';
imu_auxa = AUXA(12:14,:)';
IMU = [A.imu_static_raw;A.imu_raw];
t_imu = IMU(:,7)/200;
imu_disk2 = IMU(:,1:3);
valid_auxa = isfinite(t_auxa) & all(isfinite(imu_auxa),2);
t_auxa = t_auxa(valid_auxa);
imu_auxa = imu_auxa(valid_auxa,:);
valid_imu = isfinite(t_imu) & all(isfinite(imu_disk2),2);
t_imu = t_imu(valid_imu);
imu_disk2 = imu_disk2(valid_imu,:);
if isempty(t_auxa) || isempty(t_imu)
    warning('%s：IMU对比数据为空。',A.name);
    return;
end
[t_auxa,id] = sort(t_auxa);
imu_auxa = imu_auxa(id,:);
[t_imu,id] = sort(t_imu);
imu_disk2 = imu_disk2(id,:);
t0 = A.auxa_anchor_machine_time;
ta = t_auxa-t0;
ti = t_imu-t0;
MAX_PLOT_POINTS = 150000;
id_auxa = local_plot_index(length(ta),MAX_PLOT_POINTS);
id_imu = local_plot_index(length(ti),MAX_PLOT_POINTS);
axis_name = {'Axis 1','Axis 2','Axis 3'};
figure('Name',sprintf('%s AUXA-Disk2 IMU三轴对比',A.name),'NumberTitle','off');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
for j = 1:3
    nexttile;
    hold on;
    grid on;
    plot(ta(id_auxa),imu_auxa(id_auxa,j),'LineWidth',1.0,'DisplayName','AUXA IMU');
    plot(ti(id_imu),imu_disk2(id_imu,j),'LineWidth',0.7,'DisplayName','Disk2 IMU');
    xline(0,'--','430.01');
    ylabel(axis_name{j});
    if j == 1
        title(sprintf('%s：AUXA(12:14) 与 Disk2 IMU(1:3)',A.name));
        legend('Location','best');
    end
    if j == 3
        xlabel('相对430.01上机时间 / s');
    end
end
LOCAL_WINDOW = 5.0;
id_auxa_local = abs(ta) <= LOCAL_WINDOW;
id_imu_local = abs(ti) <= LOCAL_WINDOW;
if any(id_auxa_local) && any(id_imu_local)
    figure('Name',sprintf('%s 430.01附近IMU对比',A.name),'NumberTitle','off');
    tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
    for j = 1:3
        nexttile;
        hold on;
        grid on;
        plot(ta(id_auxa_local),imu_auxa(id_auxa_local,j),'o-','LineWidth',1.0,'MarkerSize',3,'DisplayName','AUXA IMU');
        plot(ti(id_imu_local),imu_disk2(id_imu_local,j),'.-','LineWidth',0.8,'MarkerSize',6,'DisplayName','Disk2 IMU');
        xline(0,'--','430.01');
        ylabel(axis_name{j});
        if j == 1
            title(sprintf('%s：430.01前后±%.1f s局部检查',A.name,LOCAL_WINDOW));
            legend('Location','best');
        end
        if j == 3
            xlabel('相对430.01上机时间 / s');
        end
    end
end
[~,ia] = min(abs(t_auxa-t0));
[~,ii] = min(abs(t_imu-t0));
fprintf('\n%s AUXA / Disk2 IMU对比：\n',A.name);
fprintf('  AUXA最近marker：run %.6f，IMU %.9f %.9f %.9f\n', ...
    t_auxa(ia),imu_auxa(ia,1),imu_auxa(ia,2),imu_auxa(ia,3));
fprintf('  Disk2最近marker：run %.6f，IMU %.9f %.9f %.9f\n', ...
    t_imu(ii),imu_disk2(ii,1),imu_disk2(ii,2),imu_disk2(ii,3));
fprintf('  上机时间差：%+.6f s\n',t_imu(ii)-t_auxa(ia));
fprintf('  三轴差：%.9e %.9e %.9e\n',imu_disk2(ii,:)-imu_auxa(ia,:));
end
function id = local_plot_index(n,max_points)
if n <= max_points
    id = 1:n;
else
    step = ceil(n/max_points);
    id = 1:step:n;
end
end