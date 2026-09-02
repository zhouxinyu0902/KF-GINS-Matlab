clear;
clc;
close all;

%% ========================================================================
% 2025-12-07
%
% 目的：
%
% 1. 根据 Compass + 830 两段有效试验时间
% 2. 现场读取 Disk1_000 ~ Disk1_007 AUAX
% 3. 找到试验时间附近的精对准结束 / 导航开始历元
%
%       0x5820 精对准
%           ↓
%       430.010 / 430.015
%           ↓
%       0x4300 导航
%
% 4. 保存导航起始点对应 AUAX 信息
% 5. 读取对应 Disk2 原始 IMU
% 6. 使用：
%
%       AUAX(12:14)
%           ↕
%       IMU(:,1:3)
%
%    自动搜索最佳匹配点
%
% 7. 输出匹配点附近数据，进一步检查 AUAX(12:18)
%
% ========================================================================


%% ========================================================================
%% 0. 路径
% ========================================================================

raw_dir = ...
    'D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1207-longtime\raw';

temp_dir = ...
    'D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1207-longtime\temp';


compass_file = ...
    fullfile(temp_dir,'compass.mat');

ref830_file = ...
    fullfile(temp_dir,'ref830_1207.mat');


save_file = ...
    fullfile(temp_dir,'alignment120_imu_1207.mat');


%% ========================================================================
%% 1. 参数
% ========================================================================

GPS_WEEK = 2396;

% 保留1207旧程序时间处理
AUAX_TIME_OFFSET = 18;

MIN_VALID_GPS = 100;


% AUAX状态
STATUS_FINE = hex2dec('5820');    % 22560
STATUS_NAV  = hex2dec('4300');    % 17152


% AUAX第1行的关键时间
ALIGN_MARK = [430.010,430.015];

ALIGN_MARK_TOL = 0.002;


% 在实验起点前多长时间寻找对准事件
SEARCH_BACK = 30*60;

% 允许稍晚于实验时间
SEARCH_FORWARD = 120;


% -------------------------------------------------------------------------
% IMU匹配
%
% AUAX(12:14) ↔ IMU(:,1:3)
%
% 先使用这3维作为可靠匹配依据。
% -------------------------------------------------------------------------

AUAX_MATCH_ROWS = 12:14;

IMU_MATCH_COLS = 1:3;


% 找到粗略位置后，打印前后多少IMU点
IMU_PRINT_RADIUS = 3;


%% ========================================================================
%% 2. 读取 Compass + 830
% ========================================================================

load(compass_file,'Compass');
load(ref830_file,'Ref830');


Experiment = struct();


fprintf('\n');
fprintf('============================================================\n');
fprintf('1. 两次实验时间\n');
fprintf('============================================================\n');


for k = 1:2

    compass_start = Compass{k}.time(1);
    compass_end   = Compass{k}.time(end);

    ref_start = Ref830{k}.time(1);
    ref_end   = Ref830{k}.time(end);


    %% 公共有效区间

    Experiment(k).start = ...
        max(compass_start,ref_start);

    Experiment(k).end = ...
        min(compass_end,ref_end);

    Experiment(k).name = ...
        Compass{k}.name;


    fprintf('\n%s\n',Experiment(k).name);

    fprintf('Compass：%.3f ~ %.3f\n', ...
        compass_start,compass_end);

    fprintf('830：    %.3f ~ %.3f\n', ...
        ref_start,ref_end);

    fprintf('参考起点：%.3f\n', ...
        Experiment(k).start);

end


%% ========================================================================
%% 3. 读取全部 AUAX
%
% 注意：
% 不把所有数据强行拼成连续导航。
%
% 这里只用于寻找对准事件。
% ========================================================================

AUAX_file = cell(1,8);


fprintf('\n');
fprintf('============================================================\n');
fprintf('2. 现场读取 AUAX\n');
fprintf('============================================================\n');


for file_id = 0:7

    filename = ...
        fullfile( ...
        raw_dir, ...
        sprintf('Disk1_%03d.dat',file_id));


    if ~exist(filename,'file')

        warning('不存在：%s',filename);

        continue;

    end


    fprintf('\n读取 Disk1_%03d.dat\n',file_id);


    A = read_auax_120(filename);


    if isempty(A)
        continue;
    end


    %% GPS时间修正

    A(2,:) = ...
        A(2,:) + AUAX_TIME_OFFSET;


    %% 只删除GPS明显无效数据

    valid = ...
        isfinite(A(2,:)) & ...
        A(2,:) > MIN_VALID_GPS;


    A = A(:,valid);


    AUAX_file{file_id+1} = A;


    if ~isempty(A)

        fprintf('  %d历元\n',size(A,2));

        fprintf('  GPS %.3f ~ %.3f\n', ...
            min(A(2,:)), ...
            max(A(2,:)));

    end

end


%% ========================================================================
%% 4. 搜索所有 AUAX 对准 → 导航事件
% ========================================================================

Candidate = struct([]);

nCandidate = 0;


for file_id = 0:7

    A = AUAX_file{file_id+1};


    if isempty(A)
        continue;
    end


    status_row = size(A,1)-2;

    run_time = A(1,:);
    gps_time = A(2,:);
    status   = A(status_row,:);


    %% --------------------------------------------------------------------
    % 找430.010 / 430.015
    % ---------------------------------------------------------------------

    mark = false(size(run_time));


    for v = ALIGN_MARK

        mark = mark | ...
            abs(run_time-v) <= ALIGN_MARK_TOL;

    end


    idx_mark = find(mark);


    % 连续重复只留第一个
    if ~isempty(idx_mark)

        idx_mark = ...
            idx_mark([true,diff(idx_mark)>1]);

    end


    %% --------------------------------------------------------------------
    % 分析每一个marker
    % ---------------------------------------------------------------------

    for m = 1:length(idx_mark)

        im = idx_mark(m);


        %% marker前最近一个精对准状态

        idx_fine = find( ...
            status(1:im) == STATUS_FINE, ...
            1,'last');


        if isempty(idx_fine)
            continue;
        end


        %% marker之后第一个导航状态

        idx_nav_rel = find( ...
            status(im:end) == STATUS_NAV, ...
            1,'first');


        if isempty(idx_nav_rel)
            continue;
        end


        idx_nav = ...
            im + idx_nav_rel - 1;


        %% ----------------------------------------------------------------
        % 防止精对准和导航相隔过远
        % -----------------------------------------------------------------

        if gps_time(idx_nav)-gps_time(idx_fine) > 60
            continue;
        end


        %% 保存候选

        nCandidate = nCandidate+1;


        Candidate(nCandidate).file_id = ...
            file_id;

        Candidate(nCandidate).mark_index = ...
            im;

        Candidate(nCandidate).fine_end_index = ...
            idx_fine;

        Candidate(nCandidate).nav_index = ...
            idx_nav;


        Candidate(nCandidate).mark_run_time = ...
            run_time(im);

        Candidate(nCandidate).mark_gps = ...
            gps_time(im);

        Candidate(nCandidate).fine_end_gps = ...
            gps_time(idx_fine);

        Candidate(nCandidate).nav_gps = ...
            gps_time(idx_nav);

    end

end


%% ========================================================================
%% 5. 输出全部候选
% ========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('3. AUAX 对准候选\n');
fprintf('============================================================\n');


for i = 1:length(Candidate)

    fprintf('\n候选 %d\n',i);

    fprintf('  Disk1_%03d\n', ...
        Candidate(i).file_id);

    fprintf('  430.01：%.3f\n', ...
        Candidate(i).mark_gps);

    fprintf('  精对准结束：%.3f\n', ...
        Candidate(i).fine_end_gps);

    fprintf('  导航开始：%.3f\n', ...
        Candidate(i).nav_gps);

end


%% ========================================================================
%% 6. 根据 Compass + 830 时间选择下午 / 晚上的候选事件
% ========================================================================

Alignment120 = struct();


candidate_time = ...
    [Candidate.nav_gps];


for k = 1:2

    target = ...
        Experiment(k).start;


    %% 搜索合理范围内候选

    idx = find( ...
        candidate_time >= target-SEARCH_BACK & ...
        candidate_time <= target+SEARCH_FORWARD);


    if isempty(idx)

        error( ...
            '%s附近没有找到120对准事件。', ...
            Experiment(k).name);

    end


    %% 优先选实验起点之前最近的一次

    idx_before = ...
        idx(candidate_time(idx) <= target);


    if ~isempty(idx_before)

        [~,ii] = min( ...
            target-candidate_time(idx_before));

        selected = ...
            idx_before(ii);

    else

        [~,ii] = min( ...
            abs(candidate_time(idx)-target));

        selected = ...
            idx(ii);

    end


    C = Candidate(selected);


    %% --------------------------------------------------------------------
    % 保存基本信息
    % ---------------------------------------------------------------------

    Alignment120(k).name = ...
        Experiment(k).name;

    Alignment120(k).file_id = ...
        C.file_id;

    Alignment120(k).candidate_id = ...
        selected;


    Alignment120(k).fine_end_gps = ...
        C.fine_end_gps;

    Alignment120(k).nav_gps = ...
        C.nav_gps;


    %% --------------------------------------------------------------------
    % 关键：
    % 保存导航开始点附近的AUAX信息
    % ---------------------------------------------------------------------

    A = AUAX_file{C.file_id+1};


    nav_index = ...
        C.nav_index;


    Alignment120(k).auax_nav_index = ...
        nav_index;


    %% 导航开始单点

    Alignment120(k).auax_nav = ...
        A(:,nav_index);


    %% 保存前后若干点，后面检查方便

    idx1 = max(1,nav_index-5);

    idx2 = min(size(A,2),nav_index+5);


    Alignment120(k).auax_near = ...
        A(:,idx1:idx2);


    fprintf('\n');
    fprintf('============================================================\n');

    fprintf('%s\n',Experiment(k).name);

    fprintf('============================================================\n');


    fprintf('Disk1_%03d.dat\n', ...
        C.file_id);


    fprintf('实验参考起点：%.3f\n',target);

    fprintf('120导航开始： %.3f\n',C.nav_gps);

    fprintf('相差：%.3f s\n', ...
        target-C.nav_gps);


    fprintf('\n导航开始 AUAX：\n');

    fprintf('AUAX index = %d\n',nav_index);

    fprintf('AUAX(1:2)：\n');

    fprintf('  %.6f  %.6f\n', ...
        A(1,nav_index), ...
        A(2,nav_index));


    fprintf('AUAX(12:18)：\n');

    fprintf('  ');

    fprintf('%.9f  ', ...
        A(12:18,nav_index));

    fprintf('\n');


    fprintf('状态字：%s\n', ...
        dec2hex(A(status_row,nav_index)));

end


%% ========================================================================
%% 7. 根据AUAX对应文件读取原始IMU
%
% 目前已知：
%
% 下午 / 晚上原代码对应：
%
% Disk1_003 → Disk2_003
% Disk1_006 → Disk2_006
%
% 因此这里直接按相同文件号现场读取。
% ========================================================================

frameHeader = [235,144,32];


for k = 1:2

    file_id = ...
        Alignment120(k).file_id;


    filename_imu = ...
        fullfile( ...
        raw_dir, ...
        sprintf('Disk2_%03d.dat',file_id));


    fprintf('\n');
    fprintf('============================================================\n');

    fprintf('读取 %s 对应IMU\n', ...
        Alignment120(k).name);

    fprintf('============================================================\n');

    fprintf('%s\n',filename_imu);


    if ~exist(filename_imu,'file')

        error('IMU文件不存在：%s',filename_imu);

    end


    %% --------------------------------------------------------------------
    % 原始IMU读取
    % ---------------------------------------------------------------------

    fid = fopen(filename_imu,'rb');

    allData = fread(fid,inf,'uint8');

    fclose(fid);


    headerPositions = find( ...
        allData(1:end-2) == frameHeader(1) & ...
        allData(2:end-1) == frameHeader(2) & ...
        allData(3:end) == frameHeader(3));


    IMU = ...
        read_stdimu_120( ...
        filename_imu, ...
        headerPositions);


    Alignment120(k).imu_file = ...
        sprintf('Disk2_%03d.dat',file_id);


    Alignment120(k).imu = ...
        IMU;


    fprintf('解析IMU：%d历元\n',size(IMU,1));


    %% ====================================================================
    % 8. AUAX ↔ IMU 自动匹配
    %
    % 使用：
    %
    % AUAX(12:14, nav_index)
    %
    % 和
    %
    % IMU(:,1:3)
    %
    % 做差。
    %
    % =====================================================================

    auax_target = ...
        Alignment120(k).auax_nav( ...
        AUAX_MATCH_ROWS);


    %% --------------------------------------------------------------------
    % 不直接使用绝对差：
    %
    % 三个量的尺度可能不同，
    % 先按每维尺度归一化。
    % ---------------------------------------------------------------------

    imu_data = ...
        IMU(:,IMU_MATCH_COLS);


    scale = ...
        std(imu_data,0,1,'omitnan');


    scale(scale == 0 | ~isfinite(scale)) = 1;


    diff_norm = ...
        (imu_data - auax_target(:)') ...
        ./ scale;


    score = ...
        sqrt(sum(diff_norm.^2,2));


    %% --------------------------------------------------------------------
    % 最小误差点
    % ---------------------------------------------------------------------

    [best_score,best_idx] = ...
        min(score);


    Alignment120(k).imu_match_index = ...
        best_idx;


    Alignment120(k).imu_match_score = ...
        best_score;


    Alignment120(k).imu_match = ...
        IMU(best_idx,:);


    %% ====================================================================
    % 9. 输出匹配结果
    % =====================================================================

    fprintf('\nAUAX / IMU 自动匹配\n');


    fprintf('AUAX index：%d\n', ...
        Alignment120(k).auax_nav_index);


    fprintf('IMU index ：%d\n', ...
        best_idx);


    fprintf('IMU内部时间：%.6f s\n', ...
        IMU(best_idx,7)/200);


    fprintf('\nAUAX(12:14)：\n');

    fprintf('  %.9f %.9f %.9f\n', ...
        auax_target);


    fprintf('IMU(1:3)：\n');

    fprintf('  %.9f %.9f %.9f\n', ...
        IMU(best_idx,1:3));


    fprintf('差值：\n');

    fprintf('  %.9e %.9e %.9e\n', ...
        IMU(best_idx,1:3) ...
        - auax_target(:)');


    fprintf('\nAUAX(12:18)：\n');

    fprintf('  ');

    fprintf('%.9f  ', ...
        Alignment120(k).auax_nav(12:18));

    fprintf('\n');


    %% --------------------------------------------------------------------
    % 打印IMU最佳点前后几帧
    % ---------------------------------------------------------------------

    i1 = ...
        max(1,best_idx-IMU_PRINT_RADIUS);

    i2 = ...
        min(size(IMU,1),best_idx+IMU_PRINT_RADIUS);


    fprintf('\n匹配点附近IMU：\n');

    fprintf( ...
        'index       col1            col2            col3          time/200\n');


    for ii = i1:i2

        fprintf( ...
            '%6d  %14.9f  %14.9f  %14.9f  %12.6f', ...
            ii, ...
            IMU(ii,1), ...
            IMU(ii,2), ...
            IMU(ii,3), ...
            IMU(ii,7)/200);


        if ii == best_idx

            fprintf('   <-- MATCH');

        end

        fprintf('\n');

    end

end


%% ========================================================================
%% 10. 最终保存
%
% 不建议把完整IMU重复存进去。
%
% 保存：
%
% 对准信息
% AUAX关键点
% IMU匹配关键点
%
% ========================================================================

AlignmentResult = Alignment120;


% 删除体量很大的完整IMU
for k = 1:length(AlignmentResult)

    if isfield(AlignmentResult(k),'imu')

        AlignmentResult(k) = ...
            rmfield(AlignmentResult(k),'imu');

    end

end


save( ...
    save_file, ...
    'AlignmentResult', ...
    'Experiment');


fprintf('\n');
fprintf('============================================================\n');

fprintf('保存完成：\n%s\n',save_file);

fprintf('============================================================\n');