clear;
clc;
close all;
%% ========================================================================
% 2025-12-07
% 120 AUAX + 原始IMU时间统一（新版）
%
% 核心时间逻辑：
%
% 1. 先在AUXA中找到 430.010 / 430.015 对准关键时刻；
% 2. 该AUXA历元同时具有：
%       A(1,:)      上机时间
%       A(2,:)      GPS周秒
%       A(12:14,:)  对应IMU三轴数据
% 3. 用AUXA(12:14)在原始IMU(:,1:3)中寻找同一物理历元；
% 4. 核对：
%       AUXA上机时间 A(1,anchor_idx)
%       IMU 上机时间 IMU(best_idx,7)/200
% 5. 确认后使用：
%
%       GPS时间 = IMU上机时间 + (AUXA_GPS - AUXA上机时间)
%
%    给整段原始IMU赋GPS绝对时间。
%
% 不使用任何人为420 s补偿。
%
% 根据 Compass + 830 有效实验时间，寻找对应的 120 AUAX 对准/导航起点
%
% 输入：
%
% temp\compass_1207.mat
%       Compass{1} : 下午
%       Compass{2} : 晚上
%
% temp\ref830_1207.mat
%       Ref830{1} : 下午
%       Ref830{2} : 晚上
%
%
% 120 AUAX：
%
% raw\Disk1_000.dat ~ Disk1_007.dat
%
%
% 判断依据：
%
% 1. Compass + 830 确定实验公共有效时间起点
%
% 2. 在该时间之前寻找最近的 AUAX 对准/导航转换
%
% 3. 候选必须同时满足：
%
%       AUAX状态：
%           0x5820  精对准
%              ↓
%           0x4300  导航
%
%       AUAX上级时间：
%           430.010 或 430.015 附近
%
%
% 输出：
%
% Alignment120(1)   下午
% Alignment120(2)   晚上
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
%% ========================================================================
%% 1. 参数
% ========================================================================
GPS_WEEK = 2396;
% -------------------------------------------------------------------------
% AUAX 时间处理
%
% 保持1207原程序：
%
% AUAX(2,:) = AUAX(2,:) + 18;
%
% 原始GPS无效时一般为0：
%
% 0 + 18 = 18
%
% 因此：
%
% AUAX(2,:) > 100
%
% 才认为时间有效。
% -------------------------------------------------------------------------
AUAX_TIME_OFFSET = 18;
MIN_VALID_TIME = 100;
%% ------------------------------------------------------------------------
% 状态字
% -------------------------------------------------------------------------
STATUS_FINE = hex2dec('5820');     % 22560
STATUS_NAV  = hex2dec('4300');     % 17152
%% ------------------------------------------------------------------------
% 对准完成附近的上级时间
% -------------------------------------------------------------------------
ALIGN_MARKER = [430.010,430.015];
% 浮点误差容差
MARKER_TOL = 0.002;
%% ------------------------------------------------------------------------
% 搜索范围
%
% 以 Compass + 830 公共数据开始时间为参考，
% 向前搜索30 min。
%
% 正常情况下430 s对准完成后很快进入正式实验，
% 因此30 min足够宽松。
% -------------------------------------------------------------------------
SEARCH_BACK_TIME = 30*60;
% 允许导航开始稍微晚于Compass/830共同开始时间
SEARCH_FORWARD_TIME = 60;
%% ------------------------------------------------------------------------
% 430.01附近寻找状态切换的时间范围
% -------------------------------------------------------------------------
MARKER_STATUS_WINDOW = 20;

%% ------------------------------------------------------------------------
% AUXA上机时间与原始IMU上机时间一致性检查
%
% 这里只报警，不自动补偿。
% 120 IMU为100 Hz时，正常应只相差很小的采样级误差。
% -------------------------------------------------------------------------
MACHINE_TIME_WARN = 0.10;      % s
%% ========================================================================
%% 2. 加载 Compass 和 830
% ========================================================================
fprintf('\n');
fprintf('============================================================\n');
fprintf('1. 读取 Compass + 830 实验时间\n');
fprintf('============================================================\n');
load(compass_file,'Compass');
load(ref830_file,'Ref830');
if numel(Compass) ~= 2 || numel(Ref830) ~= 2
    error('Compass和Ref830均应包含下午、晚上两段数据。');
end
Experiment = struct();
for k = 1:2
    %% Compass
    compass_start = ...
        Compass{k}.time(1);
    compass_end = ...
        Compass{k}.time(end);
    %% 830
    ref_start = ...
        Ref830{k}.time(1);
    ref_end = ...
        Ref830{k}.time(end);
    %% ---------------------------------------------------------------
    % 公共有效时间
    %
    % 后面寻找120导航起点时主要以这个起点作为参考。
    % ---------------------------------------------------------------
    common_start = ...
        max(compass_start,ref_start);
    common_end = ...
        min(compass_end,ref_end);
    if common_end <= common_start
        error( ...
            '实验%d：Compass与830没有公共时间范围。',k);
    end
    Experiment(k).name = ...
        Compass{k}.name;
    Experiment(k).compass_range = ...
        [compass_start,compass_end];
    Experiment(k).ref830_range = ...
        [ref_start,ref_end];
    Experiment(k).common_range = ...
        [common_start,common_end];
    fprintf('\n');
    fprintf('---------------- 实验 %d：%s ----------------\n', ...
        k,Experiment(k).name);
    fprintf('Compass ：%.3f ~ %.3f\n', ...
        compass_start,compass_end);
    fprintf('830     ：%.3f ~ %.3f\n', ...
        ref_start,ref_end);
    fprintf('公共区间：%.3f ~ %.3f\n', ...
        common_start,common_end);
    fprintf('公共起点UTC：%s\n', ...
        gpssec_to_utc_string( ...
        GPS_WEEK,common_start));
end
%% ========================================================================
%% 3. 现场读取 Disk1_000 ~ Disk1_007
% ========================================================================
fprintf('\n');
fprintf('============================================================\n');
fprintf('2. 现场读取全部 120 AUAX\n');
fprintf('============================================================\n');
AUAX_file = cell(1,8);
for file_id = 0:7
    filename = ...
        fullfile( ...
        raw_dir, ...
        sprintf('Disk1_%03d.dat',file_id));
    fprintf('\n读取 Disk1_%03d.dat\n',file_id);
    if ~exist(filename,'file')
        warning('文件不存在：%s',filename);
        continue;
    end
    A = read_auax_120(filename);
    if isempty(A)
        fprintf('  无解析数据。\n');
        continue;
    end
    %% ---------------------------------------------------------------
    % 时间处理
    % ---------------------------------------------------------------
    A(2,:) = ...
        A(2,:) + AUAX_TIME_OFFSET;
    %% ---------------------------------------------------------------
    % 当前只删除时间无效的数据
    %
    % 不根据位置删数据。
    % 因为这里只分析：
    %
    % 时间
    % 状态字
    % 430.01
    %
    % ---------------------------------------------------------------
    valid_time = ...
        isfinite(A(2,:)) & ...
        A(2,:) > MIN_VALID_TIME;
    A = A(:,valid_time);
    if isempty(A)
        fprintf('  无有效GPS时间数据。\n');
        continue;
    end
    AUAX_file{file_id+1} = A;
    fprintf('  有效历元：%d\n',size(A,2));
    fprintf('  GPS：%.3f ~ %.3f\n', ...
        min(A(2,:)), ...
        max(A(2,:)));
end
%% ========================================================================
%% 4. 每个文件独立寻找“对准 → 导航”事件
%
% 不直接先把所有文件硬拼接。
%
% 因为AUAX存在数据缺失，
% 每个文件内部判断状态变化更加可靠。
% ========================================================================
Candidate = struct([]);
candidate_count = 0;
fprintf('\n');
fprintf('============================================================\n');
fprintf('3. 搜索 AUAX 对准/导航候选事件\n');
fprintf('============================================================\n');
for file_id = 0:7
    A = AUAX_file{file_id+1};
    if isempty(A)
        continue;
    end
    status_row = size(A,1)-2;
    run_time = A(1,:);
    gps_time = A(2,:);
    status = A(status_row,:);
    %% ====================================================================
    % 4.1 找430.010 / 430.015
    % =====================================================================
    marker_mask = false(size(run_time));
    for value = ALIGN_MARKER
        marker_mask = ...
            marker_mask | ...
            abs(run_time-value) <= MARKER_TOL;
    end
    marker_idx = find(marker_mask);
    %% 连续重复点只保留第一个
    if ~isempty(marker_idx)
        keep = ...
            [true,diff(marker_idx)>1];
        marker_idx = ...
            marker_idx(keep);
    end
    if isempty(marker_idx)
        continue;
    end
    fprintf('\nDisk1_%03d：检测到 %d 个430.01附近标志\n', ...
        file_id,length(marker_idx));
    %% ====================================================================
    % 4.2 对每个430.01标志检查状态关系
    % =====================================================================
    for m = 1:length(marker_idx)
        im = marker_idx(m);
        tm = gps_time(im);
        %% ---------------------------------------------------------------
        % 在marker之前寻找最近的精对准点
        % ---------------------------------------------------------------
        fine_candidate = find( ...
            status == STATUS_FINE & ...
            gps_time <= tm & ...
            gps_time >= tm-MARKER_STATUS_WINDOW);
        %% ---------------------------------------------------------------
        % 在marker附近寻找第一个导航状态
        % ---------------------------------------------------------------
        nav_candidate = find( ...
            status == STATUS_NAV & ...
            gps_time >= tm-MARKER_STATUS_WINDOW & ...
            gps_time <= tm+MARKER_STATUS_WINDOW);
        if isempty(fine_candidate) || ...
                isempty(nav_candidate)
            fprintf( ...
                '  marker %.3f：附近没有完整5820→4300关系，跳过\n', ...
                tm);
            continue;
        end
        %% ---------------------------------------------------------------
        % 最后一个精对准点
        % ---------------------------------------------------------------
        idx_fine_end = ...
            fine_candidate(end);
        %% ---------------------------------------------------------------
        % 第一个位于精对准之后的导航点
        % ---------------------------------------------------------------
        nav_after_fine = ...
            nav_candidate( ...
            nav_candidate > idx_fine_end);
        if isempty(nav_after_fine)
            continue;
        end
        idx_nav_start = ...
            nav_after_fine(1);
        %% =================================================================
        % 构成一个候选对准事件
        % ==================================================================
        candidate_count = ...
            candidate_count+1;
        Candidate(candidate_count).file_id = ...
            file_id;
        Candidate(candidate_count).marker_index = ...
            im;
        Candidate(candidate_count).marker_run_time = ...
            run_time(im);
        Candidate(candidate_count).marker_time = ...
            tm;
        Candidate(candidate_count).fine_end_index = ...
            idx_fine_end;
        Candidate(candidate_count).fine_end_time = ...
            gps_time(idx_fine_end);
        Candidate(candidate_count).nav_start_index = ...
            idx_nav_start;
        Candidate(candidate_count).nav_start_time = ...
            gps_time(idx_nav_start);
        Candidate(candidate_count).fine_to_nav = ...
            gps_time(idx_nav_start) ...
            - gps_time(idx_fine_end);
        Candidate(candidate_count).marker_to_nav = ...
            gps_time(idx_nav_start) ...
            - tm;
        %% ---------------------------------------------------------------
        % UTC显示
        % ---------------------------------------------------------------
        Candidate(candidate_count).nav_start_utc = ...
            gpssec_to_utc_string( ...
            GPS_WEEK, ...
            gps_time(idx_nav_start));
        fprintf('\n');
        fprintf('  候选 %d\n',candidate_count);
        fprintf('    marker运行时间：%.3f\n', ...
            run_time(im));
        fprintf('    marker GPS：%.3f\n',tm);
        fprintf('    精对准结束：%.3f\n', ...
            gps_time(idx_fine_end));
        fprintf('    导航开始：%.3f\n', ...
            gps_time(idx_nav_start));
        fprintf('    精对准→导航：%.3f s\n', ...
            Candidate(candidate_count).fine_to_nav);
        fprintf('    UTC：%s\n', ...
            Candidate(candidate_count).nav_start_utc);
    end
end
if isempty(Candidate)
    error('没有找到满足430.01 + 5820→4300条件的候选事件。');
end
%% ========================================================================
%% 5. 根据Compass + 830时间匹配下午和晚上
% ========================================================================
Alignment120 = struct();
fprintf('\n');
fprintf('============================================================\n');
fprintf('4. 根据实验时间匹配120对准事件\n');
fprintf('============================================================\n');
candidate_nav_time = ...
    [Candidate.nav_start_time];
for k = 1:2
    target_time = ...
        Experiment(k).common_range(1);
    %% --------------------------------------------------------------------
    % 候选搜索区间
    %
    % 正常情况下120导航开始应略早于：
    %
    % Compass + 830公共有效数据起点
    %
    % ---------------------------------------------------------------------
    search_start = ...
        target_time-SEARCH_BACK_TIME;
    search_end = ...
        target_time+SEARCH_FORWARD_TIME;
    candidate_valid = ...
        candidate_nav_time >= search_start & ...
        candidate_nav_time <= search_end;
    candidate_id = ...
        find(candidate_valid);
    fprintf('\n');
    fprintf('---------------- %s ----------------\n', ...
        Experiment(k).name);
    fprintf('目标公共起点：%.3f\n',target_time);
    fprintf('目标UTC：%s\n', ...
        gpssec_to_utc_string( ...
        GPS_WEEK,target_time));
    if isempty(candidate_id)
        warning( ...
            '%s：附近没有找到120对准事件。', ...
            Experiment(k).name);
        continue;
    end
    %% --------------------------------------------------------------------
    % 优先选择：
    %
    % 导航开始时间 <= 实验公共起点
    %
    % 且距离公共起点最近
    %
    % ---------------------------------------------------------------------
    before_id = ...
        candidate_id( ...
        candidate_nav_time(candidate_id) <= target_time);
    if ~isempty(before_id)
        [~,ii] = min( ...
            target_time ...
            - candidate_nav_time(before_id));
        selected_id = ...
            before_id(ii);
    else
        %% 如果没有早于目标时间的事件
        %  则取绝对时间最近的
        [~,ii] = min( ...
            abs( ...
            candidate_nav_time(candidate_id) ...
            - target_time));
        selected_id = ...
            candidate_id(ii);
    end
    C = Candidate(selected_id);
    %% --------------------------------------------------------------------
    % 保存
    % ---------------------------------------------------------------------
    Alignment120(k).name = ...
        Experiment(k).name;
    Alignment120(k).candidate_id = ...
        selected_id;
    Alignment120(k).file_id = ...
        C.file_id;
    Alignment120(k).marker_run_time = ...
        C.marker_run_time;
    Alignment120(k).marker_time = ...
        C.marker_time;
    Alignment120(k).fine_end_time = ...
        C.fine_end_time;
    Alignment120(k).nav_start_time = ...
        C.nav_start_time;
    Alignment120(k).nav_start_utc = ...
        C.nav_start_utc;
    Alignment120(k).target_time = ...
        target_time;
    Alignment120(k).target_utc = ...
        gpssec_to_utc_string( ...
        GPS_WEEK,target_time);
    Alignment120(k).nav_to_target = ...
        target_time-C.nav_start_time;
    %% --------------------------------------------------------------------
    % 打印
    % ---------------------------------------------------------------------
    fprintf('\n最终匹配：\n');
    fprintf('  120文件：Disk1_%03d.dat\n', ...
        C.file_id);
    fprintf('  430.01位置：%.3f\n', ...
        C.marker_run_time);
    fprintf('  精对准结束GPS：%.3f\n', ...
        C.fine_end_time);
    fprintf('  导航开始GPS：  %.3f\n', ...
        C.nav_start_time);
    fprintf('  导航开始UTC：  %s\n', ...
        C.nav_start_utc);
    fprintf('  实验公共起点： %.3f\n', ...
        target_time);
    fprintf('  两者相差：     %.3f s\n', ...
        target_time-C.nav_start_time);
end
%% ========================================================================
%% 6. 输出全部候选与实验时间的关系
%
% 用图检查自动匹配是否合理。
% ========================================================================
figure( ...
    'Name','120对准事件与实验时间关系', ...
    'NumberTitle','off');
hold on;
grid on;
%% 候选事件
for i = 1:length(Candidate)
    plot( ...
        Candidate(i).nav_start_time, ...
        1, ...
        'o', ...
        'MarkerSize',7);
    text( ...
        Candidate(i).nav_start_time, ...
        1.05, ...
        sprintf('D%03d',Candidate(i).file_id), ...
        'HorizontalAlignment','center');
end
%% 下午和晚上实验起点
for k = 1:2
    xline( ...
        Experiment(k).common_range(1), ...
        '--', ...
        Experiment(k).name);
end
yticks(1);
yticklabels({'120导航开始候选'});
xlabel('GPS周秒 / s');
title('120对准/导航事件与Compass+830实验起点');
%% ========================================================================
%% 7. 根据选出的文件，提取对应120 AUAX导航数据
%
% 由于120 AUAX可能存在缺失：
%
% 这里只保存当前Disk1文件中实际存在的导航数据，
% 不把AUAX最后时刻作为真实试验结束时刻。
% ========================================================================

for k = 1:length(Alignment120)

    if ~isfield(Alignment120(k),'file_id')
        continue;
    end

    file_id = Alignment120(k).file_id;

    A = AUAX_file{file_id+1};

    status_row = size(A,1)-2;

    nav_start = Alignment120(k).nav_start_time;

    % Compass + 830公共结束时间
    nav_end = Experiment(k).common_range(2);

    idx = ...
        A(2,:) >= nav_start & ...
        A(2,:) <= nav_end & ...
        A(status_row,:) == STATUS_NAV;

    Alignment120(k).auax = A(:,idx);


    fprintf('\n');
    fprintf('============================================================\n');
    fprintf('%s：120 AUAX导航数据\n',Alignment120(k).name);
    fprintf('============================================================\n');

    fprintf('Disk1_%03d.dat\n',file_id);

    fprintf('AUAX导航数据：%d 历元\n', ...
        size(Alignment120(k).auax,2));

    if ~isempty(Alignment120(k).auax)

        fprintf('AUAX实际覆盖：%.3f ~ %.3f\n', ...
            Alignment120(k).auax(2,1), ...
            Alignment120(k).auax(2,end));

    end

end


%% ========================================================================
%% 8. 读取对应的120原始IMU数据 Disk2
%
% Disk1_003 -> Disk2_003
% Disk1_006 -> Disk2_006
%
% read_stdimu_120输出9列原始IMU：
%
%   IMU(:,1:3)     原始三轴IMU数据
%   IMU(:,7)/200   IMU上机时间
%   IMU(:,8:9)     当前无实际意义
%
% 处理顺序必须是：
%
%   原始IMU
%      ↓
%   AUXA(12:14)匹配原始IMU(:,1:3)
%      ↓
%   核对AUXA第1行与IMU第7列/200的上机时间
%      ↓
%   根据AUXA第1、2行给整段IMU赋GPS时间
%      ↓
%   把GPS时间写入原始第8列
%      ↓
%   最后调用imuFUR2FRD
%
% imu_raw始终保留read_stdimu_120原始9列，不做覆盖。
% ========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('5. 读取120原始IMU数据\n');
fprintf('============================================================\n');

frameHeader = [235,144,32];

for k = 1:length(Alignment120)

    if ~isfield(Alignment120(k),'file_id')
        continue;
    end

    file_id = Alignment120(k).file_id;

    filename_imu = fullfile( ...
        raw_dir, ...
        sprintf('Disk2_%03d.dat',file_id));

    fprintf('\n');
    fprintf('------------------------------------------------------------\n');
    fprintf('%s\n',Alignment120(k).name);
    fprintf('读取：Disk2_%03d.dat\n',file_id);
    fprintf('------------------------------------------------------------\n');

    if ~exist(filename_imu,'file')
        warning('IMU文件不存在：%s',filename_imu);
        continue;
    end

    %% --------------------------------------------------------------------
    % 读取二进制并查找帧头
    % ---------------------------------------------------------------------
    fid = fopen(filename_imu,'rb');
    allData2 = fread(fid,inf,'uint8');
    fclose(fid);

    headerPositions2 = find( ...
        allData2(1:end-2) == frameHeader(1) & ...
        allData2(2:end-1) == frameHeader(2) & ...
        allData2(3:end)   == frameHeader(3));

    fprintf('检测到IMU数据包：%d\n',length(headerPositions2));

    if isempty(headerPositions2)
        warning('Disk2_%03d.dat 中没有找到IMU帧头。',file_id);
        continue;
    end

    %% --------------------------------------------------------------------
    % 解析120原始IMU
    % ---------------------------------------------------------------------
    IMU120 = read_stdimu_120( ...
        filename_imu, ...
        headerPositions2);

    fprintf('解析完成：%d × %d\n', ...
        size(IMU120,1),size(IMU120,2));

    if size(IMU120,2) < 9
        warning(['Disk2_%03d.dat：当前IMU解析结果不足9列。' ...
                 '本脚本按9列原始格式处理，请检查read_stdimu_120。'], ...
            file_id);
        continue;
    end

    %% --------------------------------------------------------------------
    % 这里必须保存“未经坐标转换、未经列重排”的原始9列IMU
    %
    % 当前已知：
    %
    %   IMU120(:,1:3)   原始三轴IMU数据，用于和AUXA(12:14)匹配
    %   IMU120(:,7)/200  IMU上机时间
    %   IMU120(:,8:9)   当前没有实际意义
    %
    % 非常重要：
    % 此处不能提前调用 imuFUR2FRD。
    %
    % 因为 imuFUR2FRD 会进行坐标转换，并把原第8列移到第1列。
    % 当前第8列还不是UTC/GPS时间，如果现在转换，会破坏后续
    % IMU(:,1:3)物理量匹配及时间字段含义。
    % ---------------------------------------------------------------------
    Alignment120(k).imu_file = ...
        sprintf('Disk2_%03d.dat',file_id);

    Alignment120(k).imu_raw = ...
        IMU120;

    %% --------------------------------------------------------------------
    % IMU上机时间：直接从原始第7列获得
    % ---------------------------------------------------------------------
    Alignment120(k).imu_machine_time = ...
        IMU120(:,7)/200;

    % 保留旧字段名，兼容已有compare或其他脚本
    Alignment120(k).imu_time_internal = ...
        Alignment120(k).imu_machine_time;

    fprintf('IMU上机时间：%.6f ~ %.6f s\n', ...
        Alignment120(k).imu_machine_time(1), ...
        Alignment120(k).imu_machine_time(end));

end


%% ========================================================================
%% 9. 用AUXA对准时刻的IMU数据定位原始IMU同一历元
%
% 关键点采用 Candidate.marker_index：
%
%   A(1,anchor_idx)       AUXA上机时间
%   A(2,anchor_idx)       AUXA GPS时间
%   A(12:14,anchor_idx)   AUXA中的IMU三轴数据
%
% 使用 A(12:14) 与原始 IMU(:,1:3) 做数值匹配，
% 找到原始IMU中同一个物理历元 best_idx。
%
% 然后核对：
%
%   A(1,anchor_idx)
%       vs
%   IMU(best_idx,7)/200
%
% 这里非常关键：
% 如果两者相差很大，只报警，不做任何人工时间补偿。
% ========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('6. AUXA对准时刻与120原始IMU匹配\n');
fprintf('============================================================\n');

for k = 1:length(Alignment120)

    if ~isfield(Alignment120(k),'imu_raw') || ...
            isempty(Alignment120(k).imu_raw)
        continue;
    end

    file_id = Alignment120(k).file_id;
    A = AUAX_file{file_id+1};

    candidate_id = Alignment120(k).candidate_id;

    %% --------------------------------------------------------------------
    % 对准关键点：430.010 / 430.015 所在AUAX历元
    % ---------------------------------------------------------------------
    anchor_idx = ...
        Candidate(candidate_id).marker_index;

    Alignment120(k).auax_anchor_index = ...
        anchor_idx;

    Alignment120(k).auax_anchor = ...
        A(:,anchor_idx);

    %% --------------------------------------------------------------------
    % AUXA关键时间
    % ---------------------------------------------------------------------
    auxa_machine_time = ...
        A(1,anchor_idx);

    auxa_gps_time = ...
        A(2,anchor_idx);

    Alignment120(k).auxa_anchor_machine_time = ...
        auxa_machine_time;

    Alignment120(k).auxa_anchor_gps_time = ...
        auxa_gps_time;

    %% --------------------------------------------------------------------
    % AUXA该时刻IMU三轴
    % ---------------------------------------------------------------------
    auax_target = ...
        A(12:14,anchor_idx)';

    % 注意：此时imu_raw仍然是read_stdimu_120得到的原始9列数据，
    % 尚未调用imuFUR2FRD，因此1:3列仍可直接和AUXA(12:14)匹配。
    IMU120 = ...
        Alignment120(k).imu_raw;

    imu_match_data = ...
        IMU120(:,1:3);

    %% --------------------------------------------------------------------
    % 全文件寻找最相似IMU三轴历元
    %
    % 先只根据IMU数值定位，不使用时间人为限制。
    % ---------------------------------------------------------------------
    diff_data = ...
        imu_match_data - auax_target;

    score = ...
        sqrt(sum(diff_data.^2,2));

    [score_sorted,index_sorted] = ...
        sort(score,'ascend');

    best_idx = ...
        index_sorted(1);

    best_score = ...
        score_sorted(1);

    Alignment120(k).imu_match_index = ...
        best_idx;

    Alignment120(k).imu_match_score = ...
        best_score;

    Alignment120(k).imu_match_value = ...
        IMU120(best_idx,:);

    %% --------------------------------------------------------------------
    % 匹配到的IMU上机时间
    % ---------------------------------------------------------------------
    imu_match_machine_time = ...
        Alignment120(k).imu_machine_time(best_idx);

    Alignment120(k).imu_match_machine_time = ...
        imu_match_machine_time;

    % 保留旧字段
    Alignment120(k).imu_match_internal_time = ...
        imu_match_machine_time;

    %% --------------------------------------------------------------------
    % 核对AUXA上机时间与IMU上机时间
    % ---------------------------------------------------------------------
    machine_time_error = ...
        imu_match_machine_time - auxa_machine_time;

    Alignment120(k).machine_time_error = ...
        machine_time_error;

    %% --------------------------------------------------------------------
    % 保存GPS锚点
    % ---------------------------------------------------------------------
    Alignment120(k).imu_anchor_gps = ...
        auxa_gps_time;

    %% --------------------------------------------------------------------
    % 打印详细结果
    % ---------------------------------------------------------------------
    fprintf('\n');
    fprintf('---------------- %s ----------------\n', ...
        Alignment120(k).name);

    fprintf('Disk1：Disk1_%03d.dat\n',file_id);
    fprintf('Disk2：Disk2_%03d.dat\n',file_id);

    fprintf('\nAUXA对准关键点：\n');
    fprintf('  AUXA index        ：%d\n',anchor_idx);
    fprintf('  AUXA上机时间      ：%.6f s\n', ...
        auxa_machine_time);
    fprintf('  AUXA GPS时间      ：%.6f s\n', ...
        auxa_gps_time);
    fprintf('  AUXA UTC          ：%s\n', ...
        gpssec_to_utc_string(GPS_WEEK,auxa_gps_time));

    fprintf('\nAUXA(12:14)：\n');
    fprintf('  %.9f  %.9f  %.9f\n', ...
        A(12:14,anchor_idx));

    fprintf('\n匹配原始IMU：\n');
    fprintf('  IMU index         ：%d\n',best_idx);
    fprintf('  IMU(1:3)          ：%.9f  %.9f  %.9f\n', ...
        IMU120(best_idx,1:3));
    fprintf('  三维差            ：%.9e  %.9e  %.9e\n', ...
        IMU120(best_idx,1:3)-auax_target);
    fprintf('  匹配score         ：%.9e\n',best_score);
    fprintf('  IMU上机时间       ：%.6f s\n', ...
        imu_match_machine_time);

    fprintf('\n上机时间一致性：\n');
    fprintf('  IMU - AUXA        ：%+.6f s\n', ...
        machine_time_error);

    % 打印前3个数值匹配候选，便于排查静止阶段重复值
    n_show = min(3,length(index_sorted));

    fprintf('\n最接近的%d个IMU候选：\n',n_show);

    for ii = 1:n_show

        idx_i = index_sorted(ii);

        fprintf( ...
            '  #%d index=%d  score=%.6e  machine=%.6f  dt_machine=%+.6f s\n', ...
            ii, ...
            idx_i, ...
            score_sorted(ii), ...
            Alignment120(k).imu_machine_time(idx_i), ...
            Alignment120(k).imu_machine_time(idx_i)-auxa_machine_time);

    end

    if abs(machine_time_error) > MACHINE_TIME_WARN

        warning( ...
            ['%s：AUXA与原始IMU虽然数值匹配，但上机时间相差 %.6f s。' ...
             '这里不做人工补偿，请优先检查匹配点/时间字段定义。'], ...
            Alignment120(k).name, ...
            machine_time_error);

    end

end


%% ========================================================================
%% 10. 利用AUXA“上机时间 <-> GPS时间”关系给整段IMU赋GPS时间
%
% AUXA关键点提供：
%
%   t_auxa_machine = A(1,anchor_idx)
%   t_auxa_gps     = A(2,anchor_idx)
%
% 因此：
%
%   gps_minus_machine =
%       t_auxa_gps - t_auxa_machine
%
% 对所有原始IMU历元：
%
%   t_imu_gps =
%       t_imu_machine + gps_minus_machine
%
%
% 注意：
% 这里不使用人为420 s补偿。
%
% 同时再计算一条“匹配IMU历元锚定”的结果用于核对：
%
%   t_imu_gps_match =
%       t_auxa_gps
%       + (t_imu_machine - t_imu_match_machine)
%
% 两种方法之间的恒定差值应等于：
%
%   imu_match_machine - auxa_machine
%
% 即前面打印的 machine_time_error。
% ========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('7. 根据AUXA上机时间/UTC关系构造120 IMU绝对GPS时间\n');
fprintf('============================================================\n');

for k = 1:length(Alignment120)

    if ~isfield(Alignment120(k),'imu_raw') || ...
            isempty(Alignment120(k).imu_raw)
        continue;
    end

    required_fields = { ...
        'imu_machine_time', ...
        'auxa_anchor_machine_time', ...
        'auxa_anchor_gps_time', ...
        'imu_match_machine_time'};

    has_all = true;

    for ii = 1:length(required_fields)

        if ~isfield(Alignment120(k),required_fields{ii})
            has_all = false;
            break;
        end

    end

    if ~has_all
        warning('%s：缺少时间锚定字段，无法构造IMU GPS时间。', ...
            Alignment120(k).name);
        continue;
    end

    t_imu_machine = ...
        Alignment120(k).imu_machine_time;

    t_auxa_machine = ...
        Alignment120(k).auxa_anchor_machine_time;

    t_auxa_gps = ...
        Alignment120(k).auxa_anchor_gps_time;

    t_imu_match_machine = ...
        Alignment120(k).imu_match_machine_time;

    %% --------------------------------------------------------------------
    % 10.1 正式时间解：使用AUXA上机时间 <-> GPS时间关系
    % ---------------------------------------------------------------------
    gps_minus_machine = ...
        t_auxa_gps - t_auxa_machine;

    Alignment120(k).gps_minus_machine = ...
        gps_minus_machine;

    Alignment120(k).imu_time_gps = ...
        t_imu_machine + gps_minus_machine;

    %% --------------------------------------------------------------------
    % 10.2 仅用于核对的另一条时间解
    % ---------------------------------------------------------------------
    imu_time_gps_by_match = ...
        t_auxa_gps ...
        + (t_imu_machine - t_imu_match_machine);

    time_solution_difference = ...
        Alignment120(k).imu_time_gps ...
        - imu_time_gps_by_match;

    Alignment120(k).time_solution_difference = ...
        time_solution_difference(1);

    %% --------------------------------------------------------------------
    % 10.3 在匹配IMU历元处检查赋值后的GPS时间
    % ---------------------------------------------------------------------
    best_idx = ...
        Alignment120(k).imu_match_index;

    matched_imu_gps = ...
        Alignment120(k).imu_time_gps(best_idx);

    matched_gps_error = ...
        matched_imu_gps - t_auxa_gps;

    Alignment120(k).matched_gps_error = ...
        matched_gps_error;

    %% --------------------------------------------------------------------
    % 10.4 最后一步才进行 FUR -> FRD 坐标转换和列重排
    %
    % read_stdimu_120 原始数据共有9列：
    %
    %   1:3   原始IMU三轴
    %   7     上机计数（/200得到上机时间）
    %   8:9   原本无实际意义
    %
    % imuFUR2FRD 的既有约定是：
    %   - 完成FUR -> FRD坐标转换
    %   - 将原第8列移到输出第1列
    %
    % 因此必须先把已经求出的GPS绝对时间写入原始第8列，
    % 然后再调用imuFUR2FRD。
    %
    % 这样最终：
    %
    %   imu120FRD(:,1)   GPS绝对时间
    %   imu120FRD(:,2:4) FRD三轴IMU
    %
    % 同时 Alignment120(k).imu_raw 始终保留原始9列，不被覆盖。
    % ---------------------------------------------------------------------
    IMU120_with_gps = ...
        Alignment120(k).imu_raw;

    % 原第8列原本没有实际意义，现在正式赋为GPS绝对时间
    IMU120_with_gps(:,8) = ...
        Alignment120(k).imu_time_gps(:);

    Alignment120(k).imu_raw_with_gps = ...
        IMU120_with_gps;

    % 到这里才允许转换坐标系和重排列
    Alignment120(k).imu120FRD = ...
        imuFUR2FRD(IMU120_with_gps);

    % 基本检查：转换后第一列应当就是GPS时间
    if size(Alignment120(k).imu120FRD,2) < 4

        error('%s：imuFUR2FRD输出列数不足4列。', ...
            Alignment120(k).name);

    end

    time_reorder_error = ...
        max(abs( ...
        Alignment120(k).imu120FRD(:,1) ...
        - Alignment120(k).imu_time_gps(:)));

    Alignment120(k).frd_time_reorder_error = ...
        time_reorder_error;

    fprintf('  FRD转换后第1列时间核对：最大误差 %.9e s\n', ...
        time_reorder_error);

    %% --------------------------------------------------------------------
    % 打印
    % ---------------------------------------------------------------------
    fprintf('\n%s\n',Alignment120(k).name);

    fprintf('  AUXA上机时间         ：%.6f s\n', ...
        t_auxa_machine);

    fprintf('  AUXA GPS时间         ：%.6f s\n', ...
        t_auxa_gps);

    fprintf('  GPS - 上机时间       ：%.6f s\n', ...
        gps_minus_machine);

    fprintf('  匹配IMU上机时间      ：%.6f s\n', ...
        t_imu_match_machine);

    fprintf('  IMU-AUXA上机时间差   ：%+.6f s\n', ...
        Alignment120(k).machine_time_error);

    fprintf('  匹配点赋值后GPS误差  ：%+.6f s\n', ...
        matched_gps_error);

    fprintf('  两种GPS构造方法差值  ：%+.6f s\n', ...
        time_solution_difference(1));

    fprintf('  IMU GPS范围          ：%.6f ~ %.6f\n', ...
        Alignment120(k).imu_time_gps(1), ...
        Alignment120(k).imu_time_gps(end));

    fprintf('  IMU UTC范围          ：%s\n', ...
        gpssec_to_utc_string( ...
        GPS_WEEK, ...
        Alignment120(k).imu_time_gps(1)));

    fprintf('                      ~ %s\n', ...
        gpssec_to_utc_string( ...
        GPS_WEEK, ...
        Alignment120(k).imu_time_gps(end)));

end


%% ========================================================================
%% 11. 保存
%
% 注意：
%
% 现在Alignment120中包含完整IMU数据：
%
% Alignment120(k).imu_raw
%
% 数据量可能比较大，所以直接使用-v7.3。
% ========================================================================

save_file = ...
    fullfile( ...
    temp_dir, ...
    'alignment120_1207.mat');


save( ...
    save_file, ...
    'Alignment120', ...
    'Candidate', ...
    'Experiment', ...
    '-v7.3');


fprintf('\n');
fprintf('============================================================\n');
fprintf('120数据保存完成\n');
fprintf('============================================================\n');

fprintf('%s\n',save_file);


%% ========================================================================
%% 12. 保存内容检查
% ========================================================================

fprintf('\n');
fprintf('保存的数据结构：\n');


for k = 1:length(Alignment120)

    fprintf('\n%s：\n',Alignment120(k).name);

    fprintf('  Disk1        ：%03d\n', ...
        Alignment120(k).file_id);

    if isfield(Alignment120(k),'imu_file')

        fprintf('  IMU文件      ：%s\n', ...
            Alignment120(k).imu_file);

    end


    if isfield(Alignment120(k),'imu_raw')

        fprintf('  IMU数据      ：%d × %d\n', ...
            size(Alignment120(k).imu_raw,1), ...
            size(Alignment120(k).imu_raw,2));

    end


    if isfield(Alignment120(k),'auax')

        fprintf('  AUAX导航数据 ：%d历元\n', ...
            size(Alignment120(k).auax,2));

    end


    if isfield(Alignment120(k),'imu_match_index')

        fprintf('  IMU匹配index ：%d\n', ...
            Alignment120(k).imu_match_index);

        fprintf('  AUXA上机时间 ：%.6f\n', ...
            Alignment120(k).auxa_anchor_machine_time);

        fprintf('  AUXA锚点GPS   ：%.6f\n', ...
            Alignment120(k).auxa_anchor_gps_time);

        fprintf('  IMU上机时间  ：%.6f\n', ...
            Alignment120(k).imu_match_machine_time);

        fprintf('  上机时间差   ：%+.6f s\n', ...
            Alignment120(k).machine_time_error);

        if isfield(Alignment120(k),'imu_time_gps')

            fprintf('  IMU GPS范围  ：%.6f ~ %.6f\n', ...
                Alignment120(k).imu_time_gps(1), ...
                Alignment120(k).imu_time_gps(end));

        end

    end

end

%% ========================================================================
%% 局部函数
% ========================================================================
function str = ...
    gpssec_to_utc_string( ...
    gps_week, ...
    gps_second)
%GPSSEC_TO_UTC_STRING
%
% GPS时间：
%
% GPS epoch = 1980-01-06
%
% 2025年：
%
% GPS - UTC = 18 s
    GPS_UTC_LEAP = 18;
    epoch = ...
        datetime( ...
        1980,1,6, ...
        0,0,0, ...
        'TimeZone','UTC');
    t = ...
        epoch ...
        + days(gps_week*7) ...
        + seconds(gps_second-GPS_UTC_LEAP);
    t.Format = ...
        'yyyy-MM-dd HH:mm:ss.SSS';
    str = char(t);
end