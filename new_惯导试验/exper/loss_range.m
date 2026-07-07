clear;
param = Param();
glvs
%%
path='D:\GitHub\KF-GINS-Matlab\旋转收缩方案1/input/input6';
cfg = ProcessConfigforSemiPhy_all(path);
rangstd = 6;
depthstd = 0.4;
rng(1)

% ==================== 核心开关设置 ====================
backwardIsOpen = 0;
smoothWay = 'RTS';
SmoothIsOpen = 1;        % 开启平滑
backwardIsOpen_1s = 0;   % 关闭后向滤波
IsEKFRotate = 0;         % 关闭旋转
feedback = 1;
% ======================================================

tic
type = {"single","moving","3","2"};
beacontype = type{3};
type = {"Range","Range+azi","Range+azi+pos","pos"};
meas = type{1};

%% importdata data
% imudata
imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);

% range data
cfg.userange=1;
rangedata1 = importdata(cfg.rangefile1path);
rangedata2 = importdata(cfg.rangefile2path);
rangedata3 = importdata(cfg.rangefile3path);
range = {rangedata1,rangedata2,rangedata3};

% 构造
id = 420; % 420s=7min数据周期
for i=1:3
    range{i} = range{i}(id:id:end,:);
end
rangedata = zeros(size(range{1}));
seq = [1,2,3];
for i=1:3
    rangedata(i:3:end,:)=range{seq(i)}(i:3:end,:);
end

%% 获取处理时间，调整时间
if cfg.starttime < imustarttime
    cfg.starttime = imustarttime;
end
if cfg.endtime > imuendtime
    cfg.endtime = imuendtime;
end

% data in process interval
imudata = imudata(imudata(:,1) >= cfg.starttime, :);
imudata = imudata(imudata(:,1) <= cfg.endtime, :);
rangedata = rangedata(rangedata(:, 1) >= cfg.starttime, :);
rangedata = rangedata(rangedata(:, 1) <= cfg.endtime, :);

% 添加噪声 (在循环外进行，确保不同对比组的底噪完全一致)
rangedata_true = rangedata(:,3) ;
rangedata(:,3) = rangedata(:,3) + normrnd(6,rangstd,size(rangedata(:,3)));
rangstd = 6;

truth = importdata(cfg.truthpath);
height = truth(:,[2,5]);
height = height(height(:, 1) >= cfg.starttime, :);
height = height(height(:, 1) <= cfg.endtime, :);
height(:,2) = height(:,2) + normrnd(1,depthstd,size(height(:,2)));

% 保存原始完整的 rangedata 以备用
rangedata_orig = rangedata;

%% ==================== 定义测试场景 (分别丢失单点) ====================
% 每一行代表一个测试场景: {要删除的Range序号数组, '输出文件后缀名'}
test_cases = {
    [], 'Baseline';        % 场景1: 完整数据无丢失 (作为参考基准)
    [1],  'Drop_1';        % 场景2: 仅丢失第 1 个点
    [3],  'Drop_3';        % 场景3: 仅丢失第 3 个点
    [5],  'Drop_5';        % 场景4: 仅丢失第 5 个点
    [7],  'Drop_7';        % 场景5: 仅丢失第 7 个点
    [9],  'Drop_9';        % 场景6: 仅丢失第 9 个点
    [11], 'Drop_11';       % 场景7: 仅丢失第 11 个点
};

%% ==================== 开始批量处理对比 ====================
for case_idx = 1:size(test_cases, 1)
    drop_indices = test_cases{case_idx, 1};
    case_name    = test_cases{case_idx, 2};
    
    disp('==================================================');
    disp(['开始处理场景 (', num2str(case_idx), '/', num2str(size(test_cases,1)), '): ', case_name]);
    
    % 1. 构造当前场景的 rangedata
    rangedata = rangedata_orig;
    if ~isempty(drop_indices)
        % 剔除超出范围的索引 (防止误报)
        drop_indices = drop_indices(drop_indices <= size(rangedata, 1));
        % 删除指定的行
        rangedata(drop_indices, :) = [];
    end
    
    % 2. 动态设置当前场景的保存路径 (带有后缀以区分)
    navpath = [cfg.outputfolder, sprintf('/Origin_%s.nav', case_name)];
    navfp = fopen(navpath, 'wt');
    if SmoothIsOpen == 1
        navpath1 = [cfg.outputfolder, sprintf('/Compensation-double_%s.nav', case_name)];
        navfp1 = fopen(navpath1, 'wt');
        navpath2 = [cfg.outputfolder, sprintf('/Compensation_%s.nav', case_name)];
        navfp2 = fopen(navpath2, 'wt');
    end

    % 3. 重新初始化滤波器状态 (确保每次循环互不干扰)
    [kf, navstate] = myInitialize_15state(cfg);
    laststate = navstate;
    kf.rangstd = rangstd;
    kf.depthstd = depthstd;
    
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
    Pk_propa     = zeros(MAX_BUFFER_SIZE, 225);
    PHI          = zeros(MAX_BUFFER_SIZE, 225);
    prev_state_buffer = [];
    prev_Pk_propa = [];
    prev_Pk_k1propa = [];
    prev_PHI = [];
    prev_rangeindex = 0;
    buf_idx = 1;
    navstate0 = navstate;
    nav_record = zeros(length(imudata),11);
    
    lastprecent = 0;
    
    % 4. 核心解算循环
    for imuindex = 2:size(imudata, 1)-1
        lastimu = thisimu;
        laststate = navstate;
        old_state = navstate;
        thisimu = imudata(imuindex, :)';
        imudt = thisimu(1, 1) - lastimu(1, 1);
        
        while (rangeindex <= size(rangedata, 1) && rangedata(rangeindex, 1) < lastimu(1, 1))
            rangeindex = rangeindex + 1;
        end
        
        if rangeindex <= size(rangedata, 1) && lastimu(1, 1) == rangedata(rangeindex, 1) && cfg.userange==1
            % ================ 测量更新 ================
            kf = myRangeUpdate(navstate, rangedata(rangeindex,:), height(imuindex,:), kf);
            rangeindex = rangeindex + 1;
           
            if SmoothIsOpen == 1
                if buf_idx > 1
                    valid_len = buf_idx - 1;
                    state_buffer = state_buffer(1:valid_len, :);
                    if strcmp(smoothWay,'RTS')
                        xk_final = kf.x;
                        Xk_k1propa  = Xk_k1propa(1:valid_len, :);
                        Pk_k1propa  = Pk_k1propa(1:valid_len, :);
                        Pk_propa    = Pk_propa(1:valid_len, :);
                        PHI         = PHI(1:valid_len, :);
                        
                        [nav_matrix, bridge_error, rtsstate_buffer] = perform_RTS_smoothing(state_buffer,  Pk_propa, Pk_k1propa,  PHI, xk_final, param, rangeindex);
                        fprintf(navfp2, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix);
                        
                        if isempty(prev_state_buffer)
                            prev_state_buffer = rtsstate_buffer;
                            prev_Pk_propa     = Pk_propa;
                            prev_Pk_k1propa   = Pk_k1propa;
                            prev_PHI          = PHI;
                            prev_rangeindex   = rangeindex;
                        else
                            nav_matrix_prev_resmoothed = perform_block_smoothing(prev_state_buffer, bridge_error, param, prev_rangeindex);
                            fprintf(navfp1, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix_prev_resmoothed);
                            
                            prev_state_buffer = rtsstate_buffer;
                            prev_Pk_propa     = Pk_propa;
                            prev_Pk_k1propa   = Pk_k1propa;
                            prev_PHI          = PHI;
                            prev_rangeindex   = rangeindex;
                        end
                    elseif strcmp(smoothWay,'Linear')
                        nav_matrix = perform_block_smoothing(state_buffer, kf.x, param, rangeindex);
                        fprintf(navfp2, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix);
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
            
        elseif rangeindex <= size(rangedata, 1) && (lastimu(1, 1) < rangedata(rangeindex, 1) && thisimu(1, 1) > rangedata(rangeindex, 1)) && cfg.userange==1
            kf = myRangeUpdate(navstate, rangedata(rangeindex,:), height(imuindex,:), kf);
            if feedback == 1
                [kf, navstate] = myErrorFeedback_range(kf, navstate);
            end
            rangeindex = rangeindex + 1;
            laststate = navstate;
            lastimu = firstimu;
            imudt = secondimu(1, 1) - lastimu(1, 1);
            navstate = InsMech(laststate, lastimu, secondimu);
            kf = myInsPropagate_15state(navstate, secondimu, imudt, kf);
            
        else
            % ================ 纯惯导推算 ================
            navstate = InsMech(laststate, lastimu, thisimu);
            navstate.pos(3) = height(imuindex,2);
            
            if SmoothIsOpen == 1
                Pk_propa(buf_idx,:) = kf.P(:)';
            end
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
        
        % 写入基础 nav 结果
        nav = zeros(11, 1);
        nav(2, 1) = navstate.time;
        nav(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
        nav(6:8, 1) = navstate.vel;
        nav(9:11, 1) = navstate.att * param.R2D;
        fprintf(navfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);
        nav_record(imuindex-1,:) = nav;
        
        % 进度打印
        if (imuindex / size(imudata, 1) - lastprecent > 0.20)
            disp(["Processing ", num2str(floor(imuindex * 100 / size(imudata, 1))), " %"]);
            lastprecent = imuindex / size(imudata, 1);
        end
    end
    
    % 5. 循环结束，处理最后一段残留的平滑
    if ~isempty(prev_state_buffer)
        nav_matrix = perform_block_smoothing(rtsstate_buffer, zeros(15,1), param, rangeindex);
        fprintf(navfp1, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix);
    end
    
    fclose all;
end

toc
disp("All specific drop testing scenarios completed!");
%%
ii = [1,3,5,7,9,11];
navpath = '旋转收缩方案1/output/output6/Origin-Baseline.nav';
for i=1:6
navph{i}=['旋转收缩方案1/output/output6/Origin-Drop-',num2str(ii(i)),'.nav'];
end
[fig,finalExcelData]=calc_radial_error(cfg.truthpath,navpath,navph{:});
outputExcelName = [cfg.outputfolder,'/导航系统径向误差统计报告-all-loserange','.xlsx'];
writecell(finalExcelData, outputExcelName);
exportgraphics(fig, fullfile(cfg.outputfolder, 'loserange.png'), 'Resolution', 600);