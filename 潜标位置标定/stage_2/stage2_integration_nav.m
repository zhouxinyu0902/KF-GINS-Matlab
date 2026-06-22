clear
glvs
tic
%% 定义参数+加载过程配置
param = Param();
% path_used = 'D:\Github\KF-GINS-Matlab\潜标位置标定\Square_ArrayCenter_Trj\';
% trj='-square';

% path_used = 'D:\Github\KF-GINS-Matlab\潜标位置标定\Line_ArrayCenter_Trj\';
% trj='-line';

path_used = 'D:\Github\KF-GINS-Matlab\潜标位置标定\stage_2\data\Circle_ArrayCenter_Trj\';
trj='-circle';


% path_used = 'D:\Github\KF-GINS-Matlab\潜标位置标定\Square_Beacon1_Trj\';
% trj='-square-bea';

% path_used = 'D:\Github\KF-GINS-Matlab\潜标位置标定\Line_Beacon1_Trj\';
% trj='-line-bea';

% path_used = 'D:\Github\KF-GINS-Matlab\潜标位置标定\Circle_Beacon1_Trj\';
% trj='-circle-bea';

cfg = Config_10state(path_used,'integration');
%% 加载数据
% imudata
imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);
truth = importdata(cfg.truthpath);
feedback = 1;

SmoothIsOpen = 0;
smoothWay = 'Linear';
%% 获取距离
% rangedataget('D:\Github\KF-GINS-Matlab\graduation\DR_INS\input')
% 构造距离信息
id = 420/20;
% dataType = 2;
for dataType = 1:4
    switch dataType
        case 1
            disp('>>> [数据流切换]: 载入【未修正前】的原始距离数据...');
            rangedata1 = importdata([cfg.input, '/range1.txt']);
            rangedata2 = importdata([cfg.input, '/range2.txt']);
            rangedata3 = importdata([cfg.input, '/range3.txt']);

            % 动态输出对应的标准导航结果文件名
            navFileName = '/Origin.nav';
            if feedback == 0
                xkFileName  = '/xk-origin.nav';
            end

        case 2
            disp('>>> [数据流切换]: 载入【二次修正后】的标定补偿距离数据...');
            % 绑定刚才 range_reconstruct 导出的标定代偿文本
            rangedata1 = importdata([cfg.input, '/range1_calib_2_20s.txt']);
            rangedata2 = importdata([cfg.input, '/range2_calib_2_20s.txt']);
            rangedata3 = importdata([cfg.input, '/range3_calib_2_20s.txt']);

            navFileName = '/Calib-twice.nav';
            if feedback == 0
                xkFileName  = '/xk-calib-twice.nav';
            end
        case 3
            disp('>>> [数据流切换]: 载入【真实的】潜标无误差距离数据...');
            % 绑定刚才 range_reconstruct 导出的绝对真实真值文本
            rangedata1 = importdata([cfg.input, '/range1_true_2_20s.txt']);
            rangedata2 = importdata([cfg.input, '/range2_true_2_20s.txt']);
            rangedata3 = importdata([cfg.input, '/range3_true_2_20s.txt']);

            navFileName = '/True.nav';
            if feedback == 0
                xkFileName  = '/xk-true.nav';
            end
        case 4
            disp('>>> [数据流切换]: 载入【USBL】潜标无误差距离数据...');
            % 绑定刚才 range_reconstruct 导出的绝对真实真值文本
            rangedata1 = importdata([cfg.input, '/range1_usbl.txt']);
            rangedata2 = importdata([cfg.input, '/range2_usbl.txt']);
            rangedata3 = importdata([cfg.input, '/range3_usbl.txt']);

            navFileName = '/USBL.nav';
            if feedback == 0
                xkFileName  = '/xk-USBL.nav';
            end
        otherwise
            error('错误：未知的数据类型配置类型(dataType)，请在1~4之间选择。');
    end

    % ==================== 设置文件保存路径 ====================
    navpath = [cfg.outputfolder, navFileName];
    navfp = fopen(navpath, 'wt');
    if feedback == 0
        xkpath = [cfg.outputfolder, xkFileName];
        xkfp = fopen(xkpath, 'wt');
    end
    if SmoothIsOpen == 1
        if strcmp(smoothWay,'RTS')
            navpath1 = [cfg.outputfolder, navFileName(1:end-4),smoothWay,'-double.nav'];
            navfp1 = fopen(navpath1, 'wt');
        end
        navpath2 = [cfg.outputfolder, navFileName(1:end-4),smoothWay,'.nav'];
        navfp2 = fopen(navpath2, 'wt');
    end
    fprintf('>>> 导航轨迹输出路径已自动重定向至: %s\n', navpath);
    if feedback==0
    fprintf('>>> 状态误差输出路径已自动重定向至: %s\n\n', xkpath);
    end
    range = {rangedata1,rangedata2,rangedata3};

    for i=1:length(range)
        range{i} = range{i}(id+1:id:end,:);
    end
    seq=[1,2,3;1,3,2;2,3,1];
    nnn=1;
    rangedata = zeros(size(range{1}));
    for i=1:3
        rangedata(i:3:end,:)=range{1,seq(nnn,i)}(i:3:end,:);
    end
    % rangedata(:,3) = rangedata(:,3) + normrnd(0,6,size(rangedata(:,3)));
    %% 获取处理时间，调整时间
    starttime = imustarttime;
    endtime = imuendtime;
    if cfg.starttime < starttime
        cfg.starttime = starttime;
    end
    if cfg.endtime > endtime
        cfg.endtime = endtime;
    end
    % data in process interval
    imudata = imudata(imudata(:,1) >= cfg.starttime, :);
    imudata = imudata(imudata(:,1) <= cfg.endtime, :);
    heightdata = truth(:,[2,5]);
    heightdata = heightdata(heightdata(:,1) >= cfg.starttime, :);
    heightdata = heightdata(heightdata(:,1) <= cfg.endtime, :);
    heightdata(:,2) = heightdata(:,2) + normrnd(0,0.2,size(heightdata(:,2)));
    rangedata = rangedata(rangedata(:, 1) >= cfg.starttime, :);
    rangedata = rangedata(rangedata(:, 1) <= cfg.endtime, :);

    %% 调试
    disp("Start INS Processing!");
    lastprecent = 0;
    %% 初始化
    [kf, navstate] = myInitialize_15state(cfg);
    kf.depthstd = 0.2;
    laststate = navstate;
    kf.rangstd = 10;
    lastimu = imudata(1, :)';
    thisimu = imudata(1, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);

    rangeindex = 1;
    while rangedata(rangeindex, 1) < thisimu(1, 1)
        rangeindex = rangeindex + 1;
    end
    cfg.userange=1;

    MAX_BUFFER_SIZE = 54000;

    state_buffer = zeros(MAX_BUFFER_SIZE, 10);
    Xk_k1propa   = zeros(MAX_BUFFER_SIZE, 15);
    Pk_k1propa   = zeros(MAX_BUFFER_SIZE, 225);
    Pk_propa = zeros(MAX_BUFFER_SIZE, 225);
    PHI   = zeros(MAX_BUFFER_SIZE, 225); % 建议变量名区分开


    prev_state_buffer = [];
    prev_Pk_propa = [];
    prev_Pk_k1propa = [];
    prev_PHI = [];
    prev_rangeindex = 0;

    buf_idx = 1;

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
            kf = myRangeUpdate_m(navstate, rangedata(rangeindex,:), heightdata(imuindex,:), kf);
            rangeindex = rangeindex + 1;

            if SmoothIsOpen == 1
                if buf_idx > 1
                    valid_len = buf_idx - 1;
                    state_buffer = state_buffer(1:valid_len, :);
                    if strcmp(smoothWay,'RTS')
                        xk_final = kf.x;
                        Xk_k1propa    = Xk_k1propa(1:valid_len, :);
                        Pk_k1propa    = Pk_k1propa(1:valid_len, :);
                        Pk_propa    = Pk_propa(1:valid_len, :);
                        PHI   = PHI(1:valid_len, :);

                        % % 调用 RTS 平滑函数
                        [nav_matrix, bridge_error,rtsstate_buffer] = perform_RTS_smoothing(state_buffer,  Pk_propa, Pk_k1propa,  PHI, xk_final, param, rangeindex);
                        % 批量写入文件
                        fprintf(navfp2, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix);
                        bridge_err(rangeindex,:) = bridge_error;
                        % 1. 先对【当前的 7 分钟】做一次常规 RTS 平滑，拿到桥接误差 bridge_error

                        % [nav_matrix, bridge_error] = perform_RTS_smoothing_new(state_buffer, Pk_propa, Pk_k1propa, PHI, xk_final, param, rangeindex);
                        % fprintf(navfp2, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix);

                        % 2. 判断是否有【上一个 7 分钟】的数据被缓存
                        if isempty(prev_state_buffer)
                            % 如果是第一段数据 (比如 0-7min)，没法进行二次平滑，直接暂存起来
                            prev_state_buffer = rtsstate_buffer;
                            prev_Pk_propa     = Pk_propa;
                            prev_Pk_k1propa   = Pk_k1propa;
                            prev_PHI          = PHI;
                            prev_rangeindex   = rangeindex;

                        else
                            % 如果有上一个 7 分钟的数据 (比如现在算完了 7-14min)
                            % 核心操作：把 7-14min 算出的起始误差 (bridge_error)，当作 0-7min 的终点误差！

                            % [nav_matrix_prev_resmoothed, ~] = perform_RTS_smoothing_new(prev_state_buffer, prev_Pk_propa, prev_Pk_k1propa, prev_PHI, bridge_error, param, prev_rangeindex);
                            % [nav_matrix_prev_resmoothed, ~] = perform_RTS_smoothing(prev_state_buffer,  prev_Pk_propa, prev_Pk_k1propa,  prev_PHI, bridge_error, param, rangeindex);

                            nav_matrix_prev_resmoothed = perform_block_smoothing(prev_state_buffer, bridge_error, param, prev_rangeindex);
                            % 将经历过“未来信息”二次洗礼的上一段数据，写入文件！
                            fprintf(navfp1, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav_matrix_prev_resmoothed);

                            % 更新缓存：把当前的 7-14min 变成新的“上一段”，等待 14-21min 来救赎它
                            prev_state_buffer = rtsstate_buffer;
                            prev_Pk_propa     = Pk_propa;
                            prev_Pk_k1propa   = Pk_k1propa;
                            prev_PHI          = PHI;
                            prev_rangeindex   = rangeindex;
                        end
                    elseif strcmp(smoothWay,'Linear')
                        nav_matrix = perform_block_smoothing(state_buffer, kf.x, param, rangeindex);
                        % 批量写入文件
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
                [kf, navstate] = myErrorFeedback_range_m(kf, navstate);
                % [kf, navstate] = myErrorFeedback_15state(kf, navstate);
            end
            laststate = navstate;

            % 惯导推算
            imudt = thisimu(1, 1) - lastimu(1, 1);
            navstate = InsMech(laststate, lastimu, thisimu);
            kf = myInsPropagate_15state_m(navstate, thisimu, imudt, kf);




        elseif (lastimu(1, 1) < rangedata(rangeindex, 1) && thisimu(1, 1) > rangedata(rangeindex, 1))&& cfg.userange==1
            kf = myRangeUpdate_m(navstate, rangedata(rangeindex,:), heightdata(imuindex,:), kf);
            if feedback == 1
                [kf, navstate] = myErrorFeedback_range_m(kf, navstate);
                % [kf, navstate] = myErrorFeedback_15state(kf, navstate);
            end
            rangeindex = rangeindex + 1;
            laststate = navstate;
            lastimu = firstimu;

            % do propagation for second imu
            imudt = secondimu(1, 1) - lastimu(1, 1);
            navstate = InsMech(laststate, lastimu, secondimu);
            kf = myInsPropagate_15state_m(navstate, secondimu, imudt, kf);
            pos0 = navstate.pos;
            vel0 = navstate.vel;
        else
            %% only do propagation
            % INS mechanization
            navstate = InsMech(laststate, lastimu, thisimu);
            % navstate.pos(3) = heightdata(imuindex,2);

            % 调用高度卡尔曼量测更新
            kf = myHeightUpdate_m(navstate, heightdata(imuindex, :), kf);

            % 反馈修正惯导状态 (仅修正天向)
            navstate.pos(3) = navstate.pos(3) + kf.x(3);
            navstate.vel(3) = navstate.vel(3) - kf.x(6);

            % 💡 反馈后，将整个误差状态量清零（代表误差已融入主状态）
            % kf.x = zeros(size(kf.x));
            kf.x(3) = 0;kf.x(6) = 0;

            if SmoothIsOpen == 1
                Pk_propa(buf_idx,:) = kf.P(:)';
            end

            % error propagation
            kf = myInsPropagate_15state_m(navstate, thisimu, imudt, kf);

            if SmoothIsOpen == 1
                % 【核心注入】将当前状态存入缓存，不立即写入文件
                nav = [navstate.time;navstate.pos;navstate.vel;navstate.att];
                state_buffer(buf_idx,:) =  nav';
                Xk_k1propa(buf_idx,:) = kf.x(:)';
                Pk_k1propa(buf_idx,:) = kf.P(:)';
                PHI(buf_idx,:) = kf.phi(:)';
                buf_idx = buf_idx + 1;
            end
        end

        % 6、save data
        nav = zeros(11, 1);
        nav(2, 1) = navstate.time;
        nav(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
        nav(6:8, 1) = navstate.vel;
        nav(9:11, 1) = navstate.att * param.R2D;
        fprintf(navfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);

        % 保存估计的状态值
        if feedback == 0
            xk = zeros(16, 1);
            xk(1) = navstate.time;
            xk(2:16) = kf.x(1:15);
            fprintf(xkfp, '%12.6f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f\n', xk);
        end
        % print processing information
        if (imuindex / size(imudata, 1) - lastprecent > 0.20)
            disp("processing " + num2str(floor(imuindex * 100 / size(imudata, 1))) + " %!");
            lastprecent = imuindex / size(imudata, 1);
        end

    end
    %%
    fclose all;
    disp("Integration Processing Finished!");
    %%
    % plot_result(navpath)
    %%
    % calc_radial_error(cfg.truthpath,navpath)
    calc_error(navpath,cfg.truthpath);
end
toc
%%
% calc_radial_error(cfg.truthpath,navpath,navpath2);
