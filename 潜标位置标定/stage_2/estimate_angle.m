% close all
clear
glvs
%% 定义参数+加载过程配置
param = Param();
% path_used = 'D:\Github\KF-GINS-Matlab\潜标位置标定\data3\';
% path_input = 'D:\Github\KF-GINS-Matlab\潜标位置标定\data3\input\';
TYPE = {'square','line','circle','square-bea','line-bea','circle-bea'};

% path_used = 'D:\Github\KF-GINS-Matlab\潜标位置标定\Square_ArrayCenter_Trj\';
% type =  TYPE{1};

% path_used = 'D:\Github\KF-GINS-Matlab\潜标位置标定\Line_ArrayCenter_Trj\';
% type =  TYPE{2};

% path_used = 'D:\Github\KF-GINS-Matlab\潜标位置标定\Circle_ArrayCenter_Trj\';
% type =  TYPE{3};

% path_used = 'D:\Github\KF-GINS-Matlab\潜标位置标定\Square_Beacon1_Trj\';
% type =  TYPE{4};
% 
% path_used = 'D:\Github\KF-GINS-Matlab\潜标位置标定\Line_Beacon1_Trj\';
% type =  TYPE{5};

path_used = 'D:\Github\KF-GINS-Matlab\潜标位置标定\Circle_Beacon1_Trj\';
type =  TYPE{6};

cfg = Config_10state(path_used,'align');
theta_next_step = 20;
phi_next_step = 45;

for id = 1:2
    %% 加载数据
    % imudata
    imudata = importdata([cfg.input, '/imu_data.txt']);
    imustarttime = imudata(1, 1);
    imuendtime = imudata(end, 1);
    truth = importdata([cfg.input, '/truth.nav']);
    feedback = 1;
    %% 获取距离
    % 构造距离信息
    if id == 1
        rangedata1 = importdata([cfg.input, '/range1.txt']);
        rangedata2 = importdata([cfg.input, '/range2.txt']);
        rangedata3 = importdata([cfg.input, '/range3.txt']);
        %
        % rangedata1 = importdata(cfg.rangefile1path);
        % rangedata2 = importdata(cfg.rangefile2path);
        % rangedata3 = importdata(cfg.rangefile3path);
    else
        rangedata1 = importdata([cfg.input, '/range1_calib_1.txt']);
        rangedata2 = importdata([cfg.input, '/range2_calib_1.txt']);
        rangedata3 = importdata([cfg.input, '/range3_calib_1.txt']);
    end
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
    rangedata1 = rangedata1(rangedata1(:, 1) >= cfg.starttime, :);
    rangedata1 = rangedata1(rangedata1(:, 1) <= cfg.endtime, :);
    rangedata2 = rangedata2(rangedata2(:, 1) >= cfg.starttime, :);
    rangedata2 = rangedata2(rangedata2(:, 1) <= cfg.endtime, :);
    rangedata3 = rangedata3(rangedata3(:, 1) >= cfg.starttime, :);
    rangedata3 = rangedata3(rangedata3(:, 1) <= cfg.endtime, :);
    %% 设置文件保存路径
    navpath = [cfg.outputfolder, '/NavResult_est_angle.nav'];
    navfp = fopen(navpath, 'wt');

    xkpath = [cfg.outputfolder, '/xk_est_angle.nav'];
    xkfp = fopen(xkpath, 'wt');
    %% 调试
    disp("Start Processing!");
    lastprecent = 0;
    %% 初始化
    [kf, navstate] = myInitialize_10state(cfg);
    kf.depthstd = 0.2;
    laststate = navstate;
    kf.rangstd = 10;
    lastimu = imudata(1, :)';
    thisimu = imudata(1, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);

    rangeindex = 2;
    rangedata = [rangedata1(rangeindex,:);
        rangedata2(rangeindex,:);
        rangedata3(rangeindex,:)];
    while rangedata(rangeindex, 1) < thisimu(1, 1)
        rangeindex = rangeindex + 1;

    end
    cfg.userange = 1;
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
    %%
    fclose all;
    disp("PureIns Integration Processing Finished!");
    %%
    % plot_result(navpath)
    %%
    truthpath=[cfg.input, '/truth.nav'];
    % calc_radial_error(cfg.truthpath,navpath)
    calc_error(navpath,truthpath);
    xk = importdata(xkpath);
    figure
    subplot 121
    plot(r2d(xk(:,10)));
    subplot 122
    plot(r2d(xk(:,11)));

    path_pos=[cfg.input,'\beacon_pos.mat'];
    load(path_pos)
    %%
    theta_est =  xk(end,10);
    phi_est = xk(end,11);
    S_gnss_geo = rangedata(:,4:6);
    string = [type,'_',num2str(id)];
    %%
    [S_est_xyz,theta_next_step,phi_next_step] = show_result(string,S_gnss_geo,S_true_geo,pos0_geo,theta_est,phi_est,theta_next_step,phi_next_step);
    
    %% 重构阶段1的距离数据，用于二次处理，增加精度
    path11 = {[cfg.input, '/range1.txt'],[cfg.input, '/range2.txt'],[cfg.input, '/range3.txt']};
    range_reconstruct(path11, path_pos,cfg.input, id ,S_est_xyz);
    %% 重构阶段2的距离数据
    path_range = {[cfg.input, '/range1_stage2.txt'],[cfg.input, '/range2_stage2.txt'],[cfg.input, '/range3_stage2.txt']};
    range_reconstruct(path_range,path_pos,cfg.input,id,S_est_xyz);
end