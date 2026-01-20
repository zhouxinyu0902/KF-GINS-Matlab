clear
%% 定义全局参数
global rangstd
rangstd = 1;
global depstd
depstd = 0.5;
rng(1)
feedback = 0; % 是否反馈，不反馈则可以观察参数
glvs
dxyz_original = [0, 200, 0;
                -250, 200, 0;
                -400, 100, 0;
                -400,-150, 0;
                -250, -250, 0;
                0, -250, 0;
                100, -150, 0;
                100, 100, 0;
                -200, 0, 0];
%%
for ii=1
    %% 定义参数+加载过程配置
    param = Param();
    cfg = ProcessConfig_imuInforOpt('基于优化惯性数据');
    
    %% 加载数据
    % imudata
    imudata = importdata(cfg.imufilepath);
    imustarttime = imudata(1, 1);
    imuendtime = imudata(end, 1);

    % 构造距离信息
    cfg.userange=1;
    fieldname = sprintf('rangefile%dpath', ii);
    range = importdata(cfg.(fieldname));
    id=10;
    rangedata=range(id:id:end,:);
    rangedata(:,3) = rangedata(:,3) + normrnd(0,rangstd,size(rangedata(:,3)));
    rangestarttime = rangedata(1, 1);
    rangeendtime = rangedata(end, 1);
    % id = 420;
    % for i=1:3
    %     range{i} = range{i}(id:id:end,:);
    % end
    % rangedata = zeros(size(range{1}));
    % for i=1:3
    %     rangedata(i:3:end,:)=range{seq(nnn,i)}(i:3:end,:);
    % end
    % % rangedata(1,:)=[];
   

    % height data
    height = importdata(cfg.depthfilepath);
    % height = truth(:,[2,5]);
    % height(:,2) = height(:,2) + normrnd(0,depstd,size(height(:,2)));
    heistarttime = height(1, 1);
    heitendtime = height(end, 1);
    heightdata = height;
    heightstarttime = heightdata(1, 1);
    heightendtime = heightdata(end, 1);
    %% 获取处理时间，调整时间
    if imustarttime > heistarttime
        starttime = imustarttime;
    else
        starttime = heistarttime;
    end
    if imuendtime > heitendtime
        endtime = heitendtime;
    else
        endtime = imuendtime;
    end
    if cfg.starttime < starttime
        cfg.starttime = starttime;
    end
    if cfg.endtime > endtime
        cfg.endtime = endtime;
    end
    % data in process interval
    imudata = imudata(imudata(:,1) >= cfg.starttime, :);
    imudata = imudata(imudata(:,1) <= cfg.endtime, :);
    rangedata = rangedata(rangedata(:, 1) >= cfg.starttime, :);
    rangedata = rangedata(rangedata(:, 1) <= cfg.endtime, :);
    % rangedata(length(rangedata)+1,:)=zeros(1,6);
    heightdata = heightdata(heightdata(:, 1) >= cfg.starttime, :);
    heightdata = heightdata(heightdata(:, 1) <= cfg.endtime, :);
    %% 画信标图
    % myfigurestartup(5,5,'prese')
    % plot(truth(:,4),truth(:,3))
    % hold on
    % plot(truth(1,4),truth(1,3),'.')
    % for i=1:3
    %     plot(rangedata(i,5)/pi*180,rangedata(i,4)/pi*180,'*')
    % end
    % legend('轨迹','起点','信标1','信标2','信标3')
    %% 设置文件保存路径
    % navp = [cfg.outputfolder, '/NavResult'];
    navp = [cfg.outputfolder, '/'];
    if cfg.userange
        % navpath = [navp, 'RANGE-avpfedbck',num2str(ii)]; % 反馈时选择反馈avp
        % navpath = [navp, 'RANGE',num2str(ii)]; % 仅仅使用滤波
        % navpath = [navp, 'RANGE',num2str(ii)]; % 使用滤波-参数
        % navpath = [navp, 'RANGE1-filter+Optim-',num2str(ii)]; % 滤波+优化
        % navpath = [navp, 'pureins-1']; % 纯惯导-参数
        navpath = [navp, 'pureins-2']; % 纯惯导-优化
        % navpath = [navp, 'pureins']; % 纯惯导
        % navpath = [navp, 'RANGE',num2str(nnn),'-adap'];
        % navpath = [navp, 'RANGE',num2str(nnn),'-adap-42min'];
        disp("use RANGE data!");
    end
    % max_interv = 42;
    navpath = [navpath, '.nav'];
    navfp = fopen(navpath, 'wt');
    %
    imuerrpath = [cfg.outputfolder, '/ImuError.txt'];
    imuerrfp = fopen(imuerrpath, 'wt');
    %
    % stdpath = [cfg.outputfolder, '/NavSTD.txt'];
    % stdfp = fopen(stdpath, 'wt');
    %
    if feedback==0
        xkpath = [cfg.outputfolder, '/xk_range.txt'];
        xkfp = fopen(xkpath, 'wt');
    end

    %% 调试
    disp("Start GNSS/RANGE Processing!");
    lastprecent = 0;
    % %% 用于对比几类改进方法的参数
    % ki=1;
    % ki2=1;
    % indexrecord=zeros(1,12);
    % indexrecord2=zeros(1,12);
    % z=zeros(11,6);
    % indexrecord(1) = 1;
    % indexrecord2(1) = 1;
    % % === 前向滤波结果 (FFR) ===
    % % 存储协方差矩阵 (3D 矩阵)
    % P_F_store = zeros(2, 2, 463555);
    %
    % % === 后向滤波结果 (BFR) ===
    % % 存储状态估计
    % nav11 = zeros(3, 463555);
    % nav112 = zeros(3, 463555);
    % % 存储协方差矩阵 (3D 矩阵)
    % P_B_store = zeros(2, 2, 463555);
    % P_B_store2 = zeros(2, 2, 463555);
    %% initialization
    [kf, navstate] = myInitialize_15state(cfg);
    laststate = navstate;
    pos0 = navstate.pos;
    % data index preprocess
    lastimu = imudata(1, :)';
    thisimu = imudata(1, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);

    rangeindex = 1;
    while rangedata(rangeindex, 1) < thisimu(1, 1)
        rangeindex = rangeindex + 1;
    end
    %% 正式滤波
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% MAIN PROCEDD PROCEDURE!
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for imuindex = 2:size(imudata, 1)
        % 1、set value of last state
        lastimu = thisimu;
        laststate = navstate;
        thisimu = imudata(imuindex, :)';
        imudt = thisimu(1, 1) - lastimu(1, 1);

        % 2、compensate IMU error
        % thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * navstate.gyrbias)./(ones(3, 1) + navstate.gyrscale);
        % thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * navstate.accbias)./(ones(3, 1) + navstate.accscale);
        % thisimu(2:4, 1) = thisimu(2:4, 1) - imudt * cfg.gyrbiasstd;
        % thisimu(5:7, 1) = thisimu(5:7, 1) - imudt * cfg.accbiasstd;

        % 3、adjust range index
        while (rangeindex <= size(rangedata, 1) && rangedata(rangeindex, 1) < lastimu(1, 1))
            rangeindex = rangeindex + 1;
        end
        if (rangeindex > size(rangedata, 1))
            % rangeindex = rangeindex - 1;
            disp('range file END!');
            break;
        end
        % 4、determine whether gnss update is required
        if lastimu(1, 1) == rangedata(rangeindex, 1)
            % 测量更新
            Rangedata = rangedata(rangeindex,:);
            depthdata = height(imuindex,1:2);         
            % 惯导推算
            [A,C,M]=calculate_Matrix(navstate,Rangedata);
            bg=cfg.gyrbiasstd*imudt;
            ba=cfg.accbiasstd*imudt;
            sigma_g=cfg.gyrarw;
            sigma_a=cfg.accvrw;
            [thisimu(2:4),thisimu(5:7), num_iter(rangeindex)] = solve_coupled_imus(thisimu(2:4), thisimu(5:7), bg, ba, ...
                                                    M, C, A, ...
                                                    sigma_g, sigma_a);
            % thisimu(2:4, 1) = thisimu(2:4, 1) - imudt * cfg.gyrbiasstd;
            % thisimu(5:7, 1) = thisimu(5:7, 1) - imudt * cfg.accbiasstd;
            imudt = thisimu(1, 1) - lastimu(1, 1);
            navstate = InsMech(laststate, lastimu, thisimu);
            navstate.pos(3) = height(imuindex,2); % 天向位置约束
            kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);

            kf = myRangeUpdate_imuOpti(navstate, Rangedata, depthdata, kf);

            if feedback==1
                [kf, navstate] = myErrorFeedback_range(kf, navstate);
                % [kf, navstate] = myErrorFeedback_range_posonly(kf, navstate);
                % [kf, navstate] = myErrorFeedback_15state(kf, navstate);
            end
            z(rangeindex,:) = kf.Z;
            rangeindex = rangeindex + 1;
            laststate = navstate;
        elseif (lastimu(1, 1) < rangedata(rangeindex, 1) && thisimu(1, 1) > rangedata(rangeindex, 1))
            % 插值imu
            [firstimu, secondimu] = interpolate(lastimu, thisimu, rangedata(rangeindex, 1));
            % 惯导推算
            imudt = firstimu(1, 1) - lastimu(1, 1);
            navstate = InsMech(laststate, lastimu, firstimu);
            % navstate.pos(3) = height(imuindex,2);
            kf = myInsPropagate_15state(navstate, firstimu, imudt, kf);
            % 测量更新
            Rangedata = rangedata(rangeindex,:);
            depthdata = [height(imuindex,1),height(imuindex,2)];
            % if abs(navstate.time-122235-max_interv*60)<20||abs(navstate.time-122235-max_interv*2*60)<20||abs(navstate.time-122235-max_interv*3*60)<20
            %     tt = Rangedata(1);
            %     Rangedata1 = rangedata1(rangedata1(:,1)==tt,:) ;
            %     Rangedata2 = rangedata2(rangedata2(:,1)==tt,:) ;
            %     Rangedata3 = rangedata3(rangedata3(:,1)==tt,:) ;
            %     Rangedata1(3) = Rangedata1(3) + randn * rangstd ;
            %     Rangedata2(3) = Rangedata2(3) + randn * rangstd ;
            %     Rangedata3(3) = Rangedata3(3) + randn * rangstd ;
            %     kf = my3RangeUpdate(navstate,Rangedata1 ,Rangedata2, Rangedata2, depthdata, kf);
            %     fprintf('   3距离\n')
            % else
            kf = myRangeUpdate(navstate, Rangedata, depthdata, kf);
            % kf = myRangeUpdate_only(navstate, Rangedata, kf);
            % end

            if feedback==1
                [kf, navstate] = myErrorFeedback_range(kf, navstate);
                % [kf, navstate] = myErrorFeedback_15state(kf, navstate);
                % [kf, navstate] = myErrorFeedback_range_posonly(kf, navstate);
            end
            z(rangeindex,1) =kf.Z(1);% 残差记录
            z(rangeindex,2) =kf.Znew;
            z(rangeindex,3) =kf.alpha;
            z(rangeindex,4) =kf.d_squared ;
            z(rangeindex,5) =kf.chi2_threshold ;
            z(rangeindex,6) =kf.is_anomaly ;
            rangeindex = rangeindex + 1;
            laststate = navstate;
            lastimu = firstimu;
            % do propagation for second imu
            imudt = secondimu(1, 1) - lastimu(1, 1);
            navstate = InsMech(laststate, lastimu, secondimu);
            % navstate.pos(3) = height(imuindex,2) + randn*depstd; % 天向位置约束
            kf = myInsPropagate_15state(navstate, secondimu, imudt, kf);

            %% 反向推算
            % thisimu1 = imudata(imuindex-1, :)';
            % lastimu1 = imudata(imuindex, :)';
            % navstate_1 = navstate;
            % laststate_1 = navstate;
            %
            % kf1 = kf;
            % ki = ki+1;
            %
            % nav11(:,imuindex) = laststate_1.pos;
            % P_B_store(:,:,imuindex)=kf1.P(1:2,1:2);
            % indexrecord(ki) = imuindex;
            %
            % for ii=indexrecord(ki)-1:-1:indexrecord(ki-1)
            %     laststate_1 = InsMechBackward(navstate_1,lastimu1,thisimu1);
            %     laststate_1.pos(3) = height(ii,2);
            %     kf1 = myInsPropagate_15state(laststate_1, thisimu1, 0.01, kf1);
            %     nav11(:,ii) = laststate_1.pos;
            %     P_B_store(:,:,ii)=kf1.P(1:2,1:2);
            %     lastimu1 = thisimu1;
            %     thisimu1 = imudata(ii, :)';
            %     navstate_1 = laststate_1;
            % end
            %% 反向推算+滤波
            % thisimu2 = imudata(imuindex-1, :)';
            % lastimu2 = imudata(imuindex, :)';
            % navstate_2 = navstate;
            % laststate_2 = navstate;
            %
            % kf2 = kf;
            % ki2 = ki2+1;
            %
            % nav112(:,imuindex) = laststate_2.pos;
            % P_B_store2(:,:,imuindex)=kf2.P(1:2,1:2);
            % indexrecord2(ki2) = imuindex;
            % for ii = indexrecord2(ki2)-1:-1:indexrecord2(ki2-1)
            %     if ii==indexrecord2(ki2-1)
            %         laststate_2 = InsMechBackward(navstate_2,lastimu2,thisimu2);
            %         laststate_2.pos(3) = height(ii,2);
            %         % 反向滤波
            %         if rangeindex == 2
            %             Rangedata = zeros(1,6);
            %             Rangedata(4:6) = rangedata3(1,4:6);
            %             Rangedata(3) = caldot2dot(Rangedata(4:6),pos0');
            %         else
            %             Rangedata = rangedata(rangeindex-2,:);
            %             % Rangedata(3) = Rangedata(3) + randn*rangstd ;
            %         end
            %         depthdata = height(ii,1:2);
            %         kf2 = myRangeUpdate(laststate_2, Rangedata, depthdata, kf2);
            %         [kf2, laststate_2] = myErrorFeedback_range(kf2, laststate_2);
            %
            %         % if rangeindex>2 & abs(kf2.Z(1))-abs(z(rangeindex-2,1))>40
            %         %     break;
            %         % else
            %         %     nav11(:,ii-1) = laststate_1.pos;
            %         % end
            %         z_back(ki2-1,1) = kf2.Z(1);% 残差记录
            %         z_back(ki2-1,2) = kf2.Znew;
            %         z_back(ki2-1,3) = kf2.alpha;
            %         z_back(ki2-1,4) = kf2.d_squared ;
            %         z_back(ki2-1,5) = kf2.chi2_threshold ;
            %         z_back(ki2-1,6) = kf2.is_anomaly ;
            %     else
            %         laststate_2 = InsMechBackward(navstate_2,lastimu2,thisimu2);
            %         laststate_2.pos(3) = height(ii,2);
            %         kf2 = myInsPropagate_15state(laststate_2, thisimu2, 0.01, kf2);
            %     end
            %     lastimu2 = thisimu2;
            %     thisimu2 = imudata(ii, :)';
            %     navstate_2 = laststate_2;
            %     nav112(:,ii) = laststate_2.pos;
            %     P_B_store2(:,:,ii) = kf2.P(1:2,1:2);
            % end
        else
            % 5、only do propagation
            % INS mechanization
            navstate = InsMech(laststate, lastimu, thisimu);
            navstate.pos(3) = height(imuindex,2) ; % 天向位置约束
            % navstate.vel(3) = -(height(imuindex,2)+ randn*depstd-height(imuindex-1,2)- randn*depstd)/imudt; % 天向速度约束
            % error propagation
            kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
        end

        % 6、save data
        % xkk(imuindex-1,:)=[navstate.time;kf.x];
        % write navresult to file
        % P_F_store(:,:,imuindex-1)=kf.P(1:2,1:2);
        % nav = zeros(11, 1);
        nav(2, 1) = navstate.time;
        nav(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
        nav(6:8, 1) = navstate.vel;
        nav(9:11, 1) = navstate.att * param.R2D;
        fprintf(navfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);
        % 保存估计的状态值
        if feedback==0
            xk = zeros(16, 1);
            xk(1) = navstate.time;
            xk(2:16) = kf.x(1:15);
            fprintf(xkfp, '%12.6f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f\n', xk);
        end

        % write imu error, convert to common unit
        imuerror = zeros(13, 1);
        imuerror(1, 1) = navstate.time;
        imuerror(2:4, 1) = navstate.gyrbias * param.R2D * 3600;
        imuerror(5:7, 1) = navstate.accbias * 1e5;
        imuerror(8:10, 1) = navstate.gyrscale * 1e6;
        imuerror(11:13, 1) = navstate.accscale * 1e6;
        fprintf(imuerrfp, '%12.6f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', imuerror);
        %
        % % write state std, convert to common unit
        % std = zeros(1, 22);
        % std(1) = navstate.time;
        % for idx=1:21
        %     std(idx + 1) = sqrt(kf.P(idx, idx));
        % end
        % std(8:10) = std(8:10) * param.R2D;
        % std(11:13) = std(11:13) * param.R2D *3600;
        % std(14:16) = std(14:16) * 1e5;
        % std(17:22) = std(17:22) * 1e6;
        % fprintf(stdfp, '%12.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f %8.6f \n', std);
        %

        % print processing information
        % if (imuindex / size(imudata, 1) - lastprecent > 0.20)
        %     disp("processing " + num2str(floor(imuindex * 100 / size(imudata, 1))) + " %!");
        %     lastprecent = imuindex / size(imudata, 1);
        % end
        % check whether gnss data is valid

    end
    % close file
    fclose(navfp);
    if feedback==0
        fclose(xkfp);
    end
    fclose(imuerrfp);
    % fclose(stdfp);
    disp("range/INS Integration Processing Finished!");
end
%%
pureins_path='基于优化惯性数据/output/pureins.nav';
pureins_path1='基于优化惯性数据/output/pureins-1.nav';
pureins_path2='基于优化惯性数据/output/pureins-2.nav';
plot_trj(cfg.truthpath,pureins_path,pureins_path1,pureins_path2)
calc_radial_error(cfg.truthpath,pureins_path,pureins_path1,pureins_path2)
%% 残差及自适应因子
% adap_factor_visualize(z,z_back)
% adap_factor_visualize_gui(z, z_back)
%% 误差绘图
% navpath='基于优化惯性数据/output/NavResult-pureins-1.nav';
calc_error(navpath,cfg.truthpath)
plot_result(navpath,'full')
%% 对比纯惯性优化前后
% plot_trj(cfg.truthpath,navpath,pureins_path)
% plot_result('基于优化惯性数据/output/NavResult-pureins.nav','full')
% plot_imuerror
%% 对比距离辅助的优化前后
% navpath1='基于优化惯性数据/output/NavResult-RANGE-11.nav';
% navpath2='基于优化惯性数据/output/NavResult-RANGE1.nav';
% plot_trj(cfg.truthpath,navpath1,navpath2)
% calc_radial_error(cfg.truthpath,navpath1,navpath2)
%% 估计误差对比
if feedback==0
    plot_xk(xkpath,navpath,cfg.truthpath)
end
%%
navp = [cfg.outputfolder, '/NavResult'];
for i=1:9
    path{i} = [navp, '-RANGE',num2str(i),'.nav'];
    % path{i} = [navp, '-RANGE-avpfedbck',num2str(i),'.nav'];
    path1{i} = [navp, '-RANGE1',num2str(i),'.nav'];
end
plot_trj_subplot(dxyz_original,cfg.truthpath,path{:})
calc_radial_error(cfg.truthpath,pureins_path,path{:},path1{:})
hold on
ylim([0,1500])
% ylim([0 600])
% hold on
% plot(420:420:420*11,abs(z(:,1)),'*')
% plot_result(navpath,'full')
%% 预备数据
% refadappath='惯导实验数据/output/NavResult-RANGE1-adap.nav';
% pureinspath = '惯导实验数据/output/NavResult-pureins.nav';
% refpath='惯导实验数据/output/NavResult-RANGE-11.nav';
% forward = importdata(navpath);%% 参考的位置
% nav000 = [d2r(truth(1:imuindex-1,3:4)),truth(1:imuindex-1,5)]';
% nav00 = [nav000(:,1),[d2r(forward(:,3:4)),forward(:,5)]'];
% nav11(:,1) = nav000(:,1);
% nav112(:,1) = nav000(:,1);
% %% 多个不同的间隔，进行对比
% refadappath1='惯导实验数据/output/NavResult-RANGE1-adap-28min.nav';
% refadappath2='惯导实验数据/output/NavResult-RANGE1-adap-42min.nav';
% refadappath3='惯导实验数据/output/NavResult-RANGE1-adap-70min.nav';
% calc_radial_error(cfg.truthpath,pureinspath, refadappath,refadappath1, refadappath2, refadappath3)
% hold on
% plot(xlim,[400,400],'DisplayName','400m界限')
% ylim([0 1500])
% %% 保存一些数据
% save 惯导实验数据\output\data.mat nav11 nav112 nav00 nav000 P_B_store P_F_store indexrecord ...
%     refadappath pureinspath refpath navp P_B_store2 navpath
% %% 计算多个结果径向误差，对比
% calc_radial_error(cfg.truthpath, '惯导实验数据/output/NavResult-pureins.nav', ...
%     '惯导实验数据/output/NavResult-RANGE-11.nav',navpath, ...
%     output_file_backforward,output_file_rotate,output_file_rotateback)
% hold on
% ylim([0,1500])
% plot(xlim,[400,400],'DisplayName','400m界限')
% % 多个轨迹对比
% plot_trj(cfg.truthpath,navpath,output_file_backforward,output_file_rotate,output_file_rotateback)
