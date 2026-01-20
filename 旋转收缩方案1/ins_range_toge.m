clear
%% 定义全局参数
nnn=1;
seq=[1,2,3];
global rangstd
rangstd = 10;
global depstd
depstd = 0.5;
rng(1)
feedback=1; % 是否反馈，不反馈则可以观察参数
glvs
%% 定义参数+加载过程配置
param = Param();
path='旋转收缩方案1/input6';
% dataget_truth
cfg = ProcessConfigforSemiPhy_all(path);
%% 加载数据
% imudata
imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);

% range data
rangedata1 = importdata(cfg.rangefile1path);
rangedata2 = importdata(cfg.rangefile2path);
rangedata3 = importdata(cfg.rangefile3path);
range = {rangedata1,rangedata2,rangedata3};
% 构造距离信息
id = 420;
for i=1:3
    range{i} = range{i}(id:id:end,:);
end
rangedata = zeros(size(range{1}));
for i=1:3
    rangedata(i:3:end,:)=range{seq(nnn,i)}(i:3:end,:);
end
% rangedata(1,:)=[];
rangedata(:,3) = rangedata(:,3) + normrnd(0,rangstd,size(rangedata(:,3)));
rangestarttime = rangedata(1, 1);
rangeendtime = rangedata(end, 1);

% height data
truth = importdata(cfg.truthpath);
height = truth(:,[2,5]);
height(:,2) = height(:,2) + normrnd(0,depstd,size(height(:,2)));
heistarttime = height(1, 1);
heitendtime = height(end, 1);
heightdata = height(id*100:id*100:end,:);

heightstarttime = heightdata(1, 1);
heightendtime = heightdata(end, 1);
%%
output_file  = ['D:\Github\KF-GINS-main\dataset_exper','\rangedata', '.txt'];
try
    writematrix(rangedata, output_file, 'Delimiter', ' ');
    fprintf('信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
output_file  = ['D:\Github\KF-GINS-main\dataset_exper','\heightdata', '.txt'];
try
    writematrix(heightdata, output_file, 'Delimiter', ' ');
    fprintf('信息已成功写入到 %s\n', output_file);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
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
myfigurestartup(5,5,'prese')
plot(truth(:,4),truth(:,3))
hold on
plot(truth(1,4),truth(1,3),'*')
marker=[">","hexagram","pentagram"];
for i=1:3
    plot(rangedata(i,5)/pi*180,rangedata(i,4)/pi*180,marker(i))
end
legend('轨迹','起点','信标1','信标2','信标3')
xygo('经度/°','纬度/°')
% title('仿真信标及仿真轨迹')

title('仿真信标及实测轨迹')
%%
for type = ["PureIns","EKF","AEKF"]
    if strcmp(type,'PureIns')
        cfg.userange=0;
        cfg.adap=0;
    elseif strcmp(type,'EKF')
        cfg.userange=1;
        cfg.adap=0;
    elseif strcmp(type,'AEKF')
        cfg.userange=1;
        cfg.adap=1;
    end
    %% 设置文件保存路径
    if cfg.userange
        navpath = [cfg.outputfolder, '/',sprintf('%s.nav',type)];
        disp("use RANGE data!");
        disp("Start GNSS/RANGE Processing!");
    else
        navpath =[cfg.outputfolder,'/PureIns.nav'];
        disp("PURE INS!");
        disp("Start INS Processing!");
    end
    % max_interv = 42;

    navfp = fopen(navpath, 'wt');

    %
    % imuerrpath = [cfg.outputfolder, '/ImuError.txt'];
    % imuerrfp = fopen(imuerrpath, 'wt');
    %
    % stdpath = [cfg.outputfolder, '/NavSTD.txt'];
    % stdfp = fopen(stdpath, 'wt');
    %
    % if feedback==0
    %     xkpath = [cfg.outputfolder, '/xk_range.txt'];
    %     xkfp = fopen(xkpath, 'wt');
    % end
    %% 调试
    
    lastprecent = 0;
    %% 用于对比几类改进方法的参数
    ki=1;
    ki2=1;
    indexrecord=zeros(1,12);
    indexrecord2=zeros(1,12);
    z=zeros(11,6);
    indexrecord(1) = 1;
    indexrecord2(1) = 1;
    % === 前向滤波结果 (FFR) ===
    % 存储协方差矩阵 (3D 矩阵)
    P_F_store = zeros(2, 2, 463555);

    % === 后向滤波结果 (BFR) ===
    % 存储状态估计
    nav11 = zeros(3, 463555);
    nav112 = zeros(3, 463555);
    % 存储协方差矩阵 (3D 矩阵)
    P_B_store = zeros(2, 2, 463555);
    P_B_store2 = zeros(2, 2, 463555);
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
        % thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * navstate.gyrbias);
        % thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * navstate.accbias);

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
        if lastimu(1, 1) == rangedata(rangeindex, 1)&& cfg.userange==1
            % 测量更新
            Rangedata = rangedata(rangeindex,:);
            depthdata = height(imuindex,1:2);
            if cfg.adap==1
                kf = myRangeUpdate_adap(navstate, Rangedata, depthdata, kf);
            else
                kf = myRangeUpdate(navstate, Rangedata, depthdata, kf);
            end
            if feedback==1
                [kf, navstate] = myErrorFeedback_range(kf, navstate);
                % [kf, navstate] = myErrorFeedback_range_posonly(kf, navstate);
            end

            rangeindex = rangeindex + 1;
            laststate = navstate;
            % 惯导推算
            imudt = thisimu(1, 1) - lastimu(1, 1);
            navstate = InsMech(laststate, lastimu, thisimu);
            kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);

            %% 反向推算
            thisimu1 = imudata(imuindex-1, :)';
            lastimu1 = imudata(imuindex, :)';
            navstate_1 = navstate;
            laststate_1 = navstate;

            kf1 = kf;
            ki = ki+1;

            nav11(:,imuindex) = laststate_1.pos;
            P_B_store(:,:,imuindex)=kf1.P(1:2,1:2);
            indexrecord(ki) = imuindex;

            for ii=indexrecord(ki)-1:-1:indexrecord(ki-1)
                laststate_1 = InsMechBackward(navstate_1,lastimu1,thisimu1);
                laststate_1.pos(3) = height(ii,2);
                kf1 = myInsPropagate_15state(laststate_1, thisimu1, 0.01, kf1);
                nav11(:,ii) = laststate_1.pos;
                P_B_store(:,:,ii)=kf1.P(1:2,1:2);
                lastimu1 = thisimu1;
                thisimu1 = imudata(ii, :)';
                navstate_1 = laststate_1;
            end
            %% 反向推算+滤波
            thisimu2 = imudata(imuindex-1, :)';
            lastimu2 = imudata(imuindex, :)';
            navstate_2 = navstate;
            laststate_2 = navstate;

            kf2 = kf;
            ki2 = ki2+1;

            nav112(:,imuindex) = laststate_2.pos;
            P_B_store2(:,:,imuindex)=kf2.P(1:2,1:2);
            indexrecord2(ki2) = imuindex;
            for ii = indexrecord2(ki2)-1:-1:indexrecord2(ki2-1)
                if ii==indexrecord2(ki2-1)
                    laststate_2 = InsMechBackward(navstate_2,lastimu2,thisimu2);
                    laststate_2.pos(3) = height(ii,2);
                    % 反向滤波
                    if rangeindex == 2
                        Rangedata = zeros(1,6);
                        Rangedata(4:6) = rangedata3(1,4:6);
                        Rangedata(3) = caldot2dot(Rangedata(4:6),pos0');
                    else
                        Rangedata = rangedata(rangeindex-2,:);
                    end
                    depthdata = height(ii,1:2);
                    kf2 = myRangeUpdate_adap(laststate_2, Rangedata, depthdata, kf2);
                    [kf2, laststate_2] = myErrorFeedback_range(kf2, laststate_2);

                    % if rangeindex>2 & abs(kf2.Z(1))-abs(z(rangeindex-2,1))>40
                    %     break;
                    % else
                    %     nav11(:,ii-1) = laststate_1.pos;
                    % end
                    z_back(ki2-1,1) = kf2.Z(1);% 残差记录
                    z_back(ki2-1,2) = kf2.Znew;
                    z_back(ki2-1,3) = kf2.alpha;
                    z_back(ki2-1,4) = kf2.d_squared ;
                    z_back(ki2-1,5) = kf2.chi2_threshold ;
                    z_back(ki2-1,6) = kf2.is_anomaly ;
                else
                    laststate_2 = InsMechBackward(navstate_2,lastimu2,thisimu2);
                    laststate_2.pos(3) = height(ii,2);
                    kf2 = myInsPropagate_15state(laststate_2, thisimu2, 0.01, kf2);
                end
                lastimu2 = thisimu2;
                thisimu2 = imudata(ii, :)';
                navstate_2 = laststate_2;
                nav112(:,ii) = laststate_2.pos;
                P_B_store2(:,:,ii) = kf2.P(1:2,1:2);
            end

        elseif (lastimu(1, 1) < rangedata(rangeindex, 1) && thisimu(1, 1) > rangedata(rangeindex, 1))&& cfg.userange==1
            % 插值imu
            [firstimu, secondimu] = interpolate(lastimu, thisimu, rangedata(rangeindex, 1));
            % 惯导推算
            imudt = firstimu(1, 1) - lastimu(1, 1);
            navstate = InsMech(laststate, lastimu, firstimu);
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
            %     kf = my3RangeUpdate(navstate,Rangedata1 ,Rangedata2, Rangedata3, depthdata, kf);
            %     fprintf('   3距离\n')
            % else
            if cfg.adap==1
                kf = myRangeUpdate_adap(navstate, Rangedata, depthdata, kf);
            else
                kf = myRangeUpdate(navstate, Rangedata, depthdata, kf);
            end
            % end

            if feedback==1
                [kf, navstate] = myErrorFeedback_range(kf, navstate);
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
            kf = myInsPropagate_15state(navstate, secondimu, imudt, kf);

            %% 反向推算
            thisimu1 = imudata(imuindex-1, :)';
            lastimu1 = imudata(imuindex, :)';
            navstate_1 = navstate;
            laststate_1 = navstate;

            kf1 = kf;
            ki = ki+1;

            nav11(:,imuindex) = laststate_1.pos;
            P_B_store(:,:,imuindex)=kf1.P(1:2,1:2);
            indexrecord(ki) = imuindex;

            for ii=indexrecord(ki)-1:-1:indexrecord(ki-1)
                laststate_1 = InsMechBackward(navstate_1,lastimu1,thisimu1);
                laststate_1.pos(3) = height(ii,2);
                kf1 = myInsPropagate_15state(laststate_1, thisimu1, 0.01, kf1);
                nav11(:,ii) = laststate_1.pos;
                P_B_store(:,:,ii)=kf1.P(1:2,1:2);
                lastimu1 = thisimu1;
                thisimu1 = imudata(ii, :)';
                navstate_1 = laststate_1;
            end
            %% 反向推算+滤波
            thisimu2 = imudata(imuindex-1, :)';
            lastimu2 = imudata(imuindex, :)';
            navstate_2 = navstate;
            laststate_2 = navstate;

            kf2 = kf;
            ki2 = ki2+1;

            nav112(:,imuindex) = laststate_2.pos;
            P_B_store2(:,:,imuindex)=kf2.P(1:2,1:2);
            indexrecord2(ki2) = imuindex;
            for ii = indexrecord2(ki2)-1:-1:indexrecord2(ki2-1)
                if ii==indexrecord2(ki2-1)
                    laststate_2 = InsMechBackward(navstate_2,lastimu2,thisimu2);
                    laststate_2.pos(3) = height(ii,2);
                    % 反向滤波
                    if rangeindex == 2
                        Rangedata = zeros(1,6);
                        Rangedata(4:6) = rangedata3(1,4:6);
                        Rangedata(3) = caldot2dot(Rangedata(4:6),pos0');
                    else
                        Rangedata = rangedata(rangeindex-2,:);
                    end
                    depthdata = height(ii,1:2);
                    kf2 = myRangeUpdate_adap(laststate_2, Rangedata, depthdata, kf2);
                    [kf2, laststate_2] = myErrorFeedback_range(kf2, laststate_2);

                    % if rangeindex>2 & abs(kf2.Z(1))-abs(z(rangeindex-2,1))>40
                    %     break;
                    % else
                    %     nav11(:,ii-1) = laststate_1.pos;
                    % end
                    z_back(ki2-1,1) = kf2.Z(1);% 残差记录
                    z_back(ki2-1,2) = kf2.Znew;
                    z_back(ki2-1,3) = kf2.alpha;
                    z_back(ki2-1,4) = kf2.d_squared ;
                    z_back(ki2-1,5) = kf2.chi2_threshold ;
                    z_back(ki2-1,6) = kf2.is_anomaly ;
                else
                    laststate_2 = InsMechBackward(navstate_2,lastimu2,thisimu2);
                    laststate_2.pos(3) = height(ii,2);
                    kf2 = myInsPropagate_15state(laststate_2, thisimu2, 0.01, kf2);
                end
                lastimu2 = thisimu2;
                thisimu2 = imudata(ii, :)';
                navstate_2 = laststate_2;
                nav112(:,ii) = laststate_2.pos;
                P_B_store2(:,:,ii) = kf2.P(1:2,1:2);
            end
        else
            % 5、only do propagation
            % INS mechanization
            navstate = InsMech(laststate, lastimu, thisimu);
            navstate.pos(3) = height(imuindex,2) ; % 天向位置约束
            % error propagation
            kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
        end

        % 6、save data
        % xkk(imuindex-1,:)=[navstate.time;kf.x];
        % write navresult to file
        P_F_store(:,:,imuindex-1)=kf.P(1:2,1:2);
        nav = zeros(11, 1);
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
        % imuerror = zeros(13, 1);
        % imuerror(1, 1) = navstate.time;
        % imuerror(2:4, 1) = navstate.gyrbias * param.R2D * 3600;
        % imuerror(5:7, 1) = navstate.accbias * 1e5;
        % imuerror(8:10, 1) = navstate.gyrscale * 1e6;
        % imuerror(11:13, 1) = navstate.accscale * 1e6;
        % fprintf(imuerrfp, '%12.6f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', imuerror);
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
    % fclose(imuerrfp);
    % fclose(stdfp);
    disp("range/INS Integration Processing Finished!");

    %% 误差绘图
    calc_error(navpath,cfg.truthpath)
    %%
    if cfg.adap==1
        nav11(:,imuindex:end)=[];
        nav112(:,imuindex:end)=[];
        forward = importdata(navpath);%% 参考的位置
        nav000 = [d2r(truth(1:imuindex-1,3:4)),truth(1:imuindex-1,5)]';
        nav00 = [nav000(:,1),[d2r(forward(:,3:4)),forward(:,5)]'];
        nav11(:,1) = nav000(:,1);
        nav112(:,1) = nav000(:,1);
        nav_rotatesum1(:,1) = nav000(:,1);
        myfigurestartup(14,7,'paper')
        for ki=2: size(rangedata,1)+1
            % nav_rotate = rotateTrajectory(nav00(:,indexrecord(ki-1)+1:indexrecord(ki)-1),...
            %     nav00(:,indexrecord(ki)));
            trajectory = nav112(:,indexrecord(ki-1)+1:indexrecord(ki)-1);
            if ki==5
                newstartPoint = nav00(:,indexrecord(ki-1));
            else
                newstartPoint = nav112(:,indexrecord(ki-1));
            end
            if ki==4
                newEndPoint = nav00(:,indexrecord(ki));
            else
                newEndPoint = nav112(:,indexrecord(ki));
            end
            % if ki==2||ki==3
            %     nav_rotate = nav00(:,indexrecord(ki-1)+1:indexrecord(ki)-1);
            % else
            trajectory = flip(trajectory,2);
            [nav_rotate, ~, ~] = rotateAndScaleTrajectory(trajectory, newstartPoint);
            trajectory = flip(trajectory,2);
            nav_rotate = flip(nav_rotate,2);
            [nav_rotate, ~, ~] = rotateAndScaleTrajectory(nav_rotate, newEndPoint);
            % end
            nav_rotatesum1(:,indexrecord(ki-1):indexrecord(ki)) = [newstartPoint,nav_rotate,newEndPoint];
            subplot(4,4,ki-1)
            plot(trajectory(2,:), ...
                trajectory(1,:))
            hold on
            plot(newEndPoint(2),newEndPoint(1),'*')
            plot(nav_rotate(2,1:end),nav_rotate(1,1:end))
            plot(nav000(2,indexrecord(ki-1)+1:indexrecord(ki)-1), ...
                nav000(1,indexrecord(ki-1)+1:indexrecord(ki)-1))
            if ki==2
                legend('backforward','dot','rotate','truth')
            end
        end
        subplot(4,4,ki)
        plot(nav_rotatesum1(2,:),nav_rotatesum1(1,:))
        % 不加滤波

        myfigurestartup(14,7,'paper')
        for ki=2:size(rangedata,1)+1
            % nav_rotate = rotateTrajectory(nav00(:,indexrecord(ki-1)+1:indexrecord(ki)-1),...
            %     nav00(:,indexrecord(ki)));
            trajectory = nav11(:,indexrecord(ki-1)+1:indexrecord(ki)-1);
            if ki==5
                newstartPoint = nav00(:,indexrecord(ki-1));
            else
                newstartPoint = nav11(:,indexrecord(ki-1));
            end
            if ki==4
                newEndPoint = nav00(:,indexrecord(ki));
            else
                newEndPoint = nav11(:,indexrecord(ki));
            end
            % if ki==2||ki==3
            %     nav_rotate = nav00(:,indexrecord(ki-1)+1:indexrecord(ki)-1);
            % else
            trajectory = flip(trajectory,2);
            [nav_rotate, ~, ~] = rotateAndScaleTrajectory(trajectory, newstartPoint);
            trajectory = flip(trajectory,2);
            nav_rotate = flip(nav_rotate,2);
            [nav_rotate, ~, ~] = rotateAndScaleTrajectory(nav_rotate, newEndPoint);
            % end
            nav_rotatesum2(:,indexrecord(ki-1):indexrecord(ki)) = [newstartPoint,nav_rotate,newEndPoint];
            subplot(4,4,ki-1)
            plot(trajectory(2,:), ...
                trajectory(1,:))
            hold on
            plot(newEndPoint(2),newEndPoint(1),'*')
            plot(nav_rotate(2,1:end),nav_rotate(1,1:end))
            plot(nav000(2,indexrecord(ki-1)+1:indexrecord(ki)-1), ...
                nav000(1,indexrecord(ki-1)+1:indexrecord(ki)-1))
            if ki==2
                legend('backforward','dot','rotate','truth')
            end
        end
        subplot(4,4,ki)
        plot(nav_rotatesum1(2,:),nav_rotatesum1(1,:))

        % 旋转缩放后向结果
        nav_rotscale1=importdata(navpath);
        nav_rotscale1(:,3:4)=r2d(nav_rotatesum1(1:2,2:end)');
        output_file_rotateback  = [cfg.outputfolder,'/GTS-BRC-AEKF', '.txt'];
        try
            writematrix(nav_rotscale1, output_file_rotateback, 'Delimiter', ' ');
            fprintf('nav_rotscale信息已成功写入到 %s\n', output_file_rotateback);
        catch ME
            error('错误：写入文件失败。错误信息：%s', ME.message);
        end
        %
        % 旋转缩放后向结果（不加后向滤波-效果不好）
        nav_rotscale1=importdata(navpath);
        nav_rotscale1(:,3:4)=r2d(nav_rotatesum2(1:2,2:end)');
        output_file_rotateback  = [cfg.outputfolder,'/GTS-BRC-1-AEKF', '.txt'];
        try
            writematrix(nav_rotscale1, output_file_rotateback, 'Delimiter', ' ');
            fprintf('nav_rotscale信息已成功写入到 %s\n', output_file_rotateback);
        catch ME
            error('错误：写入文件失败。错误信息：%s', ME.message);
        end
        % 旋转缩放后向结果
        nav_back=importdata(navpath);
        nav_back(:,3:4)=r2d(nav112(1:2,2:end)');
        output_file_back  = [cfg.outputfolder,'/BRC-AEKF', '.txt'];
        try
            writematrix(nav_back, output_file_back, 'Delimiter', ' ');
            fprintf('nav_rotscale信息已成功写入到 %s\n', output_file_back);
        catch ME
            error('错误：写入文件失败。错误信息：%s', ME.message);
        end
    end
    % close all
end
%%
path_EKF=[cfg.outputfolder,'/EKF.nav'];
path_AEKF=[cfg.outputfolder,'/AEKF.nav'];
path_BRC_AEKF=[cfg.outputfolder,'/BRC-AEKF.txt'];
path_GTS_BRC_AEKF=[cfg.outputfolder,'/GTS-BRC-AEKF.txt'];
path_GTS_BRC_AEKF_1=[cfg.outputfolder,'/GTS-BRC-1-AEKF.txt'];
myfigurestartup(7,3,'paper')
calc_radial_error(cfg.truthpath,path_EKF,path_AEKF,path_BRC_AEKF,path_GTS_BRC_AEKF,path_GTS_BRC_AEKF_1)
hold on
plot(xlim,[400,400],'DisplayName','400m界限')
ylim([0 600])
%% 残差及自适应因子
% adap_factor_visualize(z,z_back)
% adap_factor_visualize_gui(z, z_back)

%% 估计误差对比
if feedback==0
    plot_xk(xkpath,navpath,cfg.truthpath)
end

% pureins='旋转收缩方案/output_simu/pureins.nav';
% path1='旋转收缩方案/output_simu/RANGE1-001.nav';
% path2='旋转收缩方案/output_simu/RANGE1-005.nav';
% path3='旋转收缩方案/output_simu/RANGE1-0003.nav';
% path4='旋转收缩方案/output_simu/RANGE1-0005.nav';
% figure
% calc_radial_error(cfg.truthpath,path1,path3,path4)
% legend('0.01°/h','0.003°/h','0.005°/h')