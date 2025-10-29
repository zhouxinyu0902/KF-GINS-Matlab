clear;
% clc;
% 生成多个次序的距离辅助导航
seq=[1,2,3;
    1,3,2;
    3,2,1;
    3,1,2;
    2,3,1;
    2,1,3];
for nnn=1:6

    %% define parameters and importdata process config
    param = Param();
    cfg = ProcessConfig_exper();
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
    id=420;
    for i=1:3
        range{i} = range{i}(id:id:end,:);
    end
    rangedata = zeros(size(range{1}));
    for i=1:3
        rangedata(i:3:end,:)=range{seq(nnn,i)}(i:3:end,:);
    end
    % id=60;
    % rangedata1 = rangedata1(id:id:end,:);
    % rangedata2 = rangedata2(id:id:end,:);
    % rangedata3 = rangedata3(id:id:end,:);
    % rangedata = zeros(size(rangedata1));
    % rangedata(1:2:end,:)=rangedata1(1:2:end,:);
    % rangedata(2:2:end,:)=rangedata2(2:2:end,:);

    % id=1;
    % rangedata = rangedata2;

    rangestarttime = rangedata(1, 1);
    rangeendtime = rangedata(end, 1);

    % gnss data
    gnssdata = importdata(cfg.gnssfilepath);
    gnssdata(:, 2:3) = gnssdata(:, 2:3) * param.D2R;
    if (size(gnssdata, 2) < 13)
        cfg.usegnssvel = false;
    end
    gnssstarttime = gnssdata(1, 1);
    gnssendtime = gnssdata(end, 1);

    %
    % height data

    % heightdata = importdata(cfg.depthfilepath);
    truth = importdata(cfg.truthpath);
    height = truth(:,[2,5]);
    heightdata = height(id*100:id*100:end,:);
    heightstarttime = heightdata(1, 1);
    heightendtime = heightdata(end, 1);
    %%
    % myfigurestartup(5,5,'prese')
    % plot(truth(:,4),truth(:,3))
    % hold on
    % plot(truth(1,4),truth(1,3),'.')
    % for i=1:3
    %     plot(rangedata(i,5)/pi*180,rangedata(i,4)/pi*180,'*')
    % end
    % legend('轨迹','起点','信标1','信标2','信标3')
    %% 设置文件保存路径
    navpath = [cfg.outputfolder, '/NavResult'];
    if cfg.userange
        navpath = [navpath, '-RANGE-',num2str(nnn)];
        disp("use RANGE data!");
    end

    navpath = [navpath, '.nav'];
    navfp = fopen(navpath, 'wt');
    %
    % imuerrpath = [cfg.outputfolder, '/ImuError.txt'];
    % imuerrfp = fopen(imuerrpath, 'wt');
    %
    % stdpath = [cfg.outputfolder, '/NavSTD.txt'];
    % stdfp = fopen(stdpath, 'wt');
    %
    % xkpath = [cfg.outputfolder, '/xk_range.txt'];
    % xkfp = fopen(xkpath, 'wt');
    %% 获取处理时间，调整时间
    % start time and end time
    % if imustarttime > gnssstarttime
    %     starttime = imustarttime;
    % else
    %     starttime = gnssstarttime;
    % end
    % if imuendtime > gnssendtime
    %     endtime = gnssendtime;
    % else
    %     endtime = imuendtime;
    % end
    if cfg.starttime < imustarttime
        cfg.starttime = imustarttime;
    end
    if cfg.endtime > imuendtime
        cfg.endtime = imuendtime;
    end

    % data in process interval
    imudata = imudata(imudata(:,1) >= cfg.starttime, :);
    imudata = imudata(imudata(:,1) <= cfg.endtime, :);
    gnssdata = gnssdata(gnssdata(:, 1) >= cfg.starttime, :);
    gnssdata = gnssdata(gnssdata(:, 1) <= cfg.endtime, :);
    rangedata = rangedata(rangedata(:, 1) >= cfg.starttime, :);
    rangedata = rangedata(rangedata(:, 1) <= cfg.endtime, :);
    heightdata = heightdata(heightdata(:, 1) >= cfg.starttime, :);
    heightdata = heightdata(heightdata(:, 1) <= cfg.endtime, :);
    %% for debug
    disp("Start GNSS/RANGE Processing!");
    lastprecent = 0;
    %% initialization
    [kf, navstate] = myInitialize_15state(cfg);
    laststate = navstate;

    % data index preprocess
    lastimu = imudata(1, :)';
    thisimu = imudata(1, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);

    rangeindex = 1;
    while rangedata(rangeindex, 1) < thisimu(1, 1)
        rangeindex = rangeindex + 1;
    end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% MAIN PROCEDD PROCEDURE!
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for imuindex = 2:size(imudata, 1)-1

        %% set value of last state
        lastimu = thisimu;
        laststate = navstate;
        thisimu = imudata(imuindex, :)';
        imudt = thisimu(1, 1) - lastimu(1, 1);


        %% compensate IMU error
        % thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * navstate.gyrbias)./(ones(3, 1) + navstate.gyrscale);
        % thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * navstate.accbias)./(ones(3, 1) + navstate.accscale);
        thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * navstate.gyrbias);
        thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * navstate.accbias);
        %% adjust range index
        while (rangeindex <= size(rangedata, 1) && rangedata(rangeindex, 1) < lastimu(1, 1))
            rangeindex = rangeindex + 1;
        end
        % check whether gnss data is valid
        if (rangeindex > size(rangedata, 1))
            disp('range file END!');
            break;
        end
        %% determine whether gnss update is required
        if lastimu(1, 1) == rangedata(rangeindex, 1)
            % do gnss update for the current state
            % thisgnss = gnssdata(gnssindex, :)';
            % kf = myGNSSUpdate_15state(navstate, thisgnss, kf);
            % % kf = myGNSSUpdate(navstate, thisgnss, kf, cfg.antlever);
            Rangedata = rangedata(rangeindex,:);
            depthdata = heightdata(rangeindex,:);
            kf = myRangeUpdate(navstate, Rangedata, depthdata, kf);
            [kf, navstate] = myErrorFeedback_range(kf, navstate);
            % [kf, navstate] = myErrorFeedback_range_posonly(kf, navstate);

            rangeindex = rangeindex + 1;
            laststate = navstate;

            % do propagation for current imu data
            imudt = thisimu(1, 1) - lastimu(1, 1);
            navstate = InsMech(laststate, lastimu, thisimu);
            kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
        elseif (lastimu(1, 1) < rangedata(rangeindex, 1) && thisimu(1, 1) > rangedata(rangeindex, 1))
            % ineterpolate imu to gnss time
            [firstimu, secondimu] = interpolate(lastimu, thisimu, rangedata(rangeindex, 1));

            % do propagation for first imu
            imudt = firstimu(1, 1) - lastimu(1, 1);
            navstate = InsMech(laststate, lastimu, firstimu);
            kf = myInsPropagate_15state(navstate, firstimu, imudt, kf);

            % do gnss update
            % thisgnss = gnssdata(gnssindex, :)';
            % kf = myGNSSUpdate_15state(navstate, thisgnss, kf);
            % % kf = myGNSSUpdate(navstate, thisgnss, kf, cfg.antlever);
            Rangedata = rangedata(rangeindex,:);
            depthdata = heightdata(rangeindex,:);
            kf = myRangeUpdate(navstate, Rangedata, depthdata, kf);
            [kf, navstate] = myErrorFeedback_range(kf, navstate);
            % [kf, navstate] = myErrorFeedback_range_posonly(kf, navstate);
            rangeindex = rangeindex + 1;
            laststate = navstate;
            lastimu = firstimu;

            % do propagation for second imu
            imudt = secondimu(1, 1) - lastimu(1, 1);
            navstate = InsMech(laststate, lastimu, secondimu);
            kf = myInsPropagate_15state(navstate, secondimu, imudt, kf);
        else
            %% only do propagation
            % INS mechanization
            navstate = InsMech(laststate, lastimu, thisimu);
            navstate.pos(3) = height(imuindex,2) + randn*0.02;
            navstate.vel(3) = -(height(imuindex,2)-height(imuindex-1,2))/imudt;
            % error propagation
            kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
        end


        %% determine whether range update is required
        % if lastimu(1, 1) == rangedata(rangeindex, 1)
        %     % do gnss update for the current state
        %     thisrange = rangedata(rangeindex, :);
        %     thisheight = heightdata(rangeindex, :);
        %     kf = myRangeUpdate(navstate, thisrange, thisheight, kf);
        %     [kf, navstate] = myErrorFeedback(kf, navstate);
        %     gnssindex = gnssindex + 1;
        %     laststate = navstate;
        %
        %     % do propagation for current imu data
        %     imudt = thisimu(1, 1) - lastimu(1, 1);
        %     navstate = InsMech(laststate, lastimu, thisimu);
        %     kf = myInsPropagate(navstate, thisimu, imudt, kf);
        % elseif (lastimu(1, 1) < rangedata(rangeindex, 1) && thisimu(1, 1) > rangedata(rangeindex, 1))
        %     % ineterpolate imu to gnss time
        %     [firstimu, secondimu] = interpolate(lastimu, thisimu, rangedata(rangeindex, 1));
        %
        %     % do propagation for first imu
        %     imudt = firstimu(1, 1) - lastimu(1, 1);
        %     navstate = InsMech(laststate, lastimu, firstimu);
        %     kf = myInsPropagate(navstate, firstimu, imudt, kf);
        %
        %     % do gnss update
        %     thisrange = rangedata(rangeindex, :);
        %     thisheight = heightdata(rangeindex, :);
        %     kf = myRangeUpdate(navstate, thisrange, thisheight, kf);
        %     [kf, navstate] = myErrorFeedback(kf, navstate);
        %     rangeindex = rangeindex + 1;
        %     laststate = navstate;
        %     lastimu = firstimu;
        %
        %     % do propagation for second imu
        %     imudt = secondimu(1, 1) - lastimu(1, 1);
        %     navstate = InsMech(laststate, lastimu, secondimu);
        %     kf = myInsPropagate(navstate, secondimu, imudt, kf);
        % else
        % % only do propagation
        %     % INS mechanization
        %     navstate = InsMech(laststate, lastimu, thisimu);
        %     % error propagation
        %     kf = myInsPropagate(navstate, thisimu, imudt, kf);
        % end

        % avp_kfgins(imuindex-1,1:3)=[navstate.att(2),navstate.att(1),2*pi-navstate.att(3)];
        % avp_kfgins(imuindex-1,4:6)=[navstate.vel(2),navstate.vel(1),-navstate.vel(3)];
        % avp_kfgins(imuindex-1,7:9)=[navstate.pos(1),navstate.pos(2),navstate.pos(3)];
        % avp_kfgins(imuindex-1,10)=navstate.time;

        % if cfg.useodonhc
        %     %% update odo index
        %     while ododata(odoindex, 1) < thisimu(1, 1) && odoindex < size(ododata, 1)
        %         odoindex = odoindex + 1;
        %     end
        %
        %     %% odonhc udpate
        %     if (thisimu(1, 1) >= odoupdatetime)
        %         startindex = odoindex - round(EPOCH_TO_GETVEL / 2);
        %         endindex = odoindex + round(EPOCH_TO_GETVEL / 2);
        %         if (startindex < 1)
        %             startindex = 1;
        %         end
        %         if (endindex > size(ododata, 1))
        %             endindex = size(ododata, 1);
        %         end
        %
        %         % get odovel and update
        %         [odovel, valid] = GetOdoVel(ododata(startindex:endindex, :), thisimu(1, 1));
        %         if valid
        %             odonhc_vel = [odovel; 0; 0];
        %             kf = ODONHCUpdate(navstate, odonhc_vel, kf, cfg, thisimu, imudt);
        %             [kf, navstate] = ErrorFeedback(kf, navstate);
        %         end
        %         odoupdatetime = odoupdatetime + 1 / cfg.odoupdaterate;
        %     end
        % end


        %% save data
        % xkk(imuindex-1,:)=[navstate.time;kf.x];
        % write navresult to file
        nav = zeros(11, 1);
        nav(2, 1) = navstate.time;
        nav(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
        nav(6:8, 1) = navstate.vel;
        nav(9:11, 1) = navstate.att * param.R2D;
        fprintf(navfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);
        % 保存估计的状态值
        % xk = zeros(16, 1);
        % xk(1) = navstate.time;
        % xk(2:16) = kf.x(1:15);
        % fprintf(xkfp, '%12.6f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f\n', xk);
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

        %% print processing information
        % if (imuindex / size(imudata, 1) - lastprecent > 0.20)
        %     disp("processing " + num2str(floor(imuindex * 100 / size(imudata, 1))) + " %!");
        %     lastprecent = imuindex / size(imudata, 1);
        % end
    end

    % close file
    % fclose(imuerrfp);
    fclose(navfp);
    % fclose(stdfp);
    % disp("range/INS Integration Processing Finished!");

    %%
    % plot_xk(xkpath,navpath,truthpath)
    %%
    % close all
    % truthpath=cfg.truthpath;
    % calc_error("惯导实验数据/output/NavResult-pureins.nav",cfg.truthpath)
    % ylim([0 1000])
    calc_error(navpath,cfg.truthpath)
    ylim([0 1000])
end
%%
% plot_result(navpath)
% plot_result(cfg.truthpath)
%%
for i=1:6
    navpath1{i} = [cfg.outputfolder, '/NavResult','-RANGE-',num2str(i),'.nav'];
end
calc_radial_error(cfg.truthpath, '惯导实验数据/output/NavResult-pureins.nav',navpath1{:})
plot_trj(cfg.truthpath,'惯导实验数据/output/NavResult-pureins.nav',navpath1{:})
%%
% myfigurestartup(12,3,'prese')
% subplot 131
% plot(xkk(:,1),xkk(:,17))
% subplot 132
% plot(xkk(:,1),xkk(:,18))
% subplot 133
% plot(xkk(:,1),xkk(:,19))
%
% myfigurestartup(12,3,'prese')
% subplot 131
% plot(xkk(:,1),xkk(:,20))
% subplot 132
% plot(xkk(:,1),xkk(:,21))
% subplot 133
% plot(xkk(:,1),xkk(:,22))
%
% myfigurestartup(12,3,'prese')
% subplot 131
% plot(xkk(:,1),xkk(:,17))
% subplot 132
% plot(xkk(:,1),xkk(:,18))
% subplot 133
% plot(xkk(:,1),xkk(:,19))
%
% myfigurestartup(12,3,'prese')
% subplot 131
% plot(xkk(:,1),xkk(:,20))
% subplot 132
% plot(xkk(:,1),xkk(:,21))
% subplot 133
% plot(xkk(:,1),xkk(:,22))