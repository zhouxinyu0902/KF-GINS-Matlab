clear;
%% define parameters and importdata process config
param = Param();
cfg = ProcessConfigPHINS();
%% importdata data
% imudata
imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);

% LBL data
LBLdata = importdata(cfg.LBLfilepath);
LBLdata(:, 1:2) = LBLdata(:, 1:2) * param.D2R;
LBLstarttime = LBLdata(1, 4);
LBLendtime = LBLdata(end, 4);

% % gnss data
% gnssdata = importdata(cfg.gnssfilepath);
% gnssdata(:, 2:3) = gnssdata(:, 2:3) * param.D2R;
% if (size(gnssdata, 2) < 13)
%     cfg.usegnssvel = false;
% end
% gnssstarttime = gnssdata(1, 1);
% gnssendtime = gnssdata(end, 1);

% % range data
% rangedata = importdata(cfg.rangefilepath);
% rangestarttime = rangedata(1, 1);
% rangeendtime = rangedata(end, 1);
% 
% height data
heightdata = importdata(cfg.depthfilepath);
heightstarttime = heightdata(1, 2);
heightendtime = heightdata(end, 2);
%% save result
navpath = [cfg.outputfolder, '/NavResult_PURE_INS'];
navpath = [navpath, '.nav'];
navfp = fopen(navpath, 'wt');

% imuerrpath = [cfg.outputfolder, '/ImuError.txt'];
% imuerrfp = fopen(imuerrpath, 'wt');
% 
% stdpath = [cfg.outputfolder, '/NavSTD.txt'];
% stdfp = fopen(stdpath, 'wt');
% 
% xkpath = [cfg.outputfolder, '/xk_gnss.txt'];
% xkfp = fopen(xkpath, 'wt');
%% get process time
% start time and end time
if imustarttime > LBLstarttime
    starttime = imustarttime;
else
    starttime = LBLstarttime;
end
if imuendtime > LBLendtime
    endtime = LBLendtime;
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
LBLdata = LBLdata(LBLdata(:,4) >= cfg.starttime, :);
LBLdata = LBLdata(LBLdata(:,4) <= cfg.endtime, :);
% gnssdata = gnssdata(gnssdata(:, 1) >= cfg.starttime, :);
% gnssdata = gnssdata(gnssdata(:, 1) <= cfg.endtime, :);
% rangedata = rangedata(rangedata(:, 1) >= cfg.starttime, :);
% rangedata = rangedata(rangedata(:, 1) <= cfg.endtime, :);
heightdata = heightdata(heightdata(:, 2) >= cfg.starttime, :);
heightdata = heightdata(heightdata(:, 2) <= cfg.endtime, :);
%% for debug
disp("Start GNSS/INS Processing!");
lastprecent = 0;
%% initialization 
[kf, navstate] = myInitialize_15state(cfg);
laststate = navstate;

% data index preprocess
lastimu = imudata(1, :)';
thisimu = imudata(1, :)';
imudt = thisimu(1, 1) - lastimu(1, 1);

% rangeindex = 1;
% while rangedata(rangeindex, 1) < thisimu(1, 1)
%     rangeindex = rangeindex + 1;
% end
% gnssindex = 1;
% while gnssdata(gnssindex, 1) < thisimu(1, 1)
%     gnssindex = gnssindex + 1;
% end
% heightindex = 1;
% while heightdata(heightindex, 2) < thisimu(1, end)
%     heightindex = heightindex + 1;
% end
LBLindex = 1;
while LBLdata(LBLindex, end) < thisimu(1, end)
    LBLindex = LBLindex + 1;
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

    
    % adjust GNSS index
    while (LBLindex <= size(LBLdata, 1) && LBLdata(LBLindex, end) < lastimu(1, 1))
        LBLindex = LBLindex + 1;
    end
    % check whether LBL data is valid
    if (LBLindex > size(LBLdata, 1))
        disp('LBL file END!');
        break;
    end

    % determine whether LBL update is required
    % if lastimu(1, 1) == LBLdata(LBLindex, end)
    %     % do LBL update for the current state
    %     thisLBL = LBLdata(LBLindex, :)';
    %     % kf = GNSSUpdate(navstate, thisLBL, kf, cfg.antlever, cfg.useLBLvel, lastimu, imudt);
    %     kf = myLBLUpdate(navstate, thisLBL, kf);
    %     [kf, navstate] = myErrorFeedback_15state(kf, navstate);
    %     LBLindex = LBLindex + 1;
    %     laststate = navstate;
    % 
    %     % do propagation for current imu data
    %     imudt = thisimu(1, 1) - lastimu(1, 1);
    %     navstate = InsMech(laststate, lastimu, thisimu);
    %     kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
    % elseif (lastimu(1, 1) < LBLdata(LBLindex, end) && thisimu(1, 1) > LBLdata(LBLindex, end))
    %     % ineterpolate imu to LBL time
    %     [firstimu, secondimu] = interpolate(lastimu, thisimu, LBLdata(LBLindex, end));
    % 
    %     % do propagation for first imu
    %     imudt = firstimu(1, 1) - lastimu(1, 1);
    %     navstate = InsMech(laststate, lastimu, firstimu);
    %     kf = myInsPropagate_15state(navstate, firstimu, imudt, kf);
    % 
    %     % do LBL update
    %     thisLBL = LBLdata(LBLindex, :)';
    %     kf = myLBLUpdate(navstate, thisLBL, kf);
    %     [kf, navstate] = myErrorFeedback_15state(kf, navstate);
    %     LBLindex = LBLindex + 1;
    %     laststate = navstate;
    %     lastimu = firstimu;
    % 
    %     % do propagation for second imu
    %     imudt = secondimu(1, 1) - lastimu(1, 1);
    %     navstate = InsMech(laststate, lastimu, secondimu);
    %     kf = myInsPropagate_15state(navstate, secondimu, imudt, kf);
    % else
        %% only do propagation
        % INS mechanization
        navstate = InsMech(laststate, lastimu, thisimu);
        % navstate.vel(3)=(-heightdata(imuindex,1)-(-heightdata(imuindex-1,1)))/imudt;
        navstate.pos(3)=-heightdata(imuindex,1);
        % error propagation
        kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
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
    % % write imu error, convert to common unit
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
    if (imuindex / size(imudata, 1) - lastprecent > 0.05) 
        disp("processing " + num2str(floor(imuindex * 100 / size(imudata, 1))) + " %!");
        lastprecent = imuindex / size(imudata, 1);
    end
end

% close file
% fclose(imuerrfp);
fclose(navfp);
% fclose(stdfp);
disp("range/INS Integration Processing Finished!");
%%
truthpath=cfg.truthpath;
%%
% plot_xk(xkpath,navpath,truthpath)
%%
calc_error(navpath,truthpath)
plot_result(navpath)
plot_result(truthpath)
% plot_result("dataset1/truth.nav")
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