clear;
% clc;
% 支持gnss组合生成参考轨迹
%% define parameters and importdata process config
param = Param();
cfg = ProcessConfig_exper();
%% importdata data
% imudata
imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);

% gnss data
truthdata = importdata(cfg.truthreffilepath);
truthdata(:, 2:3) = truthdata(:, 2:3) * param.D2R;
if (size(truthdata, 2) < 13)
    cfg.usegnssvel = false;
end

% height data
heightdata = importdata(cfg.depthfilepath);
heightstarttime = heightdata(1, 1);
heightendtime = heightdata(end, 1);
%% save result
navpath = [cfg.outputfolder, '/NavResult-truth'];
navpath = [navpath, '.nav'];
navfp = fopen(navpath, 'wt');

%% get process time
% start time and end time
if cfg.starttime < imustarttime
    cfg.starttime = imustarttime;
end
if cfg.endtime > imuendtime
    cfg.endtime = imuendtime;
end

% data in process interval
imudata = imudata(imudata(:,1) >= cfg.starttime, :);
imudata = imudata(imudata(:,1) <= cfg.endtime, :);
truthdata = truthdata(truthdata(:, 1) >= cfg.starttime, :);
truthdata = truthdata(truthdata(:, 1) <= cfg.endtime, :);
heightdata = heightdata(heightdata(:, 1) >= cfg.starttime, :);
heightdata = heightdata(heightdata(:, 1) <= cfg.endtime, :);
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

truthindex = 1;
while truthdata(truthindex, 1) < thisimu(1, 1)
    truthindex = truthindex + 1;
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
    thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * navstate.gyrbias)./(ones(3, 1) + navstate.gyrscale);
    thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * navstate.accbias)./(ones(3, 1) + navstate.accscale);

    %% adjust GNSS index
    while (truthindex <= size(truthdata, 1) && truthdata(truthindex, 1) < lastimu(1, 1))
        truthindex = truthindex + 1;
    end
    % check whether gnss data is valid
    if (truthindex > size(truthdata, 1))
        disp('GNSS file END!');
        break;
    end
    %% determine whether gnss update is required
    if lastimu(1, 1) == truthdata(truthindex, 1)
        % do gnss update for the current state
        thisgnss = truthdata(truthindex, :)';
        kf = myGNSSUpdate_15state(navstate, thisgnss, kf);
        % kf = myGNSSUpdate(navstate, thisgnss, kf, cfg.antlever);
        [kf, navstate] = myErrorFeedback_15state(kf, navstate);
        truthindex = truthindex + 1;
        laststate = navstate;

        % do propagation for current imu data
        imudt = thisimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, thisimu);
        % navstate.pos(3) = heightdata(imuindex,2);
        kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
    elseif (lastimu(1, 1) < truthdata(truthindex, 1) && thisimu(1, 1) > truthdata(truthindex, 1))
        % ineterpolate imu to gnss time
        [firstimu, secondimu] = interpolate(lastimu, thisimu, truthdata(truthindex, 1));

        % do propagation for first imu
        imudt = firstimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, firstimu);
        navstate.pos(3) = heightdata(imuindex,2);
        kf = myInsPropagate_15state(navstate, firstimu, imudt, kf);

        % do gnss update
        thisgnss = truthdata(truthindex, :)';
        kf = myGNSSUpdate_15state(navstate, thisgnss, kf);
        % kf = myGNSSUpdate(navstate, thisgnss, kf, cfg.antlever);
        [kf, navstate] = myErrorFeedback_15state(kf, navstate);
        truthindex = truthindex + 1;
        laststate = navstate;
        lastimu = firstimu;

        % do propagation for second imu
        imudt = secondimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, secondimu);
        navstate.pos(3) = heightdata(imuindex,2);
        kf = myInsPropagate_15state(navstate, secondimu, imudt, kf);
    else
        %% only do propagation
        % INS mechanization
        navstate = InsMech(laststate, lastimu, thisimu);
        navstate.pos(3) = heightdata(imuindex,2);
        % error propagation
        kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
    end
    %% save data
    % xkk(imuindex-1,:)=[navstate.time;kf.x];
    % write navresult to file
    nav = zeros(11, 1);
    nav(2, 1) = navstate.time;
    nav(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
    nav(6:8, 1) = navstate.vel;
    nav(9:11, 1) = navstate.att * param.R2D;
    fprintf(navfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);
    % print processing information
    % if (imuindex / size(imudata, 1) - lastprecent > 0.20) 
    %     disp("processing " + num2str(floor(imuindex * 100 / size(imudata, 1))) + " %!");
    %     lastprecent = imuindex / size(imudata, 1);
    % end
end
% close file
fclose(navfp);
disp("gnss/INS Integration Processing Finished!");
%%
calc_error(cfg.truthpath,'惯导实验数据/output/NavResult-pureins.nav')
%%
calc_error(cfg.truthpath,'惯导实验数据/input/pva_430.txt')
%%
calc_error(cfg.truthpath,'惯导实验数据/input/pva_830.txt')
% %%
% calc_error(cfg.truthpath,'惯导实验数据/input/pva_RS.txt')
%%
leg={'参考','nav','range','puerins'};
plot_trj(cfg.truthpath,navpath)
% plot_result(navpath)                  
% plot_result("dataset1/truth.nav")
