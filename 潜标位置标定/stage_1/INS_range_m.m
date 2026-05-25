close all
clear
glvs
%% 定义参数+加载过程配置
param = Param();
cfg = Config_10state('D:\Github\KF-GINS-Matlab\潜标位置标定\data1');
%% 加载数据
% imudata
imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);
truth = importdata(cfg.truthpath);
feedback = 1;
%% 获取距离
% rangedataget('D:\Github\KF-GINS-Matlab\graduation\DR_INS\input')
% 构造距离信息
id = 420/20;
% 修正后的
% rangedata1 = importdata(cfg.rangefile1calpath);
% rangedata2 = importdata(cfg.rangefile2calpath);
% rangedata3 = importdata(cfg.rangefile3calpath);

% 没有修正前的
rangedata1 = importdata(cfg.rangefile1path);
rangedata2 = importdata(cfg.rangefile2path);
rangedata3 = importdata(cfg.rangefile3path);

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
rangedata(:,3) = rangedata(:,3) + normrnd(0,6,size(rangedata(:,3)));
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

%% 设置文件保存路径
navpath = [cfg.outputfolder, '/NavResult-pureins-height.nav'];
navfp = fopen(navpath, 'wt');

xkpath = [cfg.outputfolder, '/xk.nav'];
xkfp = fopen(xkpath, 'wt');
%% 调试
disp("Start INS Processing!");
lastprecent = 0;
%% 初始化
[kf, navstate] = myInitialize_15state(cfg);
kf.depthstd = 0.2;
laststate = navstate;
kf.rangstd = 6;
lastimu = imudata(1, :)';
thisimu = imudata(1, :)';
imudt = thisimu(1, 1) - lastimu(1, 1);

rangeindex = 1;
while rangedata(rangeindex, 1) < thisimu(1, 1)
    rangeindex = rangeindex + 1;
end
cfg.userange=1;
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
        % error propagation
        kf = myInsPropagate_15state_m(navstate, thisimu, imudt, kf);

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
disp("PureIns Integration Processing Finished!");
%%
% plot_result(navpath)
%%
% calc_radial_error(cfg.truthpath,navpath)
calc_error(navpath,cfg.truthpath);
