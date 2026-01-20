nnn=1;
seq=[1,2,3];
global rangstd
rangstd = 10;
global depstd
depstd = 0.5;
rng(1)
%% 定义参数+加载过程配置
param = Param();
cfg = ProcessConfig_exper();
%% 加载数据
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
% 构造距离信息
id = 420;
for i=1:3
    range{i} = range{i}(id:id:end,:);
end
rangedata = zeros(size(range{1}));
for i=1:3
    rangedata(i:3:end,:)=range{seq(nnn,i)}(i:3:end,:);
end
rangedata(:,3) = rangedata(:,3)+normrnd(0,rangstd,size(rangedata(:,3))) ;

rangestarttime = rangedata(1, 1);
rangeendtime = rangedata(end, 1);

% height data
truth = importdata(cfg.truthpath);
height = truth(:,[2,5]);
height(:,2) = height(:,2)+normrnd(0,depstd,size(height(:,2))) ;
heightdata = height(id*100:id*100:end,:);
heightstarttime = heightdata(1, 1);
heightendtime = heightdata(end, 1);
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
navpath = [cfg.outputfolder, '/NavResult'];
if cfg.userange
    navpath = [navpath, '-RANGE-test',num2str(nnn)];
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
feedback=1;
if feedback==0
    xkpath = [cfg.outputfolder, '/xk_range.txt'];
    xkfp = fopen(xkpath, 'wt');
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
heightdata = heightdata(heightdata(:, 1) >= cfg.starttime, :);
heightdata = heightdata(heightdata(:, 1) <= cfg.endtime, :);
%% 调试
disp("Start GNSS/RANGE Processing!");
lastprecent = 0;
% initialization
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
%% 正式滤波
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MAIN PROCEDD PROCEDURE!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for imuindex = 2:size(imudata, 1)-1

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
    % check whether gnss data is valid
    if (rangeindex > size(rangedata, 1))
        disp('range file END!');
        break;
    end
    % 4、determine whether gnss update is required
    if lastimu(1, 1) == rangedata(rangeindex, 1)
        % 测量更新
        Rangedata = rangedata(rangeindex,:);
        depthdata = height(imuindex,:);
        kf = myRangeUpdate(navstate, Rangedata, depthdata, kf);
        if feedback==1
            [kf, navstate] = myErrorFeedback_range(kf, navstate);
            % [kf, navstate] = myErrorFeedback_range_posonly(kf, navstate);
        end
        z(rangeindex,:)=kf.Z;
        rangeindex = rangeindex + 1;
        laststate = navstate;
        % 惯导推算
        imudt = thisimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, thisimu);
        kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
    elseif (lastimu(1, 1) < rangedata(rangeindex, 1) && thisimu(1, 1) > rangedata(rangeindex, 1))
        % 插值imu
        [firstimu, secondimu] = interpolate(lastimu, thisimu, rangedata(rangeindex, 1));
        % 惯导推算
        imudt = firstimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, firstimu);
        kf = myInsPropagate_15state(navstate, firstimu, imudt, kf);
        % 测量更新
        Rangedata = rangedata(rangeindex,:);
        depthdata = height(imuindex,:);
        % if abs(navstate.time-122235-28*60)<20||abs(navstate.time-122235-28*2*60)<20||abs(navstate.time-122235-28*3*60)<20
        %     tt = Rangedata(1);
        %     id1 = find(rangedata1(:,1)==tt);
        %     id2 = find(rangedata2(:,1)==tt);
        %     id3 = find(rangedata3(:,1)==tt);
        %     kf = my3RangeUpdate(navstate, rangedata1(id1,:), rangedata2(id2,:), rangedata3(id3,:), depthdata, kf);
        % else
            kf = myRangeUpdate(navstate, Rangedata, depthdata, kf);
        % end
        if feedback==1
            [kf, navstate] = myErrorFeedback_range(kf, navstate);
            % [kf, navstate] = myErrorFeedback_range_posonly(kf, navstate);
        end
        z(rangeindex,:)=kf.Z;
        rangeindex = rangeindex + 1;
        laststate = navstate;
        lastimu = firstimu;
        % do propagation for second imu
        imudt = secondimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, secondimu);
        % navstate.pos(3) = height(imuindex,2) + randn*depstd; % 天向位置约束
        kf = myInsPropagate_15state(navstate, secondimu, imudt, kf);
        % disp('2')

        
    else
        % 5、only do propagation
        % INS mechanization
        navstate = InsMech(laststate, lastimu, thisimu);
        navstate.pos(3) = height(imuindex,2);% 天向位置约束
        % error propagation
        kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
    end

    % 6、save data
    % xkk(imuindex-1,:)=[navstate.time;kf.x];
    % write navresult to file
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
ylim([0 600])
hold on
plot(420:420:420*11,abs(z(:,1)),'*')
%%
plot_result(cfg.truthpath,'full')
%% 计算径向误差
calc_radial_error(cfg.truthpath, '惯导实验数据/output/NavResult-pureins.nav',navpath)
%% 轨迹对比
plot_trj(cfg.truthpath,'惯导实验数据/output/NavResult-pureins.nav',navpath)
%% 估计误差对比
if feedback==0
    plot_xk(xkpath,navpath,cfg.truthpath)
end