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
navp = [cfg.outputfolder, '/NavResult'];
if cfg.userange
    navpath = [navp, '-RANGE',num2str(nnn),'-adap'];
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


if feedback==0
    xkpath = [cfg.outputfolder, '/xk_range.txt'];
    xkfp = fopen(xkpath, 'wt');
end

%% 调试
disp("Start GNSS/RANGE Processing!");
lastprecent = 0;
%% 用于对比几类改进方法的参数
ki=1;
indexrecord(1) = 1;
nav11=[];
nav00=[];
nav_rotate=[];
pkk00=[];
pkk11=[];
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
    if lastimu(1, 1) == rangedata(rangeindex, 1)
        % 测量更新
        
        Rangedata = rangedata(rangeindex,:);
        % Rangedata(3) = Rangedata(3)+randn*rangstd ;
        depthdata = height(imuindex,1:2);
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
        % navstate.pos(3) = height(imuindex,2); % 天向位置约束
        % navstate.vel(3) = -(height(imuindex,2)+ randn*depstd-height(imuindex-1,2)- randn*depstd)/imudt; % 天向速度约束
        kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
        disp('1')
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
        % Rangedata(3) = Rangedata(3) + randn*rangstd ;
        depthdata = [height(imuindex,1),height(imuindex,2)];
        % if abs(navstate.time-122235-28*60)<20||abs(navstate.time-122235-28*2*60)<20||abs(navstate.time-122235-28*3*60)<20
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
        % navstate.pos(3) = height(imuindex,2) + randn*depstd; % 天向位置约束
        kf = myInsPropagate_15state(navstate, secondimu, imudt, kf);
        
        %% 反向推算
        thisimu1 = imudata(imuindex-1, :)';
        lastimu1 = imudata(imuindex, :)';
        navstate_1 = navstate;
        kf1 = kf;
        ki = ki+1;
        indexrecord(ki) = imuindex;
        for ii=indexrecord(ki):-1:indexrecord(ki-1)+1
            laststate_1 = InsMechBackward(navstate_1,lastimu1,thisimu1);
            kf1 = myInsPropagate_15state(laststate_1, thisimu1, 0.01, kf1);
            laststate_1.pos(3) = height(ii,2); 
            nav11(:,ii) = laststate_1.pos;
            pkk11(:,ii) = [laststate_1.time;diag(kf1.P)];
            lastimu1 = thisimu1;
            thisimu1 = imudata(ii, :)';
            navstate_1 = laststate_1;
            % if ii==indexrecord(ki-1)+1
            %     % 反向滤波
            %     if rangeindex==2
            %         Rangedata = zeros(1,6);
            %         Rangedata(4:6)=rangedata3(1,4:6);
            %         Rangedata(3) = caldot2dot(Rangedata(4:6),pos0');
            %     else
            %         Rangedata = rangedata(rangeindex-2,:);
            %         % Rangedata(3) = Rangedata(3) + randn*rangstd ; 
            %     end
            %     depthdata = height(ii,1:2);
            %     kf1 = myRangeUpdate(laststate_1, Rangedata, depthdata, kf1);
            %     [kf1, laststate_1] = myErrorFeedback_range(kf1, laststate_1);
            %     if rangeindex>2 & abs(kf1.Z(1))-abs(z(rangeindex-2,1))>40 
            %         break;
            %     else
            %         nav11(:,ii-1) = laststate_1.pos;
            %     end
            %     z_back(ki-1,1) = kf1.Z(1);% 残差记录
            %     z_back(ki-1,2) = kf1.Znew;
            %     z_back(ki-1,3) = kf1.alpha;
            %     z_back(ki-1,4) = kf1.d_squared ;
            %     z_back(ki-1,5) = kf1.chi2_threshold ;
            %     z_back(ki-1,6) = kf1.is_anomaly ; 
            % end
        end
        
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
    
    pkk00(:,imuindex) = [navstate.time;diag(kf.P)];
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

%% 残差及自适应因子
% adap_factor_visualize(z,z_back)
% adap_factor_visualize_gui(z, z_back)
%% 误差绘图
% calc_error(navpath,cfg.truthpath)
% ylim([0 600])
% hold on
% plot(420:420:420*11,abs(z(:,1)),'*')
%% 参考的位置
pureinspath = '惯导实验数据/output/NavResult-pureins.nav';
refpath='惯导实验数据/output/NavResult-RANGE-11.nav';
forward=importdata(navpath);
nav000 = [d2r(truth(1:imuindex-1,3:4)),truth(1:imuindex-1,5)]';
nav00 = [nav000(:,1),[d2r(forward(:,3:4)),forward(:,5)]'];
nav11(:,1) = nav000(:,1);
%% 后向推算
for ki=2:12
    if ki==2
        figure('Name','后向推算');
        myfigurestartup(15,7,'paper') 
    end
    subplot(3,4,ki-1)
    plot(nav11(2,indexrecord(ki-1):indexrecord(ki)-1), ...
        nav11(1,indexrecord(ki-1):indexrecord(ki)-1))
    hold on
    plot(nav11(2,indexrecord(ki)-1),nav11(1,indexrecord(ki)-1),'*')
    % 
    plot(nav00(2,indexrecord(ki-1):indexrecord(ki)), ...
        nav00(1,indexrecord(ki-1):indexrecord(ki)))
    plot(nav00(2,indexrecord(ki-1)),nav00(1,indexrecord(ki-1)),'*')
    % 参考结果
    plot(nav000(2,indexrecord(ki-1):indexrecord(ki)), ...
        nav000(1,indexrecord(ki-1):indexrecord(ki)))
    plot(nav000(2,indexrecord(ki-1)),nav000(1,indexrecord(ki-1)),'*')
    if ki==2
        legend('backward','start','forward','start','truth','start')
    end
end
subplot(3,4,ki)
plot(nav11(2,:),nav11(1,:))
%% 后向结果
nav_backforward=importdata(navpath);
nav_backforward(1:end,3:4)=r2d(nav11(1:2,2:end)');
output_file_backforward = [navp,'-backforward', '.txt'];
try
    writematrix(nav_backforward, output_file_backforward, 'Delimiter', ' ');
    fprintf('nav_backforwad信息已成功写入到 %s\n', output_file_backforward);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
%% 查看误差
calc_radial_error(cfg.truthpath,pureinspath, navpath, output_file_backforward,...
    [navp,'-backforward+filter', '.txt'])
hold on
plot(xlim,[400,400],'DisplayName','400m界限')
%% 旋转+缩放处理
nav_rotatesum(:,1) = nav000(:,1);
figure('Name','旋转+缩放处理');
myfigurestartup(14,7,'paper')
for ki=2:12
    % nav_rotate = rotateTrajectory(nav00(:,indexrecord(ki-1)+1:indexrecord(ki)-1),...
    %     nav00(:,indexrecord(ki)));
    trajectory = nav00(:,indexrecord(ki-1)+1:indexrecord(ki)-1);
    newEndPoint = nav00(:,indexrecord(ki));
    [nav_rotate, scFa, rotaAngdeg] = rotateAndScaleTrajectory(trajectory, newEndPoint);
    
    nav_rotatesum(:,indexrecord(ki-1)+1:indexrecord(ki)) = [nav_rotate,newEndPoint];
    subplot(4,3,ki-1)
    plot(nav00(2,indexrecord(ki-1)+1:indexrecord(ki)), ...
        nav00(1,indexrecord(ki-1)+1:indexrecord(ki)))
    hold on
    plot(nav00(2,indexrecord(ki)),nav00(1,indexrecord(ki)),'*')
    plot(nav_rotate(2,1:end),nav_rotate(1,1:end))
    plot(nav000(2,indexrecord(ki-1)+1:indexrecord(ki)-1), ...
        nav000(1,indexrecord(ki-1)+1:indexrecord(ki)-1))
    if ki==2
        legend('forward','dot','rotate','truth')
    end
end
subplot(4,3,ki)
plot(nav_rotatesum(2,:),nav_rotatesum(1,:))
%% 旋转+缩放处理后向结果
nav_rotatesum1(:,1) = nav000(:,1);
figure('Name','旋转+缩放处理后向结果');
myfigurestartup(14,7,'paper')
for ki=2:12
    % nav_rotate = rotateTrajectory(nav00(:,indexrecord(ki-1)+1:indexrecord(ki)-1),...
    %     nav00(:,indexrecord(ki)));
    trajectory = nav11(:,indexrecord(ki-1)+1:indexrecord(ki)-1);
    newEndPoint = nav11(:,indexrecord(ki-1));

    trajectory = flip(trajectory,2);
    [nav_rotate, scFa, rotaAngdeg] = rotateAndScaleTrajectory(trajectory, newEndPoint);
    trajectory = flip(trajectory,2);
    nav_rotate = flip(nav_rotate,2);

    nav_rotatesum1(:,indexrecord(ki-1)+1:indexrecord(ki)) = [newEndPoint,nav_rotate];
    subplot(4,3,ki-1)
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
subplot(4,3,ki)
plot(nav_rotatesum1(2,:),nav_rotatesum1(1,:))
%% 查看协方差矩阵
% figure
% subplot 121
% plot(pkk00(1,2:end),pkk00(2,2:end))
% hold on
% plot(pkk11(1,2:end),pkk11(2,2:end))
% subplot 122
% plot(pkk00(1,2:end),pkk00(3,2:end))
% hold on
% plot(pkk11(1,2:end),pkk11(3,2:end))

%% 旋转缩放结果
nav_rotscale=importdata(navpath);
nav_rotscale(:,3:4)=r2d(nav_rotatesum(1:2,2:end)');
output_file_rotate  =[navp,'-rotate', '.txt'];
try
    writematrix(nav_rotscale, output_file_rotate, 'Delimiter', ' ');
    fprintf('nav_rotscale信息已成功写入到 %s\n', output_file_rotate);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
%% 旋转缩放后向结果
nav_rotscale1=importdata(navpath);
nav_rotscale1(:,3:4)=r2d(nav_rotatesum1(1:2,2:end)');
output_file_rotateback  =[navp,'-rotate+backforward', '.txt'];
try
    writematrix(nav_rotscale1, output_file_rotateback, 'Delimiter', ' ');
    fprintf('nav_rotscale信息已成功写入到 %s\n', output_file_rotateback);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
%% 计算多个结果径向误差，对比
calc_radial_error(cfg.truthpath, '惯导实验数据/output/NavResult-pureins.nav', ...
    '惯导实验数据/output/NavResult-RANGE-11.nav',navpath, ...
    output_file_backforward,output_file_rotate,output_file_rotateback)
hold on
ylim([0,1500])
plot(xlim,[400,400],'DisplayName','400m界限')
%% 多个轨迹对比
plot_trj(cfg.truthpath,navpath,output_file_backforward,output_file_rotate,output_file_rotateback)
%% 估计误差对比
if feedback==0
    plot_xk(xkpath,navpath,cfg.truthpath)
end