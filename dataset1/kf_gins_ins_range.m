% clear;
% clc;
%% 定义参数和加载配置
param = Param();
cfg = ProcessConfig1_zxy();

%% 导入数据
% imudata
imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);

% range data
id=42;
cfg.userange=1;

% rangedata1 = importdata(cfg.rangefile1path);
% rangedata2 = importdata(cfg.rangefile2path);
% rangedata3 = importdata(cfg.rangefile3path);
% rangedata4 = importdata(cfg.rangefile4path);
% rangedata5 = importdata(cfg.rangefile5path);
% 
% rangedata1 = rangedata1(id:id:end,:);
% rangedata2 = rangedata2(id:id:end,:);
% rangedata3 = rangedata3(id:id:end,:);
% rangedata = zeros(size(rangedata1));
% 
% rangedata(1:3:end,:)=rangedata1(1:3:end,:);
% rangedata(2:3:end,:)=rangedata2(2:3:end,:);
% rangedata(3:3:end,:)=rangedata3(3:3:end,:);


rangedata = importdata(cfg.rangefile1path);
rangedata = rangedata(id:id:end,:);

rangestarttime = rangedata(1, 1);
rangeendtime = rangedata(end, 1);
% height data

heightdata = importdata(cfg.depthfilepath);
heightdata = heightdata(id:id:end,:);
heightstarttime = heightdata(1, 1);
heightendtime = heightdata(end, 1);

heightdata1 = importdata(cfg.depthfile1path);
height1starttime = heightdata1(1, 1);
height1endtime = heightdata1(end, 1);
%% 设置文件保存路径
navpath = [cfg.outputfolder, '/NavResult'];
if cfg.userange
    navpath = [navpath, '_RANGE'];
    disp("use RANGE data!");
end

% navpath = [navpath, '_pureINS'];
navpath = [navpath, '.nav'];
navfp = fopen(navpath, 'wt');

imuerrpath = [cfg.outputfolder, '/ImuError.txt'];
imuerrfp = fopen(imuerrpath, 'wt');

stdpath = [cfg.outputfolder, '/NavSTD.txt'];
stdfp = fopen(stdpath, 'wt');

xkpath = [cfg.outputfolder, '/xk_range.txt'];
xkfp = fopen(xkpath, 'wt');
%% 获取处理时间，调整时间
if imustarttime > rangestarttime
    starttime = imustarttime;
else
    starttime = rangestarttime;
end
if imuendtime > rangeendtime
    endtime = rangeendtime;
else
    endtime = imuendtime;
end
if cfg.starttime < starttime
    cfg.starttime = starttime;
end
if cfg.endtime > endtime
    cfg.endtime = endtime;
end

imudata = imudata(imudata(:,1) >= cfg.starttime, :);
imudata = imudata(imudata(:,1) <= cfg.endtime, :);

rangedata = rangedata(rangedata(:, 1) >= cfg.starttime, :);
rangedata = rangedata(rangedata(:, 1) <= cfg.endtime, :);
heightdata = heightdata(heightdata(:, 1) >= cfg.starttime, :);
heightdata = heightdata(heightdata(:, 1) <= cfg.endtime, :);

%% for debug
disp("Start RANGE/INS Processing!");
lastprecent = 0;

%% initialization 
[kf, navstate] = myInitialize(cfg);
laststate = navstate;

% data index preprocess
lastimu = imudata(1, :)';
thisimu = imudata(1, :)';
imudt = thisimu(1, 1) - lastimu(1, 1);

rangeindex = 1;
while rangedata(rangeindex, 1) < thisimu(1, 1)
    rangeindex = rangeindex + 1;
end
% gnssindex = 1;
% while gnssdata(gnssindex, 1) < thisimu(1, 1)
%     gnssindex = gnssindex + 1;
% end

% if cfg.useodonhc
%     odoindex = 1;
%     while ododata(odoindex, 1) < thisimu(1, 1) && odoindex < size(ododata, 1)
%         odoindex = odoindex + 1;
%     end
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MAIN PROCEDD PROCEDURE!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
kk=1;
for imuindex = 2:size(imudata, 1)-1

    % 设置上一个状态的值
    lastimu = thisimu;
    laststate = navstate;
    thisimu = imudata(imuindex, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);
    % 补偿IMU误差
    % thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * navstate.gyrbias)./(ones(3, 1) + navstate.gyrscale);
    % thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * navstate.accbias)./(ones(3, 1) + navstate.accscale);

    % 调整索引
    while (rangeindex <= size(rangedata, 1) && rangedata(rangeindex, 1) < lastimu(1, 1))
        rangeindex = rangeindex + 1;
    end    
    if (rangeindex > size(rangedata, 1))
        disp('Range file END!');
        break;
    end
     %% determine whether range update is required
    % if lastimu(1, 1) == rangedata(rangeindex, 1)
    %     % do gnss update for the current state
    %     thisrange = rangedata(rangeindex, :);
    %     thisheight = heightdata(rangeindex, :);
    %     kf = myRangeUpdate(navstate, thisrange, thisheight, kf);
    %     [kf, navstate] = myErrorFeedback(kf, navstate);
    %     rangeindex = rangeindex + 1;
    %     laststate = navstate;
    % 
    %     % do propagation for current imu data
    %     imudt = thisimu(1, 1) - lastimu(1, 1);
    %     navstate = InsMech(laststate, lastimu, thisimu);
    %     kf = myInsPropagate(navstate, thisimu, imudt, kf, cfg.corrtime);
    % elseif (lastimu(1, 1) < rangedata(rangeindex, 1) && thisimu(1, 1) > rangedata(rangeindex, 1))
    %     % ineterpolate imu to gnss time
    %     [firstimu, secondimu] = interpolate(lastimu, thisimu, rangedata(rangeindex, 1));
    % 
    %     % do propagation for first imu
    %     imudt = firstimu(1, 1) - lastimu(1, 1);
    %     navstate = InsMech(laststate, lastimu, firstimu);
    %     % navstate.vel(3) = 0;% 天向速度置零
    %     kf = myInsPropagate(navstate, firstimu, imudt, kf, cfg.corrtime);
    % 
    %     % do range update
    %     thisrange = rangedata(rangeindex, :);
    %     thisheight = heightdata(rangeindex, :);
    %     thispos(kk,:) = navstate.pos;
    %     thisvel(kk,:) = navstate.vel;
    %     thisphi(kk,:) = navstate.att(3);
    %     [~,thisR] = caldot2dot(navstate.pos',thisrange(4:6));
    %     kf = myRangeUpdate(navstate, thisrange, thisheight, kf);
    % 
    %     % 根据角度相对位置调整反馈增益
    %     % [~, navstate1] = myErrorFeedback_range(kf, navstate, 1);
    %     % calphi(kk,:)=calangel(thisrange(4:6)', navstate1.pos',navstate1.att(3));
    %     % b(kk,:)=1-calphi(kk,:)/90;
    %     % [kf, navstate] = myErrorFeedback_range(kf, navstate, b(kk,:));
    % 
    %     % 首先根据0.8的反馈比计算，再以这个反馈结果计算残差的残差
    %     [~, navstate1] = myErrorFeedback_range(kf, navstate, 0.5);
    %     [~,nextRR]=caldot2dot(navstate1.pos',thisrange(4:6));
    %     realR = thisrange(3);
    %     resresPRO(kk,:)=abs(nextRR-realR)-abs(thisR-realR);
    % 
    %     if resresPRO(kk,:)>0
    %         [kf, navstate] = myErrorFeedback_range(kf, navstate, 0.5);
    %     else
    %         [kf, navstate] = myErrorFeedback(kf, navstate);
    %     end
    % 
    %     % [kf, navstate] = myErrorFeedback(kf, navstate);
    % 
    %     nextpos(kk,:) = navstate.pos;
    %     nextvel(kk,:) = navstate.vel;
    %     nextphi(kk,:) = navstate.att(3);
    %     [~,neaxtR] = caldot2dot(navstate.pos',thisrange(4:6));
    %     realR = thisrange(3);
    %     res(kk,:)=[navstate.time,neaxtR-realR,thisR-realR];% 记录修正前后的距离残差
    % 
    %     kk=kk+1;
    %     rangeindex = rangeindex + 1;
    %     laststate = navstate;
    %     lastimu = firstimu;
    % 
    %     % do propagation for second imu
    %     imudt = secondimu(1, 1) - lastimu(1, 1);
    %     navstate = InsMech(laststate, lastimu, secondimu);
    %     % navstate.vel(3) = 0;% 天向速度置零
    %     kf = myInsPropagate(navstate, secondimu, imudt, kf, cfg.corrtime);
    %     % navstate.pos(3)=heightdata(rangeindex,2); % 使用深度计直接修正
    % else
    % % only do propagation
    %     % INS mechanization
    %     navstate = InsMech(laststate, lastimu, thisimu);
    %     % navstate.vel(3) = 0;% 天向速度置零
    %     % error propagation
    %     kf = myInsPropagate(navstate, thisimu, imudt, kf, cfg.corrtime);
    % end
    
    % 区分是否需要距离更新
    if lastimu(1, 1) == rangedata(rangeindex, 1)
        % 先对当前状态进行测量更新
        thisRange = rangedata(rangeindex, :);
        depthdata = [heightdata(rangeindex,1),heightdata(rangeindex,2)];
        kf = myRangeUpdate(navstate, thisRange, depthdata, kf);
        [kf, navstate] = myErrorFeedback_15state(kf, navstate);
        rangeindex = rangeindex + 1;
        laststate = navstate;

        % 惯导解算
        imudt = thisimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, thisimu);
        kf = myInsPropagate(navstate, thisimu, imudt, kf, cfg.corrtime);
    elseif (lastimu(1, 1) < rangedata(rangeindex, 1) && thisimu(1, 1) > rangedata(rangeindex, 1))
        % 将imu插值到测量值的时间
        [firstimu, secondimu] = interpolate(lastimu, thisimu, rangedata(rangeindex, 1));

        % 前一个imu到测量值时间的惯导更新
        imudt = firstimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, firstimu);
        kf = myInsPropagate(navstate, firstimu, imudt, kf, cfg.corrtime);

        % 测量更新
        thisRange = rangedata(rangeindex, :);
        depthdata = [heightdata(rangeindex,1),heightdata(rangeindex,2)];
        kf = myRangeUpdate(navstate, thisRange, depthdata, kf);
        [kf, navstate] = myErrorFeedback(kf, navstate);
        rangeindex = rangeindex + 1;
        laststate = navstate;
        lastimu = firstimu;

        % 测量时间到下一个惯导值的更新
        imudt = secondimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, secondimu);
        kf = myInsPropagate(navstate, secondimu, imudt, kf, cfg.corrtime);
    else
        % 仅做惯导结算
        % INS 编排
        navstate = InsMech(laststate, lastimu, thisimu);
        % 尝试做天向速度和位置约束
        % navstate.vel(3)=(-heightdata(imuindex,1)-(-heightdata(imuindex-1,1)))/imudt;
        % navstate.pos(3)=heightdata1(imuindex,2);
        % navstate.pos(3)=20.9;
        % error propagation
        % if mod(imuindex,20)==0
        %     navstate.pos(3)=heightdata(floor(imuindex/20),1);
        % end
        % 状态传播
        kf = myInsPropagate(navstate, thisimu, imudt, kf, cfg.corrtime);
    end

    %% save data
    % write navresult to file
    nav = zeros(11, 1);
    nav(2, 1) = navstate.time;
    nav(3:5, 1) = [navstate.pos(1) * param.R2D; navstate.pos(2) * param.R2D; navstate.pos(3)];
    nav(6:8, 1) = navstate.vel;
    nav(9:11, 1) = navstate.att * param.R2D;
    fprintf(navfp, '%2d %12.9f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', nav);
    % % 保存估计的状态值
    % xk = zeros(16, 1);
    % xk(1) = navstate.time;
    % xk(2:16) = kf.x(1:15);
    % fprintf(xkfp, '%12.6f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f %12.8f\n', xk);
    % % 
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

    % print processing information
    if (imuindex / size(imudata, 1) - lastprecent > 0.05) 
        disp("processing " + num2str(floor(imuindex * 100 / size(imudata, 1))) + " %!");
        lastprecent = imuindex / size(imudata, 1);
    end
end

% close file
% fclose(imuerrfp);
fclose(navfp);
% fclose(stdfp);
fclose(xkfp);
disp("range/INS Integration Processing Finished!");
%%
truthpath=cfg.truthpath;
%%
% plot_xk(xkpath,navpath,truthpath)
% 
%%
% plot_imuerror
%%
calc_error(navpath,cfg.truthpath)
%%  观察校正前后的残差
resres(:,2)=abs(res(:,2))-abs(res(:,3));
resres(:,1)=res(:,1);
in=find(resres(:,2)>0);% 校正后残差增加的点

figure,plot(resres(:,1),resres(:,2))
legend('残差的残差（next-this）')
% 找出校正点的真实参考位置
ref=importdata(truthpath);
for i=1:length(resres)
    index=find(ref(:,2)>resres(i,1)&ref(:,2)<resres(i,1)+0.005)-1;
    point1=ref(index,3:5);
    point1(1:2)=point1(1:2)/180*pi;
    ref11(i,:)=[ref(index,2),point1];
end
% figure
% plot(ref11(:,3),ref11(:,2),".")
%% 起始点，计算dxyz的参考点
start=ref(1,3:5);
startrrm=start;
startrrm(1:2)=start(1:2)/180*pi;

% figure
% plot(startrrm(2),startrrm(1),".")
% hold on
% plot(ref(1:120000,4)/180*pi,ref(1:120000,3)/180*pi)
% for i=1:5
% hold on
% plot(thispos(i,2),thispos(i,1),"r*")
% hold on
% plot(nextpos(i,2),nextpos(i,1),"g*")
% hold on
% plot(ref11(i,3),ref11(i,2),"m.")
% end
%% 对真实轨迹、解算轨迹和校正点前后进行绘图
glvs
% 真实轨迹点
ref11rrm=ref11;
% ref11rrm(:,2:3)=ref11rrm(:,2:3)*pi/180;
% 计算轨迹
nav=importdata(navpath);
navrrm=nav(:,3:5);
navrrm(:,1:2)=navrrm(:,1:2)*pi/180;
% 真实轨迹
refrrm=ref(:,3:5);
refrrm(:,1:2)=refrrm(:,1:2)*pi/180;

dxyzref=pos2dxyz(refrrm,startrrm');% 真实轨迹
dxyznav=pos2dxyz(navrrm,startrrm');% 解算轨迹
dxyzthis=pos2dxyz(thispos,startrrm');% 校正前的点
dxyznext=pos2dxyz(nextpos,startrrm');% 校正后的点
dxyzref11=pos2dxyz(ref11rrm(:,2:4),startrrm');% 真实轨迹点
dxyz=pos2dxyz(rangedata(1,4:6),startrrm');% 信标位置
figure
plot(0,0,".")
hold on
plot(dxyzref(:,1),dxyzref(:,2))
hold on
plot(dxyznav(:,1),dxyznav(:,2))
hold on
plot(dxyz(1),dxyz(2),'*')
for i=1:7
hold on
plot(dxyzthis(i,1),dxyzthis(i,2),"r*")
hold on
plot(dxyznext(i,1),dxyznext(i,2),"g*")
hold on
plot(dxyzref11(i,1),dxyzref11(i,2),"m.")
end

%% 计算速度向量和相对位置向量的角度
ddxxyy=dxyznext-dxyz;
for i=1:length(ddxxyy)
    phi(i)=acos(ddxxyy(i,1:2).*nextvel(i,1:2)/(norm(ddxxyy(i,1:2))*nextvel(i,1:2)));
    phi(i)=phi(i)*180/pi;
end
%% 计算位置向量和航向角的角度
ddxxyy=dxyznext-dxyz;
vector_angle = mod(atan2d(ddxxyy(:,1), ddxxyy(:,2)),360);
heading=r2d(thisphi);
heading(heading<0)=heading(heading<0)+360;
angle_diff = abs(vector_angle-heading);
angle_diff(angle_diff>90 & angle_diff<180) = 180-angle_diff(angle_diff>90 & angle_diff<180);
angle_diff(angle_diff>180 & angle_diff<270) = angle_diff(angle_diff>180 & angle_diff<270)-180;
angle_diff(angle_diff>270 & angle_diff<360) = 360-angle_diff(angle_diff>270 & angle_diff<360);
%% 将其与误差对比
RadialERfind(navpath,truthpath)
subplot 121,
hold on,plot(ref11(:,1),r2d(thisphi),'*')
subplot 122
hold on
plot(ref11(:,1),abs(phi'-90))
hold on
plot(ref11(:,1),r2d(thisphi))
hold on
plot(ref11(:,1),angle_diff)
legend('位置误差增量','速度计算出的夹角','航向角','航向角计算出的夹角')
% calc_error("dataset1/output/NavResult_pureINS.nav",cfg.truthpath)
%%
calc_error("dataset1/output/NavResult_full_pureINS.nav",cfg.truthpath)
%%
calc_error("dataset1/output/NavResult_depth_full_pureINS.nav",cfg.truthpath)
%%
plot_result(navpath)
%%
plot_result("dataset1/truth.nav")
%%
plot_cmp(navpath,cfg.truthpath)
%%
load('D:\GitHub\PSINS\psins2401\mytest\06_anlysis\WHU\Leador-A15\data_Leador-A15.mat')
avp_Ref=avpref(2:120001,[1:9,11]);
trjsee(avp_Ref,'2d',avp_kfgins)
plotTrajectoriesComparison(avp_Ref, avp_kfgins, 0.005);
%% 辅助函数
function angle_diff=calangel(posbea, pospred,thisphi)
dxy=pos2dxyz(pospred,posbea);
vector_angle = mod(atan2d(dxy(1), dxy(2)),360);
heading=r2d(thisphi);
heading(heading<0)=heading(heading<0)+360;
angle_diff = abs(vector_angle-heading);
angle_diff(angle_diff>90 & angle_diff<180) = 180-angle_diff(angle_diff>90 & angle_diff<180);
angle_diff(angle_diff>180 & angle_diff<270) = angle_diff(angle_diff>180 & angle_diff<270)-180;
angle_diff(angle_diff>270 & angle_diff<360) = 360-angle_diff(angle_diff>270 & angle_diff<360);
end