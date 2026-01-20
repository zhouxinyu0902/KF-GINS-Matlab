clear;
% clc;
%% 定义参数和加载配置
param = Param();
cfg = patent_Configsimu();
global rangstd
rangstd=2;
global depstd
depstd=0.2;
nnn=1;
seq=[1,2,3];
beaconseq=1;
beaconnums=0;
%% 导入数据
for beaconnums=1
    for beaconseq=[1,2,3]
        for interval=4
            % imudata
            imudata = importdata(cfg.imufilepath);
            imustarttime = imudata(1, 1);
            imuendtime = imudata(end, 1);

            % range data
            % rangedata = importdata(cfg.rangefilepath);
            rangedata1 = importdata(cfg.rangefile1path);
            rangedata2 = importdata(cfg.rangefile2path);
            rangedata3 = importdata(cfg.rangefile3path);
            range = {rangedata1,rangedata2,rangedata3};
            % 构造
            id = interval*60*100;
            for i=1:3
                range{i} = range{i}(id:id:end,:);
            end

            rangedata = zeros(size(range{1}));
            if beaconnums==1
                rangedata = range{beaconseq};
            else
                for i=1:3
                    rangedata(i:3:end,:)=range{seq(nnn,i)}(i:3:end,:);
                end
            end
            rangedata(:,3) = rangedata(:,3)+normrnd(0,rangstd,size(rangedata(:,3)));
            rangestarttime = rangedata(1, 1);
            rangeendtime = rangedata(end, 1);
            % cfg.userange=1;
            % rangedata1 = importdata(cfg.rangefile1path);
            % rangedata2 = importdata(cfg.rangefile2path);
            % rangedata3 = importdata(cfg.rangefile3path);
            % rangedata4 = importdata(cfg.rangefile4path);
            % rangedata5 = importdata(cfg.rangefile5path);
            % id=42;
            % rangedata1 = rangedata1(id:id:end,:);
            % rangedata2 = rangedata2(id:id:end,:);
            % rangedata3 = rangedata3(id:id:end,:);
            % rangedata = zeros(size(rangedata1));
            % rangedata(1:3:end,:)=rangedata1(1:3:end,:);
            % rangedata(2:3:end,:)=rangedata2(2:3:end,:);
            % rangedata(3:3:end,:)=rangedata3(3:3:end,:);
            % rangestarttime = rangedata(1, 1);
            % rangeendtime = rangedata(end, 1);
            % height data

            % heightdata = importdata(cfg.depthfilepath);
            % heightdata = heightdata(id:id:end,:);
            % heightstarttime = heightdata(1, 1);
            % heightendtime = heightdata(end, 1);
            %% 设置文件保存路径
            % navpath = [cfg.outputfolder, '/NavResult'];
            navpath = [cfg.outputfolder,'/'];
            % if cfg.userange
            %     navpath = [navpath, '_RANGE'];
            %     disp("use RANGE data!");
            % end

            % 
            if beaconnums==1
                switch interval
                    case 1/12
                        navpath = [navpath, 'single',num2str(beaconseq),'-5s'];
                    case 0.5
                        navpath = [navpath, 'single',num2str(beaconseq),'-30s'];
                    case 1
                        navpath = [navpath, 'single',num2str(beaconseq),'-1min'];
                    case 2
                        navpath = [navpath, 'single',num2str(beaconseq),'-2min'];
                    case 4
                        navpath = [navpath, 'single',num2str(beaconseq),'-4min_6T'];
                    case 1/60
                        navpath = [navpath, 'single',num2str(beaconseq),'-1s'];
                end
            elseif beaconnums==3
                switch interval
                    case 1/60
                        navpath = [navpath, '3-1s'];
                    case 1/12
                        navpath = [navpath, '3-5s'];
                    case 0.5
                        navpath = [navpath, '3-30s'];
                    case 1
                        navpath = [navpath, '3-1min'];
                    case 2
                        navpath = [navpath, '3-2min'];
                    case 4
                        navpath = [navpath, '3-4min'];
                end
            else
                navpath = [navpath, 'pureINS'];
            end
            navpath = [navpath, '.nav'];
            navfp = fopen(navpath, 'wt');

            imuerrpath = [cfg.outputfolder, '/ImuError.txt'];
            imuerrfp = fopen(imuerrpath, 'wt');

            stdpath = [cfg.outputfolder, '/NavSTD.txt'];
            stdfp = fopen(stdpath, 'wt');

            xkpath = [cfg.outputfolder, '/xk_range.txt'];
            xkfp = fopen(xkpath, 'wt');
            %% 获取处理时间，调整时间
            % if imustarttime > rangestarttime
            %     starttime = imustarttime;
            % else
            %     starttime = rangestarttime;
            % end
            % if imuendtime > rangeendtime
            %     endtime = rangeendtime;
            % else
            %     endtime = imuendtime;
            % end
            % if cfg.starttime < starttime
            %     cfg.starttime = starttime;
            % end
            % if cfg.endtime > endtime
            %     cfg.endtime = endtime;
            % end

            imudata = imudata(imudata(:,1) >= cfg.starttime, :);
            imudata = imudata(imudata(:,1) <= cfg.endtime, :);

            rangedata = rangedata(rangedata(:, 1) >= cfg.starttime, :);
            rangedata = rangedata(rangedata(:, 1) <= cfg.endtime, :);
            % heightdata = heightdata(heightdata(:, 1) >= cfg.starttime, :);
            % heightdata = heightdata(heightdata(:, 1) <= cfg.endtime, :);


            %% 调试
            disp("Start RANGE/INS Processing!");
            lastprecent = 0;

            %% 初始化
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

                % 区分是否需要距离更新
                if lastimu(1, 1) == rangedata(rangeindex, 1)&&beaconnums~=0
                    % 先对当前状态进行测量更新
                    thisRange = rangedata(rangeindex, :);
                    depthdata = [0,depstd*randn];
                    % if mod(navstate.time,interval*60*6)==0
                    %     tt = thisRange(1);
                    %     Rangedata1 = rangedata1(rangedata1(:,1)==tt,:) ;
                    %     Rangedata2 = rangedata2(rangedata2(:,1)==tt,:) ;
                    %     Rangedata3 = rangedata3(rangedata3(:,1)==tt,:) ;
                    %     Rangedata1(3) = Rangedata1(3) + randn * rangstd ;
                    %     Rangedata2(3) = Rangedata2(3) + randn * rangstd ;
                    %     Rangedata3(3) = Rangedata3(3) + randn * rangstd ;
                    %     kf = my3RangeUpdate(navstate,Rangedata1 ,Rangedata2, Rangedata3, depthdata, kf);
                    %     % fprintf('   3距离\n')
                    % else
                        kf = myRangeUpdate(navstate, thisRange, depthdata, kf);
                    % end
                    [kf, navstate] = myErrorFeedback_range(kf, navstate);
                    rangeindex = rangeindex + 1;
                    laststate = navstate;

                    % 惯导解算
                    imudt = thisimu(1, 1) - lastimu(1, 1);
                    navstate = InsMech(laststate, lastimu, thisimu);
                    navstate.pos(3)=0;
                    kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
                elseif (lastimu(1, 1) < rangedata(rangeindex, 1) && thisimu(1, 1) > rangedata(rangeindex, 1))&&beaconnums~=0
                    % 将imu插值到测量值的时间
                    [firstimu, secondimu] = interpolate(lastimu, thisimu, rangedata(rangeindex, 1));

                    % 前一个imu到测量值时间的惯导更新
                    imudt = firstimu(1, 1) - lastimu(1, 1);
                    navstate = InsMech(laststate, lastimu, firstimu);
                    navstate.pos(3)=0;
                    kf = myInsPropagate_15state(navstate, firstimu, imudt, kf);

                    % 测量更新
                    thisRange = rangedata(rangeindex, :);
                    depthdata = [0,depstd*randn];
                    kf = myRangeUpdate(navstate, thisRange, depthdata, kf);
                    [kf, navstate] = myErrorFeedback_range(kf, navstate);
                    rangeindex = rangeindex + 1;
                    laststate = navstate;
                    lastimu = firstimu;

                    % 测量时间到下一个惯导值的更新
                    imudt = secondimu(1, 1) - lastimu(1, 1);
                    navstate = InsMech(laststate, lastimu, secondimu);
                    navstate.pos(3)=0;
                    kf = myInsPropagate_15state(navstate, secondimu, imudt, kf);
                else
                    % 仅做惯导结算
                    % INS 编排
                    navstate = InsMech(laststate, lastimu, thisimu);
                    % 尝试做天向速度和位置约束
                    % navstate.vel(3)=(-heightdata(imuindex,1)-(-heightdata(imuindex-1,1)))/imudt;
                    navstate.pos(3)=0+randn*depstd;
                    % error propagation
                    % if mod(imuindex,20)==0
                    %     navstate.pos(3)=heightdata(floor(imuindex/20),1);
                    % end
                    % 状态传播
                    kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
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
                % if (imuindex / size(imudata, 1) - lastprecent > 0.05)
                %     disp("processing " + num2str(floor(imuindex * 100 / size(imudata, 1))) + " %!");
                %     lastprecent = imuindex / size(imudata, 1);
                % end
            end
            % 关闭文件
            % fclose(imuerrfp);
            fclose(navfp);
            % fclose(stdfp);
            % fclose(xkfp);
            disp("range/INS Integration Processing Finished!");
            %%

            %%
            calc_error(navpath,cfg.truthpath)
            plot_result(navpath,'full')
            truthpath=cfg.truthpath;
        end
    end
end
%%
navp = [cfg.outputfolder,'/'];
pureins=[navp,'pureINS.nav'];
for i=1:3
    single4{i}=[navp,'single',num2str(i),'-4min.nav'];
    single1{i}=[navp,'single',num2str(i),'-1min.nav'];
    single2{i}=[navp,'single',num2str(i),'-2min.nav'];
    single1s{i}=[navp,'single',num2str(i),'-1s.nav'];
end
three{1}=[navp,'3-1s.nav'];
three{2}=[navp,'3-5s.nav'];
three{3}=[navp,'3-30s.nav'];
three{4}=[navp,'3-1min.nav'];
% three{5}=[navp,'3-2min.nav'];
three{5}=[navp,'3-4min.nav'];
three{6}=[navp,'3-5min.nav'];
%%
plot_trj(cfg.truthpath,pureins,single4{2},three{5})
% calc_radial_error(cfg.truthpath,pureins)
% calc_error(three{5},cfg.truthpath)
%%
calc_radial_error(cfg.truthpath,pureins,single4{:},three{5})
%%
calc_radial_error(cfg.truthpath,pureins,single{[2,5,8]},three{3})
%%
calc_radial_error(cfg.truthpath,pureins,single{[3,6,9]},three{3})
%%
calc_radial_error(cfg.truthpath,pureins,single{10:12},three{3})
%%
calc_radial_error(cfg.truthpath,pureins,single{[7,8,9,12]},three{5})
ylim([0 1000])
%%
calc_radial_error(cfg.truthpath,pureins,three{:})
ylim([0 200])