%% 用于生成标准参考
clear;
%% 定义全局参数
rng(1)
feedback = 1; % 是否反馈，不反馈则可以观察参数
glvs
%% 定义参数+加载过程配置
param = Param();
% path='旋转收缩方案1/input/input_nt';
% path='旋转收缩方案1/input/input_pm';
ID = 6;
path=['F:/2_Data/惯导试验/实验数据/All_data/input',num2str(ID)];
% cfg = ProcessConfigforSemiPhy_all(path);
cfg = ProcessConfig_truth(path);
cfg.outputfolder =['D:\Github\KF-GINS-Matlab\new_惯导试验\output/output',num2str(ID)];
mkdir(cfg.outputfolder)
%% 加载数据
% imudata
imudata = importdata(cfg.imufilepath);
imustarttime = imudata(1, 1);
imuendtime = imudata(end, 1);
%%
% ==============================================================
% 1. gnss data 读取与合并
% ==============================================================
gnssdata_raw = importdata(cfg.gnssfilepath);

find(gnssdata_raw(:,3)>37|gnssdata_raw(:,3)<36)

std_raw = importdata(cfg.stdfilepath);

% 提取列: [时间sec, lat, lon, alt, std_lat, std_lon, std_alt]
gnssdata = [gnssdata_raw(:,2:5), std_raw(:,2:4)];

% 去除可能存在的重复时间戳 (必须做，否则 interp1 函数会报错)
[~, unique_idx] = unique(gnssdata(:,1));
gnssdata = gnssdata(unique_idx, :);



% ==============================================================
% 2. 🌟 核心：时间轴线性插值，完全覆盖掉帧
% ==============================================================
% 估算稳定的中位数时间步长 (通常是 0.01 或 1 秒)
dt_gnss = round(median(diff(gnssdata(:,1))), 3); 

% 构建“完美无缺”的连续理想时间轴
t_ideal = (gnssdata(1,1) : dt_gnss : gnssdata(end,1))'; 

% 对时间列之外的其他所有列进行插值，并覆盖原 gnssdata
gnssdata_interp = interp1(gnssdata(:,1), gnssdata(:,2:end), t_ideal, 'linear', 'extrap');
gnssdata = [t_ideal, gnssdata_interp];

% 经纬度转弧度 (此时第1列是时间，2、3列是经纬度)
gnssdata(:, 2:3) = gnssdata(:, 2:3) * param.D2R;

if (size(gnssdata, 2) < 13)
    cfg.usegnssvel = false;
end
gnssstarttime = gnssdata(1, 1);
gnssendtime = gnssdata(end, 1);

gnssrtk=gnssdata;
gnssrtk(:,2:3)=r2d(gnssrtk(:,2:3));
writematrix(gnssrtk,'exper\GNSS_RTK.txt')


gnssrtk_1s=gnssdata(1:100:end,:);
gnssrtk_1s(:,2:3)=r2d(gnssrtk_1s(:,2:3));
writematrix(gnssrtk_1s,'exper\GNSS_RTK_1s.txt')
% ==============================================================
% 3. height data 处理 (强制对齐到同一个插值时间轴)
% ==============================================================
truth = importdata(cfg.gnssfilepath);
[~, unique_idx_truth] = unique(truth(:,2));
truth = truth(unique_idx_truth, :);

% 直接插值到上面生成的 t_ideal 时间轴上，保证行数和时间完全对齐
height_interp  = interp1(truth(:,2), truth(:,5), t_ideal, 'linear', 'extrap');
heightv_interp = interp1(truth(:,2), truth(:,8), t_ideal, 'linear', 'extrap');

height  = [t_ideal, height_interp];
heightv = [t_ideal, heightv_interp];

heistarttime = height(1, 1);
heitendtime  = height(end, 1);

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
gnssdata = gnssdata(gnssdata(:, 1) >= cfg.starttime, :);
gnssdata = gnssdata(gnssdata(:, 1) <= cfg.endtime, :);

%% 设置文件保存路径
navpath = [path,'/truth.nav'];
navfp = fopen(navpath, 'wt');

% imuerrpath = [cfg.outputfolder, '/ImuError-gnss.txt'];
% imuerrfp = fopen(imuerrpath, 'wt');
% 
% stdpath = [cfg.outputfolder, '/NavSTD.txt'];
% stdfp = fopen(stdpath, 'wt');
% 
if feedback==0
    xkpath = [cfg.outputfolder, '/xk-gnss.txt'];
    xkfp = fopen(xkpath, 'wt');
end
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

gnssindex = 1;
while gnssdata(gnssindex, 1) < thisimu(1, 1)
    gnssindex = gnssindex + 1;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MAIN PROCEDD PROCEDURE!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for imuindex = 2:size(imudata, 1)

    %% set value of last state
    lastimu = thisimu;
    laststate = navstate;
    thisimu = imudata(imuindex, :)';
    imudt = thisimu(1, 1) - lastimu(1, 1);

    %% compensate IMU error
    % thisimu(2:4, 1) = (thisimu(2:4, 1) - imudt * navstate.gyrbias)./(ones(3, 1) + navstate.gyrscale);
    % thisimu(5:7, 1) = (thisimu(5:7, 1) - imudt * navstate.accbias)./(ones(3, 1) + navstate.accscale);

    %% adjust GNSS index
    while (gnssindex <= size(gnssdata, 1) && gnssdata(gnssindex, 1) < lastimu(1, 1))
        gnssindex = gnssindex + 1;
    end
    % check whether gnss data is valid
    if (gnssindex > size(gnssdata, 1))
        disp('GNSS file END!');
        break;
    end

    %% determine whether gnss update is required
    if lastimu(1, 1) == gnssdata(gnssindex, 1)
        % do gnss update for the current state
        thisgnss = gnssdata(gnssindex, :)';
        kf = myGNSSUpdate_15state(navstate, thisgnss, kf);
        % kf = myGNSSUpdate(navstate, thisgnss, kf, cfg.antlever);
        if feedback==1
            [kf, navstate] = myErrorFeedback_noatt(kf, navstate);
        end
        gnssindex = gnssindex + 1;
        laststate = navstate;

        % do propagation for current imu data
        imudt = thisimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, thisimu);
        navstate.pos(3) = height(imuindex,2);
        navstate.vel(3) = heightv(imuindex,2);
        kf = myInsPropagate_15state(navstate, thisimu, imudt, kf);
    elseif (lastimu(1, 1) < gnssdata(gnssindex, 1) && thisimu(1, 1) > gnssdata(gnssindex, 1))
        % ineterpolate imu to gnss time
        [firstimu, secondimu] = interpolate(lastimu, thisimu, gnssdata(gnssindex, 1));

        % do propagation for first imu
        imudt = firstimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, firstimu);
        kf = myInsPropagate_15state(navstate, firstimu, imudt, kf);

        % do gnss update
        thisgnss = gnssdata(gnssindex, :)';
        kf = myGNSSUpdate_15state(navstate, thisgnss, kf);
        % kf = myGNSSUpdate(navstate, thisgnss, kf, cfg.antlever);
        if feedback==1
            [kf, navstate] = myErrorFeedback_noatt(kf, navstate);
        end
        gnssindex = gnssindex + 1;
        laststate = navstate;
        lastimu = firstimu;

        % do propagation for second imu
        imudt = secondimu(1, 1) - lastimu(1, 1);
        navstate = InsMech(laststate, lastimu, secondimu);
        navstate.pos(3) = height(imuindex,2);
        navstate.vel(3) = heightv(imuindex,2);
        kf = myInsPropagate_15state(navstate, secondimu, imudt, kf);
    else
        %% only do propagation
        % INS mechanization
        navstate = InsMech(laststate, lastimu, thisimu);
        navstate.pos(3) = height(imuindex,2);
        navstate.vel(3) = heightv(imuindex,2);
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
    % % 保存估计的状态值
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
    if (imuindex / size(imudata, 1) - lastprecent > 0.20) 
        disp("processing " + num2str(floor(imuindex * 100 / size(imudata, 1))) + " %!");
        lastprecent = imuindex / size(imudata, 1);
    end
end
%%
% close file
% fclose(imuerrfp);
% fclose(xkfp);
fclose(navfp);
% fclose(stdfp);
disp("gnss/INS Integration Processing Finished!");
%%
calc_error(navpath,cfg.gnssfilepath);
% calc_error(navpath,[path,'/pva_RS.txt'])
% calc_error(navpath,[path,'/pva_430.txt'])
% calc_error([path,'/pva_830.txt'],[path,'/pva_430.txt'])

%%
rangedataget_back(path,'right');
%%
% plot_result(cfg.gnssfilepath)
% plot_result(navpath)

