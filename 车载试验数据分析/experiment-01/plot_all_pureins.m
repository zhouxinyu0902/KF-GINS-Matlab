clear;clc;close all;
%% =========================================================================
% Pure INS 纯惯导前3600s精度评估
% 文件：
%   case-XX\navigation-results\PureIns-rad.nav
% 数据格式：
%   第2列：时间/s
%   第3列：纬度/deg
%   第4列：经度/deg
%   第5列：高度/m
%
% 评价原则：
% 1. 有效时长>=3600s：仅统计前3600s，不归一化。
% 2. 有效时长<3600s：使用全部有效数据，并备注不足3600s。
% 3. 常规指标：Max、RMSE、Mean、Std、Final，单位m。
% 4. CEP50：
%       R50 = K50*sqrt(1/N*sum(RER_i^2))
%       RER_i = radial_error_i(n mile)/T_i(h)
%       K50 = 0.8326
%    CEP50单位：n mile/h
%% ======================== 1. 配置 ==============================
experiment_root='D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-01';
target_cases=1:8;
evaluation_duration=3600;
duration_tolerance=1.0;
K50=0.8326;
meter_per_nmile=1852;
second_per_hour=3600;
cfg=setup_experiment_01();
%% ======================== 2. 输出目录 ==========================
output_dir=fullfile(experiment_root,'summary');
if ~exist(output_dir,'dir')
    mkdir(output_dir);
end
%% ======================== 3. 结果容器 ==========================
case_list=[];
duration_list=[];
complete_list=[];
max_list=[];
rmse_list=[];
mean_list=[];
std_list=[];
final_list=[];
cep50_list=[];
rer_rms_list=[];
note_list=strings(0,1);
fprintf('\n============================================================\n');
fprintf('Pure INS 前3600s精度评估\n');
fprintf('============================================================\n');
fprintf('经纬度单位：deg\n');
fprintf('CEP50方法 ：R50=K50*RMS(RER)\n');
fprintf('K50       ：%.4f\n',K50);
fprintf('============================================================\n\n');
%% 画图
fig=myfigurestartup(15,7,'prese');
%% ======================== 4. 遍历各组实验 =====================
for case_id=target_cases
    fprintf('------------------------------------------------------------\n');
    fprintf('Case %02d\n',case_id);
    case_dir=cfg.case_navigation(case_id);
    nav_file=fullfile(cfg.case_navigation(case_id),'PureIns-rad.nav');
    if ~exist(nav_file,'file')
        warning('Case %02d 找不到 PureIns-rad.nav：\n%s',case_id,nav_file);
        continue;
    end
    truth_file=fullfile(cfg.case_input(case_id),'truth.nav');
    if isempty(truth_file)
        warning('Case %02d 找不到 truth.nav。',case_id);
        continue;
    end
    fprintf('Pure INS : %s\n',nav_file);
    fprintf('Truth    : %s\n',truth_file);
    %% 读取数据
    nav_raw=importdata(nav_file);
    truth_raw=importdata(truth_file);
    if isstruct(nav_raw)
        nav=nav_raw.data;
    else
        nav=nav_raw;
    end
    if isstruct(truth_raw)
        truth=truth_raw.data;
    else
        truth=truth_raw;
    end
    if isempty(nav)||isempty(truth)
        warning('Case %02d：nav或truth为空。',case_id);
        continue;
    end
    if size(nav,2)<5||size(truth,2)<5
        warning('Case %02d：nav或truth少于5列。',case_id);
        continue;
    end
    %% 删除无效数据
    nav_valid=isfinite(nav(:,2))&isfinite(nav(:,3))&isfinite(nav(:,4))&isfinite(nav(:,5));
    truth_valid=isfinite(truth(:,2))&isfinite(truth(:,3))&isfinite(truth(:,4))&isfinite(truth(:,5));
    nav=nav(nav_valid,:);
    truth=truth(truth_valid,:);
    if isempty(nav)||isempty(truth)
        warning('Case %02d：删除无效值后数据为空。',case_id);
        continue;
    end
    %% 时间排序及去重
    nav=sortrows(nav,2);
    truth=sortrows(truth,2);
    [~,idx_nav]=unique(nav(:,2),'stable');
    nav=nav(idx_nav,:);
    [~,idx_truth]=unique(truth(:,2),'stable');
    truth=truth(idx_truth,:);
    %% 确定共同有效时间
    t_start=max(nav(1,2),truth(1,2));
    t_end_available=min(nav(end,2),truth(end,2));
    if t_end_available<=t_start
        warning('Case %02d：导航结果与真值没有共同时间区间。',case_id);
        continue;
    end
    t_end_eval=min(t_start+evaluation_duration,t_end_available);
    mask=nav(:,2)>=t_start&nav(:,2)<=t_end_eval;
    nav_eval=nav(mask,:);
    if size(nav_eval,1)<2
        warning('Case %02d：有效数据过少。',case_id);
        continue;
    end
    time=nav_eval(:,2);
    elapsed_time=time-t_start;
    %% Truth插值
    truth_interp=interp1(truth(:,2),truth(:,3:5),time,'linear');
    valid_interp=all(isfinite(truth_interp),2);
    nav_eval=nav_eval(valid_interp,:);
    truth_interp=truth_interp(valid_interp,:);
    time=time(valid_interp);
    elapsed_time=time-t_start;
    if isempty(elapsed_time)
        warning('Case %02d：Truth插值后无有效数据。',case_id);
        continue;
    end
    %% 经纬度误差转NE位置误差
    lat_nav_deg=nav_eval(:,3);
    lon_nav_deg=nav_eval(:,4);
    lat_truth_deg=truth_interp(:,1);
    lon_truth_deg=truth_interp(:,2);
    h_truth=truth_interp(:,3);
    [north_error,east_error]=position_error_deg_to_ne(lat_nav_deg,lon_nav_deg,lat_truth_deg,lon_truth_deg,h_truth);
    %% 水平径向误差
    radial_error=hypot(north_error,east_error);
    valid_error=isfinite(radial_error)&isfinite(elapsed_time);
    radial_error=radial_error(valid_error);
    elapsed_time=elapsed_time(valid_error);
    if isempty(radial_error)
        warning('Case %02d：没有有效径向误差。',case_id);
        continue;
    end
    actual_duration=elapsed_time(end);
    %% 判断是否达到3600s
    if actual_duration>=evaluation_duration-duration_tolerance
        complete_flag=true;
        note_text='使用前3600s原始数据统计，不归一化。';
    else
        complete_flag=false;
        note_text=sprintf('有效时长%.2fs，不足3600s；CEP50按实际运行时间计算径向误差率。',actual_duration);
    end
    %% 常规径向误差统计
    max_error=max(radial_error);
    rmse_error=sqrt(mean(radial_error.^2));
    mean_error=mean(radial_error);
    std_error=std(radial_error);
    final_error=radial_error(end);
    %% CEP50
    % R50=K50*sqrt(1/N*sum(RER_i^2))
    % RER_i=径向误差(n mile)/运行时间(h)
    % t=0不能参与计算
    cep_mask=elapsed_time>0;
    cep_time_s=elapsed_time(cep_mask);
    cep_radial_error_m=radial_error(cep_mask);
    if isempty(cep_time_s)
        RER_RMS=NaN;
        CEP50=NaN;
        warning('Case %02d：没有T>0的数据，无法计算CEP50。',case_id);
    else
        radial_error_nmile=cep_radial_error_m/meter_per_nmile;
        elapsed_time_h=cep_time_s/second_per_hour;
        RER=radial_error_nmile./elapsed_time_h;
        valid_rer=isfinite(RER)&RER>=0;
        RER=RER(valid_rer);
        if isempty(RER)
            RER_RMS=NaN;
            CEP50=NaN;
        else
            RER_RMS=sqrt(mean(RER.^2));
            CEP50=K50*RER_RMS;
        end
    end
    %% 画图
    subplot(2,4,case_id);
    grid on;
    hold on;
    plot(elapsed_time, radial_error, 'b-');
    yline(CEP50*meter_per_nmile, 'r--');
    text(100,CEP50*meter_per_nmile,sprintf("%.2f n mile",CEP50))
    xlabel('Time/s');
    ylabel('径向误差/m');
    if case_id==1
    legend('径向误差','CEP径向误差','Location','best');
    end
    hold off;
    %% 保存结果
    case_list(end+1,1)=case_id;
    duration_list(end+1,1)=actual_duration;
    complete_list(end+1,1)=complete_flag;
    max_list(end+1,1)=max_error;
    rmse_list(end+1,1)=rmse_error;
    mean_list(end+1,1)=mean_error;
    std_list(end+1,1)=std_error;
    final_list(end+1,1)=final_error;
    rer_rms_list(end+1,1)=RER_RMS;
    cep50_list(end+1,1)=CEP50;
    note_list(end+1,1)=string(note_text);
    %% 控制台输出
    fprintf('有效时长 : %.2f s\n',actual_duration);
    fprintf('Max      : %.3f m\n',max_error);
    fprintf('RMSE     : %.3f m\n',rmse_error);
    fprintf('Mean     : %.3f m\n',mean_error);
    fprintf('Std      : %.3f m\n',std_error);
    fprintf('Final    : %.3f m\n',final_error);
    fprintf('RER RMS  : %.6f n mile/h\n',RER_RMS);
    fprintf('CEP50    : %.6f n mile/h\n',CEP50);
    fprintf('备注     : %s\n',note_text);
end
%% ======================== 5. 汇总结果 =========================
summary_table=table(case_list,duration_list,logical(complete_list),max_list,rmse_list,mean_list,std_list,final_list,rer_rms_list,cep50_list,note_list,...
    'VariableNames',{'Case','Duration_s','Full3600s','Max_m','RMSE_m','Mean_m','Std_m','Final_m','RER_RMS_nmile_per_h','CEP50_nmile_per_h','Note'});
fprintf('\n============================================================\n');
fprintf('汇总结果\n');
fprintf('============================================================\n');
disp(summary_table);

%% ======================== 6. 保存 ==============================
excel_path=fullfile(output_dir,'PureIns_first3600s_GJB729_statistics.xlsx');
csv_path=fullfile(output_dir,'PureIns_first3600s_GJB729_statistics.csv');
writetable(summary_table,excel_path);
writetable(summary_table,csv_path);
fprintf('============================================================\n');
fprintf('统计完成\n');
fprintf('Excel : %s\n',excel_path);
fprintf('CSV   : %s\n',csv_path);
fprintf('============================================================\n');

exportgraphics(fig,fullfile(output_dir,'8组数据纯惯导误差汇总对比图.png'),'Resolution',600)
%% ======================== Local Function =======================
function [north_error,east_error]=position_error_deg_to_ne(lat_est_deg,lon_est_deg,lat_true_deg,lon_true_deg,h_true)
a=6378137.0;
f=1/298.257223563;
e2=f*(2-f);
lat_true_rad=deg2rad(lat_true_deg);
dlat_deg=lat_est_deg-lat_true_deg;
dlon_deg=lon_est_deg-lon_true_deg;
dlon_deg=mod(dlon_deg+180,360)-180;
dlat_rad=deg2rad(dlat_deg);
dlon_rad=deg2rad(dlon_deg);
sin_lat=sin(lat_true_rad);
temp=sqrt(1-e2.*sin_lat.^2);
RM=a*(1-e2)./temp.^3;
RN=a./temp;
north_error=dlat_rad.*(RM+h_true);
east_error=dlon_rad.*(RN+h_true).*cos(lat_true_rad);
end