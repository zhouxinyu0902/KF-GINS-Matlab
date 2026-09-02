clear all
glvs
load("D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1207-longtime\matdata\830.mat")
load("D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1207-longtime\matdata\120.mat")
%%
[PVA_830_PM,IMU_FRD_830_PM,IMU_FRD_120_PM,pva_file_PM]=allProcess(AUAX_PM,imu_830_PM,IMU120_PM,pva_830_PM,IMU_DATA_830_PM);
%%
path="D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1207-longtime\cese-01\";

output_file=path+"\pva_file.txt";
writematrix(pva_file_PM, output_file, 'Delimiter', ' ');
fprintf('120导航信息已成功写入到 %s\n', output_file);

output_file=path+"\pva_830.txt";
writematrix(PVA_830_PM, output_file, 'Delimiter', ' ');
fprintf('830导航信息已成功写入到 %s\n', output_file);

output_file=path+"\imu_830.txt";
writematrix(IMU_FRD_830_PM, output_file, 'Delimiter', ' ');
fprintf('830imu信息已成功写入到 %s\n', output_file);

output_file=path+"\IMU_120.txt";
writematrix(IMU_FRD_120_PM, output_file, 'Delimiter', ' ');
fprintf('imu信息已成功写入到 %s\n', output_file);

%%
[PVA_830_NT,IMU_FRD_830_NT,IMU_FRD_120_NT,pva_file_NT]=allProcess(AUAX_NT,imu_830_NT,IMU120_NT,pva_830_NT,IMU_DATA_830_NT);

%%
path="D:\Github\KF-GINS-Matlab\data\experiment-data\experiment-02\1207-longtime\case-02";

output_file=path+"\pva_file.txt";
writematrix(pva_file_NT, output_file, 'Delimiter', ' ');
fprintf('120导航信息已成功写入到 %s\n', output_file);

output_file=path+"\pva_830.txt";
writematrix(PVA_830_NT, output_file, 'Delimiter', ' ');
fprintf('830导航信息已成功写入到 %s\n', output_file);

output_file=path+"\imu_830.txt";
writematrix(IMU_FRD_830_NT, output_file, 'Delimiter', ' ');
fprintf('830imu信息已成功写入到 %s\n', output_file);

output_file=path+"\IMU_120.txt";
writematrix(IMU_FRD_120_NT, output_file, 'Delimiter', ' ');
fprintf('imu信息已成功写入到 %s\n', output_file);
%% 辅助函数
function [PVA_830,IMU_FRD_830,IMU_FRD_120,pva_file_PM]=allProcess(AUAX,imu_830,IMU120,pva_830,IMU_DATA_830)
%% 1、处理120的auax数据
% 导航部分数据可视化
[imu_file_PM,pva_file_PM,~]=visualize_auax_navigation_results(AUAX, AUAX(:,1));
% pva_init = pva_file_PM(:,1);
%% 2、处理120的IMU文件数据
% imu_file：串口输出的imu数据，带上电时间以及GPS时间，以串口输出的为标准，调整原始imu
% IMU_FUR_file：读取的imu原始数据，只有上电时间
IMU_FUR_120  = IMU120;  
IMU_FUR_120(:,7) = IMU_FUR_120(:,7)/200; % 处理时间戳
idstart=find(abs(IMU_FUR_120(:,1) - imu_file_PM(2,1))<0.0001 &...
    abs(IMU_FUR_120(:,2) - imu_file_PM(2,2))<0.0001&...
    abs(IMU_FUR_120(:,3) - imu_file_PM(2,3))<0.0001);
if isempty(idstart)
    idstart = find(abs(IMU_FUR_120(:,7)-imu_file_PM(2,7))<0.007);
end


IMU_FUR_120 = IMU_FUR_120(idstart(end):end,:); % 裁剪imu原始数据
% 向imu原始数据增加新的一列GPS时间，按照串口输出的添加
IMU_FUR_120(:,8) = IMU_FUR_120(:,7)+imu_file_PM(2,8)-IMU_FUR_120(1,7); 
% 对齐完成后，检查一下数据是否大致一样
% 转换坐标系，原始的数据是FUR--->FRD--->RFU
IMU_FRD_120 = imuFUR2FRD(IMU_FUR_120);
% IMU_RFU_120_PM = imuFRD2RFU(IMU_FRD_120_PM);
%% 3、处理IMU数据 数据可视化
ss = find(abs(IMU_DATA_830(:,7)-IMU_FUR_120(1,8))<0.005);
ee = find(abs(IMU_DATA_830(:,7)-IMU_FUR_120(end,8))<0.005);
if isempty(ee)
    IMU_830=IMU_DATA_830(ss:end,:);
else
    IMU_830=IMU_DATA_830(ss:ee,:);
end

IMU_RFU_830=[];
IMU_RFU_830(:,1:3)=IMU_830(:,1:3)/180*pi;
IMU_RFU_830(:,4:6)=IMU_830(:,4:6)*9.806;
IMU_RFU_830(:,[2,5])=-IMU_RFU_830(:,[2,5]);
IMU_RFU_830(:,7)=IMU_830(:,7);
IMU_FRD_830=imuRFU2FRD(IMU_RFU_830);

imu_830_RFU_INCRE = imu_830;
imu_830_RFU_INCRE(:,1:3)=imu_830_RFU_INCRE(:,1:3)/180*pi*0.01;
imu_830_RFU_INCRE(:,4:6)=imu_830_RFU_INCRE(:,4:6)*9.806*0.01;
imu_FRD_830=imuRFU2FRD(imu_830_RFU_INCRE);
close all
plot_imu_frd(IMU_FRD_120);
plot_imu_frd(IMU_FRD_830);
plot_imu_frd(imu_FRD_830);
%% 处理PVA830
ss = find(abs(pva_830(:,2)-IMU_FUR_120(1,8))<0.005);
ee = find(abs(pva_830(:,2)-IMU_FUR_120(end,8))<0.005);
PVA_830=pva_830(ss:ee,:);

%%
disp('-----------------');
starttime = utc_week_seconds_to_bjt_with_leap(2396, IMU_FRD_830(1,1), 18);
endtime = utc_week_seconds_to_bjt_with_leap(2396, IMU_FRD_830(end,1), 18);
fprintf('830的IMU起始时间: %s\n', starttime);
fprintf('结束时间: %s\n', endtime);
disp('-----------------');
starttime = utc_week_seconds_to_bjt_with_leap(2396, PVA_830(1,2), 18);
endtime = utc_week_seconds_to_bjt_with_leap(2396, PVA_830(end,2), 18);
fprintf('PVA830的IMU起始时间: %s\n', starttime);
fprintf('结束时间: %s\n', endtime);
disp('-----------------');
starttime = utc_week_seconds_to_bjt_with_leap(2396, IMU_FUR_120(1,8), 18);
endtime = utc_week_seconds_to_bjt_with_leap(2396, IMU_FUR_120(end,8), 18);
fprintf('120的IMU起始时间: %s\n', starttime);
fprintf('结束时间: %s\n', endtime);
disp('-----------------');
%%
end
