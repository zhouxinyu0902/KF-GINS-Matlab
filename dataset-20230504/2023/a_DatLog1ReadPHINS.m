% 读PHINS导出的数据
clear
load('matData\LBL&USBL.mat')
%% 深度计数据
% fid=fopen('log1-20230430-0347-ins-depth.xpf.txt','rt'); 
% fid=fopen('log1-20230504-0124-ins-depth.xpf.txt','rt'); 
fid=fopen('log1-20230504-0017-ins-depth.xpf.txt','rt'); 
% fid=fopen('PHINS_6000-D-20140924-103004-ins-depth.xpf.txt','rt'); 
% 2023/05/04	01:24:05.8512	7	4127.930664	1.000000	42039320.000000
for i=1:10
fgets(fid);
end
depth=fscanf(fid,'%d/%d/%d %d:%d:%d.%f %d %f %f %f\n',[11,inf]);
depth(12,:)=depth(4,:)*3600+depth(5,:)*60+depth(6,:)+depth(7,:)/10000;
fclose(fid);
depth(1:8,:)=[];
% 采样时间0.048s左右
% figure
% plot(diff(depth(end,:)))

%% DVL数据
% fid=fopen('log1-20230430-0347-ins-dvl.xpf.txt','rt'); 
fid=fopen('log1-20230504-0017-dvl.xpf.txt','rt'); 
% fid=fopen('log1-20230504-0124-ins-DVL.xpf.txt','rt'); 
% fid=fopen('PHINS_6000-D-20140924-103004-ins-dvl.xpf.txt','rt'); 
for i=1:15
fgets(fid);
end
DVL=fscanf(fid,'%d/%d/%d %d:%d:%d.%f %f %f %f %f %f %f %f %f %f\n',[16,inf]);
DVL(17,:)=DVL(4,:)*3600+DVL(5,:)*60+DVL(6,:)+DVL(7,:)/10000;
DVL(1:7,:)=[];
% 2023/05/04	00:17:02.8618	0.936682	1.070203	0.14677 方向XYZ
% 1500.000000(传感器声速)	1528.860962（外部声速）	79.982498（高度）	0.200000	0.200000	0.200000
fclose(fid);
% % 采样时间0.62s左右
% figure
% plot(diff(DVL(end,:)))
%% 导航结果数据
% fid=fopen('log1-20230430-0347-ins-nav.xpf.txt','rt');
fid=fopen('log1-20230504-0017-ins-nav.xpf.txt','rt');
% fid=fopen('log1-20230504-0124-ins-NAV.xpf.txt','rt');
% fid=fopen('PHINS_6000-D-20140924-103004-ins-nav.xpf.txt','rt'); 
for i=1:21
fgets(fid);
end
Ins_Nav=fscanf(fid,'%d/%d/%d %d:%d:%d.%f %d %d %d %f %f %f %f %f %f %f %f %f %f %f %f\n',[22,inf]);
Ins_Nav(23,:)=Ins_Nav(4,:)*3600+Ins_Nav(5,:)*60+Ins_Nav(6,:)+Ins_Nav(7,:)/10000;
Ins_Nav(1:10,:)=[];
% 2023/05/04	00:17:02.6748	3146529	0	67108864	15.820634196889	115.147240101896	-4128.846191	
% 351.475006	-0.744000	4.775000	1.460148	-0.375065	0.012396	0.154284	0.003127	-0.141090
% heading	roll	pitch  speedNorth	speedEast	speedUp
% heave	 surge	sway
fclose(fid);
% 采样时间1s左右
% figure
% plot(diff(Ins_Nav(end,:)))
%% 惯导数据 
% fid=fopen('log1-20230430-0347-ins-imu.xpf.txt','rt'); 
fid=fopen('log1-20230504-0017-ins.xpf.txt','rt'); 
% fid=fopen('PHINS_6000-D-20140924-103004-ins-imu.xpf.txt','rt');
for i=1:21
fgets(fid);
end
Ins_D=fscanf(fid,'%d/%d/%d %d:%d:%d.%f %d %f %f %f %f %f %f %d\n',[15,inf]);
Ins_D(16,:)=Ins_D(4,:)*3600+Ins_D(5,:)*60+Ins_D(6,:)+Ins_D(7,:)/10000;
Ins_D([1:8,15],:)=[];
% 2023/05/04	00:17:02.6273	5847	1672.526	-45181.953	-71571.743	
% 31882.687	-164349.172	1952256.858	0
fclose(fid);
% figure
% plot(diff(Ins_D(end,:)))
% 采样时间0.01s
%% IMU数据
gyro=d2r([Ins_D(1,:);Ins_D(2,:);Ins_D(3,:)]*1e-3/3600);
acc=[Ins_D(4,:);Ins_D(5,:);Ins_D(6,:)]*1e-7;
imu=[gyro;acc;Ins_D(end,:)]';
imu_phins = imuRFU2FRD(imu);

%% 长基线数据，与USBL的一致
fid=fopen('log1-20230504-0017-lbl.xpf.txt','rt'); 
% fid=fopen('log1-20230504-0124-ins-LBL.xpf.txt','rt'); 
% fid=fopen('PHINS_6000-D-20140924-103004-ins-lbl.xpf.txt','rt'); 
for i=1:12
fgets(fid);
end
LBL=fscanf(fid,'%d/%d/%d %d:%d:%d.%f %d %f %f %f %f %f \n',[13,inf]);
% 2023/05/04	00:17:04.4559	132	15.837422227487	115.142674930394	4209.720215	1936.098022	5.944000
LBL(14,:)=LBL(4,:)*3600+LBL(5,:)*60+LBL(6,:)+LBL(7,:)/10000;
LBL(1:7,:)=[];
fclose(fid);

beaconID=[132,133,134,135];% 132=90
beacon=cell(1,4);
for i=1:4
    beacon{i}=LBL(:,LBL(1,:)==beaconID(i));
    subplot(2,2,i)
    plot(diff(beacon{i}(end,:)))
end
% 采样时间 14s
%% 深度对比：惯导结果和深度计
myfigurestartup(4,4,'prese')
plot(depth(end,:),-depth(1,:),Ins_Nav(end,:),Ins_Nav(3,:),timeLBL,-posLBL(10,:),timeUSBL,-posUSBL(12,:))
legend('depther','INS','LBL','USBL')
grid on
title('Depth Comparasion')
%% 长基线与PHINS-nav对比
myfigurestartup(5,5,'prese')
set(0,'defaultLineMarkerSize',6);
plot3(Ins_Nav(2,:),Ins_Nav(1,:),Ins_Nav(3,:),'.')
hold on
plot3(posLBL(9,:),posLBL(8,:),-posLBL(10,:),'.')
xygo('lon','lat')

%% 信标和导航结果对比
myfigurestartup(4,4,'prese')
for i=1:4
plot(beacon{i}(3,:),beacon{i}(2,:),'*')
hold on
end
plot(Ins_Nav(2,:),Ins_Nav(1,:))
xygo('lon','lat')
title('trajectory and beacons from PHINS')
%% 
imupath="dataset-20230504\depth.nav";
imufp=fopen(imupath,'wt');
fprintf(imufp, '%.9f %.10f\n', depth([1,4],:));
fclose(imufp);

imupath="dataset-20230504\imu_phins.nav";
imufp=fopen(imupath,'wt');
fprintf(imufp, '%.10f %.10f %.10f %.10f %.10f %.10f %.10f \n', imu_phins');
fclose(imufp);

depth_intep=interp1(depth(end,:),depth(1,:),Ins_D(end,:),'linear');
imupath="dataset-20230504\depth-nav.nav";
imufp=fopen(imupath,'wt');
fprintf(imufp, '%.10f %.10f \n', [depth_intep',Ins_D(end,:)']');
fclose(imufp);
%%
LBLresult=[posLBL(8:10,:);timeLBL]';
imupath="dataset-20230504\LBL.nav";
imufp=fopen(imupath,'wt');
fprintf(imufp, '%.10f %.10f %.10f %.10f \n', LBLresult');
fclose(imufp);
save dataset-20230504\2023\matData\PHINS-0017.mat Ins_D Ins_Nav DVL depth beacon imu
