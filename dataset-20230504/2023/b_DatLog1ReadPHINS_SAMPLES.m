%% 将示例中的数据读取
%% data={'depth','DVL','GPS','Ins_D','Ins_Nav'};
%% 深度计数据
clear
fid=fopen('PHINS_6000-D-20140924-103004-ins-depth.xpf.txt','rt'); 
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
t(1)=mean(diff(depth(end,:)));
%% DVL的数据
fid=fopen('PHINS_6000-D-20140924-103004-ins-dvl.xpf.txt','rt'); 
for i=1:15
fgets(fid);
end
DVL=fscanf(fid,'%d/%d/%d %d:%d:%d.%f %f %f %f %f %f %f %f %f %f\n',[16,inf]);
DVL(17,:)=DVL(4,:)*3600+DVL(5,:)*60+DVL(6,:)+DVL(7,:)/10000;
DVL(1:7,:)=[];
% 2023/05/04	00:17:02.8618	0.936682	1.070203	0.14677 方向XYZ
% 1500.000000(传感器声速)	1528.860962（外部声速）	79.982498（高度）	0.200000	0.200000	0.200000
fclose(fid);
% % 采样时间0.20s左右
% figure
% plot(diff(DVL(end,:)))
t(2)=mean(diff(DVL(end,:)));
%% 导航结果数据
fid=fopen('PHINS_6000-D-20140924-103004-ins-nav.xpf.txt','rt'); 
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
% plot(diff(Ins_Nav(end,:)),'.')
t(5)=mean(diff(Ins_Nav(end,:)));
%% 惯导数据 
% fid=fopen('log1-20230430-0347-ins-imu.xpf.txt','rt'); 
% fid=fopen('log1-20230504-0017-ins.xpf.txt','rt'); 
fid=fopen('PHINS_6000-D-20140924-103004-ins-imu.xpf.txt','rt');
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
t(4)=mean(diff(Ins_D(end,:)));
%% 长基线数据
% 与USBL的一致
fid=fopen('PHINS_6000-D-20140924-103004-ins-lbl.xpf.txt','rt'); 
for i=1:12
fgets(fid);
end
LBL=fscanf(fid,'%d/%d/%d %d:%d:%d.%f %d %f %f %f %f %f \n',[13,inf]);
% 2023/05/04	00:17:04.4559	132	15.837422227487	115.142674930394	4209.720215	1936.098022	5.944000
LBL(14,:)=LBL(4,:)*3600+LBL(5,:)*60+LBL(6,:)+LBL(7,:)/10000;
LBL(1:7,:)=[];
fclose(fid);
% 长基线的数据只有信标的位置和距离
beaconID=[0,1,2,3];
BCNN=cell(1,3);
RNG=cell(1,3);
beacon=cell(1,4);
% 按照id筛选位置信息
figure
for i=1:4
    beacon{i}=LBL(:,LBL(1,:)==beaconID(i));
    subplot(2,2,i)
    plot(beacon{i}(3,:),beacon{i}(2,:),'.')
end
% 按照位置筛选
BCNN{1}=LBL(:,LBL(3,:)<-4.441&LBL(2,:)<48.313);
BCNN{2}=LBL(:,LBL(3,:)>-4.441&LBL(2,:)<48.313);
BCNN{3}=LBL(:,LBL(3,:)>-4.441&LBL(2,:)>48.313);

figure
for i=1:3
    RNG{i}=[BCNN{i}(5,:);BCNN{i}(end,:)];
    plot(BCNN{i}(3,:),BCNN{i}(2,:),'.')
    hold on
end
grid on
% % 距离信息
% figure
% for i=1:4
% subplot(2,2,i),plot(RNG{i})
% end
% 
% 采样时间 14s
%% 深度对比：惯导结果和深度计
myfigurestartup(4,4,'prese')
plot(depth(end,:),-depth(1,:),Ins_Nav(end,:),Ins_Nav(3,:))
legend('depther','INS')
grid on
title('Depth Comparasion')
%% GPS
fid=fopen('PHINS_6000-D-20140924-103004-ins-gps.xpf.txt','rt'); 
for i=1:15
fgets(fid);
end
GPS=fscanf(fid,'%d/%d/%d %d:%d:%d.%f %d %f %f %f %f %f %f %f %f \n',[16,inf]);
% 2014/09/24	08:30:04.9997	2	48.314111391082	-4.447853732854	
% 0.000000	0.010000	0.010000	0.010000	0.000000	0.000000
GPS(17,:)=GPS(4,:)*3600+GPS(5,:)*60+GPS(6,:)+GPS(7,:)/10000;
GPS([1:8,15:16],:)=[];
tt=7900;
t1=tt+1250;
myfigurestartup(10,5,'prese')
plot(GPS(2,1:tt),GPS(1,1:tt))
hold on
plot(GPS(2,tt:t1),GPS(1,tt:t1))
hold on 
plot(GPS(2,t1:end),GPS(1,t1:end))
xygo('lon','lat')
legend('PHASE 1','PHASE 2','PHASE 3')
% 采样时间1s
% figure
% plot(diff(GPS(end,:)))
t(3)=mean(diff(GPS(end,:)));
%% 信标和导航结果对比
myfigurestartup(12,5,'prese')
plot(GPS(2,:),GPS(1,:))
xygo('lon','lat')
for i=1:3
plot(BCNN{i}(3,:),BCNN{i}(2,:),'*')
hold on
end
title('trajectory and beacons from PHINS')
%% 时间，各类数据的起始和结束时间
format bank;
data1=["depth","DVL","GPS","Ins_D","Ins_Nav"];
data={'depth','DVL','GPS','Ins_D','Ins_Nav'};
disp(data)
for i=1:5
    value=eval(data{i});
    start(i)=value(end,1);
    endtime(i)=value(end,end);
end
disp(start)
disp(endtime)
disp(t)
% 选择起始时间和终止时间：30605~42421 
%% 保存数据
save matData\PHINS-sample.mat Ins_D GPS Ins_Nav depth DVL BCNN RNG t
