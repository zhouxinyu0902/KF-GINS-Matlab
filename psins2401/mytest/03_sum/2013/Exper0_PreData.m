clear
glvs
% 流程见幕布文档笔记，已形成优化版本
%% 1.舱内的数据导入 --------------------------------------------------
path = 'D:\Github\KF-GINS-Matlab\data\psins\data_1\';
% COMPASS-OCTANS-DVL-RANGE-DEPTH    
fid=fopen([path,'1COMPS_2OCTANS_3DVL_4SHIP_5RANGE_6TSD.txt'],'rt'); 
fgets(fid);
DistData_rw=fscanf(fid,'%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d/%d/%d %d:%d:%d\n',[23,inf]);
fclose(fid);
fid=fopen([path,'DVLheight_heighter.txt'],'rt'); 
fgets(fid);
height=fscanf(fid,'%f,%f,%d/%d/%d %d:%d:%d\n',[8,inf]);
fclose(fid);
% LBL的位置信息
fid=fopen([path,'POS20130628_LBL.txt'],'rt');
DistData2=fscanf(fid,'%f,%f,%f,%d/%d/%d %d:%d:%d\n',[9,inf]);
fclose(fid);

% -----航向角绘图------
myfigurestartup(7,3,'zxy');
subplot 121
plot(DistData_rw(1,:))% 罗盘航向角
hold on
plot(DistData_rw(6,:))% OCTANS航向角
legend('compass','OCTANS')
subplot 122
plot(DistData_rw(6,:)-DistData_rw(1,:))
ylim([-10,10])

% 将长基线的位置信息加入DistData1
DistData_rw(18:20,:)=DistData2(1:3,:);
tmp1=DistData_rw(1,:);tmp2=DistData_rw(3,:);DistData_rw(1,:)=tmp2;DistData_rw(3,:)=tmp1;
DistData_rw(23:25,:)=DistData_rw(21:23,:);
DistData_rw(21:22,:)=height(1:2,:);

clear DistData2 fid ans tmp1 tmp2 height
%% 2.超短基线的数据导入---------------------------------------------------
fid = fopen([path,'USBL_BOX-R-20130628-080921_PTXAG.log'],'rt');
PTSAG_rw = fscanf(fid,'$PTSAG,#%d,%2d%2d%f,%d,%d,%d,%d,%2d%f,%c,%3d%f,%c,%X,%f,%d,%f*%X\n',[19,inf]);                
fclose(fid);

fid = fopen([path,'USBL_BOX-R-20130628-080921_PIXOG.log'],'rt');
PIXOG_rw = fscanf(fid,'$PIXOG,PPC,DETEC,%2d%2d%f,%d,%d,%d,%d,  %f,%f,%f,  %f,%f,%f,  %f,%f,%f,  %f,%f,%d,%f,   %f,%f,%d,%f,   %f,%f,%d,%f,   %f,%f,%d,     %f*%X\n',[33,inf]);
fclose(fid);

fid = fopen([path,'USBL_BOX-R-20130628-080921_PTSAX.log'],'rt');
PTSAX_rw = fscanf(fid,'$PTSAX,#%d,%2d%2d%f,%d,%d,%d,%d,%f,%f,%X,%f,%d,%f*%X\n',[15,inf]);
fclose(fid);
clear fid ans;
% 处理PTSAG的时间，放在最后三行
PTSAG_rw(end-2:end,:) = PTSAG_rw(2:4,:); 
% 处理PTSAG的纬度经度，放在9和12行
Lat = PTSAG_rw(9,:) + PTSAG_rw(10,:)/60;
Ins = find( PTSAG_rw(11, :)==83 ); %'S'
Lat(Ins) = -Lat(Ins); 
Lon = PTSAG_rw(12,:) + PTSAG_rw(13,:)/60;
Iew = find( PTSAG_rw(14,:)==87 ); %'W'
Lon(Iew) = -Lon(Iew); 
PTSAG_rw(9,:)=Lat;
PTSAG_rw(12,:)=Lon;
PTSAG_rw([2:7,10:11,13:14],:)=[];% 2~7时间，10~11纬度分，13~14经度分
clear Lat Lon Ins Iew

% 将船和潜水器分开
PTSAG_rw(end,:)=floor(PTSAG_rw(end,:));
PTSAGShip_rw = PTSAG_rw( :, PTSAG_rw(2,:)==0);
PTSAGHov_rw = PTSAG_rw( :, PTSAG_rw(2,:)~=0);

% 处理PIXOG的时间，放在最后三行
PIXOG_rw(34:36,:)=PIXOG_rw(1:3,:);
PIXOG_rw(end,:)=round(PIXOG_rw(end,:));
% 处理PTSAX的时间，放在最后三行
PTSAX_rw(16:18,:)=PTSAX_rw(2:4,:); 
PTSAX_rw(end,:)=round(PTSAX_rw(end,:));
clear PTSAG_rw
%% 时间段选取
% 时间定义以及滞后时间 深度计校正
ATdelay = 38;
AdepC = 15.5;
% AdepC=10;
TimeUSBL=[41458,50258]; % 11:30:58--13:57:38
TimeLBL=TimeUSBL+ATdelay; % 11:31:36--13:58:16
TT_USBL=TimeUSBL(1):8:TimeUSBL(end);
tt_usbl=0:8:TT_USBL(end)-TT_USBL(1);
%% 航行器数据 处理时间
%--------------------------------------------------------------------------
% 航行器数据说明
% DistData 1~3（罗盘） 4~6（octans） 7~8（DVL） 9~10（母船） 
% 11~14（长基线四个信标距离） 15~17（盐度温度深度） 18~20（长基线定位结果） 
% 21~22（DVL高度 高度计） 23~25（时间）
%--------------------------------------------------------------------------
TimeZone = 0; % 航行控制计算机所设的时区，东区为正，西区为负。
DistData=DistData_rw;
[TT,~,DistData] = timepro(TimeLBL,TimeZone,DistData,'LBL');
% 找出只出现了一次的时间点，复制这些时间点
[~,rptedN1,~,~,~] = repeated(TT); 
B=zeros(size(DistData,1),length(DistData)+length(rptedN1));
B(:,1:length(DistData))=DistData;
for i=1:length(rptedN1)
    B(:,1:rptedN1(i)+i-1)=B(:,1:rptedN1(i)+i-1);
    B(:,rptedN1(i)+i)=DistData(:,rptedN1(i));
    B(:,rptedN1(i)+i+1:end-length(rptedN1)+i)=DistData(:,rptedN1(i)+1:end);
end
[TT,~,B]=timepro(TimeLBL,TimeZone,B,'LBL');
% 找出出现了三次的时间点，删除这些时间点
[~,~,rptedN3,~,~] = repeated(TT);
colsToKeep =  setdiff(1:size(B,2), rptedN3);
DistData = B(:, colsToKeep);
[TT_LBL,tt_lbl,DistData]=timepro(TimeLBL,TimeZone,DistData,'LBL');
% [misgNs,rptedN1,rptedN3,~,~] = repeated(TT); % 最后一次查询
clear colsToKeep rptedN1 rptedN3 B TimeZone  fid  i TT
%% 处理选取后绘图
myfigurestartup(5,2,'paper');
TT_raw = DistData_rw(end-2,:)*3600+DistData_rw(end-1,:)*60+DistData_rw(end,:);
TT_new = DistData(end-2,:)*3600+DistData(end-1,:)*60+DistData(end,:);
subplot 121,
set(0,'defaultLineMarkerSize',6)
plot(TT_raw,DistData_rw(7,:),'.'),
set(0,'defaultLineMarkerSize',4)
hold on,
plot(TT_new,DistData(7,:),'.')
xlim([TT_raw(1) TT_raw(end)])
ylim([-0.2 0.2])
ConvertXAxisTime;
xygo('hh mm ss','velocity-x/ m/s')
legend('all time','chosen time')
DeciPoin(0,2)
subplot 122,
set(0,'defaultLineMarkerSize',6)
plot(TT_raw,DistData_rw(8,:),'.'),
hold on,
set(0,'defaultLineMarkerSize',4)
plot(TT_new,DistData(8,:),'.')
xlim([TT_raw(1) TT_raw(end)])
ylim([-0.2 1])
ConvertXAxisTime;
xygo('hh mm ss','velocity-y/ m/s')
legend('all time','chosen time')
DeciPoin(0,1)

%% 超短基线数据 时间处理
% 航行器数据说明
% PTSAGHov 超短基线对潜水器的定位结果
% PTSAX 超短基线前向右向的相对距离
% PIXOG 超短基线的传播时间
TimeZone=8;
[TimePTSAGShip,ttPTSAGShip,PTSAGShip]=timepro(TimeUSBL,TimeZone,PTSAGShip_rw,'USBL');

[TimePTSAGHov,ttPTSAGHov,PTSAGHov]=timepro(TimeUSBL,TimeZone,PTSAGHov_rw,'USBL');
[TimePTSAX,ttPTSAX,PTSAX]=timepro(TimeUSBL,TimeZone,PTSAX_rw,'USBL');
[TimePIXOG,ttPIXOG,PIXOG]=timepro(TimeUSBL,TimeZone,PIXOG_rw,'USBL');
clear index TimeZone 

%% 绘图 超短基线选取时间段
% 定位结果经纬度
myfigurestartup(5,2,'paper');
TT_raw = PTSAGHov_rw(end-2,:)*3600+PTSAGHov_rw(end-1,:)*60+PTSAGHov_rw(end,:)+8*3600;
TT_new = PTSAGHov(end-2,:)*3600+PTSAGHov(end-1,:)*60+PTSAGHov(end,:)+8*3600;
subplot 121,
set(0,'defaultLineMarkerSize',6)
plot(TT_raw,PTSAGHov_rw(3,:),'.'),
hold on,
set(0,'defaultLineMarkerSize',4)
plot(TT_new,PTSAGHov(3,:),'.')
xlim([TT_raw(1) TT_raw(end)])
ConvertXAxisTime;
xygo('hh mm ss','lat/deg')
legend('all time','chosen time')
DeciPoin(0,2)
subplot 122,
set(0,'defaultLineMarkerSize',6)
plot(TT_raw,PTSAGHov_rw(4,:),'.'),
hold on,
set(0,'defaultLineMarkerSize',4)
plot(TT_new,PTSAGHov(4,:),'.')
xlim([TT_raw(1) TT_raw(end)])
ConvertXAxisTime;
xygo('hh mm ss','lon/deg')
legend('all time','chosen time')
DeciPoin(0,3)
% print(gcf, 'New Folder\试验预处理-chosedtime.png', '-dpng','-r600');
% print(gcf, 'C:\Users\23764\OneDrive\文档\LATEX\距离辅助航位推算总报告（2024）\picture\试验预处理-chosedtime.png', '-dpng','-r600');
%% 绘图-超短基线和长基线对齐   
% 将舱内和支持母船的位置存储的母船位置进行对齐
myfigurestartup(7,3,'paper');
subplot 121,
plot(ttPTSAGShip,PTSAGShip(3,:)),
hold on,plot(tt_lbl+ATdelay,DistData(10,:),'.')
hold on,plot(tt_lbl,DistData(10,:),'.')
xygo('t/s','lat/deg')
xlim([0 8800]);
legend('USBL-raw','AUV','AUV-processed')
DeciPoin(0,3)
subplot 122,
plot(ttPTSAGShip,PTSAGShip(3,:)),
hold on,plot(tt_lbl+ATdelay,DistData(10,:),'.')
hold on,plot(tt_lbl,DistData(10,:),'.')
legend('USBL-raw','AUV','AUV-processed')
xygo('t/s','lat/deg')
xlim([2000 3500]);
DeciPoin(0,3)
% print(gcf, 'New Folder\试验预处理-对齐纬度.png', '-dpng','-r600');
%% 航行器本体传感器数据处理（yaw depth）
% 航向角数据
compass=DistData(1:3,:)';
yaw=-compass(:,3)+360;
yaw(yaw>180)=yaw(yaw>180)-360;
compass(:,3)=yaw;
compass=d2r(compass);
octans=DistData(4:6,:)';
yaw=-octans(:,3)+360;
yaw(yaw>180)=yaw(yaw>180)-360;
octans(:,3)=yaw;
octans=d2r(octans);
% denoise(yaw,3,0.2)
clear yaw
% 绘图
myfigurestartup(5,2,'paper');
subplot 121,plot(tt_lbl,DistData(3,:))
xygo('t/s','phi/deg')
title('raw yaw')
subplot 122,plot(tt_lbl,compass(:,3))
xygo('t/s','phi/rad')
title('reversed yaw')
% print(gcf, 'New Folder\试验预处理-本体-yaw.png', '-dpng','-r600');
% print(gcf, 'C:\Users\23764\OneDrive\文档\LATEX\距离辅助航位推算总报告（2024）\picture\试验预处理-本体-yaw.png', '-dpng','-r600');
% 深度计 高度计数据
depther=DistData(17,:)'+AdepC;
height=DistData(21,:)';
%% 速度信息处理 
% VXYZ 原始数据 
% VXYZ_n 去除零点后的 
% vxy 声速补偿后的
VXYZ=DistData(7:8,:)';
vx=setvals(VXYZ(:,1));
[~ ,indice]=findzerosfrc(vx,50);% 去除为零的点，连续点数小于50
for i=1:length(indice)
    vx(indice(1,i):indice(2,i))=mean(nonzeros(vx(indice(1,i):indice(2,i)+2)));
end
VXYZ_n(:,1)=vx;
vy=setvals(VXYZ(:,2));
[~ ,indice]=findzerosfrc(vy,50);% 去除为零的点，连续点数小于50
for i=1:length(indice)
    vy(indice(1,i):indice(2,i))=mean(nonzeros(vy(indice(1,i):indice(2,i)+2)));
end
VXYZ_n(:,2)=vy;
% 对温度 盐度进行降噪
salinity=DistData(15,:); % 盐度
T_water=DistData(16,:); % 温度
salinity=denoise(salinity,2,0.2);
T_water=denoise(T_water,2,0.2);
close all
% 计算声速，修正速度
ss=prealloc(length(depther),1);
for i=1:length(depther)
    ss(i,1)=1449.2+4.6*T_water(i)-0.055*T_water(i)^2+0.00029*T_water(i)^3+(1.34-0.01*T_water(i))*(salinity(i)*10-35)+0.016*depther(i,1);
end
Creal_dvl=ss/1500; 
vxy=[VXYZ_n(:,1).*Creal_dvl,VXYZ_n(:,2).*Creal_dvl];
% 绘图
myfigurestartup(5,2,'paper');set(0,'defaultLineMarkerSize',6)
subplot 121,plot(tt_lbl,VXYZ(:,1), '.',...
    tt_lbl,vxy(:,1),'.'),xygo('t/s','velocity-x')
DeciPoin(0,2)
xlim([0 8800])
legend('raw','corrected')
subplot 122,plot(tt_lbl,VXYZ(:,2),'.', ...
    tt_lbl,vxy(:,2),'.'),xygo('t/s','velocity-y')
DeciPoin(0,1)
legend('raw','corrected')
xlim([0 8800])

%% 长基线信标及距离 
BCN=cell(1,4);
BCN{1}=[deg2rad([17+34/60+53.2675/3600,117+48/60+13.9232/3600]),-3856.6];
BCN{2}=[deg2rad([17+35/60+54.6676/3600,117+47/60+33.9740/3600]),-3848.4];
BCN{3}=[deg2rad([17+35/60+5.121/3600,117+46/60+27.3108/3600]),-3806.06];
BCN{4}=[deg2rad([17+34/60+1.3920/3600,117+47/60+27.03/3600]),-3856.3];
RNG=cell(1,4);
RNG_raw=cell(1,4);
% [RANGE1,TimeLBL1] = indentify_error(TT_LBL,DistData(11,:)',30,4,'RANGE/m');
% [RANGE2,TimeLBL2] = indentify_error(TT_LBL,DistData(12,:)',30,9,'RANGE/m');
% [RANGE3,TimeLBL3] = indentify_error(TT_LBL,DistData(13,:)',30,10,'RANGE/m');
% [RANGE4,TimeLBL4] = indentify_error(TT_LBL,DistData(14,:)',30,140,'RANGE/m');
for i=1:4
    RNG_raw{i}=DistData(10+i,:)';
    RNG{i}=smooth(tt_lbl,RNG_raw{i},0.02,'rloess'); % 平滑原始距离
end
%%
% 距离信息平滑之后的绘图对比
myfigurestartup(5,4,'paper');
for i=1:4
    subplot(2,2,i);
    plot(tt_lbl,RNG_raw{i}(:));hold on
    plot(tt_lbl,RNG{i}(:));
    y=sprintf('beacon-%d',i);
    title(y)
    xygo('t/s','range/m')
    legend('raw data','smoothed','Location','best')
end
clear i
% 母船数据
LatLonShipCabin=[d2r(DistData(10,:));d2r(DistData(9,:));tt_lbl];

%% 长基线位置平滑
ID=1:16:16*length(tt_usbl);
lat=smooth(tt_lbl(ID),d2r(DistData(19,ID)),0.02,'rloess');
lon=smooth(tt_lbl(ID),d2r(DistData(18,ID)),0.035,'rloess');
lat=interp1(tt_lbl(ID),lat,tt_lbl,'linear')';
lon=interp1(tt_lbl(ID),lon,tt_lbl,'linear')';
lat(end)=d2r(DistData(19,end));
lon(end)=d2r(DistData(18,end));
% 画图
myfigurestartup(5,2,'paper');
set(0,'defaultLineMarkerSize',4)
subplot 121 ,plot(tt_lbl,d2r(DistData(19,:)),tt_lbl,lat);
xygo('t/s','lat/rad')
legend('raw data','smoothed')
DeciPoin(0,5)
subplot 122 ,plot(tt_lbl,d2r(DistData(18,:)),tt_lbl,lon);
xygo('t/s','lon/rad')
legend('raw data','smoothed')
avp_m=[compass,zeros(length(compass),3),lat,lon,-depther,tt_lbl'];
clear lon lat
DeciPoin(0,5)% 使用这个限制横纵坐标小数点
%% 航行器 使用长基线的定位结果计算距离
close all
ll=length(DistData(19,:));
avp_ll=[zeros(ll,6),d2r(DistData(19,:))',d2r(DistData(18,:))',-depther];
myfigurestartup(5,5,'paper');
for i=1:4
    range{i}=RCompu(avp_ll(:,7:9),BCN{i});
    subplot(2,2,i)
    plot(range{i}),hold on,plot(RNG{i})
    RNG1{i}=range{i};
end
%% 深度计以及超短基线定位深度处理
% 将深度异常值识别并处理
DepthHovPTSAG=PTSAGHov(6,:);
DepthHovPTSAGrw=DepthHovPTSAG;
index=DepthHovPTSAG>3940|DepthHovPTSAG<3860;
DepthHovPTSAG(index)=[];
DepthHovPTSAG = interp1(ttPTSAGHov(~index), DepthHovPTSAG, tt_usbl, 'linear'); % 线性插值
% DepthHovPTSAG = smooth(tt_usbl,DepthHovPTSAG,0.01,'rloess')';

% 超短基线的定位深度和深度计的深度对比
close all,
myfigurestartup(5,2,'paper');
plot(tt_usbl,DepthHovPTSAG),
hold on,plot(tt_lbl,DistData(17,:)+AdepC)
hold on,plot(tt_lbl,DistData(17,:))
legend('depth from USBL','depther+15.5m','depther')
xygo('t/s','depth/m')
clear dep
% 深度处理画图
myfigurestartup(5,2,'paper'),
set(0,'defaultLineMarkerSize',8)
plot(ttPTSAGHov,-DepthHovPTSAGrw,'.') % 处理前后
hold on
set(0,'defaultLineMarkerSize',4)
plot(tt_usbl,-DepthHovPTSAG,'.')
axis([tt_usbl(1) tt_usbl(end) -4100 -3500]);
legend('raw','processed')
xygo('t/s','depth/m')
clear t_dep index

%% 超短基线PTSAG处理 
% PTSAG HOV位置处理（超短基线导出）
LatHovPTSAG=PTSAGHov(3,:);
LatHovPTSAGrw=LatHovPTSAG;
close all
[LatHovPTSAG1,TimePTSAGHov1] = indentify_error(TimePTSAGHov,LatHovPTSAGrw,100,0.0025,'lat/deg');
% print(gcf, 'New Folder\试验预处理-PTSAG-LAT-KNN.png', '-dpng','-r600');
% print(gcf, 'C:\Users\23764\OneDrive\文档\LATEX\距离辅助航位推算总报告（2024）\picture\试验预处理-PTSAG-LAT-KNN.png', '-dpng','-r600');
[LatHovPTSAG2,TimePTSAGHov2] = indentify_error(TimePTSAGHov1,LatHovPTSAG1,50,0.0016,'lat/deg');
[LatHovPTSAG3,TimePTSAGHov3] = indentify_error(TimePTSAGHov2,LatHovPTSAG2,50,0.0012,'lat/deg');

xlim([TimePTSAGHov(1) TimePTSAGHov(end)])
LatHovPTSAG=interp1(TimePTSAGHov3,LatHovPTSAG3,TT_USBL,'linear');
LatHovPTSAG=smooth(tt_usbl,LatHovPTSAG,0.02,'rloess')';
clear LatHovPTSAG1 TimePTSAGHov1 LatHovPTSAG2 TimePTSAGHov2 LatHovPTSAG3 TimePTSAGHov3
% 处理经度
LonHovPTSAG=PTSAGHov(4,:);
LonHovPTSAGrw=LonHovPTSAG;
close all
[LonHovPTSAG1,TimePTSAGHov1] = indentify_error(TimePTSAGHov,LonHovPTSAGrw,100,0.0020,'lon/deg');
% print(gcf, 'New Folder\试验预处理-PTSAG-LON-KNN.png', '-dpng','-r600');
% print(gcf, 'C:\Users\23764\OneDrive\文档\LATEX\距离辅助航位推算总报告（2024）\picture\试验预处理-PTSAG-LON-KNN.png', '-dpng','-r600');
[LonHovPTSAG2,TimePTSAGHov2] = indentify_error(TimePTSAGHov1,LonHovPTSAG1,50,0.0012,'lon/deg');
xlim([TimePTSAGHov(1) TimePTSAGHov(end)])
LonHovPTSAG=interp1(TimePTSAGHov2,LonHovPTSAG2,TT_USBL,'linear');
LonHovPTSAG=smooth(tt_usbl,LonHovPTSAG,0.02,'rloess')';
clear LonHovPTSAG1 TimePTSAGHov1 LonHovPTSAG2 TimePTSAGHov2 

% 处理完之后的画图
myfigurestartup(5,2,'paper'),set(0,'defaultLineMarkerSize',8)
subplot 121,plot(TimePTSAGHov,LatHovPTSAGrw,'.',TT_USBL,LatHovPTSAG)
xlim([TimePTSAGHov(1) TimePTSAGHov(end)])
legend('raw data','processed')
ConvertXAxisTime
xygo('hh mm ss','lat/deg')
DeciPoin(0,3)
subplot 122,plot(TimePTSAGHov,LonHovPTSAGrw,'.',TT_USBL,LonHovPTSAG)
ConvertXAxisTime
xlim([TimePTSAGHov(1) TimePTSAGHov(end)])
legend('raw data','processed')
xygo('hh mm ss','lon/deg')
DeciPoin(0,3)
% print(gcf, 'New Folder\试验预处理-PTSAG-SUM-KNN.png', '-dpng','-r600');
% print(gcf, 'C:\Users\23764\OneDrive\文档\LATEX\距离辅助航位推算总报告（2024）\picture\试验预处理-PTSAG-SUM-KNN.png', '-dpng','-r600');
% clear DepthHovPTSAGrw LatHovPTSAGrw LonHovPTSAGrw id result

% 潜水器的位置-不同来源
[XUTMHovPTSAG,YUTMHovPTSAG] = ll2utm(LatHovPTSAG,LonHovPTSAG); 
%% 超短基线PISAX处理
close all
% 潜水器的PTSAX的深度和PTSAG的深度一样
DepthHovPTSAX = DepthHovPTSAG;
% figure,plot(DepthHovPTSAX)

% 前向位移滤波处理
XForwardPTSAX = PTSAX(9,:);
XForwardPTSAXrw = XForwardPTSAX;
% close all 
[XForwardPTSAX1,TimePTSAX1] = indentify_error(TimePTSAX,XForwardPTSAXrw,50,450,'XForward/m');
% print(gcf, 'New Folder\试验预处理-PTSAX-X-KNN.png', '-dpng','-r600');
% print(gcf, 'C:\Users\23764\OneDrive\文档\LATEX\距离辅助航位推算总报告（2024）\picture\试验预处理-PTSAX-X-KNN.png', '-dpng','-r600');

xlim([TimePTSAX(1) TimePTSAX(end)])
XForwardPTSAX=interp1(TimePTSAX1,XForwardPTSAX1,TT_USBL,'linear');
XForwardPTSAX=smooth(tt_usbl,XForwardPTSAX,0.01,'rloess')';
clear TimePTSAX1 XForwardPTSAX1

% 右向位移滤波处理
YStarboardPTSAX=PTSAX(10,:);
YStarboardPTSAXrw=YStarboardPTSAX;

% close all 
[YStarboardPTSAX1,TimePTSAX1] = indentify_error(TimePTSAX,YStarboardPTSAXrw,30,500,'YStarboard/m');
% print(gcf, 'New Folder\试验预处理-PTSAX-Y-KNN.png', '-dpng','-r600');
% print(gcf, 'C:\Users\23764\OneDrive\文档\LATEX\距离辅助航位推算总报告（2024）\picture\试验预处理-PTSAX-Y-KNN.png', '-dpng','-r600');
[YStarboardPTSAX2,TimePTSAX2] = indentify_error(TimePTSAX1,YStarboardPTSAX1,35,430,'YStarboard/m');
[YStarboardPTSAX3,TimePTSAX3] = indentify_error(TimePTSAX2,YStarboardPTSAX2,30,400,'YStarboard/m');

xlim([TimePTSAX(1) TimePTSAX(end)])
YStarboardPTSAX=interp1(TimePTSAX3,YStarboardPTSAX3,TT_USBL,'linear');
YStarboardPTSAX=smooth(tt_usbl,YStarboardPTSAX,0.01,'rloess')';

clear TimePTSAX1 YStarboardPTSAX1 TimePTSAX2 YStarboardPTSAX2 TimePTSAX3 YStarboardPTSAX3

% 处理完之后的画图
% close all
myfigurestartup(5,2,'paper'),set(0,'defaultLineMarkerSize',8)
subplot 121,plot(TimePTSAX,XForwardPTSAXrw,'.',TT_USBL,XForwardPTSAX),
xlim([TimePTSAX(1),TimePTSAX(end)])
ConvertXAxisTime
legend('raw data','processed')
xygo('hh mm ss','XForward distance/m')
subplot 122,plot(TimePTSAX,YStarboardPTSAXrw,'.',TT_USBL,YStarboardPTSAX)
xlim([TimePTSAX(1),TimePTSAX(end)])
ConvertXAxisTime
xygo('hh mm ss','YStarboard distance/m')
legend('raw data','processed')
% print(gcf, 'New Folder\试验预处理-PTSAX-SUM-KNN.png', '-dpng','-r600');
% print(gcf, 'C:\Users\23764\OneDrive\文档\LATEX\距离辅助航位推算总报告（2024）\picture\试验预处理-PTSAX-SUM-KNN.png', '-dpng','-r600');
%% 超短基线PIXOG处理
% PIXOG, 处理距离的传播时间，20ms为信标的周转时间
close all
TimeH1PIXOG = (PIXOG(17,:)-20*1e3) * 1e-6;
TimeH1PIXOGrw = TimeH1PIXOG;
[TimeH1PIXOG1,TimePIXOG1] = indentify_error(TimePIXOG,TimeH1PIXOGrw,20,0.28,'PropaTime/(s)');
% print(gcf, 'New Folder\试验预处理-PIXOG-T-KNN.png', '-dpng','-r600');
% print(gcf, 'C:\Users\23764\OneDrive\文档\LATEX\距离辅助航位推算总报告（2024）\picture\试验预处理-PIXOG-T-KNN.png', '-dpng','-r600');
xlim([TimePIXOG(1) TimePIXOG(end)])
TimeH1PIXOG=interp1(TimePIXOG1,TimeH1PIXOG1,TT_USBL,'spline');

myfigurestartup(2.5,2.5,'paper');
plot(TimePIXOG,TimeH1PIXOGrw,'.')
hold on,plot(TT_USBL,TimeH1PIXOG)
axis([TT_USBL(1) TT_USBL(end) 1 4])
ConvertXAxisTime
xygo('hh mm ss','PropaTime/(s)')
legend('raw data','processed')
% print(gcf, 'New Folder\试验预处理-PIXOG-SUM-KNN.png', '-dpng','-r600');
% print(gcf, 'C:\Users\23764\OneDrive\文档\LATEX\距离辅助航位推算总报告（2024）\picture\试验预处理-PIXOG-SUM-KNN.png', '-dpng','-r600');
% 处理母船的位置 用于换能器位置的转换
LatShipPIXOG=interp1(TimePIXOG,r2d(PIXOG(8, :)),TT_USBL,'linear');
LonShipPIXOG=interp1(TimePIXOG,r2d(PIXOG(9, :)),TT_USBL,'linear');
HeadingShip=interp1(TimePIXOG,r2d(PIXOG(11,:)),TT_USBL,'linear');

DepthShipPIXOG=interp1(TimePIXOG,PIXOG(10, :),TT_USBL,'linear');
% DepthShipPIXOGsm=smooth(tt_usbl,DepthShipPIXOG,0.008,'rloess')';
% figure,plot(tt_usbl,DepthShipPIXOG,tt_usbl,DepthShipPIXOGsm)
%%  LL2UTM将坐标LAT、LON (度)转换为UTM X、Y (米)。 默认数据为WGS84
close all
% 计算换能器的位置 in UTM and deg
[XUTMShipPIXOG,YUTMShipPIXOG,f] =...
ll2utm(LatShipPIXOG,LonShipPIXOG);
lv1 = 3.1; lv2 = -0.66; lv3 = 27.3;  % 向阳红九号杠杆臂
H1rangeGPSUSBL = sqrt( (-lv1+0.29)^2 + (lv2+0)^2 ); 
H1angleGPSUSBL = rad2deg (atan2( -lv1+0.29, lv2+0 ));
% 换能器的位置 in UTM
xdelta = H1rangeGPSUSBL * cos( deg2rad(H1angleGPSUSBL+HeadingShip-90) );
ydelta = H1rangeGPSUSBL * sin( deg2rad(H1angleGPSUSBL+HeadingShip-90) );
myfigurestartup(5,2,'paper');
subplot 211,plot(tt_usbl,xdelta)
xygo('t/s','dx/m')
subplot 212,plot(tt_usbl,ydelta)
xygo('t/s','dy/m')
% print(gcf, 'New Folder\试验预处理-SHIP2TRANS.png', '-dpng','-r600');
% print(gcf, 'C:\Users\23764\OneDrive\文档\LATEX\距离辅助航位推算总报告（2024）\picture\试验预处理-SHIP2TRANS.png', '-dpng','-r600');
XUTMTranPIXOG = XUTMShipPIXOG + xdelta;
YUTMTranPIXOG = YUTMShipPIXOG - ydelta;
% 换能器的位置 in deg
[LatTranPIXOG,LonTranPIXOG] = utm2ll(XUTMTranPIXOG,YUTMTranPIXOG,f);
LonTranPIXOG=LonTranPIXOG';
LatTranPIXOG=LatTranPIXOG';
% myfigurestartup(7,3,'paper'),
% plot(LonShipPIXOG,LatShipPIXOG);
% hold on;
% plot(LonTranPIXOG,LatTranPIXOG)
% legend('Ship','Transponder')
% xygo('lon','lat')
%% 根据声学定位结果和传感器数据进行航位推算融合 长基线的定位误差设置2m
LenLBL=length(tt_lbl);
LenUSBL=length(tt_usbl);
avp_lbl_raw=avp_m;
avp_usbl_raw=[zeros(LenUSBL,6),d2r([LatHovPTSAG',LonHovPTSAG']),-depther(1:16:16*LenUSBL),tt_usbl'];
Oweb=d2r(0.2);Oeb=d2r(0.025); % OCTANS的指标
close all
[avp_USBL_DR,avp_dr_octans_usbl] = AcousticDeadR('USBL',tt_lbl,avp_lbl_raw,Oweb,avp_usbl_raw, ...
    octans,vxy,depther,1);

[avp_LBL_DR,avp_dr_octans_lbl] = AcousticDeadR('LBL',tt_lbl,avp_lbl_raw,Oweb,avp_lbl_raw, ...
    octans,vxy,depther,1);
web=d2r(0.5);eb=d2r(0.3); % 罗盘的指标
[avp_LBL_DR1,avp_dr_compass_lbl] = AcousticDeadR('LBL',tt_lbl,avp_lbl_raw,web,avp_lbl_raw, ...
    compass,vxy,depther,1);
%% 对比长基线、长基线融合octans、长基线融合compass
myfigurestartup(5,2.5,'paper');
subplot 121,
dxyz = pos2dxyz([avp_LBL_DR(:,7:8),avp_LBL_DR(:,9)*0]);
dxyzship = pos2dxyz([d2r(DistData(19,:))',d2r(DistData(18,:))',DistData(18,:)'*0],avp_LBL_DR(1,7:9)');
dxyzdr=pos2dxyz([avp_dr_octans_lbl(:,7:8),avp_dr_octans_lbl(:,9)*0]);
dxyzdr1=pos2dxyz([avp_dr_compass_lbl(:,7:8),avp_dr_compass_lbl(:,9)*0]);
plot(dxyzship(:,1), dxyzship(:,2));
hold on, plot(dxyzdr(:,1), dxyzdr(:,2));
hold on, plot(dxyzdr1(:,1), dxyzdr1(:,2));
hold on, plot(dxyz(:,1), dxyz(:,2)); xygo('est', 'nth');
hold on, plot(0, 0, '*');
axis([-200,1000,-1000,200])
axis equal 
legend('LBL-raw','DR(OCTANS)','DR(compass)','LBL/DR(OCTANS)')
% 画误差
subplot 122
err1=RCompu([d2r(DistData(19,:))',d2r(DistData(18,:))',DistData(18,:)'*0], ...
    [avp_LBL_DR(:,7:8),avp_LBL_DR(:,9)*0]);
err2=RCompu([avp_LBL_DR(:,7:8),avp_LBL_DR(:,9)*0], ...
    [avp_dr_octans_lbl(:,7:8),avp_dr_octans_lbl(:,9)*0]);
err3=RCompu([avp_LBL_DR(:,7:8),avp_LBL_DR(:,9)*0], ...
    [avp_dr_compass_lbl(:,7:8),avp_dr_compass_lbl(:,9)*0]);
plot(tt_lbl,err1,tt_lbl,err2,tt_lbl,err3)
axis([tt_lbl(1) tt_lbl(end) 0 60])
xygo('t/s','error/m')
legend('LBL','DR(OCTANS)','DR(compass)','Location','best')
% print(gcf, 'New Folder\试验预处理-reference-cmp.png', '-dpng','-r600');
% print(gcf, 'C:\Users\23764\OneDrive\文档\LATEX\距离辅助航位推算总报告（2024）\picture\试验预处理-reference-cmp.png', '-dpng','-r600');
%% 信标轨迹对比图
% 长基线的原始轨迹
avp_raw=[zeros(length(DistData(19,:)),6),d2r(DistData(19,:)'),d2r(DistData(18,:)'),-depther];
close all
myfigurestartup(3,3,'paper')
dxyz = pos2dxyz([avp_raw(:,7:8),avp_raw(:,9)]);
dxyzship = pos2dxyz([d2r(LatShipPIXOG)',d2r(LonShipPIXOG)',LatShipPIXOG'*0],avp_raw(1,7:9)');
plot(0, 0, 'rp');
hold on, plot(dxyz(:,1), dxyz(:,2)); xygo('est', 'nth');
hold on, plot(dxyzship(1,1), dxyzship(1,2),'*');
hold on, plot(dxyzship(:,1), dxyzship(:,2));
for i=1:4
BCN1=pos2dxyz(BCN{i},avp_raw(1,7:9)');
hold on
plot(BCN1(1),BCN1(2),'*')
end
xx=xticks;
xticks(xx(1):500:xx(end))
yy=yticks;
yticks(yy(1):500:yy(end))
axis equal 
legend('start','trajectory','start','support ship','beacon1','beacon2','beacon3','beacon4','Location','best')
%%
figure
plot3(dxyz(:,1), dxyz(:,2),-depther)
xygo('est', 'nth');
zlabel('Up/m')
% print(gcf, 'New Folder\试验预处理-LBL-3D.png', '-dpng','-r600');
% print(gcf, 'C:\Users\23764\OneDrive\文档\LATEX\距离辅助航位推算总报告（2024）\picture\试验预处理-LBL-3D.png', '-dpng','-r600');

%% 长基线的定位结果 参考基准
LatHovCABIN=r2d(avp_m(ID,7))';
LonHovCABIN=r2d(avp_m(ID,8))';
DepthHovCABIN=-avp_m(ID,9)'; % 深度计的深度
% LBL融合航位推算OCTANS
LatHovCABIN1=r2d(avp_LBL_DR(ID,7))';
LonHovCABIN1=r2d(avp_LBL_DR(ID,8))';
DepthHovCABIN1=-avp_LBL_DR(ID,9)'; % 深度计的深度
[XUTMHovCABIN,YUTMHovCABIN] = ll2utm(LatHovCABIN,LonHovCABIN);
[XUTMHovCABIN1,YUTMHovCABIN1] = ll2utm(LatHovCABIN1,LonHovCABIN1);
%% 计算参考的水平距离
% (长基线定位结果 长基线融合航位推算结果)
Tran=[d2r(LatTranPIXOG)',d2r(LonTranPIXOG)',-DepthShipPIXOG'];
Hov=[d2r(LatHovCABIN)',d2r(LonHovCABIN)',-DepthHovCABIN'];
SlantRangePIXOGCABIN=RCompu(Tran,Hov)';
SlantRangeUTMPIXOGCABIN = sqrt( (XUTMTranPIXOG-XUTMHovCABIN).^2 + ... 
                (YUTMTranPIXOG-YUTMHovCABIN).^2 +(DepthShipPIXOG-DepthHovCABIN).^2);
HorizRangeUTMPIXOGCABIN = sqrt( (XUTMTranPIXOG-XUTMHovCABIN).^2 + ... 
                                (YUTMTranPIXOG-YUTMHovCABIN).^2 );

Hov1=[d2r(LatHovCABIN1)',d2r(LonHovCABIN1)',-DepthHovCABIN1'];
SlantRangePIXOGCABIN1=RCompu(Tran,Hov1)';
SlantRangeUTMPIXOGCABIN1 = sqrt( (XUTMTranPIXOG-XUTMHovCABIN1).^2 + ... 
                (YUTMTranPIXOG-YUTMHovCABIN1).^2 +(DepthShipPIXOG-DepthHovCABIN1).^2);
HorizRangeUTMPIXOGCABIN1 = sqrt( (XUTMTranPIXOG-XUTMHovCABIN1).^2 + ... 
                                (YUTMTranPIXOG-YUTMHovCABIN1).^2 );
%% 处理滞后的深度（根据传播时间）
close all
for ii=1:4
    ttt=8:8:tt_usbl(end);
    switch(ii)
        case 1
            ttt=ttt;
        case 4
            ttt=ttt-2;
        case 3
            ttt=ttt-1.5;
        case 2
            ttt=ttt-TimeH1PIXOG(2:end)-0.02; % 减去传播时间
    end
    tttt=[0:0.5:tt_usbl(end),ttt];
    tttt=sort(tttt);
    ki=1;
    index=[];
    for i=1:length(ttt)
        temp=find(tttt==ttt(i));
        if ~isempty(temp)
            index(ki)=temp(end);
            ki=ki+1;
        end
    end
    DepthHovCABINfull=-avp_m(ID(1):ID(end),9)';
    DepthHovCABINfull1=interp1(0:0.5:tt_usbl(end),DepthHovCABINfull,tttt,'linear');
    DepthHovCABINinterp=DepthHovCABINfull1([4,index]);
    clear ki ttt tttt temp DepthHovCABINfull DepthHovCABINfull1 index

    dep=DepthHovCABINinterp;
    % figure,plot(DepthHovCABIN),hold on,plot(DepthHovCABINinterp)
    Est_range= PropaTcmp(DepthShipPIXOG,dep,TimeH1PIXOG);

    % 平滑
    Est1=smooth(tt_usbl,Est_range(1,:),0.008,'rloess')';
    Est2=smooth(tt_usbl,Est_range(2,:),0.008,'rloess')';
    % 水平距离误差
    HorizErrPG(1,:)=Est_range(1,:)-HorizRangeUTMPIXOGCABIN1;
    HorizErrPG(2,:)=Est_range(2,:)-HorizRangeUTMPIXOGCABIN1;
    HorizErrPG(3,:)=Est1-HorizRangeUTMPIXOGCABIN1;
    HorizErrPG(4,:)=Est2-HorizRangeUTMPIXOGCABIN1;
    title11={'t','t-2s','t-1.5s','t-PropT'};
    if ii==1
        myfigurestartup(7,5,'paper'),
    end
    subplot(2,2,ii)
    plot(tt_usbl,HorizErrPG(3,:), ...
        tt_usbl,HorizErrPG(4,:))
    axis([tt_usbl(1) tt_usbl(end) -10 10])
    updownlabel;
    legend('Proposed-sm','ML-sm'),
    xygo('t/s','dHorizR/(m)')
    title(title11{ii})
    xlim([0 8800])
end
% print(gcf, 'New Folder\试验预处理-滞后时间选择.png', '-dpng','-r600');
%% 计算距离----------------------------------------------------
%% PIXOG PTSAG 计算水平距离
% 计算斜距
SlantRangeUTMPIXOGPTSAG = sqrt( (XUTMTranPIXOG-XUTMHovPTSAG).^2 + ... 
                (YUTMTranPIXOG-YUTMHovPTSAG).^2 +(DepthShipPIXOG-DepthHovPTSAG).^2);
% 直接计算水平距离
HorizRangeUTMPIXOGPTSAG = sqrt( (XUTMTranPIXOG-XUTMHovPTSAG).^2 + ... 
                                (YUTMTranPIXOG-YUTMHovPTSAG).^2 );
% 根据计算斜距和真实深度（传播时间处理后）计算的水平距离
HorizRangeUTMPIXOGFebck = sqrt(SlantRangeUTMPIXOGPTSAG.^2- ...
                        (DepthShipPIXOG-DepthHovCABINinterp).^2);
HorizRangeUTMPIXOGFebck1 = sqrt(SlantRangeUTMPIXOGPTSAG.^2- ...
                        (DepthShipPIXOG-DepthHovCABIN).^2);
% 斜距误差
SlantErrPP(1,:)=SlantRangeUTMPIXOGPTSAG-SlantRangeUTMPIXOGCABIN;
SlantErrPP(2,:)=SlantRangeUTMPIXOGPTSAG-SlantRangeUTMPIXOGCABIN1;

% 水平距离误差
HorizErrPP(1,:)=HorizRangeUTMPIXOGPTSAG-HorizRangeUTMPIXOGCABIN;
HorizErrPP(2,:)=HorizRangeUTMPIXOGFebck1-HorizRangeUTMPIXOGCABIN;
HorizErrPP(3,:)=HorizRangeUTMPIXOGFebck-HorizRangeUTMPIXOGCABIN;
% 绘图
myfigurestartup(7,3,'paper'),
subplot 121,plot(tt_usbl,SlantErrPP(1,:),tt_usbl,SlantErrPP(2,:))
legend('LBL REF','LBL/DR REF')
xygo('t/s','dSlantR/(m)')
xlim([0 8800])
subplot 122,plot(tt_usbl,HorizErrPP(1,:), ...
    tt_usbl,HorizErrPP(2,:), ...
    tt_usbl,HorizErrPP(3,:))
legend('PTSAG-CABIN','FEDBCK-CABIN','FEDBCK-offset-CABIN'),
xygo('t/s','dHorizR/(m)')
xlim([0 8800])
% print(gcf, 'New Folder\试验预处理-PTSAG-ERR.png', '-dpng','-r600');
%% PTSAX 计算水平距离
% 计算斜距
SlantRangePTSAX = sqrt(XForwardPTSAX.^2 + YStarboardPTSAX.^2 ...
                    +(DepthHovPTSAX-DepthShipPIXOG).^2);
% 计算水平距离
HorizRangePTSAX = sqrt(XForwardPTSAX.^2 + YStarboardPTSAX.^2);
HorizRangePTSAXfebck = sqrt(SlantRangePTSAX.^2-(DepthShipPIXOG-DepthHovCABINinterp).^2);
HorizRangePTSAXfebck1 = sqrt(SlantRangePTSAX.^2-(DepthShipPIXOG-DepthHovCABIN).^2);
% 斜距误差
SlantErrPX(1,:)=SlantRangePTSAX-SlantRangeUTMPIXOGCABIN;
SlantErrPX(2,:)=SlantRangePTSAX-SlantRangeUTMPIXOGCABIN1;

% 水平距离误差
HorizErrPX(1,:)=HorizRangePTSAX-HorizRangeUTMPIXOGCABIN;
HorizErrPX(2,:)=HorizRangePTSAXfebck1-HorizRangeUTMPIXOGCABIN;
HorizErrPX(3,:)=HorizRangePTSAXfebck-HorizRangeUTMPIXOGCABIN;

% 绘图
myfigurestartup(7,3,'paper'),
subplot 121,plot(tt_usbl,SlantErrPX(1,:),tt_usbl,SlantErrPX(2,:))
legend('LBL REF','LBL/DR REF')
xygo('t/s','dSlantR/(m)')
xlim([0 8800])
subplot 122,plot(tt_usbl,HorizErrPX(1,:), ...
    tt_usbl,HorizErrPX(2,:), ...
    tt_usbl,HorizErrPX(3,:))
legend('PTSAX-CABIN','FEDBCK-CABIN','FEDBCK-offset-CABIN'),
xygo('t/s','dHorizR/(m)')
xlim([0 8800])
% print(gcf, 'New Folder\试验预处理-PISAX-ERR.png', '-dpng','-r600');
%% PIXOG 传播时间计算水平距离
% close all
dep=DepthHovCABIN;
dep=DepthHovCABINinterp;
% figure,plot(DepthHovCABIN),hold on,plot(DepthHovCABINinterp)
Est_range= PropaTcmp(DepthShipPIXOG,dep,TimeH1PIXOG);

dhgt=DepthShipPIXOG-dep;
SlantRangePropaTfedbck1=sqrt(Est_range(1,:).^2+dhgt.^2);
SlantRangePropaTfedbck2=sqrt(Est_range(2,:).^2+dhgt.^2);
SlantRangePropaTfedbck3=sqrt(Est_range(3,:).^2+dhgt.^2);
SlantRangePropaTfedbck4=sqrt(Est_range(4,:).^2+dhgt.^2);

% 平滑
Est1=smooth(tt_usbl,Est_range(1,:),0.008,'rloess')';
Est2=smooth(tt_usbl,Est_range(2,:),0.008,'rloess')';
% figure,plot(tt_usbl,Est_range(1,:),tt_usbl,Est1)
% figure,plot(tt_usbl,Est_range(2,:),tt_usbl,Est2)

% 斜距误差
SlantErrPG(1,:)=SlantRangePropaTfedbck1-SlantRangeUTMPIXOGCABIN1;
SlantErrPG(2,:)=SlantRangePropaTfedbck2-SlantRangeUTMPIXOGCABIN1;
SlantErrPG(3,:)=SlantRangePropaTfedbck3-SlantRangeUTMPIXOGCABIN1;
SlantErrPG(4,:)=SlantRangePropaTfedbck4-SlantRangeUTMPIXOGCABIN1;

% 水平距离误差
HorizErrPG(1,:)=Est_range(1,:)-HorizRangeUTMPIXOGCABIN1;
HorizErrPG(2,:)=Est_range(2,:)-HorizRangeUTMPIXOGCABIN1;
HorizErrPG(3,:)=Est1-HorizRangeUTMPIXOGCABIN1;
HorizErrPG(4,:)=Est2-HorizRangeUTMPIXOGCABIN1;
rangemeas = [path,'range_meas.mat'];
save(rangemeas,'HorizRangeUTMPIXOGCABIN1','Est_range','TimeH1PIXOG')
% myfigurestartup(7,3,'prese'),
% subplot 121,
% plot(tt_usbl,HorizErrPG(1,:), ...
%     tt_usbl,HorizErrPG(2,:))
% axis([tt_usbl(1) tt_usbl(end) -8 8])
% updownlabel;
% legend('Proposed','ML'),
% xygo('t/s','dHorizR/(m)')
% xlim([0 8800])
myfigurestartup(3,3,'paper'),
plot(tt_usbl,HorizErrPG(3,:), ...
    tt_usbl,HorizErrPG(4,:))
axis([tt_usbl(1) tt_usbl(end) -8 8])
updownlabel;
legend('Proposed-sm','ML-sm'),
xygo('t/s','dHorizR/(m)')
xlim([0 8800])
% print(gcf, 'New Folder\试验预处理-PIXOG-ERR.png', '-dpng','-r600');
%% 保存数据
HorizRangePropaT = Est_range(1:4,:);% 四类传播时间计算的（提出+ML+算术平均+几何平均）
HorizRangePropaTsm=[Est1;Est2];% 平滑后的
LatLonDepTran=[LatTranPIXOG;LonTranPIXOG;-DepthShipPIXOG;tt_usbl];
LatLonDepShip=[LatShipPIXOG;LonShipPIXOG;-DepthShipPIXOG;tt_usbl];
LatLonDepHov=[LatHovPTSAG;LonHovPTSAG;-DepthHovPTSAX;tt_usbl];
deepsea_data = [path,'output\','deep-sea.mat'];
save(deepsea_data, ...
    'avp_m', 'compass', 'octans', 'vxy', 'depther', ...
    'BCN', 'RNG', 'RNG_raw', 'RNG1', ...
    'LatLonShipCabin', ...
    'HorizRangePropaT', 'HorizRangePropaTsm', ...
    'HorizRangePTSAXfebck', ...
    'HorizRangeUTMPIXOGFebck', ...
    'LatLonDepTran', 'LatLonDepHov', 'LatLonDepShip', ...
    'HorizRangeUTMPIXOGCABIN', ...
    'avp_USBL_DR', 'avp_LBL_DR', ...
    'avp_dr_octans_lbl', 'avp_dr_octans_usbl', ...
    'avp_lbl_raw', 'avp_usbl_raw');
