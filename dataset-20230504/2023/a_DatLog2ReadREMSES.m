%% 读取长基线的数据
clear
% 原始数据:换能器位置（包括AUV和信标）
fid = fopen('log2-20230504-0017_SEL.dat','rt');
POSrw= fscanf(fid,'$PIXOR,DBG,POS,%2d%2d%f,%d,%d,%d,%d,%2d%f,%c,%3d%f,%c,%f,%d,%f,%f,%f*%X\n',[19,inf]);
% $PIXOR,DBG,POS,001703.491,04,05,2023,0,1549.2404860,N,11508.8309863,E,4129.370,2,0000.189,0000.294,0000.041*7A
fclose(fid);
% fid = fopen('log2-20230504-0124_SEL.dat','rt');
% POS1rw= fscanf(fid,'$PIXOR,DBG,POS,%2d%2d%f,%d,%d,%d,%d,%2d%2d%f,%c,%3d%2d%f,%c,%f,%d,%f,%f,%f*%X\n',[21,inf]);
% % $PIXOR,DBG,POS,001703.491,04,05,2023,0,1549.2404860,N,11508.8309863,E,4129.370,2,0000.189,0000.294,0000.041*7A
% fclose(fid);

% Vehicle position sent by an external sensor.（外部传感器的AUV位置）
fid = fopen('log2-20230504-0017_SEL_POSIN.dat','rt');
POSex= fscanf(fid,'$PIXOR,POS_IN,%2d%2d%f,%d,%d,%d,%d,%d,%2d%f,%c,%3d%f,%c,%f,%f*%X\n',[17,inf]);
% $PIXOR,POS_IN,001702.705,04,05,2023,1,0,1549.2380676,N,11508.8344018,E,0001.730,0003.010*4B
fclose(fid);

% 距离信息(使用的是REMSES LF)
fid = fopen('log2-20230504-0017_SEL_DIST.dat','rt');
DISTrw= fscanf(fid,'$PIXOR,DBG,DIST,%2d%2d%f,%d,%d,%d,%d,%f,%f,%d,%f,%f,%f*%X\n',[14,inf]);
fclose(fid);
% fid = fopen('log2-20230504-0124_SEL_DIST.dat','rt');
% DIST1rw= fscanf(fid,'$PIXOR,DBG,DIST,%2d%2d%f,%d,%d,%d,%d,%f,%f,%d,%f,%f,%f*%X\n',[14,inf]);
% fclose(fid);
% $PIXOR,DBG,DIST,001702.705,04,05,2023,90,1917.985（计算距离）,0000.598,
% 1,1964.278（测量距离）,0006.028（距离测量残差）,0021.868*07

fid = fopen('log2-20230504-0017_SEL_TTR.dat','rt');
TTRrw= fscanf(fid,'$PIXOR,TTR_IN,%2d%2d%f,%d,%d,%d,%d,%d,%d,%f,%f,%f*%X\n',[13,inf]);
% $PIXOR,TTR_IN,001705.757,04,05,2023,0,90,402(Type signal ID),2.5862888,0.0039494,0*5F
fclose(fid);
%% 位置信息
close all

POS_beacon0=POSrw(:,POSrw(7,:)==90);
POS_beacon1=POSrw(:,POSrw(7,:)==91);
POS_beacon2=POSrw(:,POSrw(7,:)==92);
POS_beacon3=POSrw(:,POSrw(7,:)==93);
POS_auv=POSrw(:,POSrw(7,:)==0);

load('2023\matData\USBL.mat')
timeUSBL=PTSAGpos(2,:)*3600+PTSAGpos(3,:)*60+PTSAGpos(4,:);

[Lon,Lat]=ProcPos(POS_auv,8,11); % 长基线的位置
timeLBL=POS_auv(1,:)*3600+POS_auv(2,:)*60+POS_auv(3,:);
depthShip=POS_auv(14,:);
stdDevLat=POS_auv(16,:);
stdDevLon=POS_auv(17,:);
stdDevDep=POS_auv(18,:);
[Lonl,Latl]=ProcPos(POSex,9,12); % 外部位置

[Lon0,Lat0]=ProcPos(POS_beacon0,8,11); % 信标位置
[Lon1,Lat1]=ProcPos(POS_beacon1,8,11);
[Lon2,Lat2]=ProcPos(POS_beacon2,8,11);
[Lon3,Lat3]=ProcPos(POS_beacon3,8,11);
bea{1}=[Lat0(1),Lon0(1)];
bea{2}=[Lat1(1),Lon1(1)];
bea{3}=[Lat2(1),Lon2(1)];
bea{4}=[Lat3(1),Lon3(1)];

timebea{1}=POS_beacon0(1,:)*3600+POS_beacon0(2,:)*60+POS_beacon0(3,:);
timebea{2}=POS_beacon1(1,:)*3600+POS_beacon1(2,:)*60+POS_beacon1(3,:);
timebea{3}=POS_beacon2(1,:)*3600+POS_beacon2(2,:)*60+POS_beacon2(3,:);
timebea{4}=POS_beacon3(1,:)*3600+POS_beacon3(2,:)*60+POS_beacon3(3,:);
%%
myfigurestartup(12,5,'prese')
plot(Lon,Lat,'.',Lonl,Latl,'.')
hold on

plot(PTSAGpos(10,:),PTSAGpos(9,:))
hold on
legend('LBL','extern','USBL')
plot(Lon0,Lat0,'*',Lon1,Lat1,'*',Lon2,Lat2,'*',Lon3,Lat3,'*')

xygo('lon','lat')
title('LBL vs USBL vs NAV result')

for i=1:4
    hold on
    plot(bcn{i}(2),bcn{i}(1),'*')
end
xygo('lon','lat')
title('beacon position from USBL')

myfigurestartup(12,5,'prese')
subplot 121,
plot(timeLBL,depthShip)
ConvertXAxisTime;
subplot 122,
plot(timeLBL,stdDevLat,timeLBL,stdDevLon)
ConvertXAxisTime;
%%
figure
plot(Lon,Lat,'.')
axis equal
%% 距离信息
beaconID=[90,91,92,93];
DIS=cell(1,4);
TTR=cell(1,4);
for i=1:4
    DIS{i}=DISTrw(:,DISTrw(7,:)==beaconID(i));
    TTR{i}=TTRrw(:,TTRrw(8,:)==beaconID(i));
end
myfigurestartup(10,10,'prese')
for i=1:4
    subplot(2,2,i)
    plot(DIS{i}(8,:)),hold on% 计算距离
    plot(DIS{i}(11,:))% 测量距离
    legend('computed dist','measured dist')
end
myfigurestartup(10,10,'prese')
for i=1:4
    subplot(2,2,i)
    plot(TTR{i}(10,:))
end
[POS_auv(11,:),POS_auv(8,:)]=ProcPos(POS_auv,8,11);
POS_auv([9,10,12,13],:)=[];
posLBL=POS_auv;
posUSBL=PTSAGpos;
% ProcPos(POS_auv,8,11);
save 2023\matData\LBL.mat TTR DIS POS_auv timebea timeLBL
save 2023\matData\LBL&USBL.mat TTR DIS posUSBL posLBL timebea timeLBL timeUSBL 
%% 函数
function [lon,lat]=ProcPos(pos,row1,row2)
lat = pos(row1,:) + pos(row1+1,:)/60;
Ins = find( pos(row1+2, :)==83 ); %'S'=83,'N'=78
lat(Ins) = -lat(Ins);
lon = pos(row2,:) + pos(row2+1,:)/60;
Iew = find( pos(row2+2,:)==87 ); %'W'=87,'E'=69
lon(Iew) = -lon(Iew);
end