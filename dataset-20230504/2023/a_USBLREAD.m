%% 超短基线的数据，用于读取信标位置结果
%% 长基线信标的位置(来自超短基线的定位结果)
close all
clear
filename={'USBL-BOX-R-20230429-230048_PTSAG.log',...
          'USBL-BOX-R-20230430-030644_PTSAG.log',...
          'USBL-BOX-R-20230501-033958_PTSAG.log',...
          'USBL-BOX-R-20230501-081206_PTSAG.log'};
for i=1:4
fid = fopen(filename{i},'rt');
PTSAG_rw = fscanf(fid,'$PTSAG,#%d,%2d%2d%f,%d,%d,%d,%d,%2d%f,%c,%3d%f,%c,%X,%f,%d,%f*%X\n',[19,inf]);                
fclose(fid);

PTSAG=PTSAG_rw(:,PTSAG_rw(8,:)~=0); % 定位的信标数据
depth=PTSAG(16,:);
index=find(depth<4180|depth>4230);
PTSAG(:,index)=[];
depth(index)=[];

PTSAG11=PTSAG_rw(:,PTSAG_rw(8,:)==0);% 超短基线的数据

% 处理PTSAG的纬度经度，放在9和12行
Latbea = PTSAG(9,:) + PTSAG(10,:)/60 ;
Ins = find( PTSAG(11, :)==83 ); %'S'
Latbea(Ins) = -Latbea(Ins); 
Lonbea = PTSAG(12,:) + PTSAG(13,:)/60 ;
Iew = find( PTSAG(14,:)==87 ); %'W'
Lonbea(Iew) = -Lonbea(Iew); 

Mealonbea=mean(Lonbea);
Mealatbea=mean(Latbea);
Meadep=mean(depth);
figure
subplot 121,
plot(Lonbea,Latbea,'.')

% USBL的位置
Lat = PTSAG11(9,:) + PTSAG11(10,:)/60 ;
Ins = find( PTSAG11(11, :)==83 ); %'S'
Lat(Ins) = -Lat(Ins); 
Lon = PTSAG11(12,:) + PTSAG11(13,:)/60 ;
Iew = find( PTSAG11(14,:)==87 ); %'W'
Lon(Iew) = -Lon(Iew); 
hold on
plot(Lon,Lat,'.')
subplot 122,
plot(depth) 

beacon{i}=[Mealatbea,Mealonbea,Meadep];
end
%% 顺序 93 91 92 90
myfigurestartup(5,5,'prese')
for i=1:4
plot(beacon{i}(2),beacon{i}(1),'*')
hold on
end
xygo('lon','lat')
bcn{1}=beacon{4};
bcn{2}=beacon{2};
bcn{3}=beacon{1};
bcn{4}=beacon{3};

%% 超短基线的定位结果，超短基线加了8h
fid = fopen('USBL-BOX-R-20230504-061132_SEL_PTSAG.log','rt');
PTSAG_rw1 = fscanf(fid,'$PTSAG,#%d,%2d%2d%f,%d,%d,%d,%d,%2d%f,%c,%3d%f,%c,%X,%f,%d,%f*%X\n',[19,inf]);                
fclose(fid);

fid = fopen('USBL-BOX-R-20230504-082710_SEL_PTSAG.log','rt');
PTSAG_rw2 = fscanf(fid,'$PTSAG,#%d,%2d%2d%f,%d,%d,%d,%d,%2d%f,%c,%3d%f,%c,%X,%f,%d,%f*%X\n',[19,inf]);                
fclose(fid);

PTSAG=[PTSAG_rw1(:,8192:end),PTSAG_rw2(:,1:3503)];

% PTSAG=PTSAG(:,PTSAG(8,:)==0); % 母船位置
% Lat = PTSAG(9,:) + PTSAG(10,:)/60 ;
% Ins = find( PTSAG(11, :)==83 ); %'S'
% Lat(Ins) = -Lat(Ins); 
% Lon = PTSAG(12,:) + PTSAG(13,:)/60 ;
% Iew = find( PTSAG(14,:)==87 ); %'W'
% Lon(Iew) = -Lon(Iew); 
% figure
% plot(Lon,Lat,'.')

PTSAGpos=PTSAG(:,PTSAG(8,:)~=0); % 定位的信标数据
[PTSAGpos(10,:),PTSAGpos(9,:)]=ProcPos(PTSAGpos,9,12);
PTSAGpos(11:14,:)=[];
timeUSBL=(PTSAGpos(2,:))*3600+PTSAGpos(3,:)*60+PTSAGpos(4,:);
figure
plot(PTSAGpos(10,:),PTSAGpos(9,:),'.')
figure
subplot 121
plot(timeUSBL,PTSAGpos(10,:))
subplot 122
plot(timeUSBL,PTSAGpos(9,:))

% 绘图信标间距
BCNddm=[bcn{1};bcn{2};bcn{3};bcn{4}];
BCNrrm=BCNddm;
BCNrrm(:,1:2)=d2r(BCNrrm(:,1:2));
trjddm=[PTSAGpos(9,:)',PTSAGpos(10,:)',PTSAGpos(12,:)'];
plot_beacon_distances_with_custom_func(BCNddm,BCNrrm,trjddm)


save 2023\matData\USBL.mat bcn PTSAGpos

function [lon,lat]=ProcPos(pos,row1,row2)
lat = pos(row1,:) + pos(row1+1,:)/60;
Ins = find( pos(row1+2, :)==83 ); %'S'=83,'N'=78
lat(Ins) = -lat(Ins);
lon = pos(row2,:) + pos(row2+1,:)/60;
Iew = find( pos(row2+2,:)==87 ); %'W'=87,'E'=69
lon(Iew) = -lon(Iew);
end