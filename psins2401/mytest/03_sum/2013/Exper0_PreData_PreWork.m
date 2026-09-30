%% 将一些数据准备好，处理一些数据，合成距离信息
% 信标位置距离计算好 
% 参考位置准备好 
% 传感器数据准备好
clear
% close all
glvs
path = 'D:\Github\KF-GINS-Matlab\data\psins\data_1\output\';
% load('03_sum\data_1\deep-sea.mat')
load([path,'deep-sea_optimized.mat'])
t_lbl = avp_m(:,end)';
t_usbl = LatLonDepHov(end,:);
LenLBL = length(t_lbl);
LenUSBL = length(t_usbl);
wrng = 6;
ID = 1:16:16*LenUSBL;
avp_ref = avp_LBL_DR(ID,:);
% insplot(avp_LBL_DR)

%% 纯航位推算
tt = t_lbl;
pos0 = avp_ref(1,7:9)';
dr1 = mydr('init',pos0,[0.1;0.1;0.1],0.5);
for i = 1:length(compass)
    t = tt(i);
    dr1 = mydr('update',dr1,-depther(i),compass(i,3),vxy(i,1:2));
    avp_dr(i,:) = [dr1.avp',t];% 航位推算
end
myfigurestartup(3,3,'paper');
plot(avp_ref(:,end),RCompu(avp_ref(:,7:9),avp_dr(ID,7:9)),'m')

%% 计算静止信标的三类距离
% RNG1使用长基线定位结果计算，RNG原始数据平滑，RNG_raw原始数据，RNG2参考位置计算的距离（需要加噪声）
for i=1:4
    % 计算参考水平距离+白噪声
    RNG2{i}=RCompu(avp_LBL_DR(:,7:9),BCN{i});% 参考位置计算的距离
    cmp{i}=sqrt(RNG2{i}.^2-(-depther-BCN{i}(3)).^2);% 参考位置计算的水平距离
    RNG2{i}=cmp{i}+normrnd(0,wrng,length(RNG2{i}),1);% 加噪声
    RNG2{i}=RNG2{i}(ID);% 挑选8s时间间隔
    % 原始信标声学距离结合深度计进行计算
    RNG{i}=sqrt(RNG{i}.^2-(-depther-BCN{i}(3)).^2);% 四个长基线信标的距离
    RNG{i}=RNG{i}(ID);
    % RNG1是根据定位结果反算的距离，然后再计算水平距离
    RNG1{i}=sqrt(RNG1{i}.^2-(-depther-BCN{i}(3)).^2);
    RNG1{i}=RNG1{i}(ID);     
end
% 设置的固定的一个理想信标5
%% 信标5，设置的一个理想信标
BCN{5} = dxyz2pos([1000,1000,-20],avp_ref(1,7:9)'); 
SlantR = RCompu(avp_ref(:,7:9),BCN{5});
dhgt = avp_ref(:,9)-BCN{5}(3);
HoriR_cmp = (sqrt(SlantR.^2-dhgt.^2)+normrnd(0,wrng,LenUSBL,1))';
RNG{5} = HoriR_cmp;
clear SlantR dhgt HoriR_cmp

myfigurestartup(5,5,'prese');
plot(avp_ref(:,8),avp_ref(:,7))
hold on
plot(BCN{5}(2),BCN{5}(1),'*')
%% 移动信标
% 母船位置，用于计算位移差，dllh可用于仿真
beacon = [LatLonShipCabin(1:2,:)',zeros(length(LatLonShipCabin),1)];
beacon = beacon(ID,:);
% 根据母船位置和航行器参考位置计算出dllh
dllh = beacon(:,1:2)-avp_ref(:,7:8);
save 03_sum\data_1\dllh.mat dllh

myfigurestartup(5,5,'prese');
plot(avp_ref(:,8),avp_ref(:,7))
hold on
plot(beacon(:,2),beacon(:,1),'*','MarkerSize',2)
%%
% 移动信标6，信标6-10都是移动信标的水平距离
BCN{6} = beacon; 
SlantR = RCompu(avp_ref(:,7:9),BCN{6});
dhgt = avp_ref(:,9)-BCN{6}(:,3);
HoriR_cmp = (sqrt(SlantR.^2-dhgt.^2)+normrnd(0,wrng,LenUSBL,1))';
RNG{6} = HoriR_cmp;
clear beacon SlantR dhgt HoriR_cmp
% 使用USBL导出的换能器位置和理想计算的距离
BCN{7}=[d2r([LatLonDepTran(1,:)',LatLonDepTran(2,:)']),...
    LatLonDepTran(3,:)']; 
SlantR=RCompu(avp_ref(:,7:9),BCN{7});
dhgt=avp_ref(:,9)-BCN{7}(:,3);
HoriR_cmp=(sqrt(SlantR.^2-dhgt.^2)+normrnd(0,wrng,LenUSBL,1))';
RNG{7} = HoriR_cmp;

% BCN{8} = BCN{7}; 
% BCN{9} = BCN{7}; 
% BCN{10} = BCN{7}; 
% % 绘图
% RNG{8}=HorizRangeUTMPIXOGFebck;% 使用USBL导出的换能器位置和三类方法计算的距离PTSAG
% RNG{9}=HorizRangePTSAXfebck;% 使用USBL导出的换能器位置和三类方法计算的距离PTSAX
% RNG{10}=HorizRangePropaTsm(1,:);% 使用USBL导出的换能器位置和三类方法计算的距离PIXOG

% 信标2和4效果不行，将其纬度向下移，11~14
for i=[2,4]
    BCN{10+i}=dxyz2pos([-1000,0,0],BCN{i}');
    RNG{10+i}=RCompu(avp_ref(:,7:9),BCN{10+i})+normrnd(0,6,length(t_usbl));
end
for i=[1,3]
    BCN{10+i}=dxyz2pos([0,-1000,0],BCN{i}');
    RNG{10+i}=RCompu(avp_ref(:,7:9),BCN{10+i})+normrnd(0,6,length(t_usbl));
end

% save 03_sum\data_1\DataNeed_1.mat avp_LBL_DR avp_ref LenLBL LenUSBL avp_dr...
%     RNG RNG1 RNG2 BCN...
%     vxy compass depther wrng
% save 03_sum\data_1\DataNeed_1.mat avp_LBL_DR avp_ref LenLBL LenUSBL avp_dr...
%     RNG RNG1 RNG2 BCN...
%     vxy compass depther wrng
 save 03_sum\data_1\DataNeed_optimized.mat avp_LBL_DR avp_ref LenLBL LenUSBL avp_dr...
    RNG RNG1 RNG2 BCN...
    vxy compass depther wrng
%% 挪完信标后的绘图
myfigurestartup(3,3,'paper');
plot(r2d(avp_LBL_DR(:,8)),r2d(avp_LBL_DR(:,7)));
hold on
plot(r2d(avp_LBL_DR(1,8)),r2d(avp_LBL_DR(1,7)),'.')
for i=1:4
    hold on
    plot(r2d(BCN{i}(2)),r2d(BCN{i}(1)),'*')
end
for i=1:4
    hold on
    plot(r2d(BCN{10+i}(2)),r2d(BCN{10+i}(1)),'o')
end
for i=1:4
% 创建起点和终点坐标
startPoint = [r2d(BCN{i}(2)),r2d(BCN{i}(1))];  % [x_start, y_start]
endPoint = [r2d(BCN{10+i}(2)),r2d(BCN{10+i}(1))];    % [x_end, y_end]
% 计算向量方向
deltaX = endPoint(1) - startPoint(1);
deltaY = endPoint(2) - startPoint(2);
% 绘制箭头
quiver(startPoint(1), startPoint(2), deltaX, deltaY, 0,...
       'Color', 'b', 'LineWidth', 1, 'MaxHeadSize', 0.7,'LineStyle','--')
end
legend('trajectory','start','beacon1','beacon2','beacon3','beacon4','Location','best')
% print(gcf, 'New Folder\试验-静止信标挪地方.png', '-dpng','-r600');
% print(gcf, 'C:\Users\智能计算\OneDrive\文档\LATEX\距离辅助航位推算总报告（2024）\picture\试验-静止信标挪地方.png', '-dpng','-r600');
%% 对速度，姿态角进行绘图
myfigurestartup(5,3,'zxy');
plot(t_lbl,vxy(:,2))
hold on
plot(t_lbl,vxy(:,1))
legend('横向','前向速度')
myfigurestartup(5,3,'zxy');
subplot 121
plot(t_lbl,compass(:,1))
hold on
plot(t_lbl,octans(:,1))
legend('compass-pitch','compass-roll')
subplot 122
plot(t_lbl,compass(:,2))
hold on
plot(t_lbl,octans(:,2))
legend('octans-pitch','octans-roll')
%% 画图-实际信标轨迹
myfigurestartup(3,3,'paper');
plot(r2d(avp_LBL_DR(:,8)),r2d(avp_LBL_DR(:,7)));
hold on
plot(r2d(avp_LBL_DR(1,8)),r2d(avp_LBL_DR(1,7)),'.')
for i=1:5
hold on
plot(r2d(BCN{i}(2)),r2d(BCN{i}(1)),'*')
end
hold on
plot(LatLonDepTran(2,1),LatLonDepTran(1,1),'.')
hold on
plot(LatLonDepTran(2,:),LatLonDepTran(1,:))
axis equal;
legend('trajectory','start','beacon1','beacon2','beacon3','beacon4','ideal-beacon5', ...
    'start','moving beacon','Location','best')
xygo('lat','lon')
DeciPoin(3,3)
% print(gcf, 'New Folder\试验-实际信标轨迹.png', '-dpng','-r600');
%% 3D图
myfigurestartup(3,3,'paper');
plot3(r2d(avp_LBL_DR(:,8)),r2d(avp_LBL_DR(:,7)),avp_LBL_DR(:,9));
hold on
plot3(r2d(avp_LBL_DR(1,8)),r2d(avp_LBL_DR(1,7)),avp_LBL_DR(1,9),'.')
for i=1:5
hold on
plot3(r2d(BCN{i}(2)),r2d(BCN{i}(1)),BCN{i}(3),'*')
end
hold on
plot3(LatLonDepTran(2,1),LatLonDepTran(1,1),LatLonDepTran(3,1),'.')
hold on
plot3(LatLonDepTran(2,:),LatLonDepTran(1,:),LatLonDepTran(3,:))
legend('trajectory','start','beacon1','beacon2','beacon3','beacon4','ideal-beacon5', ...
    'start','moving beacon','Location','best')
xygo('lat','lon')
DeciPoin(3,3)
%% 静止信标--LBL信标的距离误差对比
myfigurestartup(5,5,'paper');
for i=1:4
    subplot(2,2,i),plot(t_lbl(ID),RNG{i}-cmp{i}(ID),t_lbl(ID),RNG1{i}-cmp{i}(ID))
    % subplot(2,2,i),plot(t_lbl(ID),RNG2{i},t_lbl(ID),RNG{i},t_lbl(ID),RNG1{i})
    axis([0,8800,-20,20]),xygo('t/s','range/m'),
    legend('LBL-raw-data','LBL-Result-compu')
    title(sprintf('beacon %d',i))
end
% print(gcf, 'New Folder\试验-LBL信标的距离误差对比.png', '-dpng','-r600');

myfigurestartup(5,5,'paper');
for i=1:4
    subplot(2,2,i),plot(t_lbl(ID),RNG{i}-cmp{i}(ID))
    % subplot(2,2,i),plot(t_lbl(ID),RNG2{i},t_lbl(ID),RNG{i},t_lbl(ID),RNG1{i})
    axis([0,8800,-20,20]),xygo('t/s','range/m'),
    legend('LBL-raw-data')
    title(sprintf('beacon %d',i))
end
% print(gcf, 'New Folder\试验-LBL信标的距离误差.png', '-dpng','-r600');
%% 移动信标--三类方法与理想白噪声的距离差
cmp=sqrt(SlantR.^2-dhgt.^2)';
ERR(1,:)=RNG{8}-cmp;
ERR(2,:)=RNG{9}-cmp;
ERR(3,:)=RNG{10}-cmp;
ERR(4,:)=RNG{7}-cmp;
myfigurestartup(3,3,'paper');
plot(t_usbl,ERR(4,:),t_usbl,ERR(1,:), ...
    t_usbl,ERR(2,:),t_usbl,ERR(3,:)),
axis([t_usbl(1) t_usbl(end) -20 20])
xygo('t/s','HorizRange-error/ ( m )')
% updownlabel;
% legend('REF','PTSAG','PTSAX','PropaT-proposed')
% print(gcf, 'New Folder\试验-三类方法与理想白噪声的距离差.png', '-dpng','-r600');

myfigurestartup(3,3,'paper');
plot(t_usbl,ERR(1,:), ...
    t_usbl,ERR(2,:),t_usbl,ERR(3,:)),
axis([t_usbl(1) t_usbl(end) -20 20])
xygo('t/s','HorizRange-error/ ( m )')
% updownlabel;
% legend('PTSAG','PTSAX','PropaT-proposed')
% print(gcf, 'New Folder\试验-三类方法计算的误差.png', '-dpng','-r600');