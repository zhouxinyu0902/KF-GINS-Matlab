%% 将两个信标的距离带入,  双信标辅助
clear
close all
glvs
load('data_1\DataNeed.mat')
t_lbl=avp_LBL_DR(:,end)';
t_usbl=avp_ref(:,end)';
LenLBL=length(t_lbl);
LenUSBL=length(t_usbl);
%% 距离辅助double
% close all
% 参数设置
wrng=6;
web=d2r(0.5);
tt=t_lbl;
% compass=denoise(compass,3,0.2);
inte=[1,2;1,3;1,4;2,3;2,4;3,4]; % 探究不同组合的固定信标
for ii=1:6
id1=inte(ii,1);
id2=inte(ii,2);
pos0=avp_ref(1,7:9)';
beacon1=BCN{id1};
beacon2=BCN{id2};
yaw=compass;
dr = mydr('init',pos0,[0.1;0.1;0.1],0.5);

x0=[0.001;d2r(0.5);0.1/glv.Re;0.1/glv.Re]*0.1;
dx0=[0.001;d2r(0.5);0.1/glv.Re;0.1/glv.Re];

x0=[0.001;d2r(0.5);0.1/glv.Re;0.1/glv.Re]*0.00;
dx0=[0.002;d2r(0.5);0.1/glv.Re;0.1/glv.Re];

kf = myekf('init',0.5, x0, dx0 ,[0,web,0,0],[wrng,wrng]);% 1、改测量噪声矢量
[avp_range,avp_dr,xkpk,kk_1]=prealloc(length(avp_ref),10,10,9,5);
ki=1;
for i=1:length(yaw)
    t=tt(i);
    dr = mydr('update',dr,-depther(i),yaw(i,3),vxy(i,1:2));
    avp_dr(i,:)=[dr.avp',t];% 航位推算
    kf = myekf('fk', kf, dr);
    kf = myekf('algo',kf,'T');
    if mod(t,8)==0
        if size(beacon1,1)==1
            dr.beacon1=beacon1;
            dr.beacon2=beacon2;
        else
            dr.beacon1=beacon1(ki,:);
            dr.beacon2=beacon2(ki,:);
        end
        r=zeros(2,1);
        r(1,1)=RNG{id1}(ki);% 测量值
        r(2,1)=RNG{id2}(ki);
        kf.r_dr(1,1)=sqrt(RCompu(dr.pos',dr.beacon1)^2-(-depther(i)-dr.beacon1(3))^2); % 计算值
        kf.r_dr(2,1)=sqrt(RCompu(dr.pos',dr.beacon2)^2-(-depther(i)-dr.beacon2(3))^2);
        kf.yk=kf.r_dr-r;
        kf = myekf('hk',kf, dr,'2range');
        kf = myekf('algo',kf,'M');
        kk_1(ki,1:4)=kf.xk;
        kk_1(ki,5)=t;
        avp_range(ki,:) = [dr.avp', t];
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';ki=ki+1;
    end
end
avp_range(ki:end,:) = [];  xkpk(ki:end,:) = [];
kk_1(ki:end,:) = [];
avp_range(:,7)=avp_range(:,7)-kk_1(:,3);
avp_range(:,8)=avp_range(:,8)-kk_1(:,4);
avp_RNG1{ii+10}=avp_range;
KK{ii+10}=kk_1;
end
% %% 单个绘图
% myfigurestartup(3,3,'paper'),
% plot(avp_ref(:,end),RCompu(avp_ref(:,7:9),avp_dr(ID,7:9)),'m--')
% hold on
% plot(avp_range(:,end),RCompu(avp_ref(:,7:9),avp_RNG1{12}(:,7:9)));
% xygo('t/s','error/(m)')
% axis([0 8800 0 60])
%% 组合绘图
ID=1:16:length(t_lbl);
for i=1:6
    if i==1
        myfigurestartup(5,3,'paper'),
        plot(avp_ref(:,end),RCompu(avp_ref(:,7:9),avp_dr(ID,7:9)),'m--')
    end
    hold on
    plot(avp_range(:,end),RCompu(avp_ref(:,7:9),avp_RNG1{i+10}(:,7:9)));
end
legend('dr','beacon1&2','beacon1&3','beacon1&4','beacon2&3','beacon2&4','beacon3&4','Location','best')
xygo('t/s','error/(m)')
axis([0 8800 0 60])
% print(gcf, 'New Folder\试验-双固定信标组合的径向误差.png', '-dpng','-r600');
% print(gcf, 'C:\Users\智能计算\OneDrive\文档\LATEX\距离辅助航位推算总报告（2024）\picture\试验-双固定信标组合的径向误差.png', '-dpng','-r600');
%%
% close all
% myfigurestartup(5,5,'paper'),
% lon_lat_err(avp_LBL_DR,{'range'},avp_dr,avp_RNG1([1,2,8]));
% print(gcf, 'paper\lonlaterr_试验.svg', '-dsvg');
dr_err=avpcmp(avp_dr,avp_LBL_DR);
myfigurestartup(5,5,'paper'),
xk_plot(0.002,d2r(0.5),dr_err,{'beacon1&2','beacon1&3'},KK([11,12]),0)
% print(gcf, 'New Folder\试验-双固定信标组合的EKF(1&2,1&3).png', '-dpng','-r600');
