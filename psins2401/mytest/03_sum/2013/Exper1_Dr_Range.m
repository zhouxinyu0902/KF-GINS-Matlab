%% 距离辅助航位推算
% 静止和移动信标对比；
% 单个新标的效果查看
% 将信标1 3 移动信标拿出来单独分析
clear
close all
glvs
% load('data_1\DataNeed_1.mat')% 在prework里面重构的一些数据,20260702生成的
% load('data_1\DataNeed.mat')% 在prework里面重构的一些数据
load('data_1\DataNeed_optimized.mat')% 在prework里面重构的一些数据
t_lbl = avp_LBL_DR(:,end)';
t_usbl = avp_ref(:,end)';
LenLBL = length(t_lbl);
LenUSBL = length(t_usbl);
rng(1)
%% 距离辅助
% close all
tt = t_lbl;
x0=[0;0;0;0];
dx0=[0.02;d2r(0.5); 1/glv.Re; 1/glv.Re];
vk = [0,d2r(0.5),0,0];
wrng = 6;
for id = 6
    pos0=avp_ref(1,7:9)';
    beacon=BCN{id}; range=RNG{id};
    yaw=compass;
    dr = mydr('init',pos0,[0;0;0],0.5);
    kf = myekf('init', 0.5, x0, dx0 , vk, wrng);
    [avp_range,avp_dr,xkpk,kk_1]=prealloc(length(avp_ref),10,10,9,5);
    ki=1;
    for i=1:length(yaw)
        t=tt(i);
        dr = mydr('update',dr,-depther(i),yaw(i,3),vxy(i,1:2));
        avp_dr(i,:)=[dr.avp',t];% 航位推算
        kf = myekf('fk', kf, dr);
        kf = myekf('algo',kf,'T');
        if mod(t,8)==0
            if size(beacon,1)==1
                dr.beacon=beacon;
            else
                dr.beacon=beacon(ki,:);
            end
            % r = range(ki);% 测量值
            
            r = sqrt(RCompu(avp_LBL_DR(i,7:9),dr.beacon)^2-(-depther(i)-dr.beacon(3))^2) + randn*6;
            kf.r_dr=sqrt(RCompu(dr.pos',dr.beacon)^2-(-depther(i)-dr.beacon(3))^2); % 计算值
            kf.yk=kf.r_dr-r;
            kf = myekf('hk',kf, dr,'range');
            kf = myekf('algo',kf,'M','UKF');
            kk_1(ki,1:4) = kf.xk;
            kk_1(ki,5) = t;
            avp_range(ki,:) = [dr.avp', t];
            xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';ki=ki+1;
        end
    end
    avp_range(ki:end,:) = [];  xkpk(ki:end,:) = [];
    kk_1(ki:end,:) = [];
    avp_range(:,7)=avp_range(:,7)-kk_1(:,3);
    avp_range(:,8)=avp_range(:,8)-kk_1(:,4);
    avp_RNG1{id}=avp_range;
    KK{id}=kk_1;
end  
%%
labell={'dr','beacon1','beacon2','beacon3','beacon4','ideal beacon','moving beacon',...
    '6-ref','6-PTSAG','6-PTSAX','6-PIXOG','1-optimzed','2-optimzed','3-optimzed','4-optimzed'};
% bbb=[1,3,11,13];
% bbb=[6:10];
bbb = id;
label_1=labell(1,[1,bbb+1]);
[radial_errors, stats] = calc_radial_error_avp(avp_ref,label_1,...
    avp_dr,avp_RNG1{bbb});