function [avp_acoustic_DR,avp_dr_octans_usbl] = AcousticDeadR(CASE,t_dr,avp_ref,Oweb,avp_acoustic_raw,octans,vxy,depther,isfig)
% 声学定位结果和航位推算相结合
% 处理深海数据时建立基准
glvs
pos0=avp_acoustic_raw(1,7:9)';
yaw=octans;
switch CASE
    case 'LBL'
        err=2;
        dt=0.5;
    case 'USBL'
        err=20;
        dt=8;
end
dx0 = [0.002;d2r(Oweb);0.1/glv.Re;0.1/glv.Re]; % 初始状态估计误差
x0 = [0.002;d2r(Oweb);0.1/glv.Re;0.1/glv.Re]*0.1; % 初始状态
kf = myekf('init',0.5, x0,dx0,[0,Oweb,0,0], ...
    [1/glv.Re,1/glv.Re]*err);
dr = mydr('init',pos0,[0.1;0.1;0.1],0.5);
[avp_acoustic_DR,avp_dr_octans_usbl,xkpk,kk] = prealloc(length(yaw),10,10,kf.m*2+1,kf.m+1);
ki=1;
for i=1:length(yaw)
    t=t_dr(i);
    dr = mydr('update',dr,-depther(i),yaw(i,3),vxy(i,1:2));% 航位推算
    avp_dr_octans_usbl(i,:) = [dr.avp', t];
    kf = myekf('fk', kf, dr);
    kf = myekf('algo',kf,'T');
    if mod(t,dt)==0
        kf.yk = dr.pos(1:2)-avp_acoustic_raw(ki,7:8)';
        kf = myekf('hk',kf, dr,'LBL');
        kf = myekf('algo',kf,'M');
        kk(ki,1:4)=kf.xk;
        kk(ki,5)=t;
        avp_acoustic_DR(ki,:) = [dr.avp', t];
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';
        ki=ki+1;
    end
end
avp_acoustic_DR(ki:end,:) = [];  xkpk(ki:end,:) = [];
kk(ki:end,:) = [];
avp_acoustic_DR(:,7) = avp_acoustic_DR(:,7)-xkpk(:,3);
avp_acoustic_DR(:,8) = avp_acoustic_DR(:,8)-xkpk(:,4);
if isfig==1
    myfigurestartup(7,7,'prese');lon_lat_err(avp_ref,{'raw','fused'}, ...
        avp_dr_octans_usbl,{avp_acoustic_raw,avp_acoustic_DR})
end
end

