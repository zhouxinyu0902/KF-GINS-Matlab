function [laterr,lonerr]=lonlat(avp_true,avp_dr,type,isfig,iserror,leg)
% only compare true trajectory and one trajectory
% isfig: plot lattitude and longitude comparison, 1/0, yes/no
% iserror: plot lat-lon-position error, 1/0, yes/no
glvs
for i=1:length(avp_dr)
    index(i)=find(avp_true(:,end)==avp_dr(i,end));
end
laterr=avp_dr(:,7)-avp_true(index,7);
lonerr=avp_dr(:,8)-avp_true(index,8);
%% 纬度经度对比
if isfig==1 
    myfigurestartup(7,5,type)
    subplot 211
    plot(avp_true(:,end),r2d(avp_true(:,7)));xygo('t/s','lat');title('lat-lon comparison')
    hold on
    plot(avp_dr(:,end),r2d(avp_dr(:,7)))
    legend('true trj',leg{:})
    subplot 212
    plot(avp_true(:,end),r2d(avp_true(:,8)));xygo('t/s','lon');
    hold on
    plot(avp_dr(:,end),r2d(avp_dr(:,8)))
    legend('true trj',leg{:})
end
%% 纬度经度误差
if iserror==1
    myfigurestartup(7,5,type)
    subplot 311,xygo('t/s','dlat'),title('lat-lon-position error comparison')
    plot(avp_true(index,end),laterr*glv.Re)
    subplot 312,xygo('t/s','dlon')
    plot(avp_true(index,end),lonerr*glv.Re)
    subplot 313,xygo('t/s','dRange')
    poserr=RCompu(avp_dr(:,7:9),avp_true(index,7:9));
    plot(avp_true(index,end),poserr)
end
end

