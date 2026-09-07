function myerrorplot(avpref,avptocmp)
% AVPERRPLOT avperr show
% FUCTION used: glvs, tscaleget, xygo
% 根据参考avp和待比较的avp画出avp误差
% if ~(length(avpref)==length(avptocmp))
    [A, ~] = ismember(avpref(:,10),avptocmp(:,10));
    avpref=avpref(A,:);
% end
glvs
err(:,1:9)=avptocmp(:,1:9)-avpref(:,1:9);

t=avptocmp(:,end);
subplot(221), 
plot(t, [r2d(err(:,1:2)),r2d(yawcvt(err(:,3)))],'LineWidth',1); 
title('Attitude Error');
xlabel('Time[s]');
ylabel('Error[deg]');
legend('Pitch', 'Roll', 'Yaw');
grid on

subplot(222), 
plot(t, err(:,4:6),'LineWidth',1); 
title('Velocity Error');
xlabel('Time[s]');
ylabel('Error[m/s]');
legend('East', 'North', 'Up');
grid on

subplot(223), 
plot(t, [err(:,7:8)*glv.Re,err(:,9)],'LineWidth',1); 
title('Position Error');
xlabel('Time[s]');
ylabel('Error[m]');
legend('East', 'North', 'Up');
grid on

subplot(224), 
plot(t,RCompu(avptocmp(:,7:9),avpref(:,7:9)),'LineWidth',1); 
title('Radial Error');
xlabel('Time[s]');
ylabel('Error[m]');
grid on
end

