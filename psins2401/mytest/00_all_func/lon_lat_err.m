function [err,err_R]=lon_lat_err(avp,leg,avp_dr,avp_range)
ref_color='r';
color={'b','g','b','m'};
%% 观察avp_dr，avp和多个avp_range的纬经度误差对比
glvs
err1=avpcmp(avp_dr,avp);
leg=[{'dr'},leg(:)'];
subplot 311,plot(avp(:,end),err1(:,7)*glv.Re,ref_color),
xygo('t/s','dlat'),title('latitude error comparison')
n=length(avp_range);
for i=1:n
    hold on;
    err{i}=avpcmp(avp_range{i},avp);
    plot(err{i}(:,end),err{i}(:,7)*glv.Re,color{i});
end
legend(leg,'Location','best')
subplot 312,plot(avp(:,end),err1(:,8)*glv.Re,ref_color),
xygo('t/s','dlon'),title('longitude error comparison')
for i=1:n
    hold on;
    plot(err{i}(:,end),err{i}(:,8)*glv.Re,color{i});
end
legend(leg,'Location','best') 
subplot 313,err_range=RCompu(avp(:,7:9),avp_dr(:,7:9));
plot(avp(:,end),err_range,ref_color),
% yticks(0:10:ceil(max(err_range(1:end-1))))
xygo('t/s','dRange'),title('position error comparison')

for i=1:n
    hold on
    for j=1:length(avp_range{i})
        index(j)=find(avp(:,end)==avp_range{i}(j,end));
    end
    err_R{i}=RCompu(avp(index,7:9),avp_range{i}(:,7:9));
    plot(avp_range{i}(:,end),err_R{i},color{i});
end
legend(leg,'Location','best')
currentColors = get(gca, 'ColorOrder');
% 如果颜色不够用，可以扩展颜色循环
extendedColors = [currentColors; 1 0 0;
                  0 1 0;
                  0 0 1;
                  0 1 1]; 
set(gca, 'ColorOrder', extendedColors);
end

