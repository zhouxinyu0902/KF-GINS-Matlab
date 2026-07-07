myfigurestartup(10,10,'paper')
for i=4:7
    subplot(2,2,i-3)
    disp(['-------------',num2str(i),'---------------------'])
    path=['旋转收缩方案1/output',num2str(i)];
    truthpath=['旋转收缩方案1/input',num2str(i),'/truth.nav'];
    path_pureins=[path,'/PureIns.nav'];
    path_EKF=[path,'/EKF.nav'];
    path_AEKF=[path,'/AEKF.nav'];
    path_BRC_AEKF=[path,'/BRC-AEKF.txt'];
    path_GTS_BRC_AEKF=[path,'/GTS-BRC-AEKF.txt']; cd
    % path_GTS_BRC_AEKF_1=[path,'/GTS-BRC-1-AEKF.txt'];
    % calc_radial_error(truthpath,path_EKF,path_AEKF,path_BRC_AEKF,path_GTS_BRC_AEKF,path_GTS_BRC_AEKF_1)
    calc_radial_error(truthpath,path_EKF,path_AEKF,path_BRC_AEKF,path_GTS_BRC_AEKF)
    % calc_radial_error(truthpath,path_pureins,path_EKF,path_AEKF,path_BRC_AEKF,path_GTS_BRC_AEKF)
    legend('EKF', 'AEKF', 'FR-IFA AEKF', 'FFBS-Fusion')
    hold on
    plot(xlim,[400,400],'DisplayName','400m界限')
    ylim([0 600])
end
%%
myfigurestartup(10,10,'paper')
for i=4:7
    subplot(2,2,i-3)
    path=['旋转收缩方案1/output',num2str(i)];
    truthpath=['旋转收缩方案1/input',num2str(i),'/truth.nav'];
    truth = importdata(truthpath);
    rangedata1 = importdata(['旋转收缩方案1/input',num2str(i),'/range1.txt']);
    rangedata2 = importdata(['旋转收缩方案1/input',num2str(i),'/range2.txt']);
    rangedata3 = importdata(['旋转收缩方案1/input',num2str(i),'/range3.txt']);
    rangedata=[rangedata1(1,:);rangedata2(2,:);rangedata3(3,:)];
    plot(truth(:,4),truth(:,3))
    hold on
    plot(truth(1,4),truth(1,3),'*')
    plot(truth(end,4),truth(end,3),'*')
    marker=[">","hexagram","pentagram"];
    for ii=1:3
        plot(rangedata(ii,5)/pi*180,rangedata(ii,4)/pi*180,marker(ii))
    end
    legend('轨迹','起点','终点','信标1','信标2','信标3')
    xygo('经度/°','纬度/°')
end
%%
for i=4:5
    path=['旋转收缩方案1/output',num2str(i)];
    truthpath=['旋转收缩方案1/input',num2str(i),'/truth.nav'];
    truth = importdata(truthpath);
    lat{i-3}=truth(:,3);
    lon{i-3}=truth(:,4);
end
myfigurestartup(10,10,'paper')
gx=geoaxes;
geoplot(gx, lat{1}, lon{1},'r');
hold on
geoplot(gx, lat{1}(1), lon{1}(1),'*');
geoplot(gx, lat{1}(end), lon{1}(end),'*');

geoplot(gx, lat{2}, lon{2},'y');
geoplot(gx, lat{2}(1), lon{2}(1),'*');
geoplot(gx, lat{2}(end), lon{2}(end),'*');
% geobasemap('streets');
% geobasemap('topographic')
geobasemap('satellite')
legend('轨迹1','起点','终点','轨迹2','起点','终点')
