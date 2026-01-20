path=cell(1,4);
truth=cell(1,4);
% beacon=cell(1,4);
myfigurestartup(7,7,'paper')
for i=1:4
    truth{i}=['旋转收缩方案/input_simu',num2str(i),'/truth.txt'];
    range1=importdata(['旋转收缩方案/input_simu',num2str(i),'/range1.txt']);
    beacon(:,1)=rad2deg(range1(1,4:6));
    range2=importdata(['旋转收缩方案/input_simu',num2str(i),'/range2.txt']);
    beacon(:,2)=rad2deg(range2(1,4:6));
    range3=importdata(['旋转收缩方案/input_simu',num2str(i),'/range3.txt']);
    beacon(:,3)=rad2deg(range3(1,4:6));

    trj1=importdata(truth{i});
    trj=trj1(:,3:5);
    subplot(2,2,i)
    plot(trj(:,2),trj(:,1))
    hold on
    plot(trj(1,2),trj(1,1),'diamond')
    plot(beacon(2,1),beacon(1,1),'*')
    plot(beacon(2,2),beacon(1,2),'x')
    plot(beacon(2,3),beacon(1,3),'+')
    legend('轨迹','起点','潜标基站1','潜标基站2','潜标基站3','Location','south')
    xygo('lat','lon')
end
%% 查看误差
myfigurestartup(7,7,'paper')
for i=1:4
    subplot(2,2,i)
    path=['旋转收缩方案/output_simu',num2str(i),'/RANGE1.nav'];
    truth=['旋转收缩方案/input_simu',num2str(i),'/truth.txt'];
    ins=['旋转收缩方案/output_simu',num2str(i),'/pureins.nav'];
    calc_radial_error(truth,ins,path)
    ylim([0 1500])
end
%%
myfigurestartup(7,7,'paper')
for i=1:4
    subplot(2,2,i)
    truth=['旋转收缩方案/input_simu',num2str(i),'/truth.txt'];
    path1=['旋转收缩方案/output_simu',num2str(i),'/RANGE1.nav'];
    navpath=['旋转收缩方案/output_simu',num2str(i),'/RANGE1-adap.nav'];
    output_file_back=['旋转收缩方案/output_simu',num2str(i),'/backforward.txt'];
    output_file_rotateback=['旋转收缩方案/output_simu',num2str(i),'/rotate+backforward.txt'];
    % ins=['旋转收缩方案/output_simu',num2str(i),'/pureins.nav'];
    % calc_radial_error(truth,ins,path1,navpath,output_file_back,output_file_rotateback)
    calc_radial_error(truth,path1,navpath,output_file_back,output_file_rotateback)
    hold on
    plot(xlim,[400,400],'DisplayName','400m界限')
    % legend('EKF','AEKF','BRC+AEKF','GTS+BRC+AEKF')
    legend('EKF', 'AEKF', 'FR-IFA AEKF', 'FFBS-Fusion')
    ylim([0 600])
    title(['senerio',num2str(i)])
end
