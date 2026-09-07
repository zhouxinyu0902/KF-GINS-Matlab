function xk_plot(kod,eb,dr_err,leg,kk_1,flag)
%% 观察扩展卡尔曼滤波的输出
%  观察多个kk_1
%  kk_1:struct
%  flag:==1只看关心的变量（纬经度）
glvs
ref_color='m--';
color={'r','g','b','k'};
leg=[leg(:)',{'ref'}];
n=length(kk_1);
if flag==0
    subplot 411,title('output of EKF')
    for i=1:n
        hold on
        plot(kk_1{i}(:,5),kk_1{i}(:,1),color{i});
    end
    hold on
    plot(kk_1{1}(:,5),ones(size(kk_1{1}(:,1)))*kod,ref_color)
    hold on
    plot(kk_1{1}(:,5),-ones(size(kk_1{1}(:,1)))*kod,ref_color)
    xygo('t/s','dkod');legend(leg );
    subplot 412,
    for i=1:n
        hold on
        plot(kk_1{i}(:,5),r2d(kk_1{i}(:,2))*60,color{i});
    end
    hold on
    plot(kk_1{1}(:,5),-ones(size(kk_1{1}(:,1)))*r2d(eb)*60,ref_color)
    hold on
    plot(kk_1{1}(:,5),ones(size(kk_1{1}(:,1)))*r2d(eb)*60,ref_color)
    legend(leg )
    xygo('t/s','dyaw')
    subplot 413;
    for i=1:n
        hold on
        plot(kk_1{i}(:,5),kk_1{i}(:,3)*glv.Re,color{i});
    end
    hold on
    plot(dr_err(:,end),dr_err(:,7)*glv.Re,ref_color);
    xygo('t/s','dlat');legend(leg )
    subplot 414
    for i=1:n
        hold on
        plot(kk_1{i}(:,5),kk_1{i}(:,4)*glv.Re,color{i});
    end
    hold on
    plot(dr_err(:,end),dr_err(:,8)*glv.Re,ref_color);
    xygo('t/s','dlon');legend(leg )
else
    subplot 211;
    for i=1:n
        hold on
        plot(kk_1{i}(:,5),kk_1{i}(:,3)*glv.Re,color{i});
    end
    hold on
    plot(dr_err(:,end),dr_err(:,7)*glv.Re,ref_color);
    xygo('t/s','dlat');legend(leg,'Location','best')
    subplot 212
    for i=1:n
        hold on
        plot(kk_1{i}(:,5),kk_1{i}(:,4)*glv.Re,color{i});
    end
    hold on
    plot(dr_err(:,end),dr_err(:,8)*glv.Re,ref_color);
    xygo('t/s','dlon');legend(leg,'Location','best')
end
end

