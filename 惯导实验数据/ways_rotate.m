% clear all
load('D:\Github\KF-GINS-Matlab\惯导实验数据\output\data.mat')
%%
truthpath="惯导实验数据/output/NavResult-truth.nav";
%% 旋转+缩放处理，28min时，将整条轨迹进行旋转收缩
nav_rotatesum=[]; 
nav_rotatesum(:,1) = nav000(:,1);
nav0028 = importdata("惯导实验数据\output\NavResult-RANGE1-adap-28min.nav");
nav0028 = [nav000(:,1),[d2r(nav0028(:,3:4))';nav0028(:,5)']];
figure('Name','旋转+缩放处理');
myfigurestartup(14,7,'paper')
for ki=2:12
    % nav_rotate = rotateTrajectory(nav00(:,indexrecord(ki-1)+1:indexrecord(ki)-1),...
    %     nav00(:,indexrecord(ki)));
    trajectory = nav0028(:,indexrecord(ki-1):indexrecord(ki)-1);
    newEndPoint = nav0028(:,indexrecord(ki));
    if ki==2||ki==3
        nav_rotate=nav0028(:,indexrecord(ki-1):indexrecord(ki)-1);
    elseif  ki==5||ki==9
        nav_rotatesum28{(ki-1)/4} = rotateAndScaleTrajectory([nav_rotatesum(:,indexrecord(ki-4):indexrecord(ki-1)-1), ...
            trajectory], newEndPoint);
        [nav_rotate, scFa, rotaAngdeg] = rotateAndScaleTrajectory(trajectory, newEndPoint);      
    else
        [nav_rotate, scFa, rotaAngdeg] = rotateAndScaleTrajectory(trajectory, newEndPoint);
    end

    nav_rotatesum(:,indexrecord(ki-1):indexrecord(ki)) = [nav_rotate,newEndPoint];

    subplot(4,3,ki-1)
    plot(nav0028(2,indexrecord(ki-1)+1:indexrecord(ki)), ...
        nav0028(1,indexrecord(ki-1)+1:indexrecord(ki)))
    hold on
    plot(nav0028(2,indexrecord(ki)),nav0028(1,indexrecord(ki)),'*')
    plot(nav_rotate(2,1:end),nav_rotate(1,1:end))
    plot(nav000(2,indexrecord(ki-1)+1:indexrecord(ki)-1), ...
        nav000(1,indexrecord(ki-1)+1:indexrecord(ki)-1))
    if ki==2
        legend('forward','dot','rotate','truth')
    end
end
nav_rotatesum_28=[nav_rotatesum28{1},nav_rotatesum28{2},nav_rotatesum(:,indexrecord(9):indexrecord(end))];
figure('Name','轨迹对比图')
    myfigurestartup(12,5,'prese')
plot(nav_rotatesum(2,:),nav_rotatesum(1,:))
hold on
plot(nav_rotatesum_28(2,:),nav_rotatesum_28(1,:))
plot(nav000(2,:),nav000(1,:))
plot(nav0028(2,:),nav0028(1,:))
legend('常规旋转后','28min整体旋转','参考','未旋转')
%% 旋转+缩放处理，42min时，将整条轨迹进行旋转收缩
nav_rotatesum=[]; 
nav_rotatesum(:,1) = nav000(:,1);
nav0042 = importdata("惯导实验数据\output\NavResult-RANGE1-adap-42min.nav");
nav0042 = [nav000(:,1),[d2r(nav0042(:,3:4))';nav0042(:,5)']];
figure('Name','旋转+缩放处理');
myfigurestartup(14,7,'paper')
for ki=2:12
    % nav_rotate = rotateTrajectory(nav00(:,indexrecord(ki-1)+1:indexrecord(ki)-1),...
    %     nav00(:,indexrecord(ki)));
    trajectory = nav0042(:,indexrecord(ki-1):indexrecord(ki)-1);
    newEndPoint = nav0042(:,indexrecord(ki));
    if ki==2||ki==3
        nav_rotate=nav0042(:,indexrecord(ki-1):indexrecord(ki)-1);
    elseif  ki==7
        nav_rotatesum42{(ki-1)/6} = rotateAndScaleTrajectory([nav_rotatesum(:,indexrecord(ki-6):indexrecord(ki-1)-1), ...
            trajectory], newEndPoint);
        [nav_rotate, scFa, rotaAngdeg] = rotateAndScaleTrajectory(trajectory, newEndPoint);      
    else
        [nav_rotate, scFa, rotaAngdeg] = rotateAndScaleTrajectory(trajectory, newEndPoint);
    end

    nav_rotatesum(:,indexrecord(ki-1):indexrecord(ki)) = [nav_rotate,newEndPoint];

    subplot(4,3,ki-1)
    plot(nav0042(2,indexrecord(ki-1)+1:indexrecord(ki)), ...
        nav0042(1,indexrecord(ki-1)+1:indexrecord(ki)))
    hold on
    plot(nav0042(2,indexrecord(ki)),nav0042(1,indexrecord(ki)),'*')
    plot(nav_rotate(2,1:end),nav_rotate(1,1:end))
    plot(nav000(2,indexrecord(ki-1)+1:indexrecord(ki)-1), ...
        nav000(1,indexrecord(ki-1)+1:indexrecord(ki)-1))
    if ki==2
        legend('forward','dot','rotate','truth')
    end
end
% subplot(4,3,ki)
nav_rotatesum_42=[nav_rotatesum42{1},nav_rotatesum(:,indexrecord(7):indexrecord(end))];
figure('Name','轨迹对比图')
    myfigurestartup(12,5,'prese')
plot(nav_rotatesum(2,:),nav_rotatesum(1,:))
hold on
plot(nav_rotatesum_42(2,:),nav_rotatesum_42(1,:))
plot(nav000(2,:),nav000(1,:))
plot(nav0042(2,:),nav0042(1,:))
legend('常规旋转后','42min整体旋转','参考','未旋转')
%% 旋转+缩放处理，70min时，将整条轨迹进行旋转收缩
nav_rotatesum=[]; 
nav_rotatesum(:,1) = nav000(:,1);
nav0070 = importdata("惯导实验数据\output\NavResult-RANGE1-adap-70min.nav");
nav0070 = [nav000(:,1),[d2r(nav0070(:,3:4))';nav0070(:,5)']];
figure('Name','旋转+缩放处理');
myfigurestartup(14,7,'paper')
for ki=2:12
    % nav_rotate = rotateTrajectory(nav00(:,indexrecord(ki-1)+1:indexrecord(ki)-1),...
    %     nav00(:,indexrecord(ki)));
    trajectory = nav0070(:,indexrecord(ki-1):indexrecord(ki)-1);
    newEndPoint = nav0070(:,indexrecord(ki));
    if ki==2||ki==3
        nav_rotate=nav0070(:,indexrecord(ki-1):indexrecord(ki)-1);
    elseif  ki==11
        nav_rotatesum70{(ki-1)/10} = rotateAndScaleTrajectory([nav_rotatesum(:,indexrecord(ki-10):indexrecord(ki-1)-1), ...
            trajectory], newEndPoint);
        [nav_rotate, scFa, rotaAngdeg] = rotateAndScaleTrajectory(trajectory, newEndPoint);      
    else
        [nav_rotate, scFa, rotaAngdeg] = rotateAndScaleTrajectory(trajectory, newEndPoint);
    end

    nav_rotatesum(:,indexrecord(ki-1):indexrecord(ki)) = [nav_rotate,newEndPoint];

    subplot(4,3,ki-1)
    plot(nav0070(2,indexrecord(ki-1)+1:indexrecord(ki)), ...
        nav0070(1,indexrecord(ki-1)+1:indexrecord(ki)))
    hold on
    plot(nav0070(2,indexrecord(ki)),nav0070(1,indexrecord(ki)),'*')
    plot(nav_rotate(2,1:end),nav_rotate(1,1:end))
    plot(nav000(2,indexrecord(ki-1)+1:indexrecord(ki)-1), ...
        nav000(1,indexrecord(ki-1)+1:indexrecord(ki)-1))
    if ki==2
        legend('forward','dot','rotate','truth')
    end
end
% subplot(4,3,ki)
nav_rotatesum_70=[nav_rotatesum70{1},nav_rotatesum(:,indexrecord(11):indexrecord(end))];
figure('Name','轨迹对比图')
    myfigurestartup(12,5,'prese')
plot(nav_rotatesum(2,:),nav_rotatesum(1,:))
hold on
plot(nav_rotatesum_70(2,:),nav_rotatesum_70(1,:))
plot(nav000(2,:),nav000(1,:))
plot(nav0070(2,:),nav0070(1,:))
legend('常规旋转后','70min整体旋转','参考','未旋转')
%% 旋转缩放结果保存---28min
nav_rotscale_28 = importdata(refpath);
nav_rotscale_28(:,3:4)=r2d(nav_rotatesum_28(1:2,2:end)');
output_file_rotate_28  =[navp,'-rotate+28', '.txt'];
try
    writematrix(nav_rotscale_28, output_file_rotate_28, 'Delimiter', ' ');
    fprintf('nav_rotscale信息已成功写入到 %s\n', output_file_rotate_28);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
%% 旋转缩放结果保存---42min
nav_rotscale_42 = importdata(refpath);
nav_rotscale_42(:,3:4)=r2d(nav_rotatesum_42(1:2,2:end)');
output_file_rotate_42  =[navp,'-rotate+42', '.txt'];
try 
    writematrix(nav_rotscale_28, output_file_rotate_42, 'Delimiter', ' ');
    fprintf('nav_rotscale信息已成功写入到 %s\n', output_file_rotate_42);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
%% 旋转缩放结果保存---70min
nav_rotscale_70 = importdata(refpath);
nav_rotscale_70(:,3:4)=r2d(nav_rotatesum_70(1:2,2:end)');
output_file_rotate_70  =[navp,'-rotate+70', '.txt'];
try
    writematrix(nav_rotscale_70, output_file_rotate_70, 'Delimiter', ' ');
    fprintf('nav_rotscale信息已成功写入到 %s\n', output_file_rotate_70);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end

%% 旋转+缩放处理
glvs
nav_rotatesum(:,1) = nav000(:,1);
figure('Name','旋转+缩放处理');
myfigurestartup(14,7,'paper')
for ki=2:12
    % nav_rotate = rotateTrajectory(nav00(:,indexrecord(ki-1)+1:indexrecord(ki)-1),...
    %     nav00(:,indexrecord(ki)));
    trajectory = nav00(:,indexrecord(ki-1)+1:indexrecord(ki)-1);
    newEndPoint = nav00(:,indexrecord(ki));
    if ki==2||ki==3
        nav_rotate=nav00(:,indexrecord(ki-1)+1:indexrecord(ki)-1);
    else
        [nav_rotate, scFa, rotaAngdeg] = rotateAndScaleTrajectory(trajectory, newEndPoint);
    end
    nav_rotatesum(:,indexrecord(ki-1)+1:indexrecord(ki)) = [nav_rotate,newEndPoint];
    subplot(4,3,ki-1)
    plot(nav00(2,indexrecord(ki-1)+1:indexrecord(ki)), ...
        nav00(1,indexrecord(ki-1)+1:indexrecord(ki)))
    hold on
    plot(nav00(2,indexrecord(ki)),nav00(1,indexrecord(ki)),'*')
    plot(nav_rotate(2,1:end),nav_rotate(1,1:end))
    plot(nav000(2,indexrecord(ki-1)+1:indexrecord(ki)-1), ...
        nav000(1,indexrecord(ki-1)+1:indexrecord(ki)-1))
    if ki==2
        legend('forward','dot','rotate','truth')
    end
end
figure
plot(nav_rotatesum(2,:),nav_rotatesum(1,:),'b')
hold on
plot(nav000(2,:),nav000(1,:),'r')
plot(nav00(2,:),nav00(1,:),'g')
plot(nav11(2,:),nav11(1,:),'m')
plot(nav00(2,indexrecord(2:end)),nav00(1,indexrecord(2:end)),'g*')
plot(nav000(2,indexrecord(2:end)),nav000(1,indexrecord(2:end)),'r*')
plot(nav11(2,indexrecord(2:end-1)+1),nav11(1,indexrecord(2:end-1)+1),'m*')
legend('旋转收缩','truth','forward','backforward')
%% 旋转缩放结果保存
nav_rotscale=importdata(refpath);
nav_rotscale(:,3:4)=r2d(nav_rotatesum(1:2,2:end)');
output_file_rotate  =[navp,'-rotate', '.txt'];
try
    writematrix(nav_rotscale, output_file_rotate, 'Delimiter', ' ');
    fprintf('nav_rotscale信息已成功写入到 %s\n', output_file_rotate);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
%% 查看误差
calc_radial_error(cfg.truthpath, pureinspath, refpath, output_file_rotate, output_file_rotate_28,output_file_rotate_42, output_file_rotate_70)
hold on
plot(xlim,[400,400],'DisplayName','400m界限')
ylim([0 1500])
%% 旋转+缩放处理后向结果
nav_rotatesum1(:,1) = nav000(:,1);
figure('Name','旋转+缩放处理后向结果');
myfigurestartup(14,7,'paper')
for ki=2:12
    % nav_rotate = rotateTrajectory(nav00(:,indexrecord(ki-1)+1:indexrecord(ki)-1),...
    %     nav00(:,indexrecord(ki)));
    trajectory = nav112(:,indexrecord(ki-1)+1:indexrecord(ki)-1);
    if ki==5
        newstartPoint = nav00(:,indexrecord(ki-1));
    else
        newstartPoint = nav112(:,indexrecord(ki-1));
    end
    if ki==4
        newEndPoint = nav00(:,indexrecord(ki));
    else
        newEndPoint = nav112(:,indexrecord(ki));
    end
    if ki==2||ki==3
        nav_rotate = nav00(:,indexrecord(ki-1)+1:indexrecord(ki)-1);
    else
        trajectory = flip(trajectory,2);
        [nav_rotate, ~, ~] = rotateAndScaleTrajectory(trajectory, newstartPoint);
        trajectory = flip(trajectory,2);
        nav_rotate = flip(nav_rotate,2);
        [nav_rotate, ~, ~] = rotateAndScaleTrajectory(nav_rotate, newEndPoint);
    end
    nav_rotatesum1(:,indexrecord(ki-1):indexrecord(ki)) = [newstartPoint,nav_rotate,newEndPoint];
    subplot(4,3,ki-1)
    plot(trajectory(2,:), ...
        trajectory(1,:))
    hold on
    plot(newEndPoint(2),newEndPoint(1),'*')
    plot(nav_rotate(2,1:end),nav_rotate(1,1:end))
    plot(nav000(2,indexrecord(ki-1)+1:indexrecord(ki)-1), ...
        nav000(1,indexrecord(ki-1)+1:indexrecord(ki)-1))
    if ki==2
        legend('backforward','dot','rotate','truth')
    end
end
subplot(4,3,ki)
plot(nav_rotatesum1(2,:),nav_rotatesum1(1,:))
%% 旋转缩放后向结果
nav_rotscale1=importdata(refpath);
nav_rotscale1(:,3:4)=r2d(nav_rotatesum1(1:2,2:end)');
output_file_rotateback  = [navp,'-rotate+backforward', '.txt'];
try
    writematrix(nav_rotscale1, output_file_rotateback, 'Delimiter', ' ');
    fprintf('nav_rotscale信息已成功写入到 %s\n', output_file_rotateback);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
%% 查看误差
calc_radial_error(truthpath, pureinspath, refpath,navpath , output_file_rotate,output_file_rotateback)
hold on
plot(xlim,[400,400],'DisplayName','400m界限')
ylim([0 1500])
