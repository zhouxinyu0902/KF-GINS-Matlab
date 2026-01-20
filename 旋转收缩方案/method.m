%%
pos0=nav000(:,1);
nav000xyz = pos2dxyz(nav000',pos0)';
nav00xyz = pos2dxyz(nav00',pos0)';
nav11xyz = pos2dxyz(nav11',pos0)';
nav112xyz = pos2dxyz(nav112',pos0)';
figure
plot(nav000xyz(1,:),nav000xyz(2,:),'LineWidth',4,Color='k')
hold on
plot(nav00xyz(1,:),nav00xyz(2,:),'r')
plot(nav00xyz(1,indexrecord(2:end)),nav00xyz(2,indexrecord(2:end)),'r*')
plot(nav11xyz(1,:),nav11xyz(2,:),'m')
plot(nav112xyz(1,:),nav112xyz(2,:),'b')
plot(nav112xyz(1,indexrecord(1:end-1)),nav112xyz(2,indexrecord(1:end-1)),'b*')
legend('真实','前向滤波','*','后向推算','后向+滤波','*')
axis equal
%% 旋转+缩放处理后向结果

nav_rotatesum1(:,1) = nav000(:,1);
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
    % if ki==2||ki==3
    %     nav_rotate = nav00(:,indexrecord(ki-1)+1:indexrecord(ki)-1);
    % else
        trajectory = flip(trajectory,2);
        [nav_rotate, ~, ~] = rotateAndScaleTrajectory(trajectory, newstartPoint);
        trajectory = flip(trajectory,2);
        nav_rotate = flip(nav_rotate,2);
        [nav_rotate, ~, ~] = rotateAndScaleTrajectory(nav_rotate, newEndPoint);
    % end
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
nav_rotscale1=importdata(navpath);
nav_rotscale1(:,3:4)=r2d(nav_rotatesum1(1:2,2:end)');
output_file_rotateback  = [cfg.outputfolder,'/rotate+backforward', '.txt'];
try
    writematrix(nav_rotscale1, output_file_rotateback, 'Delimiter', ' ');
    fprintf('nav_rotscale信息已成功写入到 %s\n', output_file_rotateback);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end

%% 旋转缩放后向结果
nav_back=importdata(navpath);
nav_back(:,3:4)=r2d(nav112(1:2,2:end)');
output_file_back  = [cfg.outputfolder,'/backforward', '.txt'];
try
    writematrix(nav_back, output_file_back, 'Delimiter', ' ');
    fprintf('nav_rotscale信息已成功写入到 %s\n', output_file_back);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
%%
pos0=nav000(:,1);
nav000xyz = pos2dxyz(nav000',pos0)';
nav00xyz = pos2dxyz(nav00',pos0)';
nav11xyz = pos2dxyz(nav11',pos0)';
nav112xyz = pos2dxyz(nav112',pos0)';
navresultxyz = pos2dxyz(nav_rotatesum1',pos0)';

figure
plot(nav000xyz(1,:),nav000xyz(2,:),'LineWidth',4,Color='k')
hold on
plot(nav00xyz(1,:),nav00xyz(2,:),'r')
plot(nav00xyz(1,indexrecord(2:end)),nav00xyz(2,indexrecord(2:end)),'r*')
plot(nav11xyz(1,:),nav11xyz(2,:),'m')
plot(nav112xyz(1,:),nav112xyz(2,:),'b')
plot(nav112xyz(1,indexrecord(1:end-1)),nav112xyz(2,indexrecord(1:end-1)),'b*')
plot(navresultxyz(1,:),navresultxyz(2,:),'LineWidth',2,Color='r')
legend('真实','前向滤波','*','后向推算','后向+滤波','*','最后结果')
axis equal