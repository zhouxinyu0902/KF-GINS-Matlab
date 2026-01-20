% clear
load('D:\Github\KF-GINS-Matlab\惯导实验数据\output\data.mat')
truthpath="惯导实验数据/output/NavResult-truth.nav";
%%
pos0=nav000(:,1);
nav000xyz = pos2dxyz(nav000',pos0)';
nav00xyz = pos2dxyz(nav00',pos0)';
nav11xyz = pos2dxyz(nav11',pos0)';
nav112xyz = pos2dxyz(nav112',pos0)';
%%
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
% %% 后向推算阶段绘图
% for ki=2:12
%     if ki==2
%         figure('Name','后向推算');
%         myfigurestartup(15,7,'paper') 
%     end
%     subplot(3,4,ki-1)
%     plot(nav11(2,indexrecord(ki-1):indexrecord(ki)-1), ...
%         nav11(1,indexrecord(ki-1):indexrecord(ki)-1))
%     hold on
%     plot(nav11(2,indexrecord(ki)-1),nav11(1,indexrecord(ki)-1),'*')
%     % 
%     plot(nav00(2,indexrecord(ki-1):indexrecord(ki)), ...
%         nav00(1,indexrecord(ki-1):indexrecord(ki)))
%     plot(nav00(2,indexrecord(ki-1)),nav00(1,indexrecord(ki-1)),'*')
%     % 参考结果
%     plot(nav000(2,indexrecord(ki-1):indexrecord(ki)), ...
%         nav000(1,indexrecord(ki-1):indexrecord(ki)))
%     plot(nav000(2,indexrecord(ki-1)),nav000(1,indexrecord(ki-1)),'*')
%     if ki==2
%         legend('backward','start','forward','start','truth','start')
%     end
% end
% subplot(3,4,ki)
% plot(nav11(2,:),nav11(1,:))
%% 后向结果
nav_backforward=importdata(refpath);
nav_backforward(1:end,3:4)=r2d(nav11(1:2,2:end)');
output_file_backforward = [navp,'-backforward', '.txt'];
try
    writematrix(nav_backforward, output_file_backforward, 'Delimiter', ' ');
    fprintf('nav_backforwad信息已成功写入到 %s\n', output_file_backforward);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
%% 后向结果+滤波
nav_backforward=importdata(refpath);
nav_backforward(1:end,3:4)=r2d(nav112(1:2,2:end)');
output_file_backforward_filter = [navp,'-backforward-filter', '.txt'];
try
    writematrix(nav_backforward, output_file_backforward_filter, 'Delimiter', ' ');
    fprintf('nav_backforwad信息已成功写入到 %s\n', output_file_backforward_filter);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
%% 查看误差
calc_radial_error(truthpath,pureinspath, navpath, output_file_backforward_filter,output_file_backforward)
hold on
plot(xlim,[400,400],'DisplayName','400m界限')
ylim([0 1500])
%% 双向平滑器
% nav01 = Bidirectional_smoother(P_F_store,P_B_store,nav00,nav11,nav000);
nav011 = Bidirectional_smoother(P_F_store,P_B_store2,nav00,nav112,nav000);
% 后向结果+滤波+双向平滑
nav_backforward=importdata(refpath);
nav_backforward(1:end,3:4)=r2d(nav011(1:2,2:end)');
output_file_Bismoother = [navp,'-Bismoother', '.txt'];
try
    writematrix(nav_backforward, output_file_Bismoother, 'Delimiter', ' ');
    fprintf('nav_backforwad信息已成功写入到 %s\n', output_file_Bismoother);
catch ME
    error('错误：写入文件失败。错误信息：%s', ME.message);
end
% 查看误差
calc_radial_error(truthpath,pureinspath, refpath, output_file_backforward_filter,output_file_Bismoother)
hold on
plot(xlim,[400,400],'DisplayName','400m界限')
ylim([0 1500])