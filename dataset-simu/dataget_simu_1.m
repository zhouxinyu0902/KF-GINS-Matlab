clear all
load('D:\GitHub\KF-GINS-Matlab\dataset-simu\data\data-1.mat')
glvs
%% 航向角仿真
trueyaw=d2r(pva_ref(:,11));
% for i=1:length(trueyaw)-1
%     calyaw(i,:)=poscalyaw(trj.avp(i,7:8),trj.avp(i+1,7:8));
% end
% plot(1:i,calyaw,1:i+1,trueyaw)
% legend('cal','trueyaw')
compass=trueyaw+normrnd(0,d2r(0.5),size(trueyaw));
compass(:,2)=pva_ref(:,2);
yawpath="dataset-simu\compass.nav";
yawfp=fopen(yawpath,'wt');
for i=1:length(compass)
    fprintf(yawfp, '%12.6f %.10f \n', compass(i,:));
end
fclose(yawfp);
%% gnss数据
gnss(:,1)=pva_ref(200:200:end,2);
gnss(:,2:3)=pva_ref(200:200:end,3:4)+...
    normrnd(0,r2d(0.02/glv.Re),size(pva_ref(200:200:end,3:4)));
gnss(:,4)=pva_ref(200:200:end,5)+...
    normrnd(0,0.02,size(pva_ref(200:200:end,5)));
gnsspath="dataset-simu\gnss.txt";
try
    writematrix(gnss, gnsspath, 'Delimiter', ' ');
    fprintf('高度信息已成功写入到 %s\n', gnsspath);
catch ME
    error('错误：写入 gnss.txt 文件失败。错误信息：%s', ME.message);
end

gnss(:,1)=pva_ref(200:200:end,2);
gnss(:,2:3)=pva_ref(200:200:end,3:4)+...
    normrnd(0,r2d(2/glv.Re),size(pva_ref(200:200:end,3:4)));
gnss(:,4)=pva_ref(200:200:end,5)+...
    normrnd(0,2,size(pva_ref(200:200:end,5)));
gnsspath="dataset-simu\gnss-2m.txt";
try
    writematrix(gnss, gnsspath, 'Delimiter', ' ');
    fprintf('高度信息已成功写入到 %s\n', gnsspath);
catch ME
    error('错误：写入 gnss.txt 文件失败。错误信息：%s', ME.message);
end
%% 信标数据
dxyz_bcn=[500,0,600;
    500,1000,600;
    500,2500,600;
    0,2500,600;
    -500,2500,600;
    -700,1000,600];
BCN=dxyz2pos(dxyz_bcn,trj.avp0(7:9));
BCNddm=BCN;
BCNddm(:,1:2)=BCNddm(:,1:2)/pi*180;

dxyztrj=pos2dxyz(trj.avp(:,7:9),trj.avp0(7:9));
myfigurestartup(6,6,'prese')
plot(dxyztrj(:,1),dxyztrj(:,2))
for i=1:6
    hold on
    plot(dxyz_bcn(i,1),dxyz_bcn(i,2),'*');
end
%%
% tic
% range=cell(1,6);
% for i=1:6
%     range{i}=bcn2range(pva_ref,BCNddm(i,:));
%     range{i}(:,2:3)=range{i}(:,2:3)+normrnd(0,1,size(range{i}(:,2:3)));
%     rangepath = sprintf("dataset-simu/range-%d.txt", i);
%     rangefp=fopen(rangepath,'wt');
%     for ii=1:length(range{i})
%         fprintf(rangefp,'%12.6f %.8f %.8f %12.8f %12.8f %8.4f\n', range{i}(ii,:));
%     end
%     fclose(rangefp);
% end
% toc
% 123s
%%
tic
range = cell(1, 6);  % 预先分配内存

% 并行处理（如果安装了Parallel Computing Toolbox）
% parfor i = 1:6  % 如果可用可以改用parfor
for i = 1:6
    % 1. 避免重复计算
    current_range = bcn2range(pva_ref, BCNddm(i, :));
    
    % 2. 向量化噪声添加（已优化）
    noise = normrnd(0, 1, [size(current_range, 1), 2]);
    current_range(:, 2:3) = current_range(:, 2:3) + noise;
    
    % 3. 优化文件写入：一次写入所有数据
    rangepath = sprintf("dataset-simu/range-%d.nav", i);
    
    % 4. 使用更高性能的文件写入函数
    % % 选项1: 使用fprintf矩阵写入（比循环快10-100倍）
    % writematrix(rangepath, current_range, 'delimiter', '\t', 'precision', 12);
    
    % 选项2: 如果必须保持精确格式（更快但需要格式控制）
    fileID = fopen(rangepath, 'wt');
    fprintf(fileID, '%12.6f %.8f %.8f %12.8f %12.8f %8.4f\n', current_range');
    fclose(fileID);
    
    range{i} = current_range;  % 保存处理后的数据
end
toc
% 26s
%% depth数据
depth=[trj.avp(:,end),trj.avp(:,end-1)];
depth(:,2)=depth(:,2)+normrnd(0,0.2,size(depth(:,2)));
depthpath="dataset-simu\depth.nav";
fileID = fopen(depthpath, 'wt');
fprintf(fileID, '%12.6f %.8f\n', depth');  % 注意转置操作
fclose(fileID);
%% 保存数据
avpref=trj.avp;
gnssinrad=gnss;
gnssinrad(:,2:3)=d2r(gnss(:,2:3));
save D:/GitHub/PSINS/psins2401/mytest/04_simu_insrange/data/data-simu.mat trjimu avpref range1 gnssinrad imuerr