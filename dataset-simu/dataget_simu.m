%% 仿真轨迹,生成imu和avp数据
clear all
glvs
avp0 = [[0;0;d2r(90)]; [0;0;0]; [d2r([15;115]);-1200]];
ts=0.005;
%% 方形轨迹
xxx = [];
% seg = trjsegment(xxx, 'init',         0);
% seg = trjsegment(seg, 'uniform',      20); % 保持原来的状态不变
% seg = trjsegment(seg, 'accelerate',   5, xxx, 0.3); % 加速
% seg = trjsegment(seg, 'uniform',      120);
% % seg = trjsegment(seg, 'headdown', 10, 5);
% seg = trjsegment(seg, 'turnleft', 15, 6);
% seg = trjsegment(seg, 'uniform',      90);
% seg = trjsegment(seg, 'turnleft', 15, 6);
% seg = trjsegment(seg, 'uniform',      120);
% seg = trjsegment(seg, 'turnleft', 15, 6);
% seg = trjsegment(seg, 'uniform',      80);
% seg = trjsegment(seg, 'turnright', 15, 6);
% seg = trjsegment(seg, 'uniform',      90);
% seg = trjsegment(seg, 'turnright', 15, 6);
% seg = trjsegment(seg, 'uniform',      120);
% seg = trjsegment(seg, 'deaccelerate',   5, xxx, 0.3); % 加速
% trj= trjsimu(avp0, seg.wat, ts, 5); 

seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      50); % 保持原来的状态不变
seg = trjsegment(seg, 'accelerate',   5, xxx, 0.4); % 加速
seg = trjsegment(seg, 'uniform',      1200);
% seg = trjsegment(seg, 'headdown', 10, 5);
seg = trjsegment(seg, 'turnleft', 15, 6);
seg = trjsegment(seg, 'uniform',      900);
seg = trjsegment(seg, 'turnleft', 15, 6);
seg = trjsegment(seg, 'uniform',      1200);
seg = trjsegment(seg, 'turnleft', 15, 6);
seg = trjsegment(seg, 'uniform',      900);
seg = trjsegment(seg, 'deaccelerate',   5, xxx, 0.4); % 加速
trj= trjsimu(avp0, seg.wat, ts, 1); 
myfigurestartup(10,10,'prese')
insplot(trj.avp)
fprintf('总时长%d（s）',sum(trj.wat(:,1)))
%% Z轨迹
xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      50); % 保持原来的状态不变
seg = trjsegment(seg, 'accelerate',   5, xxx, 0.4); % 加速
seg = trjsegment(seg, 'uniform',      1200);
% seg = trjsegment(seg, 'headdown', 10, 5);
seg = trjsegment(seg, 'turnleft', 15, 6);
seg = trjsegment(seg, 'uniform',      90);
% seg = trjsegment(seg, 'headdown', 10, 5);
seg = trjsegment(seg, 'turnleft', 15, 6);
seg = trjsegment(seg, 'uniform',      1200);
seg = trjsegment(seg, 'turnright', 15, 6);
seg = trjsegment(seg, 'uniform',      90);
seg = trjsegment(seg, 'turnright', 15, 6);
seg = trjsegment(seg, 'uniform',      1200);
seg = trjsegment(seg, 'deaccelerate',   5, xxx, 0.4); % 加速
trj1= trjsimu(avp0, seg.wat, ts, 1); 
myfigurestartup(10,10,'prese')
insplot(trj1.avp)
fprintf('总时长%d（s）',sum(trj1.wat(:,1)))
%% STRAIGHT LINE轨迹

xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      50); % 保持原来的状态不变
seg = trjsegment(seg, 'accelerate',   5, xxx, 0.4); % 加速
seg = trjsegment(seg, 'uniform',      3600);
seg = trjsegment(seg, 'deaccelerate',   5, xxx, 0.4); % 加速
trj2= trjsimu(avp0, seg.wat, ts, 1); 
myfigurestartup(10,10,'prese')
insplot(trj2.avp)
fprintf('总时长%d（s）',sum(trj2.wat(:,1)))
%% 仿真imu数据,增加噪声
% LEADOR-a15的参数
% eb=0.027;
% db=15/0.98;
% web=0.003;
% wdb=0.03*1e6/(60*9.8);
% 
% cfg.gyrarw = 0.003; % [deg/sqrt(h)]
% cfg.accvrw = 0.03; % [m/s/sqrt(h)]
% cfg.gyrbiasstd = 0.027; % [deg/h]
% cfg.accbiasstd = 15; % [mGal]

% eb=2;db=0.36*980;web=0.15;wdb=200/9.8;
% eb=0.1;db=0.1;web=0.05;wdb=10;
% 设置仿真参数，STG120零偏稳定性+随机游走，加速度计的零偏选20ug，随机游走随便取的
eb=0.005;
db=20;
web=0.0003;
wdb=0.005;
rng(1);

imuerr = imuerrset(eb, db, web, wdb); 
trjimu_square= imuadderr(trj.imu, imuerr);
trjimu_zz= imuadderr(trj1.imu, imuerr);
trjimu_line= imuadderr(trj2.imu, imuerr);
%% 将仿真的数据右前上转为真实的测量前右下，并保存
imu_square = imuRFU2FRD(trjimu_square);
imupath="dataset-simu\square\imu_square.nav";
imufp=fopen(imupath,'wt');
fprintf(imufp, '%.9f %.10f %.10f %.10f %.10f %.10f %.10f \n', imu_square');
fclose(imufp);

imu_zz = imuRFU2FRD(trjimu_zz);
imupath="dataset-simu\zz\imu_zz.nav";
imufp=fopen(imupath,'wt');
fprintf(imufp, '%.9f %.10f %.10f %.10f %.10f %.10f %.10f \n', imu_zz');
fclose(imufp);

imu_line = imuRFU2FRD(trjimu_line);
imupath="dataset-simu\line\imu_line.nav";
imufp=fopen(imupath,'wt');
fprintf(imufp, '%.9f %.10f %.10f %.10f %.10f %.10f %.10f \n', imu_line');
fclose(imufp);
%% 将仿真的参考值ENU转到NED下
pva_ref_square = avpENU2NED(trj.avp);
truthpath="dataset-simu\square\truth.nav";
truthfp=fopen(truthpath,'wt');
fprintf(truthfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', pva_ref_square');
fclose(truthfp);

pva_ref_zz = avpENU2NED(trj1.avp);
truthpath="dataset-simu\zz\truth.nav";
truthfp=fopen(truthpath,'wt');
fprintf(truthfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', pva_ref_zz');
fclose(truthfp);

pva_ref_line = avpENU2NED(trj2.avp);
truthpath="dataset-simu\line\truth.nav";
truthfp=fopen(truthpath,'wt');
fprintf(truthfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', pva_ref_line');
fclose(truthfp);
%% 仿真除信标以外的数据
outputpath='dataset-simu\square';
% save dataset-simu\data\data-1.mat pva_ref trj trjimu
[gnss,gnss_2m,depth]=gengnssdata(pva_ref_square,outputpath);
%% 仿真静止信标
bcn=[-5,5*sqrt(3),-0.6;
    -15,-5*sqrt(3),-0.6;
    5,-5*sqrt(3),-0.6]*1000;
range=genStaticRangedata(pva_ref_square,bcn,outputpath,10);
%% 仿真运动信标

%% 辅助函数
%%% gnss误差0.02/2m 深度计误差0.2m
function [gnss,gnss_2m,depth]=gengnssdata(pva_ref,outputpath)
%% gnss数据
% 0.02m的误差
glvs
gnss(:,1)=pva_ref(1:200:end,2);
gnss(:,2:3)=pva_ref(1:200:end,3:4)+...
    normrnd(0,r2d(0.02/glv.Re),size(pva_ref(1:200:end,3:4)));
gnss(:,4)=pva_ref(1:200:end,5)+...
    normrnd(0,0.02,size(pva_ref(1:200:end,5)));
gnsspath=outputpath+"\gnss.txt";
try
    writematrix(gnss, gnsspath, 'Delimiter', ' ');
    fprintf('gnss信息已成功写入到 %s\n', gnsspath);
catch ME
    error('错误：写入 gnss.txt 文件失败。错误信息：%s', ME.message);
end
% 2m的误差
gnss_2m(:,1)=pva_ref(1:200:end,2);
gnss_2m(:,2:3)=pva_ref(1:200:end,3:4)+...
    normrnd(0,r2d(2/glv.Re),size(pva_ref(1:200:end,3:4)));
gnss_2m(:,4)=pva_ref(1:200:end,5)+...
    normrnd(0,2,size(pva_ref(1:200:end,5)));
gnsspath=outputpath+"\gnss-2m.txt";
try
    writematrix(gnss_2m, gnsspath, 'Delimiter', ' ');
    fprintf('gnss信息已成功写入到 %s\n', gnsspath);
catch ME
    error('错误：写入 gnss.txt 文件失败。错误信息：%s', ME.message);
end
%% depth数据,精度0.2m
depth=[pva_ref(:,end),pva_ref(:,end-1)];
depth(:,2)=depth(:,2)+normrnd(0,0.2,size(depth(:,2)));
depthpath=outputpath+"\depth.nav";
fileID = fopen(depthpath, 'wt');
try
    fprintf(fileID, '%12.6f %.8f\n', depth');  % 注意转置操作
    fprintf('高度/深度信息已成功写入到 %s\n', depthpath);
catch ME
    error('错误：写入 depth.txt 文件失败。错误信息：%s', ME.message);
end
fclose(fileID);
end

%%% 距离误差sigma给定
function range = genStaticRangedata(pva_ref,bcn_dxyz,outputpath,sigma)
% %% 信标数据
% dxyz_bcn=[500,0,600;
%     500,1000,600;
%     500,2500,600;
%     0,2500,600;
%     -500,2500,600;
%     -700,1000,600];
nums=length(bcn_dxyz);

trjrrm=[pva_ref(:,3:4)/180*pi,pva_ref(:,5)];
dxyztrj=pos2dxyz(trjrrm,trjrrm(1,:)');

BCN=dxyz2pos(bcn_dxyz,trjrrm(1,:)');
BCNddm=BCN;
BCNddm(:,1:2)=BCNddm(:,1:2)/pi*180;

myfigurestartup(6,6,'prese')
plot(dxyztrj(:,1),dxyztrj(:,2))
for i=1:nums
    hold on
    plot(bcn_dxyz(i,1),bcn_dxyz(i,2),'*');
end
tic
range = cell(1, nums);  % 预先分配内存
for i = 1:nums
    current_range = bcn2range(pva_ref, BCNddm(i, :));
    % 噪声添加
    noise = normrnd(0, sigma, [size(current_range, 1), 2]);
    current_range(:, 2:3) = current_range(:, 2:3) + noise;
    % 一次写入所有数据    
    filename = sprintf("//range-%d.nav", i);
    rangepath = outputpath + filename;
    fileID = fopen(rangepath, 'wt');
    try
        fprintf(fileID, '%12.6f %.8f %.8f %12.8f %12.8f %8.4f\n', current_range');  % 注意转置操作
        fprintf('距离信息已成功写入到 %s\n', rangepath);
    catch ME
        error('错误：写入 depth.txt 文件失败。错误信息：%s', ME.message);
    end
    fclose(fileID);
    range{i} = current_range;  % 保存处理后的数据
end
toc
% % 26s
end
%% 保存数据
% avpref=trj.avp;
% gnssinrad=gnss;
% gnssinrad(:,2:3)=d2r(gnss(:,2:3));
