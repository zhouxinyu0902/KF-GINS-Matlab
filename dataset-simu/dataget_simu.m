%% 仿真轨迹,生成imu和avp数据
% 生成三种类型的轨迹
clear all
glvs
avp0 = [[0;0;d2r(90)]; [0;0;0]; [d2r([15;115]);-1200]];
ts=0.005;
%% 方形轨迹
xxx = [];
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
%%
myfigurestartup(7,7,'prese')
insplot(trj.avp)
fprintf('方形轨迹总时长%d（s）\n',sum(trj.wat(:,1)))
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
myfigurestartup(7,7,'prese')
insplot(trj1.avp)
fprintf('Z形轨迹总时长%d（s）',sum(trj1.wat(:,1)))
%% STRAIGHT LINE轨迹
xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      50); % 保持原来的状态不变
seg = trjsegment(seg, 'accelerate',   5, xxx, 0.4); % 加速
seg = trjsegment(seg, 'uniform',      3600);
seg = trjsegment(seg, 'deaccelerate',   5, xxx, 0.4); % 加速
trj2= trjsimu(avp0, seg.wat, ts, 1); 
myfigurestartup(7,7,'prese')
insplot(trj2.avp)
fprintf('直线轨迹总时长%d（s）\n',sum(trj2.wat(:,1)))
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
eb=0.003;
db=7;
web=0.0003;
wdb=2.7778e-05;%1e-6m/s/sqrt(Hz)
%1e-6*1e5/3600 = 2.7778e-05
rng(1);
imuerr = imuerrset(eb, db, web, wdb, web, 4, wdb ,4, 5 , 10, 5, 10, 10, 10, 10); 
% imuerr = imuerrset(eb, db, web, wdb); 
trjimu_square= imuadderr(trj.imu, imuerr);
trjimu_zz= imuadderr(trj1.imu, imuerr);
trjimu_line= imuadderr(trj2.imu, imuerr);
%% 将仿真的数据右前上转为真实的测量前右下，并保存
imu_square = imuRFU2FRD(trjimu_square);
imupath="dataset-simu\square\input\imu.nav";
imufp=fopen(imupath,'wt');
fprintf(imufp, '%.9f %.10f %.10f %.10f %.10f %.10f %.10f \n', imu_square');
fclose(imufp);

imu_zz = imuRFU2FRD(trjimu_zz);
imupath="dataset-simu\zz\input\imu.nav";
imufp=fopen(imupath,'wt');
fprintf(imufp, '%.9f %.10f %.10f %.10f %.10f %.10f %.10f \n', imu_zz');
fclose(imufp);

imu_line = imuRFU2FRD(trjimu_line);
imupath="dataset-simu\line\input\imu.nav";
imufp=fopen(imupath,'wt');
fprintf(imufp, '%.9f %.10f %.10f %.10f %.10f %.10f %.10f \n', imu_line');
fclose(imufp);
%% 将仿真的参考值ENU转到NED下
pva_ref_square = avpENU2NED(trj.avp);
truthpath="dataset-simu\square\input\truth.nav";
truthfp=fopen(truthpath,'wt');
fprintf(truthfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', pva_ref_square');
fclose(truthfp);

pva_ref_zz = avpENU2NED(trj1.avp);
truthpath="dataset-simu\zz\input\truth.nav";
truthfp=fopen(truthpath,'wt');
fprintf(truthfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', pva_ref_zz');
fclose(truthfp);

pva_ref_line = avpENU2NED(trj2.avp);
truthpath="dataset-simu\line\input\truth.nav";
truthfp=fopen(truthpath,'wt');
fprintf(truthfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', pva_ref_line');
fclose(truthfp);
