clear all
glvs
avp0 = [[0;0;d2r(90)]; [0;0;0]; [d2r([15;115]);-1200]];
ts=0.01;
%% STRAIGHT LINE轨迹
% 2m/s=7.2km/h  20km需要
xxx = [];
seg = trjsegment(xxx, 'init',         0);
seg = trjsegment(seg, 'uniform',      50); % 保持原来的状态不变
seg = trjsegment(seg, 'accelerate',   5, xxx, 0.4); % 加速
seg = trjsegment(seg, 'uniform',      3600*2);
seg = trjsegment(seg, 'deaccelerate',   5, xxx, 0.4); % 加速
trj2= trjsimu(avp0, seg.wat, ts, 1); 
myfigurestartup(7,7,'prese')
insplot(trj2.avp)
fprintf('直线轨迹总时长%d（s）\n',sum(trj2.wat(:,1)))
%%
% eb=0.003;
% db=7;
% web=0.0003;
% wdb=1e-6*1e5/3600;%1e-6m/s/sqrt(Hz),1e-6*1e5/3600 = 2.7778e-05
eb=0.01;
db=10;
web=0.0003;
wdb=1e-5*1e5/3600;%1e-6 m/s/sqrt(Hz)
%1e-6*1e5/3600 = 2.7778e-05
rng(1);
% imuerr = imuerrset(eb, db, web, wdb, web, 4, wdb ,4, 5 , 10, 5, 10, 10, 10, 10); 
imuerr = imuerrset(eb, db, web, wdb, web); 
trjimu_line= imuadderr(trj2.imu, imuerr);

imu_line = imuRFU2FRD(trjimu_line);
imupath="dataset-simu\line\input\imu.nav";
imufp=fopen(imupath,'wt');
fprintf(imufp, '%.9f %.10f %.10f %.10f %.10f %.10f %.10f \n', imu_line');
fclose(imufp);

pva_ref_line = avpENU2NED(trj2.avp);
truthpath="dataset-simu\line\input\truth.nav";
truthfp=fopen(truthpath,'wt');
fprintf(truthfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', pva_ref_line');
fclose(truthfp);