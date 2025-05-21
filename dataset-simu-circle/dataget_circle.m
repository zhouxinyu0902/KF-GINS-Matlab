clear all
glvs
pos0 = [d2r([15;115]);-1200];
R=100;
V=1;
N=6;
avp_circle = circle_seg(pos0,R,V,N);

insplot(avp_circle)
imu=avp2imu(avp_circle);
%% 仿真imu数据
eb=0.027;
db=15/0.98;
web=0.003;
wdb=0.03*1e6/(60*9.8);

cfg.gyrarw = 0.003; % [deg/sqrt(h)]
cfg.accvrw = 0.03; % [m/s/sqrt(h)]
cfg.gyrbiasstd = 0.027; % [deg/h]
cfg.accbiasstd = 15; % [mGal]

% eb=2;db=0.36*980;web=0.15;wdb=200/9.8;
% eb=0.1;db=0.1;web=0.05;wdb=10;
rng(1);
imuerr = imuerrset(eb, db, web, wdb); %% adi-16465
trjimu= imuadderr(imu, imuerr);
% 将仿真的数据右前上转为真实的测量前右下，并保存
imu(:,1)=trjimu(:,7);
imu(:,2)=trjimu(:,2);
imu(:,3)=trjimu(:,1);
imu(:,4)=-trjimu(:,3);
imu(:,5)=trjimu(:,5);
imu(:,6)=trjimu(:,4);
imu(:,7)=-trjimu(:,6);

imupath="dataset-simu-circle\imu.txt";
imufp=fopen(imupath,'wt');
for i=1:length(imu)
    fprintf(imufp, '%.9f %.10f %.10f %.10f %.10f %.10f %.10f \n', imu(i,:));
end
fclose(imufp);
%% 将仿真的参考值ENU转到NED下

pva_ref(:,3:4)=r2d(avp_circle(:,7:8));
pva_ref(:,5)=avp_circle(:,9);

pva_ref(:,6)=avp_circle(:,5);
pva_ref(:,7)=avp_circle(:,4);
pva_ref(:,8)=-avp_circle(:,6);

pva_ref(:,9)=r2d(avp_circle(:,2));
pva_ref(:,10)=r2d(avp_circle(:,1));
pva_ref(:,11)=r2d(yawcvt(avp_circle(:,3),"cc180c360"));
pva_ref(:,2)=avp_circle(:,end);
pva_ref(:,1)=zeros(size(pva_ref(:,11)));

truthpath="dataset-simu-circle\truth.nav";
truthfp=fopen(truthpath,'wt');
for i=1:length(pva_ref)
    fprintf(truthfp, '%2d %12.6f %12.8f %12.8f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f \n', pva_ref(i,:));
end
fclose(truthfp);
%%
% inspure(imu,avp_circle(1,1:9));
%%
bcn=dxyz2pos([-R,0,0],avp_circle(1,7:9)');
bcn(1:2)=r2d(bcn(1:2));
range1=bcn2range(pva_ref,bcn);
range1(:,2:3)=range1(:,2:3)+normrnd(0,1,size(range1(:,2:3)));
rangepath="dataset-simu-circle\range-100m.txt";
rangefp=fopen(rangepath,'wt');
for i=1:length(range1)
    fprintf(rangefp,'%12.6f %.8f %.8f %12.8f %12.8f %8.4f\n', range1(i,:));
end
fclose(rangefp);

%% depth数据
depth=[avp_circle(:,end),avp_circle(:,end-1)];
depth(:,2)=depth(:,2)+normrnd(0,0.2,size(depth(:,2)));
depthpath="dataset-simu-circle\depth.txt";
depthfp=fopen(depthpath,'wt');
for i=1:length(range1)
    fprintf(depthfp,'%12.6f %.8f\n', depth(i,:));
end
fclose(depthfp);
