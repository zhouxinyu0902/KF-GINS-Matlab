% glvs
% pos=[d2r(30);d2r(120);20];
% vel=[0;0;0];
% ath=ethinit(pos,vel);
% 
% 
% att=d2r([30;45;60]);
% [c312,c321]=a2mat(att);
% a2qua(att);
clear
glvs
load('matlab.mat')
load('Leador-A15\data_Leador-A15.mat')
D=importdata("imu_Leador-A15.txt");
c=importdata("D://GitHub//KF-GINS-Matlab//dataset1//avpfile.txt");
avp_Ref=avpref(1:120000,[1:9,11]);
%%
att00=d2r([-2.03480295, 0.85421502, 185.70235133]');
att00(3)=yawcvt(att00(3));
att0=att00;
vel0=[0.0, 0.0, 0.0]';
pos0=[30.4447873701, 114.4718632047, 20.899]';
pos0(1:2)=d2r(pos0(1:2));
avp0=[att0;vel0;pos0];
ins = myins('initial',0.005,avp0);
tic
for i=1:120000
    ins = myins('update',ins,imu(10000+2,1:6),1);
    avp(i,:)=[ins.avp',imu(10000+i,end)];
end
toc
%%
myfigurestartup(10,10,'prese')
insplot(avp1(:,7:9))
insplot(avp(:,7:9))
insplot(avp_Ref(:,7:9))
insplot(avp_kfgins(:,7:9))
insplot(c(:,7:9))
legend('start','C','start','M','start',"ref")
%%
avp1=importdata("D:/GitHub/ZXY_NNEW/dataset/output_avp_data.txt");
myfigurestartup(7,7,'prese')
trjsee(avp_Ref,'2d',avp1,avp,avp_kfgins)
legend('ref','c-psins','m-psins','m-kfgins','start')
%%
avpcmpplot(avp_Ref,avp1);
avpcmpplot(avp_Ref,avp);
avpcmpplot(avp_Ref,avp_kfgins);
%%
plotTrajectoriesComparison(avp_Ref, avp1, 0.005);
plotTrajectoriesComparison(avp_Ref, avp, 0.005);
plotTrajectoriesComparison(avp_Ref, avp_kfgins, 0.005);
%%
vel0=[0.0, 0.0, 0.0]';
vel1=[1.0, 1.0, 1.0]';
pos0=[d2r(30.4447873701), d2r(114.4718632047), 20.899]';
pos1=[d2r(30.4447873702), d2r(114.4718632048), 20.8]';
eth1=ethinit(pos0,vel0);
eth1=ethupdate(eth1,pos1,vel1);